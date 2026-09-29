"""Catalog AI crack candidates and extract their authored topology at source scale.

The skeleton tracing follows 4WIDS tools/branch_crack_fixture.py (Apache-2.0).
Only existing image edges are traced; this module does not invent crack paths.
"""
import argparse
import heapq
import json
import math
from pathlib import Path

import cv2
import numpy as np
from PIL import Image
from scipy.ndimage import gaussian_filter, gaussian_filter1d

from .stage_b_scene import digest, load_spec, peak_rss_bytes


KINDS = ('longitudinal','transverse','diagonal','curved','branching','network')


def trace_skeleton(mask):
    smooth = gaussian_filter(mask.astype(np.float32),.65)>=.45
    sk = cv2.ximgproc.thinning(smooth.astype(np.uint8)*255)>0
    points = set(zip(*np.nonzero(sk)))

    def neighbors(p):
        y,x = p
        found = []
        for dy in (-1,0,1):
            for dx in (-1,0,1):
                q = (y+dy,x+dx)
                if (dx==0 and dy==0) or q not in points:
                    continue
                if dx and dy and ((y+dy,x) in points or (y,x+dx) in points):
                    continue
                found.append(q)
        return found

    # Remove only tiny terminal extraction burrs; preserve network loops and branches.
    for _ in range(2):
        removed = set()
        for start in sorted(points):
            if len(neighbors(start)) != 1:
                continue
            path, previous, current = [start],None,start
            while len(path)<=3:
                following = [q for q in neighbors(current) if q!=previous]
                if len(following)!=1:
                    break
                previous,current = current,following[0]
                if len(neighbors(current))!=2:
                    break
                path.append(current)
            if len(path)<=3 and len(neighbors(current))>=3:
                removed.update(path)
        points-=removed
    visited, paths = set(),[]
    nodes = [p for p in sorted(points) if len(neighbors(p))!=2]
    for start in nodes+sorted(points):
        for nxt in neighbors(start):
            edge = tuple(sorted((start,nxt)))
            if edge in visited:
                continue
            path,previous,current = [start],start,nxt
            visited.add(edge)
            while True:
                path.append(current)
                following = [q for q in neighbors(current) if q!=previous]
                if len(following)!=1:
                    break
                nxt = following[0]
                edge = tuple(sorted((current,nxt)))
                if edge in visited:
                    break
                visited.add(edge)
                previous,current = current,nxt
            if len(path)>=2:
                paths.append(np.asarray(path,dtype=np.float32)[:,::-1])
    return paths


def sample_widths(count, parameters, seed):
    low, high, mu, sigma = (parameters[k] for k in ('width_min_mm','width_max_mm','width_mean_mm','width_std_mm'))
    if count<0 or not all(math.isfinite(v) for v in (low,high,mu,sigma)) or not 0<low<mu<high or sigma<=0:
        raise ValueError('invalid truncated normal width distribution')
    # Extreme sigma would make rejection sampling arbitrarily slow.
    if sigma>10*(high-low):
        raise ValueError('width distribution too broad for bounded sampling')
    rng = np.random.default_rng(seed)
    values = []
    while len(values)<count:
        batch = rng.normal(mu,sigma,max(32,2*(count-len(values))))
        values.extend(batch[(batch>=low)&(batch<=high)].tolist())
    return np.asarray(values[:count])


def sample_types(count, parameters, seed):
    """Sample scene composition, independently of source orientation/topology labels."""
    weights=parameters['composition_weights']
    types=['long_slender','short_slender','network']
    if count<0 or set(types)!=set(weights):
        raise ValueError('invalid crack type mix')
    probabilities=np.asarray([weights[k] for k in types],dtype=float)
    if not np.isfinite(probabilities).all() or (probabilities<0).any() or abs(probabilities.sum()-1)>1e-12:
        raise ValueError('crack type probabilities must sum to one')
    return np.random.default_rng(seed).choice(types,size=count,p=probabilities)


def sample_branches(groups, parameters, seed):
    probability=parameters['branch_probability_in_slender']
    if not math.isfinite(probability) or not 0<=probability<=1:
        raise ValueError('invalid slender branching probability')
    groups=np.asarray(groups)
    if not np.isin(groups,['long_slender','short_slender','network']).all():
        raise ValueError('unknown crack composition class')
    # Independent stream avoids coupling branch choice to the composition draw.
    rng=np.random.default_rng(np.random.SeedSequence([seed,1]))
    return (rng.random(groups.shape)<probability)&(groups!='network')


def catalog(source, spec_path, output):
    spec = load_spec(spec_path)
    source = Path(source).resolve()
    with Image.open(source) as image:
        if image.mode!='RGBA':
            raise ValueError('crack atlas must have an alpha channel')
        if image.width*image.height*32 > spec['resources']['asset_working_set_bytes']:
            raise ValueError('crack atlas exceeds extraction memory budget')
        alpha = np.asarray(image)[:,:,3].copy()
    if alpha.min()!=0 or alpha.max()<96:
        raise ValueError('atlas is not transparent or contains no opaque fissures')
    count,labels,stats,centres = cv2.connectedComponentsWithStats((alpha>=96).astype(np.uint8),8)
    selected = sorted(range(1,count),key=lambda i:int(stats[i,4]),reverse=True)[:18]
    if len(selected)!=18 or min(stats[i,4] for i in selected)<300:
        raise ValueError('expected 18 substantial isolated crack candidates')
    # This atlas has variable-height rows; centroid order identifies six rows of three.
    selected.sort(key=lambda i:centres[i,1])
    items = []
    for row, kind in enumerate(KINDS):
        group = sorted(selected[row*3:row*3+3],key=lambda i:centres[i,0])
        for col,label in enumerate(group):
            x,y,w,h,area = map(int,stats[label])
            if min(x,y,alpha.shape[1]-(x+w),alpha.shape[0]-(y+h))<4:
                raise ValueError('crack candidate touches atlas border')
            paths = trace_skeleton(labels[y:y+h,x:x+w]==label)
            if not paths:
                raise ValueError('candidate has no traceable centreline')
            items.append(dict(id=f'{kind}_{col+1:02d}',kind=kind,crop_xywh_px=[x,y,w,h],area_px=area,
                              paths_xy_px=[p.round(4).tolist() for p in paths],
                              status='candidate',physical_length_m=None,physical_width_mm=None))
    out = Path(output)
    if out.exists():
        raise ValueError('catalog output already exists')
    out.parent.mkdir(parents=True,exist_ok=True)
    result = dict(schema='ssb.crack_candidates.v1',source_file=str(source),source_sha256=digest(source),
                  source_size_px=[alpha.shape[1],alpha.shape[0]],generator='built-in image_gen',
                  spec_sha256=digest(spec_path),motifs=items,ignored_small_components=count-1-len(selected),
                  width_distribution=spec['cracks'],procedural_paths=False,
                  preparation_peak_rss_bytes=peak_rss_bytes(),
                  review='Visual/metric final selection pending. Source pixel thickness is not a physical width.')
    out.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(dict(output=str(out.resolve()),candidates=len(items),types=list(KINDS),peak_rss_bytes=result['preparation_peak_rss_bytes'])))
    return result


def main_path(paths):
    """Trace the source graph between its leftmost/rightmost nodes; no invented joins."""
    graph = {}
    for i,path in enumerate(paths):
        if len(path)<2:
            continue
        a,b = tuple(path[0]),tuple(path[-1])
        length = float(np.linalg.norm(np.diff(path,axis=0),axis=1).sum())
        graph.setdefault(a,[]).append((b,length,i,False))
        graph.setdefault(b,[]).append((a,length,i,True))
    if not graph:
        raise ValueError('empty crack graph')
    start,end = min(graph),max(graph)
    if start==end:
        raise ValueError('long crack needs distinct endpoints')
    distances,previous,queue = {start:0.},{},[(0.,start)]
    while queue:
        cost,node = heapq.heappop(queue)
        if cost>distances[node]:
            continue
        if node==end:
            break
        for neighbor,length,index,reverse in graph[node]:
            new = cost+length
            if new<distances.get(neighbor,float('inf')):
                distances[neighbor]=new
                previous[neighbor]=(node,index,reverse)
                heapq.heappush(queue,(new,neighbor))
    if end not in previous:
        raise ValueError('long crack main path is disconnected')
    pieces,node = [],end
    while node!=start:
        before,index,reverse = previous[node]
        pieces.append(paths[index][::-1] if reverse else paths[index])
        node=before
    ordered = list(reversed(pieces))
    return np.concatenate([ordered[0]]+[p[1:] for p in ordered[1:]])


def calibrate_long(motif, length_m, width_mm, origin=(3.,.5)):
    if not math.isfinite(length_m) or length_m<=0 or not .2<=width_mm<=.6:
        raise ValueError('invalid physical crack dimensions')
    # Source-scale smoothing removes thinning stair steps; junction endpoints stay shared.
    paths = []
    for original in motif['paths_xy_px']:
        original = np.asarray(original,dtype=np.float64)
        smooth = gaussian_filter1d(original,.9,axis=0,mode='nearest') if len(original)>4 else original.copy()
        smooth[0],smooth[-1]=original[0],original[-1]
        paths.append(smooth)
    spine = main_path(paths)
    source_length = float(np.linalg.norm(np.diff(spine,axis=0),axis=1).sum())
    scale = length_m/source_length
    anchor = spine[0]
    def mapped(points):
        # Source image y increases down; q in the preview increases up.
        return (points-anchor)*np.array([scale,-scale])+np.asarray(origin)
    physical = [mapped(p) for p in paths]
    spine_m = mapped(spine)
    return dict(id=motif['id']+'_10m' if length_m==10 else motif['id']+'_metric',
                source_motif=motif['id'],paths_xq_m=[p.round(12).tolist() for p in physical],
                main_path_xq_m=spine_m.round(12).tolist(),body_width_mm=float(width_mm),
                main_length_m=float(np.linalg.norm(np.diff(spine_m,axis=0),axis=1).sum()),
                longitudinal_span_m=float(np.ptp(spine_m[:,0])),source_pixel_scale_m=scale,
                length_definition='main centreline arclength; branch lengths excluded',
                width_definition='independently assigned body diameter, not the resized source stroke',
                status='metric_candidate_not_installed_in_optix')


def coverage_patch(paths, width_mm, bounds, texel_m=.0002, samples=4, max_pixels=1048576):
    """Area-sample ONLY a local metric patch; never allocate a dense 10 m crack strip."""
    x0,x1,q0,q1=map(float,bounds)
    if not all(math.isfinite(v) for v in bounds) or x1<=x0 or q1<=q0 or texel_m<=0 or width_mm<=0 or not 1<=samples<=16:
        raise ValueError('invalid patch geometry')
    nx,ny=math.ceil((x1-x0)/texel_m),math.ceil((q1-q0)/texel_m)
    if nx*ny>max_pixels:
        raise ValueError('local crack patch exceeds pixel budget')
    radius=width_mm*.0005
    segments=[]
    for path in paths:
        path=np.asarray(path,dtype=np.float64)
        for a,b in zip(path[:-1],path[1:]):
            lo=np.floor((np.minimum(a,b)-radius-[x0,q0])/texel_m).astype(int)
            hi=np.ceil((np.maximum(a,b)+radius-[x0,q0])/texel_m).astype(int)
            left,bottom=np.maximum(lo,0)
            right,top=np.minimum(hi,[nx,ny])
            if right>left and top>bottom:
                segments.append((a,b,int(left),int(right),int(bottom),int(top)))
    counts=np.zeros((ny,nx),dtype=np.uint16)
    for sy in range(samples):
        for sx in range(samples):
            inside=np.zeros((ny,nx),dtype=bool)
            for a,b,left,right,bottom,top in segments:
                xs=x0+(np.arange(left,right)+(sx+.5)/samples)*texel_m
                qs=q0+(np.arange(bottom,top)+(sy+.5)/samples)*texel_m
                dx,dq=xs[None,:]-a[0],qs[:,None]-a[1]
                v=b-a
                norm=float(v@v)
                t=np.clip((dx*v[0]+dq*v[1])/norm,0,1) if norm>0 else np.zeros((top-bottom,right-left))
                inside[bottom:top,left:right]|=(dx-t*v[0])**2+(dq-t*v[1])**2<radius*radius
            counts+=inside
    return counts.astype(np.float32)/(samples*samples)


def catalog_long(source, spec_path, output):
    spec=load_spec(spec_path)
    source=Path(source).resolve()
    with Image.open(source) as image:
        if image.mode!='RGBA' or image.width*image.height*32>spec['resources']['asset_working_set_bytes']:
            raise ValueError('long crack source needs bounded RGBA data')
        alpha=np.asarray(image)[:,:,3].copy()
    count,labels,stats,centres=cv2.connectedComponentsWithStats((alpha>=96).astype(np.uint8),8)
    selected=sorted(range(1,count),key=lambda i:int(stats[i,4]),reverse=True)[:3]
    if len(selected)!=3 or min(stats[i,4] for i in selected)<300 or alpha.min()!=0:
        raise ValueError('expected three isolated transparent long crack candidates')
    selected.sort(key=lambda i:centres[i,1])
    motifs=[]
    for index,label in enumerate(selected):
        x,y,w,h,area=map(int,stats[label])
        if w<4*h or min(x,y,alpha.shape[1]-x-w,alpha.shape[0]-y-h)<4:
            raise ValueError('long crack must be slender with intact endpoints')
        paths=trace_skeleton(labels[y:y+h,x:x+w]==label)
        main_path(paths)  # Refuse a disconnected topology before assigning a physical length.
        motifs.append(dict(id=f'long_{index+1:02d}',kind='longitudinal',crop_xywh_px=[x,y,w,h],
                           area_px=area,paths_xy_px=[p.tolist() for p in paths],status='candidate'))
    length=spec['cracks']['required_long_crack_length_m']
    widths=sample_widths(len(motifs),spec['cracks'],spec['seed'])
    instances=[calibrate_long(m,length,float(w)) for m,w in zip(motifs,widths)]
    result=dict(schema='ssb.long_crack_candidates.v1',source_file=str(source),source_sha256=digest(source),
                spec_sha256=digest(spec_path),generator='built-in image_gen',motifs=motifs,
                metric_candidates=instances,selected_candidate='long_01',selection_basis='slender unbranched main course',
                source_shape_limit='Source pixel scale bounds shape detail; physical width is assigned independently.',
                preparation_peak_rss_bytes=peak_rss_bytes())
    out=Path(output)
    if out.exists():
        raise ValueError('long crack catalog already exists')
    out.parent.mkdir(parents=True,exist_ok=True)
    out.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(dict(candidates=len(motifs),main_lengths_m=[i['main_length_m'] for i in instances],
                          widths_mm=[i['body_width_mm'] for i in instances],peak_rss_bytes=result['preparation_peak_rss_bytes'])))
    return result


def metric_preview(catalog_path, output):
    """Scientific plot of width-calibrated coverage, not a photo or optical simulation."""
    import os
    # Keep font/cache preparation writable and reusable inside the project or /tmp.
    os.environ.setdefault('MPLCONFIGDIR',str(Path(output).resolve().parent/'matplotlib_cache'))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    data=json.loads(Path(catalog_path).read_text())
    item=data['metric_candidates'][0]
    paths=[np.asarray(p) for p in item['paths_xq_m']]
    spine=np.asarray(item['main_path_xq_m'])
    centre=spine[len(spine)//2]
    bounds=[centre[0]-.1,centre[0]+.1,centre[1]-.05,centre[1]+.05]
    fig,axes=plt.subplots(4,1,figsize=(12,8),layout='constrained')
    for path in paths:
        axes[0].plot(path[:,0],path[:,1],color='.15',linewidth=.6)
    axes[0].set_title('10 m main centreline; overview stroke enlarged for visibility')
    axes[0].set_xlabel('Tunnel x (m)')
    axes[0].set_ylabel('q (m)')
    areas={}
    for axis,width in zip(axes[1:],[.2,.4,.6]):
        alpha=coverage_patch(paths,width,bounds)
        # Relative linear albedo preview only; no lamp, blur, exposure or noise.
        image=.30*(1-alpha)+.05*alpha
        axis.imshow(image,origin='lower',extent=[0,200,-50,50],cmap='gray',vmin=0,vmax=.5,
                    interpolation='nearest',aspect='equal')
        axis.set_title(f'Body width {width:.1f} mm | 0.2 mm/pixel | local albedo preview')
        axis.set_xlabel('Local x (mm)')
        axis.set_ylabel('Local q (mm)')
        areas[str(width)]=float(alpha.sum()*.0002**2)
    output=Path(output)
    if output.exists():
        raise ValueError('metric preview already exists')
    output.parent.mkdir(parents=True,exist_ok=True)
    fig.savefig(output,dpi=150)
    plt.close(fig)
    report=dict(catalog_sha256=digest(catalog_path),preview_sha256=digest(output),
                local_patch_m=bounds,local_texel_m=.0002,local_coverage_area_m2=areas,
                preparation_peak_rss_bytes=peak_rss_bytes(),optical_simulation=False)
    output.with_suffix('.json').write_text(json.dumps(report,indent=2)+'\n')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',required=True,type=Path)
    parser.add_argument('--spec',required=True,type=Path)
    parser.add_argument('--output',required=True,type=Path)
    parser.add_argument('--mode',choices=['atlas','long'],default='atlas')
    parser.add_argument('--metric-preview',type=Path)
    args = parser.parse_args()
    if args.mode=='long':
        catalog_long(args.source,args.spec,args.output)
        if args.metric_preview:
            metric_preview(args.output,args.metric_preview)
    else:
        catalog(args.source,args.spec,args.output)


if __name__=='__main__':
    main()
