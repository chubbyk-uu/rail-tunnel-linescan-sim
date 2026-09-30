"""Place authored AI crack paths in metres and build a small optical spatial index."""
import argparse
import json
import math
from pathlib import Path

import numpy as np
import yaml

from .stage_b_cracks import calibrate_long, sample_widths
from .stage_b_scene import digest, load_spec, peak_rss_bytes

SEGMENT=np.dtype([('x0','<f4'),('q0','<f4'),('x1','<f4'),('q1','<f4'),('r0','<f4'),('r1','<f4')])
# Optional parallel array (depths.bin): effective visible depth at both segment ends, metres.
DEPTH=np.dtype([('d0','<f4'),('d1','<f4')])


def exact_composition(count,weights,rng):
    if count<=0 or set(weights)!={'long_slender','short_slender','network'}:
        raise ValueError('invalid instance composition')
    names=list(weights)
    probabilities=np.array([weights[n] for n in names],float)
    if not np.isfinite(probabilities).all() or (probabilities<0).any() or abs(probabilities.sum()-1)>1e-9:
        raise ValueError('invalid instance weights')
    quotas=count*probabilities
    counts=np.floor(quotas).astype(int)
    for index in np.argsort(-(quotas-counts),kind='stable')[:count-counts.sum()]: counts[index]+=1
    kinds=np.repeat(names,counts)
    rng.shuffle(kinds)
    return kinds


def valid_motifs(catalog):
    valid,excluded=[],[]
    for motif in catalog['motifs']:
        try:
            calibrate_long(motif,1.,.4,origin=(0,0))
        except ValueError as error:
            excluded.append(dict(id=motif['id'],reason=str(error)))
        else: valid.append(motif)
    return valid,excluded


def assemble(config,spec,long_catalog,short_catalog,count=60):
    rng=np.random.default_rng(np.random.SeedSequence([spec['seed'],22]))
    kinds=exact_composition(count,spec['cracks']['composition_weights'],rng)
    # Ensure the mandatory ten-metre instance is part of the slender quota.
    first=np.flatnonzero(kinds=='long_slender')
    if not len(first): raise ValueError('ten-metre instance requires a long slender quota')
    kinds[0],kinds[first[0]]=kinds[first[0]],kinds[0]
    widths=sample_widths(count,spec['cracks'],spec['seed'])
    branch_rng=np.random.default_rng(np.random.SeedSequence([spec['seed'],23]))
    xlo,xhi=config['acceptance']['valid_x_m']
    radius=config['tunnel']['radius_m']
    qlo=radius*math.radians(config['gate']['start_deg'])
    qhi=radius*math.radians(config['gate']['end_deg'])
    motifs,_=valid_motifs(short_catalog)  # Reject defects; never fabricate a connection.
    result=[]
    for index,kind in enumerate(kinds):
        branched=kind!='network' and index!=0 and branch_rng.random()<spec['cracks']['branch_probability_in_slender']
        if kind=='network':
            candidates=[m for m in motifs if m['kind']=='network']
            length=float(rng.uniform(.2,.7))
        elif branched:
            candidates=[m for m in motifs if m['kind']=='branching']
            length=float(rng.uniform(1.2,4.5) if kind=='long_slender' else rng.uniform(.08,.6))
        elif kind=='long_slender':
            candidates=long_catalog['motifs']
            length=spec['cracks']['required_long_crack_length_m'] if index==0 else float(rng.uniform(1.2,5.))
        else:
            candidates=[m for m in motifs if m['kind'] in ('longitudinal','transverse','diagonal','curved')]
            length=float(rng.uniform(.08,.6))
        if not candidates: raise ValueError('missing authored crack type')
        motif=candidates[0] if index==0 else candidates[int(rng.integers(len(candidates)))]
        metric=calibrate_long(motif,length,float(widths[index]),origin=(0,0))
        paths=[np.asarray(p) for p in metric['paths_xq_m']]
        spine=np.asarray(metric['main_path_xq_m'])
        if kind!='network' and not branched: paths=[spine]
        # Slender cracks mostly longitudinal; short cracks also cross/diagonal.
        angle=0. if index==0 else float(rng.uniform(-.22,.22) if kind=='long_slender' else rng.uniform(-math.pi,math.pi))
        rotation=np.array([[math.cos(angle),-math.sin(angle)],[math.sin(angle),math.cos(angle)]])
        paths=[p@rotation.T for p in paths];spine=spine@rotation.T
        points=np.concatenate(paths);lo=points.min(axis=0);hi=points.max(axis=0)
        low=np.array([xlo+.05,qlo+.05])-lo;high=np.array([xhi-.05,qhi-.05])-hi
        if np.any(high<low): raise ValueError('crack does not fit the capture domain')
        offset=np.array([3.,.5]) if index==0 else rng.uniform(low,high)
        if np.any(offset<low) or np.any(offset>high): raise ValueError('required crack does not fit domain')
        paths=[p+offset for p in paths];spine+=offset
        # Source topology remains connected: smooth local widths, no paths invented.
        endpoint_degree={}
        for p in paths:
            for v in (p[0],p[-1]):
                key=tuple(np.round(v,9));endpoint_degree[key]=endpoint_degree.get(key,0)+1
        radii=[]
        for path_index,p in enumerate(paths):
            arc=np.r_[0,np.cumsum(np.linalg.norm(np.diff(p,axis=0),axis=1))]
            modulation=.015*np.sin(arc/.04+index*.53)+.01*np.sin(arc/.011+path_index)
            width=np.clip(widths[index]+modulation,spec['cracks']['width_min_mm'],spec['cracks']['width_max_mm'])
            taper=np.ones(len(p))
            if endpoint_degree[tuple(np.round(p[0],9))]==1: taper=np.minimum(taper,arc/.005)
            if endpoint_degree[tuple(np.round(p[-1],9))]==1: taper=np.minimum(taper,(arc[-1]-arc)/.005)
            radii.append((width*.0005*np.clip(taper,0,1)).tolist())
        result.append(dict(id=f'crack_{index:03d}',composition=str(kind),has_minor_branches=bool(branched),
                           source_motif=motif['id'],paths_xq_m=[p.tolist() for p in paths],vertex_radius_m=radii,
                           main_path_xq_m=spine.tolist(),main_length_m=float(np.linalg.norm(np.diff(spine,axis=0),axis=1).sum()),
                           longitudinal_span_m=float(np.ptp(spine[:,0])),body_width_mm=float(widths[index]),
                           tip_taper_m=.005,source_shape_scale_m=metric['source_pixel_scale_m']))
    return result


def build_grid(instances,bounds,cell_m=.01,max_index_bytes=32<<20,with_depths=False):
    """Segments, cell offsets, indices and grid metadata; with_depths also returns the per-segment
    depths (DEPTH) interpolated like the radii, from each instance's vertex_depth_m."""
    x0,x1,q0,q1=bounds
    if not 0<cell_m<=.2 or x1<=x0 or q1<=q0: raise ValueError('invalid crack grid')
    nx,nq=math.ceil((x1-x0)/cell_m),math.ceil((q1-q0)/cell_m)
    if (nx*nq+1)*4>max_index_bytes: raise ValueError('crack index exceeds budget')
    grid={};segments=[];depths=[]
    for item in instances:
        vertex_depths=item['vertex_depth_m'] if with_depths else [[0.]*len(p) for p in item['paths_xq_m']]
        for path,radii,dd in zip(item['paths_xq_m'],item['vertex_radius_m'],vertex_depths):
            for a,b,r0,r1,d0,d1 in zip(path[:-1],path[1:],radii[:-1],radii[1:],dd[:-1],dd[1:]):
                if np.linalg.norm(np.asarray(b)-a)<1e-9: continue
                # Subdivide long source edges to bound grid fanout; preserve straight source edges.
                pieces=max(1,math.ceil(np.linalg.norm(np.asarray(b)-a)/cell_m))
                for j in range(pieces):
                    t0,t1=j/pieces,(j+1)/pieces
                    u=(1-t0)*np.asarray(a)+t0*np.asarray(b);v=(1-t1)*np.asarray(a)+t1*np.asarray(b)
                    ra,rb=(1-t0)*r0+t0*r1,(1-t1)*r0+t1*r1
                    index=len(segments);segments.append((*u,*v,ra,rb));depths.append(((1-t0)*d0+t0*d1,(1-t1)*d0+t1*d1))
                    # Conservative float-coordinate error guard at up to 150 m.
                    margin=max(ra,rb)+2e-5
                    lo=np.maximum(0,np.floor((np.minimum(u,v)-margin-[x0,q0])/cell_m).astype(int))
                    hi=np.minimum([nx-1,nq-1],np.floor((np.maximum(u,v)+margin-[x0,q0])/cell_m).astype(int))
                    for iq in range(lo[1],hi[1]+1):
                        for ix in range(lo[0],hi[0]+1): grid.setdefault(iq*nx+ix,[]).append(index)
    offsets=np.zeros(nx*nq+1,dtype='<u4');indices=[]
    for i in range(nx*nq):
        indices.extend(grid.get(i,[]));offsets[i+1]=len(indices)
    packed=np.array(segments,dtype=SEGMENT);indices=np.array(indices,dtype='<u4')
    extra=np.array(depths,dtype=DEPTH) if with_depths else None
    total=packed.nbytes+offsets.nbytes+indices.nbytes+(extra.nbytes if with_depths else 0)
    if total>max_index_bytes: raise ValueError('crack index exceeds budget')
    meta=dict(origin_xq_m=[x0,q0],cell_m=cell_m,cells_xq=[nx,nq],segments=len(packed),index_entries=len(indices),
              bytes=total,max_cell_segments=max((len(v) for v in grid.values()),default=0))
    return (packed,offsets,indices,meta,extra) if with_depths else (packed,offsets,indices,meta)


def write_index(output,packed,offsets,indices,depths=None):
    """Write the crack index payloads; returns the defects.json 'files' entry."""
    data=[('segments.bin',packed),('offsets.bin',offsets),('indices.bin',indices)]+([('depths.bin',depths)] if depths is not None else [])
    for name,array in data: array.tofile(output/name)
    return {name:dict(file=name,sha256=digest(output/name)) for name,_ in data}


def prepare(config_path,spec_path,long_path,short_path,output,count=60):
    config=yaml.safe_load(Path(config_path).read_text());spec=load_spec(spec_path)
    output=Path(output).resolve()
    if output.exists(): raise ValueError('defect output already exists')
    long=json.loads(Path(long_path).read_text());short=json.loads(Path(short_path).read_text())
    for data in (long,short):
        if digest(data['source_file'])!=data['source_sha256']: raise ValueError('AI source hash mismatch')
    items=assemble(config,spec,long,short,count)
    r=config['tunnel']['radius_m']
    bounds=[config['tunnel']['x_min_m'],config['tunnel']['x_max_m'],-math.pi*r,math.pi*r]
    packed,offsets,indices,grid=build_grid(items,bounds)  # default 10 mm cells
    output.mkdir(parents=True)
    for name,data in (('segments.bin',packed),('offsets.bin',offsets),('indices.bin',indices)): data.tofile(output/name)
    result=dict(schema='ssb.defect_layout.v1',seed=spec['seed'],instances=items,grid=grid,
                composition_counts={kind:sum(i['composition']==kind for i in items) for kind in ('long_slender','short_slender','network')},
                branching_instances=sum(i['has_minor_branches'] for i in items),
                excluded_source_motifs=valid_motifs(short)[1],
                files={name:dict(file=name,sha256=digest(output/name)) for name in ('segments.bin','offsets.bin','indices.bin')},
                inputs={name:dict(file=str(Path(path).resolve()),sha256=digest(path)) for name,path in
                        dict(config=config_path,spec=spec_path,long_catalog=long_path,short_catalog=short_path).items()},
                assumption='60 instances/20 m, long 1.2-5 m + mandatory 10 m, short .08-.6 m; density/lengths are adjustable development choices.',
                optical_model='Albedo coverage of tapered metric vectors; crack depth/relief is not yet calibrated.',
                preparation_peak_rss_bytes=peak_rss_bytes())
    (output/'defects.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(dict(counts=result['composition_counts'],branching=result['branching_instances'],grid=grid)))
    return result


def self_affine(rng, s, lam_min, lam_max, amplitude, lam_ref, hurst, per_octave=3):
    """Smooth random profile over arc length s: log-spaced sinusoids, a(lam)=amplitude*(lam/lam_ref)^H."""
    k = max(1, int(round(math.log2(lam_max/lam_min)*per_octave)))
    lams = np.geomspace(lam_min, lam_max, k)
    amps = amplitude*(lams/lam_ref)**hurst/math.sqrt(per_octave)
    phases = rng.uniform(0, 2*math.pi, k)
    return (amps[:,None]*np.sin(2*math.pi*s[None,:]/lams[:,None]+phases[:,None])).sum(axis=0)


def refine_path(path, radius, shared, rng, spec, cracks):
    """Resample, smooth the source staircase, add self-affine wiggle and width variation.
    shared: (start, end) endpoint is shared with another path (keep fixed, wiggle fades)."""
    from scipy.ndimage import gaussian_filter1d
    f = cracks['refine']; path = np.asarray(path,float); radius = np.asarray(radius,float)
    arc = np.r_[0,np.cumsum(np.linalg.norm(np.diff(path,axis=0),axis=1))]; L = arc[-1]
    n = max(2, int(math.ceil(L/f['resample_m']))+1); s = np.linspace(0, L, n)
    pts = np.column_stack([np.interp(s,arc,path[:,0]),np.interp(s,arc,path[:,1])])
    ds = s[1]-s[0] if n > 1 else 1.
    smooth = gaussian_filter1d(pts, f['smooth_m']/ds, axis=0, mode='nearest')
    # Endpoints stay exactly where the source (and any connected path) has them.
    fade = lambda d: np.clip(d/(3*f['smooth_m']),0,1)**2
    for end, w in ((0, 1-fade(s)), (-1, 1-fade(L-s))):
        smooth += w[:,None]*(pts[end]-smooth[end])
    tangent = np.gradient(smooth, axis=0); tangent /= np.maximum(np.linalg.norm(tangent,axis=1,keepdims=True),1e-12)
    normal = np.column_stack([-tangent[:,1], tangent[:,0]])
    w = f['wiggle']
    offset = self_affine(rng, s, w['min_wavelength_m'], w['max_wavelength_m'], w['amplitude_m'], w['reference_wavelength_m'], w['hurst'])
    taper = np.ones(n)
    if shared[0]: taper = np.minimum(taper, np.clip(s/f['junction_taper_m'],0,1))
    if shared[1]: taper = np.minimum(taper, np.clip((L-s)/f['junction_taper_m'],0,1))
    # Free ends also keep their source position (tips stay where the width tapers).
    taper = np.minimum(taper, np.minimum(np.clip(s/f['junction_taper_m'],0,1), np.clip((L-s)/f['junction_taper_m'],0,1)))
    refined = smooth+(offset*taper)[:,None]*normal
    base = np.interp(s, arc, radius)
    slow, fine = f['width_log_sigma']
    unit = lambda z: z/max(float(z.std()),1e-12)
    body = np.exp(slow*unit(self_affine(rng,s,.005,.02,1.,.01,0.))+fine*unit(self_affine(rng,s,.001,.002,1.,.0015,0.)))
    lo, hi = cracks['width_min_mm']*.0005, cracks['width_max_mm']*.0005
    # Body radius varies within the width range; the source tip taper (0 at free tips) scales it.
    peak = max(float(base.max()), 1e-12)
    r = np.clip(peak*body, lo, hi)*(base/peak)
    return refined, r


def refine(source, output, spec_path):
    """New defect version: same instances and topology, refined paths/widths (synthetic detail)."""
    source=Path(source).resolve();output=Path(output).resolve()
    if output.exists(): raise ValueError('defect output already exists')
    old=json.loads((source/'defects.json').read_text());spec=load_spec(spec_path);cracks=spec['cracks']
    config=yaml.safe_load(Path(old['inputs']['config']['file']).read_text());rr=config['tunnel']['radius_m']
    bounds=[config['tunnel']['x_min_m'],config['tunnel']['x_max_m'],-math.pi*rr,math.pi*rr]
    items=[]
    for index,item in enumerate(old['instances']):
        rng=np.random.default_rng(np.random.SeedSequence([spec['seed'],41,index]))
        item={k:v for k,v in item.items() if k!='vertex_depth_m'}   # depths follow the old vertices
        degree={}
        for p in item['paths_xq_m']:
            for v in (p[0],p[-1]):key=tuple(np.round(v,9));degree[key]=degree.get(key,0)+1
        paths,radii=[],[]
        for p,r in zip(item['paths_xq_m'],item['vertex_radius_m']):
            shared=(degree[tuple(np.round(p[0],9))]>1,degree[tuple(np.round(p[-1],9))]>1)
            q,w=refine_path(p,r,shared,rng,spec,cracks);paths.append(q.tolist());radii.append(w.tolist())
        spine=np.asarray(item['main_path_xq_m'])
        same=[k for k,p in enumerate(item['paths_xq_m']) if len(p)==len(spine) and np.allclose(p,spine)]
        main=np.asarray(paths[same[0]]) if same else refine_path(spine,np.full(len(spine),1e-4),(False,False),rng,spec,cracks)[0]
        items.append(dict(item,paths_xq_m=paths,vertex_radius_m=radii,main_path_xq_m=main.tolist(),
                          main_length_m=float(np.linalg.norm(np.diff(main,axis=0),axis=1).sum()),
                          longitudinal_span_m=float(np.ptp(main[:,0])),refined=True))
    packed,offsets,indices,grid=build_grid(items,bounds)
    output.mkdir(parents=True)
    result=dict(old,instances=items,grid=grid,files=write_index(output,packed,offsets,indices),
                refined_from=dict(file=str(source/'defects.json'),sha256=digest(source/'defects.json'),spec_sha256=digest(spec_path),parameters=cracks['refine']),
                assumption=old['assumption']+' Refined: source staircase below smooth_m replaced by synthetic self-affine wiggle; widths vary along the crack (synthetic).',
                preparation_peak_rss_bytes=peak_rss_bytes())
    (output/'defects.json').write_text(json.dumps(result)+'\n')
    print(json.dumps(dict(grid=grid)))
    return result


def depth_profile(s,radius,rng,depth):
    """Effective visible depth along one path (synthetic appearance parameter, not a measured or
    total crack depth): D = aspect(s) x local width, aspect log-normal along the crack with the
    given wavelength band, plus short shallow stretches (debris/dust plugs)."""
    lo,hi=depth['wavelengths_m']
    z=self_affine(rng,s,lo,hi,1.,hi,0.);z=z/max(float(z.std()),1e-12) if len(s)>2 else np.zeros_like(s)
    log_aspect=math.log(depth['aspect_median'])+depth['aspect_log_sigma']*z
    L=float(s[-1])
    for _ in range(rng.poisson(depth['plug_rate_per_m']*L)):
        c=rng.uniform(0,L);half=.5*rng.uniform(*depth['plug_length_m'])
        w=.5*(1+np.cos(np.pi*np.clip(np.abs(s-c)/half,0,1)))           # smooth bump, 1 at the centre
        log_aspect=(1-w)*log_aspect+w*math.log(depth['plug_aspect'])
    return np.minimum(np.exp(log_aspect)*2*np.asarray(radius),depth['max_depth_m'])


def add_depth(source,output,spec_path):
    """New defect version: same instances, plus per-vertex effective visible depth (depths.bin)."""
    source=Path(source).resolve();output=Path(output).resolve()
    if output.exists(): raise ValueError('defect output already exists')
    old=json.loads((source/'defects.json').read_text());spec=load_spec(spec_path);depth=spec['cracks']['depth']
    config=yaml.safe_load(Path(old['inputs']['config']['file']).read_text());rr=config['tunnel']['radius_m']
    bounds=[config['tunnel']['x_min_m'],config['tunnel']['x_max_m'],-math.pi*rr,math.pi*rr]
    items=[]
    for index,item in enumerate(old['instances']):
        rng=np.random.default_rng(np.random.SeedSequence([spec['seed'],43,index]));vertex_depth=[]
        for p,r in zip(item['paths_xq_m'],item['vertex_radius_m']):
            p=np.asarray(p);s=np.r_[0,np.cumsum(np.linalg.norm(np.diff(p,axis=0),axis=1))]
            vertex_depth.append(depth_profile(s,r,rng,depth).tolist())
        items.append(dict(item,vertex_depth_m=vertex_depth))
    packed,offsets,indices,grid,depths=build_grid(items,bounds,old['grid']['cell_m'],with_depths=True)
    output.mkdir(parents=True)
    result=dict(old,instances=items,grid=grid,files=write_index(output,packed,offsets,indices,depths),
                depth_from=dict(file=str(source/'defects.json'),sha256=digest(source/'defects.json'),spec_sha256=digest(spec_path),parameters=depth),
                optical_model='Cavity reflectance of a V-profiled slot from the width and a synthetic effective visible depth (vertex_depth_m); not a measured or total depth, no geometric relief.',
                preparation_peak_rss_bytes=peak_rss_bytes())
    (output/'defects.json').write_text(json.dumps(result)+'\n')
    print(json.dumps(dict(grid=grid)))
    return result


def regrid(source,output,cell_m):
    """Same crack instances, new spatial index cell size (renderer lookup cost only)."""
    source=Path(source).resolve();output=Path(output).resolve()
    if output.exists(): raise ValueError('defect output already exists')
    old=json.loads((source/'defects.json').read_text())
    config=yaml.safe_load(Path(old['inputs']['config']['file']).read_text());r=config['tunnel']['radius_m']
    bounds=[config['tunnel']['x_min_m'],config['tunnel']['x_max_m'],-math.pi*r,math.pi*r]
    g=old['grid']
    if abs(g['origin_xq_m'][0]-bounds[0])>1e-12 or abs(g['origin_xq_m'][1]-bounds[2])>1e-12: raise ValueError('grid bounds changed')
    with_depths='vertex_depth_m' in old['instances'][0]
    packed,offsets,indices,grid,*depths=build_grid(old['instances'],bounds,cell_m,with_depths=with_depths)
    output.mkdir(parents=True)
    result=dict(old,grid=grid,files=write_index(output,packed,offsets,indices,*depths),
                regridded_from=dict(file=str(source/'defects.json'),sha256=digest(source/'defects.json'),cell_m=g['cell_m']),
                preparation_peak_rss_bytes=peak_rss_bytes())
    (output/'defects.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(dict(grid=grid)))
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--regrid',type=Path,help='existing defect layout: rebuild only its crack index')
    p.add_argument('--refine',type=Path,help='existing defect layout: refine paths/widths (needs --spec)')
    p.add_argument('--depth',type=Path,help='existing defect layout: add effective visible depth (needs --spec)')
    p.add_argument('--cell-m',type=float,default=.01)
    for arg in ('config','spec','long-catalog','short-catalog'): p.add_argument('--'+arg,type=Path)
    p.add_argument('--output',required=True,type=Path);p.add_argument('--count',type=int,default=60)
    a=p.parse_args()
    if a.refine: refine(a.refine,a.output,a.spec)
    elif a.depth: add_depth(a.depth,a.output,a.spec)
    elif a.regrid: regrid(a.regrid,a.output,a.cell_m)
    else: prepare(a.config,a.spec,a.long_catalog,a.short_catalog,a.output,a.count)


if __name__=='__main__': main()
