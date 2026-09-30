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


def build_grid(instances,bounds,cell_m=.01,max_index_bytes=32<<20):
    x0,x1,q0,q1=bounds
    if not 0<cell_m<=.2 or x1<=x0 or q1<=q0: raise ValueError('invalid crack grid')
    nx,nq=math.ceil((x1-x0)/cell_m),math.ceil((q1-q0)/cell_m)
    if (nx*nq+1)*4>max_index_bytes: raise ValueError('crack index exceeds budget')
    grid={};segments=[]
    for item in instances:
        for path,radii in zip(item['paths_xq_m'],item['vertex_radius_m']):
            for a,b,r0,r1 in zip(path[:-1],path[1:],radii[:-1],radii[1:]):
                if np.linalg.norm(np.asarray(b)-a)<1e-9: continue
                # Subdivide long source edges to bound grid fanout; preserve straight source edges.
                pieces=max(1,math.ceil(np.linalg.norm(np.asarray(b)-a)/cell_m))
                for j in range(pieces):
                    t0,t1=j/pieces,(j+1)/pieces
                    u=(1-t0)*np.asarray(a)+t0*np.asarray(b);v=(1-t1)*np.asarray(a)+t1*np.asarray(b)
                    ra,rb=(1-t0)*r0+t0*r1,(1-t1)*r0+t1*r1
                    index=len(segments);segments.append((*u,*v,ra,rb))
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
    if packed.nbytes+offsets.nbytes+indices.nbytes>max_index_bytes: raise ValueError('crack index exceeds budget')
    return packed,offsets,indices,dict(origin_xq_m=[x0,q0],cell_m=cell_m,cells_xq=[nx,nq],
                                     segments=len(packed),index_entries=len(indices),
                                     bytes=packed.nbytes+offsets.nbytes+indices.nbytes,
                                     max_cell_segments=max((len(v) for v in grid.values()),default=0))


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


def regrid(source,output,cell_m):
    """Same crack instances, new spatial index cell size (renderer lookup cost only)."""
    source=Path(source).resolve();output=Path(output).resolve()
    if output.exists(): raise ValueError('defect output already exists')
    old=json.loads((source/'defects.json').read_text())
    config=yaml.safe_load(Path(old['inputs']['config']['file']).read_text());r=config['tunnel']['radius_m']
    bounds=[config['tunnel']['x_min_m'],config['tunnel']['x_max_m'],-math.pi*r,math.pi*r]
    g=old['grid']
    if abs(g['origin_xq_m'][0]-bounds[0])>1e-12 or abs(g['origin_xq_m'][1]-bounds[2])>1e-12: raise ValueError('grid bounds changed')
    packed,offsets,indices,grid=build_grid(old['instances'],bounds,cell_m)
    output.mkdir(parents=True)
    for name,data in (('segments.bin',packed),('offsets.bin',offsets),('indices.bin',indices)): data.tofile(output/name)
    result=dict(old,grid=grid,files={name:dict(file=name,sha256=digest(output/name)) for name in ('segments.bin','offsets.bin','indices.bin')},
                regridded_from=dict(file=str(source/'defects.json'),sha256=digest(source/'defects.json'),cell_m=g['cell_m']),
                preparation_peak_rss_bytes=peak_rss_bytes())
    (output/'defects.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(dict(grid=grid)))
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--regrid',type=Path,help='existing defect layout: rebuild only its crack index')
    p.add_argument('--cell-m',type=float,default=.01)
    for arg in ('config','spec','long-catalog','short-catalog'): p.add_argument('--'+arg,type=Path)
    p.add_argument('--output',required=True,type=Path);p.add_argument('--count',type=int,default=60)
    a=p.parse_args()
    if a.regrid: regrid(a.regrid,a.output,a.cell_m)
    else: prepare(a.config,a.spec,a.long_catalog,a.short_catalog,a.output,a.count)


if __name__=='__main__': main()
