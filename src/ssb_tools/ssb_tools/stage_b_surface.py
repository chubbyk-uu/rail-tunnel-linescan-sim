# Minimum-cut quilting adapted from 4WIDS_agv/tools/concrete_quilt.py.
# Copyright 2026 chubbyk-uu; SPDX-License-Identifier: Apache-2.0
"""Reproducible multi-source PBR quilting and bounded, independently replayable tiles.

The development bake uses native 2K sources, not invented 0.2 mm background detail.
Cracks remain separate metric vectors and are evaluated during camera sampling.
"""
import argparse
import json
import math
from pathlib import Path
import time

import cv2
import numpy as np
from PIL import Image
import yaml

from .stage_b_materials import apply_transform, srgb_to_linear, sources_match_spec, image_info
from .stage_b_scene import digest, load_spec, peak_rss_bytes

TEXEL = np.dtype([('albedo','<u2'),('roughness','u1'),('reserved','u1'),('nx','<i2'),('nq','<i2')])


def minimum_cut(error):
    rows, columns = error.shape
    total = error.astype(np.float64).copy()
    back = np.zeros((rows,columns),np.int8)
    for row in range(1,rows):
        previous = total[row-1]
        options = np.vstack((np.r_[np.inf,previous[:-1]],previous,np.r_[previous[1:],np.inf]))
        choice = np.argmin(options,axis=0)
        back[row] = choice-1
        total[row] += options[choice,np.arange(columns)]
    seam = np.empty(rows,np.int32)
    seam[-1] = np.argmin(total[-1])
    for row in range(rows-2,-1,-1):
        seam[row] = seam[row+1]+back[row+1,seam[row+1]]
    return seam


def inverse_orientation(k, mirror, size, height=None):
    # Map transformed image coordinates back to source coordinates (metres).
    # size is the source extent along image x, height along image y (default square).
    w = size; h = size if height is None else height
    matrices = [np.eye(2),np.array([[0,-1],[1,0]]),-np.eye(2),np.array([[0,1],[-1,0]])]
    offsets = [np.zeros(2),np.array([w,0]),np.array([w,h]),np.array([0,h])]
    matrix, offset = matrices[k].astype(float), offsets[k].astype(float)
    if mirror:
        offset += matrix[:,0]*(w if k%2==0 else h)
        matrix[:,0] *= -1
    return matrix, offset


def prepare_sources(sources_path, recipe_path, spec, output, brightness):
    sources_path, recipe_path = Path(sources_path), Path(recipe_path)
    info, recipe = json.loads(sources_path.read_text()), json.loads(recipe_path.read_text())
    if not sources_match_spec(info,spec) or recipe['sources_sha256']!=digest(sources_path):
        raise ValueError('source/colour recipe identity mismatch')
    if not math.isfinite(brightness) or not 0<brightness<=1:
        raise ValueError('invalid reflectance multiplier')
    output.mkdir()
    results, guides = [], []
    gsd = spec['materials'].get('guide_texel_m',.02)
    for material in info['materials']:
        name = material['id']
        size = material['channels']['diffuse']['resolution']
        if len(size)!=2 or size[0]!=size[1]:
            raise ValueError('square co-registered sources required')
        for channel in material['channels'].values():
            path = sources_path.parent/name/channel['file']
            if channel['resolution']!=size or digest(path)!=channel['sha256']:
                raise ValueError('PBR source hash/dimensions mismatch')
            image_info(path,spec['resources']['asset_working_set_bytes'])
        target = output/(name+'.bin')
        packed = np.memmap(target,dtype=TEXEL,mode='w+',shape=(size[1],size[0]))
        with Image.open(sources_path.parent/name/material['channels']['diffuse']['file']) as im:
            rgb = np.asarray(im.convert('RGB'),dtype=np.float32)/255
        corrected = apply_transform(srgb_to_linear(rgb),recipe['transforms'][name],brightness)
        mono = corrected @ np.array([.2126,.7152,.0722],np.float32)
        packed['albedo'] = np.uint16(np.clip(mono*65535+.5,0,65535))
        side = round(material['source_width_m']/gsd)
        guides.append(cv2.resize(mono,(side,side),interpolation=cv2.INTER_AREA))
        del corrected, rgb, mono
        with Image.open(sources_path.parent/name/material['channels']['roughness']['file']) as im:
            packed['roughness'] = np.asarray(im.convert('L'))
        with Image.open(sources_path.parent/name/material['channels']['normal_gl']['file']) as im:
            normal = np.asarray(im.convert('RGB'),dtype=np.float32)/127.5-1
        normal /= np.maximum(np.linalg.norm(normal,axis=2,keepdims=True),1e-6)
        packed['nx'] = np.int16(np.clip(normal[:,:,0]*32767,-32767,32767))
        # GL y points towards decreasing source image v.
        packed['nq'] = np.int16(np.clip(-normal[:,:,1]*32767,-32767,32767))
        packed['reserved'] = 0
        packed.flush()
        del packed, normal
        results.append(dict(id=name,file=target.name,side=size[0],source_width_m=material['source_width_m'],
                            native_texel_m=material['source_width_m']/size[0],sha256=digest(target)))
    return results, guides


def quilt_layout(guides, source_widths, bounds, seed, directory, patch_m=.8, overlap_m=.2,
                 guide_texel_m=.02, min_repeat_x_m=5., near_crop_m=.12, candidates=48,
                 valid=None, weights=None, repeat_metric='x'):
    """Choose native crops, reject nearby source-region reuse, replay one mask on all channels.

    source_widths: per source, the extent (m) along image x, or (x, y) extents for
    non-square sources. valid: optional per-source boolean masks at guide resolution;
    a crop is used only if it lies entirely inside the valid region. weights: optional
    per-source selection probabilities. repeat_metric 'x' compares placements whose x
    distance is below min_repeat_x_m (original rule); 'wall' uses the 2D wall distance,
    for small sources that cannot satisfy a long axial exclusion strip.
    """
    x0,x1,q0,q1 = bounds
    patch, overlap = round(patch_m/guide_texel_m),round(overlap_m/guide_texel_m)
    extents = [tuple(w) if np.ndim(w) else (w,w) for w in source_widths]
    if valid is None: valid = [np.ones(g.shape,bool) for g in guides]
    if not 2<=overlap<patch or any(min(g.shape)<patch for g in guides) or any(v.shape!=g.shape for v,g in zip(valid,guides)):
        raise ValueError('invalid patch geometry')
    if weights is None: weights = [.45,.4,.15] if len(guides)==3 else None
    # Crop origins whose whole patch is valid, per source and orientation (summed-area table).
    def usable(mask):
        table = np.pad(mask.astype(np.int64),((1,0),(1,0))).cumsum(0).cumsum(1)
        full = table[patch:,patch:]-table[:-patch,patch:]-table[patch:,:-patch]+table[:-patch,:-patch]
        return np.argwhere(full==patch*patch)
    step = patch-overlap
    cols = math.ceil(((x1-x0)/guide_texel_m-overlap)/step)
    rows = math.ceil(((q1-q0)/guide_texel_m-overlap)/step)
    if min(rows,cols)<=0 or (rows*step+overlap)*(cols*step+overlap)>8_000_000:
        raise ValueError('guide canvas exceeds bounded preparation size')
    directory.mkdir()
    canvas = np.zeros((rows*step+overlap,cols*step+overlap),np.float32)
    orient = lambda a,k,flip: np.fliplr(np.rot90(a,k)) if flip else np.rot90(a,k)
    oriented = [[orient(g,k,flip) for k in range(4) for flip in (False,True)] for g in guides]
    origins = [[usable(orient(v,k,flip)) for k in range(4) for flip in (False,True)] for v in valid]
    if any(not len(o) for per in origins for o in per):
        raise ValueError('a source has no fully valid crop at this patch size')
    rng, placements, reused = np.random.default_rng(seed), [], 0
    # Vectorised repeat guard: previous placements' grid position, source and crop centre.
    history = np.zeros((0,5))
    coverage = np.zeros((2*canvas.shape[0],2*canvas.shape[1]),np.float32)
    # Advance in x first: only the previous 5 m strip participates in the reuse guard.
    for col in range(cols):
        for row in range(rows):
            top,left = row*step,col*step
            window = canvas[top:top+patch,left:left+patch]
            occupied = np.zeros((patch,patch),bool)
            if col: occupied[:,:overlap]=True
            if row: occupied[:overlap,:]=True
            pool=[]
            if repeat_metric=='wall':
                near=history[np.hypot(history[:,0]-left,history[:,1]-top)*guide_texel_m<min_repeat_x_m]
            else:
                near=history[np.abs(history[:,0]-left)*guide_texel_m<min_repeat_x_m]
            for attempt in range(candidates*16):
                material=int(rng.choice(len(guides),p=weights))
                orientation=int(rng.integers(8))
                guide=oriented[material][orientation]
                candidates_xy=origins[material][orientation]
                sy,sx=map(int,candidates_xy[int(rng.integers(len(candidates_xy)))])
                matrix,offset=inverse_orientation(orientation//2,bool(orientation%2),*extents[material])
                centre=matrix@((np.array([sx,sy])+patch/2)*guide_texel_m)+offset
                same=near[near[:,2]==material]
                if len(same) and np.min(np.hypot(same[:,3]-centre[0],same[:,4]-centre[1]))<near_crop_m:
                    reused+=1
                    continue
                block=guide[sy:sy+patch,sx:sx+patch]
                delta=(block-window)[occupied]
                cost=float(delta@delta)
                pool.append((cost,material,orientation,sx,sy,matrix,offset,centre,block))
                if len(pool)>=candidates: break
            if not pool:
                raise ValueError('source diversity cannot satisfy the repeat-distance constraint; add sources or adjust layout')
            best=min(p[0] for p in pool)
            options=[p for p in pool if p[0]<=best*1.1+1e-12]
            cost,material,orientation,sx,sy,matrix,offset,centre,block=options[int(rng.integers(len(options)))]
            mask=np.ones((patch,patch),np.uint8)
            # Keep the cut >= 3 guide px from the overlap edges: the sigma 0.65 px feather must
            # not leak below full opacity into the last overlap row/column, where the previous
            # patch's coverage ends.
            guard=max(3,overlap//5)
            if overlap<2*guard+2: raise ValueError('overlap too small for the seam guard')
            if col:
                error=(block[:,:overlap]-window[:,:overlap])**2
                error[:,:guard]=np.inf;error[:,-guard:]=np.inf
                seam=minimum_cut(error)
                mask[:,:overlap]&=np.arange(overlap)[None,:]>=seam[:,None]
            if row:
                error=((block[:overlap,:]-window[:overlap,:])**2).T
                error[:,:guard]=np.inf;error[:,-guard:]=np.inf
                seam=minimum_cut(error)
                mask[:overlap,:]&=np.arange(overlap)[:,None]>=seam[None,:]
            alpha=cv2.GaussianBlur(mask.astype(np.float32),(0,0),.65,borderType=cv2.BORDER_REPLICATE)
            alpha[~occupied]=1
            alpha=np.uint8(np.clip(alpha*255+.5,0,255))
            af=alpha.astype(np.float32)/255
            # Coverage between guide pixel centres as well (2x, bilinear like the generators).
            a2=cv2.resize(af,(2*patch,2*patch),interpolation=cv2.INTER_LINEAR)
            c2=coverage[2*top:2*top+2*patch,2*left:2*left+2*patch];c2[:]=c2*(1-a2)+a2
            window[:]=window*(1-af)+block*af
            file=f'alpha_{len(placements):04d}.png'
            if not cv2.imwrite(str(directory/file),alpha): raise OSError('cannot save quilt mask')
            history=np.vstack([history,[left,top,material,*centre]])
            placements.append(dict(top=top,left=left,material=material,orientation=orientation,
                                   source_centre_m=centre.tolist(),source_matrix=matrix.tolist(),
                                   source_offset_m=(offset+matrix@(np.array([sx,sy])*guide_texel_m)).tolist(),
                                   alpha=file,alpha_sha256=digest(directory/file)))
    if coverage[1:-1,1:-1].min()<.999: raise ValueError('quilt coverage incomplete between patches')
    layout=dict(schema='ssb.quilt_layout.v1',seed=seed,bounds_xq_m=bounds,origin_xq_m=[x0,q0],
                guide_texel_m=guide_texel_m,patch_pixels=patch,overlap_pixels=overlap,guide_size=list(canvas.shape),
                min_repeat_distance_x_m=min_repeat_x_m,near_source_crop_centre_m=near_crop_m,repeat_metric=repeat_metric,
                repeat_guard='same source region regardless of rotation/mirror; canonical crop centres within threshold',
                repeat_guard_limit='Not a global perceptual similarity or feature-level uniqueness guarantee.',
                rejected_near_reuse_candidates=reused,placements=placements)
    (directory/'layout.json').write_text(json.dumps(layout,indent=2)+'\n')
    cv2.imwrite(str(directory/'guide.png'),np.uint8(np.clip(canvas*255+.5,0,255)))
    return layout


class QuiltSampler:
    def __init__(self,layout,directory,sources):
        self.layout,self.directory,self.sources=layout,Path(directory),sources
        self.masks=[cv2.imread(str(self.directory/p['alpha']),cv2.IMREAD_GRAYSCALE) for p in layout['placements']]
        if any(mask is None for mask in self.masks): raise ValueError('missing quilt mask')

    def sample(self,xs,qs):
        layout=self.layout
        gsd=layout['guide_texel_m'];patch=layout['patch_pixels']
        gx=(np.asarray(xs)-layout['origin_xq_m'][0])/gsd
        gy=(np.asarray(qs)-layout['origin_xq_m'][1])/gsd
        result=np.zeros((len(qs),len(xs),4),np.float32)
        filled=np.zeros(result.shape[:2],np.float32)
        for p,mask in zip(layout['placements'],self.masks):
            xi=np.flatnonzero((gx>=p['left'])&(gx<p['left']+patch))
            yi=np.flatnonzero((gy>=p['top'])&(gy<p['top']+patch))
            if not len(xi) or not len(yi): continue
            section=np.s_[yi[0]:yi[-1]+1,xi[0]:xi[-1]+1]
            px,py=np.meshgrid(gx[xi]-p['left'],gy[yi]-p['top'])
            alpha=cv2.remap(mask,np.float32(px-.5),np.float32(py-.5),cv2.INTER_LINEAR,
                            borderMode=cv2.BORDER_REPLICATE).astype(np.float32)/255
            matrix=np.asarray(p['source_matrix']);offset=p['source_offset_m']
            packed,width=self.sources[p['material']]
            native=width/packed.shape[1]
            u=np.float32((matrix[0,0]*px*gsd+matrix[0,1]*py*gsd+offset[0])/native-.5)
            v=np.float32((matrix[1,0]*px*gsd+matrix[1,1]*py*gsd+offset[1])/native-.5)
            # Remap the same physical crop and the same seam alpha for every PBR channel.
            value=np.empty((*u.shape,4),np.float32)
            for i,(channel,scale) in enumerate((('albedo',65535),('roughness',255),('nx',32767),('nq',32767))):
                value[:,:,i]=cv2.remap(packed[channel],u,v,cv2.INTER_LINEAR,borderMode=cv2.BORDER_REPLICATE)/scale
            normal=value[:,:,2:].copy()
            value[:,:,2:]=normal@matrix  # inverse transpose of source->wall orientation
            af=alpha[:,:,None]
            result[section]=result[section]*(1-af)+value*af
            filled[section]=filled[section]*(1-alpha)+alpha
        if np.any(filled<.999): raise ValueError('quilt sample outside complete domain')
        return result


def pack(values):
    result=np.zeros(values.shape[:2],TEXEL)
    result['albedo']=np.uint16(np.clip(values[:,:,0]*65535+.5,0,65535))
    result['roughness']=np.uint8(np.clip(values[:,:,1]*255+.5,0,255))
    n=values[:,:,2:]
    n=n/np.maximum(1,np.linalg.norm(n,axis=2,keepdims=True)/.999)
    result['nx']=np.int16(np.clip(n[:,:,0]*32767,-32767,32767))
    result['nq']=np.int16(np.clip(n[:,:,1]*32767,-32767,32767))
    return result


def bake(config_path,spec_path,sources_path,recipe_path,output,texel_m=.001,brightness=.8):
    start=time.monotonic()
    config=yaml.safe_load(Path(config_path).read_text());spec=load_spec(spec_path)
    output=Path(output).resolve()
    if output.exists(): raise ValueError('surface output already exists')
    if not .0001<=texel_m<=.005: raise ValueError('invalid bake texel size')
    core,gutter=(spec['materials'][k] for k in ('tile_core_pixels','tile_gutter_pixels'))
    if core<=0 or not 1<=gutter<=16 or (core+2*gutter)**2*80>spec['resources']['asset_working_set_bytes']:
        raise ValueError('tile working set exceeds preparation budget')
    output.mkdir(parents=True)
    try:
        materials,guides=prepare_sources(sources_path,recipe_path,spec,output/'sources',brightness)
        r=config['tunnel']['radius_m'];q0=-math.pi*r;period=2*math.pi*r
        x0,x1=(config['tunnel'][k] for k in ('x_min_m','x_max_m'))
        # Exact cyclic period with a texel near the request avoids a discontinuous q seam.
        nq_pixels=math.ceil(period/texel_m);dq=period/nq_pixels
        nx_pixels=math.ceil((x1-x0)/texel_m)
        margin=(gutter+1)*texel_m
        bounds=[x0-margin,x0+nx_pixels*texel_m+margin,q0-margin,q0+period+margin]
        layout=quilt_layout(guides,[m['source_width_m'] for m in materials],bounds,spec['seed'],output/'quilt',
                            min_repeat_x_m=spec['materials']['min_repeat_distance_x_m'])
        sources=[(np.memmap(output/'sources'/m['file'],dtype=TEXEL,mode='r',shape=(m['side'],m['side'])),m['source_width_m']) for m in materials]
        sampler=QuiltSampler(layout,output/'quilt',sources)
        folder=output/'tiles';folder.mkdir()
        tiles=[]
        nx,nq=math.ceil(nx_pixels/core),math.ceil(nq_pixels/core)
        for iq in range(nq):
            for ix in range(nx):
                xp=ix*core+np.arange(-gutter,core+gutter)
                qp=iq*core+np.arange(-gutter,core+gutter)
                xs=x0+(xp+.5)*texel_m
                xs=np.clip(xs,x0+texel_m*.5,x0+(nx_pixels-.5)*texel_m)
                qs=q0+((qp+.5)%nq_pixels)*dq
                # Wrapped q gutters can be non-monotonic: sample each monotone run separately.
                cuts=np.r_[0,np.flatnonzero(np.diff(qs)<0)+1,len(qs)]
                values=np.concatenate([sampler.sample(xs,qs[a:b]) for a,b in zip(cuts[:-1],cuts[1:])])
                data=pack(values)
                file=f'x{ix:03d}_q{iq:03d}.bin';data.tofile(folder/file)
                tiles.append(dict(ix=ix,iq=iq,file='tiles/'+file,sha256=digest(folder/file),bytes=data.nbytes))
        manifest=dict(schema='ssb.surface_tiles.v1',tunnel=config['tunnel'],origin_xq_m=[x0,q0],period_q_m=period,
                      texel_xq_m=[texel_m,dq],pixels_xq=[nx_pixels,nq_pixels],tiles_xq=[nx,nq],
                      core_pixels=core,gutter_pixels=gutter,texel_format='mono16_rough8_reserved8_nx16_nq16_le',texel_bytes=8,
                      materials=materials,tiles=tiles,brightness=brightness,brightness_status='provisional_development_choice',
                      inputs={k:dict(file=str(Path(v).resolve()),sha256=digest(v)) for k,v in
                              dict(config=config_path,spec=spec_path,sources=sources_path,colour_recipe=recipe_path).items()},
                      layout_sha256=digest(output/'quilt/layout.json'),resources=spec['resources'],
                      background_detail_limit='Native source texels are 0.98-1.95 mm with the cached 2K package. Bake density is not optical resolution.',
                      preparation_seconds=time.monotonic()-start,preparation_peak_rss_bytes=peak_rss_bytes(),
                      tile_disk_bytes=sum(t['bytes'] for t in tiles))
        if manifest['preparation_peak_rss_bytes']>spec['resources']['asset_working_set_bytes']:
            raise ValueError('measured preparation peak exceeded budget')
        (output/'surface.json').write_text(json.dumps(manifest,indent=2)+'\n')
        print(json.dumps({k:manifest[k] for k in ('tiles_xq','tile_disk_bytes','preparation_seconds','preparation_peak_rss_bytes')}))
        return manifest
    except Exception as error:
        (output/'FAILED').write_text(str(error)+'\n')
        raise


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for arg in ('config','spec','sources','recipe','output'): parser.add_argument('--'+arg,required=True,type=Path)
    parser.add_argument('--texel-m',type=float,default=.001)
    parser.add_argument('--brightness',type=float,default=.8)
    args=parser.parse_args()
    bake(args.config,args.spec,args.sources,args.recipe,args.output,args.texel_m,args.brightness)



def tag_cracks(surface_path,defects_path,output,guard_m=.003):
    """Add a conservative acceleration flag AFTER background baking; never rasterise crack widths here."""
    import shutil
    surface_path,defects_path=Path(surface_path).resolve(),Path(defects_path).resolve()
    original=json.loads(surface_path.read_text());defects=json.loads(defects_path.read_text())
    output=Path(output).resolve()
    if output.exists(): raise ValueError('tagged surface output already exists')
    if not .002<=guard_m<=.01: raise ValueError('invalid adaptive guard')
    entry=defects['files']['segments.bin'];path=defects_path.parent/entry['file']
    if digest(path)!=entry['sha256']: raise ValueError('defect segment identity mismatch')
    from .stage_b_defects import SEGMENT
    segments=np.fromfile(path,dtype=SEGMENT)
    xlo=np.minimum(segments['x0'],segments['x1'])-guard_m-.00031
    xhi=np.maximum(segments['x0'],segments['x1'])+guard_m+.00031
    qlo=np.minimum(segments['q0'],segments['q1'])-guard_m-.00031
    qhi=np.maximum(segments['q0'],segments['q1'])+guard_m+.00031
    output.mkdir(parents=True)
    for folder in ('sources','quilt'): shutil.copytree(surface_path.parent/folder,output/folder)
    (output/'tiles').mkdir()
    core=original['core_pixels'];gutter=original['gutter_pixels'];side=core+2*gutter
    dx,dq=original['texel_xq_m'];x0,q0=original['origin_xq_m'];nx_pixels,nq_pixels=original['pixels_xq']
    marked=0;start=time.monotonic()
    for tile in original['tiles']:
        src=surface_path.parent/tile['file']
        if digest(src)!=tile['sha256']: raise ValueError('input surface tile hash mismatch')
        packed=np.fromfile(src,dtype=TEXEL).reshape(side,side)
        packed['reserved']=0
        xs=x0+(tile['ix']*core+np.arange(-gutter,core+gutter)+.5)*dx
        xs=np.clip(xs,x0+dx*.5,x0+(nx_pixels-.5)*dx)
        qs=q0+((tile['iq']*core+np.arange(-gutter,core+gutter)+.5)%nq_pixels)*dq
        candidates=np.flatnonzero((xhi>=xs.min())&(xlo<=xs.max())&(qhi>=qs.min())&(qlo<=qs.max()))
        for index in candidates:
            xi=np.flatnonzero((xs>=xlo[index])&(xs<=xhi[index]))
            yi=np.flatnonzero((qs>=qlo[index])&(qs<=qhi[index]))
            packed['reserved'][yi[:,None],xi[None,:]]=1
        marked+=int(packed['reserved'].sum())
        dest=output/tile['file'];packed.tofile(dest);tile['sha256']=digest(dest)
    original['adaptive_crack_guard_m']=guard_m
    original['adaptive_defects_sha256']=digest(defects_path)
    original['untagged_surface_sha256']=digest(surface_path)
    original['adaptive_layer']=dict(method='conservative metric segment bounding boxes, not crack opacity',
                                    guard_m=guard_m,marked_texels_including_gutters=marked,
                                    seconds=time.monotonic()-start,peak_rss_bytes=peak_rss_bytes(),
                                    generator_sha256=digest(Path(__file__)))
    (output/'surface.json').write_text(json.dumps(original,indent=2)+'\n')
    print(json.dumps(original['adaptive_layer']))
    return original


def tag_main():
    p=argparse.ArgumentParser(description=tag_cracks.__doc__)
    for arg in ('surface','defects','output'): p.add_argument('--'+arg,required=True,type=Path)
    p.add_argument('--guard-m',type=float,default=.003)
    a=p.parse_args();tag_cracks(a.surface,a.defects,a.output,a.guard_m)


def preview_texture(surface_path,output,max_side=2048):
    from .stage_b_materials import linear_to_srgb
    surface_path=Path(surface_path).resolve()
    surface=json.loads(surface_path.read_text())
    layout_path=surface_path.parent/'quilt/layout.json'
    if digest(layout_path)!=surface['layout_sha256']: raise ValueError('preview quilt identity mismatch')
    layout=json.loads(layout_path.read_text())
    guide=cv2.imread(str(surface_path.parent/'quilt/guide.png'),cv2.IMREAD_GRAYSCALE)
    if guide is None: raise ValueError('missing quilt guide')
    x0,x1=(surface['tunnel'][k] for k in ('x_min_m','x_max_m'))
    q0=surface['origin_xq_m'][1];period=surface['period_q_m']
    if not 32<=max_side<=2048: raise ValueError('invalid overview size')
    width=max_side;height=round(width*period/(x1-x0))
    if height>max_side: height=max_side;width=round(height*(x1-x0)/period)
    xs=x0+(np.arange(width)+.5)*(x1-x0)/width
    qs=q0+period-(np.arange(height)+.5)*period/height
    u,v=np.meshgrid((xs-layout['origin_xq_m'][0])/layout['guide_texel_m']-.5,
                    (qs-layout['origin_xq_m'][1])/layout['guide_texel_m']-.5)
    image=cv2.remap(guide,u.astype('float32'),v.astype('float32'),cv2.INTER_LINEAR,
                    borderMode=cv2.BORDER_REPLICATE).astype('float32')/255
    Image.fromarray(np.uint8(linear_to_srgb(image)*255+.5)).save(output)
    return dict(file=str(output),size=[width,height],purpose='Gazebo coarse overview, not line-camera image')


if __name__=='__main__': main()
