"""Native high-resolution source preparation and compact deterministic quilt recipes.

libvips sequential PNG decode preserves source bit depth. Only bounded strips are
converted to float; optical texels are generated on demand instead of saved for
the entire tunnel. Downloading honours the inherited proxy configuration.
"""
import argparse
import json
import math
import mmap
import os
from pathlib import Path
import subprocess
import time

import cv2
import numpy as np
import yaml

from .stage_b_scene import digest, load_spec, peak_rss_bytes
from .stage_b_materials import download, srgb_to_linear
from .stage_b_surface import TEXEL, quilt_layout, pack


def fetch(output, resolution='16k'):
    output=Path(output).resolve()
    if output.exists(): raise ValueError('source output exists; do not overwrite')
    output.mkdir(parents=True)
    channels={}
    for name,suffix in [('diffuse','diff'),('normal_gl','nor_gl'),('roughness','rough')]:
        file=f'plastered_wall_04_{suffix}_{resolution}.png'
        url=f'https://dl.polyhaven.org/file/ph-assets/Textures/png/{resolution}/plastered_wall_04/{file}'
        download(url,output/file,2<<30)
        header=subprocess.check_output(['vipsheader','-a',str(output/file)],text=True)
        channels[name]=dict(file=file,url=url,sha256=digest(output/file),bytes=(output/file).stat().st_size,header=header)
    data=dict(asset='plastered_wall_04',resolution=resolution,source_width_m=3.2,
              source_url='https://polyhaven.com/a/plastered_wall_04',license='CC0',channels=channels)
    (output/'downloads.json').write_text(json.dumps(data,indent=2)+'\n')
    return data


def _decode(path,raw):
    header=subprocess.check_output(['vipsheader','-a',str(path)],text=True)
    fields={l.split(':',1)[0]:l.split(':',1)[1].strip() for l in header.splitlines()[1:] if ':' in l}
    width,height=int(fields['width']),int(fields['height'])
    if fields['format'] not in ['uchar','ushort']:
        raise ValueError('native uchar/ushort sources required')
    dtype=np.dtype('u1' if fields['format']=='uchar' else '<u2');bands=int(fields['bands'])
    # Input requests sequential access; libvips cache is separately bounded.
    env=dict(os.environ,VIPS_CONCURRENCY='2',VIPS_DISC_THRESHOLD='64m')
    subprocess.run(['vips','rawsave',str(path)+'[access=sequential]',str(raw),'--vips-cache-max-memory=67108864'],env=env,check=True)
    if raw.stat().st_size!=width*height*bands*dtype.itemsize: raise ValueError('decoded dimensions mismatch')
    return np.memmap(raw,dtype=dtype,mode='r',shape=(height,width,bands))


def _pack_channels(source, root, target, scratch, albedo_scale):
    """Decode diffuse/normal/roughness in bounded strips into one packed texel file.
    Albedo is linear luminance times albedo_scale. Returns (channel records, (width, height))."""
    packed=None;shape=None;channels={}
    for name in ['diffuse','normal_gl','roughness']:
        entry=source['channels'][name];path=Path(root)/entry['file']
        if digest(path)!=entry['sha256']:raise ValueError('source hash mismatch')
        raw=Path(scratch)/('decode_'+name+'.raw');decoded=_decode(path,raw)
        if shape is None:
            shape=decoded.shape[:2];packed=np.memmap(target,dtype=TEXEL,mode='w+',shape=shape)
        if decoded.shape[:2]!=shape:raise ValueError('PBR channels not co-registered')
        if name in ['diffuse','normal_gl'] and decoded.shape[2]<3:raise ValueError('RGB source required')
        maxcode=np.iinfo(decoded.dtype).max
        for row in range(0,shape[0],128):
            section=np.s_[row:row+128,:];value=decoded[section].astype(np.float32)/maxcode
            if name=='diffuse':
                mono=srgb_to_linear(value[:,:,:3])@np.array([.2126,.7152,.0722],np.float32)
                packed['albedo'][section]=np.uint16(np.clip(mono*albedo_scale*65535+.5,0,65535));packed['reserved'][section]=0
            elif name=='normal_gl':
                n=value[:,:,:3]*2-1;n/=np.maximum(np.linalg.norm(n,axis=2,keepdims=True),1e-6)
                packed['nx'][section]=np.int16(np.clip(n[:,:,0]*32767,-32767,32767))
                packed['nq'][section]=np.int16(np.clip(-n[:,:,1]*32767,-32767,32767))
            else:packed['roughness'][section]=np.uint8(np.clip(value[:,:,0]*255+.5,0,255))
            packed.flush();decoded._mmap.madvise(mmap.MADV_DONTNEED);packed._mmap.madvise(mmap.MADV_DONTNEED)
        channels[name]=dict(**entry,native_size=[shape[1],shape[0]],native_bits=decoded.dtype.itemsize*8)
        decoded._mmap.close();del decoded;raw.unlink() # owned, disposable decoder scratch only
    packed._mmap.close();return channels,(shape[1],shape[0])


def native_grid(period, native):
    """Generated pitch equal to the source pitch, adjusted (a few ppm) so the q period holds whole texels."""
    pixel_q=round(period/native)
    return period/pixel_q,pixel_q


def align_offsets(placements, grid_origin, guide_origin, guide_step, texel, material=None):
    """Snap source offsets to whole texels (<= half a texel shift), so every generated texel
    centre lands on a source texel centre: single-patch texels are exact copies and the
    camera sample is the only interpolation. Orientations are orthogonal, so this holds
    for all eight of them."""
    for p in placements:
        if material is not None and p['material']!=material:continue
        m=np.asarray(p['source_matrix'],float)
        base=m@(np.subtract(grid_origin,guide_origin)-np.array([p['left'],p['top']])*guide_step)
        p['source_offset_m']=(texel*np.round((base+np.asarray(p['source_offset_m']))/texel)-base).tolist()


def prepare(downloads, config_path, spec_path, output, brightness=.8, working_set=4<<30):
    start=time.monotonic();downloads=Path(downloads).resolve();source=json.loads(downloads.read_text())
    spec=load_spec(spec_path);config=yaml.safe_load(Path(config_path).read_text());output=Path(output).resolve()
    if output.exists():raise ValueError('runtime surface output exists; do not overwrite')
    if source['asset']!='plastered_wall_04' or source['source_width_m']!=3.2:raise ValueError('Wall 04 native scale required')
    if not 0<brightness<=1 or working_set<512<<20:raise ValueError('invalid preparation parameters')
    output.mkdir(parents=True)
    try:
        input_channels,(side,_)=_pack_channels(source,downloads.parent,output/'source.bin',output,brightness)
        packed=np.memmap(output/'source.bin',dtype=TEXEL,mode='r+',shape=(side,side))
        guide_step=.02;gs=round(3.2/guide_step)
        centres=np.minimum(side-1,((np.arange(gs)+.5)*side/gs).astype(int))
        guide=packed['albedo'][centres[:,None],centres[None,:]].astype(np.float32)/65535
        packed.flush();packed._mmap.close();del packed
        r=config['tunnel']['radius_m'];period=2*math.pi*r;q0=-math.pi*r
        x0,x1=[config['tunnel'][k] for k in ['x_min_m','x_max_m']]
        texel_m,pixel_q=native_grid(period,3.2/side);dq=texel_m;pixel_x=math.ceil((x1-x0)/texel_m-1e-9)
        core=spec['materials']['tile_core_pixels'];gutter=spec['materials']['tile_gutter_pixels'];margin=(gutter+1)*texel_m
        layout=quilt_layout([guide],[3.2],[x0-margin,x0+pixel_x*texel_m+margin,q0-margin,q0+period+margin],
            spec['seed'],output/'quilt',min_repeat_x_m=spec['materials']['min_repeat_distance_x_m'],near_crop_m=.08,candidates=96)
        # One compact mask per placement; source data is never expanded to wall size.
        align_offsets(layout['placements'],[x0,q0],layout['origin_xq_m'],layout['guide_texel_m'],texel_m)
        alpha=np.stack([cv2.imread(str(output/'quilt'/p['alpha']),cv2.IMREAD_GRAYSCALE) for p in layout['placements']])
        alpha.tofile(output/'alpha.bin')
        recipe=dict(schema='ssb.surface_recipe.v1',interpolation='bilinear_fixed32_v1',origin_xq_m=layout['origin_xq_m'],
            guide_texel_m=guide_step,patch_pixels=layout['patch_pixels'],placements=layout['placements'],
            offsets_aligned_to_texel_m=texel_m, # quilt/layout.json keeps the unaligned offsets
            sources=[dict(id='plastered_wall_04',file='source.bin',side=side,source_width_m=texel_m*side,nominal_width_m=3.2,native_texel_m=texel_m,sha256=digest(output/'source.bin'))],
            alpha=dict(file='alpha.bin',sha256=digest(output/'alpha.bin')),brightness=brightness,
            source_downloads_sha256=digest(downloads),input_channels=input_channels)
        (output/'recipe.json').write_text(json.dumps(recipe,indent=2)+'\n')
        surface=dict(schema='ssb.surface_runtime.v1',tunnel=config['tunnel'],origin_xq_m=[x0,q0],period_q_m=period,
            texel_xq_m=[texel_m,dq],pixels_xq=[pixel_x,pixel_q],tiles_xq=[math.ceil(pixel_x/core),math.ceil(pixel_q/core)],
            core_pixels=core,gutter_pixels=gutter,texel_format='mono16_rough8_reserved8_nx16_nq16_le',texel_bytes=8,
            recipe=dict(file='recipe.json',sha256=digest(output/'recipe.json')),resources={**spec['resources'],'asset_working_set_bytes':working_set,'gpu_source_budget_bytes':4<<30},
            layout_sha256=digest(output/'quilt/layout.json'),brightness=brightness,
            native_detail_m=texel_m,source_scale_ppm=(texel_m*side/3.2-1)*1e6,preparation_seconds=time.monotonic()-start,preparation_peak_rss_bytes=peak_rss_bytes(),
            source_disk_bytes=(output/'source.bin').stat().st_size,generated_tile_disk_bytes=0,
            assumptions='Native scale is 3.2 m, adjusted by source_scale_ppm so the generated grid equals the source grid; no absolute reflectance calibration. Single-source crop-centre guard 0.08 m, not perceptual uniqueness.',
            inputs={k:dict(file=str(Path(v).resolve()),sha256=digest(v)) for k,v in dict(config=config_path,spec=spec_path,downloads=downloads).items()})
        if surface['preparation_peak_rss_bytes']>working_set:raise ValueError('preparation exceeded working-set budget')
        (output/'surface.json').write_text(json.dumps(surface,indent=2)+'\n')
        return surface
    except Exception as e:
        (output/'FAILED').write_text(str(e)+'\n');raise


def build_macro(item, sources_root, output, period, x0, x1, panels, seed):
    """Low-frequency albedo modulation map over the whole wall (synthetic arrangement of a real
    wall's large-scale variation, plus per-segment tone). Returns the recipe entry."""
    downloads=Path(sources_root)/item['dir']/'downloads.json';source=json.loads(downloads.read_text())
    entry=source['channels']['diffuse'];path=downloads.parent/entry['file']
    if digest(path)!=entry['sha256']:raise ValueError('macro source hash mismatch')
    raw=Path(output)/'decode_macro.raw';decoded=_decode(path,raw)
    v=decoded.astype(np.float32)/np.iinfo(decoded.dtype).max;decoded._mmap.close();del decoded;raw.unlink()
    lum=srgb_to_linear(v[:,:,:3])@np.array([.2126,.7152,.0722],np.float32);del v
    nq=round(period/item['pitch_m']);pitch=period/nq;extent=source['source_width_m']
    side=round(extent/pitch)
    low=cv2.resize(lum,(side,side),interpolation=cv2.INTER_AREA)
    # Keep variations longer than cutoff_m only; wrap: the source is seamless.
    sigma=item['cutoff_m']/pitch/2
    pad=side//2;low=cv2.GaussianBlur(np.pad(low,pad,mode='wrap'),(0,0),sigma)[pad:-pad,pad:-pad]
    field=np.log(low/low.mean())*item.get('strength',1.)
    nx=math.ceil((x1-x0)/pitch)+1
    rng=np.random.default_rng(seed);B=side  # one block per source extent
    acc=np.zeros((nq,nx),np.float64);w2=np.zeros((nq,nx),np.float64)
    for bq in range(-1,nq//B+2):
        for bx in range(-1,nx//B+2):
            k=int(rng.integers(8));f=np.rot90(field,k//2);f=np.fliplr(f) if k%2 else f
            oy,ox=map(int,rng.integers(0,B,size=2))
            cy,cx=bq*B+B//2,bx*B+B//2   # block centre (macro pixels)
            ys=np.arange(cy-B,cy+B);xs=np.arange(cx-B,cx+B)
            wy=np.clip(1-np.abs(ys-cy)/B,0,None);wx=np.clip(1-np.abs(xs-cx)/B,0,None)
            val=f[np.ix_((ys+oy)%B,(xs+ox)%B)]
            yi=ys%nq;keep=(xs>=0)&(xs<nx)
            w=np.outer(wy,wx[keep]);np.add.at(acc,(yi[:,None],xs[keep][None,:]),w*val[:,keep]);np.add.at(w2,(yi[:,None],xs[keep][None,:]),w*w)
    field=acc/np.sqrt(np.maximum(w2,1e-12))   # variance-preserving blend of overlapping blocks
    mod=np.exp(field-field.mean())
    # Per-segment tone, same ring/segment partition as the panel geometry.
    tone=item.get('segment_tone_sigma',0.)
    if tone:
        xs=x0+(np.arange(nx)+.5)*pitch;qs=-period/2+(np.arange(nq)+.5)*pitch
        ring=np.floor(xs/panels['ring_width_m']).astype(int)
        edges=np.cumsum([0]+list(panels['angles_deg']))
        factors={};srng=np.random.default_rng(seed+1)
        deg=np.degrees(qs/(period/(2*math.pi)))
        for r in np.unique(ring):
            a0=-panels['angles_deg'][0]/2+(r%2)*panels['alternating_stagger_deg']
            index=np.searchsorted(edges,np.mod(deg-a0,360),side='right')-1
            f=1+tone*np.clip(srng.standard_normal(len(panels['angles_deg'])),-3,3)
            mod[:,ring==r]*=f[index][:,None]
    scale=32768;code=np.uint16(np.clip(np.rint(mod*scale),0,65535))
    code.tofile(Path(output)/'macro.bin')
    return dict(file='macro.bin',sha256=digest(Path(output)/'macro.bin'),width=nx,height=nq,origin_xq_m=[x0,-period/2],pitch_m=pitch,scale=scale,
                source=item['dir'],source_downloads_sha256=digest(downloads),cutoff_m=item['cutoff_m'],strength=item.get('strength',1.),
                segment_tone_sigma=tone,block_m=B*pitch,std=float(mod.std()),
                synthetic='arrangement (random orientation/offset per block, variance-preserving blend) and segment tone are synthetic; variation content is a real wall low-pass')


def prepare_set(set_path, sources_root, config_path, spec_path, output, working_set=4<<30):
    """Multi-source runtime surface from a material set (non-square sources, usable-region masks).

    The generated grid either aligns to one source (grid.align_to; that source's scale is
    adjusted by a few ppm so the q period holds whole texels) or uses grid.pitch_m.
    """
    start=time.monotonic();sources_root=Path(sources_root).resolve();output=Path(output).resolve()
    ms=yaml.safe_load(Path(set_path).read_text());spec=load_spec(spec_path);config=yaml.safe_load(Path(config_path).read_text())
    if ms.get('schema')!='ssb.material_set.v1':raise ValueError('material set schema')
    if output.exists():raise ValueError('runtime surface output exists; do not overwrite')
    brightness=ms['brightness'];lay=ms['layout'];grid=ms['grid']
    if not 0<brightness<=1 or working_set<512<<20 or not 1<=len(ms['sources'])<=8:raise ValueError('invalid preparation parameters')
    output.mkdir(parents=True)
    try:
        r=config['tunnel']['radius_m'];period=2*math.pi*r;q0=-math.pi*r;x0,x1=[config['tunnel'][k] for k in ['x_min_m','x_max_m']]
        guide_step=lay['guide_texel_m'];entries=[];guides=[];valid=[];extents=[]
        for item in ms['sources']:
            downloads=sources_root/item['dir']/'downloads.json';source=json.loads(downloads.read_text())
            gain=float(item.get('albedo_gain',1.))
            if not 0<gain*brightness<=1.5:raise ValueError('invalid albedo gain')
            channels,(w,h)=_pack_channels(source,downloads.parent,output/(item['id']+'.bin'),output,brightness*gain)
            if abs(source['source_width_m']/w-source['source_height_m']/h)>1e-3*source['source_width_m']/w:
                raise ValueError('non-square texels')
            native=source['source_width_m']/w
            packed=np.memmap(output/(item['id']+'.bin'),dtype=TEXEL,mode='r',shape=(h,w))
            gw,gh=round(w*native/guide_step),round(h*native/guide_step)
            guides.append(cv2.resize(np.asarray(packed['albedo']),(gw,gh),interpolation=cv2.INTER_AREA).astype(np.float32)/65535)
            del packed
            if item.get('mask'):
                m=cv2.imread(str(sources_root/item['mask']),cv2.IMREAD_GRAYSCALE)
                if m is None or m.shape[1]*h!=m.shape[0]*w:raise ValueError('mask missing or aspect mismatch')
                # A guide cell is usable only if every mask pixel inside it is usable.
                valid.append(cv2.resize((m>=128).astype(np.float32),(gw,gh),interpolation=cv2.INTER_AREA)>=.999)
            else:valid.append(np.ones((gh,gw),bool))
            # Authored exclusions (source metres, image x right / y down): a guide cell is
            # invalid if any part of it lies within a circle.
            for cx,cy,rad in item.get('exclude_circles_m',[]):
                gy,gx=np.mgrid[:gh,:gw]
                dx=np.maximum(np.abs((gx+.5)*guide_step-cx)-guide_step/2,0);dy=np.maximum(np.abs((gy+.5)*guide_step-cy)-guide_step/2,0)
                valid[-1]&=np.hypot(dx,dy)>rad
            extents.append((w*native,h*native))
            entries.append(dict(id=item['id'],file=item['id']+'.bin',width=w,height=h,source_width_m=w*native,nominal_width_m=source['source_width_m'],
                native_texel_m=native,albedo_gain=gain,weight=item.get('weight'),mask=item.get('mask'),exclude_circles_m=item.get('exclude_circles_m',[]),
                mask_sha256=digest(sources_root/item['mask']) if item.get('mask') else None,
                downloads_sha256=digest(downloads),input_channels=channels))
        ids=[e['id'] for e in entries]
        if 'align_to' in grid:
            aligned=ids.index(grid['align_to']);texel_m,pixel_q=native_grid(period,entries[aligned]['native_texel_m'])
            e=entries[aligned];e['source_scale_ppm']=(texel_m/e['native_texel_m']-1)*1e6;e['native_texel_m']=texel_m;e['source_width_m']=texel_m*e['width']
        else:
            aligned=None;texel_m,pixel_q=native_grid(period,grid['pitch_m'])
        pixel_x=math.ceil((x1-x0)/texel_m-1e-9)
        core=spec['materials']['tile_core_pixels'];gutter=spec['materials']['tile_gutter_pixels'];margin=(gutter+1)*texel_m
        weights=[item.get('weight') for item in ms['sources']]
        weights=None if None in weights else list(np.asarray(weights,float)/sum(weights))
        layout=quilt_layout(guides,extents,[x0-margin,x0+pixel_x*texel_m+margin,q0-margin,q0+period+margin],ms['seed'],output/'quilt',
            patch_m=lay['patch_m'],overlap_m=lay['overlap_m'],guide_texel_m=guide_step,min_repeat_x_m=lay['min_repeat_m'],
            near_crop_m=lay['near_crop_m'],candidates=lay['candidates'],valid=valid,weights=weights,repeat_metric='wall',
            max_same_orientation_overlap=lay.get('max_same_orientation_overlap'),
            orientations=[item.get('orientations',list(range(8))) for item in ms['sources']])
        if aligned is not None:align_offsets(layout['placements'],[x0,q0],layout['origin_xq_m'],guide_step,texel_m,material=aligned)
        alpha=np.stack([cv2.imread(str(output/'quilt'/p['alpha']),cv2.IMREAD_GRAYSCALE) for p in layout['placements']])
        alpha.tofile(output/'alpha.bin')
        for e in entries:e['sha256']=digest(output/e['file'])
        macro=build_macro(ms['macro'],sources_root,output,period,x0,x0+pixel_x*texel_m,spec['panels'],ms['seed']) if ms.get('macro') else None
        recipe=dict(schema='ssb.surface_recipe.v1',interpolation='bilinear_fixed32_v1',origin_xq_m=layout['origin_xq_m'],
            guide_texel_m=guide_step,patch_pixels=layout['patch_pixels'],placements=layout['placements'],
            offsets_aligned_to_texel_m=texel_m if aligned is not None else None,aligned_source=ids[aligned] if aligned is not None else None,
            sources=entries,alpha=dict(file='alpha.bin',sha256=digest(output/'alpha.bin')),brightness=brightness,
            material_set_sha256=digest(set_path),macro=macro)
        (output/'recipe.json').write_text(json.dumps(recipe,indent=2)+'\n')
        mix={i:sum(p['material']==n for p in layout['placements'])/len(layout['placements']) for n,i in enumerate(ids)}
        surface=dict(schema='ssb.surface_runtime.v1',tunnel=config['tunnel'],origin_xq_m=[x0,q0],period_q_m=period,
            texel_xq_m=[texel_m,texel_m],pixels_xq=[pixel_x,pixel_q],tiles_xq=[math.ceil(pixel_x/core),math.ceil(pixel_q/core)],
            core_pixels=core,gutter_pixels=gutter,texel_format='mono16_rough8_reserved8_nx16_nq16_le',texel_bytes=8,
            recipe=dict(file='recipe.json',sha256=digest(output/'recipe.json')),resources={**spec['resources'],'asset_working_set_bytes':working_set,'gpu_source_budget_bytes':4<<30},
            layout_sha256=digest(output/'quilt/layout.json'),brightness=brightness,native_detail_m=min(e['native_texel_m'] for e in entries),
            placement_fraction=mix,preparation_seconds=time.monotonic()-start,preparation_peak_rss_bytes=peak_rss_bytes(),
            source_disk_bytes=sum((output/e['file']).stat().st_size for e in entries),generated_tile_disk_bytes=0,
            assumptions='Source scales from publisher metadata; aligned source adjusted by source_scale_ppm. Usable-region masks exclude authored defects. Repeat guard is 2D wall distance, not perceptual uniqueness.',
            inputs={k:dict(file=str(Path(v).resolve()),sha256=digest(v)) for k,v in dict(config=config_path,spec=spec_path,material_set=set_path).items()})
        if surface['preparation_peak_rss_bytes']>working_set:raise ValueError('preparation exceeded working-set budget')
        (output/'surface.json').write_text(json.dumps(surface,indent=2)+'\n')
        return surface
    except Exception as e:
        (output/'FAILED').write_text(str(e)+'\n');raise


def prepare_filler(downloads, output, mean_albedo=.17, flatten_m=.01, contrast=.35):
    """Joint filler (grey mortar) albedo map: fine detail of a real sandy plaster, tone above
    flatten_m removed (moss, shading), mean normalised; tiled by wall (x, q) at render time."""
    downloads=Path(downloads).resolve();output=Path(output).resolve();source=json.loads(downloads.read_text())
    if output.exists():raise ValueError('filler output exists; do not overwrite')
    if not 0<mean_albedo<1 or flatten_m<=0 or not 0<contrast<=1:raise ValueError('invalid filler parameters')
    entry=source['channels']['diffuse'];path=downloads.parent/entry['file']
    if digest(path)!=entry['sha256']:raise ValueError('filler source hash mismatch')
    output.mkdir(parents=True)
    raw=output/'decode.raw';decoded=_decode(path,raw);h,w=decoded.shape[:2]
    v=decoded.astype(np.float32)/np.iinfo(decoded.dtype).max;decoded._mmap.close();del decoded;raw.unlink()
    lum=srgb_to_linear(v[:,:,:3])@np.array([.2126,.7152,.0722],np.float32);del v
    pitch=source['source_width_m']/w;sigma=flatten_m/pitch;pad=int(3*sigma)+1
    low=cv2.GaussianBlur(np.pad(lum,pad,mode='wrap'),(0,0),sigma)[pad:-pad,pad:-pad]   # seamless source
    detail=lum/np.maximum(low,1e-6);detail/=detail.mean()
    # The source keeps residual grain shading; filler faces have no normal map, so this is the
    # only grain cue, scaled down to a sandy-mortar level (assumption).
    detail=np.clip(1+contrast*(detail-1),0,None)
    scale=32768;code=np.uint16(np.clip(np.rint(detail*scale),0,65535));code.tofile(output/'filler.bin')
    meta=dict(schema='ssb.joint_filler.v1',file='filler.bin',sha256=digest(output/'filler.bin'),width=w,height=h,pitch_m=pitch,scale=scale,
              mean_albedo=mean_albedo,roughness=.9,std=float(detail.std()),flatten_m=flatten_m,contrast=contrast,source_downloads_sha256=digest(downloads),
              assumption='Grey mortar appearance (user choice) from a real sandy plaster; mean albedo is an assumption, not a measurement.')
    (output/'filler.json').write_text(json.dumps(meta,indent=2)+'\n');return meta


class ReferenceRecipe:
    """Float64 NumPy reference, independently samples the packed native sources."""
    def __init__(self,surface_path):
        path=Path(surface_path).resolve();self.surface=json.loads(path.read_text());s=self.surface
        rp=path.parent/s['recipe']['file']
        if digest(rp)!=s['recipe']['sha256']:raise ValueError('recipe identity mismatch')
        self.recipe=json.loads(rp.read_text());self.sources=[]
        for e in self.recipe['sources']:
            p=rp.parent/e['file']
            if digest(p)!=e['sha256']:raise ValueError('source identity mismatch')
            self.sources.append(np.memmap(p,TEXEL,mode='r',shape=(e.get('height',e.get('side')),e.get('width',e.get('side')))))
        e=self.recipe['alpha'];p=rp.parent/e['file']
        if digest(p)!=e['sha256']:raise ValueError('alpha identity mismatch')
        n=self.recipe['patch_pixels'];self.alpha=np.fromfile(p,np.uint8).reshape(-1,n,n)
        self.macro=None;m=self.recipe.get('macro')
        if m:
            p=rp.parent/m['file']
            if digest(p)!=m['sha256']:raise ValueError('macro identity mismatch')
            self.macro=np.fromfile(p,'<u2').reshape(m['height'],m['width'])

    def macro_at(self,xs,qs):
        m=self.recipe['macro'];X,Q=np.meshgrid(np.asarray(xs,float),np.asarray(qs,float))
        u=(X-m['origin_xq_m'][0])/m['pitch_m']-.5;v=np.mod((Q-m['origin_xq_m'][1])/m['pitch_m'],m['height'])-.5
        x0=np.floor(u).astype(int);y0=np.floor(v).astype(int);fx=u-x0;fy=v-y0;out=np.zeros(X.shape)
        for j in range(2):
            for i in range(2):
                out+=self.macro[np.mod(y0+j,m['height']),np.clip(x0+i,0,m['width']-1)]*(fx if i else 1-fx)*(fy if j else 1-fy)
        return out/m['scale']

    @staticmethod
    def interp(image,x,y):
        x=np.floor(x*32+.5).astype(np.int64);y=np.floor(y*32+.5).astype(np.int64)
        ix,fx=np.divmod(x,32);iy,fy=np.divmod(y,32)
        out=np.zeros(x.shape,np.float64)
        for j in range(2):
            for i in range(2):
                out+=image[np.clip(iy+j,0,image.shape[0]-1),np.clip(ix+i,0,image.shape[1]-1)].astype(np.float64)*(fx if i else 32-fx)*(fy if j else 32-fy)/1024
        return out

    def sample(self,xs,qs):
        r=self.recipe;gx=(np.asarray(xs)-r['origin_xq_m'][0])/r['guide_texel_m'];gy=(np.asarray(qs)-r['origin_xq_m'][1])/r['guide_texel_m']
        out=np.zeros((len(qs),len(xs),4),np.float64);filled=np.zeros(out.shape[:2]);side=r['patch_pixels']
        for id,p in enumerate(r['placements']):
            xi=np.flatnonzero((gx>=p['left'])&(gx<p['left']+side));yi=np.flatnonzero((gy>=p['top'])&(gy<p['top']+side))
            if not len(xi) or not len(yi):continue
            px,py=np.meshgrid(gx[xi]-p['left'],gy[yi]-p['top']);a=self.interp(self.alpha[id],px-.5,py-.5)/255
            m=np.array(p['source_matrix']);o=p['source_offset_m'];e=r['sources'][p['material']];src=self.sources[p['material']];native=e['source_width_m']/e.get('width',e.get('side'))
            u=(m[0,0]*px*r['guide_texel_m']+m[0,1]*py*r['guide_texel_m']+o[0])/native-.5
            v=(m[1,0]*px*r['guide_texel_m']+m[1,1]*py*r['guide_texel_m']+o[1])/native-.5
            value=np.stack([self.interp(src[c],u,v)/scale for c,scale in [('albedo',65535),('roughness',255),('nx',32767),('nq',32767)]],axis=-1)
            value[:,:,2:]=value[:,:,2:]@m
            idx=np.ix_(yi,xi);out[idx]=out[idx]*(1-a[:,:,None])+value*a[:,:,None];filled[idx]=filled[idx]*(1-a)+a
        if (filled<.999).any():raise ValueError('reference unfilled texels')
        if self.macro is not None:out[:,:,0]*=self.macro_at(xs,qs)
        return pack(out)

    def tile(self,index):
        s=self.surface;core=s['core_pixels'];g=s['gutter_pixels'];nx=s['tiles_xq'][0];dx,dq=s['texel_xq_m'];x0,q0=s['origin_xq_m'];px,pq=s['pixels_xq']
        xs=x0+np.clip(index%nx*core+np.arange(-g,core+g)+.5,.5,px-.5)*dx
        qs=q0+((index//nx*core+np.arange(-g,core+g)+.5)%pq)*dq
        return self.sample(xs,qs)


def main():
    p=argparse.ArgumentParser(description=__doc__);sub=p.add_subparsers(dest='command',required=True)
    f=sub.add_parser('fetch');f.add_argument('--output',required=True);f.add_argument('--resolution',choices=['8k','16k'],default='16k')
    a=sub.add_parser('prepare')
    for k in ['downloads','config','spec','output']:a.add_argument('--'+k,required=True)
    a.add_argument('--brightness',type=float,default=.8)
    fl=sub.add_parser('prepare-filler');fl.add_argument('--downloads',required=True);fl.add_argument('--output',required=True)
    fl.add_argument('--mean-albedo',type=float,default=.17);fl.add_argument('--contrast',type=float,default=.35)
    m=sub.add_parser('prepare-set')
    for k in ['set','sources','config','spec','output']:m.add_argument('--'+k,required=True)
    args=p.parse_args();kw=vars(args);command=kw.pop('command')
    if command in ['prepare','prepare-set']:kw['config_path']=kw.pop('config');kw['spec_path']=kw.pop('spec')
    if command=='prepare-set':kw['set_path']=kw.pop('set');kw['sources_root']=kw.pop('sources')
    if command=='prepare-filler':print(json.dumps(prepare_filler(kw['downloads'],kw['output'],kw['mean_albedo'],contrast=kw['contrast'])));return
    result=dict(fetch=fetch,prepare=prepare)[command](**kw) if command!='prepare-set' else prepare_set(**kw)
    print(json.dumps({k:v for k,v in result.items() if k not in ['channels','inputs']}))


if __name__=='__main__':main()
