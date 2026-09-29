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
    side=int(fields['width'])
    if side!=int(fields['height']) or fields['format'] not in ['uchar','ushort']:
        raise ValueError('square native uchar/ushort sources required')
    dtype=np.dtype('u1' if fields['format']=='uchar' else '<u2');bands=int(fields['bands'])
    # Input requests sequential access; libvips cache is separately bounded.
    env=dict(os.environ,VIPS_CONCURRENCY='2',VIPS_DISC_THRESHOLD='64m')
    subprocess.run(['vips','rawsave',str(path)+'[access=sequential]',str(raw),'--vips-cache-max-memory=67108864'],env=env,check=True)
    if raw.stat().st_size!=side*side*bands*dtype.itemsize: raise ValueError('decoded dimensions mismatch')
    return np.memmap(raw,dtype=dtype,mode='r',shape=(side,side,bands))


def native_grid(period, native):
    """Generated pitch equal to the source pitch, adjusted (a few ppm) so the q period holds whole texels."""
    pixel_q=round(period/native)
    return period/pixel_q,pixel_q


def align_offsets(placements, grid_origin, guide_origin, guide_step, texel):
    """Snap source offsets to whole texels (<= half a texel shift), so every generated texel
    centre lands on a source texel centre: single-patch texels are exact copies and the
    camera sample is the only interpolation. Orientations are orthogonal, so this holds
    for all eight of them."""
    for p in placements:
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
        packed=None;side=None;input_channels={}
        for name in ['diffuse','normal_gl','roughness']:
            entry=source['channels'][name];path=downloads.parent/entry['file']
            if digest(path)!=entry['sha256']:raise ValueError('source hash mismatch')
            raw=output/('decode_'+name+'.raw');decoded=_decode(path,raw)
            if side is None:
                side=decoded.shape[0];packed=np.memmap(output/'source.bin',dtype=TEXEL,mode='w+',shape=(side,side))
            if decoded.shape[0]!=side:raise ValueError('PBR channels not co-registered')
            if name in ['diffuse','normal_gl'] and decoded.shape[2]<3:raise ValueError('RGB source required')
            maxcode=np.iinfo(decoded.dtype).max
            for row in range(0,side,128):
                section=np.s_[row:row+128,:];value=decoded[section].astype(np.float32)/maxcode
                if name=='diffuse':
                    mono=srgb_to_linear(value[:,:,:3])@np.array([.2126,.7152,.0722],np.float32)
                    packed['albedo'][section]=np.uint16(np.clip(mono*brightness*65535+.5,0,65535));packed['reserved'][section]=0
                elif name=='normal_gl':
                    n=value[:,:,:3]*2-1;n/=np.maximum(np.linalg.norm(n,axis=2,keepdims=True),1e-6)
                    packed['nx'][section]=np.int16(np.clip(n[:,:,0]*32767,-32767,32767))
                    packed['nq'][section]=np.int16(np.clip(-n[:,:,1]*32767,-32767,32767))
                else:packed['roughness'][section]=np.uint8(np.clip(value[:,:,0]*255+.5,0,255))
                packed.flush();decoded._mmap.madvise(mmap.MADV_DONTNEED);packed._mmap.madvise(mmap.MADV_DONTNEED)
            input_channels[name]=dict(**entry,native_side=side,native_bits=decoded.dtype.itemsize*8)
            decoded._mmap.close();del decoded;raw.unlink() # owned, disposable decoder scratch only
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
            self.sources.append(np.memmap(p,TEXEL,mode='r',shape=(e['side'],e['side'])))
        e=self.recipe['alpha'];p=rp.parent/e['file']
        if digest(p)!=e['sha256']:raise ValueError('alpha identity mismatch')
        n=self.recipe['patch_pixels'];self.alpha=np.fromfile(p,np.uint8).reshape(-1,n,n)

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
            m=np.array(p['source_matrix']);o=p['source_offset_m'];e=r['sources'][p['material']];src=self.sources[p['material']];native=e['source_width_m']/e['side']
            u=(m[0,0]*px*r['guide_texel_m']+m[0,1]*py*r['guide_texel_m']+o[0])/native-.5
            v=(m[1,0]*px*r['guide_texel_m']+m[1,1]*py*r['guide_texel_m']+o[1])/native-.5
            value=np.stack([self.interp(src[c],u,v)/scale for c,scale in [('albedo',65535),('roughness',255),('nx',32767),('nq',32767)]],axis=-1)
            value[:,:,2:]=value[:,:,2:]@m
            idx=np.ix_(yi,xi);out[idx]=out[idx]*(1-a[:,:,None])+value*a[:,:,None];filled[idx]=filled[idx]*(1-a)+a
        if (filled<.999).any():raise ValueError('reference unfilled texels')
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
    args=p.parse_args();kw=vars(args);command=kw.pop('command')
    if command=='prepare':kw['config_path']=kw.pop('config');kw['spec_path']=kw.pop('spec')
    result=(fetch if command=='fetch' else prepare)(**kw)
    print(json.dumps({k:v for k,v in result.items() if k not in ['channels','inputs']}))


if __name__=='__main__':main()
