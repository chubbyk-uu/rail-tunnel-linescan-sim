"""Bounded Concrete034 GUI assets from the capture recipe and exact optical mesh.

Preview only: filtered albedo and approximate crack coverage; never optical truth.
No changes to the camera scene, geometry, exposure or source assets.
"""
import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import cv2
import numpy as np
from PIL import Image

from .stage_b_materials import linear_to_srgb
from .stage_b_runtime_surface import ReferenceRecipe
from .stage_b_scene import digest


class PreviewRecipe(ReferenceRecipe):
    def __init__(self, path):
        super().__init__(path)
        self.filtered = {}

    def albedo(self, xs, qs, pitch):
        # Area-filter native detail before downsampling; no full tunnel bake.
        if pitch not in self.filtered:
            maps = []
            for src, entry in zip(self.sources, self.recipe['sources']):
                native = entry['source_width_m'] / src.shape[1]
                w = max(1, round(src.shape[1]*native/(pitch/2)))
                h = max(1, round(src.shape[0]*native/(pitch/2)))
                maps.append(cv2.resize(src['albedo'], (w,h), interpolation=cv2.INTER_AREA))
            self.filtered[pitch] = maps
        r = self.recipe
        gx = (np.asarray(xs)-r['origin_xq_m'][0])/r['guide_texel_m']
        gy = (np.asarray(qs)-r['origin_xq_m'][1])/r['guide_texel_m']
        out = np.zeros((len(qs),len(xs)), np.float64)
        filled = np.zeros_like(out)
        side = r['patch_pixels']
        if not hasattr(self,'placement_origins'):
            self.placement_origins=np.array([[p['left'],p['top']] for p in r['placements']])
        origins=self.placement_origins
        candidates=np.flatnonzero((origins[:,0]<=gx.max()) & (origins[:,0]+side>gx.min()) &
                                 (origins[:,1]<=gy.max()) & (origins[:,1]+side>gy.min()))
        for ident in candidates:
            p=r['placements'][ident]
            xi = np.flatnonzero((gx>=p['left']) & (gx<p['left']+side))
            yi = np.flatnonzero((gy>=p['top']) & (gy<p['top']+side))
            if not len(xi) or not len(yi): continue
            px,py = np.meshgrid(gx[xi]-p['left'],gy[yi]-p['top'])
            alpha = self.interp(self.alpha[ident],px-.5,py-.5)/255
            m = np.array(p['source_matrix']); o = p['source_offset_m']
            entry = r['sources'][p['material']]; src = self.filtered[pitch][p['material']]
            original = self.sources[p['material']]
            u = (m[0,0]*px*r['guide_texel_m']+m[0,1]*py*r['guide_texel_m']+o[0])/entry['source_width_m']*src.shape[1]-.5
            height_m = entry['source_width_m']*original.shape[0]/original.shape[1]
            v = (m[1,0]*px*r['guide_texel_m']+m[1,1]*py*r['guide_texel_m']+o[1])/height_m*src.shape[0]-.5
            idx = np.ix_(yi,xi)
            out[idx] = out[idx]*(1-alpha)+self.interp(src,u,v)*alpha/65535
            filled[idx] = filled[idx]*(1-alpha)+alpha
        if np.any(filled<.999): raise ValueError('unfilled preview texels')
        if self.macro is not None: out *= self.macro_at(xs,qs)
        return out


def read_mesh(path):
    vertices, normals, uv, faces = [], [], [], []
    with Path(path).open() as stream:
        for line in stream:
            if line.startswith('v '): vertices.append(tuple(map(float,line.split()[1:])))
            elif line.startswith('vn '): normals.append(tuple(map(float,line.split()[1:])))
            elif line.startswith('vt '): uv.append(tuple(map(float,line.split()[1:])))
            elif line.startswith('f '):
                corners = [tuple(int(x)-1 for x in token.split('/')) for token in line.split()[1:]]
                if len(corners)!=3 or any(len(c)!=3 or c[0]!=c[1] or c[0]!=c[2] for c in corners):
                    raise ValueError('expected indexed triangular optical mesh with matching UV/normal indices')
                faces.append([c[0] for c in corners])
    return np.asarray(vertices), np.asarray(normals), np.asarray(uv), np.asarray(faces)


def mesh_plan(vertices, uv, faces, tunnel, budget):
    x0,x1 = tunnel['x_min_m'],tunnel['x_max_m']; period = 2*math.pi*tunnel['radius_m']
    xq = uv*np.array([x1-x0,period])+np.array([x0,-period/2])
    centres = xq[faces].mean(axis=1)
    keys = np.stack((np.floor(centres[:,0]/1.2),np.floor((centres[:,1]+period/2)/(period/6))),axis=1).astype(int)
    unique, inverse = np.unique(keys,axis=0,return_inverse=True)
    plan=[]; memory=0
    for i,key in enumerate(unique):
        ids=np.flatnonzero(inverse==i); used=np.unique(faces[ids])
        bounds=np.stack((xq[used].min(axis=0),xq[used].max(axis=0)))
        pitch=.001 if bounds[1,0]>=3 and bounds[0,0]<=6 else .002
        # Full vertex extent plus a two-texel border keeps bilinear/mip reads inside the tile.
        lo=np.floor(bounds[0]/pitch)*pitch-2*pitch
        hi=np.ceil(bounds[1]/pitch)*pitch+2*pitch
        size=np.rint((hi-lo)/pitch).astype(int)
        if max(size)>8192: raise ValueError('preview texture exceeds 8K')
        memory+=int(size.prod())*4*4/3+4096
        plan.append(dict(ids=ids,used=used,lo=lo,hi=hi,size=size,pitch=pitch))
    if memory>budget: raise ValueError(f'GUI RGBA8+mips estimate {int(memory)} exceeds {budget}')
    return xq,plan,int(memory)


def crack_preview(albedo, lo, pitch, segments, depths):
    """4x coverage raster, true metric width; cavity ratio approximated per short segment.

    It is a GUI level of detail, not the OptiX exposure/union photometry reference.
    """
    h,w=albedo.shape; scale=4
    mask=np.zeros((h*scale,w*scale),np.uint8)
    x1,q1=lo+np.array([w,h])*pitch
    pad=.001
    ids=np.flatnonzero((np.maximum(segments[:,0],segments[:,2])>=lo[0]-pad)&
        (np.minimum(segments[:,0],segments[:,2])<=x1+pad)&
        (np.maximum(segments[:,1],segments[:,3])>=lo[1]-pad)&
        (np.minimum(segments[:,1],segments[:,3])<=q1+pad))
    # Evaluate finite tapered capsules at subpixel centres. Raster line thickness
    # rounding would enlarge 0.2--0.6 mm cracks, especially at the near 1 mm tier.
    for i in ids:
        segment=segments[i].astype(float); a=segment[:2]; b=segment[2:4]
        radius=max(segment[4:]); step=pitch/scale
        lower=np.maximum(0,np.floor((np.minimum(a,b)-radius-lo)/step).astype(int))
        upper=np.minimum([w*scale,h*scale],np.ceil((np.maximum(a,b)+radius-lo)/step).astype(int))
        if np.any(upper<=lower): continue
        xs=lo[0]+(np.arange(lower[0],upper[0])+.5)*step
        qs=lo[1]+(np.arange(lower[1],upper[1])+.5)*step
        X,Q=np.meshgrid(xs,qs); delta=b-a
        t=np.clip(((X-a[0])*delta[0]+(Q-a[1])*delta[1])/max(delta@delta,1e-20),0,1)
        radii=segment[4]*(1-t)+segment[5]*t
        inside=(X-a[0]-t*delta[0])**2+(Q-a[1]-t*delta[1])**2<=radii**2
        depth=depths[i,0]*(1-t)+depths[i,1]*t
        opening=1/(1+depth/np.maximum(radii,1e-9))
        wall=albedo[np.clip(((Q-lo[1])/pitch).astype(int),0,h-1),np.clip(((X-lo[0])/pitch).astype(int),0,w-1)]
        ratio=opening/(1-wall*(1-opening))
        opacity=np.uint8(np.clip(np.rint(255*(1-ratio)*inside),0,255))
        region=mask[lower[1]:upper[1],lower[0]:upper[0]]
        np.maximum(region,opacity,out=region)
    return albedo*(1-cv2.resize(mask,(w,h),interpolation=cv2.INTER_AREA)/255)


def prepare(scene_path, world_path, output, budget=1280<<20):
    output=Path(output).resolve()
    if output.exists(): raise ValueError('refuse to overwrite GUI assets')
    scene_path=Path(scene_path).resolve(); scene=json.loads(scene_path.read_text())
    def asset(entry):
        p=(scene_path.parent/entry['file']).resolve()
        if digest(p)!=entry['sha256']: raise ValueError('optical asset identity mismatch')
        return p
    surface=asset(scene['surface']); defects_path=asset(scene['defects'])
    panels=asset(next(x for x in scene['meshes'] if x['material']==0))
    recipe=PreviewRecipe(surface); t=recipe.surface['tunnel']
    vertices,normals,uv,faces=read_mesh(panels)
    xq,plan,estimate=mesh_plan(vertices,uv,faces,t,budget)
    defects=json.loads(defects_path.read_text())
    arrays=[]
    for name,cols in [('segments.bin',6),('depths.bin',2)]:
        entry=defects['files'][name]; p=defects_path.parent/entry['file']
        if digest(p)!=entry['sha256']: raise ValueError('crack data identity mismatch')
        if name=='segments.bin':
            from .stage_b_defects import read_segments
            packed=read_segments(p,defects)
            arrays.append(np.column_stack([packed[field] for field in packed.dtype.names]))
        else:
            arrays.append(np.fromfile(p,'<f4').reshape(-1,cols))
    segments,depths=arrays
    root=ET.parse(world_path); lining=root.find(".//model[@name='tunnel']/link[@name='lining']")
    # Retain original joint/filler/gap meshes; replace only concrete previews.
    for visual in list(lining.findall('visual')):
        keep=visual.get('name') in ('joints','filler','gap')
        if keep:
            with Path(visual.findtext('geometry/mesh/uri')).open() as f:
                keep=any(line.startswith('f ') for line in f)
        if not keep: lining.remove(visual)
    output.mkdir(parents=True); (output/'preview').mkdir()
    records=[]; period=2*math.pi*t['radius_m']; ox,oq=recipe.surface['origin_xq_m']; dx,dq=recipe.surface['texel_xq_m']; nx,nq=recipe.surface['pixels_xq']
    for index,p in enumerate(plan):
        name=f'concrete_{index:03d}'; mesh=output/'preview'/(name+'.obj'); image=mesh.with_suffix('.png')
        ids,used,lo,hi,size,pitch=(p[k] for k in ('ids','used','lo','hi','size','pitch'))
        remap=np.full(len(vertices),-1,dtype=int);remap[used]=np.arange(1,len(used)+1)
        with mesh.open('w') as f:
            for v in vertices[used]: f.write('v %.9f %.9f %.9f\n'%tuple(v))
            for n in normals[used]: f.write('vn %.9f %.9f %.9f\n'%tuple(n))
            for v in (xq[used]-lo)/(hi-lo): f.write('vt %.9f %.9f\n'%tuple(v))
            for face in remap[faces[ids]]: f.write('f '+' '.join(f'{i}/{i}/{i}' for i in face)+'\n')
        w,h=map(int,size); albedo=np.empty((h,w),np.float32)
        for row in range(0,h,64):
            rows=np.arange(row,min(h,row+64)); value=np.zeros((len(rows),w))
            for sx,sy in ((.25,.25),(.75,.25),(.25,.75),(.75,.75)):
                xs=np.clip(lo[0]+(np.arange(w)+sx)*pitch,ox+dx/2,ox+(nx-.5)*dx)
                qs=oq+np.mod(lo[1]+(rows+sy)*pitch-oq,period)
                value+=recipe.albedo(xs,qs,pitch)*.25
            albedo[row:row+len(rows)]=value
        albedo=crack_preview(albedo,lo,pitch,segments,depths)
        pixels=np.uint8(np.clip(linear_to_srgb(albedo[::-1])*255+.5,0,255))
        Image.fromarray(pixels).convert("RGB").save(image)
        v=ET.SubElement(lining,'visual',name=name)
        ET.SubElement(ET.SubElement(ET.SubElement(v,'geometry'),'mesh'),'uri').text=str(mesh)
        material=ET.SubElement(v,'material')
        for tag in ('ambient','diffuse'):ET.SubElement(material,tag).text='1 1 1 1'
        metal=ET.SubElement(ET.SubElement(material,'pbr'),'metal')
        for tag,text in [('albedo_map',str(image)),('metalness','0'),('roughness','0.9')]:ET.SubElement(metal,tag).text=text
        records.append(dict(name=name,bounds_xq_m=[*lo,*hi],size=[w,h],pitch_m=pitch,triangles=len(ids),mesh_sha256=digest(mesh),texture_sha256=digest(image)))
    root.write(output/'world.sdf',encoding='unicode',xml_declaration=True)
    report=dict(schema='ssb.gui_preview.v2',surface_sha256=digest(surface),defects_sha256=digest(defects_path),
                optical_mesh_sha256=digest(panels),source_world_sha256=digest(world_path),
                world_sha256=digest(output/'world.sdf'),
                triangles=len(faces),output_triangles=sum(x['triangles'] for x in records),
                rgba8_mip_bytes_estimate=estimate,budget_bytes=budget,tiles=records,
                limitations='GUI filtered albedo and approximate 4x crack raster; camera assets unchanged. '
                '1 mm near x=3..6 m; 2 mm elsewhere. Mips handled by Ogre. Fixed bounded residency, not streaming. '
                'Sub-mm cracks fade at distance; no minimum-width enlargement or photometric accuracy claim.')
    (output/'manifest.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='tiles'}),flush=True)
    return report


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for key in ('scene','world','output'):p.add_argument('--'+key,required=True,type=Path)
    p.add_argument('--budget-mib',type=int,default=1280)
    a=p.parse_args();prepare(a.scene,a.world,a.output,a.budget_mib<<20)


if __name__=='__main__':main()
