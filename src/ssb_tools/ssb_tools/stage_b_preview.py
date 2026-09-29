"""Bounded Gazebo previews from baked albedo, separate from camera rendering."""
from collections import OrderedDict
import json
import math
from pathlib import Path

import cv2
import numpy as np
from PIL import Image

from .stage_b_materials import linear_to_srgb
from .stage_b_scene import Mesh, cylinder_strip, digest
from .stage_b_surface import TEXEL


class AlbedoTiles:
    """Small CPU LRU; only the albedo channel is retained after hash checking."""
    def __init__(self, manifest, cache_bytes=32 << 20):
        self.path = Path(manifest).resolve()
        self.surface = json.loads(self.path.read_text())
        s = self.surface
        self.core, self.gutter = s['core_pixels'], s['gutter_pixels']
        self.side = self.core + 2*self.gutter
        self.limit = cache_bytes // (self.side*self.side*2)
        if self.limit < 1:
            raise ValueError('preview tile cache too small')
        self.entries = {(t['ix'],t['iq']):t for t in s['tiles']}
        self.cache = OrderedDict()
        self.verified = set()

    def tile(self, ix, iq):
        key = (ix,iq)
        if key not in self.cache:
            entry = self.entries[key]
            path = self.path.parent/entry['file']
            if key not in self.verified:
                if digest(path) != entry['sha256']:
                    raise ValueError('preview surface tile identity mismatch')
                self.verified.add(key)
            packed = np.fromfile(path,dtype=TEXEL)
            if packed.size != self.side*self.side:
                raise ValueError('preview tile dimensions mismatch')
            albedo = packed.reshape(self.side,self.side)['albedo'].copy()
            while len(self.cache) >= self.limit:
                self.cache.popitem(last=False)
            self.cache[key] = albedo
        self.cache.move_to_end(key)
        return self.cache[key]

    def crop(self, x0, x1, q0, q1, max_bytes):
        s = self.surface
        dx,dq = s['texel_xq_m']
        ox,oq = s['origin_xq_m']
        nx,nq = s['pixels_xq']
        left,right = max(0,math.floor((x0-ox)/dx)),min(nx,math.ceil((x1-ox)/dx))
        bottom,top = math.floor((q0-oq)/dq),math.ceil((q1-oq)/dq)
        if not (right>left and top>bottom) or (right-left)*(top-bottom)*2 > max_bytes:
            raise ValueError('preview crop exceeds working-set budget')
        image = np.empty((top-bottom,right-left),np.uint16)
        row = bottom
        while row < top:
            periodic = row % nq
            iq, local_q = divmod(periodic,self.core)
            count_q = min(top-row,self.core-local_q,nq-periodic)
            col = left
            while col < right:
                ix,local_x = divmod(col,self.core)
                count_x = min(right-col,self.core-local_x)
                tile = self.tile(ix,iq)
                g = self.gutter
                image[row-bottom:row-bottom+count_q,col-left:col-left+count_x] = \
                    tile[g+local_q:g+local_q+count_q,g+local_x:g+local_x+count_x]
                col += count_x
            row += count_q
        return image


def prepare_preview(surface, out, geometry, config, spec):
    t = config['tunnel']
    reader = AlbedoTiles(surface)
    if reader.surface['tunnel'] != t:
        raise ValueError('preview surface and geometry domain mismatch')
    texel = spec['materials'].get('gui_preview_texel_m',.002)
    if not math.isfinite(texel) or texel < min(reader.surface['texel_xq_m']):
        raise ValueError('preview resolution must not invent native detail')
    r,zc = t['radius_m'],t['axis_z_m']
    plan = []
    for i,p in enumerate(geometry['panels']):
        x0,x1 = p['x_m']; a,b = p['angle_rad']
        width,height = math.ceil((x1-x0)/texel),math.ceil(r*(b-a)/texel)
        plan.append(dict(name=f'panel_{i:03d}',mesh=f'preview/panel_{i:03d}.obj',
                         texture=f'preview/panel_{i:03d}.png',size=[width,height]))
    # Conservative RGBA8 estimate including a complete mip chain and alignment.
    gpu_bytes = sum(math.ceil(p['size'][0]*p['size'][1]*4*4/3)+4096 for p in plan)
    budget = spec['resources'].get('gpu_preview_texture_budget_bytes',768 << 20)
    if gpu_bytes > budget:
        raise ValueError('Gazebo preview texture budget exceeded before allocation')
    directory = out/'preview'; directory.mkdir()
    counters = dict(vertices=0,triangles=0)
    for p,item in zip(geometry['panels'],plan):
        x0,x1 = p['x_m']; a,b = p['angle_rad']
        native = reader.crop(x0,x1,r*a,r*b,spec['resources']['asset_working_set_bytes']//2)
        reduced = cv2.resize(native,tuple(item['size']),interpolation=cv2.INTER_AREA)
        # OBJ v grows with q; PNG row zero is the largest q in this panel.
        srgb = np.uint8(np.clip(linear_to_srgb(reduced[::-1].astype(np.float32)/65535)*255+.5,0,255))
        Image.fromarray(np.repeat(srgb[:,:,None],3,axis=2)).save(out/item['texture'])
        mesh = Mesh(out/item['mesh'],counters,spec['resources'],(x0,x1,zc,a,b))
        try:
            cylinder_strip(mesh,x0,x1,a,b,r,zc,math.radians(spec['panels']['angular_step_deg']))
        finally:
            mesh.close()
    return plan,dict(texel_m=texel,panels=len(plan),rgba8_mip_bytes_estimate=gpu_bytes,
                     gpu_budget_bytes=budget,cpu_tile_cache_bytes=32 << 20,
                     triangles=counters['triangles'],source='hashed baked albedo tiles',
                     limitations='GUI preview only; crop boundaries rounded to native texels. '
                                 'Vector cracks remain in the separate camera rendering layer.')
