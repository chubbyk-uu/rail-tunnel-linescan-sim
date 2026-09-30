"""Bind hashed scene assets to an explicit Stage B imaging configuration."""
import argparse
import json
from pathlib import Path
import yaml

from .stage_b_scene import digest,load_spec


def crack_optics(defects):
    """Crack appearance. With per-vertex effective depths (stage_b_defects --depth): V-profiled
    slot cavity reflectance. Otherwise the flat B2 opening: interior reflectance as a fraction of
    the wall albedo with debris-like variation. Both keep a slightly darker edge band (worn/dirty
    lips). Appearance assumptions, not measurements."""
    if 'depths.bin' in json.loads(Path(defects).read_text()).get('files',{}):
        return dict(model='cavity_v1',edge_band_m=.00025,edge_darkening=.15)
    return dict(interior_ratio=.2,interior_variation=.35,edge_band_m=.00025,edge_darkening=.15)


def prepare(config_path,geometry,surface,defects,output,area_samples=16,time_samples=3,spec_path=None,adaptive=False,integrated=False,
            texture_footprint_samples=2,texture_prefilter=False,filler=None,area_pattern='rooks'):
    output=Path(output).resolve()
    if output.exists(): raise ValueError('optical configuration output already exists')
    geometry,surface,defects=(Path(p).resolve() for p in (geometry,surface,defects))
    if any((p/'FAILED').exists() for p in (geometry,surface.parent,defects.parent)):
        raise ValueError('refuse failed preparation output')
    if not 1<=area_samples<=16 or not 1<=time_samples<=16 or not 1<=texture_footprint_samples<=8: raise ValueError('sample limits')
    if area_pattern not in ('grid','rooks'): raise ValueError('area pattern')
    if integrated and time_samples!=3: raise ValueError('integrated cracks require three exposure frames')
    def entry(path,**extra): return dict(file=str(path),sha256=digest(path),**extra)
    scene=dict(schema='ssb.optical_scene.v1',surface=entry(surface),defects=entry(defects),
               meshes=[entry(geometry/name,material=i) for i,name in enumerate(('panels.obj','joints.obj','filler.obj','gap.obj'))
                       if (geometry/name).exists()],
               # Groove walls/floor (material 1): dusty, shadowed concrete; detail from the filler
               # map at an offset so it is not continuous with the lining texture (assumption).
               joint_concrete=dict(albedo=.12,detail_contrast=.3),
               crack_optics=crack_optics(defects),
               # Full-ray pixels (critical geometry): 'grid' traces area_samples^2 rays per exposure
               # sample; 'rooks' traces area_samples (N-rooks lattice, offset per exposure sample).
               sampling=dict(area_axis_samples=area_samples,area_pattern=area_pattern,time_samples=time_samples,
                             adaptive_area=adaptive,integrated_cracks=integrated,
                             # Accepted B1 setting: 2x2 texture taps over the pixel footprint on the
                             # integrated background path (centre-point sampling showed beat stripes).
                             texture_footprint_samples=texture_footprint_samples,texture_prefilter=texture_prefilter),
               convex_panel_visibility=True,
               lamp=dict(enabled=True,shadows=True,samples=4,length_m=.02,width_m=.02,footprint_m=[1.2,.12],
                         offset_axial_m=-.115,offset_tangential_m=0.,offset_radial_m=0.),response_gain=3.2,
               limitations='Relative engineered-lens beam profile with an equivalent COB-area source; no refractive lens transport; absolute lux, camera gain, lens MTF/noise and crack relief remain uncalibrated.')
    if (geometry/'filler.obj').exists():
        if filler is None: raise ValueError('geometry has filled joints: --filler (prepare-filler output) is required')
        scene['filler']=entry(Path(filler).resolve())
    if spec_path:
        spec=load_spec(spec_path)
        robot=spec['robot']
        scene['lamp'].update(length_m=robot['lamp_length_m'],width_m=robot['lamp_width_m'],
                             offset_axial_m=robot['lamp_offset_axial_m'],footprint_m=robot['lamp_wall_footprint_m'],
                             offset_tangential_m=robot['lamp_offset_tangential_m'],
                             offset_radial_m=robot['lamp_offset_radial_m'])
        scene['texture_budgets']=dict(gpu_bytes=spec['resources']['gpu_texture_budget_bytes'],
                                     cpu_bytes=spec['resources']['cpu_texture_cache_bytes'])
        scene['runtime_spec']=entry(Path(spec_path).resolve())
    config=yaml.safe_load(Path(config_path).read_text())
    output.mkdir(parents=True)
    (output/'scene.json').write_text(json.dumps(scene,indent=2)+'\n')
    config['render']['optical_scene']=str(output/'scene.json')
    (output/'capture.yaml').write_text(yaml.safe_dump(config,sort_keys=False))
    print(json.dumps(dict(output=str(output),scene_sha256=digest(output/'scene.json'))))
    return scene


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for arg in ('config','geometry','surface','defects','output'): p.add_argument('--'+arg,required=True,type=Path)
    p.add_argument('--area-samples',type=int,default=16);p.add_argument('--time-samples',type=int,default=3)
    p.add_argument('--spec',type=Path)
    p.add_argument('--adaptive',action='store_true',help='enable guarded background area reduction; verify convergence before use')
    p.add_argument('--integrated',action='store_true',help='integrate metric crack coverage analytically; keep full ray sampling at critical geometry')
    p.add_argument('--texture-footprint-samples',type=int,default=2,help='texture taps per axis over the pixel footprint (integrated path); 2 is the accepted B1 setting')
    p.add_argument('--texture-prefilter',action='store_true',help='average taps before shading (faster; less exact with strong normal maps)')
    p.add_argument('--area-pattern',choices=('grid','rooks'),default='rooks',help="full-ray pattern: rooks (N per exposure sample; default, N=16) or grid (N x N per exposure sample, the pre-2026-09-30 setting with N=8)")
    p.add_argument('--filler',type=Path,help='joint filler texture (stage_b_runtime_surface prepare-filler filler.json)')
    a=p.parse_args();prepare(a.config,a.geometry,a.surface,a.defects,a.output,a.area_samples,a.time_samples,a.spec,a.adaptive,a.integrated,
                             a.texture_footprint_samples,a.texture_prefilter,a.filler,a.area_pattern)


if __name__=='__main__': main()
