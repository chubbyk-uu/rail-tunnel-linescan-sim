#!/usr/bin/env python3
"""Reuse accepted optical/GUI assets with contact physics and chassis work lights."""
import argparse
import json
import math
from pathlib import Path
import shutil
import sys
import subprocess
import os
import xml.etree.ElementTree as ET
import yaml

REPO=Path(__file__).resolve().parents[1]
from ssb_tools.stage_b_gui_world import prepare
from ssb_tools.stage_b_scene import digest
from ssb_tools.stage_b_track import replace_track
from ssb_tools.optical_identity import ensure_optical_key, check_calibration
from ssb_tools.robot_geometry import mount_geometry
from ssb_tools.stage_b_robot import WHEEL_MASS_KG, AXLE_MASS_KG, running_wheel_load_mass


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--demo',type=Path,default=REPO/'local_data/stage_b/contact_demo',
                   help='complete prepared input bundle; individual inputs may be overridden')
    p.add_argument('--world',type=Path)
    p.add_argument('--config',type=Path)
    p.add_argument('--spec',type=Path)
    p.add_argument('--output',required=True,type=Path)
    p.add_argument('--calibrate',action='store_true',help='render independent targets and fit this demo rig')
    p.add_argument('--calibration',type=Path,help='reuse a compatible measured calibration')
    p.add_argument('--gate-margin-deg',type=float,default=5.,
                   help='extra capture on each side of the fixed upper 240 degree output (0..10; default 5)')
    p.add_argument('--response-gain',type=float,
                   help='override the camera radiometric gain; changed gain requires a compatible new calibration')
    p.add_argument('--geometry',type=Path,
                   help='lining meshes (prepare_stage_b_scene output, same layout) replacing those the source scene and '
                        'world name; optical settings, key and calibration are kept (meshes are not in the optical '
                        'signature). Default: retain the input bundle meshes')
    p.add_argument('--track-chord-mm',type=float,choices=(0.,2.,5.),default=0.,
                   help='vertical rail irregularity tier: max 10 m mid-chord offset (0 = flat rails)')
    p.add_argument('--track-cross-level-mm',type=float,choices=(0.,2.,4.),default=0.,
                   help='cross-level tier: bounds left-right level difference and its 5 m twist (0 = none)')
    p.add_argument('--track-seed',type=int,default=20261001,help='rail irregularity random seed')
    p.add_argument('--odo-truth-mm',type=float,nargs=2,metavar=('LEFT','RIGHT'),
                   help='true measuring-wheel diameters (simulation truth; the world is built with them). '
                        'Default: the nominal spec diameter')
    p.add_argument('--odo-calibration-mm',type=float,nargs=2,metavar=('LEFT','RIGHT'),
                   help='calibrated measuring-wheel diameters the controller and reconstruction use. '
                        'Default: the nominal spec diameter')
    p.add_argument('--mount-offset-mm',type=float,nargs=2,metavar=('LATERAL','VERTICAL'),
                   help='simulation-only fixed scanner offset; rebuilds the head/cradle, requires new image calibration')
    p.add_argument('--mount-tilt-mrad',type=float,nargs=2,metavar=('Y','Z'),
                   help='simulation-only shaft tilt Ry then Rz; rebuilds assembly, requires new image calibration')
    p.add_argument('--wheel-deflection-mm',type=float,default=.2,
                   help='realized polyurethane tread static deflection (assumption, >=0.15; 0 = rigid wheels)')
    a=p.parse_args();out=a.output.resolve()
    if not math.isfinite(a.gate_margin_deg) or not 0<=a.gate_margin_deg<=10:
        p.error('gate margin must be finite and within 0..10 degrees')
    if a.response_gain is not None and (not math.isfinite(a.response_gain) or a.response_gain<=0):
        p.error('response gain must be finite and positive')
    a.demo=a.demo.resolve()
    a.world=a.world or a.demo/'world/world.sdf'
    a.config=a.config or a.demo/'capture.yaml'
    a.spec=a.spec or a.demo/'spec.yaml'
    if not a.calibrate and not a.calibration and (a.demo/'calibration.json').is_file():
        a.calibration=a.demo/'calibration.json'
    if a.calibrate and a.calibration:
        p.error('choose --calibrate or --calibration')
    if out.exists():raise ValueError('refuse to overwrite prepared demo')
    out.mkdir(parents=True)
    c=yaml.safe_load(a.config.read_text());spec=yaml.safe_load(a.spec.read_text())
    # Fresh contact worlds allow roll/heave before the lateral limit engages.
    # Explicit archived values remain authoritative; absent legacy values are
    # upgraded only here while regenerating the world, never during replay.
    spec['robot'].setdefault('guide_bearing_clearance_m',.002)
    for values, keys, limit in ((a.mount_offset_mm, ('dy_m','dz_m'), 20.),
                                (a.mount_tilt_mrad, ('tilt_y_rad','tilt_z_rad'), 2.)):
        if values is not None:
            if not all(math.isfinite(v) and abs(v)<=limit for v in values):
                p.error(f'fixed mount sensitivity exceeds supported +/-{limit} mm or mrad')
            c['truth']['mount'].update({k:v/1000 for k,v in zip(keys,values)})
    ensure_optical_key(c)
    base,height=mount_geometry(c)
    c['robot']={'base_reference_z_m':base,'scan_axis_height_m':height}
    c['contact']={'enabled':True,'settle_s':2.}
    c['motion']['start_x_m']=3.
    c['motion']['start_theta_deg']=180.  # bottom -> right lower gate -> top -> left
    c['gate']={'start_deg':-120.-a.gate_margin_deg,'end_deg':120.+a.gate_margin_deg}
    c['motion']['profile']=[[0.,0.],[1.,1.],[15.,1.],[16.,0.],[17.,0.]]
    c['acceptance']['valid_x_m']=[3.1,5.7]
    # Physical truth options; the world is generated from them and the plugin checks the match.
    c['truth'].pop('track_irregularity',None);c['truth'].pop('wheel_compliance',None)
    if a.track_chord_mm>0 or a.track_cross_level_mm>0:
        c['truth']['track_irregularity']=dict(model='beijing_subway_vertical_v1',chord10_max_m=a.track_chord_mm/1000,
                                              cross_level_tier_m=a.track_cross_level_mm/1000,seed=a.track_seed,
                                              band_m=[.5,10.],common_mode=a.track_cross_level_mm==0)
    if a.wheel_deflection_mm>0:
        # Solve the SDF spring for the realized static deflection in DART at the capture step.
        from ssb_tools.wheel_stiffness import calibrate
        result=calibrate(a.wheel_deflection_mm/1000,.2,running_wheel_load_mass(spec),AXLE_MASS_KG,
                         WHEEL_MASS_KG,c['truth']['wheel_diameter_m']/2,c['motion']['sample_period_s'])
        c['truth']['wheel_compliance']=dict(static_deflection_m=a.wheel_deflection_mm/1000,damping_ratio=.2,
            stiffness_n_m=result['stiffness_n_m'],damping_n_s_m=result['damping_n_s_m'],
            calibration=dict(realized_static_deflection_m=result['realized_static_deflection_m'],
                             nominal_stiffness_n_m=result['nominal_stiffness_n_m'],method=result['method']))
    # Odometry runs on the spring-loaded measuring wheels. A diameter error is set here, never by
    # editing the generated configuration: the world's wheel collision radii follow the truth.
    nominal=spec['robot']['measuring_wheel']['diameter_m']
    for section,values in (('truth',a.odo_truth_mm),('calibration',a.odo_calibration_mm)):
        left,right=(v/1000 for v in values) if values else (nominal,nominal)
        c[section]['odo_left_diameter_m'],c[section]['odo_right_diameter_m']=left,right
    source=Path(c['render']['optical_scene'])
    if not source.is_absolute():source=a.config.resolve().parent/source
    scene=json.loads(source.read_text())
    if a.response_gain is not None:
        scene['response_gain']=a.response_gain
    # Resolve all old scene-relative asset references before changing its directory.
    def resolve(node):
        if isinstance(node,dict):
            if 'file' in node:node['file']=str((source.parent/node['file']).resolve())
            for child in node.values():resolve(child)
        elif isinstance(node,list):
            for child in node:resolve(child)
    resolve(scene)
    replaced={}
    geometry=a.geometry.resolve() if a.geometry else None
    if geometry and not geometry.is_dir() and a.geometry!=p.get_default('geometry'):
        p.error(f'--geometry {a.geometry} does not exist')
    meshes=scene.get('meshes',[])
    if geometry and geometry.is_dir() and meshes:
        # The whole lining is replaced, or nothing: a complete, audited, unmodified set built for
        # this tunnel and panel layout.
        from ssb_tools.stage_b_scene import LINING_MESHES, lining_layout
        def lining_name(mesh):
            name=Path(mesh['file']).name
            prefix,_,suffix=name.partition('_')
            return suffix if prefix.isdigit() else name
        names=sorted(lining_name(m) for m in meshes)
        if names!=sorted(LINING_MESHES):raise ValueError(f'source scene lining {names} is not the four lining meshes')
        missing=[n for n in (*LINING_MESHES,'manifest.json','mesh_audit.json') if not (geometry/n).is_file()]
        if missing:raise ValueError(f'replacement geometry {geometry} is incomplete; missing {missing}')
        built=json.loads((geometry/'manifest.json').read_text());audit=json.loads((geometry/'mesh_audit.json').read_text())
        for name in LINING_MESHES:
            actual=digest(geometry/name)
            if built['assets'].get(name)!=actual or audit.get('meshes',{}).get(name)!=actual:
                raise ValueError(f'{geometry/name} differs from its geometry manifest or leak audit')
        if audit['leaks']['edges']:raise ValueError('replacement geometry has light leaks')
        if built.get('layout')!=lining_layout(c,spec):
            raise ValueError('replacement geometry was built for a different tunnel, panel layout or seed')
        for mesh in meshes:
            new=geometry/lining_name(mesh)
            replaced[mesh['file']]=str(new);mesh['file']=str(new);mesh['sha256']=digest(new)
        scene['geometry_replacement']=dict(folder=str(geometry),manifest_sha256=digest(geometry/'manifest.json'),
            light_leak_edges=0,reason='watertight lining (no T-junction gaps); same layout, joints and seed')
    (out/'spec.yaml').write_text(yaml.safe_dump(spec,sort_keys=False))
    scene['runtime_spec']={'file':str(out/'spec.yaml'),'sha256':digest(out/'spec.yaml')}
    scene['indirect_fill_relative']=spec['preview']['indirect_fill_relative']
    scene['work_light_preview']={k:v for k,v in spec['preview'].items() if k.startswith('work_light')}
    scene['work_light_preview']['direct_transport_in_capture']=False
    note=' GUI work lights illuminate the side wall ahead and behind the current camera stripe; their direct illumination is not yet rendered in acquisition. Reflected fill is an uncalibrated weak diffuse sensitivity term.'
    if note.strip() not in scene['limitations']:scene['limitations']+=note
    (out/'scene.json').write_text(json.dumps(scene,indent=2)+'\n')
    c['render']['optical_scene']=str(out/'scene.json')
    (out/'capture.yaml').write_text(yaml.safe_dump(c,sort_keys=False))
    prepare(a.world,out/'capture.yaml',out/'spec.yaml',out/'world','robot')
    world=out/'world/world.sdf';tree=ET.parse(world);w=tree.getroot().find('world')
    # GUI visuals of the lining follow the same replacement, whatever folder the source world named.
    lining={Path(new).name:new for new in replaced.values()}
    for uri in w.iter('uri'):
        if uri.text:
            name=Path(uri.text.strip()).name
            prefix,_,suffix=name.partition('_')
            if prefix.isdigit():name=suffix
            if name in lining:uri.text=lining[name]
    replace_track(w,out/'world',c,spec)
    missing=sorted({u.text.strip() for u in w.iter('uri') if u.text and '://' not in u.text and not Path(u.text.strip()).is_file()})
    if missing:raise ValueError(f'world references missing files: {missing[:5]}')
    ET.indent(tree);tree.write(world,encoding='unicode',xml_declaration=True)
    manifest=out/'world/manifest.json';report=json.loads(manifest.read_text())
    report['track_regenerated']=True;report['world_sha256']=digest(world)
    manifest.write_text(json.dumps(report,indent=2)+'\n')
    # Physical assets (rails, wheels, springs) are recorded and the world checked against the truth.
    from ssb_tools.physical_world import write_manifest, check
    write_manifest(world,c,spec)
    physical=check(c,spec,world)
    if not physical['passed']:
        raise ValueError('generated world differs from its configuration: '+
                         ', '.join(n for n,v in physical['checks'].items() if not v['passed']))
    shutil.copyfile(REPO/'src/ssb_gazebo/worlds/stage_b_gui.config',out/'gui.config')
    if a.calibrate:
        env = dict(os.environ)
        for command in ([sys.executable,'-m','ssb_tools.optical_bench','--config',str(out/'capture.yaml'),
                         '--output',str(out/'bench'),'--render'],
                        [sys.executable,'-m','ssb_tools.optical_calibration','fit','--bench',str(out/'bench/bench.json'),
                         '--output',str(out/'calibration.json')]):
            with (out/'calibration.log').open('a') as log:
                subprocess.run(command,env=env,stdout=log,stderr=subprocess.STDOUT,check=True)
    elif a.calibration:
        check_calibration(out/'capture.yaml',a.calibration)
        shutil.copyfile(a.calibration,out/'calibration.json')
    if (out/'calibration.json').exists():
        check_calibration(out/'capture.yaml',out/'calibration.json')

if __name__=='__main__':main()
