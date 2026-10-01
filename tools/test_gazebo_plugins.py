#!/usr/bin/env python3
"""Real-server regression for the default Stage A command and both Stage B plugins."""
import argparse
import copy
import json
import os
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET
import numpy as np
import yaml

REPO=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(REPO/'src/ssb_tools'))
from ssb_tools.stage_b_scene import make_world,load_spec
from ssb_tools.session import Session, sha256_file


def run_stage_a(output):
    folder=output/'stage_a';folder.mkdir()
    env=dict(os.environ,GZ_PARTITION=f'ssb_plugin_test_{os.getpid()}_stage_a')
    env.pop('SSB_WORLD',None)
    session=folder/'session'
    # No config/world override: exercise the exact README command and installed world.
    with (folder/'run.log').open('w') as log:
        proc=subprocess.run(['bash',str(REPO/'tools/run_gz.sh'),str(session)],
                            env=env,stdout=log,stderr=subprocess.STDOUT,timeout=60)
    assert proc.returncode==0,folder/'run.log'
    s=Session(session)
    assert s.summary['motion']['complete'] and s.summary['rows']>0
    provenance=json.loads((session/'config/provenance.json').read_text())
    actual_world=Path(provenance['inputs']['world']['path'])
    assert sha256_file(actual_world)==sha256_file(REPO/'src/ssb_gazebo/worlds/stage_a.sdf')

    # The original mismatch must still be rejected, rather than exempting Stage A.
    wrong=yaml.safe_load((REPO/'src/ssb_core/config/stage_a.yaml').read_text())
    wrong['robot']['base_reference_z_m']=.3
    cfg=folder/'mismatch.yaml';cfg.write_text(yaml.safe_dump(wrong))
    rejected=folder/'rejected_session'
    with (folder/'mismatch.log').open('w') as log:
        bad=subprocess.run(['bash',str(REPO/'tools/run_gz.sh'),str(rejected),str(cfg)],
                           env=env,stdout=log,stderr=subprocess.STDOUT,timeout=60)
    assert bad.returncode!=0 and not rejected.exists()
    assert 'assembly height differs' in (folder/'mismatch.log').read_text()
    return {'name':'stage_a_default','rows':s.summary['rows'],'complete':True,
            'installed_world_matches_source':True,'mismatch_rejected_before_capture':True}


def run(output):
    output=Path(output).resolve();output.mkdir(parents=True,exist_ok=False)
    base=yaml.safe_load((REPO/'src/ssb_core/config/stage_b.yaml').read_text())
    spec=load_spec(REPO/'src/ssb_tools/config/stage_b_scene.yaml')
    results=[run_stage_a(output)]
    from ssb_tools.wheel_stiffness import calibrate
    from ssb_tools.stage_b_robot import WHEEL_MASS_KG, AXLE_MASS_KG, running_wheel_load_mass
    stiffness=calibrate(.0002,.2,running_wheel_load_mass(spec),AXLE_MASS_KG,WHEEL_MASS_KG,.1,base['motion']['sample_period_s'])
    rough=dict(track_irregularity=dict(model='beijing_subway_vertical_v1',chord10_max_m=.002,seed=20261001,
                                       cross_level_tier_m=.002,band_m=[.5,10.],common_mode=False),
               wheel_compliance=dict(static_deflection_m=.0002,damping_ratio=.2,stiffness_n_m=stiffness['stiffness_n_m'],
                                     damping_n_s_m=stiffness['damping_n_s_m']))
    cases=[('ideal',False,None,{}),('contact',True,None,{}),('mismatch',True,'assembly height differs',{}),
           ('irregular',True,None,rough),('irregular_mismatch',True,'physical world check failed',rough),
           ('start_mismatch',True,'start position differs',{}),
           # Configuration edited after the world was generated: true wheel diameter, track seed.
           ('odo_mismatch',True,'physical world check failed',rough),('seed_mismatch',True,'physical world check failed',rough)]
    for name,contact,bad,truth in cases:
        folder=output/name;folder.mkdir()
        c=copy.deepcopy(base);c['robot']={'base_reference_z_m':.37,'scan_axis_height_m':1.645}
        c['motion'].update(start_x_m=3.,start_theta_deg=-30.,profile=[[0.,0.],[.1,1.],[.5,1.],[.6,0.],[.7,0.]])
        c['camera']['width']=64;c['acceptance']['valid_x_m']=[3.,3.1]
        c['render']['debug_column_stride']=16
        c['contact']={'enabled':contact,'settle_s':.5}
        if contact:
            for section in ('truth','calibration'):
                c[section].update(odo_left_diameter_m=.08,odo_right_diameter_m=.08)
        c['truth'].update(copy.deepcopy(truth))
        for mesh in ('panels','joints','filler','gap'):
            (folder/(mesh+'.obj')).write_text('v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n')
        if name=='irregular_mismatch':
            flat=copy.deepcopy(c);flat['truth'].pop('track_irregularity');make_world(folder,flat,spec)
        else:
            make_world(folder,c,spec)
        if name=='odo_mismatch':c['truth']['odo_left_diameter_m']=.081
        if name=='seed_mismatch':c['truth']['track_irregularity']['seed']=7
        cfg=folder/'capture.yaml';cfg.write_text(yaml.safe_dump(c))
        (folder/'spec.yaml').write_text(yaml.safe_dump(spec))
        world=folder/'world.sdf'
        if name=='mismatch':
            tree=ET.parse(world);pose=tree.find("world/model[@name='scan_car']/link[@name='head']/pose")
            p=list(map(float,pose.text.split()));p[2]+=.01;pose.text=' '.join(map(str,p));tree.write(world)
        if name=='start_mismatch':
            tree=ET.parse(world);pose=tree.find("world/model[@name='scan_car']/pose")
            p=list(map(float,pose.text.split()));p[0]+=.5;pose.text=' '.join(map(str,p));tree.write(world)
        env=dict(os.environ,SSB_WORLD=str(world),GZ_PARTITION=f'ssb_plugin_test_{os.getpid()}_{name}')
        session=folder/'session'
        with (folder/'run.log').open('w') as log:
            proc=subprocess.run(['bash',str(REPO/'tools/run_gz.sh'),str(session),str(cfg)],
                                env=env,stdout=log,stderr=subprocess.STDOUT,timeout=60)
        if bad:
            assert proc.returncode!=0 and not session.exists()
            assert bad in (folder/'run.log').read_text()
            results.append({'name':name,'rejected_before_capture':True});continue
        assert proc.returncode==0,(folder/'run.log')
        s=Session(session);assert s.summary['motion']['complete'] and s.summary['rows']>0
        poses=s.evaluation('pose_stream')
        if contact:
            assert abs(poses['z']-.37).max()<.003
            assert poses['body_valid'].all()
        if truth:
            rails=[m for m in ET.parse(world).getroot().iter('model') if m.get('name','').startswith('rail_surface_')]
            assert rails and np.ptp(poses['z'])>2e-4, 'irregular track must move the body'
        results.append({'name':name,'rows':s.summary['rows'],'complete':True})
    (output/'report.json').write_text(json.dumps(results,indent=2)+'\n')
    print(json.dumps(results))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',required=True)
    run(p.parse_args().output)
