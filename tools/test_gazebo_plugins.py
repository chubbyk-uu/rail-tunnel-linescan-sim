#!/usr/bin/env python3
"""Short real-server regression for both plugins and assembly mismatch rejection."""
import argparse
import copy
import json
import os
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET
import yaml

REPO=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(REPO/'src/ssb_tools'))
from ssb_tools.stage_b_scene import make_world,load_spec
from ssb_tools.session import Session


def run(output):
    output=Path(output).resolve();output.mkdir(parents=True,exist_ok=False)
    base=yaml.safe_load((REPO/'src/ssb_core/config/stage_b.yaml').read_text())
    spec=load_spec(REPO/'src/ssb_tools/config/stage_b_scene.yaml')
    results=[]
    for name,contact,bad in [('ideal',False,False),('contact',True,False),('mismatch',True,True)]:
        folder=output/name;folder.mkdir()
        c=copy.deepcopy(base);c['robot']={'base_reference_z_m':.37,'scan_axis_height_m':1.645}
        c['motion'].update(start_x_m=3.,start_theta_deg=-30.,profile=[[0.,0.],[.1,1.],[.5,1.],[.6,0.],[.7,0.]])
        c['camera']['width']=64;c['acceptance']['valid_x_m']=[3.,3.1]
        c['render']['debug_column_stride']=16
        c['contact']={'enabled':contact,'settle_s':.5}
        cfg=folder/'capture.yaml';cfg.write_text(yaml.safe_dump(c))
        for mesh in ('panels','joints','filler','gap'):
            (folder/(mesh+'.obj')).write_text('v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n')
        make_world(folder,c,spec)
        world=folder/'world.sdf'
        if bad:
            tree=ET.parse(world);pose=tree.find("world/model[@name='scan_car']/link[@name='head']/pose")
            p=list(map(float,pose.text.split()));p[2]+=.01;pose.text=' '.join(map(str,p));tree.write(world)
        env=dict(os.environ,SSB_WORLD=str(world),GZ_PARTITION=f'ssb_plugin_test_{os.getpid()}_{name}')
        session=folder/'session'
        with (folder/'run.log').open('w') as log:
            proc=subprocess.run(['bash',str(REPO/'tools/run_gz.sh'),str(session),str(cfg)],
                                env=env,stdout=log,stderr=subprocess.STDOUT,timeout=60)
        if bad:
            assert proc.returncode!=0 and not session.exists()
            assert 'assembly height differs' in (folder/'run.log').read_text()
            results.append({'name':name,'rejected_before_capture':True});continue
        assert proc.returncode==0,(folder/'run.log')
        s=Session(session);assert s.summary['motion']['complete'] and s.summary['rows']>0
        poses=s.evaluation('pose_stream')
        if contact:
            assert abs(poses['z']-.37).max()<.003
            assert poses['body_valid'].all()
        results.append({'name':name,'rows':s.summary['rows'],'complete':True})
    (output/'report.json').write_text(json.dumps(results,indent=2)+'\n')
    print(json.dumps(results))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',required=True)
    run(p.parse_args().output)
