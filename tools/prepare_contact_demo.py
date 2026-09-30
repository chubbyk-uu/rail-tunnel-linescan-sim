#!/usr/bin/env python3
"""Reuse accepted optical/GUI assets with contact physics and chassis work lights."""
import argparse
import json
from pathlib import Path
import shutil
import sys
import yaml

REPO=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(REPO/'src/ssb_tools'))
from ssb_tools.stage_b_gui_world import prepare
from ssb_tools.stage_b_scene import digest


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--world',type=Path,default=REPO/'local_data/stage_b/track_contact_v2/world.sdf')
    p.add_argument('--config',type=Path,default=REPO/'local_data/stage_b/optics_b3_quality64/capture.yaml')
    p.add_argument('--spec',type=Path,default=REPO/'src/ssb_tools/config/stage_b_scene.yaml')
    p.add_argument('--output',required=True,type=Path)
    a=p.parse_args();out=a.output.resolve()
    if out.exists():raise ValueError('refuse to overwrite prepared demo')
    out.mkdir(parents=True)
    c=yaml.safe_load(a.config.read_text());spec=yaml.safe_load(a.spec.read_text())
    c['contact']={'enabled':True,'settle_s':2.}
    c['motion']['start_x_m']=3.
    c['motion']['profile']=[[0.,0.],[1.,1.],[15.,1.],[16.,0.],[17.,0.]]
    c['acceptance']['valid_x_m']=[3.1,5.7]
    for section in ['truth','calibration']:
        c[section].setdefault('odo_left_diameter_m',c[section]['wheel_diameter_m'])
        c[section].setdefault('odo_right_diameter_m',c[section]['wheel_diameter_m'])
    source=Path(c['render']['optical_scene'])
    if not source.is_absolute():source=a.config.resolve().parent/source
    scene=json.loads(source.read_text())
    # Resolve all old scene-relative asset references before changing its directory.
    def resolve(node):
        if isinstance(node,dict):
            if 'file' in node:node['file']=str((source.parent/node['file']).resolve())
            for child in node.values():resolve(child)
        elif isinstance(node,list):
            for child in node:resolve(child)
    resolve(scene)
    shutil.copyfile(a.spec,out/'spec.yaml')
    scene['runtime_spec']={'file':str(out/'spec.yaml'),'sha256':digest(out/'spec.yaml')}
    scene['indirect_fill_relative']=spec['preview']['indirect_fill_relative']
    scene['limitations']+=' Work-light reflection is an uncalibrated weak diffuse sensitivity term, not traced global illumination.'
    (out/'scene.json').write_text(json.dumps(scene,indent=2)+'\n')
    c['render']['optical_scene']=str(out/'scene.json')
    (out/'capture.yaml').write_text(yaml.safe_dump(c,sort_keys=False))
    prepare(a.world,out/'capture.yaml',out/'spec.yaml',out/'world','robot')
    shutil.copyfile(REPO/'src/ssb_gazebo/worlds/stage_b_gui.config',out/'gui.config')

if __name__=='__main__':main()
