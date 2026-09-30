"""Derive a GUI world without modifying archived worlds or optical assets."""
import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml
from .stage_b_scene import digest,load_spec


def prepare(world,config,spec,output,mode):
    output=Path(output).resolve()
    if output.exists(): raise ValueError('refuse to overwrite GUI world')
    config=Path(config).resolve();spec=Path(spec).resolve();world=Path(world).resolve()
    c=yaml.safe_load(config.read_text());s=load_spec(spec);tree=ET.parse(world)
    w=tree.getroot().find('world')
    rgb=s['preview']['ambient_rgb']
    if len(rgb)!=3 or not all(math.isfinite(v) and 0<=v<=1 for v in rgb):
        raise ValueError('GUI ambient must be three values in [0,1]')
    output.mkdir(parents=True)
    if mode=='lighting':
        w.find('scene/ambient').text=' '.join(map(str,[*rgb,1]))
    elif mode=='robot':
        from .stage_b_robot import make_robot
        old=w.find("model[@name='scan_car']")
        w.remove(old);w.append(make_robot(output,c,s))
    else: raise ValueError('unknown GUI update mode')
    ET.indent(tree);tree.write(output/'world.sdf',encoding='unicode',xml_declaration=True)
    report=dict(mode=mode,source_world=dict(file=str(world),sha256=digest(world)),
        config=dict(file=str(config),sha256=digest(config)),spec=dict(file=str(spec),sha256=digest(spec)),
        world_sha256=digest(output/'world.sdf'),ambient_rgb=rgb,
        limitations='GUI world only; optical configuration unchanged; ideal carriage constraint retained.')
    (output/'manifest.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report))
    return report


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for k in ('world','config','spec','output'):p.add_argument('--'+k,required=True,type=Path)
    p.add_argument('--mode',choices=('lighting','robot'),required=True)
    a=p.parse_args();prepare(a.world,a.config,a.spec,a.output,a.mode)


if __name__=='__main__':main()
