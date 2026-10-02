#!/usr/bin/env python3
"""Change the real physics step mid-capture: both plugins must drain and fail."""
import argparse
import copy
import json
import os
from pathlib import Path
import subprocess
import time
import xml.etree.ElementTree as ET
import yaml
from ssb_tools.stage_b_scene import make_world, load_spec
from ssb_tools.session import sha256_file
from ssb_tools.process_drain import stop_group


def run(output):
    repo = Path(__file__).resolve().parents[1]
    output = Path(output).resolve(); output.mkdir(parents=True, exist_ok=False)
    base = yaml.safe_load((repo/'src/ssb_core/config/stage_b.yaml').read_text())
    spec = load_spec(repo/'src/ssb_tools/config/stage_b_scene.yaml')
    results = []
    for contact in (False, True):
        folder = output/('contact' if contact else 'scan'); folder.mkdir()
        c = copy.deepcopy(base)
        c['camera']['width'] = 64; c['render']['debug_column_stride'] = 0
        c['motion'].update(start_x_m=3., start_theta_deg=-30., profile=[[0.,1.],[10.,1.]])
        c['acceptance']['valid_x_m'] = [3.1, 3.5]
        c['contact'] = dict(enabled=contact, settle_s=.5)
        if contact:
            for section in ('truth','calibration'):
                c[section].update(odo_left_diameter_m=.08, odo_right_diameter_m=.08)
        for name in ('panels','joints','filler','gap'):
            (folder/(name+'.obj')).write_text('v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n')
        make_world(folder,c,spec)
        config=folder/'capture.yaml'; config.write_text(yaml.safe_dump(c))
        (folder/'spec.yaml').write_text(yaml.safe_dump(spec))
        world=folder/'world.sdf'; world_name=ET.parse(world).find('world').get('name')
        session=folder/'session'
        env=dict(os.environ, SSB_CONFIG=str(config), SSB_WORLD=str(world), SSB_SESSION=str(session),
                 GZ_PARTITION=f'ssb_runtime_fault_{os.getpid()}_{contact}')
        with (folder/'server.log').open('w') as log:
            proc=subprocess.Popen(['gz','sim','-s','-r','-v','3',str(world)],env=env,
                                  stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            try:
                deadline=time.monotonic()+30
                while not (session/'raw/block_000000.u8').is_file():
                    assert proc.poll() is None, 'server failed before capture'
                    assert time.monotonic()<deadline, 'no raw block before fault'
                    time.sleep(.05)
                with (folder/'service.log').open('w') as service_log:
                    subprocess.run(['gz','service','-s',f'/world/{world_name}/set_physics',
                        '--reqtype','gz.msgs.Physics','--reptype','gz.msgs.Boolean',
                        '--timeout','3000','--req','max_step_size: 0.002'],env=env,
                        stdout=service_log,stderr=subprocess.STDOUT,check=True,timeout=5)
                deadline=time.monotonic()+15
                while True:
                    summary=json.loads((session/'session.json').read_text())
                    if summary['status']=='failed': break
                    assert time.monotonic()<deadline, 'runtime fault did not drain and fail'
                    time.sleep(.05)
                assert 'step changed' in summary['error']
                assert not summary['motion']['complete'] and summary['rows']>0
                index=json.loads((session/'raw/index.json').read_text())
                assert index['rows']==summary['rows']
                for block in index['blocks']:
                    assert sha256_file(session/'raw'/block['file'])==block['sha256']
                result=stop_group(proc,10.)
                assert result['returncode']==0 and not result['forced'],result
                results.append(dict(plugin='contact' if contact else 'scan',rows=summary['rows'],
                                    failed=True,raw_hashes=True,graceful_exit=True))
            finally:
                if proc.poll() is None: stop_group(proc,10.)
    (output/'report.json').write_text(json.dumps(results,indent=2)+'\n')
    print(json.dumps(results))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',required=True)
    run(parser.parse_args().output)
