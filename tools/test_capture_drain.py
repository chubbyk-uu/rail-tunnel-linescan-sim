#!/usr/bin/env python3
"""Real OptiX backlog drains after SIGINT, with progress independent of physics."""
import argparse
import copy
import json
import os
from pathlib import Path
import subprocess
import threading
import time
import yaml
from gz.transport13 import Node
from gz.msgs10.stringmsg_pb2 import StringMsg
from ssb_tools.stage_b_scene import make_world, load_spec
from ssb_tools.session import sha256_file
from ssb_tools.process_drain import stop_group


def run(output):
    repo = Path(__file__).resolve().parents[1]
    output = Path(output).resolve(); output.mkdir(parents=True, exist_ok=False)
    os.environ['GZ_PARTITION'] = f'ssb_drain_test_{os.getpid()}'
    node = Node()
    base = yaml.safe_load((repo/'src/ssb_core/config/stage_b.yaml').read_text())
    spec = load_spec(repo/'src/ssb_tools/config/stage_b_scene.yaml')
    results = []
    for contact in (False, True):
        folder = output/('contact' if contact else 'scan'); folder.mkdir()
        c = copy.deepcopy(base)
        c['camera']['width'] = 64
        c['render'].update(batch_rows=256, debug_column_stride=0, debug_delay_per_batch_s=.06)
        c['motion'].update(start_x_m=3., start_theta_deg=0., profile=[[0.,1.],[10.,1.]])
        c['contact'] = dict(enabled=contact, settle_s=.5)
        if contact:
            for section in ('truth', 'calibration'):
                c[section].update(odo_left_diameter_m=.08, odo_right_diameter_m=.08)
        for name in ('panels', 'joints', 'filler', 'gap'):
            (folder/(name+'.obj')).write_text('v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n')
        make_world(folder, c, spec)
        config = folder/'capture.yaml'; config.write_text(yaml.safe_dump(c))
        (folder/'spec.yaml').write_text(yaml.safe_dump(spec))
        world = folder/'world.sdf'; session = folder/'session'
        topic = f'/ssb/drain_test/{"contact" if contact else "scan"}'
        samples = []; lock = threading.Lock()
        def receive(message):
            data = json.loads(message.data)
            if data['session'] == str(session):
                with lock: samples.append((time.monotonic(), data['capture']))
        assert node.subscribe(StringMsg, topic+'/capture', receive)
        env = dict(os.environ, SSB_CONFIG=str(config), SSB_WORLD=str(world),
                   SSB_SESSION=str(session), SSB_MISSION_STATUS_TOPIC=topic)
        with (folder/'server.log').open('w') as log:
            proc = subprocess.Popen(['gz','sim','-s','-r','-v','3',str(world)], env=env,
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            try:
                deadline = time.monotonic()+30
                while True:
                    with lock: latest = dict(samples[-1][1]) if samples else {}
                    if latest.get('rows_generated', 0) >= 16384: break
                    assert proc.poll() is None and time.monotonic() < deadline, 'no capture progress'
                    time.sleep(.05)
                stopped_at = time.monotonic()
                def activity():
                    with lock: return samples[-1][1]['activity_sequence'] if samples else 0
                result = stop_group(proc, .75, 2., 2., progress=activity)
                assert not result['forced'] and result['returncode'] == 0, result
                assert result['elapsed_s'] > .75 and result['progress_extensions'] > 0, result
                with lock: after = [v for t, v in samples if t > stopped_at+.5]
                assert len(after) >= 2 and after[-1]['activity_sequence'] > after[0]['activity_sequence']
                summary = json.loads((session/'session.json').read_text())
                index = json.loads((session/'raw/index.json').read_text())
                assert summary['status'] == 'complete' and summary['rows'] == index['rows'] > 0
                assert not summary['motion']['complete']
                for block in index['blocks']:
                    assert sha256_file(session/'raw'/block['file']) == block['sha256']
                results.append(dict(plugin='contact' if contact else 'scan', rows=summary['rows'],
                    post_signal_reports=len(after), complete=True, raw_hashes=True, shutdown=result))
            finally:
                if proc.poll() is None: stop_group(proc, 10.)
                node.unsubscribe(topic+'/capture')
    (output/'report.json').write_text(json.dumps(results, indent=2)+'\n')
    print(json.dumps(results))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    run(parser.parse_args().output)
