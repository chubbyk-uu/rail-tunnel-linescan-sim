#!/usr/bin/env python3
"""Real wheel-contact regression: nominal and signed measuring-wheel scale errors.

Uses installed generators/controllers. Truth only generates physical wheel geometry
and independent evaluation; it never changes the calibrated 80 mm encoder conversion.
One CSV per run, no per-sample files. --capture adds full-resolution raw + independent
reimaging/Stage B checks for buffered 3 m wall targets.
"""
import argparse
import copy
import json
import os
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import yaml

from ssb_tools.mission_plan import plan, wall_plan
from ssb_tools.optical_identity import check_calibration
from ssb_tools.physical_world import check, write_manifest
from ssb_tools.provenance import stage_record
from ssb_tools.session import read_json, sha256_file
from ssb_tools.stage_b_gui_world import prepare
from ssb_tools.validate_contact import validate

REPO=Path(__file__).resolve().parents[1]


def command(argv, log, env=None):
    with Path(log).open('w') as stream:
        result=subprocess.run(argv,env=env,stdout=stream,stderr=subprocess.STDOUT,timeout=300)
    return result.returncode


def generate(demo, folder, diameter, wall=False, timeout=False):
    folder.mkdir()
    c=yaml.safe_load((demo/'capture.yaml').read_text())
    c=copy.deepcopy(c)
    c['truth'].update(odo_left_diameter_m=diameter/1000,odo_right_diameter_m=diameter/1000)
    c['calibration'].update(odo_left_diameter_m=.08,odo_right_diameter_m=.08)
    calibration=read_json(demo/'calibration.json')
    c,task=wall_plan(c,3.,3.,calibration) if wall else plan(c,3.,3.)
    if timeout:
        # Legal but intentionally insufficient: cannot finish the ramps and park.
        c['motion']['distance_stop']['timeout_s']=15.1
    c['render']['optical_scene']=str((demo/c['render']['optical_scene']).resolve())
    cfg=folder/'capture.yaml';cfg.write_text(yaml.safe_dump(c,sort_keys=False))
    spec=yaml.safe_load((demo/'spec.yaml').read_text())
    (folder/'spec.yaml').write_text(yaml.safe_dump(spec,sort_keys=False))
    prepare(demo/'world/world.sdf',cfg,folder/'spec.yaml',folder/'world','robot')
    world=folder/'world/world.sdf'
    write_manifest(world,c,spec)
    assert check(c,spec,world)['passed']
    assert check_calibration(cfg,demo/'calibration.json')
    (folder/'task.json').write_text(json.dumps(task,indent=2)+'\n')
    return c,task,world


def run(demo, output, capture=False):
    demo,output=Path(demo).resolve(),Path(output).resolve()
    output.mkdir(parents=True,exist_ok=False)
    records=[]
    for diameter in (80,79,81):
        folder=output/f'wheel_{diameter}'
        c,task,world=generate(demo,folder,diameter)
        session=folder/'physics'
        assert command(['bash',str(REPO/'tools/run_contact_dynamics.sh'),str(session),
                        str(folder/'capture.yaml'),str(world)],folder/'physics.log')==0,folder/'physics.log'
        report=validate(str(session)+'_dynamics',folder/'capture.yaml')
        assert report['passed'],report
        assert abs(report['travel_m']-3*diameter/80)<.0005
        records.append(dict(diameter_mm=diameter,calibrated_mm=80,requested_estimated_m=3.,
            actual_m=report['travel_m'],estimated_m=report['estimated_m'],
            pitch_m=report['measured_pitch_m'],expected_pitch_m=.6*diameter/80,
            end_s=report['end_time_s'],contact_passed=True))
        if capture:
            image_folder=output/f'images_{diameter}'
            _,image_task,image_world=generate(demo,image_folder,diameter,wall=True)
            raw=image_folder/'capture';reimage=image_folder/'reimage'
            env=dict(os.environ,SSB_WORLD=str(image_world),GZ_PARTITION=f'ssb_distance_{os.getpid()}_{diameter}')
            assert command(['bash',str(REPO/'tools/run_gz.sh'),str(raw),str(image_folder/'capture.yaml')],
                           image_folder/'capture.log',env)==0,image_folder/'capture.log'
            exe=REPO/'install/ssb_core/lib/ssb_core/ssb_render'
            assert command([str(exe),'--config',str(raw/'evaluation/config_source.yaml'),
                '--session',str(reimage),'--poses',str(raw/'evaluation/pose_stream.bin'),
                '--batch-rows','333'],image_folder/'reimage.log')==0,image_folder/'reimage.log'
            assert command(['python3','-m','ssb_tools.validate_stage_b',str(raw),'--compare',str(reimage)],
                image_folder/'stage_b.log')==0,image_folder/'stage_b.log'
            summary=read_json(raw/'session.json')
            assert summary['motion']['completion_basis']=='dual_encoder_distance_and_park'
            records[-1].update(raw_session=str(raw),rows=summary['rows'],stage_b_passed=True,
                buffered_estimated_travel_m=image_task['distance_m'],
                captured_motion=summary['motion'])
        (output/'report.json').write_text(json.dumps(dict(cases=records),indent=2)+'\n')
    failed=output/'timeout'
    _,_,world=generate(demo,failed,80,timeout=True)
    session=failed/'physics'
    rc=command(['bash',str(REPO/'tools/run_contact_dynamics.sh'),str(session),
                str(failed/'capture.yaml'),str(world)],failed/'physics.log')
    summary=read_json(Path(str(session)+'_dynamics')/'summary.json')
    assert rc!=0 and not summary['complete'] and 'timed out' in summary['error']
    report=dict(schema='ssb.distance_stop_regression.v1',passed=True,cases=records,
        insufficient_timeout_rejected=True,capture_and_independent_reimage=capture,
        limitation='controlled known-seed sensitivity experiment; not blind D3 accuracy acceptance')
    (output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    proof=stage_record('distance_stop_regression',[demo/'capture.yaml',demo/'calibration.json',
        Path(__file__)],[output/'report.json'],dict(diameters_mm=[80,79,81],calibrated_mm=80,capture=capture))
    (output/'provenance.json').write_text(json.dumps(proof,indent=2)+'\n')
    print(json.dumps(report))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--demo',default=REPO/'local_data/stage_b/contact_demo_buffered')
    parser.add_argument('--output',required=True);parser.add_argument('--capture',action='store_true')
    run(**vars(parser.parse_args()))
