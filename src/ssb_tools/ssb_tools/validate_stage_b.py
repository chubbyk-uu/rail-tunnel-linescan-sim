"""Validate a Stage B optical smoke session, independently of the rendering code.

This checks provenance, assets, hardware timing, CPU triangle hits, bounded texture
allocations and replay identity. It is not a full photometric/MTF acceptance report.
"""
import argparse
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import yaml

from . import ref_geometry
from .session import Session,read_json,sha256_file
from .validate_stage_a import (check,PASS,FAIL,UNMEASURABLE,verify_hashes,provenance_chain,
                              planned_motion,compare_timing,accounting,valid_region,gate_geometry,
                              advance_per_rev,compare_sessions)


def distance_stop_check(session, cfg, poses):
    """Independent encoder/rate check over the declared parking hold; no body x."""
    stop=cfg['motion']['distance_stop'];motion=session.summary['motion']
    if len(poses)<2:
        return check('distance_target_and_parking',UNMEASURABLE,reason='too few encoder samples')
    counts=cfg['odometer']['ppr']*cfg['odometer']['edges_per_cycle']*cfg['odometer']['gear_ratio']
    dl,dr=(cfg['calibration'][f'odo_{side}_diameter_m'] for side in ('left','right'))
    distance=np.zeros(len(poses));speed=np.zeros(len(poses))
    for field,diameter in (('wheel',dl),('right_wheel',dr)):
        indices=np.floor(poses[field]*counts/(2*np.pi))
        distance+=(indices-indices[0])*np.pi*diameter/counts/2
        rate='wheel_omega' if field=='wheel' else 'right_wheel_omega'
        speed+=poses[rate]*diameter/4
    held=poses['t']>=poses['t'][-1]-stop['hold_s']+cfg['motion']['sample_period_s']
    # Initial producer zero is sampled immediately before the first archived pose.
    quantum=np.pi*(dl+dr)/counts/2
    error=float(np.max(abs(distance[held]-stop['target_m'])))
    max_speed=float(np.max(abs(speed[held])))
    ok=(held.sum()>=int(stop['hold_s']/cfg['motion']['sample_period_s'])-1 and
        motion.get('completion_basis')=='dual_encoder_distance_and_park' and
        motion.get('complete') is True and error<=stop['tolerance_m']+quantum and
        max_speed<=stop['speed_tolerance_m_s'] and
        abs(motion.get('estimated_distance_m',float('inf'))-distance[-1])<=quantum+1e-12)
    return check('distance_target_and_parking',PASS if ok else FAIL,
        target_m=stop['target_m'],recomputed_estimated_m=float(distance[-1]),
        held_error_max_m=error,held_speed_max_m_s=max_speed,zero_quantization_bound_m=quantum,
        hold_samples=int(held.sum()),hold_s=stop['hold_s'])


def runtime_source_budget(scene, scene_path):
    return read_json(Path(scene_path).parent/scene['surface']['file'])['resources']['gpu_source_budget_bytes']


def gazebo_world_checks(session, source, prov):
    """Contact validation uses only protected archived physical inputs."""
    from .physical_world import snapshot_inputs
    from .validate_contact import physical_world_report
    entry = prov['inputs']['world']
    checks = []
    if prov['pose_source'] == 'gazebo_contact':
        snapshot, world_path, spec = snapshot_inputs(session.root)
        archived = {str(p.relative_to(session.root)) for p in snapshot.rglob('*') if p.is_file()}
        registered = {n for n in session.summary.get('files', {}) if n.startswith('evaluation/physical/')}
        checks.append(check('physical_snapshot_identity',
                            PASS if archived == registered and sha256_file(world_path) == entry['sha256'] else FAIL,
                            unprotected=sorted(archived-registered), missing=sorted(registered-archived),
                            world_matches_capture_input=sha256_file(world_path) == entry['sha256']))
        match = physical_world_report(session.root, None, source, spec)
        failed = [n for n, v in match['checks'].items() if not v['passed']]
        checks.append(check('physical_world_matches_truth', PASS if match['passed'] else FAIL, failed=failed))
    else:
        world_path = Path(entry['path'])
    car = ET.parse(world_path).getroot().find("world/model[@name='scan_car']")
    world_x = float(car.findtext('pose').split()[0])
    checks.append(check('gazebo_world_matches_capture_origin',
                        PASS if abs(world_x-source['motion']['start_x_m']) < 1e-9 else FAIL,
                        world_start_x_m=world_x, capture_start_x_m=source['motion']['start_x_m']))
    return checks


def optical_assets(session):
    archive=read_json(session.root/'evaluation/optical_assets.json')
    backend=read_json(session.root/'config/backend.json')['describe']
    source=yaml.safe_load((session.root/'evaluation/config_source.yaml').read_text())
    scene_path=Path(source['render']['optical_scene'])
    if not scene_path.is_absolute():
        provenance=read_json(session.root/'config/provenance.json')
        scene_path=Path(provenance['inputs']['config']['path']).parent/scene_path
    errors=[]
    scene=read_json(scene_path)
    if sha256_file(scene_path)!=backend['optical_scene_sha256'] or scene!=archive['scene']:
        errors.append('scene identity/archived content')
    for key in ('surface','defects'):
        path=scene_path.parent/scene[key]['file']
        if sha256_file(path)!=scene[key]['sha256'] or scene[key]['sha256']!=archive[key+'_sha256'] or read_json(path)!=archive[key]:
            errors.append(key+' identity/archived content')
        if archive[key+'_sha256']!=backend[key+'_sha256']: errors.append(key+' backend identity')
    files=0
    for mesh in scene['meshes']:
        if sha256_file(scene_path.parent/mesh['file'])!=mesh['sha256']: errors.append(mesh['file'])
        files+=1
    surface_path=scene_path.parent/scene['surface']['file']
    surface=archive['surface']
    if surface['schema']=='ssb.surface_runtime.v1':
        e=surface['recipe'];rp=surface_path.parent/e['file']
        if sha256_file(rp)!=e['sha256']:errors.append('runtime recipe identity')
        recipe=read_json(rp);files+=1
        # Every payload the generator reads, including the optional low-frequency macro map.
        for entry in [*recipe['sources'],recipe['alpha'],*([recipe['macro']] if recipe.get('macro') else [])]:
            if sha256_file(rp.parent/entry['file'])!=entry['sha256']:errors.append(entry['file'])
            files+=1
    else:
        for tile in surface['tiles']:
            if sha256_file(surface_path.parent/tile['file'])!=tile['sha256']: errors.append(tile['file'])
            files+=1
    if scene.get('filler'):
        fp=scene_path.parent/scene['filler']['file']
        if sha256_file(fp)!=scene['filler']['sha256']:errors.append('filler identity')
        else:
            meta=read_json(fp)
            if sha256_file(fp.parent/meta['file'])!=meta['sha256']:errors.append(meta['file'])
            files+=2
    defect_path=scene_path.parent/scene['defects']['file']
    for entry in archive['defects']['files'].values():
        if sha256_file(defect_path.parent/entry['file'])!=entry['sha256']: errors.append(entry['file'])
        files+=1
    snapshots=[e for e in archive['defects'].get('inputs',{}).values() if e.get('source_file')]
    for stage in ('refined_from','depth_from'):
        entry=archive['defects'].get(stage,{}).get('spec')
        if entry and entry.get('source_file'): snapshots.append(entry)
    for entry in snapshots:
        if sha256_file(defect_path.parent/entry['file'])!=entry['sha256']:errors.append(entry['file'])
        files+=1
    return check('optical_asset_identity',PASS if not errors else FAIL,files=files,errors=errors),scene,scene_path


def cpu_geometry(session,scene,scene_path,ray_count=64):
    vertices=[];triangles=[]
    for mesh in scene['meshes']:
        base=len(vertices)
        with (scene_path.parent/mesh['file']).open() as stream:
            for line in stream:
                if line.startswith('v '): vertices.append([float(v) for v in line.split()[1:]])
                if line.startswith('f '): triangles.append([base+int(v.split('/')[0])-1 for v in line.split()[1:]])
    # Source geometry remains float64. Validate accuracy against the intended mesh,
    # including localization/quantization, instead of copying GPU float rounding.
    vertices=np.array(vertices,dtype=np.float64);triangles=np.array(triangles)
    a=vertices[triangles[:,0]];e1=vertices[triangles[:,1]]-a;e2=vertices[triangles[:,2]]-a
    row_truth=session.evaluation('row_truth');hits=session.evaluation('debug_hits')['hits']
    columns=read_json(session.root/'evaluation/manifest.json')['debug_hits']['columns']
    if not columns or not len(row_truth): return check('cpu_triangle_hits',UNMEASURABLE,reason='no debug hits')
    rng=np.random.default_rng(20260929)
    rows=rng.integers(0,len(row_truth),size=ray_count);slots=rng.integers(0,len(columns),size=ray_count)
    origin,optical,line=ref_geometry.head_pose(row_truth['theta'][rows],row_truth['x'][rows],session.truth(),row_truth[rows])
    tangents=ref_geometry.evaluation_pixel_tangents(session.config()['camera'],session.truth(),np.array(columns)[slots])
    radius=session.truth()['tunnel']['radius_m'];zc=session.truth()['tunnel']['axis_z_m']
    worst=0.;misses=0
    for i,(row,slot) in enumerate(zip(rows,slots)):
        d=optical[i]+tangents[i]*line[i]
        p=np.cross(d,e2);det=np.einsum('ij,ij->i',e1,p);rel=origin[i]-a
        with np.errstate(divide='ignore',invalid='ignore'):
            u=np.einsum('ij,ij->i',rel,p)/det
            cross=np.cross(rel,e1);v=cross@d/det
            distance=np.einsum('ij,ij->i',e2,cross)/det
        visible=(np.abs(det)>1e-12)&(u>=0)&(v>=0)&(u+v<=1)&(distance>0)
        if not visible.any(): misses+=1;continue
        point=origin[i]+d*distance[visible].min()
        want=np.array([point[0],radius*np.arctan2(point[1],point[2]-zc)])
        worst=max(worst,float(np.max(np.abs(hits[row,slot]-want))))
    finite=bool(np.isfinite(hits).all())
    return check('cpu_triangle_hits',PASS if finite and not misses and worst<5e-6 else FAIL,
                 rays=ray_count,max_error_m=worst,threshold_m=5e-6,misses=misses,all_archived_hits_finite=finite)


def identity_path(report):
    return report.with_suffix('.identity.json')


def report_destination(session_root, output=None, read_only=False):
    if read_only:
        return None
    report = Path(output) if output else Path(session_root)/'evaluation/reports/stage_b_smoke.json'
    for path in (report, identity_path(report)):
        if path.exists() or path.is_symlink():
            raise FileExistsError(f'preserve existing evidence: {path}; use --read-only or a fresh --output')
    if 'evaluation' not in report.resolve().parts:
        raise ValueError('validation reports belong in evaluation/')
    return report


def write_report(result, report, session_root, compare=None):
    # Check both files before writing; exclusive creation also prevents replacement
    # if another validator races this one. Only two small files per validation.
    report_destination(session_root, report)
    evidence = dict(session_json_sha256=sha256_file(Path(session_root)/'session.json'),
                    compare_session_json_sha256=sha256_file(Path(compare)/'session.json') if compare else None)
    result['evidence'] = evidence
    report.parent.mkdir(parents=True, exist_ok=True)
    with report.open('x') as stream:
        stream.write(json.dumps(result, indent=2)+'\n')
    identity = dict(schema='ssb.stage_b_identity.v1', report_sha256=sha256_file(report), **evidence)
    with identity_path(report).open('x') as stream:
        stream.write(json.dumps(identity, indent=2)+'\n')


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('session');p.add_argument('--compare')
    mode=p.add_mutually_exclusive_group()
    mode.add_argument('--output', help='fresh report path inside evaluation/')
    mode.add_argument('--read-only', action='store_true', help='print checks without writing evidence')
    args=p.parse_args(argv)
    report=report_destination(args.session, args.output, args.read_only)
    session=Session(args.session);checks=[]
    try:
        checks.append(check('session_complete',PASS if session.summary.get('status')=='complete' else FAIL))
        count,bad=verify_hashes(session);checks.append(check('stored_hashes_verify',PASS if count and not bad else FAIL,files=count,errors=bad))
        cfg,truth=session.config(),session.truth();prov=read_json(session.root/'config/provenance.json');backend=read_json(session.root/'config/backend.json')
        checks.append(check('binary_matches_source',PASS if prov.get('binary_matches_source') else FAIL))
        checks.append(provenance_chain(session,cfg,truth,prov,backend));checks.append(planned_motion(session.summary))
        source=yaml.safe_load((session.root/'evaluation/config_source.yaml').read_text())
        if prov.get('pose_source') in ('gazebo','gazebo_contact'):
            checks.extend(gazebo_world_checks(session, source, prov))
        poses=session.evaluation('pose_stream');row_truth=session.evaluation('row_truth')
        if cfg['motion'].get('distance_stop') and prov.get('pose_source')=='gazebo_contact':
            checks.append(distance_stop_check(session,cfg,poses))
        timing,rows,dropped=compare_timing(session,cfg,truth,poses);checks.extend(timing)
        checks.extend([accounting(rows,dropped),valid_region(source,poses,rows,row_truth,dropped),
                       gate_geometry(cfg,truth,rows,row_truth),advance_per_rev(cfg,row_truth,poses)])
        if cfg.get('contact',{}).get('enabled'):
            from .ref_timing import lattice_crossings
            spacing=2*np.pi/(cfg['odometer']['ppr']*cfg['odometer']['edges_per_cycle']*cfg['odometer']['gear_ratio'])
            ts,counts,dirs=lattice_crossings(poses,'right_wheel','right_wheel_omega',0.,spacing)
            counts=counts-np.floor(poses['right_wheel'][0]/spacing)
            recorded=session.metadata('odometer_right_edges')
            equal=len(ts)==len(recorded)
            equal=equal and np.array_equal(counts,recorded['count']) and np.array_equal(dirs,recorded['dir'])
            delta=float(np.max(abs(ts-recorded['t']))) if equal and len(ts) else 0.
            checks.append(check('right_encoder_edges',PASS if equal and delta<1e-6 else FAIL,count=len(recorded),max_time_error_s=delta))
        identity,scene,scene_path=optical_assets(session);checks.append(identity)
        checks.append(cpu_geometry(session,scene,scene_path))
        final=session.summary['performance']['backend_final'];budget=final['gpu_texture_budget_bytes'];peak=final['texture_allocated_peak_bytes']
        checks.append(check('gpu_texture_budget',PASS if 0<peak<=budget else FAIL,peak_bytes=peak,budget_bytes=budget))
        budget=final['cpu_texture_budget_bytes'];peak=final['cpu_texture_allocated_peak_bytes']
        if final.get('runtime_surface_recipe'):
            # Tiles are generated on the GPU from the recipe: the CPU tile cache must stay
            # unused, and the recipe source upload has its own device budget.
            source_budget=runtime_source_budget(scene,scene_path)
            used=final['recipe_source_device_bytes']
            checks.append(check('cpu_texture_budget',PASS if peak==0 else FAIL,peak_bytes=peak,budget_bytes=budget,
                                note='runtime recipe: CPU tile cache unused'))
            checks.append(check('recipe_source_budget',PASS if 0<used<=source_budget else FAIL,
                                device_bytes=used,budget_bytes=source_budget))
        else:
            checks.append(check('cpu_texture_budget',PASS if 0<peak<=budget else FAIL,peak_bytes=peak,budget_bytes=budget))
        checks.append(compare_sessions(session.root,Path(args.compare)) if args.compare else
                      check('reimaging_byte_identical',UNMEASURABLE,reason='--compare required'))
    except Exception as error:
        checks.append(check('validation_error',FAIL,error=str(error)))
    result=dict(schema='ssb.stage_b_smoke_report.v1',scope=__doc__,overall=PASS if all(c['state']==PASS for c in checks) else FAIL,checks=checks)
    if report is not None:
        write_report(result, report, session.root, args.compare)
    for c in checks: print(c['name'],c['state'])
    print('overall:',result['overall']);return 0 if result['overall']==PASS else 1


if __name__=='__main__': raise SystemExit(main())
