#!/usr/bin/env python3
"""Evaluation-only ring-centre lines in an unchanged reconstruction coordinate system."""
import argparse
import json
from pathlib import Path
import numpy as np
import yaml
from scipy.interpolate import LinearNDInterpolator
from ssb_tools.evaluate_global_geometry import map_points
from ssb_tools.evaluate_defect_preservation import locate
from ssb_tools.global_resample import load_global
from ssb_tools.match_bands import verified_bands
from ssb_tools.provenance import stage_record
from ssb_tools.ref_mesh import OpticalMesh, session_scene
from ssb_tools.session import Session, read_json, sha256_file


def ring_summary(target, locations, valid, ring_count, q_count):
    target, locations, valid = np.asarray(target), np.asarray(locations), np.asarray(valid, bool)
    if ring_count < 1 or q_count < 3 or target.shape != (ring_count*q_count, 2) or locations.shape != target.shape or valid.shape != (len(target),):
        raise ValueError('one result for every declared ring and angle required')
    records, deviations = [], []
    for i in range(ring_count):
        take = slice(i*q_count, (i+1)*q_count)
        ok = valid[take]
        record = dict(true_centre_x_m=float(target[take][0, 0]), planned=q_count,
                      measured=int(ok.sum()), missing=int((~ok).sum()))
        if ok.all() and np.isfinite(locations[take]).all():
            x = locations[take, 0]
            deviation = x-np.median(x)
            deviations.extend(deviation)
            record.update(status='measured', output_median_x_m=float(np.median(x)),
                          peak_to_peak_mm=float(np.ptp(x)*1000),
                          p95_abs_from_median_mm=float(np.percentile(abs(deviation), 95)*1000))
        else:
            record['status'] = 'unmeasurable'
        records.append(record)
    complete = all(v['status'] == 'measured' for v in records)
    return dict(status='measured' if complete else 'unmeasurable', rings=records,
                planned=len(target), missing=int((~valid).sum()),
                p95_abs_from_per_ring_median_mm=float(np.percentile(abs(np.asarray(deviations)),95)*1000)
                    if complete else None,
                convention='Only each ring position is removed; no line, slope, rotation, scale or shear is fitted out.')


def run(session_root, unroll, trajectory, geometry, output, workers=4):
    output = Path(output).resolve()
    if output.exists() or 'evaluation' not in output.parts:
        raise ValueError('fresh evaluation directory required')
    session = Session(session_root)
    scene_path, scene = session_scene(session)
    spec_path = scene_path.parent/scene['runtime_spec']['file']
    if sha256_file(spec_path) != scene['runtime_spec']['sha256']:
        raise ValueError('archived geometry specification identity differs')
    spec = yaml.safe_load(spec_path.read_text())
    geometry, trajectory, unroll = map(Path, (geometry, trajectory, unroll))
    geometry_record = read_json(geometry/'provenance.json')
    sample_path = geometry/'mapping_samples.npz'
    if [v for k,v in geometry_record['outputs'].items() if Path(k).name==sample_path.name] != [sha256_file(sample_path)]:
        raise ValueError('original geometry sample identity differs')
    sampler, upstream, public_inputs = verified_bands(unroll)
    try:
        model, coefficients, fit, fit_inputs = load_global(trajectory, sampler, upstream, unroll)
        for path,digest in upstream['source_observation_hashes'].items():
            if sha256_file(session.root/path) != digest:
                raise ValueError('source capture identity differs')
        lo, hi = upstream['grid']['target_x_m']
        width = spec['panels']['ring_width_m']
        rings = np.arange(np.floor(lo/width)+1, np.ceil(hi/width))*width
        rings = rings[(rings>lo+.1)&(rings<hi-.1)]
        q = np.linspace(-np.deg2rad(115)*model.radius, np.deg2rad(115)*model.radius, 33)
        xx, qq = np.meshgrid(rings, q, indexing='ij')
        target = np.column_stack((xx.ravel(), qq.ravel()))
        if not len(target):
            raise ValueError('no internal ring centre is available')
        output.mkdir(parents=True)
        plan = dict(schema='ssb.ring_shape_plan.v1', rings_x_m=rings.tolist(), angle_deg=[-115,115],
                    samples_per_ring=33, location_tolerance_m=1e-5, iterations=5,
                    convention='fixed ring centres from archived geometry; no score-driven selection')
        (output/'sampling_plan.json').write_text(json.dumps(plan,indent=2)+'\n')
        rows, truth, camera = session.evaluation('row_truth'), session.truth(), session.config()['camera']
        mesh = OpticalMesh.from_session(session)
        result = dict(schema='ssb.ring_shape_evaluation.v1', evaluation_only=True,status='diagnostic',
            limitations=['Truth locates ring centres only; no production or saved image is changed.',
                'The central 230 degrees are sampled; endpoints and all pixels are not certified.',
                'Ring-centre coordinates measure shape, not irregular lip edges or image-intensity centroids.',
                'Image-only priors fix coordinate gauge; no absolute pose or exact installation recovery is claimed.'])
        arrays = dict(target_xq_m=target)
        with np.load(sample_path) as samples:
            a,b = np.meshgrid(samples['output_x_m'],samples['output_q_m'])
            seeds = np.stack((a,b),-1).reshape(-1,2)
            for name, c, relief in [('nominal',np.zeros_like(coefficients),False),('optimized',coefficients,True)]:
                initial = LinearNDInterpolator(samples[name+'_true_xq_m'].reshape(-1,2),seeds)(target)
                trace = lambda location: map_points(model,c,location[:,0],location[:,1],rows,camera,truth,mesh,workers,relief)
                locations, valid, residual, material, band = locate(target,initial,trace)
                result[name] = ring_summary(target,locations,valid,len(rings),len(q))
                for key,value in dict(output_xq_m=locations,valid=valid,residual_m=residual,material=material,band=band).items():
                    arrays[name+'_'+key]=value
        np.savez_compressed(output/'samples.npz',**arrays)
        report = output/'report.json'
        report.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
        inputs=[Path(__file__).resolve(),scene_path,spec_path,geometry/'report.json',geometry/'provenance.json',
                sample_path,session.root/'evaluation/truth.json',session.root/'evaluation/manifest.json',
                session.root/'evaluation'/read_json(session.root/'evaluation/manifest.json')['row_truth']['file'],
                *public_inputs,*fit_inputs,*mesh.sources]
        record=stage_record('ring_shape_evaluation',inputs,list(output.iterdir()),plan)
        (output/'provenance.json').write_text(json.dumps(record,indent=2)+'\n')
        return result
    finally:
        sampler.native.release()


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('session','unroll','trajectory','geometry','output'):
        p.add_argument('--'+name,required=True)
    p.add_argument('--workers',type=int,default=4)
    result=run(**{('session_root' if k=='session' else k):v for k,v in vars(p.parse_args()).items()})
    print(json.dumps({name:result[name]['p95_abs_from_per_ring_median_mm'] for name in ('nominal','optimized')}))
