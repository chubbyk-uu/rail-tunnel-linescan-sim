"""Evaluation-only thin-crack profiles in the unchanged saved mosaic.

Truth locates specimens for measurement; it never rectifies a production image.
All specimen outcomes, including low contrast and missing support, are retained.
Intensity FWHM is distinguished from physical crack width and geometric scale.
"""
import argparse
import json
from pathlib import Path

import numpy as np
from scipy.interpolate import LinearNDInterpolator

from .evaluate_global_geometry import map_points
from .global_resample import load_global
from .match_bands import verified_bands
from .provenance import stage_record
from .ref_mesh import OpticalMesh, session_scene
from .session import Session, read_json, sha256_file


def path_samples(path, fractions):
    path = np.asarray(path, float)
    if path.ndim != 2 or path.shape[1] != 2 or len(path) < 2 or not np.isfinite(path).all():
        raise ValueError('finite nonempty crack path required')
    distance = np.r_[0., np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
    keep = np.r_[True, np.diff(distance) > 0]
    path, distance = path[keep], distance[keep]
    if len(path) < 2 or distance[-1] <= 0:
        raise ValueError('zero-length crack cannot be evaluated')
    fractions = np.asarray(fractions, float)
    if not np.isfinite(fractions).all() or np.any((fractions < 0) | (fractions > 1)):
        raise ValueError('finite path fractions in [0,1] required')
    target = fractions*distance[-1]
    points = np.column_stack([np.interp(target, distance, path[:, k]) for k in range(2)])
    segment = np.clip(np.searchsorted(distance, target, side='right')-1, 0, len(path)-2)
    tangent = path[segment+1]-path[segment]
    tangent /= np.linalg.norm(tangent, axis=1)[:, None]
    normal = np.column_stack((-tangent[:, 1], tangent[:, 0]))
    return points, normal, float(distance[-1])


def locate(target, initial, trace, tolerance=1e-5, iterations=5):
    target, initial = np.asarray(target, float), np.asarray(initial, float)
    if (target.ndim != 2 or target.shape[1] != 2 or not len(target) or
            initial.shape != target.shape or not np.isfinite(target).all() or
            not np.isfinite(tolerance) or tolerance <= 0 or
            type(iterations) is not int or iterations <= 0):
        raise ValueError('finite N by 2 targets and matching seed coordinates required')
    supported = np.isfinite(initial).all(1)
    location = np.where(supported[:, None], initial, [10., 0.])
    for _ in range(iterations):
        actual, material, band = trace(location)
        valid = supported & (band >= 0) & np.isfinite(actual).all(1)
        residual = np.linalg.norm(actual-target, axis=1)
        if np.all(residual[supported] < min(tolerance, 1e-6)):
            break
        location[valid] -= actual[valid]-target[valid]
    # Re-trace the returned coordinates: band, material and residual describe the
    # final point, rather than the point before the last Newton update.
    actual, material, band = trace(location)
    residual = np.linalg.norm(actual-target, axis=1)
    valid = supported & (band >= 0) & np.isfinite(actual).all(1) & (residual < tolerance)
    return location, valid, residual, material, band


def intensity_width(offsets, values, minimum_contrast_dn=5.):
    offsets, values = np.asarray(offsets, float), np.asarray(values, float)
    if (offsets.ndim != 1 or offsets.shape != values.shape or len(offsets) < 9 or
            not np.isfinite(offsets).all() or not np.all(np.diff(offsets) > 0)):
        raise ValueError('ordered nonempty cross-crack profile required')
    if not np.isfinite(values).all():
        return dict(status='unmeasurable', reason='saved pixel lacks native support')
    background = float(np.median(np.r_[values[:5], values[-5:]]))
    near = np.flatnonzero(abs(offsets) <= .0004)
    if not len(near):
        raise ValueError('profile does not sample the independently located centre')
    centre = int(near[np.argmin(values[near])])
    contrast = background-float(values[centre])
    if contrast < minimum_contrast_dn:
        return dict(status='unmeasurable', reason='contrast below declared 5 DN', contrast_dn=contrast)
    half = background-contrast/2
    a = b = centre
    while a > 0 and values[a-1] < half:
        a -= 1
    while b < len(values)-1 and values[b+1] < half:
        b += 1
    if a == 0 or b == len(values)-1:
        return dict(status='unmeasurable', reason='intensity dip reaches profile boundary', contrast_dn=contrast)
    left = float(np.interp(half, [values[a], values[a-1]], [offsets[a], offsets[a-1]]))
    right = float(np.interp(half, [values[b], values[b+1]], [offsets[b], offsets[b+1]]))
    return dict(status='measured', apparent_fwhm_mm=(right-left)*1000,
                contrast_dn=contrast, centre_shift_mm=float(offsets[centre]*1000))


def saved_samples(image, grid, points):
    points = np.asarray(points, float)
    if points.shape[-1] != 2 or not np.isfinite(points).all():
        raise ValueError('finite saved-image coordinates required')
    x = (points[..., 0]-grid['target_x_m'][0])/grid['dx_m']-.5
    q = (points[..., 1]-grid['theta_rad'][0]*grid['radius_m'])/grid['dq_m']-.5
    ix, iq = np.floor(x).astype(int), np.floor(q).astype(int)
    valid = (ix >= 0) & (iq >= 0) & (ix+1 < image.shape[1]) & (iq+1 < image.shape[0])
    ix, iq = np.clip(ix, 0, image.shape[1]-2), np.clip(iq, 0, image.shape[0]-2)
    values = [np.asarray(image[iq+dy, ix+dx], float) for dy, dx in ((0,0),(0,1),(1,0),(1,1))]
    valid &= np.logical_and.reduce([v != 65535 for v in values])
    wx, wq = x-np.floor(x), q-np.floor(q)
    result = ((values[0]*(1-wx)+values[1]*wx)*(1-wq)+
              (values[2]*(1-wx)+values[3]*wx)*wq)/64
    return np.where(valid, result, np.nan)


def run(session_root, unroll, trajectory, mosaic, geometry, output, workers=4):
    session = Session(session_root)
    unroll, trajectory, mosaic, geometry, output = map(Path, (unroll, trajectory, mosaic, geometry, output))
    output = output.resolve()
    if 'evaluation' not in output.parts or output.exists():
        raise ValueError('fresh evaluation-only defect output required')
    scene_path, scene = session_scene(session)
    defect_path = scene_path.parent/scene['defects']['file']
    if sha256_file(defect_path) != scene['defects']['sha256']:
        raise ValueError('defect layout differs from archived capture')
    layout = read_json(defect_path)
    sampler, upstream, public_inputs = verified_bands(unroll)
    try:
        model, coefficients, fit, fit_inputs = load_global(trajectory, sampler, upstream, unroll)
        for name, digest in upstream['source_observation_hashes'].items():
            if sha256_file(session.root/name) != digest:
                raise ValueError('defect evaluation session differs from source observations')
        mosaic_report = read_json(mosaic/'report.json')
        if mosaic_report['grid'] != upstream['grid'] or mosaic_report.get('fusion') is not None:
            raise ValueError('unchanged optimized unblended mosaic on the same grid required')
        image_path = mosaic/'optimized/mosaic_u16.npy'
        provenance_path = mosaic/'provenance.json'
        mosaic_provenance = read_json(provenance_path)
        for path in fit_inputs:
            identities = [v for k, v in mosaic_provenance['inputs'].items()
                          if Path(k).resolve() == path.resolve()]
            if identities != [sha256_file(path)]:
                raise ValueError('saved mosaic does not belong to this fitted trajectory')
        identities = [v for k, v in mosaic_provenance['outputs'].items()
                      if Path(k).resolve() == image_path.resolve()]
        if identities != [sha256_file(image_path)]:
            raise ValueError('saved mosaic identity differs from its original provenance')
        image = np.load(image_path, mmap_mode='r')
        if image.shape != tuple(upstream['grid']['shape']) or image.dtype != np.uint16:
            raise ValueError('full saved mosaic shape or quantization differs')
        with np.load(geometry/'mapping_samples.npz') as maps:
            xx, qq = np.meshgrid(maps['output_x_m'], maps['output_q_m'])
            seed = LinearNDInterpolator(maps['optimized_true_xq_m'].reshape(-1, 2),
                                        np.stack((xx, qq), -1).reshape(-1, 2))
        specimens = []
        for item in layout['instances']:
            points, normals, length = path_samples(item['main_path_xq_m'], [.2, .5, .8])
            for index, (point, normal) in enumerate(zip(points, normals)):
                specimens.append(dict(crack_id=item['id'], fraction=[.2,.5,.8][index],
                    body_width_mm=item['body_width_mm'], centre_true_xq_m=point.tolist(),
                    normal=normal.tolist(), kind=item['composition']))
        longest = max(layout['instances'], key=lambda v: v['main_length_m'])
        chain, chain_normals, length = path_samples(longest['main_path_xq_m'], np.linspace(.001,.999,1001))
        centres = np.asarray([v['centre_true_xq_m'] for v in specimens])
        width_normals = np.asarray([v['normal'] for v in specimens])
        half_width = np.asarray([v['body_width_mm']*.0005 for v in specimens])
        edges = np.stack((centres-width_normals*half_width[:,None],
                          centres+width_normals*half_width[:,None]), axis=1)
        target = np.vstack((centres, chain, edges.reshape(-1,2)))
        profile_count = len(specimens)+len(chain)
        initial = np.asarray(seed(target))
        rows, truth = session.evaluation('row_truth'), session.truth()
        truth_path = session.root/'evaluation/truth.json'
        if sha256_file(truth_path) != session.summary['files']['evaluation/truth.json']:
            raise ValueError('archived defect evaluation truth was changed')
        mesh = OpticalMesh.from_session(session)
        camera = session.config()['camera']
        calls = [0]
        def trace(location):
            calls[0] += 1
            return map_points(model, coefficients, location[:,0], location[:,1],
                              rows, camera, truth, mesh, workers)
        location, ok, remaining, _, band = locate(target, initial, trace)
        if not ok.any():
            raise ValueError('no independently located defect specimens')
        normals = np.vstack(([v['normal'] for v in specimens], chain_normals))
        offsets = np.linspace(-.002, .002, 81)
        positions = location[:profile_count,None,:]+offsets[None,:,None]*normals[:,None,:]
        profiles = saved_samples(image, upstream['grid'], positions)
        profiles[~ok[:profile_count]] = np.nan
        measures = [intensity_width(offsets, v) for v in profiles]
        for i, item in enumerate(specimens):
            start = profile_count+2*i
            edge_ok = bool(ok[start:start+2].all())
            item.update(output_xq_m=location[i].tolist(), source_band=int(band[i]),
                        location_error_m=float(remaining[i]) if np.isfinite(remaining[i]) else None,
                        intensity=measures[i],
                        geometric_body_strip=dict(status='measured' if edge_ok else 'unmeasurable',
                            output_width_mm=float(np.linalg.norm(location[start+1]-location[start])*1000) if edge_ok else None,
                            same_source_band=bool(band[start]==band[start+1]) if edge_ok else None))
        chain_measure = measures[len(specimens):]
        measured = [v for v in specimens if v['intensity']['status'] == 'measured']
        report = dict(schema='ssb.defect_preservation_evaluation.v1', evaluation_only=True,
            status='diagnostic', specimens=specimens,
            widths=dict(planned=len(specimens), measured=len(measured), unmeasurable=len(specimens)-len(measured),
                actual_body_width_range_mm=[min(v['body_width_mm'] for v in specimens), max(v['body_width_mm'] for v in specimens)],
                apparent_fwhm_minus_body_mm=[v['intensity']['apparent_fwhm_mm']-v['body_width_mm'] for v in measured]),
            long_crack=dict(id=longest['id'], main_length_m=length, longitudinal_span_m=longest['longitudinal_span_m'],
                samples=len(chain), cadence_along_path_m=length*.998/1000,
                measured=sum(v['status']=='measured' for v in chain_measure),
                unmeasurable=sum(v['status']!='measured' for v in chain_measure),
                source_band_transitions=int(np.count_nonzero(np.diff(band[len(specimens):profile_count]))),
                measures=chain_measure),
            location=dict(method='evaluation-only inversion of independently traced native footprints',
                          tolerance_m=1e-5, iterations=calls[0]-1, final_location_retraced=True),
            limitations=['Truth only locates specimens; no saved or production image is corrected.',
                'Apparent intensity FWHM is not a physical subpixel crack-width measurement.',
                'Geometric body strips measure the mapping of specified normal offsets, not the union of intersecting crack branches.',
                'Profiles use the nominal output normal; whole-shape deformation is audited separately.',
                'Long-crack checks sample at about 1 cm, not exhaustive pixel connectivity.',
                'The existing layout does not contain exact 0.2 and 0.6 mm boundary specimens.',
                'Low contrast, intersections and missing support remain explicit, never counted as pass.'])
        output.mkdir(parents=True)
        np.savez_compressed(output/'profiles.npz', offsets_m=offsets, values_dn=profiles,
                            true_centres_xq_m=target, output_centres_xq_m=location, source_band=band, located=ok)
        (output/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
        manifest_path = session.root/'evaluation/manifest.json'
        truth_rows_path = session.root/'evaluation'/read_json(manifest_path)['row_truth']['file']
        inputs = [scene_path, defect_path, image_path, mosaic/'report.json', provenance_path,
                  geometry/'mapping_samples.npz', session.root/'evaluation/truth.json', *public_inputs,
                  manifest_path, truth_rows_path, *fit_inputs, *mesh.sources]
        provenance = stage_record('defect_preservation_evaluation', inputs, sorted(output.iterdir()),
                                  dict(width_fractions=[.2,.5,.8], long_samples=1001, minimum_contrast_dn=5.))
        (output/'provenance.json').write_text(json.dumps(provenance, indent=2)+'\n')
        return report
    finally:
        sampler.native.release()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    for name in ('session', 'unroll', 'trajectory', 'mosaic', 'geometry', 'output'):
        p.add_argument('--'+name, required=True)
    p.add_argument('--workers', type=int, default=4)
    args = p.parse_args()
    result = run(args.session, args.unroll, args.trajectory, args.mosaic, args.geometry, args.output, args.workers)
    print(json.dumps(dict(planned=result['widths']['planned'], measured=result['widths']['measured'],
                          long_measured=result['long_crack']['measured'])))


if __name__ == '__main__':
    main()
