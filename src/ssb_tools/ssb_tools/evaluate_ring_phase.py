"""Evaluation-only ring-phase diagnostics; never a D2/D3 input or selector."""
import argparse
from collections import Counter
import json
import math
from pathlib import Path

import numpy as np

from .provenance import stage_record
from .session import read_json, sha256_file


def ring_windows(windows, pitch_m, period_m, origin_m=0.):
    if not all(math.isfinite(x) for x in (pitch_m, period_m, origin_m)) or min(pitch_m, period_m) <= 0:
        raise ValueError('finite ring origin and positive grid pitch/period required')
    selected = set()
    for window in windows:
        if 'shape' not in window:
            continue
        left = window['x_first_m']
        right = left+(window['shape'][1]-1)*pitch_m
        if not math.isfinite(left) or not math.isfinite(right) or right < left:
            raise ValueError('invalid window geometry')
        if math.ceil((left-origin_m)/period_m) <= math.floor((right-origin_m)/period_m):
            selected.add(window['id'])
    return selected


def phase_summary(windows, pitch_m, period_m, origin_m=0.):
    ids = ring_windows(windows, pitch_m, period_m, origin_m)
    groups = {}
    for name, on_ring in (('ring_crossing', True), ('other', False)):
        group = [w for w in windows if 'shape' in w and ((w['id'] in ids) == on_ring)]
        rejected = [w for w in group if w['status'] != 'accepted']
        checks = Counter(k for w in rejected for k, ok in w.get('support_checks', {}).items() if not ok)
        groups[name] = dict(windows=len(group), accepted=len(group)-len(rejected), rejected=len(rejected),
            rejection_reasons=dict(Counter(w.get('reason') for w in rejected)),
            failed_support_checks=dict(checks),
            rejected_without_detailed_checks=sum('support_checks' not in w for w in rejected))
    return dict(planned=len(windows), eligible=sum('shape' in w for w in windows),
        ring_crossing=len(ids), fraction_of_all_windows=len(ids)/max(1,len(windows)), groups=groups)


def signed_group(scores, selected, pitch_m):
    if not selected.any():
        return dict(status='unmeasurable', matches=0)
    delta = np.column_stack((scores['mesh_dx_m'][selected], scores['mesh_dq_m'][selected]))/pitch_m
    if not np.isfinite(delta).all():
        raise ValueError('nonfinite mesh correspondence score')
    return dict(status='measured', matches=len(delta), mean_px=delta.mean(0).tolist(),
        std_px=delta.std(0).tolist(), norm_p95_px=float(np.percentile(np.linalg.norm(delta,axis=1),95)))


def run(matches, output, period_m=1.2, origin_m=0., correspondence_scores=None):
    matches, output = Path(matches).resolve(), Path(output).resolve()
    if 'evaluation' not in output.parts or output.is_relative_to(matches):
        raise ValueError('phase diagnostics must be separate, inside evaluation/')
    inputs = [matches/name for name in ('windows.json','report.json','provenance.json')]
    provenance = read_json(inputs[-1])
    if provenance.get('stage') != 'band_matching':
        raise ValueError('verified D2 products required')
    for path in inputs[:2]:
        identities = [v for k,v in provenance['outputs'].items() if Path(k).name == path.name]
        if identities != [sha256_file(path)]:
            raise ValueError('D2 product identity mismatch')
    windows = read_json(inputs[0]); report = read_json(inputs[1])
    pitch = report['upstream_grid']['dx_m']
    result = dict(schema='ssb.ring_phase_evaluation.v1', evaluation_only=True,
        ring_period_m=period_m, ring_origin_m=origin_m,
        definition='known ring centre intersects nominal window; evaluation labels only',
        phase=phase_summary(windows,pitch,period_m,origin_m))
    if correspondence_scores:
        path=Path(correspondence_scores).resolve()
        if 'evaluation' not in path.parts:
            raise ValueError('independent correspondence scores must be in evaluation/')
        record=read_json(path.parent/'provenance.json')
        identity=[v for k,v in record['outputs'].items() if Path(k).name==path.name]
        upstream=[v for k,v in record['inputs'].items() if Path(k).name=='matches.npy']
        if identity != [sha256_file(path)] or upstream != [sha256_file(matches/'matches.npy')]:
            raise ValueError('correspondence scores do not identify these matches')
        scores=np.load(path,allow_pickle=False)
        ids=ring_windows(windows,pitch,period_m,origin_m)
        selected=np.isin(scores['window'],list(ids))
        result['accepted_correspondences']={
            'ring_windows':signed_group(scores,selected,pitch),
            'other_windows':signed_group(scores,~selected,pitch)}
        # Material identities come only from the independent optical-mesh evaluator.
        result['accepted_correspondences']['ring_windows_by_material_pair']={
            f'{int(a)},{int(b)}':signed_group(scores,selected & (scores['material_a']==a) &
                (scores['material_b']==b),pitch)
            for a,b in np.unique(np.column_stack((scores['material_a'][selected],scores['material_b'][selected])),axis=0)}
        inputs += [path,path.parent/'provenance.json',matches/'matches.npy']
    output.mkdir(parents=True,exist_ok=False)
    target=output/'report.json'; target.write_text(json.dumps(result,indent=2)+'\n')
    (output/'provenance.json').write_text(json.dumps(stage_record('ring_phase_evaluation',inputs,[target],
        dict(period_m=period_m,origin_m=origin_m)),indent=2)+'\n')
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--matches',required=True); parser.add_argument('--output',required=True)
    parser.add_argument('--period-m',type=float,default=1.2)
    parser.add_argument('--origin-m',type=float,default=0.)
    parser.add_argument('--correspondence-scores')
    result=run(**vars(parser.parse_args()))
    print(json.dumps(result['phase']))


if __name__ == '__main__':
    main()
