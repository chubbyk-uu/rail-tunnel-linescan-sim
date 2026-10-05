import numpy as np
import pytest
from ssb_tools.evaluate_band_matches import paired_hits, run, diagnostic_indices
from ssb_tools.provenance import stage_record
from ssb_tools.session import sha256_file
from test_initial_unroll import public_fixture


def test_paired_cylinder_hit_reference_preserves_distinct_ray_origins_and_tangents():
    origins = np.array([[1., .01, -.02], [7., -.02, .03]])
    optical = np.array([[0., .6, .8], [0., -.8, .6]])
    line = np.array([[1., 0., 0.], [1., 0., 0.]])
    tangents = np.array([.07, -.1])
    radius=2.75
    hits=paired_hits(origins,optical,line,tangents,radius,0.)
    directions=optical+tangents[:,None]*line
    # Recover distance from x, then independently check cylinder and q.
    distance=(hits[:,0]-origins[:,0])/directions[:,0]
    points=origins+distance[:,None]*directions
    np.testing.assert_allclose(points[:,1]**2+points[:,2]**2,radius**2,atol=1e-12)
    np.testing.assert_allclose(radius*np.arctan2(points[:,1],points[:,2]),hits[:,1],atol=1e-12)


def test_evaluation_cannot_publish_truth_side_results_outside_evaluation(tmp_path, monkeypatch):
    import ssb_tools.evaluate_band_matches as evaluator
    monkeypatch.setattr(evaluator,'Session',lambda root: object())
    with pytest.raises(ValueError,match='evaluation/'):
        run(tmp_path/'session',tmp_path/'matches',tmp_path/'public_result')


def test_diagnostic_plan_covers_native_column_range_and_ignores_rejected_matches():
    table = np.zeros(13, dtype=[('window', 'i4'), ('inlier', 'i1'), ('a_lower_column', 'f8')])
    table['window'] = [0]*10+[1]*3
    table['inlier'] = 1
    table['inlier'][0] = 0
    table['a_lower_column'] = np.arange(13)[::-1]
    selected = diagnostic_indices(table, 3)
    assert len(selected) == 6 and 0 not in selected
    assert set(selected[:3]) == {1, 5, 9}
    assert set(selected[3:]) == {10, 11, 12}
    np.testing.assert_array_equal(selected, diagnostic_indices(table, 3))


@pytest.mark.parametrize('budget', [0, 1, 65, 2.5, True])
def test_diagnostic_plan_rejects_unbounded_or_invalid_budgets(budget):
    with pytest.raises(ValueError):
        diagnostic_indices(np.empty(0), budget)


def test_diagnostic_plan_rejects_empty_evidence():
    table = np.zeros(2, dtype=[('window', 'i4'), ('inlier', 'i1'), ('a_lower_column', 'f8')])
    with pytest.raises(ValueError, match='no inlier'):
        diagnostic_indices(table, 8)


@pytest.mark.parametrize('kind', ['optical', 'raw_identity', 'tampered_report'])
def test_evaluation_rejects_wrong_source_session_or_tampered_match_report(tmp_path, kind):
    import json
    session=tmp_path/'session'; public_fixture(session)
    matches=tmp_path/'matches'; matches.mkdir()
    np.save(matches/'matches.npy',np.empty(0))
    report=dict(optical_signature='measured',source_observation_hashes={
        name:sha256_file(session/name) for name in
        ('config/observable_config.json','metadata/manifest.json','raw/index.json')})
    expected='optical identity' if kind=='optical' else 'source public observation'
    if kind=='optical':report['optical_signature']='different'
    if kind=='raw_identity':report['source_observation_hashes']['raw/index.json']='0'*64
    path=matches/'report.json';path.write_text(json.dumps(report))
    (matches/'provenance.json').write_text(json.dumps(stage_record('band_matching',[],sorted(matches.iterdir()),{})))
    if kind=='tampered_report':
        report['optical_signature']='changed_after_hash';path.write_text(json.dumps(report));expected='match identity'
    with pytest.raises(ValueError,match=expected):run(session,matches,tmp_path/'evaluation')
    assert not (tmp_path/'evaluation').exists()
