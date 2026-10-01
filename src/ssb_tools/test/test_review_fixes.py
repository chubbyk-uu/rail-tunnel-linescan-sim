import json
import pytest
from ssb_tools.optical_identity import ensure_optical_key
from ssb_tools.validate_stage_b import runtime_source_budget


def test_key_is_random_persistent_and_rejects_invalid_values():
    a, b = {'truth': {}}, {'truth': {}}
    ensure_optical_key(a); ensure_optical_key(b)
    assert a != b
    saved = a['truth']['optical_key']
    ensure_optical_key(a)
    assert a['truth']['optical_key'] == saved and len(saved) == 64
    with pytest.raises(ValueError): ensure_optical_key({'truth': {'optical_key': 'public'}})


@pytest.mark.parametrize('absolute', [False, True])
def test_runtime_budget_resolves_against_scene_not_cwd(tmp_path, monkeypatch, absolute):
    assets = tmp_path/'assets'; assets.mkdir()
    surface = assets/'surface.json'
    surface.write_text(json.dumps({'resources': {'gpu_source_budget_bytes': 1234}}))
    elsewhere = tmp_path/'elsewhere'; elsewhere.mkdir(); monkeypatch.chdir(elsewhere)
    scene = {'surface': {'file': str(surface) if absolute else 'surface.json'}}
    assert runtime_source_budget(scene, assets/'scene.json') == 1234
