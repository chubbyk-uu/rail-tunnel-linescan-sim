import json
import numpy as np
import pytest
from ssb_tools.optical_calibration import flat_field, flat_correct, fit_geometry, correct, correct_session
from ssb_tools.session import sha256_file


def calibration():
    dark = np.full((256, 64), 8, np.uint8)
    bright = np.tile(np.arange(64, dtype=np.uint8)+80, (256, 1))
    flat = flat_field(dark, bright)
    cols = np.arange(2, 63, 4)
    geometry = fit_geometry(cols, (cols-31.5)*.001, 64, .064)
    return dict(optical_signature='test', flat=flat, geometry=geometry), bright


def test_flat_field_removes_column_gain_and_retains_texture():
    c, bright = calibration()
    out, valid = flat_correct(bright, c['flat'])
    assert valid.all()
    assert out.std() < 1e-4
    raw = bright.copy(); raw[:, 30:32] = 8
    out, valid = flat_correct(raw, c['flat'])
    assert valid.all() and (out[:, 30:32] == 0).all()


def test_invalid_flat_field_is_rejected():
    with pytest.raises(ValueError): flat_field(np.zeros((256,64),np.uint8), np.full((256,64),255,np.uint8))
    with pytest.raises(ValueError): flat_field(np.zeros((10,64),np.uint8), np.ones((10,64),np.uint8))


def test_correction_preserves_raw_and_masks_edges_and_saturation():
    c, raw = calibration(); raw[:,30] = 255; before = raw.copy()
    image, valid = correct(raw,c,'test')
    assert image.dtype == np.float32 and np.array_equal(raw,before)
    assert not valid[:,0].any() and not valid[:,30].any()
    assert np.isnan(image[~valid]).all()
    with pytest.raises(ValueError): correct(raw,c,'changed')


def test_geometry_rejects_reversed_target():
    with pytest.raises(ValueError): fit_geometry(np.arange(10), -np.arange(10),64,.064)


def test_streamed_session_is_separate_and_hash_checked(tmp_path):
    c, raw = calibration(); root=tmp_path/'session'; (root/'raw').mkdir(parents=True); (root/'config').mkdir()
    (root/'session.json').write_text('{}')
    (root/'config/observable_config.json').write_text(json.dumps(dict(camera=dict(optical_signature='test'))))
    path=root/'raw/block.bin'; raw.tofile(path); digest=sha256_file(path)
    (root/'raw/index.json').write_text(json.dumps(dict(width=64,blocks=[dict(file='block.bin',rows=256,first_sequence=0,sha256=digest)])))
    cal=tmp_path/'cal.json'; cal.write_text(json.dumps(c))
    out=tmp_path/'corrected'; manifest=correct_session(root,cal,out)
    assert sha256_file(path)==digest and len(manifest['blocks'])==1
    expected,valid=correct(raw,c,'test')
    assert np.allclose(np.load(out/manifest['blocks'][0]['image']),expected,equal_nan=True)
    assert np.array_equal(np.load(out/manifest['blocks'][0]['mask']),valid)
    with pytest.raises(FileExistsError): correct_session(root,cal,out)
