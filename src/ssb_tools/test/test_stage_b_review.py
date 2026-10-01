import numpy as np
import pytest

from ssb_tools.stage_b_review import display_rgb, locate_patch, read_corrected_crop, write_html


def test_review_page_requires_explicit_user_acceptance(tmp_path):
    report = dict(examples=[], human_review='pending')
    write_html(tmp_path, report)
    assert '人工确认待完成' in (tmp_path/'index.html').read_text()
    report['human_review'] = dict(status='accepted_by_user')
    write_html(tmp_path, report)
    page = (tmp_path/'index.html').read_text()
    assert '用户已确认当前画质作为后续 20 米采集的基线' in page
    assert '人工确认待完成' not in page


def test_display_preserves_input_and_marks_invalid_without_contrast_stretch():
    image = np.array([[100., 300., np.nan]], np.float32)
    before = image.copy(); valid = np.array([[True, True, False]])
    linear = display_rgb(image, valid)
    assert linear[0].tolist() == [[100, 100, 100], [255, 255, 255], [255, 0, 255]]
    assert display_rgb(image, valid, srgb=True)[0, 0, 0] > 100
    assert np.array_equal(image, before, equal_nan=True)


def test_crop_at_gate_boundary_stays_in_one_scan_segment():
    rows = np.zeros(2048, dtype=[('segment', 'i8')]); rows['segment'][1024:] = 1
    truth = np.zeros(2048, dtype=[('theta', 'f8'), ('x', 'f8')])
    truth['theta'] = np.tile(np.linspace(-.01, .01, 1024), 2)
    truth['x'][:1024] = 4.; truth['x'][1024:] = 4.6
    first, left, centre = locate_patch(rows, truth, 4., .01*2.75, 2.75, .85, 4096)
    assert centre == 1023 and first <= centre < first+512
    assert np.all(rows['segment'][first:first+512] == 0) and 0 <= left <= 4096-512
    with pytest.raises(ValueError, match='not covered'):
        locate_patch(rows, truth, 100., 0., 2.75, .85, 4096)


def test_corrected_crop_crosses_storage_blocks_and_rejects_missing_rows(tmp_path):
    blocks = []
    for first in (0, 4):
        image = np.repeat(np.arange(first, first+4, dtype=np.float32)[:, None], 8, axis=1)
        valid = np.ones(image.shape, bool); valid[:, 3] = False; image[:, 3] = np.nan
        np.save(tmp_path/f'{first}.npy', image); np.save(tmp_path/f'{first}_valid.npy', valid)
        blocks.append(dict(first_sequence=first, rows=4, image=f'{first}.npy', mask=f'{first}_valid.npy'))
    image, valid = read_corrected_crop(tmp_path, {'blocks': blocks}, 2, 4, 2, 3)
    np.testing.assert_array_equal(image[:, 0], np.arange(2, 6))
    assert np.isnan(image[:, 1]).all() and not valid[:, 1].any()
    with pytest.raises(ValueError, match='missing rows'):
        read_corrected_crop(tmp_path, {'blocks': blocks[:1]}, 2, 4, 2, 3)
