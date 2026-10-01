import base64
import hashlib
import io

import numpy as np
from PIL import Image
import pytest

from ssb_tools.mission_image_preview import RawImagePreview


def decode(data):
    return Image.open(io.BytesIO(base64.b64decode(data['png'])))


def test_latest_atomic_block_tail_rows_and_raw_pixels_are_preserved(tmp_path):
    raw = tmp_path/'raw'; raw.mkdir()
    (raw/'block_000000.u8').write_bytes(bytes([20])*64*8)
    pixels = np.arange(192, dtype='u1').reshape(3, 64)
    block = raw/'block_000001.u8'; block.write_bytes(pixels.tobytes())
    (raw/'block_000002.u8.tmp').write_bytes(bytes([255])*64*8)
    before = hashlib.sha256(block.read_bytes()).hexdigest()
    data = RawImagePreview().sample(tmp_path, 64, 8)
    assert (data['first_row'], data['last_row']) == (8, 10)
    assert data['source_size'] == [64, 3]
    with decode(data) as image: np.testing.assert_array_equal(np.asarray(image), pixels)
    assert hashlib.sha256(block.read_bytes()).hexdigest() == before


def test_preview_caps_size_and_area_averages_without_contrast_boost(tmp_path):
    raw = tmp_path/'raw'; raw.mkdir()
    pixels = np.tile(np.array([10, 30], dtype='u1'), (1024, 512))
    (raw/'block_000000.u8').write_bytes(pixels.tobytes())
    data = RawImagePreview().sample(tmp_path, 1024, 1024)
    assert data['preview_size'] == [512, 512]
    with decode(data) as image:
        np.testing.assert_array_equal(np.asarray(image), np.full((512, 512), 20, dtype='u1'))
    assert len(data['png']) < 512*1024


def test_unchanged_blocks_reuse_thumbnail_and_new_tasks_cannot_show_stale_image(tmp_path, monkeypatch):
    session = tmp_path/'first'; (session/'raw').mkdir(parents=True)
    (session/'raw/block_000000.u8').write_bytes(bytes([60])*64*8)
    preview = RawImagePreview(); first = preview.sample(session, 64, 8)
    def unexpected_read(*args, **kwargs): raise AssertionError('unchanged raw block was reread')
    monkeypatch.setattr(Image, 'frombuffer', unexpected_read)
    assert preview.sample(session, 64, 8) is first
    waiting = preview.sample(tmp_path/'second', 64, 8)
    assert waiting['status'] == 'waiting' and 'png' not in waiting
    disabled = preview.sample(session, 64, 8, enabled=False)
    assert disabled['status'] == 'disabled' and 'png' not in disabled


def test_malformed_raw_block_is_not_displayed(tmp_path):
    (tmp_path/'raw').mkdir()
    (tmp_path/'raw/block_000000.u8').write_bytes(b'bad size')
    with pytest.raises(ValueError, match='Invalid raw block size'):
        RawImagePreview().sample(tmp_path, 64, 8)
