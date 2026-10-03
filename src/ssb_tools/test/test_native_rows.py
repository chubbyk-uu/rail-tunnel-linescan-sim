"""Raw-image verification and mapping resources stay bounded for long captures."""
import json
from pathlib import Path
import subprocess
import sys

import numpy as np

from ssb_tools.native_rows import NativeRows
from ssb_tools.session import sha256_file


def raw_fixture(root, count=80):
    blocks = []
    pixels = np.arange(count * 2, dtype=np.uint8).reshape(count, 2)
    for i, row in enumerate(pixels):
        path = root / f'block_{i:03}.u8'
        path.write_bytes(row.tobytes())
        blocks.append(dict(file=path.name, first_sequence=i, rows=1,
                           sha256=sha256_file(path)))
    flat = dict(offset=[0., 0.], gain=[1., 1.], valid=[True, True])
    return blocks, pixels, flat


def test_verification_does_not_hold_open_raw_mappings(tmp_path):
    blocks, pixels, flat = raw_fixture(tmp_path, count=8)
    native = NativeRows(tmp_path, blocks, np.arange(8), 2, flat)
    native.verify_all()
    assert not native.maps
    np.testing.assert_array_equal(native.rows(np.arange(8)), pixels.astype(np.float32))


def test_mapping_cache_evicts_the_least_recent_block_without_rehashing(tmp_path, monkeypatch):
    import ssb_tools.native_rows as module
    blocks, pixels, flat = raw_fixture(tmp_path, count=4)
    hashes = []
    original = module.sha256_file

    def counted_hash(path):
        hashes.append(Path(path).name)
        return original(path)

    monkeypatch.setattr(module, 'sha256_file', counted_hash)
    native = NativeRows(tmp_path, blocks, np.arange(4), 2, flat, max_open_blocks=2)
    native.verify_all()
    native.raw([0, 1])
    recent = native.maps[0]
    evicted = native.maps[1]
    native.raw([0])
    native.raw([2])
    assert set(native.maps) == {0, 2} and native.maps[0] is recent
    assert evicted._mmap.closed
    np.testing.assert_array_equal(native.raw([1, 0, 3, 2]), pixels[[1, 0, 3, 2]])
    assert len(native.maps) <= 2 and len(hashes) == 4
    mappings = list(native.maps.values())
    native.close()
    native.close()
    assert not native.maps and all(array._mmap.closed for array in mappings)
    np.testing.assert_array_equal(native.rows([0]), pixels[[0]].astype(np.float32))
    native.close()


def test_long_capture_reads_under_a_small_file_descriptor_limit(tmp_path):
    blocks, _, flat = raw_fixture(tmp_path)
    manifest = tmp_path / 'fixture.json'
    manifest.write_text(json.dumps(dict(blocks=blocks, flat=flat)))
    # Isolate RLIMIT_NOFILE from pytest and other tests. Imports and manifest loading
    # happen before reducing the limit, just as in an already running ROS process.
    script = '''
import json, os, resource, sys
from pathlib import Path
import numpy as np
from ssb_tools.native_rows import NativeRows
root = Path(sys.argv[1])
fixture = json.loads((root/'fixture.json').read_text())
soft, hard = resource.getrlimit(resource.RLIMIT_NOFILE)
resource.setrlimit(resource.RLIMIT_NOFILE, (min(64, soft) if soft >= 0 else 64, hard))
before = len(os.listdir('/proc/self/fd'))
native = NativeRows(root, fixture['blocks'], np.arange(80), 2, fixture['flat'])
native.verify_all()
assert not native.maps
for ids in (np.arange(80), np.arange(79, -1, -1)):
    expected = np.column_stack([2*ids, 2*ids+1]).astype(np.uint8)
    np.testing.assert_array_equal(native.raw(ids), expected)
assert len(native.maps) <= 32
native.release()
native.close()
assert len(os.listdir('/proc/self/fd')) == before
print('80 blocks read in both directions with RLIMIT_NOFILE=64')
'''
    result = subprocess.run([sys.executable, '-c', script, str(tmp_path)],
                            text=True, capture_output=True)
    assert result.returncode == 0, result.stdout + result.stderr


def test_public_results_own_memory_after_eviction_and_close(tmp_path):
    blocks, pixels, flat = raw_fixture(tmp_path, count=3)
    native = NativeRows(tmp_path, blocks, np.arange(3), 2, flat, max_open_blocks=1)
    raw = native.raw([0])
    corrected = native.rows([0])
    gathered = native.gather([0], np.array([[0, 1]]))
    borrowed = native.maps[0]
    for array in (raw, corrected, gathered):
        assert array.flags.owndata and not np.shares_memory(array, borrowed)
    native.raw([1, 2])  # evicts block 0 and then block 1
    assert borrowed._mmap.closed
    native.close()
    for array in (raw, corrected, gathered):
        np.testing.assert_array_equal(array, pixels[[0]])
