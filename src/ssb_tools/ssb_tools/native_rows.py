"""Native sensor rows for reconstruction, flat-corrected on demand.

D1 no longer stores a float32 copy of every relevant raw row. Rows are read from the
capture's uint8 raw blocks and corrected with the measured dark/flat parameters at the
moment they are sampled, before any spatial interpolation. Values are bit-identical to
the former cache: float32 (raw - offset) * gain, NaN where the column is invalid or the
raw sample is saturated (255).
"""
import mmap
from collections import OrderedDict
from pathlib import Path
import resource
import numpy as np
from .session import sha256_file


def flat_parameters(flat, width):
    offset = np.asarray(flat['offset'], np.float32); gain = np.asarray(flat['gain'], np.float32)
    valid = np.asarray(flat['valid'], bool)
    if offset.shape != (width,) or gain.shape != (width,) or valid.shape != (width,):
        raise ValueError('flat calibration/camera width mismatch')
    if not (np.isfinite(offset).all() and np.isfinite(gain).all()):
        raise ValueError('nonfinite flat calibration')
    return offset, gain, valid


def correct(raw, offset, gain, valid):
    """The single flat-field formula shared by the CPU path, the CUDA kernel and tests."""
    values = (raw.astype(np.float32)-offset)*gain
    values[~(valid[None, :] & (raw != 255))] = np.nan
    return values


class FloatRows:
    """Adapter for an in-memory/legacy float32 native image (D1 v1 caches, tests)."""
    def __init__(self, image):
        self.image = image
        self.shape = image.shape

    def rows(self, ids):
        return np.asarray(self.image[np.asarray(ids)], np.float32)

    def gather(self, ids, *columns):
        image = np.asarray(self.image); ids = np.asarray(ids)[:, None]
        values = [image[ids, c] for c in columns]
        return values[0] if len(values) == 1 else values

    def release(self):
        mapping = getattr(self.image, '_mmap', None)
        if mapping is not None and hasattr(mapping, 'madvise'): mapping.madvise(mmap.MADV_DONTNEED)


class NativeRows:
    """Projection rows backed by hash-verified uint8 raw blocks of one capture.

    Public raw/rows/gather results own their storage. Never return a slice of a
    cached mmap: eviction and close invalidate such views without a Python error.
    """
    def __init__(self, raw_dir, blocks, sequences, width, flat, verified=(), max_open_blocks=128):
        if not isinstance(max_open_blocks, int) or max_open_blocks < 1:
            raise ValueError('positive raw mapping cache capacity required')
        soft_limit, _ = resource.getrlimit(resource.RLIMIT_NOFILE)
        # Leave descriptors for ROS, CUDA, outputs and temporary hash reads. Keep
        # enough blocks for adjacent angular tiles without reopening every tile.
        self.max_open_blocks = max_open_blocks
        if soft_limit != resource.RLIM_INFINITY:
            self.max_open_blocks = min(max_open_blocks, max(1, soft_limit // 2))
        self.raw_dir = Path(raw_dir).resolve(); self.width = int(width)
        self.offset, self.gain, self.valid = flat_parameters(flat, self.width)
        self.blocks = sorted(blocks, key=lambda b: b['first_sequence'])
        self.sequences = np.asarray(sequences, np.int64)
        if self.sequences.ndim != 1 or not len(self.sequences):
            raise ValueError('native rows need a nonempty sequence list')
        for entry in self.blocks:
            if not (self.raw_dir/entry['file']).resolve().is_relative_to(self.raw_dir):
                raise ValueError('raw block escapes the public raw directory: '+str(entry['file']))
        self.first = np.array([b['first_sequence'] for b in self.blocks], np.int64)
        self.count = np.array([b['rows'] for b in self.blocks], np.int64)
        block = np.searchsorted(self.first, self.sequences, side='right')-1
        if np.any(block < 0) or np.any(self.sequences >= self.first[block]+self.count[block]):
            raise ValueError('projection rows are not covered by the recorded raw blocks')
        self.block_of = block; self.local = self.sequences-self.first[block]
        self.shape = (len(self.sequences), self.width)
        self.verified = set(verified); self.maps = OrderedDict()

    def _verify_block(self, index):
        entry = self.blocks[index]
        path = self.raw_dir/entry['file']
        if not path.resolve().is_relative_to(self.raw_dir):
            raise ValueError('raw block escapes the public raw directory: '+str(entry['file']))
        if path.stat().st_size != entry['rows']*self.width:
            raise ValueError('raw block size mismatch: '+str(path))
        if entry['file'] not in self.verified:
            if sha256_file(path) != entry['sha256']:
                raise ValueError('raw block hash mismatch: '+str(path))
            self.verified.add(entry['file'])
        return path

    def _block(self, index):
        """Borrow a mapping internally; no view may escape or survive another lookup."""
        if index not in self.maps:
            path = self._verify_block(index)
            if len(self.maps) >= self.max_open_blocks:
                _, evicted = self.maps.popitem(last=False)
                evicted._mmap.close()
            entry = self.blocks[index]
            self.maps[index] = np.memmap(path, np.uint8, mode='r', shape=(entry['rows'], self.width))
        self.maps.move_to_end(index)
        return self.maps[index]

    def verify_all(self):
        """Hash every recorded block now (fail before any output is created)."""
        for index in range(len(self.blocks)): self._verify_block(index)

    def raw(self, ids):
        """Owned uint8 rows, safe after later reads, cache eviction and close."""
        ids = np.asarray(ids, np.int64)
        out = np.empty((len(ids), self.width), np.uint8)
        blocks = self.block_of[ids]
        for index in np.unique(blocks):
            selected = np.flatnonzero(blocks == index)
            # Advanced indexing copies; out is independently allocated as well.
            # Keep this copy inside the lookup's lifetime, before any eviction.
            out[selected] = self._block(int(index))[self.local[ids[selected]]]
        return out

    def rows(self, ids):
        return correct(self.raw(ids), self.offset, self.gain, self.valid)

    def gather(self, ids, *columns):
        """rows[ids[:, None], c] for each column array, correcting each needed row once."""
        ids = np.asarray(ids, np.int64)
        unique, inverse = np.unique(ids, return_inverse=True)
        rows = self.rows(unique); inverse = inverse.reshape(ids.shape)[:, None]
        values = [rows[inverse, c] for c in columns]
        return values[0] if len(values) == 1 else values

    def release(self):
        for array in self.maps.values():
            mapping = getattr(array, '_mmap', None)
            if mapping is not None and hasattr(mapping, 'madvise'): mapping.madvise(mmap.MADV_DONTNEED)

    def close(self):
        """Close cached descriptors; subsequent reads may reopen verified blocks."""
        while self.maps:
            _, array = self.maps.popitem(last=False)
            array._mmap.close()


class MemoryRows(NativeRows):
    """In-memory uint8 rows with the same correction path (tests and small fixtures)."""
    def __init__(self, raw, flat):
        raw = np.asarray(raw)
        if raw.dtype != np.uint8 or raw.ndim != 2: raise ValueError('uint8 native rows required')
        self.raw_rows = raw; self.width = raw.shape[1]; self.shape = raw.shape
        self.offset, self.gain, self.valid = flat_parameters(flat, self.width)

    def raw(self, ids):
        return self.raw_rows[np.asarray(ids, np.int64)]

    def release(self):
        pass
