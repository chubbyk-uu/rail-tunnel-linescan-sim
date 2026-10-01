"""Bounded, uncorrected thumbnails of immutable raw blocks, for observation only."""
import base64
import io
import mmap
from pathlib import Path
import re

from PIL import Image


class RawImagePreview:
    def __init__(self):
        self.key = None
        self.cached = None

    def sample(self, session, width, block_rows, enabled=True):
        output = str(session or '')
        empty = dict(schema='ssb.raw_preview.v1', output=output,
                     status='waiting' if enabled else 'disabled')
        if not enabled or not session:
            self.key = self.cached = None
            return empty
        raw = Path(session)/'raw'
        blocks = [p for p in raw.glob('block_*.u8')
                  if re.fullmatch(r'block_[0-9]{6}\.u8', p.name)]
        if not blocks:
            self.key = self.cached = None
            return empty
        # The writer atomically renames a block only after read-back hash verification.
        # Never read .tmp or buffered rows, and do not require the final session index.
        block = max(blocks, key=lambda p: p.name)
        size = block.stat().st_size
        if width <= 0 or block_rows <= 0 or not size or size % width or size > width*block_rows:
            raise ValueError('Invalid raw block size for image preview')
        key = (output, block.name, size, width, block_rows)
        if key == self.key:
            return self.cached
        rows = size//width
        with block.open('rb') as file, mmap.mmap(file.fileno(), 0, access=mmap.ACCESS_READ) as data:
            with Image.frombuffer('L', (width, rows), data, 'raw', 'L', 0, 1) as image:
                # Area averaging only: no flat-field, geometry, contrast or sharpening.
                image.thumbnail((512, 512), Image.Resampling.BOX)
                stream = io.BytesIO()
                image.save(stream, format='PNG', compress_level=1)
                preview_size = list(image.size)
        first = int(block.stem.split('_')[1])*block_rows
        self.cached = dict(empty, status='ready', block=block.name,
                           first_row=first, last_row=first+rows-1,
                           source_size=[width, rows], preview_size=preview_size,
                           encoding='png;base64', png=base64.b64encode(stream.getvalue()).decode('ascii'))
        self.key = key
        return self.cached
