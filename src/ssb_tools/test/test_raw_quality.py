import json
import numpy as np
import pytest

from ssb_tools.raw_quality import inspect
from ssb_tools.session import sha256_file


def capture(root, saturated=False):
    root.mkdir(); (root/'raw').mkdir()
    values=np.tile(np.array([3,40,180,230],np.uint8),(1500,1))
    if saturated: values[1200,2]=255
    values.tofile(root/'raw/block.u8')
    index=dict(width=4,blocks=[dict(file='block.u8',first_sequence=0,rows=len(values),
                                   sha256=sha256_file(root/'raw/block.u8'))])
    (root/'raw/index.json').write_text(json.dumps(index))
    (root/'session.json').write_text(json.dumps(dict(status='complete',rows=len(values),
        files={'raw/index.json':sha256_file(root/'raw/index.json')})))
    return values


@pytest.mark.parametrize('saturated',[False,True])
def test_streaming_statistics_cover_all_pixels_across_read_boundaries(tmp_path,saturated):
    root=tmp_path/'session';values=capture(root,saturated)
    report=inspect(root,tmp_path/'quality')
    np.testing.assert_array_equal(report['histogram'],np.bincount(values.ravel(),minlength=256))
    assert report['saturated_pixels']==int(saturated) and report['saturated_rows']==int(saturated)
    assert report['status']==('fail' if saturated else 'pass')
    assert report['pixels']==values.size and report['maximum_dn']==(255 if saturated else 230)
    # No generation-side or truth data exists in this independent fixture.
    assert not (root/'evaluation').exists()


def test_corrupted_pixels_cannot_produce_a_quality_report(tmp_path):
    root=tmp_path/'session';capture(root)
    with (root/'raw/block.u8').open('r+b') as file: file.write(b'\x01')
    with pytest.raises(ValueError,match='hash mismatch'):inspect(root,tmp_path/'quality')
    assert not (tmp_path/'quality').exists()


def test_raw_quality_rejects_index_redirection_and_gaps(tmp_path):
    root=tmp_path/'session';capture(root)
    path=root/'raw/index.json';index=json.loads(path.read_text())
    for key,value,expected in [('file','../../outside.u8','escapes'),('first_sequence',1,'gap')]:
        changed=json.loads(json.dumps(index));changed['blocks'][0][key]=value
        path.write_text(json.dumps(changed))
        summary=json.loads((root/'session.json').read_text())
        summary['files']['raw/index.json']=sha256_file(path)
        (root/'session.json').write_text(json.dumps(summary))
        with pytest.raises(ValueError,match=expected):inspect(root,tmp_path/'quality')
