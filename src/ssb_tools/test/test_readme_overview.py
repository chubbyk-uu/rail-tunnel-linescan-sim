import importlib.util
import json
from pathlib import Path
import numpy as np
from PIL import Image
import pytest
from ssb_tools.session import sha256_file
spec=importlib.util.spec_from_file_location('readme_export',Path(__file__).resolve().parents[3]/'tools'/'export_readme_summary.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


def fixture(root):
    root.mkdir();(root/'geometry').mkdir()
    data=np.zeros((100,1800,3),np.uint8);data[:,:,0]=np.arange(1800)[None,:]%256
    for file in ('raw_helix.png','geometry/optimized.png'):
        Image.fromarray(data).save(root/file)
    for kind in (0,1):
        for name in ('raw','optimized'):
            Image.fromarray(np.full((512,512,3),40+kind,np.uint8)).save(root/f'feature_{kind}_{name}.png')
    report=dict(schema='ssb.feature_review.v2',grid=dict(target_x_m=[0,20],radius_m=2.75,
        theta_rad=[-.01,.01],dx_m=20/1800,dq_m=.00055),preview_stride=1,
        crops=[dict(id=i,candidate=dict(kind=kind,x_m=10.,q_m=0.),q_first_m=-.0512) for i,kind in enumerate(('wide','thin'))])
    (root/'report.json').write_text(json.dumps(report))
    outputs={str(p.resolve()):sha256_file(p) for p in root.rglob('*') if p.is_file()}
    (root/'provenance.json').write_text(json.dumps(dict(stage='feature_review',outputs=outputs)))


def test_overview_crop_keeps_original_acceptance_target_and_exact_source_pixels(tmp_path):
    source=tmp_path/'source';fixture(source)
    output=tmp_path/'out';module.export(source,output,[8.,11.])
    report=json.loads((output/'report.json').read_text())
    assert report['target_x_m']==[0,20] and report['overview_x_m']==[8,11]
    assert report['source_pixels_identical']
    assert report['panels'][0]['source_box']==[720,0,990,100]


@pytest.mark.parametrize('bounds',[(11,8),(-1,1),(19,21),(np.nan,10),(8,8),(8,8.00001)])
def test_invalid_or_empty_overview_does_not_create_an_output(tmp_path,bounds):
    source=tmp_path/'source';fixture(source)
    output=tmp_path/'out'
    with pytest.raises(ValueError):module.export(source,output,bounds)
    assert not output.exists()


def test_large_default_overview_is_rejected_instead_of_clipping_panels(tmp_path):
    source=tmp_path/'source';fixture(source)
    output=tmp_path/'out'
    with pytest.raises(ValueError,match='512'):module.export(source,output)
    assert not output.exists()
