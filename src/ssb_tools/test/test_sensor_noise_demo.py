import json
import shutil
from pathlib import Path
import numpy as np
import pytest
import yaml
from ssb_tools.sensor_noise_demo import prepare
from ssb_tools.optical_bench import prepare as prepare_bench
from ssb_tools.package_paths import share_file
from ssb_tools.session import sha256_file
from ssb_tools.optical_calibration import flat_field,flat_correct


def profile():
    return dict(model='shot_read_prnu_v1',enabled=True,pattern_seed=71,realization_seed=82,
                electrons_per_dn=17,read_noise_e=7,dark_current_e_per_s=83,bias_dn=6,prnu_fraction=.004)


def bundle(root):
    root.mkdir();(root/'assets').mkdir();(root/'world').mkdir()
    config=yaml.safe_load(share_file('config/stage_b.yaml','ssb_core').read_text())
    config['truth']['optical_key']='a'*64
    config['render']['optical_scene']='assets/scene.json'
    (root/'capture.yaml').write_text(yaml.safe_dump(config))
    (root/'assets/scene.json').write_text(json.dumps(dict(sampling={},meshes=[],lamp={},response_gain=2.4)))
    (root/'world/world.sdf').write_text('<sdf><world/></sdf>')
    (root/'calibration.json').write_text('{"old":true}')
    (root/'gui.config').write_text('<gui/>')
    files={str(p.relative_to(root)):sha256_file(p) for p in root.rglob('*') if p.is_file()}
    (root/'bundle.json').write_text(json.dumps(dict(schema='ssb.demo_bundle.v1',files=files)))
    return files


def test_noise_variant_preserves_baseline_and_is_independent_of_its_directory(tmp_path):
    original=tmp_path/'original';files=bundle(original);output=tmp_path/'variant'
    config=prepare(original,output,profile())
    assert config['truth']['sensor_noise']==profile()
    assert not (output/'calibration.json').exists() and not (output/'bundle.json').exists()
    assert (output/'assets/scene.json').stat().st_ino==(original/'assets/scene.json').stat().st_ino
    assert (output/'capture.yaml').stat().st_ino!=(original/'capture.yaml').stat().st_ino
    assert all(sha256_file(original/name)==digest for name,digest in files.items())
    shutil.rmtree(original)
    copied=yaml.safe_load((output/'capture.yaml').read_text())
    assert (output/copied['render']['optical_scene']).exists()
    assert json.loads((output/'assets/scene.json').read_text())['response_gain']==2.4
    with pytest.raises(ValueError,match='fresh'):prepare(output,output,profile())


def test_noise_bench_has_independent_temporal_realizations_and_same_fixed_columns(tmp_path):
    original=tmp_path/'original';bundle(original);output=tmp_path/'variant'
    prepare(original,output,profile())
    meta=prepare_bench(output/'capture.yaml',output/'bench')
    seeds=[]
    for target in meta['targets'].values():
        config=yaml.safe_load((output/'bench'/target['config']).read_text())
        noise=config['truth']['sensor_noise']
        seeds.append(noise['realization_seed'])
        assert noise['pattern_seed']==71 and config['truth']['optical_key']=='a'*64
    assert len(set(seeds))==5 and 82 not in seeds
    assert meta['sensor_noise_enabled']
    assert 'realization_seed' not in json.dumps(meta) and 'electrons_per_dn' not in json.dumps(meta)


def test_noise_calibration_uses_independent_fixture_for_coupled_nonzero_mount(tmp_path,monkeypatch):
    from ssb_tools import sensor_noise_demo as module
    original=tmp_path/'original';bundle(original)
    config=yaml.safe_load((original/'capture.yaml').read_text())
    config['truth']['mount'].update(dy_m=.02,dz_m=-.02,tilt_y_rad=.001,tilt_z_rad=-.001)
    (original/'capture.yaml').write_text(yaml.safe_dump(config))
    manifest=json.loads((original/'bundle.json').read_text())
    manifest['files']['capture.yaml']=sha256_file(original/'capture.yaml')
    (original/'bundle.json').write_text(json.dumps(manifest))
    output=tmp_path/'variant';prepare(original,output,profile());commands=[]
    def render(command,**kwargs):
        target=yaml.safe_load(Path(command[command.index('--config')+1]).read_text())
        assert target['truth']['mount']==config['truth']['mount']
        assert command[command.index('--rows')+1]=='512'
        assert '--speed' not in command and '--omega' not in command
        commands.append(command)
    def fit(bench,path):
        assert json.loads(Path(bench).read_text())['centered_bench'] is True
        Path(path).write_text('{}')
        return dict(validation={})
    monkeypatch.setattr(module.subprocess,'run',render)
    monkeypatch.setattr(module,'fit',fit)
    monkeypatch.setattr('ssb_tools.optical_identity.check_calibration',lambda *a:None)
    module.calibrate(output,512)
    assert len(commands)==5
    assert all('--centered-bench' in command for command in commands)
    assert json.loads((output/'preparation.json').read_text())['status']=='complete'


@pytest.mark.parametrize('key,value',[('prnu_fraction',float('nan')),('realization_seed',-1),('read_noise_e',-1),('bias_dn',True)])
def test_invalid_noise_profile_is_rejected_before_copying(tmp_path,key,value):
    noise=profile();noise[key]=value
    with pytest.raises(ValueError):prepare(tmp_path/'nonexistent',tmp_path/'output',noise)
    assert not (tmp_path/'output').exists()


def test_measured_flat_removes_fixed_columns_without_removing_independent_temporal_noise():
    rng=np.random.default_rng(550)
    response=np.exp(rng.normal(0,.01,128)-.00005)
    dark=np.clip(np.rint(rng.normal(4,.4,(512,128))),0,255).astype(np.uint8)
    bright=np.clip(np.rint(rng.normal(4+110*response,np.sqrt(110*response/20+.16),(512,128))),0,255).astype(np.uint8)
    heldout=np.clip(np.rint(rng.normal(4+95*response,np.sqrt(95*response/20+.16),(512,128))),0,255).astype(np.uint8)
    measured=flat_field(dark,bright)
    corrected,valid=flat_correct(heldout,measured)
    assert valid.all()
    assert corrected.mean(0).std()/corrected.mean()<.002
    assert np.median(corrected.std(0))>1.5
    assert np.median(measured['offset'])==pytest.approx(4,abs=.06)
