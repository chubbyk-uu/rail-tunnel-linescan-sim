import copy
import json
import shutil
from pathlib import Path
import xml.etree.ElementTree as ET
from types import SimpleNamespace

import numpy as np
import pytest
import yaml
from PIL import Image

from ssb_tools.physical_world import check
from ssb_tools.stage_b_scene import load_spec, make_world
from ssb_tools.demo_bundle import export_demo
from ssb_tools.session import sha256_file
from ssb_tools.validate_contact import physical_world_report
from ssb_tools.validate_stage_b import gazebo_world_checks

ROOT = Path(__file__).resolve().parents[3]


@pytest.fixture(scope='module')
def built(tmp_path_factory):
    """A generated contact world with 2 + 2 mm rail irregularity, sprung and measuring wheels."""
    config = yaml.safe_load((ROOT/'src/ssb_core/config/stage_b.yaml').read_text())
    spec = load_spec(ROOT/'src/ssb_tools/config/stage_b_scene.yaml')
    config['tunnel'].update(x_min_m=0., x_max_m=12.)
    config['robot'] = {'base_reference_z_m': .3, 'scan_axis_height_m': 1.715}
    config['contact'] = {'enabled': True, 'settle_s': .5}
    config['motion']['start_x_m'] = 3.
    for section in ('truth', 'calibration'):
        config[section].update(odo_left_diameter_m=.08, odo_right_diameter_m=.08)
    config['truth']['track_irregularity'] = dict(model='beijing_subway_vertical_v1', chord10_max_m=.002,
                                                 cross_level_tier_m=.002, seed=20261001, band_m=[.5, 10.],
                                                 common_mode=False)
    config['truth']['wheel_compliance'] = dict(static_deflection_m=.0002, damping_ratio=.2,
                                               stiffness_n_m=2.9e6, damping_n_s_m=3300.)
    folder = tmp_path_factory.mktemp('world')
    make_world(folder, config, spec)
    return folder, config, spec


def clone(built, tmp_path):
    """Independent copy (world, manifest and heightmaps) so a test may tamper with it."""
    folder, config, spec = built
    copy_dir = tmp_path/'world'
    shutil.copytree(folder, copy_dir)
    world = copy_dir/'world.sdf'
    world.write_text(world.read_text().replace(str(folder.resolve()), str(copy_dir.resolve())))
    return copy_dir, copy.deepcopy(config), spec


def failed(report):
    return {name for name, c in report['checks'].items() if not c['passed']}


def edit_world(folder, change):
    tree = ET.parse(folder/'world.sdf'); change(tree.getroot()); tree.write(folder/'world.sdf')


def test_generated_world_passes_every_check(built, tmp_path):
    folder, config, spec = clone(built, tmp_path)
    report = check(config, spec, folder/'world.sdf')
    assert report['passed'], failed(report)
    assert report['checks']['rail_profile_matches_configuration']['count'] == 2*report['checks']['rail_coverage']['rails']['left']['spans']
    assert report['checks']['rail_overlap_surfaces']['max_difference_m'] < 1e-7   # shared samples, own quantization


def test_missing_rail_is_rejected(built, tmp_path):
    folder, config, spec = clone(built, tmp_path)
    def drop_left(root):
        world = root.find('world')
        for m in list(world.findall('model')):
            if m.get('name').startswith('rail_surface_left_'): world.remove(m)
    edit_world(folder, drop_left)
    assert {'rail_models_complete', 'rail_coverage', 'rail_files_match_manifest'} <= failed(check(config, spec, folder/'world.sdf'))


def test_missing_middle_segment_is_rejected(built, tmp_path):
    folder, config, spec = clone(built, tmp_path)
    edit_world(folder, lambda root: root.find('world').remove(root.find("world/model[@name='rail_surface_right_01']")))
    assert {'rail_models_complete', 'rail_coverage'} <= failed(check(config, spec, folder/'world.sdf'))


@pytest.mark.parametrize('index,delta,expected', [(1, .1, 'rail_placement'), (5, .01, 'rail_placement')])
def test_moved_or_rotated_segment_is_rejected(built, tmp_path, index, delta, expected):
    folder, config, spec = clone(built, tmp_path)
    def move(root):
        pose = root.find("world/model[@name='rail_surface_left_01']/pose")
        values = [float(v) for v in pose.text.split()]; values[index] += delta
        pose.text = ' '.join(map(str, values))
    edit_world(folder, move)
    result = failed(check(config, spec, folder/'world.sdf'))
    assert expected in result and 'rail_files_match_manifest' in result


def test_changed_seed_without_regeneration_is_rejected(built, tmp_path):
    folder, config, spec = clone(built, tmp_path)
    config['truth']['track_irregularity']['seed'] = 7
    assert {'manifest_matches_configuration', 'rail_profile_matches_configuration'} <= failed(check(config, spec, folder/'world.sdf'))


def test_edited_heightmap_image_is_rejected(built, tmp_path):
    folder, config, spec = clone(built, tmp_path)
    path = folder/'track/rail_top_right_02.png'
    data = np.asarray(Image.open(path)).copy(); data[:, 40] = np.minimum(data[:, 40].astype(int)+3000, 65535)
    Image.fromarray(data.astype(np.uint16), mode='I;16').save(path)
    assert {'rail_files_match_manifest', 'rail_profile_matches_configuration'} <= failed(check(config, spec, folder/'world.sdf'))


def test_changed_wheel_diameter_without_regeneration_is_rejected(built, tmp_path):
    folder, config, spec = clone(built, tmp_path)
    config['truth']['odo_left_diameter_m'] = .081
    assert {'manifest_matches_configuration', 'wheel_radii_match_truth'} <= failed(check(config, spec, folder/'world.sdf'))


def test_changed_spring_is_rejected(built, tmp_path):
    folder, config, spec = clone(built, tmp_path)
    edit_world(folder, lambda root: root.find(".//joint[@name='measure_right_slide']/axis/dynamics/spring_stiffness")
               .__setattr__('text', '2500'))
    assert 'springs_match_configuration' in failed(check(config, spec, folder/'world.sdf'))


def test_flat_world_for_an_irregular_configuration_is_rejected(built, tmp_path):
    folder, config, spec = clone(built, tmp_path)
    def drop_rails(root):
        world = root.find('world')
        for m in list(world.findall('model')):
            if m.get('name').startswith('rail_surface_'): world.remove(m)
    edit_world(folder, drop_rails)
    result = failed(check(config, spec, folder/'world.sdf'))
    assert {'rail_models_complete', 'rail_coverage', 'rail_profile_matches_configuration'} <= result


def test_partial_range_heightmap_is_rejected(built, tmp_path):
    # gz stretches each image's own pixel range over the size height: a partial range moves the rail.
    folder, config, spec = clone(built, tmp_path)
    path = folder/'track/rail_top_left_01.png'
    data = np.asarray(Image.open(path)).astype(float)
    Image.fromarray(np.rint(5000+data*.5).astype(np.uint16), mode='I;16').save(path)
    assert 'rail_profile_matches_configuration' in failed(check(config, spec, folder/'world.sdf'))


def test_contact_bundle_survives_relocation_without_source(built, tmp_path):
    folder, config, spec = clone(built, tmp_path)
    # Physics-only miniature demo; remove visual dependencies, retain all contacts.
    tree = ET.parse(folder/'world.sdf')
    for link in tree.getroot().iter('link'):
        for visual in list(link.findall('visual')): link.remove(visual)
    tree.write(folder/'world.sdf')
    demo = tmp_path/'demo'; demo.mkdir(); shutil.move(folder, demo/'world')
    world = demo/'world/world.sdf'
    world.write_text(world.read_text().replace(str(folder), str(demo/'world')))
    config['truth']['optical_key'] = '0'*64
    config['render']['optical_scene'] = 'scene.json'
    (demo/'scene.json').write_text('{}')
    (demo/'capture.yaml').write_text(yaml.safe_dump(config))
    (demo/'spec.yaml').write_text(yaml.safe_dump(spec))
    (demo/'calibration.json').write_text('{}'); (demo/'gui.config').write_text('<gui/>')
    bundle = tmp_path/'bundle'; manifest = export_demo(demo, bundle)
    shutil.rmtree(demo)
    moved = tmp_path/'relocated'; shutil.move(bundle, moved)
    assert check(config, spec, moved/'world/world.sdf')['passed']
    for name, digest in manifest['files'].items():
        assert sha256_file(moved/name) == digest
    packed = json.loads((moved/'world/physical_manifest.json').read_text())
    assert all(r['file'].startswith('0') for r in packed['actual']['rails'])
    assert yaml.safe_load((moved/'spec.yaml').read_text()) == spec


@pytest.mark.parametrize('missing', ['spec.yaml', 'physical_manifest.json', 'world.sdf', 'rail_top_left_00.png'])
def test_physical_snapshot_never_falls_back_to_source(built, tmp_path, missing):
    folder, config, spec = built
    snapshot = tmp_path/'evaluation/physical'; snapshot.mkdir(parents=True)
    for name in ('world.sdf', 'physical_manifest.json'):
        shutil.copyfile(folder/name, snapshot/name)
    for image in (folder/'track').glob('rail_top*.png'):
        shutil.copyfile(image, snapshot/image.name)
    (snapshot/'spec.yaml').write_text(yaml.safe_dump(spec))
    assert physical_world_report(tmp_path, None, config, spec, folder/'world.sdf')['passed']
    (snapshot/missing).unlink()
    with pytest.raises((FileNotFoundError, ValueError)):
        physical_world_report(tmp_path, None, config, spec, folder/'world.sdf')


@pytest.fixture
def archived(built, tmp_path):
    folder, config, spec = built
    snapshot = tmp_path/'evaluation/physical'; snapshot.mkdir(parents=True)
    for name in ('world.sdf', 'physical_manifest.json'):
        shutil.copyfile(folder/name, snapshot/name)
    for image in (folder/'track').glob('rail_top*.png'):
        shutil.copyfile(image, snapshot/image.name)
    (snapshot/'spec.yaml').write_text(yaml.safe_dump(spec))
    summary = {'files': {str(p.relative_to(tmp_path)): sha256_file(p) for p in snapshot.iterdir()}}
    prov = {'pose_source': 'gazebo_contact', 'inputs': {'world': {
        'path': str(tmp_path/'removed_source/world.sdf'), 'sha256': sha256_file(snapshot/'world.sdf')}}}
    return SimpleNamespace(root=tmp_path, summary=summary), copy.deepcopy(config), prov


def test_gazebo_validation_uses_snapshot_without_original_world(archived):
    session, config, prov = archived
    assert not Path(prov['inputs']['world']['path']).exists()
    assert all(c['state'] == 'pass' for c in gazebo_world_checks(session, config, prov))


def test_archived_world_origin_and_input_identity_are_checked(archived):
    session, config, prov = archived
    snapshot = session.root/'evaluation/physical'
    edit_world(snapshot, lambda root: root.find("world/model[@name='scan_car']/pose")
               .__setattr__('text', '8 0 0 0 0 0'))
    checks = {c['name']: c for c in gazebo_world_checks(session, config, prov)}
    assert checks['physical_snapshot_identity']['state'] == 'fail'
    assert checks['gazebo_world_matches_capture_origin']['state'] == 'fail'


def test_legacy_unprotected_snapshot_requires_recapture(archived):
    session, config, prov = archived
    session.summary['files'].pop('evaluation/physical/world.sdf')
    assert gazebo_world_checks(session, config, prov)[0]['state'] == 'fail'
    shutil.rmtree(session.root/'evaluation/physical')
    with pytest.raises(FileNotFoundError, match='recapture legacy sessions'):
        gazebo_world_checks(session, config, prov)


@pytest.mark.parametrize('rough', [False, True])
@pytest.mark.parametrize('mutation', ['height', 'gauge', 'width', 'length', 'rotation', 'model', 'link', 'missing'])
def test_rail_collision_boxes_are_checked(built, tmp_path, rough, mutation):
    _, config, spec = built
    config = copy.deepcopy(config)
    if not rough: config['truth'].pop('track_irregularity')
    folder = tmp_path/'world'; folder.mkdir(); make_world(folder, config, spec)
    assert check(config, spec, folder/'world.sdf')['passed']
    def mutate(root):
        track = root.find("world/model[@name='track']")
        link = track.find("link[@name='rails']")
        collision = link.find("collision[@name='left_head']")
        if mutation == 'missing':
            link.remove(collision); return
        if mutation in ('model', 'link'):
            ET.SubElement(track if mutation == 'model' else link, 'pose').text = '0 0 .1 0 0 0'
            return
        node = collision.find('geometry/box/size' if mutation in ('width', 'length') else 'pose')
        index = {'height': 2, 'gauge': 1, 'width': 1, 'length': 0, 'rotation': 5}[mutation]
        values = list(map(float, node.text.split())); values[index] += .1
        node.text = ' '.join(map(str, values))
    edit_world(folder, mutate)
    assert {'rail_boxes_match_configuration', 'rail_boxes_match_manifest'} <= failed(check(config, spec, folder/'world.sdf'))
