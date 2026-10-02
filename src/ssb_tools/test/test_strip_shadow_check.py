"""Geometry expectations independent of Ogre's screen-space shadow result."""
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml

from ssb_tools.stage_b_robot import make_robot
from ssb_tools.strip_shadow_check import floor_coverage, diagnostic_world


@pytest.fixture
def car(tmp_path):
    repo = Path(__file__).resolve().parents[3]
    config = yaml.safe_load((repo/'src/ssb_core/config/stage_b.yaml').read_text())
    spec = yaml.safe_load((repo/'src/ssb_tools/config/stage_b_scene.yaml').read_text())
    return make_robot(tmp_path, config, spec)


@pytest.mark.parametrize('angle,kind', [(180, 'full'), (170, 'full'), (190, 'full'),
                                      (164, 'partial'), (196, 'partial'),
                                      (160, 'none'), (200, 'none')])
def test_floor_intercepts_cones_by_angle(car, angle, kind):
    result = floor_coverage(car, angle)
    blocked, total = result['blocked_rays'], result['tested_rays']
    assert total > 0
    if kind == 'full': assert blocked == total
    elif kind == 'none': assert blocked == 0
    else: assert 0 < blocked < total


def test_diagnostic_keeps_geometry_but_removes_acquisition_and_motion(tmp_path, car):
    root = ET.Element('sdf', version='1.9'); world = ET.SubElement(root, 'world', name='stage_b')
    world.append(car)
    source = tmp_path/'source.sdf'; ET.ElementTree(root).write(source)
    output = tmp_path/'diagnostic.sdf'
    original, position = diagnostic_world(source, output)
    result = ET.parse(output).find('world')
    assert original.find("joint[@name='scan']") is not None
    assert len(result.findall("model[@name='scanner']/link/light")) == 17
    assert len(result.findall("model[@name='scan_car']/link/light")) == 4
    assert not result.findall('model/plugin') and not result.findall('model/joint')
    assert not result.findall('model/link/collision')
    assert not result.findall('model/link/sensor')
    assert output.with_suffix('.config').is_file()
    assert all(m.findtext('static') == 'true' for m in result.findall('model'))
    assert position[2] == pytest.approx(float(original.findtext("link[@name='head']/pose").split()[2]))
