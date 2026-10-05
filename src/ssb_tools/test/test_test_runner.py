"""A worker crash, omitted case or duplicate must not produce a passing summary."""
import xml.etree.ElementTree as ET

import pytest

from ssb_tools.test_runner import merge_results, partition


def test_both_cuda_modules_share_one_group_and_do_not_enter_cpu_groups():
    nodes = ['test/test_unroll_cuda.py::sample', 'test/test_fast_geometry_cuda.py::sample',
             'test/test_fast_geometry.py::cpu', 'test/test_global_optimization.py::cpu']
    groups = partition(nodes, {}, 4)
    assert set(groups[0]) == set(nodes[:2])
    assert set(node for group in groups[1:] for node in group) == set(nodes[2:])


def worker(path, names, failed=False):
    root = ET.Element('testsuite', tests=str(len(names)), failures=str(int(failed)), errors='0', skipped='0')
    for name in names:
        case = ET.SubElement(root, 'testcase', classname='sample', name=name, time='.1')
        if failed:
            ET.SubElement(case, 'failure', message='injected failure')
    ET.ElementTree(root).write(path)
    return path


def test_failed_cases_are_preserved_in_the_combined_report(tmp_path):
    a = worker(tmp_path/'a.xml', ['pass'])
    b = worker(tmp_path/'b.xml', ['fail'], failed=True)
    output = tmp_path/'merged.xml'
    assert merge_results([a, b], [{('sample', 'pass')}, {('sample', 'fail')}], output) == 2
    root = ET.parse(output).getroot()
    assert len(list(root.iter('testcase'))) == 2
    assert len(list(root.iter('failure'))) == 1


@pytest.mark.parametrize('kind', ['missing_file', 'omitted_case', 'duplicate', 'unexpected_case'])
def test_incomplete_or_duplicate_workers_cannot_overwrite_the_report(tmp_path, kind):
    a = worker(tmp_path/'a.xml', ['first'])
    b = tmp_path/'b.xml'
    if kind != 'missing_file':
        worker(b, [] if kind == 'omitted_case' else ['extra'] if kind == 'unexpected_case' else ['first'])
    output = tmp_path/'merged.xml'
    output.write_bytes(b'previous report')
    with pytest.raises((OSError, ValueError)):
        merge_results([a, b], [{('sample', 'first')}, {('sample', 'second')}], output)
    assert output.read_bytes() == b'previous report'
