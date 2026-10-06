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


def test_cpu_profile_deselects_only_marked_cuda_module_tests():
    from ssb_tools.test_runner import cpu_ci_selection
    collected = ['test/test_unroll_cuda.py::gpu', 'test/test_unroll_cuda.py::cpu_check',
                 'test/test_fast_geometry_cuda.py::gpu', 'test/test_timing.py::cpu']
    selected = ['test/test_unroll_cuda.py::cpu_check', 'test/test_timing.py::cpu']
    assert cpu_ci_selection(collected, selected) == ['test/test_fast_geometry_cuda.py::gpu',
                                                     'test/test_unroll_cuda.py::gpu']


@pytest.mark.parametrize('selected', [
    [], ['test/test_timing.py::cpu', 'test/test_timing.py::cpu'], ['test/test_other.py::new'],
    ['test/test_unroll_cuda.py::gpu', 'test/test_timing.py::cpu'],  # nothing deselected
    ['test/test_unroll_cuda.py::gpu']])  # a CPU test marked away
def test_cpu_profile_cannot_hide_cpu_tests_or_drift_from_the_full_collection(selected):
    from ssb_tools.test_runner import cpu_ci_selection
    with pytest.raises(ValueError):
        cpu_ci_selection(['test/test_unroll_cuda.py::gpu', 'test/test_timing.py::cpu'], selected)


def test_marker_is_confined_to_the_cuda_modules_in_the_real_collection():
    """The real suite: every requires_cuda test lives in a CUDA module (plain pytest, no GPU needed)."""
    import subprocess, sys
    from pathlib import Path
    root = Path(__file__).resolve().parents[1]
    def collect(*extra):
        out = subprocess.run([sys.executable, '-m', 'pytest', '-q', '-p', 'no:cacheprovider', '--collect-only',
                              *extra, str(root/'test')], cwd=root, capture_output=True, text=True, check=True).stdout
        return [line.strip() for line in out.splitlines() if line.startswith('test/') and '::' in line]
    from ssb_tools.test_runner import cpu_ci_selection
    deselected = cpu_ci_selection(collect(), collect('-m', 'not requires_cuda'))
    assert len(deselected) >= 50
