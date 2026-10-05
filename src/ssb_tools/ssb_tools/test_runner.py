"""Run the complete built workspace suite with bounded Python test parallelism."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET


def history(path):
    if not path.exists():
        return {}
    try:
        root = ET.parse(path).getroot()
        return {case.get('classname', '').removeprefix('ssb_tools.').replace('.', '/')+'.py::'+case.get('name', ''):
                float(case.get('time', 0)) for case in root.iter('testcase')}
    except (ET.ParseError, ValueError):
        return {}  # Duration history is optional, never a prerequisite for testing.


def pytest_configure(config):
    """Only loaded explicitly by this runner; keep each OpenCV worker single-threaded."""
    import cv2
    cv2.setNumThreads(1)


GPU_TEST_PREFIXES = ('test/test_unroll_cuda.py::', 'test/test_fast_geometry_cuda.py::')


def partition(nodes, durations, workers):
    if workers == 1:
        return [nodes]
    gpu = [node for node in nodes if node.startswith(GPU_TEST_PREFIXES)]
    cpu = [node for node in nodes if node not in gpu]
    count = min(workers-bool(gpu), len(cpu))
    groups, costs = [[] for _ in range(count)], [0.] * count
    for node in sorted(cpu, key=lambda node: (-durations.get(node, .02), node)):
        index = min(range(count), key=lambda index: costs[index])
        groups[index].append(node)
        costs[index] += max(.001, durations.get(node, .02))
    return ([gpu] if gpu else [])+groups


def junit_identity(node):
    file, *names = node.split('::')
    classname = 'ssb_tools.'+file.removesuffix('.py').replace('/', '.')
    if len(names) > 1:
        classname += '.'+'.'.join(names[:-1])
    return classname, names[-1]


def merge_results(paths, expected_cases, output):
    """Require fresh, complete, nonduplicated worker results before replacing the report."""
    result = ET.Element('testsuites')
    seen = set()
    for path, expected in zip(paths, expected_cases, strict=True):
        root = ET.parse(path).getroot()
        cases = list(root.iter('testcase'))
        if len(cases) != len(expected):
            raise ValueError(f'incomplete worker results: {path}: {len(cases)} != {len(expected)}')
        for case in cases:
            key = case.get('classname'), case.get('name')
            if key in seen:
                raise ValueError(f'duplicate test result: {key}')
            if key not in expected:
                raise ValueError(f'unexpected test result: {key}')
            seen.add(key)
        for suite in ([root] if root.tag == 'testsuite' else list(root)):
            result.append(suite)
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_suffix('.xml.tmp')
    ET.ElementTree(result).write(temporary, encoding='utf-8', xml_declaration=True)
    temporary.replace(output)
    return len(seen)


def run(workspace, workers=4, output=None):
    workspace = Path(workspace).resolve()
    if not 1 <= workers <= 16:
        raise ValueError('Python worker count must be between 1 and 16')
    test_root = workspace/'src/ssb_tools'
    report = workspace/'build/ssb_tools/pytest.xml'
    for required in (workspace/'install/setup.bash', workspace/'build/ssb_core/CTestTestfile.cmake'):
        if not required.is_file():
            raise ValueError(f'build the workspace first: missing {required}')
    if output is None:
        output = workspace/'log'/f'parallel_tests_{time.strftime("%Y%m%d_%H%M%S")}_{os.getpid()}'
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    environment = dict(os.environ, OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1',
                       MKL_NUM_THREADS='1', PYTHONDONTWRITEBYTECODE='1')
    environment['COLCON_DEFAULTS_FILE'] = str(workspace/'colcon_defaults.yaml')
    # This runner owns collection and selection; inherited pytest filters must not
    # silently turn a requested full run into a subset.
    environment.pop('PYTEST_ADDOPTS', None)
    # Workers must not race on pytest cache files; history comes from the report.
    base = [sys.executable, '-m', 'pytest', '--rootdir', str(test_root),
            '-p', 'no:cacheprovider', '-p', 'ssb_tools.test_runner']
    started = time.monotonic()
    collection = subprocess.run(base+['--collect-only', '-q', str(test_root/'test')],
        cwd=workspace, env=environment, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        timeout=120)
    (output/'collection.log').write_text(collection.stdout)
    if collection.returncode:
        raise RuntimeError(f'test collection failed; see {output}/collection.log')
    nodes = [line.strip() for line in collection.stdout.splitlines() if line.startswith('test/') and '::' in line]
    if not nodes or len(set(nodes)) != len(nodes):
        raise ValueError('test collection must be nonempty and unique')
    groups = partition(nodes, history(report), workers)
    if sorted(node for group in groups for node in group) != sorted(nodes):
        raise ValueError('parallel plan differs from full collection')
    report.unlink(missing_ok=True)  # A failed run must not leave yesterday's passing report.

    def execute(name, command):
        with (output/(name+'.log')).open('w') as log:
            with subprocess.Popen(command, cwd=workspace, env=environment, start_new_session=True,
                                  stdout=log, stderr=subprocess.STDOUT) as process:
                try:
                    return process.wait(timeout=900)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGTERM)
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait()
                    raise

    paths = [output/f'python_{index}.xml' for index in range(len(groups))]
    with ThreadPoolExecutor(max_workers=workers+1) as pool:
        cpp = pool.submit(execute, 'cpp', ['colcon', 'test', '--packages-skip', 'ssb_tools'])

        def python_group(index, group):
            if any(node.startswith(GPU_TEST_PREFIXES) for node in group):
                cpp.result()  # Never overlap Python CUDA contexts with OptiX tests.
            command = base+['-q', '--junit-prefix=ssb_tools', '--junit-xml='+str(paths[index])]
            command += ['--basetemp='+str(output/f'python_{index}_tmp')]
            return execute(f'python_{index}', command+[str(test_root/node) for node in group])

        jobs = [pool.submit(python_group, index, group) for index, group in enumerate(groups)]
        codes = [cpp.result(), *(job.result() for job in jobs)]
    count = merge_results(paths, [{junit_identity(node) for node in group} for group in groups], report)
    result_code = execute('results', ['colcon', 'test-result', '--all'])
    summary = dict(python_tests=count, python_workers=len(groups), return_codes=codes,
                   result_code=result_code, wall_s=time.monotonic()-started, logs=str(output))
    (output/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    print(json.dumps(summary))
    return 1 if any(codes) or result_code else 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', default=Path.cwd(), type=Path)
    parser.add_argument('--workers', type=int, default=4, help='maximum Python processes (default: 4)')
    parser.add_argument('--output', type=Path, help='new directory for logs and worker XML files')
    args = parser.parse_args()
    try:
        return run(args.workspace, args.workers, args.output)
    except (ValueError, RuntimeError, OSError, ET.ParseError, subprocess.TimeoutExpired) as error:
        print(str(error), file=sys.stderr)
        return 1
