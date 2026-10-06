"""The installed runtime wrappers choose WSL or native explicitly and never mix driver files."""
import os
from pathlib import Path
import subprocess
import sys

import pytest

from ssb_tools.package_paths import probe_command, share_file

RUNTIME = share_file('tools/ssb_runtime.sh')


def fake_optix(folder, versions=('1.2.3',), gpucomp=True):
    folder.mkdir(parents=True, exist_ok=True)
    (folder/'libnvoptix.so.1').write_text('')
    for version in versions:
        (folder/f'libnvidia-rtcore.so.{version}').write_text('')
        if gpucomp:
            (folder/f'libnvidia-gpucomp.so.{version}').write_text('')
    return folder


def launch(tmp_path, release, *command, gui=False, **extra):
    osrelease = tmp_path/'osrelease'
    osrelease.write_text(release+'\n')
    env = {key: value for key, value in os.environ.items()
           if not key.startswith(('SSB_', 'GALLIUM', 'MESA_D3D12', 'QT_QPA'))}
    env.update(SSB_OSRELEASE_FILE=str(osrelease), HOME=str(tmp_path/'home'), LD_LIBRARY_PATH='/inherited/lib', **extra)
    arguments = ['bash', str(RUNTIME), *(['--gui'] if gui else []), *(command or ['env'])]
    result = subprocess.run(arguments, env=env, capture_output=True, text=True)
    variables = dict(line.split('=', 1) for line in result.stdout.splitlines() if '=' in line)
    return result.returncode, variables, result.stderr


def test_wsl_kernel_selects_the_isolated_optix_runtime_with_the_original_library_path(tmp_path):
    runtime = fake_optix(tmp_path/'home/opt/optix-runtime-1.2.3')
    code, env, _ = launch(tmp_path, '6.18.33.2-microsoft-standard-WSL2')
    assert code == 0 and env['SSB_RUNTIME_MODE'] == 'wsl'
    assert env['LD_LIBRARY_PATH'] == f'{runtime}:/usr/lib/wsl/lib:/usr/local/cuda/lib64'
    assert env['SSB_OPTIX_RUNTIME'] == str(runtime) and env['SSB_OPTIX_DRIVER_VERSION'] == '1.2.3'
    assert 'GALLIUM_DRIVER' not in env


def test_native_kernel_keeps_the_system_environment(tmp_path):
    fake_optix(tmp_path/'home/opt/optix-runtime-1.2.3')
    code, env, _ = launch(tmp_path, '6.8.0-45-generic', gui=True)
    assert code == 0 and env['SSB_RUNTIME_MODE'] == 'native'
    assert env['LD_LIBRARY_PATH'] == '/inherited/lib'
    assert 'SSB_OPTIX_RUNTIME' not in env and 'GALLIUM_DRIVER' not in env


def test_wsl_gui_adds_d3d12_mesa_through_the_named_launcher(tmp_path):
    runtime = fake_optix(tmp_path/'home/opt/optix-runtime-1.2.3')
    mesa = tmp_path/'mesa.py'
    mesa.write_text('import os, sys\nos.environ["MESA_SEEN"] = "1"\nos.execvp(sys.argv[1], sys.argv[1:])\n')
    code, env, _ = launch(tmp_path, 'microsoft', gui=True, SSB_MESA_WRAPPER=str(mesa))
    assert code == 0 and env['MESA_SEEN'] == '1' and env['GALLIUM_DRIVER'] == 'd3d12'
    assert env['LD_LIBRARY_PATH'].startswith(f'{runtime}:')
    code, _, error = launch(tmp_path, 'microsoft', gui=True, SSB_MESA_WRAPPER=str(tmp_path/'missing.py'))
    assert code == 2 and 'Mesa' in error


@pytest.mark.parametrize('case', ['two_versions', 'no_version', 'no_gpucomp', 'two_runtimes', 'no_runtime'])
def test_ambiguous_or_incomplete_optix_runtime_is_refused(tmp_path, case):
    home = tmp_path/'home/opt'
    if case == 'two_versions':
        fake_optix(home/'optix-runtime-1.2.3', ('1.2.3', '1.2.4'))
    elif case == 'no_version':
        fake_optix(home/'optix-runtime-1.2.3', ())
    elif case == 'no_gpucomp':
        fake_optix(home/'optix-runtime-1.2.3', gpucomp=False)
    elif case == 'two_runtimes':
        fake_optix(home/'optix-runtime-1.2.3'); fake_optix(home/'optix-runtime-1.2.4', ('1.2.4',))
    code, env, error = launch(tmp_path, 'microsoft')
    assert code == 1 and 'LD_LIBRARY_PATH' not in env and error


def test_explicit_version_or_directory_resolves_several_installed_runtimes(tmp_path):
    home = tmp_path/'home/opt'
    fake_optix(home/'optix-runtime-1.2.3'); chosen = fake_optix(home/'optix-runtime-1.2.4', ('1.2.4',))
    for extra in (dict(SSB_OPTIX_DRIVER_VERSION='1.2.4'), dict(SSB_OPTIX_RUNTIME=str(chosen))):
        code, env, _ = launch(tmp_path, 'microsoft', **extra)
        assert code == 0 and env['SSB_OPTIX_RUNTIME'] == str(chosen) and env['SSB_OPTIX_DRIVER_VERSION'] == '1.2.4'


@pytest.mark.parametrize('extra,release', [(dict(SSB_RUNTIME='bogus'), 'microsoft'), ({}, None)])
def test_undecidable_runtime_is_an_error_not_a_guess(tmp_path, extra, release):
    if release is None:
        extra = dict(extra, SSB_OSRELEASE_FILE=str(tmp_path/'unreadable'))
        env = dict(os.environ, **extra)
        result = subprocess.run(['bash', str(RUNTIME), 'true'], env=env, capture_output=True, text=True)
        assert result.returncode == 2 and 'SSB_RUNTIME' in result.stderr
        return
    code, _, error = launch(tmp_path, release, 'true', **extra)
    assert code == 2 and 'SSB_RUNTIME' in error


def test_production_probe_and_installed_launchers_use_the_shared_runtime_entry():
    assert probe_command()[:2] == ['bash', str(RUNTIME)]
    for name in ('with_optix_runtime.sh', 'with_mesa_runtime.py'):
        assert share_file('tools/'+name).is_file()
    repo = Path(__file__).resolve().parents[3]
    if (repo/'tools').is_dir():  # source checkout: no launcher bypasses the shared entry
        for script in sorted((repo/'tools').glob('run_*.sh')):
            assert 'with_optix_runtime.sh' not in script.read_text(), script
            assert 'with_mesa_runtime.py' not in script.read_text(), script
    assert sys.executable
