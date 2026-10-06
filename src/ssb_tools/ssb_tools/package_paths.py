"""Read-only ROS resources are located through the installed ament index."""
from pathlib import Path
from ament_index_python.packages import get_package_prefix, get_package_share_directory


def share_file(name, package='ssb_tools'):
    return Path(get_package_share_directory(package))/name


def core_executable(name):
    return Path(get_package_prefix('ssb_core'))/'lib/ssb_core'/name


def probe_command():
    return ['bash', str(share_file('tools/ssb_runtime.sh')), str(core_executable('ssb_probe'))]
