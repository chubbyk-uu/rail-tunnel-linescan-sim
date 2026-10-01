#!/usr/bin/env python3
"""Use the installed private WSL Mesa for this child; no sibling checkout required."""
import argparse
import os
from pathlib import Path
import sys


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--system',action='store_true')
    p.add_argument('--prefix',type=Path,default=Path(os.environ.get('SSB_MESA_PREFIX',str(Path.home()/'opt/agv-mesa-25.2.8/install'))))
    p.add_argument('command',nargs=argparse.REMAINDER)
    a=p.parse_args();command=a.command[1:] if a.command[:1]==['--'] else a.command
    if not command:p.error('a command is required')
    env=dict(os.environ)
    if not a.system:
        prefix=a.prefix.resolve();lib=prefix/'lib'
        libraries=[lib/x for x in ('libgallium-25.2.8.so','libgbm.so.1','libGLX_mesa.so.0','libEGL_mesa.so.0')]
        vendor=prefix/'share/glvnd/egl_vendor.d/50_mesa.json'
        if any(c.isspace() or c==':' for c in str(prefix)):p.error('Mesa prefix cannot contain whitespace or colon')
        for path in [*libraries,vendor,lib/'dri',lib/'gbm']:
            if not path.exists():p.error(f'missing private Mesa component: {path}; set SSB_MESA_PREFIX or use --system explicitly')
        env['LD_PRELOAD']=':'.join(map(str,libraries))+(':'+env['LD_PRELOAD'] if env.get('LD_PRELOAD') else '')
        env['LD_LIBRARY_PATH']=str(lib)+(':'+env['LD_LIBRARY_PATH'] if env.get('LD_LIBRARY_PATH') else '')
        env['LIBGL_DRIVERS_PATH']=str(lib/'dri');env['GBM_BACKENDS_PATH']=str(lib/'gbm')
        env['__EGL_VENDOR_LIBRARY_FILENAMES']=str(vendor);env['__GLX_VENDOR_LIBRARY_NAME']='mesa'
    os.execvpe(command[0],command,env)


if __name__=='__main__':
    sys.exit(main())
