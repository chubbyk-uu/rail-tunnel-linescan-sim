#!/usr/bin/env python3
"""Refuse a successful Gazebo exit whose imaging session failed or was truncated."""
import json
from pathlib import Path
import sys


def check(directory):
    path = Path(directory)/'session.json'
    data = json.loads(path.read_text())
    if data.get('status')!='complete' or data.get('motion',{}).get('complete') is not True:
        raise ValueError('capture incomplete: '+data.get('error',json.dumps(data.get('motion',{}))))
    if data.get('rows',0)<=0:
        raise ValueError('capture has no exposures')
    return data['rows']


if __name__=='__main__':
    try:
        print('Session complete:',check(sys.argv[1]),'rows')
    except (ValueError,OSError,KeyError,IndexError) as error:
        print('Session failed:',error,file=sys.stderr)
        sys.exit(1)
