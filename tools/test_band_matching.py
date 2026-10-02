#!/usr/bin/env python3
"""Relocate only public D1 products; deny other workspace data in the child."""
import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
from ssb_tools.session import sha256_file

CHILD = '''
import json,sys
from pathlib import Path
public, output, workspace = map(lambda p: Path(p).resolve(), sys.argv[1:4])
def audit(event, args):
    if event != 'open' or not isinstance(args[0], (str, bytes)): return
    path = Path(args[0].decode() if isinstance(args[0], bytes) else args[0]).resolve()
    if not path.is_relative_to(workspace): return
    if path.is_relative_to(public) or path.is_relative_to(output): return
    if path.suffix in ('.py', '.pyc', '.so'): return
    raise PermissionError('non-public workspace data access: '+str(path))
sys.addaudithook(audit)
from ssb_tools.match_bands import run
from ssb_tools.band_matching import MatchSettings
settings = json.loads(sys.argv[4])
raw = public/'raw' if (public/'native_source.json').exists() else None
run(public, output, settings['spacing_m'], settings['height'], settings['max_width'],
    MatchSettings(**settings['settings']), settings['halo_m'], raw)
'''


def run(unroll, reference, output):
    unroll, reference, output = map(lambda p: Path(p).resolve(), (unroll, reference, output))
    output.mkdir(parents=True, exist_ok=False)
    public = output/'public_d1'; public.mkdir()
    names = ['provenance.json','report.json','bands.json','projection.npy','mapping.npz']
    names.append('native_source.json' if (unroll/'native_source.json').exists() else 'sensor_flat.npy')
    def place(source, target):
        try: os.link(source, target)
        except OSError: shutil.copyfile(source, target)
    for name in names: place(unroll/name, public/name)
    if (unroll/'native_source.json').exists():
        # D1 v2 samples the capture's public raw blocks: relocate exactly those, by identity.
        source = json.loads((unroll/'native_source.json').read_text())
        (public/'raw').mkdir()
        for block in source['blocks']: place(Path(source['session_root'])/'raw'/block['file'], public/'raw'/block['file'])
    parameters = json.loads((reference/'provenance.json').read_text())['parameters']
    # Locate only installed code's repository for the read audit, never sys.path.
    workspace = Path(subprocess.check_output(['git','-C',str(unroll),'rev-parse','--show-toplevel'],text=True).strip())
    result = output/'matches'
    with (output/'matching.log').open('w') as log:
        subprocess.run([sys.executable,'-c',CHILD,str(public),str(result),str(workspace),json.dumps(parameters)],
                       stdout=log,stderr=subprocess.STDOUT,check=True)
    products = ['matches.npy','windows.json']
    for name in products:
        if sha256_file(reference/name) != sha256_file(result/name): raise AssertionError('different '+name)
    provenance = json.loads((result/'provenance.json').read_text())
    assert all(Path(p).is_relative_to(public) for p in provenance['inputs'])
    report = json.loads((result/'report.json').read_text())
    proof = dict(public_only=True, relocated_D1_products=len(names),
        relocated_raw_blocks=len(list((public/'raw').iterdir())) if (public/'raw').exists() else 0,
        workspace_data_reads_denied_except_public_inputs_outputs_and_code=True,
        identical_products=products, input_paths_confined=True, status=report['status'],
        matches=report['matches'], windows=report['windows'], performance=provenance['performance'])
    (output/'report.json').write_text(json.dumps(proof,indent=2)+'\n')
    print(json.dumps(proof))


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('unroll','reference','output'):parser.add_argument('--'+name,required=True)
    args=parser.parse_args();run(args.unroll,args.reference,args.output)
