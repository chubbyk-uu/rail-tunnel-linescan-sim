#!/usr/bin/env python3
"""Download the pinned public textures used by the demo, without old local assets."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import zipfile

REPO = Path(__file__).resolve().parents[1]


def sha256(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def verify(path, entry):
    path = Path(path)
    if path.stat().st_size != entry['bytes'] or sha256(path) != entry['sha256']:
        raise ValueError(f'source differs from pinned public asset: {path}')


def fetch(path, entry):
    if path.exists():
        verify(path, entry)
        return
    partial = path.with_suffix(path.suffix + '.part')
    try:
        # curl inherits the caller's proxy; no credentials are stored in manifests.
        subprocess.run(['curl', '--fail', '--location', '--silent', '--show-error',
                        '--retry', '2', '--connect-timeout', '20', '--max-time', '1800',
                        '--max-filesize', str(2 << 30), '--user-agent', 'Mozilla/5.0',
                        entry['url'], '--output', str(partial)], check=True)
        verify(partial, entry)
        partial.replace(path)
    finally:
        partial.unlink(missing_ok=True)


def prepare(root, verify_only=False):
    root = Path(root).resolve()
    sources = json.loads((REPO/'assets/materials/demo_sources.json').read_text())['sources']
    for name, source in sources.items():
        folder = root/name
        if not verify_only:
            folder.mkdir(parents=True, exist_ok=True)
            if 'package' in source:
                entry = source['package']
                archive = folder/entry['file']
                # An already complete cache need not retain the downloaded archive.
                if not all((folder/c['file']).is_file() for c in source['channels'].values()):
                    fetch(archive, entry)
                    with zipfile.ZipFile(archive) as zipped:
                        for channel in source['channels'].values():
                            target = folder/channel['file']
                            if target.exists():
                                verify(target, channel)
                                continue
                            info = zipped.getinfo(channel['file'])
                            if info.file_size != channel['bytes']:
                                raise ValueError(f'unexpected ZIP member size: {info.filename}')
                            partial = target.with_suffix(target.suffix + '.part')
                            try:
                                # Extract only three named maps, not every file in the ZIP.
                                with zipped.open(info) as src, partial.open('wb') as dst:
                                    shutil.copyfileobj(src, dst, 1 << 20)
                                verify(partial, channel)
                                partial.replace(target)
                            finally:
                                partial.unlink(missing_ok=True)
            else:
                for channel in source['channels'].values():
                    fetch(folder/channel['file'], channel)
        for channel in source['channels'].values():
            verify(folder/channel['file'], channel)
        manifest = folder/'downloads.json'
        if manifest.exists():
            if json.loads(manifest.read_text()) != source:
                raise ValueError(f'existing source metadata differs from recipe: {manifest}')
        elif verify_only:
            raise ValueError(f'missing source metadata: {manifest}')
        else:
            temporary = manifest.with_suffix('.json.part')
            temporary.write_text(json.dumps(source, indent=2)+'\n')
            temporary.replace(manifest)
        print(f'{name}: verified', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=REPO/'local_data/stage_b/sources')
    parser.add_argument('--verify-only', action='store_true', help='offline integrity check; do not download')
    args = parser.parse_args()
    prepare(args.output, args.verify_only)


if __name__ == '__main__':
    main()
