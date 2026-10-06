"""Read-only inventory of sessions/ and local_data/ for retention decisions (DATA_RETENTION.md).

One lstat pass, no hashing, symlinks never followed. Disk use is attributed per inode:
`exclusive_bytes` is what deleting a directory would actually free, `shared_bytes` stays
allocated through hard links elsewhere. Each directory lists who refers to it: current
docs, history docs, the media manifest, evaluation/protocol records and symlinks.
Nothing is deleted or modified.
"""
import argparse
from collections import defaultdict
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import time

ROOTS = ('sessions', 'local_data')


def walk(top):
    """(path, lstat) for every non-directory entry below top, without following links."""
    pending = [top]
    while pending:
        folder = pending.pop()
        try:
            entries = list(os.scandir(folder))
        except OSError:
            continue
        for entry in entries:
            try:
                if entry.is_dir(follow_symlinks=False):
                    pending.append(entry.path)
                else:
                    yield entry.path, entry.stat(follow_symlinks=False)
            except OSError:
                continue


def attribute(owners):
    """owners: {inode: (allocated_bytes, set(directories))} -> per-directory byte split."""
    usage = defaultdict(lambda: dict(logical_inode_bytes=0, exclusive_bytes=0, shared_bytes=0))
    for size, directories in owners.values():
        for directory in directories:
            usage[directory]['logical_inode_bytes'] += size
            usage[directory]['exclusive_bytes' if len(directories) == 1 else 'shared_bytes'] += size
    return usage


def references(repo, names):
    """{directory: {kind: [referring files]}} from tracked docs and evaluation records.

    A reference must name the repository-relative path (e.g. sessions/foo) ending at a
    path boundary, so ordinary words and longer sibling names do not match.
    """
    found = defaultdict(lambda: defaultdict(list))
    patterns = [(name, re.compile(re.escape(name)+r'(?![\w.-])')) for name in names]
    tracked = subprocess.check_output(['git', 'ls-files', '*.md', '*.json'], cwd=repo, text=True).split()
    sources = [(name, 'history_docs' if name.startswith('docs/history/') else
                'media_manifest' if name == 'docs/media/manifest.json' else
                'retention_list' if name == 'docs/DATA_RETENTION.md' else 'current_docs') for name in tracked]
    evaluation = repo/'local_data/evaluation'
    if evaluation.is_dir():
        sources += [(str(path.relative_to(repo)), 'evaluation_records')
                    for path in evaluation.rglob('*.json') if path.stat().st_size < 4 << 20]
    for name, kind in sources:
        try:
            text = (repo/name).read_text(encoding='utf-8', errors='ignore')
        except OSError:
            continue
        for directory, pattern in patterns:
            if pattern.search(text):
                found[directory][kind].append(name)
    return found


def classify(refs):
    if refs.get('retention_list') or refs.get('current_docs') or refs.get('symlinked_from_retained'):
        return 'required'
    if refs.get('media_manifest'):
        return 'media_source'
    if refs.get('evaluation_records'):
        return 'evaluation_referenced'
    if refs.get('history_docs'):
        return 'history_only'
    return 'unreferenced'


def open_by_processes(paths):
    """Directories that a running process has as cwd or holds an open file in."""
    busy = set()
    for pid in filter(str.isdigit, os.listdir('/proc')):
        links = [f'/proc/{pid}/cwd'] + [f'/proc/{pid}/fd/{fd}' for fd in _fds(pid)]
        for link in links:
            try:
                target = os.readlink(link)
            except OSError:
                continue
            busy.update(path for path in paths if target == path or target.startswith(path+os.sep))
    return busy


def _fds(pid):
    try:
        return os.listdir(f'/proc/{pid}/fd')
    except OSError:
        return []


def inventory(repo):
    repo = Path(repo).resolve()
    started = time.monotonic()
    owners, files, apparent, symlinks = {}, defaultdict(int), defaultdict(int), defaultdict(set)
    tops = [repo/root/entry for root in ROOTS if (repo/root).is_dir() for entry in sorted(os.listdir(repo/root))
            if (repo/root/entry).is_dir() and not (repo/root/entry).is_symlink()]
    for top in tops:
        key = str(top.relative_to(repo))
        for path, info in walk(str(top)):
            files[key] += 1
            apparent[key] += info.st_size
            if os.path.islink(path):
                target = os.path.realpath(path)
                for other in tops:
                    if target.startswith(str(other)+os.sep) or target == str(other):
                        if other != top:
                            symlinks[str(other.relative_to(repo))].add(key)
                continue
            inode = (info.st_dev, info.st_ino)
            allocated = info.st_blocks*512
            owners.setdefault(inode, (allocated, set()))[1].add(key)
    usage = attribute(owners)
    names = [str(top.relative_to(repo)) for top in tops]
    refs = references(repo, names)
    busy = open_by_processes([str(top) for top in tops])
    rows = []
    for top, name in zip(tops, names):
        entry_refs = {kind: sorted(set(paths)) for kind, paths in refs.get(name, {}).items()}
        if symlinks.get(name):
            entry_refs['symlinked_from'] = sorted(symlinks[name])
        rows.append(dict(directory=name, files=files[name], apparent_bytes=apparent[name], **usage[name],
                         references=entry_refs, busy=str(top) in busy))
    retained = {row['directory'] for row in rows if classify(row['references']) == 'required'}
    changed = True
    while changed:  # a symlink from a retained directory makes its target retained too
        changed = False
        for row in rows:
            if row['directory'] not in retained and set(row['references'].get('symlinked_from', ())) & retained:
                row['references']['symlinked_from_retained'] = sorted(
                    set(row['references']['symlinked_from']) & retained)
                retained.add(row['directory'])
                changed = True
    for row in rows:
        row['class'] = classify(row['references'])
    return dict(schema='ssb.data_inventory.v1', generated_at=time.strftime('%Y-%m-%dT%H:%M:%S%z'),
                repo=str(repo), wall_s=time.monotonic()-started,
                note='read-only; exclusive_bytes is freed only if every hard link to it is deleted',
                directories=sorted(rows, key=lambda row: -row['exclusive_bytes']))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True,
                        help='new JSON file; a .tsv summary is written next to it')
    args = parser.parse_args(argv)
    if args.output.exists():
        raise SystemExit('inventory output must be new')
    repo = Path(subprocess.check_output(['git', 'rev-parse', '--show-toplevel'], text=True).strip())
    result = inventory(repo)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=1)+'\n')
    lines = ['directory\tclass\tfiles\texclusive_GiB\tshared_GiB\tbusy\treferenced_by']
    for row in result['directories']:
        lines.append('\t'.join([row['directory'], row['class'], str(row['files']),
                                f"{row['exclusive_bytes']/2**30:.2f}", f"{row['shared_bytes']/2**30:.2f}",
                                str(row['busy']), ','.join(sorted(row['references']))]))
    args.output.with_suffix('.tsv').write_text('\n'.join(lines)+'\n')
    print(json.dumps(dict(directories=len(result['directories']), wall_s=result['wall_s'],
                          exclusive_GiB=sum(r['exclusive_bytes'] for r in result['directories'])/2**30)),
          file=sys.stderr)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
