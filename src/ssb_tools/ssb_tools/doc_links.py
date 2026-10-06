"""Check relative Markdown links: the target is tracked by git (so it exists in every
clone, not only on this machine) and named anchors match a heading (GitHub slug
rules). External URLs are not fetched."""
import argparse
import json
from pathlib import Path
import re
import subprocess
import sys

LINK = re.compile(r'(?<!!)\[[^\]]*\]\(([^)\s]+)\)|!\[[^\]]*\]\(([^)\s]+)\)')
HEADING = re.compile(r'^(#{1,6})\s+(.*?)\s*#*\s*$')
FENCE = re.compile(r'^\s*(```|~~~)')


def without_code(text):
    """Lines outside fenced code blocks, with inline code spans blanked."""
    kept, fenced = [], False
    for line in text.splitlines():
        if FENCE.match(line):
            fenced = not fenced
            kept.append('')
            continue
        kept.append('' if fenced else re.sub(r'`[^`]*`', '', line))
    return kept


def slug(heading):
    """GitHub-style anchor: lower case, drop punctuation except '-' and '_', spaces to '-'."""
    text = re.sub(r'<[^>]+>', '', heading).strip().lower()
    text = re.sub(r'[^\w\- ]', '', text)
    return text.replace(' ', '-')


def anchors(path):
    seen, result = {}, set()
    for line in without_code(path.read_text(encoding='utf-8')):
        match = HEADING.match(line)
        if not match:
            continue
        base = slug(match.group(2))
        count = seen.get(base, 0)
        seen[base] = count+1
        result.add(base if count == 0 else f'{base}-{count}')
    return result


def check(root, files, tracked=None):
    """tracked: repository-relative files that exist in a clone (default: the files checked)."""
    root = Path(root).resolve()
    tracked = {Path(name) for name in (files if tracked is None else tracked)}
    available = tracked | {parent for name in tracked for parent in name.parents}
    problems, cache = [], {}
    for name in files:
        path = (root/name).resolve()
        for number, line in enumerate(without_code(path.read_text(encoding='utf-8')), 1):
            for match in LINK.finditer(line):
                target = match.group(1) or match.group(2)
                if re.match(r'^[a-z][a-z0-9+.-]*:', target, re.I):
                    continue  # http(s), mailto and other schemes are external
                file_part, _, anchor = target.partition('#')
                resolved = (path.parent/file_part).resolve() if file_part else path
                where = f'{name}:{number}: {target}'
                if not resolved.is_relative_to(root):
                    problems.append(where+' points outside the repository')
                elif not resolved.exists():
                    problems.append(where+' target does not exist')
                elif resolved.relative_to(root) not in available:
                    problems.append(where+' target is not tracked by git (missing from clones)')
                elif anchor and resolved.suffix == '.md':
                    if resolved not in cache:
                        cache[resolved] = anchors(resolved)
                    if anchor.lower() not in cache[resolved]:
                        problems.append(where+' anchor not found')
    return problems


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.parse_args(argv)
    root = Path(subprocess.check_output(['git', 'rev-parse', '--show-toplevel'], text=True).strip())
    files = subprocess.check_output(['git', 'ls-files', '*.md'], cwd=root, text=True).split()
    tracked = subprocess.check_output(['git', 'ls-files'], cwd=root, text=True).split()
    problems = check(root, files, tracked)
    for problem in problems:
        print(problem)
    print(json.dumps(dict(files=len(files), problems=len(problems))), file=sys.stderr)
    return 1 if problems else 0


if __name__ == '__main__':
    raise SystemExit(main())
