"""Lint what this branch changed; never reformat untouched code (docs/DEVELOPMENT_RULES.md).

ruff.toml applies pyflakes rules to every Python file. Line length and one statement
per line (E501, E701, E702, E703) apply only to changed Python lines, and clang-format
(.clang-format) only to changed C/C++/CUDA lines. Tools come from PATH or from the
RUFF, GIT_CLANG_FORMAT and CLANG_FORMAT environment variables.
"""
import argparse
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys

PYTHON_LINE_RULES = 'E501,E701,E702,E703'
CPP_SUFFIXES = ('.c', '.cc', '.cpp', '.h', '.hpp', '.cu')
HUNK = re.compile(r'^@@ -\d+(?:,\d+)? \+(\d+)(?:,(\d+))? @@')


def changed_lines(diff_text):
    """{path: new-side line numbers} from `git diff -U0`; pure deletions add nothing."""
    lines, path = {}, None
    for line in diff_text.splitlines():
        if line.startswith('+++ '):
            target = line[4:]
            path = None if target == '/dev/null' else target.removeprefix('b/')
        elif path is not None and (match := HUNK.match(line)):
            start, count = int(match.group(1)), int(match.group(2) or 1)
            lines.setdefault(path, set()).update(range(start, start+count))
    return {name: rows for name, rows in lines.items() if rows}


def on_changed_lines(diagnostics, changed, root):
    """ruff JSON diagnostics restricted to changed lines, as path:row:col messages."""
    kept = []
    for item in diagnostics:
        name = os.path.relpath(item['filename'], root)
        row = item['location']['row']
        if row in changed.get(name, ()):
            kept.append(f"{name}:{row}:{item['location']['column']}: {item['code']} {item['message']}")
    return kept


def tool(name, variable):
    path = os.environ.get(variable) or shutil.which(name)
    if not path:
        raise SystemExit(f'{name} not found: install it or set {variable}')
    return path


def resolve_base(root, requested):
    def commit(ref):
        return subprocess.run(['git', 'rev-parse', '--verify', '--quiet', ref+'^{commit}'], cwd=root,
                              capture_output=True, text=True).returncode == 0
    if requested and set(requested) != {'0'} and commit(requested):
        return requested
    for ref in ('origin/main', 'main'):
        result = subprocess.run(['git', 'merge-base', 'HEAD', ref], cwd=root, capture_output=True, text=True)
        if result.returncode == 0:
            return result.stdout.strip()
    raise SystemExit('no usable base commit: pass --base or fetch main')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--base', help='commit to compare with (default: merge-base with origin/main or main)')
    args = parser.parse_args(argv)
    root = Path(subprocess.check_output(['git', 'rev-parse', '--show-toplevel'], text=True).strip())
    base = resolve_base(root, args.base)
    diff = subprocess.check_output(['git', 'diff', '-U0', '--no-color', '--no-ext-diff', base, '--'],
                                   cwd=root, text=True)
    changed = changed_lines(diff)
    problems = []
    ruff = tool('ruff', 'RUFF')
    whole = subprocess.run([ruff, 'check', '--quiet', 'src', 'tools'], cwd=root, capture_output=True, text=True)
    if whole.returncode:
        problems.append((whole.stdout or whole.stderr).strip())
    python = sorted(name for name in changed if name.endswith('.py') and (root/name).is_file())
    if python:
        result = subprocess.run([ruff, 'check', '--isolated', '--select', PYTHON_LINE_RULES, '--line-length', '120',
                                 '--output-format', 'json', '--exit-zero', *python],
                                cwd=root, capture_output=True, text=True, check=True)
        problems += on_changed_lines(json.loads(result.stdout or '[]'), changed, root)
    cpp = sorted(name for name in changed if name.endswith(CPP_SUFFIXES) and (root/name).is_file())
    if cpp:
        result = subprocess.run([tool('git-clang-format', 'GIT_CLANG_FORMAT'), '--binary',
                                 tool('clang-format', 'CLANG_FORMAT'), '--diff', base, '--', *cpp],
                                cwd=root, capture_output=True, text=True)
        output = result.stdout.strip()
        if result.returncode not in (0, 1) or (output and not output.startswith(
                ('no modified files', 'clang-format did not modify'))):
            problems.append(output or result.stderr.strip())
    for problem in problems:
        print(problem)
    print(json.dumps(dict(base=base, changed_files=len(changed), python_files=len(python), cpp_files=len(cpp),
                          problems=len(problems))), file=sys.stderr)
    return 1 if problems else 0


if __name__ == '__main__':
    raise SystemExit(main())
