#!/bin/sh
# Prints "<git HEAD> <dirty 0|1> <digest>" for the repository containing $1. The digest
# covers the tracked diff against HEAD and every untracked, non-ignored file, so the
# same source tree always yields the same line at build time and at run time.
set -e
root=$(git -C "$1" rev-parse --show-toplevel)
cd "$root"
head=$(git rev-parse HEAD)
if [ -n "$(git status --porcelain)" ]; then dirty=1; else dirty=0; fi
digest=$({ git diff HEAD --no-ext-diff --binary; git ls-files -o --exclude-standard | LC_ALL=C sort | while IFS= read -r f; do sha256sum "$f"; done; } | sha256sum | cut -c1-64)
echo "$head $dirty $digest"
