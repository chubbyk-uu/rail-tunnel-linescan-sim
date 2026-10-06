"""Changed-line lint must look only at lines this branch touched."""
from ssb_tools.lint_changed import changed_lines, on_changed_lines

DIFF = '''diff --git a/src/a.py b/src/a.py
--- a/src/a.py
+++ b/src/a.py
@@ -3 +3 @@ x
-old
+new
@@ -10,2 +10,0 @@
-gone
-gone
@@ -20,0 +19,3 @@
+one
+two
+three
diff --git a/removed.py b/removed.py
--- a/removed.py
+++ /dev/null
@@ -1,2 +0,0 @@
-x
-y
diff --git a/new.cpp b/new.cpp
--- /dev/null
+++ b/new.cpp
@@ -0,0 +1,2 @@
+int a;
+int b;
'''


def test_hunks_give_new_side_lines_and_ignore_pure_deletions():
    assert changed_lines(DIFF) == {'src/a.py': {3, 19, 20, 21}, 'new.cpp': {1, 2}}


def test_only_diagnostics_on_changed_lines_are_reported(tmp_path):
    changed = {'src/a.py': {3, 19}}
    diagnostics = [dict(filename=str(tmp_path/'src/a.py'), code='E501', message='long', location=dict(row=row, column=1))
                   for row in (3, 4, 19)] + [dict(filename=str(tmp_path/'src/b.py'), code='E702', message='semi',
                                                  location=dict(row=3, column=5))]
    assert on_changed_lines(diagnostics, changed, tmp_path) == ['src/a.py:3:1: E501 long', 'src/a.py:19:1: E501 long']
