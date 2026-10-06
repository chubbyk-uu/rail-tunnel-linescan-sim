"""Retention inventory: per-inode attribution and conservative classification."""
import os

from ssb_tools.data_inventory import attribute, classify, walk


def test_hard_links_are_shared_not_double_counted():
    owners = {(1, 1): (100, {'sessions/a'}), (1, 2): (40, {'sessions/a', 'sessions/b'})}
    usage = attribute(owners)
    assert usage['sessions/a'] == dict(logical_inode_bytes=140, exclusive_bytes=100, shared_bytes=40)
    assert usage['sessions/b'] == dict(logical_inode_bytes=40, exclusive_bytes=0, shared_bytes=40)


def test_walk_does_not_follow_symlinks(tmp_path):
    (tmp_path/'kept').mkdir()
    (tmp_path/'kept/file').write_text('x')
    (tmp_path/'outside').mkdir()
    (tmp_path/'outside/big').write_text('y'*1000)
    os.symlink(tmp_path/'outside', tmp_path/'kept/link')
    names = sorted(os.path.basename(path) for path, _ in walk(str(tmp_path/'kept')))
    assert names == ['file', 'link']


def test_any_current_or_retention_reference_keeps_a_directory():
    assert classify({'retention_list': ['docs/DATA_RETENTION.md']}) == 'required'
    assert classify({'history_docs': ['x'], 'current_docs': ['y']}) == 'required'
    assert classify({'symlinked_from_retained': ['sessions/m']}) == 'required'
    assert classify({'media_manifest': ['m'], 'history_docs': ['h']}) == 'media_source'
    assert classify({'evaluation_records': ['e']}) == 'evaluation_referenced'
    assert classify({'history_docs': ['h']}) == 'history_only'
    assert classify({}) == 'unreferenced'
