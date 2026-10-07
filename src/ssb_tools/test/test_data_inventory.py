"""Retention inventory: per-inode attribution and conservative classification."""
import os
import subprocess

import pytest

from ssb_tools.data_inventory import attribute, classify, inventory, walk


def test_hard_links_are_shared_not_double_counted():
    owners = {(1, 1): (100, {'sessions/a'}, 1, 1), (1, 2): (40, {'sessions/a', 'sessions/b'}, 2, 2)}
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


def test_bare_directory_names_in_current_docs_need_review_not_deletion():
    assert classify({'current_docs_by_name': ['docs/SEAM_FUSION.md']}) == 'needs_review'
    assert classify({'retention_list_by_name': ['docs/DATA_RETENTION.md'], 'history_docs': ['h']}) == 'needs_review'
    assert classify({'history_docs_by_name': ['h']}) == 'unreferenced'


@pytest.fixture
def inventory_repo(tmp_path):
    subprocess.run(['git', 'init', '-q', str(tmp_path)], check=True, capture_output=True)
    (tmp_path/'sessions/target').mkdir(parents=True)
    (tmp_path/'local_data').mkdir()
    (tmp_path/'sessions/target/payload').write_bytes(b'x'*8192)
    return tmp_path


def referenced_rows(repo, name):
    (repo/'README.md').write_text('Keep '+name+' for reproducibility\n')
    subprocess.run(['git', '-C', str(repo), 'add', 'README.md'], check=True, capture_output=True)
    return {row['directory']: row for row in inventory(repo)['directories']}


@pytest.mark.parametrize('root', ['sessions', 'local_data'])
def test_referenced_root_symlink_preserves_target_without_double_counting(inventory_repo, root):
    repo = inventory_repo
    (repo/root/'current').symlink_to(repo/'sessions/target', target_is_directory=True)
    rows = referenced_rows(repo, root+'/current')
    assert rows[root+'/current']['class'] == rows['sessions/target']['class'] == 'required'
    assert rows[root+'/current']['kind'] == 'symlink'
    assert rows[root+'/current']['files'] == 1
    assert rows[root+'/current']['logical_inode_bytes'] == 0
    assert rows['sessions/target']['files'] == 1
    assert rows['sessions/target']['references']['symlinked_from_retained'] == [root+'/current']


def test_referenced_root_symlink_chain_preserves_intermediate_alias(inventory_repo):
    repo = inventory_repo
    (repo/'sessions/alias').symlink_to('target', target_is_directory=True)
    (repo/'local_data/current').symlink_to('../sessions/alias', target_is_directory=True)
    rows = referenced_rows(repo, 'local_data/current')
    assert all(rows[name]['class'] == 'required'
               for name in ('local_data/current', 'sessions/alias', 'sessions/target'))


def test_dangling_root_symlink_remains_visible_for_review(inventory_repo):
    repo = inventory_repo
    (repo/'sessions/missing').symlink_to('deleted', target_is_directory=True)
    rows = referenced_rows(repo, 'sessions/missing')
    assert rows['sessions/missing']['kind'] == 'symlink'
    assert rows['sessions/missing']['class'] == 'required'
    assert rows['sessions/missing']['symlink_target'] == str(repo/'sessions/deleted')


@pytest.mark.parametrize('outside_repo', [False, True])
def test_unobserved_hard_link_cannot_count_as_reclaimable(inventory_repo, outside_repo):
    repo = inventory_repo
    retained = repo.parent/(repo.name+'_retained') if outside_repo else repo/'retained'
    retained.mkdir()
    payload = repo/'sessions/target/payload'
    os.link(payload, retained/'payload')
    allocated = payload.stat().st_blocks*512
    rows = referenced_rows(repo, 'sessions/target')
    assert rows['sessions/target']['exclusive_bytes'] == 0
    assert rows['sessions/target']['shared_bytes'] == allocated
    payload.unlink()
    assert (retained/'payload').stat().st_blocks*512 == allocated


def test_all_hard_links_inside_one_directory_are_reclaimable_once(inventory_repo):
    repo = inventory_repo
    payload = repo/'sessions/target/payload'
    os.link(payload, payload.parent/'copy')
    allocated = payload.stat().st_blocks*512
    rows = referenced_rows(repo, 'sessions/target')
    assert rows['sessions/target']['files'] == 2
    assert rows['sessions/target']['logical_inode_bytes'] == allocated
    assert rows['sessions/target']['exclusive_bytes'] == allocated
    assert rows['sessions/target']['shared_bytes'] == 0


def test_hard_links_in_two_scanned_directories_are_shared(inventory_repo):
    repo = inventory_repo
    (repo/'sessions/second').mkdir()
    payload = repo/'sessions/target/payload'
    os.link(payload, repo/'sessions/second/payload')
    allocated = payload.stat().st_blocks*512
    rows = referenced_rows(repo, 'sessions/target')
    for name in ('sessions/target', 'sessions/second'):
        assert rows[name]['logical_inode_bytes'] == rows[name]['shared_bytes'] == allocated
        assert rows[name]['exclusive_bytes'] == 0
