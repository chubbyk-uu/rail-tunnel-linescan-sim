"""Repository docs must not carry dead relative links or links that leave the repository."""
from pathlib import Path
import subprocess

from ssb_tools.doc_links import anchors, check, slug

REPO = Path(__file__).resolve().parents[3]


def test_github_slugs_for_chinese_and_punctuated_headings(tmp_path):
    assert slug('5. 生成并运行综合场景（WSL）') == '5-生成并运行综合场景wsl'
    assert slug('D3 连续轨迹优化') == 'd3-连续轨迹优化'
    page = tmp_path/'a.md'
    page.write_text('# Same\n## Same\n```\n# not a heading\n```\n')
    assert anchors(page) == {'same', 'same-1'}


def test_dead_outside_and_anchor_links_are_reported_but_code_and_urls_are_not(tmp_path):
    (tmp_path/'docs').mkdir()
    (tmp_path/'docs/b.md').write_text('# Real heading\n')
    (tmp_path/'docs/a.md').write_text(
        '[ok](b.md#real-heading) [web](https://example.com/x) [self](#top)\n# Top\n'
        '[bad anchor](b.md#missing) [gone](c.md) [out](../../x.md)\n'
        '```\n[in code](nowhere.md)\n```\n`[inline](nowhere.md)`\n')
    problems = check(tmp_path, ['docs/a.md'], ['docs/a.md', 'docs/b.md'])
    assert len(problems) == 3
    assert any('b.md#missing anchor' in p for p in problems)
    assert any('c.md target' in p for p in problems)
    assert any('outside the repository' in p for p in problems)


def test_repository_markdown_links_are_all_valid():
    files = subprocess.check_output(['git', 'ls-files', '*.md'], cwd=REPO, text=True).split()
    tracked = subprocess.check_output(['git', 'ls-files'], cwd=REPO, text=True).split()
    assert files and check(REPO, files, tracked) == []


def test_links_to_untracked_local_files_are_dead_in_clones(tmp_path):
    (tmp_path/'docs').mkdir()
    (tmp_path/'local.pdf').write_text('only on this machine')
    (tmp_path/'docs/a.md').write_text('[pdf](../local.pdf) [dir](../docs)\n')
    assert check(tmp_path, ['docs/a.md']) == [
        'docs/a.md:1: ../local.pdf target is not tracked by git (missing from clones)']
