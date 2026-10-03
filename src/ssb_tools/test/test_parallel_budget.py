import pytest

from ssb_tools import parallel_budget as budget


@pytest.mark.parametrize('cpus,expected', [(1, 1), (4, 4), (8, 8), (24, 8)])
def test_defaults_respect_affinity_not_host_cpu_count(monkeypatch, cpus, expected):
    monkeypatch.setattr(budget.os, 'sched_getaffinity', lambda _: set(range(cpus)))
    monkeypatch.setattr(budget.os, 'cpu_count', lambda: 128)
    assert budget.resolve_workers() == expected
    assert budget.resolve_workers(1) == 1
    with pytest.raises(ValueError): budget.resolve_workers(cpus+1)


@pytest.mark.parametrize('value', [0, -1, 1.5, True, '4'])
def test_invalid_explicit_counts_fail(value):
    with pytest.raises(ValueError, match='worker count'): budget.resolve_workers(value)


def test_affinity_failure_falls_back_to_cpu_count(monkeypatch):
    def unavailable(_): raise OSError('affinity unavailable')
    monkeypatch.setattr(budget.os, 'sched_getaffinity', unavailable)
    monkeypatch.setattr(budget.os, 'cpu_count', lambda: 4)
    assert budget.resolve_workers() == 4
    monkeypatch.setattr(budget.os, 'cpu_count', lambda: None)
    assert budget.resolve_workers() == 1
