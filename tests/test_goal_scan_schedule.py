import asyncio
import time
from unittest.mock import AsyncMock

from test_approach_progress import simple
from test_goal_memory import prepared


def test_valid_goal_memory_skips_goal_sweep(tmp_path):
    s, _ = prepared(tmp_path)
    asyncio.run(s.step())
    assert s.observe.await_args.kwargs == {'goals': False}


def test_fine_alignment_does_not_scan_after_every_jump(tmp_path):
    s, _ = simple(tmp_path, (.3, .052), [])
    s.near = True
    asyncio.run(s.step())
    assert s.observe.await_args.kwargs['goals']
    asyncio.run(s.step())
    assert not s.observe.await_args.kwargs['goals']
    s.last_goal_scan -= 4
    asyncio.run(s.step())
    assert s.observe.await_args.kwargs['goals']


def test_ready_ball_without_permission_moves_instead_of_repeated_scans(tmp_path):
    s, g = simple(tmp_path, (.099, -.052), [])
    s.goal_scan_needed = False
    s.last_goal_scan = time.monotonic()
    asyncio.run(s.step())
    assert not s.observe.await_args.kwargs['goals']
    assert g._move.await_count == 1
    asyncio.run(s.step())
    assert not s.observe.await_args.kwargs['goals']
    assert g._move.await_count == 2


def test_pause_resets_scan_schedule(tmp_path):
    s, _ = simple(tmp_path, (.3, .052), [])
    s.goal_scan_needed = False
    s.last_goal_scan = time.monotonic()
    s.cancel_special('pause')
    asyncio.run(s.step())
    assert not s.observe.await_args.kwargs['goals']


def test_head_search_logs_actual_finding_pose(tmp_path):
    s, g = simple(tmp_path)
    events = []
    g.s.log = lambda *args: events.append(args[-1])
    s.observe.return_value = (None, [])
    s.look = AsyncMock(return_value={'ball': (.3,.1)})
    asyncio.run(s.step())
    assert any('FOUND pan=-1778' in line for line in events)
