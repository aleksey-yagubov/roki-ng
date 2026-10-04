"""Deterministic worker time without changing asyncio/IPC integration clocks."""
import time
from types import SimpleNamespace

import pytest


@pytest.fixture
def body_clock(monkeypatch):
    from roki_ng import body, body_imu, body_power, body_servos
    now = [time.monotonic()]
    clock = SimpleNamespace(monotonic=lambda: now[0],
                            monotonic_ns=lambda: int(now[0] * 1e9))
    for module in (body, body_imu, body_power, body_servos):
        monkeypatch.setattr(module, "time", clock)
    original_tick = body.Body.tick

    def tick(instance):
        now[0] += 0.01
        original_tick(instance)

    monkeypatch.setattr(body.Body, "tick", tick)
    return now
