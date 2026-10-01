from collections import deque
from contextlib import contextmanager
from types import SimpleNamespace as NS

import numpy as np
import pytest

from roki_ng import dataplane, field_observations, localisation_debug, localisation_worker
from roki_ng.parameters import SCHEMA


def narrow_turf():
    image = np.full((650, 800, 3), 180, np.uint8)
    for x in range(20, 800, 80):
        image[:, x:x + 8] = (40, 140, 45)
    return image


@pytest.mark.parametrize('automatic', [True, False])
def test_empty_eroded_turf_is_missing_observation(automatic):
    params = {k: v[1] for k, v in SCHEMA.items()}
    params['vision.field_auto'] = automatic
    with pytest.raises(ValueError, match='after erosion'):
        field_observations.runtime_paint_mask(narrow_turf(), params)


def test_clustering_checks_sample_count():
    for count in (0, 1, 2):
        with pytest.raises(ValueError, match='Insufficient pixels'):
            field_observations.clusters(np.zeros((count, 3), np.float32), 3)


def test_channel_checks_current_subscriber_count():
    channel = object.__new__(dataplane.Channel)
    count = [0]
    channel.service = NS(dynamic_config=lambda: NS(number_of_subscribers=lambda: count[0]))
    for value in (0, 1, 2, 0, 1):
        count[0] = value
        assert channel.has_subscribers() is (value > 0)


@pytest.mark.parametrize('failure', ['frame', 'render', 'publisher', 'loan',
                                     'not_requested', 'no_subscriber', 'subscription_changes', 'stop_during_render'])
def test_localisation_continues_after_bad_frame_or_video_failure(tmp_path, monkeypatch, failure):
    logs, events, rendered, published, channels = [], [], [], [], {}
    worker = localisation_worker.Localisation(
        {'state_dir': str(tmp_path)}, lambda *a: events.append(a), lambda *a: logs.append(a))
    if failure != 'not_requested':
        worker.video_requested.set()
    clock = [1.0]
    monkeypatch.setattr(localisation_worker, 'time', NS(monotonic=lambda: clock[0]))

    class WaitSet:
        calls = 0

        def attach_deadline(self, *args):
            return object()

        def wait_and_process(self):
            clock[0] += .3
            if self.calls == (5 if failure == 'subscription_changes' else 2):
                worker.stop_event.set()
                return
            seq = self.calls
            self.calls += 1
            image = narrow_turf() if failure == 'frame' and seq == 0 else np.zeros((650, 800, 3), np.uint8)
            header = dataplane.FRAME_HEADER.pack(seq, int(clock[0] * 1e9), 800, 650, 2400)
            channels[dataplane.FRAME_TOPIC].queue.append(header + image.tobytes())
            channels[dataplane.IMU_TOPIC].queue.append(dataplane.IMU_RECORD.pack(seq, 0, 0, 0, 0, 1, 0))

    waitset = WaitSet()
    iox = NS(WaitSetBuilder=NS(new=lambda: NS(create=lambda *_: waitset)),
             ServiceType=NS(Ipc=0), Duration=NS(from_secs=lambda n: n))

    class Channel:
        def __init__(self, topic, *, publisher=False):
            if publisher and failure == 'publisher':
                raise RuntimeError('test publisher failure')
            self.queue = deque()
            self.iox = iox
            self.listener = NS(try_wait=lambda: None)
            self.closed = False
            channels[topic] = self

        def receive(self):
            if not self.queue:
                return None
            data = self.queue.popleft()
            return NS(payload=lambda: NS(as_memory_view=lambda: memoryview(data)))

        def has_subscribers(self):
            if failure == 'no_subscriber':
                return False
            if failure == 'subscription_changes':
                return waitset.calls in (2, 4)
            if failure == 'stop_during_render':
                return not rendered
            return True

        @contextmanager
        def loan(self, size):
            if failure == 'loan':
                raise RuntimeError('test loan failure')
            data = bytearray(size)
            yield data
            published.append(dataplane.FRAME_HEADER.unpack_from(data)[0])

        def close(self):
            self.closed = True

    monkeypatch.setattr(dataplane, 'Channel', Channel)
    original_mask = field_observations.runtime_paint_mask

    def mask(image, params):
        if image.any():
            return original_mask(image, params)
        return np.zeros((650, 800), np.uint8)

    monkeypatch.setattr(field_observations, 'runtime_paint_mask', mask)
    monkeypatch.setattr(field_observations, 'detect_circle', lambda *a, **k: None)
    monkeypatch.setattr(field_observations, 'observations', lambda *a, **k: [])

    def render(image, projector, quaternion, lines, circle, posts, result, *args):
        rendered.append(result)
        if failure == 'render':
            raise RuntimeError('test render failure')
        return image

    monkeypatch.setattr(localisation_debug, 'video_frame', render)

    class Engine:
        sequence = -1
        goals, model, circles = [], [], []
        goal_foot_tolerance = .35

        def update(self, seq, *args, **kwargs):
            self.sequence = seq
            return {'frame_sequence': seq, 'valid': False, 'candidate': None}

    engine = Engine()
    worker._run(engine, NS(project=lambda *a: np.zeros((720, 720), np.uint8)), 0, worker.generation)
    assert worker.error is None
    last = 4 if failure == 'subscription_changes' else 1
    assert engine.sequence == last and worker.frames == last + 1
    assert worker.result['frame_sequence'] == last
    assert all(c.closed for c in channels.values())
    worker.tick()
    worker.tick()
    if failure == 'frame':
        assert 'after erosion' in rendered[0]['reason']
        assert published == [0, 1] and not logs
    elif failure in ('no_subscriber', 'not_requested'):
        assert not rendered and not published and not logs
    elif failure == 'subscription_changes':
        assert [r['frame_sequence'] for r in rendered] == [1, 3]
        assert published == [1, 3] and not logs
    elif failure == 'stop_during_render':
        assert len(rendered) == 1 and not published and not logs
    else:
        assert len(logs) == 1 and logs[0][0] == 'WARNING'
        assert 'computation continues' in logs[0][1]
        assert worker.video_error is not None and not published
    assert not events


def test_fatal_computation_error_is_reported_once(tmp_path):
    logs, events = [], []
    worker = localisation_worker.Localisation(
        {'state_dir': str(tmp_path)}, lambda *a: events.append(a), lambda *a: logs.append(a))
    worker.error = 'lost IMU stream'
    worker.tick()
    worker.tick()
    assert logs == [('ERROR', 'Localisation stopped: lost IMU stream')]
    assert events == [('localisation.fault', {'error': 'lost IMU stream'})]
