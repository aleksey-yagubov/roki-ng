import msgpack
import asyncio
import pytest

from roki_ng.wire import Fault, UDP_LIMIT, pack, unpack


def test_long_game_job_events_do_not_forward_internal_segment_history(tmp_path):
    from roki_ng.supervisor import Supervisor, Session
    from roki_ng.wire import envelope
    s = Supervisor({'simulate': True, 'state_dir': str(tmp_path)})
    owner = Session(1, 2, ('127.0.0.1', 1), 'owner')
    s.sessions[1], s.owner = owner, 1
    events = []
    s.send = lambda session, kind, op, body: events.append(body)
    segments = [dict(index=i+1, dx_m=.01, dy_m=0., imu_yaw_rad=0.) for i in range(256)]
    job = dict(job_id='long', operation='game.walk', status='completed', progress=256,
               segments=segments, pose='base_stand')
    s.worker_event('motherboard', 'job.completed', job)
    assert len(events) == 1 and 'segments' not in events[0]
    assert len(pack(envelope('event', 'job.completed', events[0]))) < UDP_LIMIT
    assert job['segments'] == segments  # Full internal odometry remains intact.


@pytest.mark.parametrize('size', [1399, 1400, 1401])
def test_complete_datagram_boundary(size):
    assert UDP_LIMIT == 1400
    overhead = len(msgpack.packb({'data': b'x' * 1300}, use_bin_type=True)) - 1300
    message = {'data': b'x' * (size - overhead)}
    raw = msgpack.packb(message, use_bin_type=True)
    assert len(raw) == size
    if size <= UDP_LIMIT:
        assert pack(message) == raw
        assert unpack(raw) == message
    else:
        for operation, argument in ((pack, message), (unpack, raw)):
            with pytest.raises(Fault) as caught:
                operation(argument)
            assert caught.value.code == 'too_large'


def test_reference_client_receives_full_size_sample():
    from roki_ng.client import Client
    from roki_ng.wire import envelope, udp_socket

    async def run():
        source = udp_socket()
        source.bind(('127.0.0.1', 0))
        client = Client(*source.getsockname())
        reader = asyncio.create_task(client._read())
        try:
            message = envelope('sample', 'data.sample', {'data': b'x' * 1200})
            overhead = len(pack(message)) - 1200
            message['body']['data'] = b'x' * (UDP_LIMIT - overhead)
            raw = pack(message)
            assert len(raw) == UDP_LIMIT
            await asyncio.get_running_loop().sock_sendto(
                source, raw, ('127.0.0.1', client.sock.getsockname()[1]))
            received = await asyncio.wait_for(client.events.get(), 1)
            assert received == message
        finally:
            reader.cancel()
            await asyncio.gather(reader, return_exceptions=True)
            await client.close()
            source.close()

    asyncio.run(run())


def test_game_recovery_status_with_unicode_errors_fits_datagram(tmp_path):
    from roki_ng.supervisor import Supervisor
    from roki_ng.wire import envelope
    server = Supervisor({'simulate': True, 'state_dir': str(tmp_path)})
    reason = 'Ошибка связи 🛑 ' * 100
    server.game.info.update(running=True, role='FIRA_penalty_Goalkeeper', entry='goalkeeper',
        state='reconnecting', phase='reconnecting', recovery='waiting_measurement',
        reason=reason, job_id='f'*32,
        position=dict(x_m=1., y_m=1., yaw_rad=2.2, confidence='degraded', reason=reason),
        ball=dict(valid=False, reason=reason, frame_sequence=100000,
                  sensor_timestamp_ns=123456789012345, x_m=2., y_m=1., rect=[100, 100, 200, 200]))
    server.game.info['last_correction'] = dict(frame_sequence=2**32-1,sensor_timestamp_ns=2**60,
                                              pose=[1.,2.,3.],observations=3)
    state = server.game.state()
    assert state['reason'].startswith('Ошибка связи')
    assert len(state['reason'].encode('utf-8')) <= 180
    sample = dict(subscription='game.state', sequence=100000, **server._sample('game.state'))
    for kind, op, body in [('sample', 'data.sample', sample), ('response', 'game.status', {'result': state})]:
        message = envelope(kind, op, body, session=2**63, token=2**63)
        assert unpack(pack(message)) == message
