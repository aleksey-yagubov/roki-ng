import asyncio
from contextlib import contextmanager
from types import SimpleNamespace as NS
from unittest.mock import AsyncMock

import pytest

from roki_ng.dataplane import IMU_RECORD
from roki_ng.imu import ImuPublisher
from roki_ng.synchronization import CaptureAlignment, SequenceJoiner
from roki_ng.supervisor import Supervisor, Session
from roki_ng.wire import Fault


def test_join_requires_alignment_and_clears_old_samples():
    join = SequenceJoiner(capacity=2)
    assert join.frame(10, "untrusted") is None
    assert join.measurement(10, "untrusted") is None
    join.set_alignment(4)
    assert join.frame(14, "frame") is None
    assert join.measurement(10, "exact") == ("frame", "exact")
    assert join.measurement(11, "next") is None
    assert join.frame(15, "next frame") == ("next frame", "next")
    for seq in (16, 17, 18):
        join.frame(seq, seq)
    assert join.dropped == 1
    join.set_alignment(None)
    assert not join.frames and not join.imu
    assert join.measurement(14, "old") is None
    join.set_alignment(0)
    assert join.frame(65539, "wrapped") is None
    assert join.measurement(3, "not the same frame") is None


@pytest.mark.parametrize("offset", [2, 4, 5])
@pytest.mark.parametrize("reverse", [False, True])
def test_alignment_both_arrival_orders_and_timer_wrap(offset, reverse):
    clock = dict(stm_us=0xffff0000, host_ns=1000000000, uncertainty_ns=100000)
    sync = CaptureAlignment(clock, 16667)
    for seq in range(8):
        fall = (clock["stm_us"]+20000+seq*16667) & 0xffffffff
        frame = (seq+offset, clock["host_ns"]+(20000+seq*16667)*1000+400000)
        record = dict(sequence=seq, flags=7, rise_us=(fall-8000)&0xffffffff, fall_us=fall)
        if reverse:
            sync.records([record])
            sync.frame(*frame)
        else:
            sync.frame(*frame)
            sync.records([record])
        assert (sync.offset is not None) == (seq == 7)
    assert sync.offset == offset
    assert sync.state()["max_residual_ns"] == 400000


def test_alignment_rejects_bad_clock_and_invalid_edges():
    with pytest.raises(ValueError):
        CaptureAlignment(dict(uncertainty_ns=1000001), 16667)
    sync = CaptureAlignment(dict(stm_us=0, host_ns=0, uncertainty_ns=0), 16667)
    for i in range(200):
        sync.frame(i, i*16667000)
        sync.records([dict(sequence=i, flags=6, fall_us=i*16667)])
    assert sync.offset is None and len(sync.frames) <= 128


class FakeChannel:
    def __init__(self, *args, **kwargs):
        self.records = []

    @contextmanager
    def loan(self, size):
        view = bytearray(size)
        yield view
        self.records.append(IMU_RECORD.unpack(view))

    def close(self):
        pass


def test_imu_native_batches_invalid_and_out_of_order(monkeypatch):
    monkeypatch.setattr("roki_ng.imu.Channel", FakeChannel)
    frame = NS(Orientation=NS(X=0,Y=0,Z=0,W=1), Timestamp=NS(TimeS=1,TimeNS=0), SensorID=37)
    events = []
    records = [dict(sequence=n, flags=6 if n == 1 else 7, mode=2,
                    rise_us=n*16667, fall_us=n*16667+8000, imu=frame) for n in (0,1,3,2,3)]
    mb = NS(IsConnected=lambda: True, GetStreamDrops=lambda: 0,
            ReadIMUStream=lambda **kw: records)
    imu = ImuPublisher(NS(mb=mb), lambda *args: events.append(args))
    imu.tick()
    imu.tick()
    assert [r[0] for r in imu.channel.records] == [0,3,2]
    assert imu.lost == 1 and imu.published == 3
    assert len(events) == 1 and len(events[0][1]["records"]) == 4
    mb.IsConnected = lambda: False
    with pytest.raises(Fault, match="disconnected"):
        imu.tick()


def server_setup(tmp_path, fail_start=False):
    server = Supervisor({"state_dir": str(tmp_path)})
    server.mode, server.owner, server.lease_epoch = "MANUAL", 1, 1
    calls = []
    async def camera(op, *args, **kwargs):
        calls.append(op)
        return {"prepared": False, "frame_duration_us": 16667}
    async def motherboard(op, *args, **kwargs):
        calls.append(op)
        if op == "imu.start" and fail_start:
            raise Fault("hardware_error", "start failed")
        return {"clock": dict(stm_us=0, host_ns=0, uncertainty_ns=0)}
    server.workers = {"camera": NS(call=camera), "motherboard": NS(call=motherboard),
                      "detection": NS(call=AsyncMock(return_value={})),
                      "stream": NS(call=AsyncMock(return_value={"active_streams": []}))}
    return server, calls, Session(1,1,("127.0.0.1",9999),"test")


@pytest.mark.parametrize("with_imu", [True, False])
def test_supervisor_orders_capture_and_excludes_gst(tmp_path, with_imu):
    async def run():
        server, calls, session = server_setup(tmp_path)
        await server.dispatch(session, "camera.start", {"lease_epoch":1,"with_imu":with_imu})
        assert calls == ["camera.status","camera.prepare","imu.start" if with_imu else "imu.stop","camera.start"]
        calls.clear()
        await server.dispatch(session,"camera.stop",{"lease_epoch":1})
        assert calls == ["camera.stop","imu.stop"]
        assert server.capture_session is None
        server.workers["stream"].call.return_value = {"active_streams":[{"stream_id":"test","backend":"direct-gst"}]}
        with pytest.raises(Fault, match="direct-gst"):
            await server.dispatch(session,"camera.start",{"lease_epoch":1})
    asyncio.run(run())


def test_start_failure_stops_both_workers(tmp_path):
    async def run():
        server, calls, session = server_setup(tmp_path, True)
        with pytest.raises(Fault, match="start failed"):
            await server.dispatch(session,"camera.start",{"lease_epoch":1})
        assert calls == ["camera.status","camera.prepare","imu.start","camera.stop","imu.stop"]
        assert server.capture_session is None
    asyncio.run(run())


def test_supervisor_discards_previous_capture_batches(tmp_path):
    async def run():
        server, calls, _ = server_setup(tmp_path)
        old = object()
        server.capture_session = object()
        server.alignment_pending.append((old, {"records":[]}))
        await server._forward_alignment()
        assert not calls
        server.workers["camera"].call = AsyncMock(return_value={"state":"matched"})
        active = server.capture_session
        server.alignment_pending.append((server.capture_session, {"records":[]}))
        await server._forward_alignment()
        assert calls == ["imu.normal"]
        assert server.capture_session is active
        assert server.workers["camera"].call.call_args_list[1].args == ("camera.sync_confirm",)
    asyncio.run(run())


def test_delayed_fault_does_not_stop_new_capture(tmp_path):
    async def run():
        server, calls, _ = server_setup(tmp_path)
        old = object()
        server.capture_session = object()
        await server._capture_fault(old)
        await server._capture_fault(None)
        assert not calls
        await server._capture_fault(server.capture_session)
        assert calls == ["camera.stop", "imu.stop"]
        assert server.capture_session is None
    asyncio.run(run())


def test_normal_mode_failure_stops_capture(tmp_path):
    async def run():
        server, calls, _ = server_setup(tmp_path)
        server.capture_session = object()
        async def camera(op, *args, **kwargs):
            calls.append(op)
            return {"state": "matched"}
        async def motherboard(op, *args, **kwargs):
            calls.append(op)
            if op == "imu.normal":
                raise Fault("hardware_error", "USB disconnected")
            return {}
        server.workers["camera"].call = camera
        server.workers["motherboard"].call = motherboard
        server.alignment_pending.append((server.capture_session, {"records":[]}))
        await server._forward_alignment()
        assert calls == ["camera.alignment", "imu.normal", "camera.stop", "imu.stop"]
        assert server.capture_session is None
    asyncio.run(run())


def test_alignment_keeps_recent_pairs_during_slow_video_start(tmp_path):
    async def run():
        server, calls, _ = server_setup(tmp_path)
        server.capture_session = object()
        await server.capture_lock.acquire()
        for seq in range(100):
            server.worker_event("motherboard", "imu.alignment", {"records": [{"sequence": seq}]})
        assert len(server.alignment_pending) == 16
        assert server.alignment_pending[0][1]["records"][0]["sequence"] == 84
        assert len(server.tasks) == 1
        server.workers["camera"].call = AsyncMock(return_value={"state": "aligning"})
        server.capture_lock.release()
        await server.alignment_task
        assert server.workers["camera"].call.call_count == 16
        assert not calls
    asyncio.run(run())


@pytest.mark.parametrize("role", ["camera", "motherboard"])
def test_worker_death_stops_related_capture(tmp_path, role):
    async def run():
        server, calls, _ = server_setup(tmp_path)
        server.capture_session = object()
        server.worker_event(role, "worker.fault", {"error": "process exited"})
        await asyncio.gather(*list(server.tasks))
        assert server.mode == "FAULT"
        assert calls == ["camera.stop", "imu.stop"]
        assert server.capture_session is None
    asyncio.run(run())


@pytest.mark.parametrize('reverse',[False,True])
def test_join_extended_counters_never_alias_previous_epoch(reverse):
    join=SequenceJoiner(offset=-4)
    join.measurement(3,'old')
    if reverse:
        assert join.measurement(65539,'new') is None
        assert join.frame(65543,'frame')==('frame','new')
    else:
        assert join.frame(65543,'frame') is None
        assert join.measurement(65539,'new')==('frame','new')


def test_imu_rollover_and_late_previous_epoch(monkeypatch):
    monkeypatch.setattr('roki_ng.imu.Channel',FakeChannel)
    frame=NS(Orientation=NS(X=0,Y=0,Z=0,W=1),Timestamp=NS(TimeS=1,TimeNS=0),SensorID=37)
    records=[dict(sequence=n,flags=1,mode=1,imu=frame) for n in (65534,0,65535,1,0,2)]
    mb=NS(IsConnected=lambda:True,GetStreamDrops=lambda:0,ReadIMUStream=lambda **kw:records)
    imu=ImuPublisher(NS(mb=mb));imu.tick()
    assert [r[0] for r in imu.channel.records]==[65534,65536,65535,65537,65538]
    assert imu.highest==65538


def test_imu_counter_reset_is_not_rollover(monkeypatch):
    monkeypatch.setattr('roki_ng.imu.Channel',FakeChannel)
    mb=NS(IsConnected=lambda:True,GetStreamDrops=lambda:0,ReadIMUStream=lambda **kw:[dict(sequence=0)])
    imu=ImuPublisher(NS(mb=mb));imu.highest=10000
    with pytest.raises(Fault,match='discontinuity'):imu.tick()


def test_imu_continues_across_two_full_counter_periods(monkeypatch):
    monkeypatch.setattr('roki_ng.imu.Channel',FakeChannel)
    frame=NS(Orientation=NS(X=0,Y=0,Z=0,W=1),Timestamp=NS(TimeS=1,TimeNS=0),SensorID=37)
    batch=[]
    mb=NS(IsConnected=lambda:True,GetStreamDrops=lambda:0,ReadIMUStream=lambda **kw:batch)
    imu=ImuPublisher(NS(mb=mb))
    for start in range(0,131200,128):
        batch[:]=[dict(sequence=n%65536,flags=1,mode=1,imu=frame) for n in range(start,start+128)]
        imu.tick()
    assert imu.published==131200
    assert imu.channel.records[-1][0]==131199
    assert len(imu.seen)<=512


def test_start_existing_synchronized_camera_does_not_interrupt_video(tmp_path):
    async def run():
        server,calls,session=server_setup(tmp_path)
        state={'running':True,'prepared':True,'imu_sync':{'state':'synced'}}
        server.workers['camera'].call=AsyncMock(return_value=state)
        server.workers['stream'].call.return_value={'active_streams':[{'backend':'runtime','stream_id':'main'}]}
        assert await server.dispatch(session,'camera.start',{'lease_epoch':1,'with_imu':True})==state
        assert [c.args[0] for c in server.workers['camera'].call.call_args_list]==['camera.status']
        assert not calls
    asyncio.run(run())
