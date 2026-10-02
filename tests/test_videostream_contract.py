import asyncio
from types import SimpleNamespace as NS
from unittest.mock import AsyncMock

import pytest

from roki_ng.camera import Camera
from roki_ng.client import Client
from roki_ng.dataplane import FRAME_BYTES, FRAME_HEADER
from roki_ng.runtime_video import RuntimeVideo
from roki_ng.stream import Streams
from roki_ng.video_sources import frame_source
from roki_ng.supervisor import Supervisor, Session
from roki_ng.wire import Fault, UDP_LIMIT, envelope, pack


def test_multiclient_protocol_over_udp(tmp_path):
    async def run():
        server = Supervisor({"simulate": True, "state_dir": str(tmp_path), "host": "127.0.0.1", "port": 0})
        await server.start()
        owner = Client("127.0.0.1", server.sock.getsockname()[1])
        viewer = Client("127.0.0.1", server.sock.getsockname()[1])
        try:
            await owner.connect()
            await viewer.connect()
            await owner.request("control.acquire")
            sources = [(await viewer.request("videostream.list", {"offset": i}))["items"][0] for i in range(3)]
            by_name = {s["name"]: s for s in sources}
            assert set(by_name) == {"stream", "camera", "localisation"}
            assert by_name["stream"]["available"]
            assert by_name["camera"]["reason"] == "camera_stopped"
            assert not by_name["localisation"]["available"]
            assert by_name["camera"]["controls"]["width"]["fixed"]
            assert by_name["camera"]["controls"]["max_fps"]["live"]
            assert not (await viewer.request("camera.status"))["prepared"]
            with pytest.raises(Fault):
                await viewer.request("videostream.subscribe", {"name": "stream", "rtp_port": 5006})
            first = await owner.request("videostream.subscribe", {"name": "stream", "rtp_port": 5004})
            repeat = await owner.request("videostream.subscribe", {"name": "stream", "rtp_port": 5004})
            assert repeat["receivers"] == 1 and repeat["run_id"] == first["run_id"]
            attached = await viewer.request("videostream.subscribe", {"name": "stream", "rtp_port": 5006})
            assert attached["receivers"] == 2 and attached["run_id"] == first["run_id"]
            with pytest.raises(Fault) as conflict:
                await owner.request("videostream.subscribe", {"name": "stream", "rtp_port": 5004,
                                                            "settings": {"bitrate": 3000000}})
            assert conflict.value.code == "settings_conflict"
            for op, extra in (("stop", {}), ("update", {"settings": {"bitrate": 3000000}})):
                with pytest.raises(Fault):
                    await viewer.request("videostream." + op, {"name": "stream"} | extra)
            await owner.close()
            await asyncio.sleep(.1)
            status = await viewer.request("videostream.status", {"name": "stream"})
            assert status["receivers"] == 1 and status["state"] == "running"
            assert status["run_id"] == first["run_id"]
            await viewer.request("control.acquire")
            await viewer.request("videostream.stop", {"name": "stream"})
            # Explicit stop drops all subscriptions and queries never restart it.
            status = await viewer.request("videostream.status", {"name": "stream"})
            assert not status["subscribed"] and status["receivers"] == 0 and status["state"] == "stopped"
            await viewer.request("videostream.stop", {"name": "stream"})
            second = await viewer.request("videostream.subscribe", {"name": "stream", "rtp_port": 5006})
            assert second["run_id"] != first["run_id"]
            await viewer.request("control.release")
            stopped = await viewer.request("videostream.unsubscribe", {"name": "stream"})
            assert stopped["state"] == "stopped" and stopped["receivers"] == 0
            assert server.counters["oversize"] == 0
        finally:
            if owner.sock.fileno() >= 0:
                await owner.close()
            await viewer.close()
            await server.close()
    asyncio.run(run())


def test_worker_declares_arbitrary_named_output_and_preview_cleanup(tmp_path):
    async def run():
        server = Supervisor({"state_dir": str(tmp_path)})
        server.owner = server.lease_epoch = 1
        server.mode = "MANUAL"
        session = Session(1, 1, ("127.0.0.1", 8094), "operator")
        viewer = Session(2, 2, ("127.0.0.1", 8095), "observer")
        streams = Streams({"simulate": True}, lambda *a: None, lambda *a: None)
        requests = []
        source = frame_source("Custom preview", "custom/image", 640, 480,
                              available=True, on_demand=True)
        async def producer(op, args=None, **kw):
            if op == "source.list":
                return {"items": [dict(source)]}
            requests.append(op)
            if op == "source.start":
                source["requested"] = True
            if op == "source.stop":
                source["requested"] = False
            return {}
        async def stream_call(op, args=None, **kw):
            if op == "source.list":
                return {"items": [streams.video_source()]}
            return streams.command(op, args or {})
        server.workers = {"stream": NS(alive=True, call=stream_call),
                          "custom_worker": NS(alive=True, call=producer)}
        async def cmd(op, body, who=session):
            return await server.dispatch(who, "videostream." + op, body | {"lease_epoch": 1})
        catalog = await cmd("list", {"offset": 1})
        assert catalog["items"][0]["name"] == "custom_worker"
        assert catalog["items"][0]["settings"]["width"] == 640
        args = {"name": "custom_worker", "rtp_port": 5004}
        await cmd("subscribe", args)
        await cmd("subscribe", args | {"rtp_port": 5006}, viewer)
        assert requests.count("source.start") == 1
        requests.clear()
        await cmd("unsubscribe", {"name": "custom_worker"})
        assert not requests
        await cmd("unsubscribe", {"name": "custom_worker"}, viewer)
        assert requests == ["source.stop"]
        pipeline = streams.pipelines["custom_worker"]
        original = pipeline._start
        pipeline._start = lambda: (_ for _ in ()).throw(Fault("pipeline_error", "test"))
        requests.clear()
        with pytest.raises(Fault):
            await cmd("subscribe", args)
        assert requests == ["source.start", "source.stop"]
        assert not pipeline.receivers and pipeline.status == "failed"
        pipeline._start = original
        await cmd("subscribe", args)
        requests.clear()
        await server._video_fault("stream", "worker.fault")
        assert requests == ["source.stop"]
        # Last worker declaration remains visible if producer disappears.
        server.workers["custom_worker"].alive = False
        listed = (await cmd("list", {"offset": 1}))["items"][0]
        assert listed["name"] == "custom_worker" and not listed["available"]
        streams.close()
    asyncio.run(run())


def test_receiver_deduplication_and_fixed_geometry():
    streams = Streams({"simulate": True}, lambda *a: None, lambda *a: None)
    source = frame_source("Camera", "test/frames", 800, 650, available=True) | {"name": "camera"}
    streams.command("sources.configure", {"items": [source]})
    args = {"name": "camera", "host": "127.0.0.1", "rtp_port": 5004, "session_id": 1}
    streams.command("videostream.subscribe", args)
    streams.command("videostream.subscribe", args | {"session_id": 2})
    pipeline = streams.pipelines["camera"]
    calls = []
    sink = NS(emit=lambda *args: calls.append(args))
    pipeline.pipeline = NS(get_by_name=lambda _: sink, set_state=lambda _: None)
    pipeline.Gst = NS(State=NS(NULL=0))
    streams.command("videostream.unsubscribe", {"name": "camera", "session_id": 1})
    assert not calls and pipeline.active
    streams.command("videostream.unsubscribe", {"name": "camera", "session_id": 2})
    assert calls == [("remove", "127.0.0.1", 5004)] and not pipeline.active


def test_live_max_fps_preserves_timestamps_and_drops_before_allocation(monkeypatch):
    monkeypatch.setattr('roki_ng.runtime_video.FrameReader',lambda *a,**kw:NS(error=None,skipped=0,close=lambda:None))
    buffers=[]
    def allocate(*args):
        obj=NS(fill=lambda *a:None)
        buffers.append(obj)
        return obj
    runtime=RuntimeVideo(NS(emit=lambda *a:0),NS(Buffer=NS(new_allocate=allocate),FlowReturn=NS(OK=0)),10)
    frame=bytearray(FRAME_BYTES)
    def push(seq,ms):
        FRAME_HEADER.pack_into(frame,0,seq,int(ms*1e6),800,650,2400)
        runtime.push(memoryview(frame))
    push(0,1000);push(1,1050)
    runtime.set_max_fps(2)
    push(2,1100);push(3,1490);push(4,1500)
    runtime.set_max_fps(20)
    push(5,1550);push(6,1570)
    assert runtime.submitted==3 and runtime.rate_skipped==4 and len(buffers)==3
    assert [b.pts for b in buffers]==[0,490000000,550000000]
    assert [b.duration for b in buffers]==[100000000,500000000,50000000]


def test_update_validation_is_atomic():
    source = frame_source("Test", "test/frames", 800, 650, available=True) | {"name": "custom"}
    streams = Streams({"simulate": True}, lambda *a: None, lambda *a: None)
    streams.command("sources.configure", {"items": [source]})
    args = {"name": "custom", "session_id": 1, "host": "127.0.0.1", "rtp_port": 5004}
    first = streams.command("videostream.subscribe", args | {"settings": {"max_fps": 2}})
    updated = streams.command("videostream.update", {"name": "custom", "settings": {"max_fps": 5}})
    assert updated["settings"]["max_fps"] == 5 and updated["run_id"] == first["run_id"]
    for bad in (0, 121, True, float("nan")):
        with pytest.raises(Fault):
            streams.command("videostream.update", {"name": "custom", "settings": {"max_fps": bad}})
    with pytest.raises(Fault, match="Stop"):
        streams.command("videostream.update", {"name": "custom", "settings": {"max_fps": 10, "bitrate": 3000000}})
    assert streams.command("videostream.status", {"name": "custom"})["settings"]["max_fps"] == 5
    with pytest.raises(Fault, match="fixed"):
        streams.command("videostream.update", {"name": "custom", "settings": {"height": 648}})
    streams.command("videostream.stop", {"name": "custom"})
    changed = streams.command("videostream.update", {"name": "custom", "settings": {"bitrate": 3000000}})
    assert changed["settings"]["bitrate"] == 3000000


def test_catalog_refresh_preserves_custom_settings_and_stops_changed_geometry():
    from copy import deepcopy

    events = []
    streams = Streams({"simulate": True}, lambda *args: events.append(args), lambda *a: None)
    source = frame_source("Camera", "test/frames", 800, 650, available=True) | {"name": "camera"}
    streams.command("sources.configure", {"items": [source]})
    updated = deepcopy(source)
    updated["settings"].update(fps=30, max_fps=30)
    streams.command("sources.configure", {"items": [updated]})
    pipeline = streams.pipelines["camera"]
    assert pipeline.settings["max_fps"] == 30

    streams.command("videostream.update", {"name": "camera", "settings": {"bitrate": 4000000}})
    streams.command("sources.configure", {"items": [source]})
    assert pipeline.settings["bitrate"] == 4000000
    assert pipeline.settings["max_fps"] == 30
    args = {"name": "camera", "host": "127.0.0.1", "rtp_port": 5004, "session_id": 1}
    started = streams.command("videostream.subscribe", args)
    streams.command("sources.configure", {"items": [source]})
    assert pipeline.active and pipeline.run_id == started["run_id"]

    changed = frame_source("Camera", "test/frames", 640, 480, available=True) | {"name": "camera"}
    streams.command("sources.configure", {"items": [changed]})
    assert pipeline.status == "stopped" and not pipeline.receivers
    assert pipeline.settings["width"] == 640
    assert events[-1] == ("videostream.stopped", {"name": "camera", "reason": "source_changed"})


def test_camera_controls_set_save_and_catalog_bounds(tmp_path):
    async def run():
        server=Supervisor({'state_dir':str(tmp_path)})
        server.require_control=lambda *args:None
        camera=Camera({'parameters':server.params.values},lambda *a:None,lambda *a:None)
        async def call(op,args=None,**kw):return camera.command(op,args or {})
        server.workers={'camera':NS(call=call)}
        value={'camera.analogue_gain':2.0}
        await server.dispatch(None,'camera.controls.set',{'values':value})
        assert camera.gain==2 and server.params.values['camera.analogue_gain']==1
        await server.dispatch(None,'camera.controls.save',{'values':value})
        assert server.params.values['camera.analogue_gain']==2
        for offset in range(0,6,2):
            result=await server.dispatch(None,'camera.controls.list',{'offset':offset})
            assert len(pack(envelope('response','camera.controls.list',{'result':result},session=2**63,token=2**63)))<=UDP_LIMIT
    asyncio.run(run())


def test_camera_control_catalog_with_hardware_ranges():
    camera=Camera({},lambda *a:None,lambda *a:None)
    camera.duration=16667  # camera.prepare sets this before opening libcamera.
    names=('ColourGains','ExposureTime','AnalogueGain','AeEnable','AwbEnable')
    camera.lc=NS(controls=NS(**{name:name for name in names}))
    camera.camera=NS(controls={
        'ColourGains':NS(min=.1,max=8.), 'ExposureTime':NS(min=50,max=100000),
        'AnalogueGain':NS(min=1.,max=8.),'AeEnable':NS(min=False,max=True)})
    catalog=[]
    for offset in range(0,6,2):
        catalog.extend(camera.command('camera.controls.list',{'offset':offset})['items'])
    by_key={item['key']:item for item in catalog}
    assert by_key['camera.ae_enabled']['supported'] is True
    assert by_key['camera.ae_enabled']['min'] is None
    assert by_key['camera.awb_enabled']['supported'] is False
    assert by_key['camera.exposure_us']['max']==camera.duration
    assert by_key['camera.white_balance.blue_gain']['max']==8.
