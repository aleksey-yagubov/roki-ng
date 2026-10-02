import socket
import time

import pytest

from roki_ng.stream import StreamPipeline
from roki_ng.video_sources import capture_source, frame_source
from roki_ng.dataplane import FRAME_HEADER
from roki_ng.runtime_video import RuntimeVideo


def test_real_gstreamer_rtp_loopback():
    gi = pytest.importorskip("gi")
    gi.require_version("Gst", "1.0")
    from gi.repository import Gst
    Gst.init(None)
    for name in ("videotestsrc", "x264enc", "rtph264pay", "multiudpsink", "videoconvert"):
        if not Gst.ElementFactory.find(name):
            pytest.skip(f"Missing GStreamer element: {name}")
    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver.bind(("127.0.0.1", 0))
    receiver.setblocking(False)
    second = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    second.bind(('127.0.0.1', 0))
    second.setblocking(False)
    source = capture_source() | {"name": "stream"}
    video = StreamPipeline(source, {"simulate": True, "test_video": True}, lambda *a: None, lambda *a: None)
    try:
        video.configure({"width": 320, "height": 240, "fps": 15})
        start = {'session_id':1, 'host':'127.0.0.1', 'rtp_port':receiver.getsockname()[1]}
        info = video.subscribe(start)
        original_pipeline, original_run = video.pipeline, video.run_id
        video.subscribe(start | {'session_id':2,'rtp_port':second.getsockname()[1]})
        packets = []
        second_packets = []
        deadline = time.monotonic() + 4
        while time.monotonic() < deadline and (len(packets) < 5 or len(second_packets) < 5):
            video.tick()
            for sock, target in ((receiver,packets),(second,second_packets)):
                try:
                    target.append(sock.recv(4096))
                except BlockingIOError:
                    pass
            time.sleep(0.001)
        assert len(packets) >= 5, info
        assert len(second_packets) >= 5
        assert set(packets) & set(second_packets)
        video.unsubscribe(1)
        assert video.pipeline is original_pipeline and video.run_id==original_run
        video.tick()
        assert video.status == "running"
        assert all(len(packet) <= 1400 and packet[0] >> 6 == 2 and packet[1] & 127 == 96 for packet in packets)
        assert int.from_bytes(packets[0][8:12], "big") == info["ssrc"]
        assert video.gsocket.get_option(socket.IPPROTO_IP, 10) == (True, 2)
        video._fail("injected pipeline failure")
        assert video.pipeline is None and video.status == "failed"
        video.subscribe(start)
        assert video.pipeline is not None
    finally:
        video.close()
        receiver.close()
        second.close()


def test_runtime_appsrc_h264_decode_and_colour(monkeypatch):
    gi = pytest.importorskip("gi")
    gi.require_version("Gst", "1.0")
    gi.require_version("GstApp", "1.0")
    from gi.repository import Gst
    Gst.init(None)
    for name in ("appsrc", "x264enc", "rtph264pay", "multiudpsink", "videoconvert",
                 "udpsrc", "rtph264depay", "avdec_h264", "appsink"):
        if not Gst.ElementFactory.find(name):
            pytest.skip(f"Missing GStreamer element: {name}")

    class InjectedFrames(RuntimeVideo):
        def __init__(self, appsrc, gst, fps, topic=None, geometry=(800, 650, 2400)):
            from types import SimpleNamespace
            self.appsrc, self.gst = appsrc, gst
            self.geometry = geometry
            self.period_ns = round(1e9 / fps)
            import threading
            self.rate_lock = threading.Lock()
            self.last_stamp = None
            self.reader = SimpleNamespace(error=None, skipped=0)
            self.submitted = self.rate_skipped = 0
            self.sequence = self.first_stamp = self.next_stamp = None

        def close(self):
            pass

    monkeypatch.setattr("roki_ng.runtime_video.RuntimeVideo", InjectedFrames)
    receiver = Gst.parse_launch(
        'udpsrc name=udp port=0 caps="application/x-rtp,media=video,encoding-name=H264,payload=96,clock-rate=90000" '
        '! rtph264depay ! avdec_h264 ! videoconvert ! video/x-raw,format=BGR '
        '! appsink name=decoded sync=false max-buffers=1 drop=true')
    source = frame_source("Camera", "test/frames", 800, 650, available=True) | {"name": "camera"}
    video = StreamPipeline(source, {"test_video": True}, lambda *a: None, lambda *a: None)
    try:
        receiver.set_state(Gst.State.PLAYING)
        receiver.get_state(Gst.SECOND)
        port = receiver.get_by_name("udp").get_property("port")
        video.configure({"fps": 30, "max_fps": 30})
        info = video.subscribe({'session_id':1,'host':'127.0.0.1','rtp_port':port})
        assert video.pipeline.get_by_name("camera") is None
        frame = bytearray(FRAME_HEADER.size) + bytearray(b"\x00\x00\xff") * (800 * 650)
        sink = receiver.get_by_name("decoded")
        decoded = None
        for seq in range(60):
            FRAME_HEADER.pack_into(frame, 0, seq, 1000000000 + seq * 33333333, 800, 650, 2400)
            video.runtime.push(memoryview(frame))
            video.tick()
            decoded = sink.emit("try-pull-sample", 50000000)
            if decoded:
                break
        assert decoded is not None, info
        caps = decoded.get_caps().get_structure(0)
        assert (caps.get_value("width"), caps.get_value("height")) == (800, 650)
        blue, green, red = decoded.get_buffer().extract_dup(0, 3)
        assert red > 200 and green < 30 and blue < 30, (blue, green, red)
        original_pipeline, original_run = video.pipeline, info['run_id']
        video.configure({'max_fps':5})
        assert video.pipeline is original_pipeline and video.run_id==original_run
        assert video.runtime.period_ns==200000000
        video.stop("source_stopped")
        assert not video.active and video.runtime is None
    finally:
        video.close()
        receiver.set_state(Gst.State.NULL)
