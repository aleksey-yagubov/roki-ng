import socket
import time

import pytest

from roki_ng.stream import StreamPipeline
from roki_ng.dataplane import FRAME_HEADER
from roki_ng.runtime_video import RuntimeVideo


def test_real_gstreamer_rtp_loopback():
    gi = pytest.importorskip("gi")
    gi.require_version("Gst", "1.0")
    from gi.repository import Gst
    Gst.init(None)
    for name in ("videotestsrc", "jpegenc", "rtpjpegpay", "multiudpsink", "videoconvert"):
        if not Gst.ElementFactory.find(name):
            pytest.skip(f"Missing GStreamer element: {name}")
    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver.bind(("127.0.0.1", 0))
    receiver.setblocking(False)
    second = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    second.bind(('127.0.0.1', 0))
    second.setblocking(False)
    video = StreamPipeline({"simulate": True, "test_video": True}, lambda *a: None, lambda *a: None)
    try:
        info = video.command("videostream.create", {"codec": {"name": "jpeg"},
                                              "output": {"width": 320, "height": 240, "fps": 15}})
        start = {"stream_id": info["stream_id"], 'session_id':1, 'host':'127.0.0.1', 'rtp_port':receiver.getsockname()[1]}
        video.command("videostream.start", start)
        original_pipeline, original_run = video.pipeline, info['run_id']
        video.command('videostream.attach', start | {'session_id':2,'rtp_port':second.getsockname()[1]})
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
        video.command('videostream.detach',{'stream_id':info['stream_id'],'session_id':1})
        assert video.pipeline is original_pipeline and info['run_id']==original_run
        video.tick()
        assert info["state"] == "running"
        assert all(len(packet) <= 1400 and packet[0] >> 6 == 2 and packet[1] & 127 == 26 for packet in packets)
        assert int.from_bytes(packets[0][8:12], "big") == info["ssrc"]
        assert video.gsocket.get_option(socket.IPPROTO_IP, 10) == (True, 2)
        video._fail("injected pipeline failure")
        assert video.pipeline is None and info["state"] == "failed"
        video.command("videostream.start", start)
        assert video.pipeline is not None
    finally:
        video.close()
        receiver.close()
        second.close()


def test_runtime_appsrc_jpeg_decode_and_colour(monkeypatch):
    gi = pytest.importorskip("gi")
    gi.require_version("Gst", "1.0")
    gi.require_version("GstApp", "1.0")
    from gi.repository import Gst
    Gst.init(None)
    for name in ("appsrc", "jpegenc", "rtpjpegpay", "multiudpsink", "videoconvert", "videoscale",
                 "udpsrc", "rtpjpegdepay", "jpegdec", "appsink"):
        if not Gst.ElementFactory.find(name):
            pytest.skip(f"Missing GStreamer element: {name}")

    class InjectedFrames(RuntimeVideo):
        def __init__(self, appsrc, gst, fps, topic=None):
            from types import SimpleNamespace
            self.appsrc, self.gst = appsrc, gst
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
        'udpsrc name=udp port=0 caps="application/x-rtp,media=video,encoding-name=JPEG,payload=26,clock-rate=90000" '
        '! rtpjpegdepay ! jpegdec ! videoconvert ! video/x-raw,format=BGR '
        '! appsink name=decoded sync=false max-buffers=1 drop=true')
    video = StreamPipeline({"test_video": True}, lambda *a: None, lambda *a: None)
    try:
        receiver.set_state(Gst.State.PLAYING)
        receiver.get_state(Gst.SECOND)
        port = receiver.get_by_name("udp").get_property("port")
        info = video.command("videostream.create", {"source": "runtime",
            "codec": {"name": "jpeg"}, "output": {"width": 320, "height": 240, "fps": 30}})
        video.command("videostream.start", {"stream_id": info["stream_id"], "source_frame_duration_us": 16667,
                      'session_id':1,'host':'127.0.0.1','rtp_port':port})
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
        assert (caps.get_value("width"), caps.get_value("height")) == (320, 240)
        blue, green, red = decoded.get_buffer().extract_dup(0, 3)
        assert red > 200 and green < 30 and blue < 30, (blue, green, red)
        original_pipeline, original_run = video.pipeline, info['run_id']
        video.command('videostream.update',{'stream_id':info['stream_id'],'max_fps':5})
        assert video.pipeline is original_pipeline and info['run_id']==original_run
        assert video.runtime.period_ns==200000000
        video.command("videostream.stop_runtime", {})
        assert video.active is None and video.runtime is None
    finally:
        video.close()
        receiver.set_state(Gst.State.NULL)
