import pytest
from roki_ng.stream import Streams
from roki_ng.video_sources import capture_source, frame_source
from roki_ng.wire import Fault


def setup():
    streams = Streams({"simulate": True}, lambda *a: None, lambda *a: None)
    sources = [capture_source() | {"name": "stream"},
               frame_source("Camera", "test/camera", 800, 650, available=True) | {"name": "camera"},
               frame_source("Preview", "test/debug", 800, 650, available=True,
                            dependencies=("camera",)) | {"name": "localisation"}]
    streams.command("sources.configure", {"items": sources})
    return streams


def start(streams, name, port=5004):
    return streams.command("videostream.subscribe", {"name": name,
                           "host": "127.0.0.1", "rtp_port": port, "session_id": 1})


def test_preview_and_main_output_are_independent():
    streams = setup()
    assert not streams.state()["active_streams"]
    start(streams, "camera")
    start(streams, "localisation", 5006)
    assert len(streams.state()["active_streams"]) == 2
    streams.command("videostream.stop", {"name": "localisation"})
    assert [s["name"] for s in streams.state()["active_streams"]] == ["camera"]
    start(streams, "localisation", 5006)
    streams.command("videostream.stop_source", {"name": "camera"})
    assert not streams.state()["active_streams"]
    streams.close()


def test_capture_and_duplicate_destinations_remain_exclusive():
    streams = setup()
    start(streams, "camera")
    for name, port, message in [("stream", 5006, "owns"), ("localisation", 5004, "different RTP")]:
        with pytest.raises(Fault, match=message):
            start(streams, name, port)
    assert len(streams.state()["active_streams"]) == 1
    streams.close()
