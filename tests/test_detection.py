import asyncio
from types import SimpleNamespace
from unittest.mock import AsyncMock

import pytest

from roki_ng.dataplane import FRAME_HEADER
from roki_ng.detection import Detection, colour_blobs
from roki_ng.parameters import Parameters
from roki_ng.supervisor import Supervisor
from roki_ng.wire import Fault, pack, envelope


def test_lab_blobs_and_minimum_area(tmp_path):
    np = pytest.importorskip("numpy")
    pytest.importorskip("cv2")
    params = Parameters(tmp_path)
    image = np.zeros((30, 40, 3), dtype=np.uint8)
    image[5:15, 4:16] = (0, 140, 255)
    image[0:2, 0:2] = (0, 140, 255)
    blobs, count = colour_blobs(image, params.values, "orange_ball")
    assert count == 1
    assert blobs == [{"rect": [4, 5, 16, 15], "pixels": 120, "center": [9.5, 9.5]}]
    params.set("vision.orange_ball.box_area_min", 200)
    assert colour_blobs(image, params.values, "orange_ball") == ([], 0)


def test_invalid_threshold_range_not_saved(tmp_path):
    params = Parameters(tmp_path)
    before = params.path.read_bytes()
    with pytest.raises(Fault, match="must not exceed"):
        params.set("vision.orange_ball.l_max", 10)
    assert params.path.read_bytes() == before
    assert params.values["vision.orange_ball.l_max"] == 100


def test_detector_frame_sequence_and_bounded_result(tmp_path):
    pytest.importorskip("numpy")
    pytest.importorskip("cv2")
    params = Parameters(tmp_path)
    detector = Detection({"parameters": params.values}, lambda *a: None, lambda *a: None)
    assert detector.reader is None and detector.publisher is None
    frame = bytearray(FRAME_HEADER.size) + bytearray(b"\x00\x8c\xff") * (800 * 650)
    FRAME_HEADER.pack_into(frame, 0, 53, 123456789, 800, 650, 2400)
    detector._consume(memoryview(frame))
    assert detector.result["frame_sequence"] == 53
    assert detector.result["blobs"][0]["rect"] == [0, 0, 800, 650]
    assert detector.result["blobs"][0]["pixels"] == 520000
    assert len(pack(envelope("sample", "data.sample", {"data": detector.state()}))) <= 1200
    with pytest.raises(ValueError, match="restarted"):
        detector._consume(memoryview(frame))
    detector.close()
    assert detector.state()["result"] is None


def test_live_threshold_apply_and_disk_failure_rollback(tmp_path, monkeypatch):
    async def run():
        server = Supervisor({"state_dir": str(tmp_path)})
        worker = SimpleNamespace(call=AsyncMock(return_value={}))
        server.workers["detection"] = worker
        key = "vision.orange_ball.pixels_min"
        await server._set_parameters({key: 80})
        assert worker.call.call_args.args == ("params.apply", {key: 80})
        assert server.params.values[key] == 80

        def disk_full(values):
            raise OSError("disk full")

        monkeypatch.setattr(server.params, "save", disk_full)
        with pytest.raises(OSError, match="disk full"):
            await server._set_parameters({key: 90})
        assert worker.call.call_args.args == ("params.apply", {key: 80})
        assert server.params.values[key] == 80
    asyncio.run(run())


def test_publisher_failure_stops_only_detector(tmp_path):
    import time
    params = Parameters(tmp_path)
    events = []
    detector = Detection({"parameters": params.values}, lambda *a: events.append(a), lambda *a: None)
    closed = []
    detector.reader = SimpleNamespace(error=None, close=lambda: closed.append("reader"))
    def failed_loan(size):
        raise RuntimeError("shared memory unavailable")
    detector.publisher = SimpleNamespace(loan=failed_loan, close=lambda: closed.append("publisher"))
    detector.last_frame_at = time.monotonic()
    detector.result = {"frame_sequence": 1}
    detector.tick()
    assert closed == ["reader", "publisher"]
    assert detector.reader is None and detector.result is None
    assert events[0][0] == "detection.fault"
    assert events[0][1]["error"] == "shared memory unavailable"
