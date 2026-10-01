"""PC-side UDP control + decoded RTP test against a head-only roki-ng server."""

import argparse
import asyncio
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from roki_ng.client import Client


async def check(args):
    import gi
    gi.require_version("Gst", "1.0")
    gi.require_version("GstApp", "1.0")
    from gi.repository import Gst
    Gst.init(None)
    jpeg = args.codec == "jpeg"
    encoding, pt = ("JPEG", 26) if jpeg else ("H264", 96)
    decode = "rtpjpegdepay ! jpegparse ! jpegdec" if jpeg else "rtph264depay ! h264parse ! avdec_h264"
    pipeline = Gst.parse_launch(
        f'udpsrc name=udp port=0 caps="application/x-rtp,media=video,encoding-name={encoding},payload={pt},clock-rate=90000" '
        f'! {decode} ! appsink name=frames sync=false max-buffers=2 drop=true')
    client = Client(args.robot, args.port)
    owns_capture = False
    ident = None
    try:
        pipeline.set_state(Gst.State.PLAYING)
        pipeline.get_state(Gst.SECOND)
        port = pipeline.get_by_name("udp").get_property("port")
        await client.connect()
        await client.request("control.acquire")
        await client.request("mode.set", {"mode": "MANUAL"})
        await client.request("camera.start")
        owns_capture = True
        if args.detection:
            await client.request("detection.start", {"profile": "orange_ball"})
        info = await client.request("videostream.create", {
            "source": "runtime", "codec": {"name": args.codec},
            "output": {"width": 800, "height": 648 if jpeg else 650, "fps": 30}})
        ident = info["stream_id"]
        sink = pipeline.get_by_name("frames")
        for cycle in range(2):
            await client.request("videostream.start", {"stream_id": ident, "rtp_port":port})
            count = 0
            first_received = None
            deadline = time.monotonic() + 15 + args.frames / 15
            while time.monotonic() < deadline and count < args.frames:
                sample = sink.emit("try-pull-sample", 0)
                if sample:
                    if first_received is None:
                        first_received = time.monotonic()
                    caps = sample.get_caps().get_structure(0)
                    assert caps.get_value("width") == 800
                    assert caps.get_value("height") == (648 if jpeg else 650)
                    count += 1
                else:
                    await asyncio.sleep(0.01)
            state = await client.request("videostream.status", {"stream_id": ident})
            assert count == args.frames and state["state"] == "running", (count, state)
            elapsed = time.monotonic() - first_received
            assert elapsed < args.frames / 10, f"Only {(count - 1) / elapsed:.1f} decoded FPS"
            camera = await client.request("camera.status")
            assert camera["running"] and camera["imu_sync"]["state"] == "synced", camera
            print("DECODED", args.codec, cycle, count, "fps", round((count - 1) / elapsed, 1),
                  state.get("negotiated_caps"), flush=True)
            if args.detection:
                detector = await client.request("detection.status")
                assert detector["running"] and detector["result"] and not detector["error"], detector
                assert detector["result"]["frame_sequence"] <= camera["sequence"] + 10
                print("DETECTION", detector, flush=True)
            if cycle == 0:
                await client.request("videostream.stop", {"stream_id": ident})
                await asyncio.sleep(0.3)
                after = await client.request("camera.status")
                assert after["sequence"] > camera["sequence"] and after["imu_sync"]["state"] == "synced"
                while sink.emit("try-pull-sample", 0):
                    pass
        await client.request("camera.stop")
        owns_capture = False
        assert (await client.request("videostream.status", {"stream_id": ident}))["state"] == "stopped"
        if args.detection:
            assert not (await client.request("detection.status"))["running"]
        print("STOPPED: runtime video followed camera stop", flush=True)
    finally:
        try:
            if ident:
                await client.request("videostream.destroy", {"stream_id": ident})
        finally:
            try:
                if owns_capture:
                    await client.request("camera.stop")
            finally:
                await client.close()
                pipeline.set_state(Gst.State.NULL)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot", required=True)
    parser.add_argument("--port", type=int, default=8093)
    parser.add_argument("--codec", choices=("jpeg", "h264"), default="jpeg")
    parser.add_argument("--detection", action="store_true")
    parser.add_argument("--frames", type=int, default=30)
    args = parser.parse_args()
    if not 30 <= args.frames <= 9000:
        parser.error("frames must be in 30..9000")
    asyncio.run(check(args))
