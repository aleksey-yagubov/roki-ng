"""One H.264 pipeline per worker-declared output, shared by all receivers."""

from copy import deepcopy
from fractions import Fraction
import ipaddress
import secrets
import time
import uuid

from .video_sources import capture_source, settings_for
from .wire import Fault, number, udp_socket


def pipeline_description(source, settings, test_source=False):
    fps = Fraction(str(settings["fps"])).limit_denominator(1001)
    rate = f"framerate={fps.numerator}/{fps.denominator}"
    size = f"width={settings['width']},height={settings['height']}"
    if source["transport"] == "frames":
        capture = "appsrc name=frames is-live=true format=time block=false max-buffers=2 max-bytes=0 leaky-type=downstream"
        caps = f"video/x-raw,format=BGR,{size},{rate}"
    else:
        capture = "videotestsrc name=camera is-live=true" if test_source else (
            'libcamerasrc name=camera sensor-config="sensor/config,'
            f"width={settings['sensor_width']},height={settings['sensor_height']},depth={settings['sensor_depth']}" + '"')
        caps = f"video/x-raw,format=NV12,{size},{rate}"
    convert = "videoconvert" if test_source else "v4l2convert disable-passthrough=true"
    encoder = (f"x264enc name=encoder tune=zerolatency bitrate={settings['bitrate']//1000} key-int-max=30" if test_source else
               'v4l2h264enc name=encoder extra-controls="controls,video_bitrate_mode=0,'
               f"video_bitrate={settings['bitrate']},repeat_sequence_header=1" + '"')
    return (f"{capture} ! {caps} ! queue leaky=downstream max-size-buffers=2 max-size-bytes=0 max-size-time=0 "
            f"! {convert} ! video/x-raw,format=I420,{size} ! {encoder} "
            "! video/x-h264,profile=high,level=(string)4.2 ! rtph264pay name=pay config-interval=1 pt=96 mtu=1400 "
            "! multiudpsink name=network sync=false async=false close-socket=false")


class StreamPipeline:
    def __init__(self, source, config, emit, log):
        self.source = deepcopy(source)
        self.name = source["name"]
        self.settings = settings_for(source, {})
        self.emit, self.log = emit, log
        self.simulated = config.get("simulate", False) and not config.get("test_video", False)
        self.test_source = config.get("test_video", False)
        self.active = False
        self.status = "stopped"
        self.error = self.run_id = self.ssrc = self.negotiated_caps = None
        self.pipeline = self.gsocket = self.Gst = self.Gio = self.runtime = None
        self.receivers = {}
        self.started_at = self.last_packet = self.packets = 0
        self.frame_count = self.fps_count = 0
        self.fps_at = 0
        self.actual_fps = 0.0

    def state(self, session=None):
        return {"name": self.name, "state": self.status, "settings": dict(self.settings),
                "receivers": len(self.receivers), "subscribed": session in self.receivers,
                "run_id": self.run_id, "ssrc": self.ssrc, "payload_type": 96,
                "clock_rate": 90000, "encoding_name": "H264", "mtu": 1400,
                "exact_osd": False, "error": self.error, "simulated": self.simulated,
                "negotiated_caps": self.negotiated_caps,
                "packets": self.packets, "actual_fps": self.actual_fps,
                "frames_submitted": self.runtime.submitted if self.runtime else 0,
                "frames_skipped": self.runtime.skipped if self.runtime else 0}

    def _gst(self):
        if self.Gst is None:
            import gi
            gi.require_version("Gst", "1.0")
            from gi.repository import Gst, Gio
            Gst.init(None)
            self.Gst, self.Gio = Gst, Gio

    def configure(self, changes):
        updated = settings_for(self.source, changes, self.settings)
        changed = {key for key in updated if updated[key] != self.settings[key]}
        if self.active and any(not self.source["controls"][key].get("live") for key in changed):
            raise Fault("restart_required", "Stop the output for all receivers before changing these settings")
        if self.runtime and "max_fps" in changed:
            self.runtime.set_max_fps(updated["max_fps"])
        self.settings = updated

    def subscribe(self, args):
        updated = settings_for(self.source, args.get("settings", {}), self.settings)
        if self.active and updated != self.settings:
            raise Fault("settings_conflict", "Output already running; subscribe without settings")
        self._attach(args)
        if not self.active:
            self.settings = updated
            try:
                self._start()
            except Exception as exc:
                self._fail(str(exc))
                raise Fault("pipeline_error", str(exc)) from exc
        return self.state(args["session_id"])

    def _start(self):
        self.active = True
        self.started_at = time.monotonic()
        self.packets = self.last_packet = self.frame_count = self.fps_count = 0
        self.actual_fps = 0.0
        self.fps_at = time.monotonic()
        self.status, self.error = "starting", None
        self.run_id, self.ssrc = uuid.uuid4().hex, secrets.randbits(32)
        self.negotiated_caps = None
        if self.simulated:
            return
        self._gst()
        launch = pipeline_description(self.source, self.settings, self.test_source)
        self.log("INFO", launch)
        self.pipeline = self.Gst.parse_launch(launch)
        sock = udp_socket()
        fd = sock.detach()
        try:
            self.gsocket = self.Gio.Socket.new_from_fd(fd)
        except Exception:
            import os
            os.close(fd)
            raise
        sink = self.pipeline.get_by_name("network")
        sink.set_property("socket", self.gsocket)
        for endpoint in set(self.receivers.values()):
            sink.emit("add", *endpoint)
        self.pipeline.get_by_name("encoder").get_static_pad("src").add_probe(
            self.Gst.PadProbeType.BUFFER, self._encoded_frame)
        pay = self.pipeline.get_by_name("pay")
        pay.set_property("ssrc", self.ssrc)
        pay.get_static_pad("src").add_probe(
            self.Gst.PadProbeType.BUFFER | self.Gst.PadProbeType.BUFFER_LIST, self._packet)
        if self.pipeline.set_state(self.Gst.State.PLAYING) == self.Gst.StateChangeReturn.FAILURE:
            raise Fault("pipeline_error", "GStreamer refused PLAYING")
        if self.source["transport"] == "frames":
            from .runtime_video import RuntimeVideo
            self.runtime = RuntimeVideo(self.pipeline.get_by_name("frames"), self.Gst,
                                        self.settings["max_fps"], topic=self.source["topic"],
                                        geometry=tuple(self.source["geometry"]))

    def _attach(self, args):
        session = args["session_id"]
        endpoint = (str(ipaddress.IPv4Address(args["host"])), number(args, "rtp_port", None, 1024, 65535, True))
        if session in self.receivers:
            if self.receivers[session] != endpoint:
                raise Fault("conflict", "Unsubscribe before changing destination")
            return
        if len(self.receivers) >= 4:
            raise Fault("busy", "Receiver limit reached")
        if self.pipeline is not None and endpoint not in self.receivers.values():
            self.pipeline.get_by_name("network").emit("add", *endpoint)
        self.receivers[session] = endpoint

    def unsubscribe(self, session):
        endpoint = self.receivers.pop(session, None)
        if endpoint and self.pipeline is not None and endpoint not in self.receivers.values():
            self.pipeline.get_by_name("network").emit("remove", *endpoint)
        if not self.receivers and self.active:
            self.stop("no_receivers")

    def _encoded_frame(self, pad, probe):
        self.frame_count += 1
        return self.Gst.PadProbeReturn.OK

    def _packet(self, pad, probe):
        if probe.type & self.Gst.PadProbeType.BUFFER_LIST:
            self.packets += probe.get_buffer_list().length()
        else:
            self.packets += 1
        self.last_packet = time.monotonic()
        return self.Gst.PadProbeReturn.OK

    def tick(self):
        if not self.active:
            return
        now = time.monotonic()
        if now - self.fps_at >= 1:
            self.actual_fps = round((self.frame_count - self.fps_count) / (now - self.fps_at), 2)
            self.fps_count, self.fps_at = self.frame_count, now
        if self.runtime and self.runtime.error:
            self._fail(self.runtime.error)
            return
        if self.simulated:
            if self.status == "starting" and now - self.started_at > .05:
                self.status = "running"
                self.emit("videostream.started", {"name": self.name, "run_id": self.run_id})
            return
        bus = self.pipeline.get_bus()
        for _ in range(16):
            message = bus.pop()
            if message is None:
                break
            if message.type == self.Gst.MessageType.ERROR:
                error, debug = message.parse_error()
                self.log("ERROR", f"{error}: {debug}")
                self._fail(str(error))
                return
            if message.type == self.Gst.MessageType.EOS:
                self._fail("Unexpected end of stream")
                return
            if message.type == self.Gst.MessageType.WARNING:
                warning, debug = message.parse_warning()
                self.log("WARNING", f"{warning}: {debug}")
        if self.status == "starting" and self.packets:
            self.status = "running"
            caps = self.pipeline.get_by_name("encoder").get_static_pad("sink").get_current_caps()
            if caps:
                self.negotiated_caps = caps.to_string()[:250]
            self.emit("videostream.started", {"name": self.name, "run_id": self.run_id})
        if now - (self.last_packet or self.started_at) > 10:
            self._fail("No RTP buffers for 10 seconds")

    def _fail(self, reason):
        self._stop()
        self.status, self.error = "failed", reason[:240]
        self.emit("videostream.failed", {"name": self.name, "error": self.error})

    def _stop(self):
        if self.pipeline is not None:
            self.pipeline.set_state(self.Gst.State.NULL)
            self.pipeline = None
        if self.runtime is not None:
            self.runtime.close()
            self.runtime = None
        if self.gsocket is not None:
            self.gsocket.close()
            self.gsocket = None
        self.active = False
        self.receivers.clear()
        self.actual_fps = 0.0

    def stop(self, reason="operator"):
        changed = self.active
        self._stop()
        self.status, self.error = "stopped", None
        if changed:
            self.emit("videostream.stopped", {"name": self.name, "reason": reason})

    def close(self):
        self.stop("shutdown")


class Streams:
    def __init__(self, config, emit, log):
        self.config, self.emit, self.log = config, emit, log
        self.pipelines = {}

    def video_source(self):
        source = capture_source()
        source["publishing"] = any(p.active and p.source["transport"] == "libcamera" for p in self.pipelines.values())
        return source

    def state(self):
        active = [p for p in self.pipelines.values() if p.active]
        return {"state": "ready", "active_streams": [
            {"name": p.name, "transport": p.source["transport"], "state": p.status,
             "dependencies": p.source["dependencies"]} for p in active],
            "packets": sum(p.packets for p in active),
            "frames_submitted": sum(p.runtime.submitted for p in active if p.runtime),
            "frames_skipped": sum(p.runtime.skipped for p in active if p.runtime),
            "simulated": bool(self.config.get("simulate", False))}

    def command(self, op, args):
        if op == "state":
            return self.state()
        if op == "sources.configure":
            for source in args["items"]:
                name = source["name"]
                if name not in self.pipelines:
                    self.pipelines[name] = StreamPipeline(source, self.config, self.emit, self.log)
                else:
                    p = self.pipelines[name]
                    if any(source[k] != p.source[k] for k in ("transport", "topic", "geometry", "controls")):
                        p.stop("source_changed")
                        p.settings = settings_for(source, {})
                    elif not p.active and p.settings == p.source["settings"]:
                        p.settings = settings_for(source, {})
                    p.source = deepcopy(source)
            return {}
        if op == "videostream.capabilities":
            return {"codec": "h264", "max_receivers": 4, "mtu": 1400,
                    "exact_osd": False, "rtcp": False}
        if op == "videostream.detach_session":
            for p in self.pipelines.values():
                p.unsubscribe(args["session_id"])
            return self.state()
        if op == "videostream.stop_source":
            name = args["name"]
            for p in self.pipelines.values():
                if p.name == name or name in p.source["dependencies"]:
                    p.stop("source_stopped")
            return self.state()
        name = args.get("name")
        if name not in self.pipelines:
            raise Fault("not_found", "Unknown video output")
        p = self.pipelines[name]
        if op == "videostream.status":
            return p.state(args.get("session_id"))
        if op == "videostream.unsubscribe":
            p.unsubscribe(args["session_id"])
        elif op == "videostream.stop":
            p.stop()
        elif op == "videostream.update":
            p.configure(args.get("settings", {}))
        elif op == "videostream.subscribe":
            if not p.active:
                if not p.source["available"]:
                    raise Fault("not_ready", p.source["reason"] or "Producer unavailable")
                for other in self.pipelines.values():
                    if other.active and "libcamera" in (other.source["transport"], p.source["transport"]):
                        raise Fault("camera_busy", "Another output owns the capture resource")
            endpoint = (args.get("host"), args.get("rtp_port"))
            for other in self.pipelines.values():
                if other is not p and endpoint in other.receivers.values():
                    raise Fault("invalid_argument", "Concurrent outputs require different RTP ports")
            return p.subscribe(args)
        else:
            raise Fault("not_supported", op)
        return p.state(args.get("session_id"))

    def tick(self):
        for p in self.pipelines.values():
            p.tick()

    def close(self):
        for p in self.pipelines.values():
            p.close()
        self.pipelines.clear()
