"""Direct libcamera or shared runtime frames -> encoder -> RTP, without OSD."""

from fractions import Fraction
import ipaddress
import secrets
import time
import uuid

from .wire import Fault, choice, number, udp_socket


def video_spec(args):
    backend = choice(args, "backend", "direct-gst", ("direct-gst", "runtime"))
    sensor = args.get("sensor", {})
    output = args.get("output", {})
    codec = args.get("codec", {})
    destination = args.get("destination", {})
    if not all(isinstance(v, dict) for v in (sensor, output, codec, destination)):
        raise Fault("invalid_argument", "sensor/output/codec/destination must be maps")
    sw = number(sensor, "width", 1600, 320, 4096, True)
    sh = number(sensor, "height", 1300, 240, 4096, True)
    depth = choice(sensor, "depth", 10, (8, 10))
    if backend == "runtime" and sensor:
        raise Fault("invalid_argument", "Runtime sensor belongs to camera-worker; omit sensor")
    width = number(output, "width", 800, 160, 1600, True)
    height = number(output, "height", 650, 120, 1300, True)
    if width % 2 or height % 2 or width > sw or height > sh:
        raise Fault("invalid_argument", "Output must be even-sized and no larger than sensor")
    fps = number(output, "fps", 60, 1, 120)
    name = choice(codec, "name", "h264", ("h264", "jpeg"))
    if name == "jpeg" and (width % 8 or height % 8):
        raise Fault("invalid_argument", "RTP/JPEG dimensions must be multiples of 8; use e.g. 800x648")
    if name == "jpeg" and "bitrate" in codec:
        raise Fault("not_supported", "JPEG bitrate is not controlled by this backend")
    if backend == "runtime" and (width > 800 or height > 650):
        raise Fault("invalid_argument", "Runtime output cannot exceed 800x650")
    return {"backend": backend, "sensor": {"width": sw, "height": sh, "depth": depth},
            "output": {"width": width, "height": height, "fps": fps},
            "codec": ({"name": name, "bitrate": number(codec, "bitrate", 2000000, 100000, 20000000, True)}
                      if name == "h264" else {"name": name}),
            "destination": {"rtp_port": number(destination, "rtp_port", 5004, 1024, 65535, True)},
            "mtu": number(args, "mtu", 1400, 576, 1400, True)}


def pipeline_description(spec, test_source=False):
    sensor, output, codec = spec["sensor"], spec["output"], spec["codec"]
    fps = Fraction(str(output["fps"])).limit_denominator(1001)
    source = "videotestsrc name=camera is-live=true" if test_source else (
        "libcamerasrc name=camera sensor-config=\"sensor/config,"
        f"width={sensor['width']},height={sensor['height']},depth={sensor['depth']}\"")
    caps = f"video/x-raw,format=NV12,width={output['width']},height={output['height']},framerate={fps.numerator}/{fps.denominator}"
    convert = "videoconvert" if test_source else "v4l2convert disable-passthrough=true"
    if spec["backend"] == "runtime":
        source = "appsrc name=frames is-live=true format=time block=false max-buffers=2 max-bytes=0 leaky-type=downstream"
        caps = ("video/x-raw,format=BGR,width=800,height=650,"
                f"framerate={fps.numerator}/{fps.denominator}")
        convert = "videoconvert ! videoscale" if test_source else "v4l2convert disable-passthrough=true"
    converted = "video/x-raw,format=I420"
    if spec["backend"] == "runtime":
        converted += f",width={output['width']},height={output['height']}"
    if codec["name"] == "h264":
        encoder = (f"x264enc name=encoder tune=zerolatency bitrate={codec['bitrate']//1000} key-int-max=30" if test_source else
                   "v4l2h264enc name=encoder extra-controls=\"controls,video_bitrate_mode=0,"
                   f"video_bitrate={codec['bitrate']},repeat_sequence_header=1\"")
        encoded = ("video/x-h264,profile=high,level=(string)4.2 ! "
                   "rtph264pay name=pay config-interval=1 pt=96")
    else:
        encoder = "jpegenc name=encoder" if test_source else "v4l2jpegenc name=encoder"
        encoded = "rtpjpegpay name=pay pt=26"
    return (f"{source} ! {caps} ! queue leaky=downstream max-size-buffers=2 max-size-bytes=0 max-size-time=0 "
            f"! {convert} ! {converted} ! {encoder} ! {encoded} mtu={spec['mtu']} "
            "! udpsink name=network sync=false async=false close-socket=false")


class Streams:
    def __init__(self, config, emit, log):
        self.emit, self.log = emit, log
        self.simulated = config.get("simulate", False) and not config.get("test_video", False)
        self.test_source = config.get("test_video", False)
        self.streams = {}
        self.active = None
        self.pipeline = self.gsocket = self.Gst = self.Gio = None
        self.started_at = 0
        self.packets = 0
        self.last_packet = 0
        self.runtime = None

    def _gst(self):
        if self.Gst is None:
            import gi
            gi.require_version("Gst", "1.0")
            from gi.repository import Gst, Gio
            Gst.init(None)
            self.Gst, self.Gio = Gst, Gio

    def state(self):
        active = self.streams.get(self.active)
        return {"state": "ready", "active_stream": self.active,
                "video_state": active["state"] if active else "stopped",
                "packets": self.packets, "simulated": self.simulated,
                "backend": active["spec"]["backend"] if active else None,
                "frames_submitted": self.runtime.submitted if self.runtime else 0,
                "frames_skipped": self.runtime.skipped if self.runtime else 0}

    def command(self, op, args):
        if op == "state":
            return self.state()
        if op in ("camera.capabilities", "video.capabilities"):
            return {"backends": ["direct-gst", "runtime"], "codecs": ["h264", "jpeg"],
                    "sensor_default": {"width": 1600, "height": 1300, "depth": 10},
                    "output_default": {"width": 800, "height": 650, "fps": 60},
                    "sensor_modes_probed": False, "settings_verified": False,
                    "exact_osd": False, "max_active": 1, "live_update": [], "rtcp": False}
        if op == "video.stop_runtime":
            if self.active and self.streams[self.active]["spec"]["backend"] == "runtime":
                ident = self.active
                self._stop()
                self.emit("video.stopped", {"stream_id": ident, "reason": "camera_stopped"})
            return self.state()
        if op == "video.create":
            if len(self.streams) >= 8:
                raise Fault("busy", "Stream definition limit reached", True)
            spec = video_spec(args)
            host = str(ipaddress.IPv4Address(args["host"]))
            ident = uuid.uuid4().hex
            codec = spec["codec"]["name"]
            info = {"stream_id": ident, "state": "created", "spec": spec, "host": host,
                    "ssrc": secrets.randbits(32), "payload_type": 96 if codec == "h264" else 26,
                    "clock_rate": 90000, "encoding_name": "H264" if codec == "h264" else "JPEG",
                    "exact_osd": False, "error": None}
            self.streams[ident] = info
            return info
        ident = args.get("stream_id")
        if ident not in self.streams:
            raise Fault("not_found", "Unknown stream")
        info = self.streams[ident]
        if op == "video.status":
            return info | {"packets": self.packets if ident == self.active else 0}
        if op == "video.start":
            if self.active == ident:
                return info
            if self.active:
                raise Fault("camera_busy", "Another stream owns the encoder")
            if info["spec"]["backend"] == "runtime":
                duration = number(args, "source_frame_duration_us", None, 8333, 100000, True)
                if info["spec"]["output"]["fps"] > 1000000 / duration * 1.001:
                    raise Fault("invalid_argument", "Video FPS cannot exceed runtime camera FPS")
            self._start(info)
            return info
        if op in ("video.stop", "video.destroy"):
            if self.active == ident:
                self._stop()
            info["state"] = "stopped"
            if op == "video.destroy":
                del self.streams[ident]
            return {"stream_id": ident, "state": "destroyed" if op.endswith("destroy") else "stopped"}
        if op == "video.update":
            if info["spec"]["codec"]["name"] != "h264":
                raise Fault("not_supported", "Bitrate update is H.264-only")
            if self.active == ident:
                raise Fault("restart_required", "Stop video before bitrate update")
            info["spec"]["codec"]["bitrate"] = number(args, "bitrate", None, 100000, 20000000, True)
            return info
        raise Fault("not_supported", op)

    def _start(self, info):
        self.active = info["stream_id"]
        self.started_at = time.monotonic()
        self.packets = self.last_packet = 0
        info.update(state="starting", error=None)
        if self.simulated:
            return
        try:
            self._gst()
            launch = pipeline_description(info["spec"], self.test_source)
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
            sink.set_property("host", info["host"])
            sink.set_property("port", info["spec"]["destination"]["rtp_port"])
            pay = self.pipeline.get_by_name("pay")
            pay.set_property("ssrc", info["ssrc"])
            pay.get_static_pad("src").add_probe(
                self.Gst.PadProbeType.BUFFER | self.Gst.PadProbeType.BUFFER_LIST, self._packet)
            if self.pipeline.set_state(self.Gst.State.PLAYING) == self.Gst.StateChangeReturn.FAILURE:
                raise Fault("pipeline_error", "GStreamer refused PLAYING")
            if info["spec"]["backend"] == "runtime":
                from .runtime_video import RuntimeVideo
                self.runtime = RuntimeVideo(self.pipeline.get_by_name("frames"), self.Gst,
                                            info["spec"]["output"]["fps"])
        except Exception as exc:
            self._fail(str(exc))
            raise Fault("pipeline_error", str(exc)) from exc

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
        info = self.streams[self.active]
        if self.runtime and self.runtime.error:
            self._fail(self.runtime.error)
            return
        if self.simulated:
            if time.monotonic() - self.started_at > 0.05 and info["state"] == "starting":
                info["state"] = "running"
                self.emit("video.started", dict(info))
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
        if info["state"] == "starting" and self.packets:
            info["state"] = "running"
            caps = self.pipeline.get_by_name("encoder").get_static_pad("sink").get_current_caps()
            if caps:
                info["negotiated_caps"] = caps.to_string()[:300]
            self.emit("video.started", dict(info))
        if time.monotonic() - (self.last_packet or self.started_at) > 10:
            self._fail("No RTP buffers for 10 seconds")

    def _fail(self, reason):
        ident = self.active
        self._stop()
        if ident in self.streams:
            self.streams[ident].update(state="failed", error=reason[:240])
            self.emit("video.failed", {"stream_id": ident, "error": reason[:240]})

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
        if self.active in self.streams:
            self.streams[self.active]["state"] = "stopped"
        self.active = None

    def close(self):
        self._stop()
        self.streams.clear()
