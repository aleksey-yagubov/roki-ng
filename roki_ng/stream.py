"""Direct libcamera or shared runtime frames -> encoder -> RTP, without OSD."""

from fractions import Fraction
import ipaddress
import secrets
import time
import uuid

from .wire import Fault, choice, number, page, udp_socket


def video_spec(args):
    backend = choice(args, "source", "direct-gst", ("direct-gst", "runtime", "localisation"))
    sensor = args.get("sensor", {})
    output = args.get("output", {})
    codec = args.get("codec", {})
    if not all(isinstance(v, dict) for v in (sensor, output, codec)):
        raise Fault("invalid_argument", "sensor/output/codec must be maps")
    if set(args) - {'source', 'sensor', 'output', 'codec', 'max_fps', 'mtu', 'parameters', 'lease_epoch'}:
        raise Fault('invalid_argument', 'Unknown stream setting')
    if args.get('parameters', {}) != {}:
        raise Fault('invalid_argument', 'This source has no configurable output parameters')
    sw = number(sensor, "width", 1600, 320, 4096, True)
    sh = number(sensor, "height", 1300, 240, 4096, True)
    depth = choice(sensor, "depth", 10, (8, 10))
    if backend in ("runtime", "localisation") and sensor:
        raise Fault("invalid_argument", "Runtime sensor belongs to camera-worker; omit sensor")
    width = number(output, "width", 800, 160, 1600, True)
    height = number(output, "height", 650, 120, 1300, True)
    if width % 2 or height % 2 or width > sw or height > sh:
        raise Fault("invalid_argument", "Output must be even-sized and no larger than sensor")
    fps = number(output, "fps", 60, 1, 120)
    max_fps = number(args, 'max_fps', fps, 1, 120)
    if max_fps > fps:
        raise Fault('invalid_argument', 'max_fps cannot exceed output.fps encoder ceiling')
    if backend == 'direct-gst' and 'max_fps' in args:
        raise Fault('invalid_argument', 'Use output.fps for direct-gst capture')
    name = choice(codec, "name", "h264", ("h264", "jpeg"))
    if name == "jpeg" and (width % 8 or height % 8):
        raise Fault("invalid_argument", "RTP/JPEG dimensions must be multiples of 8; use e.g. 800x648")
    if name == "jpeg" and "bitrate" in codec:
        raise Fault("not_supported", "JPEG bitrate is not controlled by this backend")
    if backend in ("runtime", "localisation") and (width > 800 or height > 650):
        raise Fault("invalid_argument", "Runtime output cannot exceed 800x650")
    return {"source": backend, "sensor": {"width": sw, "height": sh, "depth": depth},
            "output": {"width": width, "height": height, "fps": fps},
            "codec": ({"name": name, "bitrate": number(codec, "bitrate", 2000000, 100000, 20000000, True)}
                      if name == "h264" else {"name": name}),
            "max_fps": max_fps, "parameters": {},
            "mtu": number(args, "mtu", 1400, 576, 1400, True)}


def pipeline_description(spec, test_source=False):
    sensor, output, codec = spec["sensor"], spec["output"], spec["codec"]
    fps = Fraction(str(output["fps"])).limit_denominator(1001)
    source = "videotestsrc name=camera is-live=true" if test_source else (
        "libcamerasrc name=camera sensor-config=\"sensor/config,"
        f"width={sensor['width']},height={sensor['height']},depth={sensor['depth']}\"")
    caps = f"video/x-raw,format=NV12,width={output['width']},height={output['height']},framerate={fps.numerator}/{fps.denominator}"
    convert = "videoconvert" if test_source else "v4l2convert disable-passthrough=true"
    if spec["source"] in ("runtime", "localisation"):
        source = "appsrc name=frames is-live=true format=time block=false max-buffers=2 max-bytes=0 leaky-type=downstream"
        caps = ("video/x-raw,format=BGR,width=800,height=650,"
                f"framerate={fps.numerator}/{fps.denominator}")
        convert = "videoconvert ! videoscale" if test_source else "v4l2convert disable-passthrough=true"
    converted = "video/x-raw,format=I420"
    if spec["source"] in ("runtime", "localisation"):
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
            "! multiudpsink name=network sync=false async=false close-socket=false")


class StreamPipeline:
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
        self.receivers = {}
        self.frame_count = 0
        self.fps_at = time.monotonic()
        self.fps_count = 0
        self.actual_fps = 0.0

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
                "source": active["spec"]["source"] if active else None,
                "frames_submitted": self.runtime.submitted if self.runtime else 0,
                "frames_skipped": self.runtime.skipped if self.runtime else 0}

    def command(self, op, args):
        if op == "state":
            return self.state()
        if op == "videostream.capabilities":
            return {"sources": ["direct-gst", "runtime", "localisation"], "codecs": ["h264", "jpeg"],
                    "sensor_default": {"width": 1600, "height": 1300, "depth": 10},
                    "output_default": {"width": 800, "height": 650, "fps": 60},
                    "sensor_modes_probed": False, "settings_verified": False,
                    "exact_osd": False, "live_update": {"runtime": ["max_fps"], "localisation": ["max_fps"], "direct-gst": []}, "rtcp": False}
        if op == "videostream.stop_runtime":
            if self.active and self.streams[self.active]["spec"]["source"] in ("runtime", "localisation"):
                ident = self.active
                self._stop()
                self.emit("videostream.stopped", {"stream_id": ident, "reason": "camera_stopped"})
            return self.state()
        if op == "videostream.create":
            if len(self.streams) >= 8:
                raise Fault("busy", "Stream definition limit reached", True)
            spec = video_spec(args)
            ident = uuid.uuid4().hex
            codec = spec["codec"]["name"]
            info = {"stream_id": ident, "state": "created", "spec": spec, "run_id": None,
                    "ssrc": None, "payload_type": 96 if codec == "h264" else 26,
                    "clock_rate": 90000, "encoding_name": "H264" if codec == "h264" else "JPEG",
                    "exact_osd": False, "error": None}
            self.streams[ident] = info
            return info
        ident = args.get("stream_id")
        if ident not in self.streams:
            raise Fault("not_found", "Unknown stream")
        info = self.streams[ident]
        if op == "videostream.status":
            return info | {"packets": self.packets if ident == self.active else 0,
                           "receivers": len(self.receivers), "actual_fps": self.actual_fps,
                           "attached": args.get('session_id') in self.receivers,
                           "destination": self.receivers.get(args.get('session_id')),
                           "frames_submitted": self.runtime.submitted if self.runtime else 0,
                           "frames_skipped": self.runtime.skipped if self.runtime else 0}
        if op == 'videostream.attach':
            if self.active != ident:
                raise Fault('not_ready', 'Stream is not started')
            self._attach(args)
            return self.command('videostream.status', args)
        if op == 'videostream.detach':
            self._detach(args['session_id'])
            return self.command('videostream.status', args)
        if op == "videostream.start":
            if self.active == ident:
                return self.command('videostream.attach', args)
            if self.active:
                raise Fault("camera_busy", "Another stream owns the encoder")
            if info["spec"]["source"] in ("runtime", "localisation"):
                duration = number(args, "source_frame_duration_us", None, 8333, 100000, True)
                if info["spec"]["max_fps"] > 1000000 / duration * 1.001:
                    raise Fault("invalid_argument", "Video FPS cannot exceed runtime camera FPS")
            self._attach(args)
            try:
                self._start(info)
            except Exception:
                self.receivers.clear()
                raise
            return self.command('videostream.status', args)
        if op in ("videostream.stop", "videostream.destroy"):
            if self.active == ident:
                self._stop()
                self.emit('videostream.stopped', {'stream_id': ident, 'reason': 'operator'})
            info["state"] = "stopped"
            if op == "videostream.destroy":
                del self.streams[ident]
            return {"stream_id": ident, "state": "destroyed" if op.endswith("destroy") else "stopped"}
        if op == "videostream.update":
            keys = set(args) - {'stream_id', 'session_id', 'lease_epoch'}
            if not keys or keys - {'max_fps', 'bitrate'}:
                raise Fault('invalid_argument', 'Provide max_fps and/or bitrate')
            fps = None
            if 'max_fps' in keys:
                if info['spec']['source'] == 'direct-gst':
                    raise Fault('not_supported', 'direct-gst uses fixed capture output.fps')
                fps = number(args, 'max_fps', None, 1, info['spec']['output']['fps'])
            if 'bitrate' in keys:
                if info['spec']['codec']['name'] != 'h264':
                    raise Fault('not_supported', 'Bitrate update is H.264-only')
                if self.active == ident:
                    raise Fault('restart_required', 'Stop stream before bitrate update')
                bitrate = number(args, 'bitrate', None, 100000, 20000000, True)
                info['spec']['codec']['bitrate'] = bitrate
            if fps is not None:
                if self.runtime:
                    self.runtime.set_max_fps(fps)
                info['spec']['max_fps'] = fps
            return self.command('videostream.status', args)
        raise Fault("not_supported", op)

    def _start(self, info):
        self.active = info["stream_id"]
        self.started_at = time.monotonic()
        self.packets = self.last_packet = 0
        info.update(state="starting", error=None, run_id=uuid.uuid4().hex, ssrc=secrets.randbits(32))
        info.pop('negotiated_caps', None)
        self.frame_count = self.fps_count = 0
        self.actual_fps = 0.0
        self.fps_at = time.monotonic()
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
            for endpoint in set(self.receivers.values()):
                sink.emit('add', *endpoint)
            self.pipeline.get_by_name('encoder').get_static_pad('src').add_probe(
                self.Gst.PadProbeType.BUFFER, self._encoded_frame)
            pay = self.pipeline.get_by_name("pay")
            pay.set_property("ssrc", info["ssrc"])
            pay.get_static_pad("src").add_probe(
                self.Gst.PadProbeType.BUFFER | self.Gst.PadProbeType.BUFFER_LIST, self._packet)
            if self.pipeline.set_state(self.Gst.State.PLAYING) == self.Gst.StateChangeReturn.FAILURE:
                raise Fault("pipeline_error", "GStreamer refused PLAYING")
            if info["spec"]["source"] in ("runtime", "localisation"):
                from .runtime_video import RuntimeVideo
                from .dataplane import FRAME_TOPIC, LOCALISATION_TOPIC
                self.runtime = RuntimeVideo(self.pipeline.get_by_name("frames"), self.Gst,
                                            info["spec"]["max_fps"],
                                            topic=LOCALISATION_TOPIC if info["spec"]["source"]=="localisation" else FRAME_TOPIC)
        except Exception as exc:
            self._fail(str(exc))
            raise Fault("pipeline_error", str(exc)) from exc

    def _attach(self, args):
        session = args['session_id']
        endpoint = (str(ipaddress.IPv4Address(args['host'])), number(args, 'rtp_port', None, 1024, 65535, True))
        if session in self.receivers:
            if self.receivers[session] != endpoint:
                raise Fault('conflict', 'Detach before changing destination')
            return
        if len(self.receivers) >= 4:
            raise Fault('busy', 'Receiver limit reached')
        if self.pipeline is not None and endpoint not in self.receivers.values():
            self.pipeline.get_by_name('network').emit('add', *endpoint)
        self.receivers[session] = endpoint

    def _detach(self, session):
        endpoint = self.receivers.pop(session, None)
        if endpoint and self.pipeline is not None and endpoint not in self.receivers.values():
            self.pipeline.get_by_name('network').emit('remove', *endpoint)
        if not self.receivers and self.active:
            ident = self.active
            self._stop()
            self.emit('videostream.stopped', {'stream_id': ident, 'reason': 'no_receivers'})

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
        info = self.streams[self.active]
        now = time.monotonic()
        if now - self.fps_at >= 1:
            self.actual_fps = round((self.frame_count - self.fps_count) / (now - self.fps_at), 2)
            self.fps_count, self.fps_at = self.frame_count, now
        if self.runtime and self.runtime.error:
            self._fail(self.runtime.error)
            return
        if self.simulated:
            if time.monotonic() - self.started_at > 0.05 and info["state"] == "starting":
                info["state"] = "running"
                self.emit("videostream.started", {'stream_id': self.active, 'run_id': info['run_id']})
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
            self.emit("videostream.started", {'stream_id': self.active, 'run_id': info['run_id']})
        if time.monotonic() - (self.last_packet or self.started_at) > 10:
            self._fail("No RTP buffers for 10 seconds")

    def _fail(self, reason):
        ident = self.active
        self._stop()
        if ident in self.streams:
            self.streams[ident].update(state="failed", error=reason[:240])
            self.emit("videostream.failed", {"stream_id": ident, "error": reason[:240]})

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
        self.receivers.clear()
        self.actual_fps = 0.0

    def close(self):
        self._stop()
        self.streams.clear()


class Streams:
    """Shared-frame encoders with receiver fan-out; direct capture is exclusive."""
    def __init__(self,config,emit,log):
        self.config,self.emit,self.log=config,emit,log
        self.pipelines={}

    def state(self):
        active=[p.state() for p in self.pipelines.values() if p.active]
        return {'state':'ready','active_streams':[
            {'stream_id':p['active_stream'],'source':p['source'],'state':p['video_state']}
            for p in active], 'packets':sum(p['packets'] for p in active),
            'frames_submitted':sum(p['frames_submitted'] for p in active),
            'frames_skipped':sum(p['frames_skipped'] for p in active),
            'simulated':bool(self.config.get('simulate',False))}

    def command(self,op,args):
        if op=='state':return self.state()
        if op == 'videostream.capabilities':
            return StreamPipeline(self.config,self.emit,self.log).command(op,args)|{'max_streams':8,'max_receivers':4}
        if op == 'videostream.list':
            return page([{'stream_id': ident, 'source': p.streams[ident]['spec']['source'],
                          'state': p.streams[ident]['state'], 'run_id': p.streams[ident]['run_id'],
                          'receivers': len(p.receivers), 'attached': args.get('session_id') in p.receivers}
                         for ident,p in self.pipelines.items()], args, 4)
        if op == 'videostream.detach_session':
            for p in self.pipelines.values():
                p._detach(args['session_id'])
            return self.state()
        if op in ('videostream.stop_runtime','videostream.stop_localisation'):
            sources=('runtime','localisation') if op=='videostream.stop_runtime' else ('localisation',)
            for p in self.pipelines.values():
                if p.active and p.streams[p.active]['spec']['source'] in sources:
                    ident=p.active;p._stop()
                    self.emit('videostream.stopped',{'stream_id':ident,'reason':'source_stopped'})
            return self.state()
        if op=='videostream.create':
            if len(self.pipelines)>=8:raise Fault('busy','Stream definition limit reached',True)
            p=StreamPipeline(self.config,self.emit,self.log)
            info=p.command(op,args);self.pipelines[info['stream_id']]=p
            return info
        ident=args.get('stream_id')
        if ident not in self.pipelines:raise Fault('not_found','Unknown stream')
        p=self.pipelines[ident]
        if op=='videostream.start' and not p.active:
            spec=p.streams[ident]['spec'];backend=spec['source']
            for other in self.pipelines.values():
                if not other.active:continue
                info=other.streams[other.active];source=info['spec']['source']
                if 'direct-gst' in (backend,source):
                    raise Fault('camera_busy','Another stream owns this source')
        if op in ('videostream.start', 'videostream.attach'):
            endpoint = (args.get('host'), args.get('rtp_port'))
            for other in self.pipelines.values():
                if other is not p and endpoint in other.receivers.values():
                    raise Fault('invalid_argument', 'Concurrent streams require different RTP ports')
        result=p.command(op,args)
        if op=='videostream.destroy':
            p.close();del self.pipelines[ident]
        return result

    def tick(self):
        for p in self.pipelines.values():p.tick()

    def close(self):
        for p in self.pipelines.values():p.close()
        self.pipelines.clear()
