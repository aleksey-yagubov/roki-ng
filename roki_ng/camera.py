"""One libcamera ISP output, DMA-coherent copy directly into an iceoryx2 loan.

Capture sequence follows the UnicamSequence patch, not the ISP/request sequence.
The direct-libcamera pattern was reviewed in roki-next's Soccer/Vision/camera.py.
"""

import fcntl
import mmap
import struct
import time

from .dataplane import Channel, FRAME_TOPIC, FRAME_HEADER, FRAME_BYTES
from .wire import Fault, number, boolean, page
from .synchronization import CaptureAlignment

WB_KEYS = ("camera.white_balance.red_gain", "camera.white_balance.blue_gain")
CONTROL_KEYS={'camera.exposure_us':'ExposureTime','camera.analogue_gain':'AnalogueGain',
              'camera.ae_enabled':'AeEnable','camera.awb_enabled':'AwbEnable'}


class Camera:
    def __init__(self, config, emit, log):
        self.emit, self.log = emit, log
        self.simulated = config.get("simulate", False)
        self.camera = self.manager = self.allocator = self.channel = None
        self.requests, self.mapped = [], {}
        self.running = self.acquired = False
        self.error = None
        self.frames = self.bad_frames = 0
        self.sequence = None
        self.last_frame = 0
        self.alignment = None
        self.synced = False
        self.started_at = 0
        values = config.get("parameters", {})
        self.white_balance = tuple(number(values, key, 1.0, 0.01, 32.0) for key in WB_KEYS)
        self.wb_pending = False
        self.exposure=number(values,'camera.exposure_us',8000,1,100000,True)
        self.gain=number(values,'camera.analogue_gain',1.,1.,16.)
        self.ae=boolean(values,'camera.ae_enabled',False)
        self.awb=boolean(values,'camera.awb_enabled',False)
        self.pending_controls={}
        self.measured=None

    def sync_state(self):
        if self.alignment is None:
            return {"state": "disabled", "unicam_minus_stm": None}
        state = self.alignment.state()
        if self.synced:
            state["state"] = "synced"
        return state

    def state(self):
        return {"state": "fault" if self.error else "ready", "running": self.running,
                "prepared": self.camera is not None, "frames": self.frames,
                "bad_frames": self.bad_frames, "sequence": self.sequence,
                "frame_duration_us": self.duration if self.camera is not None else None,
                "error": self.error, "topic": FRAME_TOPIC, "imu_sync": self.sync_state(),
                "requested_controls":{'exposure_us':self.exposure,'gain':self.gain,
                    'ae_enabled':self.ae,'awb_enabled':self.awb,'colour_gains':self.white_balance},
                "measured_controls":self.measured if self.running else None,
                "measured_age_ms":round((time.monotonic()-self.last_frame)*1000) if self.running and self.measured else None}

    def command(self, op, args):
        if op == 'camera.capabilities':
            return {'sensor':{'width':1600,'height':1300,'depth':10},
                    'output':{'width':800,'height':650,'format':'BGR'},
                    'geometry_mutable':False, 'imu_required':True,
                    'frame_duration_us':{'min':8333,'max':100000,'default':16667,'apply':'next_start'},
                    'controls_probed':self.camera is not None}
        if op == 'camera.controls.list':
            from .parameters import SCHEMA
            values = dict(zip(WB_KEYS, self.white_balance)) | {
                'camera.exposure_us':self.exposure, 'camera.analogue_gain':self.gain,
                'camera.ae_enabled':self.ae, 'camera.awb_enabled':self.awb}
            items = []
            for key,value in values.items():
                kind,default,low,high,_,description = SCHEMA[key]
                supported = None
                if self.camera is not None:
                    control = getattr(self.lc.controls, CONTROL_KEYS.get(key, 'ColourGains'))
                    info = self.camera.controls.get(control)
                    supported = info is not None
                    if info is not None and kind != 'bool':
                        low, high = max(low, info.min), min(high, info.max)
                    if key == 'camera.exposure_us':
                        high = min(high, self.duration)
                items.append({'key':key,'type':kind,'default':default,'min':low,'max':high,
                              'value':value,'supported':supported,'apply':'next_request',
                              'description':description})
            return page(items, args, 2)
        if op == "params.apply":
            if any(key not in WB_KEYS and key not in CONTROL_KEYS for key in args):
                raise Fault("invalid_argument", "Unknown camera parameter")
            previous=dict(zip(WB_KEYS,self.white_balance)) | {
                'camera.exposure_us':self.exposure,'camera.analogue_gain':self.gain,
                'camera.ae_enabled':self.ae,'camera.awb_enabled':self.awb}
            pair = tuple(number(args, key, old, 0.01, 32.0)
                         for key, old in zip(WB_KEYS, self.white_balance))
            if self.camera is not None:
                self._validate_white_balance(pair)
            exposure=number(args,'camera.exposure_us',self.exposure,1,
                            self.duration if self.camera is not None else 100000,True)
            gain=number(args,'camera.analogue_gain',self.gain,1.,16.)
            ae=boolean(args,'camera.ae_enabled',self.ae)
            awb=boolean(args,'camera.awb_enabled',self.awb)
            pending={}
            for key,name in CONTROL_KEYS.items():
                if key not in args:continue
                if self.camera is not None:
                    control=getattr(self.lc.controls,name)
                    info=self.camera.controls.get(control)
                    if info is None:raise Fault('not_supported',f'Camera does not expose {name}')
                    if not info.min<=args[key]<=info.max:raise Fault('invalid_argument',f'{name}: outside sensor range')
                pending[name]=args[key]
            self.white_balance = pair
            self.wb_pending = self.wb_pending or any(key in args for key in WB_KEYS)
            self.exposure,self.gain,self.ae,self.awb=exposure,gain,ae,awb
            self.pending_controls.update(pending)
            return {'previous':{key:previous[key] for key in args}}
        if op == "camera.status":
            return self.state()
        if op == "camera.stop":
            self.close()
            return self.state()
        if op == "camera.alignment":
            if self.running and self.alignment:
                self.alignment.records(args["records"])
            return self.sync_state()
        if op == "camera.sync_confirm":
            if not self.running or not self.alignment or self.alignment.offset is None:
                raise Fault("invalid_state", "Camera alignment not established")
            self.synced = True
            self.log("INFO", f"Camera/IMU synced: Unicam-STM={self.alignment.offset}")
            return self.sync_state()
        if op == "camera.prepare":
            if self.camera is not None:
                raise Fault("busy", "Camera already prepared")
            if self.simulated:
                raise Fault("not_supported", "Capture requires real libcamera")
            self.duration = number(args, "frame_duration_us", 16667, 8333, 100000, True)
            self.exposure = number(args, "exposure_us", self.exposure, 1, self.duration, True)
            self.gain = number(args, "gain", self.gain, 1.0, 16.0)
            try:
                self._prepare()
            except Exception:
                self.close()
                raise
            return self.state() | {"frame_duration_us": self.duration}
        if op == "camera.start":
            if self.camera is None or self.running:
                raise Fault("invalid_state", "Prepare camera before start")
            try:
                if not args.get('clock'):
                    raise Fault('not_ready', 'Synchronized capture requires motherboard clock')
                self.alignment = CaptureAlignment(args["clock"], self.duration)
                self.synced = False
                c = self.lc.controls
                self.camera.start({c.AeEnable: self.ae, c.AwbEnable: self.awb,
                                   c.ColourGains: self.white_balance,
                                   c.ExposureTime: self.exposure, c.AnalogueGain: self.gain,
                                   c.FrameDurationLimits: (self.duration, self.duration)})
                self.wb_pending = False
                self.pending_controls={};self.measured=None
                self.running = True
                self.last_frame = time.monotonic()
                self.started_at = self.last_frame
                for request in self.requests:
                    self.camera.queue_request(request)
            except Exception:
                self.close()
                raise
            return self.state()
        raise Fault("not_supported", op)

    def _prepare(self):
        import libcamera as lc
        import numpy as np
        self.lc, self.np = lc, np
        self.sequence_control = lc.controls.rpi.UnicamSequence
        self.manager = lc.CameraManager.singleton()
        if not self.manager.cameras:
            raise Fault("camera_missing", "No libcamera cameras")
        self.camera = self.manager.cameras[0]
        self.camera.acquire()
        self.acquired = True
        config = self.camera.generate_configuration([lc.StreamRole.VideoRecording])
        main = config.at(0)
        main.pixel_format, main.size, main.buffer_count = lc.PixelFormat("RGB888"), lc.Size(800, 650), 4
        sensor = lc.SensorConfiguration()
        sensor.output_size, sensor.bit_depth = lc.Size(1600, 1300), 10
        config.sensor_config = sensor
        if config.validate() == lc.CameraConfiguration.Status.Invalid:
            raise Fault("camera_config", "Invalid libcamera configuration")
        if str(main.pixel_format) != "RGB888" or (main.size.width, main.size.height) != (800, 650):
            raise Fault("camera_config", "ISP adjusted output away from 800x650 BGR")
        actual = config.sensor_config
        if (actual.output_size.width, actual.output_size.height, actual.bit_depth) != (1600, 1300, 10):
            raise Fault("camera_config", "Full sensor RAW10 mode was not retained")
        self.camera.configure(config)
        self._validate_white_balance(self.white_balance)
        self.config, self.stream, self.stride = config, main.stream, main.stride
        self.allocator = lc.FrameBufferAllocator(self.camera)
        self.allocator.allocate(self.stream)
        for i, buffer in enumerate(self.allocator.buffers(self.stream)):
            if len(buffer.planes) != 1:
                raise Fault("camera_config", "Expected one packed BGR plane")
            plane = buffer.planes[0]
            base = plane.offset // mmap.PAGESIZE * mmap.PAGESIZE
            offset = plane.offset - base
            self.mapped[i] = (mmap.mmap(plane.fd, offset + plane.length,
                                       flags=mmap.MAP_SHARED, prot=mmap.PROT_READ,
                                       offset=base), offset)
            request = self.camera.create_request(i)
            request.add_buffer(self.stream, buffer)
            self.requests.append(request)
        if not self.requests:
            raise Fault("camera_config", "No DMA buffers allocated")
        self.channel = Channel(FRAME_TOPIC, publisher=True)
        self.error = None
        self.frames = self.bad_frames = 0
        self.sequence = None
        self.log("INFO", "Camera prepared: sensor 1600x1300 RAW10 -> 800x650 BGR; not started")

    def filenos(self):
        return [self.manager.event_fd] if self.running else []

    def _validate_white_balance(self, pair):
        info = self.camera.controls.get(self.lc.controls.ColourGains)
        if info is None:
            raise Fault("not_supported", "Camera does not expose ColourGains")
        if any(not info.min <= gain <= info.max for gain in pair):
            raise Fault("invalid_argument", "White balance outside camera ColourGains range")

    def ready(self, fd):
        try:
            for request in self.manager.get_ready_requests():
                if request.status != self.lc.Request.Status.Complete:
                    raise Fault("camera_request", "Cancelled request")
                buffer = request.buffers[self.stream]
                if buffer.metadata.status == self.lc.FrameMetadata.Status.Success:
                    self._publish(request, buffer)
                else:
                    self.bad_frames += 1
                request.reuse()
                if self.wb_pending:
                    request.set_control(self.lc.controls.ColourGains, self.white_balance)
                for name,value in self.pending_controls.items():
                    request.set_control(getattr(self.lc.controls,name),value)
                self.camera.queue_request(request)
                self.wb_pending = False
                self.pending_controls={}
        except Exception as exc:
            self._fault(exc)

    def _publish(self, request, buffer):
        sequence = request.metadata.get(self.sequence_control)
        stamp = request.metadata.get(self.lc.controls.SensorTimestamp)
        if not isinstance(sequence, int) or not 0 <= sequence <= 0xffffffff or not isinstance(stamp, int):
            raise Fault("camera_metadata", "Missing UnicamSequence or SensorTimestamp")
        c=self.lc.controls
        self.measured={'sequence':sequence,'sensor_timestamp_ns':stamp,
                       'exposure_us':request.metadata.get(c.ExposureTime),
                       'gain':request.metadata.get(c.AnalogueGain),
                       'colour_gains':request.metadata.get(c.ColourGains)}
        if self.alignment:
            self.alignment.frame(sequence, stamp)
        if buffer.metadata.planes[0].bytes_used < 649 * self.stride + 2400:
            raise Fault("camera_buffer", "Incomplete BGR buffer")
        fd = buffer.planes[0].fd
        fcntl.ioctl(fd, 0x40086200, struct.pack("Q", 1))  # DMA_BUF_SYNC_START | READ
        try:
            source = self.np.ndarray((650, 800, 3), dtype=self.np.uint8,
                                     buffer=self.mapped[request.cookie][0],
                                     offset=self.mapped[request.cookie][1],
                                     strides=(self.stride, 3, 1))
            with self.channel.loan(FRAME_BYTES) as view:
                FRAME_HEADER.pack_into(view, 0, sequence, stamp, 800, 650, 2400)
                target = self.np.ndarray(source.shape, dtype=self.np.uint8,
                                         buffer=view, offset=FRAME_HEADER.size)
                self.np.copyto(target, source)
                del target
            del source
        finally:
            fcntl.ioctl(fd, 0x40086200, struct.pack("Q", 5))  # DMA_BUF_SYNC_END | READ
        self.frames += 1
        self.sequence = sequence
        self.last_frame = time.monotonic()

    def tick(self):
        if self.running and self.alignment and not self.synced and time.monotonic()-self.started_at > 5:
            self._fault(Fault("imu_alignment_timeout", "No confirmed camera/IMU alignment within 5 seconds"))
            return
        if self.running and time.monotonic() - self.last_frame > 3:
            self._fault(Fault("camera_timeout", "No valid frames for 3 seconds"))

    def _fault(self, exc):
        self.error = str(exc)[:240]
        self.log("ERROR", self.error)
        self.close()
        self.emit("camera.fault", self.state())

    def close(self):
        try:
            if self.running:
                self.camera.stop()
                self.manager.get_ready_requests()
        finally:
            self.running = False
            self.alignment = None
            self.synced = False
            self.requests.clear()
            for mapped, _ in self.mapped.values():
                mapped.close()
            self.mapped.clear()
            self.allocator = None
            if self.acquired:
                self.camera.release()
            self.acquired = False
            self.camera = self.manager = None
            if self.channel:
                self.channel.close()
                self.channel = None
