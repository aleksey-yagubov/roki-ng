"""Synchronized, bounded diagnostic localization. Never authorizes motion."""
import json
from pathlib import Path
import threading
import time

from .wire import Fault


class Localisation:
    def __init__(self, config, emit, log):
        self.directory=Path(config['state_dir'])/'localisation'
        self.emit,self.log=emit,log
        self.thread=None
        self.stop_event=threading.Event()
        self.result=None
        self.error=None
        self.last_measurement=0.
        self.frames=0
        self.dropped=0
        self.generation=0
        self.capture_id=None
        from .parameters import SCHEMA
        self.parameters={k:v[1] for k,v in SCHEMA.items()} | config.get('parameters',{})
        self.parameter_revision=0
        self.configuration_id=None
        self.geometry=None
        self.video_error = None
        self.reported_error = None
        self.reported_video_error = None
        self.video_requested = threading.Event()
        self.video_frames = 0
        self.video_last_frame = 0.0
        self.lock = threading.Lock()
        self.game_mode = False
        self.window = None

    def state(self):
        with self.lock:
            result=dict(self.result) if self.result else None
            age=round((time.monotonic()-self.last_measurement)*1000) if result else None
        if result and 'timings_ms' in result:
            # Integer milliseconds bound status packets with four post candidates
            # plus small-mark diagnostics; internal measurements retain precision.
            result['timings_ms']={key:round(value) for key,value in result['timings_ms'].items()}
        if result and (age<0 or age>500):
            result.update(valid=False,reason='stale')
        return {'state':'fault' if self.error else 'ready',
                'running':self.thread is not None and self.thread.is_alive(),
                'mode':'stationary_correction' if self.game_mode else 'visual_pose', 'capture_id':self.capture_id,
                'frames':self.frames,'dropped':self.dropped,'age_ms':age,
                'error':self.error,'result':result,
                'configuration_id':self.configuration_id,'geometry':self.geometry}

    def video_source(self):
        from .dataplane import LOCALISATION_TOPIC
        from .video_sources import frame_source
        state = self.state()
        available = state['running'] and not state['error'] and not self.video_error
        return frame_source('Разметка локализации', LOCALISATION_TOPIC, 800, 650,
                            available=bool(available),
                            reason=None if available else 'localisation_not_ready',
                            publishing=self.video_requested.is_set() and not self.video_error
                                and time.monotonic() - self.video_last_frame < 2,
                            requested=self.video_requested.is_set(), on_demand=True,
                            dependencies=('camera',))

    def command(self, op, args):
        if op == 'localisation.window':
            from .localisation import PoseFilter
            window = args.get('window')
            if not self.game_mode:
                raise Fault('invalid_state', 'No game localisation session')
            if window is not None:
                if (not isinstance(window,dict) or not isinstance(window.get('id'),str)
                        or type(window.get('after_ns')) is not int):
                    raise Fault('invalid_argument', 'Invalid observation window')
                PoseFilter(window.get('prior'), count=256, parameters=self.parameters)
            with self.lock:
                self.window = window
                self.result = None
            return {'window_id': window['id'] if window else None}
        if op in ('source.start', 'source.stop', 'source.status'):
            if op == 'source.start':
                if args.get('parameters', {}) != {}:
                    raise Fault('invalid_argument', 'No source parameters supported')
                if not self.thread or not self.thread.is_alive() or self.error:
                    raise Fault('not_ready', 'Start localisation first')
                if self.video_error:
                    raise Fault('source_fault', 'Diagnostic output failed; restart localisation')
                self.video_requested.set()
            elif op == 'source.stop':
                self.video_requested.clear()
            return {'requested': self.video_requested.is_set(), 'frames': self.video_frames,
                    'publishing': self.video_requested.is_set() and self.video_error is None
                        and time.monotonic() - self.video_last_frame < 2,
                    'age_ms': round((time.monotonic()-self.video_last_frame)*1000) if self.video_frames else None,
                    'error': self.video_error}
        if op=='params.apply':
            from .parameters import validate_colour_ranges
            updated=self.parameters | args
            validate_colour_ranges(updated)
            with self.lock:
                self.parameters=updated;self.parameter_revision+=1;self.result=None
            return {}
        if op in ('state','localisation.status'):
            return self.state()
        if op=='localisation.stop':
            self.close()
            return self.state()
        if op!='localisation.start':
            raise Fault('not_supported',op)
        if self.thread is not None and self.thread.is_alive():
            raise Fault('busy','Localisation already running or stopping')
        # Dependencies/profile checked before any subscriptions are created.
        from .localisation import PoseFilter
        from .ground_projection import GroundProjection
        try:
            profile=json.loads((self.directory/'profile.json').read_text())
            if profile.get('schema')!=1:
                raise ValueError('Unsupported calibration profile schema')
            if profile.get('capture_size')!=[800,650]:
                raise ValueError('Incompatible capture mode')
            self.parameters=self.parameters | args.get('parameters',{})
            from .field_config import configuration_id
            configuration=configuration_id(self.parameters)
            projector=GroundProjection(self.directory,self.parameters['localisation.camera_height_m'])
            engine=PoseFilter(args['prior'],count=2048,parameters=self.parameters)
            offset=args['unicam_minus_stm']
            if type(offset) is not int or abs(offset)>2**32:
                raise ValueError('Invalid capture alignment')
        except (OSError,ValueError,KeyError,TypeError) as exc:
            raise Fault('invalid_argument',str(exc)) from exc
        self.stop_event=threading.Event()
        self.generation+=1
        self.result=self.error=None
        self.video_error = self.reported_error = self.reported_video_error = None
        self.frames=self.dropped=0
        self.video_frames = 0
        self.video_last_frame = 0.0
        self.video_requested.clear()
        self.capture_id=args['capture_id']
        self.game_mode = args.get('mode') == 'game'
        self.window = None
        self.configuration_id=configuration
        self.geometry=dict(self.parameters['field.geometry'])
        self.thread=threading.Thread(target=self._run,args=(engine,projector,offset,self.generation),daemon=True)
        self.thread.start()
        return self.state()

    def _run(self, engine, projector, offset, generation):
        channels=[];guards=[];waitset=None;debug_channel=None
        try:
            import cv2
            import numpy as np
            from .dataplane import Channel,FRAME_TOPIC,IMU_TOPIC,FRAME_HEADER,IMU_RECORD,LOCALISATION_TOPIC,FRAME_BYTES
            from .synchronization import SequenceJoiner
            from .localisation_pipeline import process_frame
            from .ball_observation import capture_time
            cv2.setNumThreads(1)
            channels=[Channel(FRAME_TOPIC),Channel(IMU_TOPIC)]
            iox=channels[0].iox
            waitset=iox.WaitSetBuilder.new().create(iox.ServiceType.Ipc)
            guards=[waitset.attach_deadline(c.listener,iox.Duration.from_secs(1)) for c in channels]
            joiner=SequenceJoiner(capacity=128)
            joiner.set_alignment(offset)
            last_pair=time.monotonic();next_compute=0.
            active_window = None
            while not self.stop_event.is_set():
                waitset.wait_and_process()
                pair=None
                for index,channel in enumerate(channels):
                    channel.listener.try_wait()
                    for _ in range(4 if index==0 else 128):
                        sample=channel.receive()
                        if sample is None:break
                        payload=sample.payload();view=payload.as_memory_view().cast('B')
                        try:
                            if index:
                                rec=IMU_RECORD.unpack(view)
                                matched=joiner.measurement(rec[0],rec)
                            else:
                                rec=FRAME_HEADER.unpack_from(view)
                                if rec[2:]!=(800,650,2400) or len(view)!=FRAME_HEADER.size+650*2400:
                                    raise ValueError('Invalid frame dimensions')
                                owned=bytes(view[FRAME_HEADER.size:])
                                captured=capture_time(rec[1],time.clock_gettime_ns(time.CLOCK_BOOTTIME),time.monotonic())
                                matched=joiner.frame(rec[0],(rec,owned,captured))
                                # Separate frame memory bound: max 2 copied BGR frames.
                                while len(joiner.frames)>2:
                                    joiner.frames.popitem(last=False);self.dropped+=1
                            if matched is not None:
                                if pair is None or matched[0][0][0]>pair[0][0][0]:pair=matched
                        finally:
                            view.release();del view,payload,sample
                now=time.monotonic()
                if pair is None:
                    if now-last_pair>5:raise RuntimeError('No synchronized pairs for 5 seconds')
                    continue
                last_pair=now
                (header,data,received),imu=pair
                if now<next_compute or not 0<=now-received<=.5:
                    self.dropped+=1;continue
                if header[0]<=engine.sequence:
                    self.dropped+=1;continue
                started=now
                image=np.frombuffer(data,np.uint8).reshape(650,800,3)
                with self.lock:
                    parameters=self.parameters;revision=self.parameter_revision
                    window=self.window
                if self.game_mode:
                    if window is None or header[1]<=window['after_ns']:
                        continue
                    if active_window != window['id']:
                        from .localisation import PoseFilter
                        engine=PoseFilter(window['prior'],parameters=parameters,heading=window['prior'][2])
                        active_window=window['id']
                result,lines,circle,posts=process_frame(image,imu[2:6],projector,engine,parameters,header[0],header[1]/1e9)
                spots=result.pop('_spot_observations',[])
                result.update(sensor_timestamp_ns=header[1],imu_sequence=imu[0],
                              processing_ms=round((time.monotonic()-started)*1000))
                if self.game_mode:
                    result['window_id']=active_window
                result['parameter_revision']=revision
                result['processing_ms']=round((time.monotonic()-started)*1000)
                # Stop invalidates calculations already in flight.
                if self.stop_event.is_set() or self.generation!=generation:break
                with self.lock:
                    if self.stop_event.is_set() or self.generation!=generation:break
                    if revision!=self.parameter_revision or window is not self.window:continue
                    if time.monotonic()-received>.5:
                        result.update(valid=False,reason='stale')
                    self.result=result;self.last_measurement=received;self.frames+=1
                if self.video_requested.is_set() and self.video_error is None:
                    try:
                        if debug_channel is None:
                            debug_channel=Channel(LOCALISATION_TOPIC,publisher=True)
                        # Stream's FrameReader exists only while video is requested.
                        if debug_channel.has_subscribers():
                            from .localisation_debug import video_frame
                            annotated=video_frame(image,projector,imu[2:6],lines,circle,posts,result,engine.model,engine.circles,spots=spots)
                            if self.stop_event.is_set() or self.generation!=generation:break
                            with self.lock:
                                if revision!=self.parameter_revision or window is not self.window:continue
                                if self.stop_event.is_set() or self.generation!=generation:break
                                if self.video_requested.is_set() and debug_channel.has_subscribers():
                                    with debug_channel.loan(FRAME_BYTES) as target:
                                        FRAME_HEADER.pack_into(target,0,*header)
                                        target[FRAME_HEADER.size:]=annotated.tobytes()
                                    self.video_frames += 1
                                    self.video_last_frame = time.monotonic()
                    except Exception as exc:
                        # A failed diagnostic sink must not stop pose estimation.
                        # Disable it until the next localisation.start, without a retry loop.
                        self.video_error = str(exc)[:200]
                next_compute=time.monotonic()+.2
        except Exception as exc:
            if not self.stop_event.is_set():
                self.error=str(exc)[:200]
                self.result=None
        finally:
            guards.clear();waitset=None
            for channel in channels:channel.close()
            if debug_channel is not None:debug_channel.close()

    def tick(self):
        if self.error is not None and self.error != self.reported_error:
            self.log('ERROR', f'Localisation stopped: {self.error}')
            self.emit('localisation.fault', {'error': self.error})
            self.reported_error = self.error
        if self.video_error is not None and self.video_error != self.reported_video_error:
            self.log('WARNING', f'Localisation video disabled; computation continues: {self.video_error}')
            self.reported_video_error = self.video_error

    def close(self):
        self.video_requested.clear()
        self.generation+=1
        self.stop_event.set()
        with self.lock:
            self.result=None
            self.window=None
        if self.thread is not None:
            self.thread.join(2)
            if self.thread.is_alive():
                raise Fault('timeout','Localisation computation is stopping')
            self.thread=None
        self.capture_id=None
