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

    def state(self):
        result=dict(self.result) if self.result else None
        age=round((time.monotonic()-self.last_measurement)*1000) if result else None
        if result and age>1500:
            result.update(valid=False,reason='stale')
        return {'state':'fault' if self.error else 'ready',
                'running':self.thread is not None and self.thread.is_alive(),
                'mode':'diagnostic_only', 'capture_id':self.capture_id,
                'frames':self.frames,'dropped':self.dropped,'age_ms':age,
                'error':self.error,'result':result}

    def command(self, op, args):
        if op=='params.apply':
            from .parameters import validate_colour_ranges
            updated=self.parameters | args
            validate_colour_ranges(updated)
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
        self.frames=self.dropped=0
        self.capture_id=args['capture_id']
        self.thread=threading.Thread(target=self._run,args=(engine,projector,offset,self.generation),daemon=True)
        self.thread.start()
        return self.state()

    def _run(self, engine, projector, offset, generation):
        channels=[];guards=[];waitset=None
        try:
            import cv2
            import numpy as np
            from .dataplane import Channel,FRAME_TOPIC,IMU_TOPIC,FRAME_HEADER,IMU_RECORD
            from .synchronization import SequenceJoiner
            from .field_observations import runtime_paint_mask,detect_circle,observations
            from .goal_observations import goal_candidates
            cv2.setNumThreads(1)
            channels=[Channel(FRAME_TOPIC),Channel(IMU_TOPIC)]
            iox=channels[0].iox
            waitset=iox.WaitSetBuilder.new().create(iox.ServiceType.Ipc)
            guards=[waitset.attach_deadline(c.listener,iox.Duration.from_secs(1)) for c in channels]
            joiner=SequenceJoiner(capacity=128)
            joiner.set_alignment(offset)
            last_pair=time.monotonic();next_compute=0.
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
                                matched=joiner.frame(rec[0],(rec,owned,time.monotonic()))
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
                if now<next_compute or now-received>.5:
                    self.dropped+=1;continue
                if header[0]<=engine.sequence:
                    self.dropped+=1;continue
                started=now
                image=np.frombuffer(data,np.uint8).reshape(650,800,3)
                try:
                    parameters=self.parameters;revision=self.parameter_revision
                    white=runtime_paint_mask(image,parameters)
                    paint=projector.project(white,imu[2:6])
                    circle=detect_circle(paint,scale=.5)
                    if circle:
                        x,y,r=circle['pixel_circle'];yy,xx=np.indices(paint.shape)
                        paint[np.abs(np.hypot(xx-x,yy-y)-r)<8]=0
                    lines=observations(paint,scale=.5)[:32]
                    result=engine.update(header[0],lines,circle)
                except ValueError as exc:
                    result={'valid':False,'candidate':None,'reason':str(exc)[:120],
                            'frame_sequence':header[0]}
                result.update(sensor_timestamp_ns=header[1],imu_sequence=imu[0],
                              processing_ms=round((time.monotonic()-started)*1000))
                result['goal_candidates']=goal_candidates(image,parameters)
                result['parameter_revision']=revision
                result['processing_ms']=round((time.monotonic()-started)*1000)
                # Stop invalidates calculations already in flight.
                if self.stop_event.is_set() or self.generation!=generation:break
                if revision!=self.parameter_revision:continue
                self.result=result;self.last_measurement=received;self.frames+=1
                next_compute=time.monotonic()+.2
        except Exception as exc:
            if not self.stop_event.is_set():self.error=str(exc)[:200]
        finally:
            guards.clear();waitset=None
            for channel in channels:channel.close()

    def tick(self):
        # Worker heartbeat carries bounded state. Avoid per-frame events.
        pass

    def close(self):
        self.generation+=1
        self.stop_event.set()
        self.result=None
        if self.thread is not None:
            self.thread.join(2)
            if self.thread.is_alive():
                raise Fault('timeout','Localisation computation is stopping')
            self.thread=None
        self.capture_id=None
