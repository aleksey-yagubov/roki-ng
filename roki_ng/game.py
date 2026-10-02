"""Autonomous football roles; only completed body jobs update odometry."""
import asyncio, math, time, uuid
from .calibration import quaternion_yaw, wrap
from .wire import Fault, choice, number

def decision(observation, deadband, max_distance):
    result=observation.get('result') or {}; age=observation.get('age_ms')
    if (not observation.get('running') or observation.get('error') or not result.get('valid') or not isinstance(age,(int,float)) or not math.isfinite(age) or not 0<=age<=500): return 'hold',result.get('reason') or 'no_fresh_ball'
    x,y=result.get('x_m'),result.get('y_m')
    if any(type(v) not in (int,float) or not math.isfinite(v) for v in (x,y)) or x<=0 or math.hypot(x,y)>max_distance: return 'hold','invalid_ball_geometry'
    return ('left' if y>deadband else 'right' if y < -deadband else 'hold','lateral_correction' if abs(y)>deadband else 'ball_centered')

class Goalkeeper:
    """The name is retained internally; it runs FIRA goalkeeper and forward."""
    def __init__(self, supervisor):
        self.s=supervisor; self.task=None; self.session=None; self.stop_lock=asyncio.Lock(); self.generation=0; self.starting=False; self.cleaning=False
        self.stop_requested=self.hard_stop_requested=self.motion_inflight=self.paused=self.pickup_requested=False; self.last_seen=0.; self.last_seen_sequence=-1; self.info=self._info()
    def _info(self): return dict(running=False,state='stopped',role=None,entry=None,phase='idle',recovery='none',pickup_ready=False,reason='',decision='hold',ball=None,position=None,last_correction=None,travel_m=0.,path_m=0.,job_id=None)
    def state(self): return dict(self.info)
    async def _call(self,role,op,args=None,**kwargs): return await self.s.workers[role].call(op,args or {},**kwargs)

    async def start(self,session,args):
        if self.info['running'] or self.starting or self.task and not self.task.done(): raise Fault('busy','Game already running or stopping')
        if set(args)-{'strategy','entry','delay_seconds','observe_only','lease_epoch'}: raise Fault('invalid_argument','Unknown game settings')
        role=choice(args,'strategy',None,('FIRA_penalty_Goalkeeper','forward'))
        entry=choice(args,'entry','center',('center','left','right')) if role=='forward' else 'goalkeeper'
        if role!='forward' and 'entry' in args: raise Fault('invalid_argument','Goalkeeper has no entry')
        delay=number(args,'delay_seconds',0,0,30,True)
        if self.s.mode!='MANUAL': raise Fault('invalid_state','Select MANUAL before starting game')
        self.s.require_motion_ready(); self.starting=True; generation=self.generation
        try: body=await self._call('motherboard','state')
        finally: self.starting=False
        self.s.require_control(session,args)
        if generation!=self.generation or self.s.mode!='MANUAL': raise Fault('cancelled','Game start cancelled')
        if body.get('active_job') or not body.get('body_connected') or body.get('error'): raise Fault('not_ready','Body must be connected, idle and healthy')
        self.session=session; self.stop_requested=self.hard_stop_requested=self.paused=self.pickup_requested=False
        self.info=self._info()|dict(running=True,state='preparing',role=role,entry=entry,phase='initialising',job_id=uuid.uuid4().hex); self.s.mode='GAME'
        self.task=asyncio.create_task(self._run(dict(self.s.params.values),role,entry,delay)); return self.state()
    async def stop(self,reason='operator_stop',*,hard=False):
        self.stop_requested=True; self.hard_stop_requested|=hard
        async with self.stop_lock:
            self.generation+=1
            if self.task and not self.task.done():
                self.info.update(state='stopping',phase='stopping',reason=reason)
                # There is no supported partial result before a body job starts.
                # Cancelling a delay/preparation therefore cannot replay motion.
                if hard or not self.motion_inflight:self.task.cancel()
                await asyncio.gather(self.task,return_exceptions=True)
            self.info.update(running=False,state='stopped',phase='stopped',decision='hold',reason=reason)
            if self.s.mode=='GAME':self.s.mode='MANUAL' if self.s.owner else 'IDLE'
            return self.state()
    async def pause(self):
        if not self.info['running']:raise Fault('invalid_state','No active game')
        self.paused=True; self.info.update(state='paused',phase='paused',decision='hold')
        if self.motion_inflight:await self._call('motherboard','motion.stop_graceful',urgent=True)
        return self.state()
    async def resume(self):
        if not self.info['running'] or self.pickup_requested:raise Fault('invalid_state','Game cannot resume')
        self.paused=False;self.info.update(state='searching',phase='searching');return self.state()
    async def pickup(self):
        if not self.info['running']:raise Fault('invalid_state','No active game')
        self.pickup_requested=self.paused=True;self.info.update(state='pickup',phase='pickup',decision='hold',recovery='cancelled')
        if getattr(self.s,'voice',None):self.s.voice.say('Pick up')
        if self.motion_inflight:await self._call('motherboard','motion.stop_graceful',urgent=True)
        return self.state()
    async def _imu(self,fallen_ok=False):
        imu=(await self._call('motherboard','body.telemetry.read')).get('body.imu',{}); stamp=imu.get('source_mono_ns'); age=(time.monotonic_ns()-stamp)/1e9 if isinstance(stamp,int) else math.inf
        if not imu.get('valid') or not 0<=age<.25:raise Fault('imu_invalid','Fresh body IMU required')
        fallen=abs(imu.get('pitch_deg',90))>25 or abs(imu.get('roll_deg',90))>25
        if fallen and not fallen_ok:raise Fault('fallen','Body is not upright')
        return quaternion_yaw(imu['quaternion_xyzw']),fallen
    def _entry_pose(self,p,entry,yaw):
        g=p['field.geometry']; l,w=g['length'],g['width']
        if entry=='left':return [-l/2,-w/2,wrap(yaw+math.pi/2)]
        if entry=='right':return [-l/2,w/2,wrap(yaw-math.pi/2)]
        return [-l/2+.18,0.,yaw]
    async def _wait_job(self,response,timeout=20):
        self.motion_inflight=True; end=time.monotonic()+timeout
        while time.monotonic()<end:
            job=await self._call('motherboard','job.status',{'job_id':response['job_id']})
            if job['status']=='completed':self.motion_inflight=False;return True
            if job['status'] in ('failed','cancelled'):self.motion_inflight=False;return False
            if self.stop_requested or self.pickup_requested:await self._call('motherboard','motion.stop_graceful',urgent=True)
            await asyncio.sleep(.05)
        self.motion_inflight=False;return False
    async def _move(self,op,args,delta,pose):
        response=await self._call('motherboard',op,args)
        if await self._wait_job(response):
            c,s=math.cos(pose[2]),math.sin(pose[2]); dx,dy=delta;pose[0]+=c*dx-s*dy;pose[1]+=s*dx+c*dy
            self.info['position']=dict(x_m=pose[0],y_m=pose[1],yaw_rad=pose[2]);return True
        return False
    async def _recover(self):
        self.info.update(state='recovering',phase='get_up',recovery='checking')
        while not self.stop_requested and not self.pickup_requested:
            try:
                _,fallen=await self._imu(True)
                if not fallen:return True
                self.info['recovery']='getting_up'
                await self._wait_job(await self._call('motherboard','motion.get_up',{'crouch':'off'}),30)
            except Fault as exc:self.info.update(recovery='waiting_measurement',reason=str(exc)[:180])
            await asyncio.sleep(.25)
        return False
    async def _ball(self):
        obs=await self._call('detection','ball.status')
        if obs.get('error'):
            raise Fault('detection_fault', str(obs['error']))
        result=obs.get('result') or {};seq=result.get('frame_sequence',-1)
        if result.get('valid') and type(seq) is int and seq>self.last_seen_sequence:self.last_seen_sequence=seq
        self.info['ball']=result or None;return obs

    def _record_ball(self, obs):
        """Keep a monotonic freshness clock; repeated frames never refresh it."""
        r=obs.get('result') or {}; seq=r.get('frame_sequence'); age=obs.get('age_ms')
        if (obs.get('running') and not obs.get('error') and r.get('valid') and type(seq) is int
                and seq>self.last_seen_sequence and isinstance(age,(int,float)) and 0<=age<=500):
            self.last_seen_sequence=seq; self.last_seen=time.monotonic()-age/1000

    async def _monitor_step(self,response,heading,p):
        """Compatibility helper and graceful barrier used by finite side steps."""
        self.motion_inflight=True; finishing=False; end=time.monotonic()+12
        while time.monotonic()<end:
            job=await self._call('motherboard','job.status',{'job_id':response['job_id']})
            if job['status']=='completed':self.motion_inflight=False;return
            if job['status'] in ('failed','cancelled'):self.motion_inflight=False;raise Fault('motion_fault',job.get('reason') or 'Game step interrupted')
            obs=await self._call('detection','ball.status');self._record_ball(obs)
            if (self.stop_requested or obs.get('error')) and not finishing:
                await self._call('motherboard','motion.stop_graceful',urgent=True);finishing=True
            await asyncio.sleep(.1)
        self.motion_inflight=False;raise Fault('timeout','Game step timed out')
    async def _run(self,p,role,entry,delay):
        started=False; approach_jumps=0; approach_sequence=-1
        try:
            if delay:self.info.update(state='waiting',phase='start_later');await asyncio.sleep(delay)
            yaw,_=await self._imu(True);pose=self._entry_pose(p,entry,yaw);self.info['position']=dict(x_m=pose[0],y_m=pose[1],yaw_rad=pose[2])
            await self._move('motion.pose',{'name':'crouch'},(0,0),pose);await self._call('motherboard','motion.head',{'pan':0,'tilt':p['game.head_tilt']})
            await self.s.dispatch(None,'camera.start',{},_from_game=True);camera=await self._call('camera','camera.status')
            if not camera.get('running') or camera.get('error'):raise Fault('camera_fault',camera.get('error') or 'Camera stopped')
            # Threshold changes made while posture/capture was being prepared
            # belong to this game; fixed geometry/motion parameters do not.
            p = p | {k:v for k,v in self.s.params.values.items() if k.startswith('vision.')}
            await self._call('detection','ball.start',{'parameters':p,'unicam_minus_stm':camera.get('imu_sync',{}).get('unicam_minus_stm',0)});started=True
            if role=='forward' and entry=='center' and not delay and p.get('game.forward.kick_off_ride',True):
                self.info.update(state='kickoff_ride',phase='kickoff_ride');await self._move('motion.jump',{'direction':'forward','fraction':1.,'crouch':'on'},(.03,0),pose)
            while not self.stop_requested:
                if self.pickup_requested:
                    # Pick-up never attempts another get-up.  A standing body is
                    # put into its transfer posture; a fallen one waits for a
                    # person to place it upright and re-enter a role explicitly.
                    try:
                        _, fallen = await self._imu(True)
                    except Fault:
                        fallen = True
                    if fallen:
                        self.info.update(pickup_ready=False,state='pickup_waiting',phase='pickup_waiting')
                    else:
                        await self._move('motion.pose', {'name':'base_stand'}, (0,0), pose)
                        self.info.update(pickup_ready=True,state='pickup_ready',phase='pickup_ready')
                    while self.pickup_requested and not self.stop_requested:await asyncio.sleep(.1)
                if self.paused:await asyncio.sleep(.1);continue
                try:
                    _,fallen=await self._imu(True)
                    if fallen:
                        if not await self._recover():break
                        self.last_seen_sequence=-1;continue
                    obs=await self._ball()
                except Fault as exc:self.info.update(state='reconnecting',phase='reconnecting',recovery='link',reason=str(exc)[:180]);await asyncio.sleep(.25);continue
                if role=='FIRA_penalty_Goalkeeper':
                    action,reason=decision(obs,p['game.deadband_m'],p['game.ball_max_distance_m']);self.info.update(state='tracking',phase='goalkeeper',decision=action,reason=reason)
                    if action!='hold':
                        side=p['game.side_step_mm']/1000;limit=p['field.goal.0']['width']/2;sign=1 if action=='left' else -1;target=max(-limit,min(limit,pose[1]+sign*side))
                        if await self._move('game.step',{'direction':action,'heading':pose[2],'side_mm':p['game.side_step_mm'],'cycles':p['game.step_cycles']},(0,target-pose[1]),pose):self.info['last_correction']='odometry'
                else:
                    self.info.update(state='searching',phase='forward_search',decision='hold');r=obs.get('result') or {}
                    if r.get('valid') and r.get('x_m',99)<.38 and abs(r.get('y_m',99))<.12:
                        # A kick is permitted only from the currently observed
                        # support pose, never from the estimate before a jump.
                        self.info.update(state='kicking',phase='kick');await self._move('motion.kick',{'leg':'right' if r['y_m']<=0 else 'left','power':80,'offset':0},(0,0),pose);approach_jumps=0
                    elif r.get('valid'):
                        sequence=r.get('frame_sequence')
                        if approach_jumps >= 5:
                            self.info.update(state='reobserving',phase='forward_reobserve')
                            if sequence == approach_sequence: await asyncio.sleep(.08);continue
                            approach_jumps=0
                        if await self._move('motion.jump',{'direction':'forward','fraction':1.,'crouch':'on'},(.03,0),pose):
                            approach_jumps+=1;approach_sequence=sequence
                await asyncio.sleep(.08)
        except asyncio.CancelledError:raise
        except Exception as exc:self.info.update(state='failed',phase='failed',reason=str(exc)[:180]);self.s.log('game','ERROR',str(exc))
        finally:
            if self.motion_inflight or self.hard_stop_requested:
                try:await self._call('motherboard','motion.stop_hard',urgent=True)
                except Exception as exc:self.s.log('game','ERROR',f'Stop unconfirmed: {exc}')
            if started:
                try:await self._call('detection','ball.stop',timeout=5)
                except Exception as exc:self.s.log('game','ERROR',f'Ball cleanup: {exc}')
            self.info['running']=False
            if self.info['state'] not in ('failed','pickup_ready'):self.info.update(state='stopped',phase='stopped')
            if self.s.mode=='GAME':self.s.mode='MANUAL' if self.s.owner else 'IDLE'
