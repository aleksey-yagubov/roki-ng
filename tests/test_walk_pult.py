"""Compare Cartesian frames against the actual supplied controller C++ code."""
import math
from pathlib import Path
import shutil
import subprocess

import pytest

from roki_ng.body.motion.backends import load
from roki_ng.body.motion.backends.roki1.walk_pult import PultWalk, drive_order
from roki_ng.parameters import Parameters
from roki_ng.wire import Fault


@pytest.fixture
def parameters(tmp_path):
    return Parameters(tmp_path, 'roki1').values


@pytest.fixture(scope='module')
def reference(tmp_path_factory):
    compiler = shutil.which('g++')
    if not compiler:
        pytest.skip('C++ compiler needed for independent trajectory reference')
    source = (Path(__file__).parent/'reference/walk_pult.cpp').read_text()
    declarations = source[source.index('float stepLength;'):source.index('//Преобразование кватерниона')]
    phases = source[source.index('void walkInit()'):source.index('//Стабилизация направления')]
    wrapper = r'''
#include <cmath>
#include <cstdio>
#include <cstdlib>
#define MATH_PI 3.14159265358979323846
#define STEP_FIRST 0
#define STEP_LAST 1
#define STEP_OTHER 2
float sfMathCos(float x) { return std::cos(x); }
'''
    recorder = r'''
int computeAlphaForWalk() {
 printf("%.9g %.9g %.9g %.9g %.9g %.9g %.9g %.9g %.9g %.9g %.9g %.9g\n",
 xtr,ytr,ztr,xtl,ytl,ztl,xr,yr,xl,yl,wr,wl);
 return 1;
}
void testDrop() {}
'''
    main = r'''
int main(int argc, char **argv) {
 fr1=8; fr2=12; framestep=2; amplitude=32; d10=59.5;
 gaitHeight=165; stepHeight=20; bodyTiltAtWalk=0; solyLandingSkew=.01;
 selfMotionShiftCorrectionX=-50.0/21; selfMotionShiftCorrectionY=70.0/21;
 rotationYieldRight=rotationYieldLeft=.23; alpha01=MATH_PI*2/fr1;
 ytr=-d10-16; ytl=d10-16; ztr=ztl=-165; zr=zl=-1;
 float full=std::atof(argv[1]); sideLength=std::atof(argv[2]); rotation=std::atof(argv[3]);
 for(int c=0;c<4;c++) {
  stepLength=c<2 ? full*(c+1)/3 : full;
  stepType=c==0 ? STEP_FIRST : c==3 ? STEP_LAST : STEP_OTHER;
  walkCycle(0);
 }
}
'''
    directory = tmp_path_factory.mktemp('pult-reference')
    cpp, executable = directory/'reference.cpp', directory/'reference'
    cpp.write_text(wrapper+declarations+recorder+phases+main)
    subprocess.run([compiler, '-O0', str(cpp), '-o', str(executable)], check=True, capture_output=True)
    return executable


@pytest.mark.parametrize('step,side,rotation', [(25,0,0),(-20,0,0),(0,18,0),(0,24,0),(25,0,.15),(25,0,-.15)])
def test_frames_match_controller_source(parameters, reference, step, side, rotation):
    class RecordingWalk(PultWalk):
        def frame(self, frames=8, pause=0):
            yield (self.xtr,self.ytr,self.ztr,self.xtl,self.ytl,self.ztl,
                   self.xr,self.yr,self.xl,self.yl,self.wr,self.wl)
    engine = RecordingWalk(parameters, backend=load('roki1'))
    list(engine.initial())
    actual = []
    for c in range(4):
        actual.extend(engine.cycle(step*(c+1)/3 if c<2 else step, side, rotation,
                                  'first' if c==0 else 'last' if c==3 else 'other'))
    output = subprocess.check_output([str(reference),str(step),str(side),str(rotation)],text=True)
    expected = [[float(n) for n in row.split()] for row in output.splitlines()]
    assert len(actual) == len(expected) == 76
    for row, target in zip(actual, expected):
        assert row == pytest.approx(target, abs=5e-5)


@pytest.mark.parametrize('step,side,right', [(25,0,True),(-20,0,True),(0,18,True),(0,24,False)])
def test_real_ik_and_transport_frames(parameters, step, side, right):
    pytest.importorskip('starkit')
    engine = PultWalk(parameters, lambda *a: None, backend=load('roki1'))
    engine.first_Leg_Is_Right_Leg = right
    initial = list(engine.initial())
    assert initial[0][2:] == (20,19)
    for c in range(4):
        commands = list(engine.cycle(step*(c+1)/3 if c<2 else step,side,0,
                                    'first' if c==0 else 'last' if c==3 else 'other'))
        assert commands
        assert all(command[0] != 'drain' for command in commands)
        for command in commands:
            if command[0] == 'servo':
                assert command[2:] == (4,0)
                assert all(1 <= v.Id <= 10 and 0 <= v.Data <= 16383 for v in command[1])
                assert any(v.Id == 4 and v.Data != 7500 for v in command[1])
    assert list(engine.final())[0][2:] == (20,19)
    assert engine.ztr == engine.ztl == -231
    assert engine.xtr == engine.xtl == 0


def test_model_and_timing_guards(parameters, tmp_path):
    assert parameters['walk.roki1_pult_enabled'] is False
    with pytest.raises(Fault):
        PultWalk(parameters, backend=load('roki2'))
    with pytest.raises(Fault):
        PultWalk(parameters | {'motion.frame_ms':10}, backend=load('roki1'))
    with pytest.raises(Fault):
        Parameters(tmp_path/'r2','roki2').set('walk.roki1_pult_enabled',True)


def test_fixed_directions_and_dominant_axis():
    assert drive_order(dict(x=1,y=0,yaw=0)) == (25,0,None,0)
    assert drive_order(dict(x=-1,y=0,yaw=0)) == (-20,0,None,0)
    assert drive_order(dict(x=.2,y=-1,yaw=0)) == (0,18,True,0)
    assert drive_order(dict(x=0,y=1,yaw=0)) == (0,24,False,0)


@pytest.mark.parametrize('ending', ['release', 'expired', 'hard_stop'])
def test_worker_manual_walk_and_stop(parameters, body_clock, ending):
    pytest.importorskip('starkit')
    from roki_ng.body.runtime import Body
    body = Body({'simulate':True,'robot_model':'roki1',
                 'parameters':parameters | {'walk.roki1_pult_enabled':True}},
                lambda *a:None,lambda *a:None)
    body.command('control.acquire',{})
    body.simulated = False  # Native IK, simulated hardware only.
    body.hardware.drained = lambda: True
    sent = []
    send = body.hardware.send
    def record(values, frames, pause):
        sent.append((values,frames,pause))
        send(values,frames,pause)
    body.hardware.send = record
    job_id = body.command('motion.drive',{'x':1})['job_id']
    stopping = False
    for _ in range(3000):
        job = body.jobs[job_id]
        if not stopping and job['progress'] >= 2:
            stopping = True
            if ending == 'release':body.command('motion.drive',{'x':0})
            if ending == 'hard_stop':body.command('motion.stop_hard',{})
        if not stopping:
            body.command('motion.drive',{'x':1})
        body.tick()
        if not body.active:break
    assert not body.active
    assert job['status'] in ('completed','cancelled'), job
    assert any(frames == 4 for _,frames,_ in sent)
    assert all(1 <= v.Id <= 10 for values,_,_ in sent for v in values)
    if ending != 'hard_stop':
        assert body.pose == 'stand'
        assert sent[-1][1:] == (20,19)
        assert all(v.Data == 7500 for v in sent[-1][0] if v.Id <= 4)


def test_fixed_walk_selects_experimental_engine_not_manual_loop(parameters, body_clock, monkeypatch):
    pytest.importorskip('starkit')
    from roki_ng.body.runtime import Body
    body = Body({'simulate':True,'robot_model':'roki1',
                 'parameters':parameters | {'walk.roki1_pult_enabled':True}},
                lambda *a:None,lambda *a:None)
    body.command('control.acquire',{})
    body.simulated = False
    body.hardware.drained = lambda:True
    def forbidden():raise AssertionError('Fixed calibration motion selected experimental manual gait')
    monkeypatch.setattr(body,'_walk_pult',forbidden)
    ident=body._start('motion.drive',body._walk(cycles=1,fixed={
        'x':.2,'y':0,'yaw':0,'speed':.5,'crouch':'off'}))['job_id']
    selected = False
    for _ in range(3000):
        body.tick()
        selected |= isinstance(body.engine, PultWalk)
        if not body.active:break
    assert body.jobs[ident]['status']=='completed', body.jobs[ident]
    assert selected


@pytest.mark.parametrize('crouch,ending', [('on','release'), ('on','expired'), ('centered','release')])
def test_manual_stop_keeps_requested_crouch_and_can_restart(parameters, body_clock, crouch, ending):
    pytest.importorskip('starkit')
    from roki_ng.body.runtime import Body
    body = Body({'simulate':True,'robot_model':'roki1',
                 'parameters':parameters | {'walk.roki1_pult_enabled':True}},
                lambda *a:None,lambda *a:None)
    body.command('control.acquire',{})
    body.simulated = False
    body.hardware.drained = lambda:True
    # Repeat to cover restarting without first standing up.
    for _ in range(2):
        ident = body.command('motion.drive', {'x':1, 'crouch':crouch})['job_id']
        stopping = False
        for tick in range(3000):
            if not stopping and body.jobs[ident]['progress'] >= 2:
                stopping = True
                if ending == 'release':
                    body.command('motion.drive', {'x':0, 'crouch':crouch})
            if not stopping:
                body.command('motion.drive', {'x':1, 'crouch':crouch})
            body.tick()
            if not body.active:break
        assert body.jobs[ident]['status'] == 'completed', body.jobs[ident]
        assert body.pose == ('crouch_centered' if crouch == 'centered' else 'crouch')
        assert isinstance(body.engine, PultWalk)
        assert body.engine.ztr == body.engine.ztl == -165
        assert body.engine.arms_up
        if crouch == 'centered':
            assert body.engine.ytr == -body.engine.ytl
    baseline = dict(body.servo_targets)
    engine = body.engine
    ident = body.command('motion.jump', {'direction':'turn_left', 'crouch':crouch})['job_id']
    for _ in range(3000):
        body.tick()
        if not body.active:break
    assert body.jobs[ident]['status'] == 'completed', body.jobs[ident]
    assert body.servo_targets == baseline
    assert body.engine is engine


@pytest.mark.parametrize('direction,expected_pelvic', [('turn_right', -1302), ('turn_left', 1302)])
def test_pult_turn_source_impulse_units_timing_and_joint_scope(direction, expected_pelvic):
    from roki_ng.body.motion.backends.roki1.primitives import pult_turn_steps
    baseline = {(5,1):7600, (5,2):7400, (10,1):7700, (10,2):7300}
    steps = pult_turn_steps(load('roki1').Robot(), baseline, direction, 1.)
    assert steps[0][2:] == steps[1][2:] == (2,1)  # 40 ms, not 80 ms
    actual = {(v.Id,v.Sio):v.Data-baseline[v.Id,v.Sio] for v in steps[0][1]}
    assert actual == {(5,1):expected_pelvic, (5,2):expected_pelvic,
                      (10,1):-1302, (10,2):1302}
    assert {(v.Id,v.Sio):v.Data for v in steps[1][1]} == baseline
    assert len(steps) == 2


def test_ready_pult_turn_uses_fresh_poll_without_extra_preflight_wait(parameters, body_clock):
    from roki_ng.body.runtime import Body
    body = Body({'simulate':True,'robot_model':'roki1',
                 'parameters':parameters | {'walk.roki1_pult_enabled':True}},
                lambda *a:None,lambda *a:None)
    body.pose = 'stand'
    body.servo_targets = {(5,1):7500, (5,2):7500, (10,1):7500, (10,2):7500}
    body.body_imu.poll(body.hardware)
    plan = body._pult_turn('turn_right', 1., 'off')
    commands = list(plan)
    assert commands[0][0] == 'servo'
    assert [c[0] for c in commands] == ['servo', 'servo', 'drain', 'sleep']
    body.read_body_quaternion = lambda: (0.,0.,0.,1.)
    with pytest.raises(Fault, match='upright'):
        list(body._pult_turn('turn_right', 1., 'off'))
