import asyncio
from types import SimpleNamespace as NS
from unittest.mock import AsyncMock

import pytest

from roki_ng.camera import Camera
from roki_ng.client import Client
from roki_ng.dataplane import FRAME_BYTES, FRAME_HEADER
from roki_ng.runtime_video import RuntimeVideo
from roki_ng.stream import Streams
from roki_ng.supervisor import Supervisor, Session
from roki_ng.wire import Fault, UDP_LIMIT, envelope, pack


def test_multiclient_protocol_over_udp(tmp_path):
    async def run():
        server = Supervisor({'simulate':True,'state_dir':str(tmp_path),'host':'127.0.0.1','port':0})
        await server.start()
        owner = Client('127.0.0.1',server.sock.getsockname()[1])
        viewer = Client('127.0.0.1',server.sock.getsockname()[1])
        try:
            await owner.connect(); await viewer.connect()
            await owner.request('control.acquire')
            # Catalog is explicit, paged, includes unavailable producers.
            sources = [(await viewer.request('videostream.sources',{'offset':i}))['items'][0] for i in range(3)]
            assert sources[0]['available']
            assert sources[1]['reason']=='camera_stopped'
            assert sources[2]['reason']=='localisation_not_ready'
            assert not (await viewer.request('camera.status'))['prepared']
            caps = await viewer.request('camera.capabilities')
            assert caps['imu_required'] and caps['sensor']['width']==1600
            controls = await viewer.request('camera.controls.list')
            assert controls['total']==6 and controls['items'][0]['supported'] is None
            with pytest.raises(Fault) as old:
                await owner.request('video.create')
            assert old.value.code=='not_supported'
            with pytest.raises(Fault):
                await viewer.request('videostream.create')
            stream = await owner.request('videostream.create')
            ident=stream['stream_id']
            with pytest.raises(Fault):
                await viewer.request('videostream.attach',{'stream_id':ident,'rtp_port':5006})
            first = await owner.request('videostream.start',{'stream_id':ident,'rtp_port':5004})
            attached = await viewer.request('videostream.attach',{'stream_id':ident,'rtp_port':5006})
            assert attached['receivers']==2 and attached['run_id']==first['run_id']
            for op in ('stop','destroy','update','start'):
                with pytest.raises(Fault):
                    await viewer.request('videostream.'+op,{'stream_id':ident,'rtp_port':5006})
            await owner.close()
            await asyncio.sleep(.1)
            status = await viewer.request('videostream.status',{'stream_id':ident})
            assert status['receivers']==1 and status['state']=='running'
            assert status['run_id']==first['run_id']
            await viewer.request('control.acquire')
            # New control owner can stop a stream created by somebody else.
            await viewer.request('videostream.stop',{'stream_id':ident})
            second = await viewer.request('videostream.start',{'stream_id':ident,'rtp_port':5006})
            assert second['run_id'] != first['run_id']
            await viewer.request('control.release')
            stopped = await viewer.request('videostream.detach',{'stream_id':ident})
            assert stopped['state']=='stopped' and stopped['receivers']==0
            listed = await viewer.request('videostream.list')
            assert listed['items'][0]['state']=='stopped'
            assert server.counters['oversize']==0
        finally:
            if owner.sock.fileno()>=0: await owner.close()
            await viewer.close(); await server.close()
    asyncio.run(run())


def test_source_reference_count_and_failed_start(tmp_path):
    async def run():
        server=Supervisor({'state_dir':str(tmp_path)})
        server.owner=1;server.lease_epoch=1;server.mode='MANUAL'
        session=Session(1,1,('127.0.0.1',8094),'operator')
        streams=Streams({'simulate':True},lambda *a:None,lambda *a:None)
        requests=[]
        async def localisation(op,args=None,**kw):
            if op=='localisation.status':return {'running':True}
            requests.append(op)
            return {}
        server.workers={'stream':NS(call=AsyncMock(side_effect=streams.command)),
                        'camera':NS(call=AsyncMock(return_value={'running':True,'prepared':True,'frame_duration_us':16667})),
                        'localisation':NS(call=localisation)}
        # WorkerPeer.call accepts omitted args; normalize them for this fake.
        async def stream_call(op,args=None,**kw):return streams.command(op,args or {})
        server.workers['stream'].call=stream_call
        async def cmd(op,body):return await server.dispatch(session,'videostream.'+op,body|{'lease_epoch':1})
        a=(await cmd('create',{'source':'localisation'}))['stream_id']
        b=(await cmd('create',{'source':'localisation'}))['stream_id']
        await cmd('start',{'stream_id':a,'rtp_port':5004})
        await cmd('start',{'stream_id':b,'rtp_port':5006})
        requests.clear()
        await cmd('stop',{'stream_id':a})
        assert 'source.stop' not in requests
        await cmd('detach',{'stream_id':b})
        assert requests==['source.stop']
        # A rejected encoder start must release the newly requested source.
        original=streams.pipelines[a]._start
        streams.pipelines[a]._start=lambda info: (_ for _ in ()).throw(Fault('pipeline_error','test'))
        requests.clear()
        with pytest.raises(Fault):await cmd('start',{'stream_id':a,'rtp_port':5004})
        assert requests==['source.start','source.stop']
        assert not streams.pipelines[a].receivers
        streams.pipelines[a]._start=original
        streams.close()
    asyncio.run(run())


def test_receiver_detach_deduplicates_endpoints_and_source_independence():
    streams=Streams({'simulate':True},lambda *a:None,lambda *a:None)
    ident=streams.command('videostream.create',{'source':'runtime'})['stream_id']
    start={'stream_id':ident,'host':'127.0.0.1','rtp_port':5004,'session_id':1,'source_frame_duration_us':16667}
    streams.command('videostream.start',start)
    streams.command('videostream.attach',start|{'session_id':2})
    pipeline=streams.pipelines[ident]
    sink=NS(emit=lambda *args: calls.append(args));calls=[]
    pipeline.pipeline=NS(get_by_name=lambda _:sink,set_state=lambda _:None)
    pipeline.Gst=NS(State=NS(NULL=0))
    streams.command('videostream.detach',{'stream_id':ident,'session_id':1})
    assert not calls and pipeline.active
    streams.command('videostream.detach',{'stream_id':ident,'session_id':2})
    assert calls==[('remove','127.0.0.1',5004)] and pipeline.active is None


def test_live_max_fps_preserves_timestamps_and_drops_before_allocation(monkeypatch):
    monkeypatch.setattr('roki_ng.runtime_video.FrameReader',lambda *a,**kw:NS(error=None,skipped=0,close=lambda:None))
    buffers=[]
    def allocate(*args):
        obj=NS(fill=lambda *a:None)
        buffers.append(obj)
        return obj
    runtime=RuntimeVideo(NS(emit=lambda *a:0),NS(Buffer=NS(new_allocate=allocate),FlowReturn=NS(OK=0)),10)
    frame=bytearray(FRAME_BYTES)
    def push(seq,ms):
        FRAME_HEADER.pack_into(frame,0,seq,int(ms*1e6),800,650,2400)
        runtime.push(memoryview(frame))
    push(0,1000);push(1,1050)
    runtime.set_max_fps(2)
    push(2,1100);push(3,1490);push(4,1500)
    runtime.set_max_fps(20)
    push(5,1550);push(6,1570)
    assert runtime.submitted==3 and runtime.rate_skipped==4 and len(buffers)==3
    assert [b.pts for b in buffers]==[0,490000000,550000000]
    assert [b.duration for b in buffers]==[100000000,500000000,50000000]


def test_update_validation_is_atomic():
    streams=Streams({'simulate':True},lambda *a:None,lambda *a:None)
    ident=streams.command('videostream.create',{'source':'runtime','max_fps':2})['stream_id']
    args={'stream_id':ident,'session_id':1}
    streams.command('videostream.start',args|{'host':'127.0.0.1','rtp_port':5004,'source_frame_duration_us':16667})
    updated=streams.command('videostream.update',args|{'max_fps':5})
    assert updated['spec']['max_fps']==5
    for bad in (0,121,True,float('nan')):
        with pytest.raises(Fault): streams.command('videostream.update',args|{'max_fps':bad})
    with pytest.raises(Fault): streams.command('videostream.update',args|{'max_fps':10,'bitrate':3000000})
    assert updated['spec']['max_fps']==5


def test_camera_controls_set_save_and_catalog_bounds(tmp_path):
    async def run():
        server=Supervisor({'state_dir':str(tmp_path)})
        server.require_control=lambda *args:None
        camera=Camera({'parameters':server.params.values},lambda *a:None,lambda *a:None)
        async def call(op,args=None,**kw):return camera.command(op,args or {})
        server.workers={'camera':NS(call=call)}
        value={'camera.analogue_gain':2.0}
        await server.dispatch(None,'camera.controls.set',{'values':value})
        assert camera.gain==2 and server.params.values['camera.analogue_gain']==1
        await server.dispatch(None,'camera.controls.save',{'values':value})
        assert server.params.values['camera.analogue_gain']==2
        for offset in range(0,6,2):
            result=await server.dispatch(None,'camera.controls.list',{'offset':offset})
            assert len(pack(envelope('response','camera.controls.list',{'result':result},session=2**63,token=2**63)))<=UDP_LIMIT
    asyncio.run(run())


def test_camera_control_catalog_with_hardware_ranges():
    camera=Camera({},lambda *a:None,lambda *a:None)
    names=('ColourGains','ExposureTime','AnalogueGain','AeEnable','AwbEnable')
    camera.lc=NS(controls=NS(**{name:name for name in names}))
    camera.camera=NS(controls={
        'ColourGains':NS(min=.1,max=8.), 'ExposureTime':NS(min=50,max=100000),
        'AnalogueGain':NS(min=1.,max=8.),'AeEnable':NS(min=False,max=True)})
    catalog=[]
    for offset in range(0,6,2):
        catalog.extend(camera.command('camera.controls.list',{'offset':offset})['items'])
    by_key={item['key']:item for item in catalog}
    assert by_key['camera.ae_enabled']['supported'] is True
    assert by_key['camera.ae_enabled']['min'] is None
    assert by_key['camera.awb_enabled']['supported'] is False
    assert by_key['camera.exposure_us']['max']==camera.duration
    assert by_key['camera.white_balance.blue_gain']['max']==8.
