import pytest
from roki_ng.stream import Streams
from roki_ng.wire import Fault


def create(streams,backend,port):
    return streams.command('video.create',{'backend':backend,'host':'127.0.0.1','destination':{'rtp_port':port}})['stream_id']


def start(streams,ident):
    return streams.command('video.start',{'stream_id':ident,'source_frame_duration_us':16667})


def test_localisation_subscription_is_independent_of_main_video():
    streams=Streams({'simulate':True},lambda *a:None,lambda *a:None)
    main=create(streams,'runtime',5004);debug=create(streams,'localisation',5006)
    assert not streams.state()['active_streams']
    start(streams,main);start(streams,debug)
    assert len(streams.state()['active_streams'])==2
    streams.command('video.destroy',{'stream_id':debug})
    assert [s['stream_id'] for s in streams.state()['active_streams']]==[main]
    debug=create(streams,'localisation',5006);start(streams,debug)
    streams.command('video.stop_localisation',{})
    assert [s['stream_id'] for s in streams.state()['active_streams']]==[main]
    streams.command('video.stop_runtime',{})
    assert not streams.state()['active_streams']
    streams.close()


def test_direct_camera_and_duplicate_destinations_remain_exclusive():
    streams=Streams({'simulate':True},lambda *a:None,lambda *a:None)
    main=create(streams,'runtime',5004);start(streams,main)
    for backend,port,message in [('direct-gst',5006,'owns'),('runtime',5006,'owns'),('localisation',5004,'different RTP')]:
        ident=create(streams,backend,port)
        with pytest.raises(Fault,match=message):start(streams,ident)
    assert len(streams.state()['active_streams'])==1
    streams.close()
