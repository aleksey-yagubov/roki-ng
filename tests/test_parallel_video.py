import pytest
from roki_ng.stream import Streams
from roki_ng.wire import Fault


def create(streams,backend,port):
    return streams.command('videostream.create',{'source':backend})['stream_id']


def start(streams,ident,port=5004):
    return streams.command('videostream.start',{'stream_id':ident,'source_frame_duration_us':16667,
                           'host':'127.0.0.1','rtp_port':port,'session_id':1})


def test_localisation_subscription_is_independent_of_main_video():
    streams=Streams({'simulate':True},lambda *a:None,lambda *a:None)
    main=create(streams,'runtime',5004);debug=create(streams,'localisation',5006)
    assert not streams.state()['active_streams']
    start(streams,main);start(streams,debug,5006)
    assert len(streams.state()['active_streams'])==2
    streams.command('videostream.destroy',{'stream_id':debug})
    assert [s['stream_id'] for s in streams.state()['active_streams']]==[main]
    debug=create(streams,'localisation',5006);start(streams,debug,5006)
    streams.command('videostream.stop_localisation',{})
    assert [s['stream_id'] for s in streams.state()['active_streams']]==[main]
    streams.command('videostream.stop_runtime',{})
    assert not streams.state()['active_streams']
    streams.close()


def test_direct_camera_and_duplicate_destinations_remain_exclusive():
    streams=Streams({'simulate':True},lambda *a:None,lambda *a:None)
    main=create(streams,'runtime',5004);start(streams,main)
    for backend,port,message in [('direct-gst',5006,'owns'),('localisation',5004,'different RTP')]:
        ident=create(streams,backend,port)
        with pytest.raises(Fault,match=message):start(streams,ident,port)
    assert len(streams.state()['active_streams'])==1
    streams.close()
