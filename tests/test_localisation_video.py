import numpy as np
import pytest
from roki_ng.stream import pipeline_description
from roki_ng.video_sources import frame_source, settings_for
from roki_ng.wire import Fault


def test_localisation_stream_uses_bounded_shared_frames_not_camera():
    source=frame_source('Test', 'test/frames', 800, 650, available=True)
    pipeline=pipeline_description(source, source['settings'])
    assert 'appsrc name=frames' in pipeline and 'libcamerasrc' not in pipeline
    assert 'width=800,height=650' in pipeline
    with pytest.raises(Fault):settings_for(source, {'sensor_width':1600})
    with pytest.raises(Fault):settings_for(source, {'width':1600})


def test_annotation_is_frame_bound_and_does_not_modify_source():
    from roki_ng.localisation_debug import video_frame
    class Projector:
        def image_points(self,points,q):
            return np.asarray(points)*100+300,np.ones(len(points),bool)
    image=np.zeros((650,800,3),np.uint8)
    lines=np.array([[[0.,0.],[1.,0.]]])
    frame=video_frame(image,Projector(),[],lines,None,
        [{'colour':'blue','rect':[50,100,20,70]}],
        {'frame_sequence':47,'reason':'insufficient_observations'},lines)
    assert frame.shape==image.shape and not image.any()
    assert frame[300,350].any() and frame[100,50].any()
    assert frame[:65].any()
