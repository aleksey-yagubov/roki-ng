"""Fixed worker registrations; no network discovery or camera probing."""

from copy import deepcopy

SOURCES = {
    'direct-gst': {'id': 'direct-gst', 'worker': 'stream', 'name': 'Прямой захват GStreamer',
                   'topic': None, 'parameters': {}, 'requires': ['camera_free']},
    'runtime': {'id': 'runtime', 'worker': 'camera', 'name': 'Камера',
                'topic': 'roki/camera/frame/v1', 'parameters': {}, 'requires': ['camera.running']},
    'localisation': {'id': 'localisation', 'worker': 'localisation', 'name': 'Разметка локализации',
                     'topic': 'roki/localisation/frame/v1', 'parameters': {},
                     'requires': ['camera.running', 'localisation.running']},
}


def describe_sources(role):
    return [deepcopy(value) for value in SOURCES.values() if value['worker'] == role]
