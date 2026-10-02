"""Video-output descriptions and validation; producers own their declarations."""

from copy import deepcopy

from .wire import Fault, number

def _typed(source):
    for key, control in source["controls"].items():
        control["type"] = "float" if key in ("fps", "max_fps") else "int"
    return source


def frame_source(title, topic, width, height, *, available, reason=None,
                 publishing=False, requested=False, on_demand=False, dependencies=(), fps=60):
    return _typed({
        "title": title, "transport": "frames", "topic": topic,
        "geometry": [width, height, width * 3], "dependencies": list(dependencies),
        "on_demand": on_demand, "available": available, "reason": reason,
        "publishing": publishing, "requested": requested,
        "settings": {"width": width, "height": height, "fps": fps,
                     "max_fps": fps, "bitrate": 2000000},
        "controls": {"width": {"fixed": True}, "height": {"fixed": True},
                     "fps": {"min": 1, "max": 120, "live": False},
                     "max_fps": {"min": 1, "max": 120, "live": True},
                     "bitrate": {"min": 100000, "max": 20000000, "live": False}},
    })


def capture_source():
    return _typed({
        "title": "Прямой захват GStreamer", "transport": "libcamera", "topic": None,
        "geometry": None, "dependencies": [], "on_demand": False,
        "available": True, "reason": None, "publishing": False, "requested": False,
        "settings": {"width": 800, "height": 650, "fps": 60, "bitrate": 2000000,
                     "sensor_width": 1600, "sensor_height": 1300, "sensor_depth": 10},
        "controls": {"width": {"min": 160, "max": 1600, "live": False},
                     "height": {"min": 120, "max": 1300, "live": False},
                     "fps": {"min": 1, "max": 120, "live": False},
                     "bitrate": {"min": 100000, "max": 20000000, "live": False},
                     "sensor_width": {"min": 320, "max": 4096, "live": False},
                     "sensor_height": {"min": 240, "max": 4096, "live": False},
                     "sensor_depth": {"choices": [8, 10], "live": False}},
    })


def settings_for(source, changes, previous=None):
    if not isinstance(changes, dict) or set(changes) - set(source["controls"]):
        raise Fault("invalid_argument", "Unknown video settings; use controls from videostream.list")
    result = deepcopy(source["settings"] if previous is None else previous)
    for key, value in changes.items():
        meta = source["controls"][key]
        if meta.get("fixed"):
            if type(value) is not type(source["settings"][key]) or value != source["settings"][key]:
                raise Fault("invalid_argument", f"{key} is fixed by the producer")
        elif "choices" in meta:
            if type(value) is not int or value not in meta["choices"]:
                raise Fault("invalid_argument", f"Unsupported {key}")
        else:
            number({key: value}, key, None, meta["min"], meta["max"], key not in ("fps", "max_fps"))
        result[key] = value
    if result["width"] % 2 or result["height"] % 2:
        raise Fault("invalid_argument", "H.264 output dimensions must be even")
    if source["transport"] == "libcamera":
        if result["width"] > result["sensor_width"] or result["height"] > result["sensor_height"]:
            raise Fault("invalid_argument", "Output exceeds sensor dimensions")
    elif result["max_fps"] > result["fps"]:
        raise Fault("invalid_argument", "max_fps exceeds encoder fps ceiling")
    return result
