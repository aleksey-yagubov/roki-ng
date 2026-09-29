"""Typed, atomic per-robot settings. Defaults are not robot calibration."""

import json
import os
from pathlib import Path
import tempfile
import copy

from . import field_config

from .wire import Fault, number, boolean

# Starting colour ranges, not the calibration of any particular robot/field.
COLOUR_DEFAULTS = {
    "orange_ball": (20, 100, 5, 65, 15, 100),
    "green_field": (10, 90, -100, -5, -10, 100),
    "white_marking": (60, 100, -15, 15, -15, 15),
    "blue_posts": (5, 80, -10, 80, -128, -5),
    "yellow_posts": (40, 100, -40, 20, 20, 127),
    "white_posts": (60, 100, -15, 15, -15, 15),
}

SCHEMA = {
    'camera.exposure_us': ('int',8000,1,100000,'next_request','Manual exposure in microseconds; must fit frame period'),
    'camera.analogue_gain': ('float',1.,1.,16.,'next_request','Manual analogue gain'),
    'camera.ae_enabled': ('bool',False,None,None,'next_request','Automatic exposure; manual values retained when enabled'),
    'camera.awb_enabled': ('bool',False,None,None,'next_request','Automatic white balance; manual values retained when enabled'),
    'localisation.camera_height_m': ('float',.4068,.2,.8,'next_localisation','Optical centre height; nominal default, measure for robot posture'),
    'vision.field_auto': ('bool',True,None,None,'next_frame','Adaptive paint segmentation; false uses saved LAB thresholds'),
    'vision.field_auto_contrast': ('int',15,0,100,'next_frame','Minimum adaptive white/turf L contrast in OpenCV 0..255 units'),
    **{key: ('object', value, None, None, 'next_localisation', 'Field geometry; metres/radians')
       for key,value in field_config.DEFAULTS.items()},
    'match.own_goal': ('int',0,0,1,'next_localisation','Own goal ID; changing sides does not rotate the map'),
    **{f"camera.white_balance.{colour}_gain":
       ("float", 1.0, 0.01, 32.0, "next_request",
        f"Manual white balance {colour} gain; calibrate for venue lighting")
       for colour in ("red", "blue")},
    "logging.stdout_enabled": ("bool", False, None, None, "live", "Mirror runtime logs to supervisor stdout"),
    "walk.max_step_mm": ("float", 24.0, 1, 64, "next_job", "Продольный масштаб ручного шага, мм при speed=1 и полном вводе. Больше: длиннее шаг; меньше: короче. Темп не меняется; тесты задают свою длину."),
    "walk.max_side_mm": ("float", 12.0, 1, 20, "next_job", "Боковой масштаб ручного шага, мм при speed=1 и полном вводе. Больше: шире боковой шаг; меньше: уже. Темп не меняется."),
    "walk.step_height_mm": ("float", 40.0, 0, 60, "next_job", "Подъём переносимой стопы, мм. Больше: нога поднимается выше; меньше: ниже. Ходьба и шаговые тесты, не прыжки и не удар."),
    "walk.max_yaw_rad": ("float", 0.15, 0, 0.3, "next_job", "Масштаб поворота ручной ходьбой, рад на цикл при полном вводе. Больше: сильнее поворот; меньше: слабее. Не калибровка фактического курса."),
    "walk.gait_height_mm": ("float", 180.0, 140, 210, "next_job", "Высота опорных стоп относительно корпуса модели, мм. Больше: выше корпус и прямее ноги; меньше: глубже присед. Не высота камеры. Новая высота требует подготовки приседа."),
    "walk.sway_amplitude_mm": ("int", 32, 8, 64, "next_job", "Боковая амплитуда переноса веса в цикле, мм. Больше: сильнее качание корпуса вбок; меньше: слабее. Внутри алгоритма дискретизация; предпочтительны значения, кратные 8. Не смещение подготовительной позы."),
    "walk.heading_kp": ("float", 1.1, 0, 5, "next_job", "Коэффициент коррекции ошибки yaw, рад/рад, в прямолинейных шаговых тестах. Больше: резче исправляется курс; меньше: мягче; 0 отключает поправку. Не балансировка."),
    "walk.heading_max_correction_rad": ("float", 0.3, 0, 0.3, "next_job", "Предел поправки курса за цикл в шаговых тестах, рад. Больше: разрешена сильнее коррекция; меньше: слабее. Планируется также для ручного heading_hold."),
    "motion.frame_ms": ("int", 20, 10, 40, "restart", "Motherboard body queue period"),
    "walk.servo_frames_per_pose": ("int", 2, 1, 10, "next_job", "Кадры интерполяции одной расчётной цели. Больше: дольше переход и больше запаздывание; меньше: быстрее переход. Не темп цикла. Общий Engine использует это также в подготовке и ударе."),
    "walk.body_tilt_forward": ("float", 0.0, -0.3, 0.3, "next_job", "Компонента X направленного вниз вектора стопы при движении вперёд (X,0,-1), безразмерная, не радианы. Плюс отклоняет вектор к +X (вперёд); при горизонтальной стопе корпус наклоняется вперёд. Минус: назад. Модуль больше: сильнее наклон, угол примерно atan(X)."),
    "walk.body_tilt_backward": ("float", 0.0, -0.3, 0.3, "next_job", "Компонента X вектора стопы при ходьбе назад, безразмерная. Знак не инвертируется из-за движения назад: плюс наклоняет корпус вперёд при горизонтальной стопе, минус назад; больший модуль усиливает наклон."),
    "kick.body_tilt": ("float", 0.0, -0.3, 0.3, "next_job", "Kicking body tilt"),
    "walk.sole_skew": ("float", 0.0, -0.2, 0.2, "next_job", "Боковая компонента вектора стопы в ходьбе, безразмерная. Плюс: направленные вниз векторы расходятся наружу (правая Y=-s, левая Y=+s); минус: сходятся внутрь. Больше модуль: сильнее наклон; это не радианы."),
    "kick.sole_skew": ("float", 0.0, -0.2, 0.2, "next_job", "Боковая компонента вектора стопы при ударе. Плюс: вниз-наружу, минус: вниз-внутрь; больше модуль: сильнее наклон. Безразмерная, независима от walk.sole_skew."),
    "motion.shift_x_mm": ("float", 0.0, -2000, 2000, "next_job", "Displacement over spot-walk test, X mm"),
    "motion.shift_y_mm": ("float", 0.0, -2000, 2000, "next_job", "Displacement over spot-walk test, Y mm"),
    "motion.rotation_yield_right": ("float", 0.23, 0.01, 1, "next_job", "Measured clockwise yield"),
    "motion.rotation_yield_left": ("float", 0.23, 0.01, 1, "next_job", "Measured counterclockwise yield"),
    "motion.jump_yaw_cw": ("float", -0.44, -1.5, -0.01, "next_job", "Clockwise full-jump yaw, radians"),
    "motion.jump_yaw_ccw": ("float", 0.41, 0.01, 1.5, "next_job", "Counterclockwise full-jump yaw, radians"),
    "motion.run_10_mm": ("float", 1000.0, 1, 10000, "next_job", "Measured short_run distance, mm"),
    "motion.run_20_mm": ("float", 2000.0, 1, 20000, "next_job", "Measured long_run distance, mm"),
    "motion.side_right_20_mm": ("float", 400.0, 1, 10000, "next_job", "Measured 20 right cycles, mm"),
    "motion.side_left_20_mm": ("float", 400.0, 1, 10000, "next_job", "Measured 20 left cycles, mm"),
    **{f"motion.jump_{direction}_mm": ("float", 30.0, 1, 500, "next_job", "Measured displacement per jump, mm")
       for direction in ("forward", "backward", "left", "right")},
    "head.field_tilt": ("int", -1000, -2600, 950, "next_job", "Head field pose, servo ticks"),
    **{f"vision.{profile}.{key}": ("int", value, 0 if key.startswith("l_") else -128,
                                   100 if key.startswith("l_") else 127, "next_frame",
                                   f"LAB threshold {profile}: {key}; L 0..100, a/b -128..127")
       for profile, values in COLOUR_DEFAULTS.items()
       for key, value in zip(("l_min", "l_max", "a_min", "a_max", "b_min", "b_max"), values)},
    **{f"vision.{profile}.{key}": ("int", 50, 1, 800 * 650, "next_frame", description)
       for profile in COLOUR_DEFAULTS
       for key, description in (("pixels_min", "Minimum foreground pixels in a colour blob"),
                                ("box_area_min", "Minimum bounding rectangle area, pixels"))},
}


def validate_colour_ranges(values):
    for profile in COLOUR_DEFAULTS:
        for axis in ("l", "a", "b"):
            prefix = f"vision.{profile}.{axis}"
            if values[prefix + "_min"] > values[prefix + "_max"]:
                raise Fault("invalid_argument", f"{prefix}_min must not exceed {prefix}_max")


class Parameters:
    def __init__(self, directory):
        self.path = Path(directory) / "parameters.json"
        self.values = {k: copy.deepcopy(v[1]) for k, v in SCHEMA.items()}
        self.extra = {}
        if self.path.exists():
            stored = json.loads(self.path.read_text())
            if not isinstance(stored, dict):
                raise Fault("configuration", "parameters.json must contain an object")
            for key, value in stored.items():
                if key in SCHEMA:
                    self.values[key] = self.validate(key, value)
                else:
                    self.extra[key] = value
        validate_colour_ranges(self.values)
        self.save(self.values)

    def describe(self, key):
        if key not in SCHEMA:
            raise Fault("not_found", f"Unknown parameter: {key}")
        kind, default, low, high, apply, description = SCHEMA[key]
        result=dict(key=key, type=kind, default=copy.deepcopy(default), min=low, max=high,
                    apply=apply, description=description)
        if key in field_config.FIELDS:result['fields']=field_config.FIELDS[key]
        return result

    def validate(self, key, value):
        meta = self.describe(key)
        if key in field_config.FIELDS:return field_config.validate(key,value)
        if meta["type"] == "bool":
            return boolean({key: value}, key)
        return number({key: value}, key, None, meta["min"], meta["max"], meta["type"] == "int")

    def set(self, key, value):
        values = self.set_many({key: value})
        return {"key": key, "value": values[key], "apply": SCHEMA[key][4]}

    def set_many(self, values):
        values = {key: self.validate(key, value) for key, value in values.items()}
        updated = self.values | values
        validate_colour_ranges(updated)
        self.save(updated)
        self.values = updated
        return values

    def save(self, values):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        fd, name = tempfile.mkstemp(dir=self.path.parent, prefix=".params-")
        try:
            with os.fdopen(fd, "w") as handle:
                json.dump(self.extra | values, handle, indent=2)
                handle.write("\n")
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(name, self.path)
            fd = os.open(self.path.parent, os.O_DIRECTORY)
            try:
                os.fsync(fd)
            finally:
                os.close(fd)
        finally:
            if os.path.exists(name):
                os.unlink(name)
