"""Original camera_elevation geometry, anchored to measured standing height."""
import math


def camera_height(parameters, model, engine, pitch_deg, roll_deg, pan, tilt):
    pitch, roll = math.radians(pitch_deg), math.radians(roll_deg)
    neck = parameters['game.neck_height_m']
    tick = .00058909
    # Calibrated height is at the configured observation tilt in base_stand.
    z = (parameters['game.camera_height_m'] + model.ztr0/1000
         + neck*(math.cos(tilt*tick)-math.cos(parameters['game.head_tilt']*tick)))
    camera = (neck*math.sin(-tilt*tick)*math.cos(pan*tick),
              neck*math.sin(-tilt*tick)*math.sin(pan*tick), z)
    def world_z(vector):
        x, y, z = vector
        return -math.sin(pitch)*x+math.cos(pitch)*(math.sin(roll)*y+math.cos(roll)*z)
    feet = [(0, -model.d10/1000, model.ztr0/1000), (0, model.d10/1000, model.ztl0/1000)]
    if engine is not None:
        feet = [(engine.xtr/1000, engine.ytr/1000, engine.ztr/1000),
                (engine.xtl/1000, engine.ytl/1000, engine.ztl/1000)]
    return world_z(camera)-min(map(world_z, feet))
