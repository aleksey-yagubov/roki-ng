"""Bounded field objects in the existing parameter store (metres/radians)."""
import copy
import math

from .wire import Fault, number, boolean, choice

# Compact descriptors also fit params.describe in one UDP datagram.
GEOMETRY = {'length':3.35,'width':2.35,'carpet_length':4.,'carpet_width':3.,
            'circle_diameter':.5,'circle_measured':False,'paint_width':.05}
MARK = {'enabled':False,'kind':'cross','x':0.,'y':0.,'size':.1,
        'size2':.1,'angle':0.,'width':.02}
FIELDS = {
    'field.geometry': {'length':[.5,12.],'width':[.5,9.],
        'carpet_length':[.5,15.],'carpet_width':[.5,12.],
        'circle_diameter':[.05,3.],'circle_measured':'bool','paint_width':[.005,.2]},
}
DEFAULTS = {'field.geometry':GEOMETRY}
for i, colour in enumerate(('yellow','blue')):
    key=f'field.goal.{i}'
    DEFAULTS[key]={'x':(-1 if i==0 else 1)*1.675,'y':0.,'width':1.,
                   'height':.6,'colour':colour,'measured':False}
    FIELDS[key]={'x':[-10.,10.],'y':[-10.,10.],'width':[.1,4.],
                 'height':[.1,3.],'colour':['yellow','blue','white','unknown'],'measured':'bool'}
for i in range(16):
    key=f'field.mark.{i:02d}'
    DEFAULTS[key]=copy.deepcopy(MARK)
    FIELDS[key]={'enabled':'bool','kind':['cross','ring','disk','line'],
                 'x':[-10.,10.],'y':[-10.,10.],'size':[.01,5.],
                 'size2':[.01,5.],'angle':[-math.pi,math.pi],'width':[.002,.2]}


def validate(key, value):
    fields=FIELDS[key]
    if not isinstance(value,dict) or set(value)!=set(fields):
        raise Fault('invalid_argument',f'{key}: provide exactly {list(fields)}')
    result={}
    for name, spec in fields.items():
        if spec=='bool': result[name]=boolean(value,name)
        elif isinstance(spec[0],str): result[name]=choice(value,name,None,spec)
        else: result[name]=number(value,name,None,*spec)
    if key=='field.geometry':
        if result['carpet_length']<result['length'] or result['carpet_width']<result['width']:
            raise Fault('invalid_argument','Carpet must contain the playing rectangle')
        if result['circle_diameter']>min(result['length'],result['width']):
            raise Fault('invalid_argument','Centre circle must fit the field')
    if key.startswith('field.mark.') and result['width']>(min(result['size'],result['size2']) if result['kind']=='cross' else result['size']):
        raise Fault('invalid_argument','Paint width exceeds mark size')
    return result


def line_model(parameters):
    import numpy as np
    g=parameters['field.geometry'];x=g['length']/2;y=g['width']/2
    rows=[[[-x,-y],[x,-y]],[[-x,y],[x,y]],
          [[-x,-y],[-x,y]],[[0,-y],[0,y]],[[x,-y],[x,y]]]
    for key,value in parameters.items():
        if not key.startswith('field.mark.') or not value['enabled']:continue
        if value['kind'] not in ('cross','line'):continue
        angles=[(value['angle'],value['size'])]
        if value['kind']=='cross':angles.append((value['angle']+math.pi/2,value['size2']))
        centre=np.array([value['x'],value['y']])
        for angle,size in angles:
            delta=np.array([math.cos(angle),math.sin(angle)])*size/2
            rows.append([centre-delta,centre+delta])
    return np.asarray(rows,float)


def circle_model(parameters):
    g=parameters['field.geometry']
    circles=[{'center':[0.,0.],'radius':g['circle_diameter']/2 if g['circle_measured'] else None}]
    for key,m in parameters.items():
        if key.startswith('field.mark.') and m['enabled'] and m['kind']=='ring':
            circles.append({'center':[m['x'],m['y']],'radius':m['size']/2})
    return circles


def configuration_id(parameters):
    """Identity of the saved metric map/extrinsics used for one capture."""
    import hashlib
    import json
    selected={key:value for key,value in parameters.items()
              if key.startswith(('field.','localisation.')) or key == 'match.own_goal'}
    return hashlib.sha256(json.dumps(selected,sort_keys=True,separators=(',',':'),
                                    allow_nan=False).encode()).hexdigest()[:16]
