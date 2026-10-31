import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

import pytest

from roki_ng.parameters import Parameters
from roki_ng.wire import Fault


@pytest.mark.parametrize('installed,expected', [
    (('roki1',), 'roki1'), (('roki2',), 'roki2'),
    (('roki1', 'roki2'), 'roki2'), ((), None),
])
def test_actual_package_with_only_installed_models(tmp_path, installed, expected):
    source = Path(__file__).resolve().parents[1]/'roki_ng'
    # Copy only selected backend directories; never remove the developer's files.
    shutil.copytree(source, tmp_path/'roki_ng', ignore=lambda path, names:
                    [n for n in names if n == '__pycache__' or
                     (Path(path).name == 'backends' and n in ('roki1', 'roki2') and n not in installed)])
    script = '''
import json
from roki_ng.body.motion.backends import load
from roki_ng.wire import Fault
def attempt(name):
    try: return load(name).NAME
    except Fault as exc: return exc.code
print(json.dumps([attempt(None), attempt('roki1'), attempt('roki2')]))
'''
    result = subprocess.run([sys.executable, '-c', script], cwd=tmp_path,
                            env=os.environ | {'PYTHONPATH': str(tmp_path)},
                            capture_output=True, text=True, check=True)
    assert json.loads(result.stdout) == [expected or 'configuration',
        'roki1' if 'roki1' in installed else 'configuration',
        'roki2' if 'roki2' in installed else 'configuration']


def test_model_defaults_persist_and_reset(tmp_path):
    p = Parameters(tmp_path, 'roki1')
    assert p.values['walk.gait_height_mm'] == 190.
    assert p.values['walk.step_height_mm'] == 32.
    p.set('walk.step_height_mm', 30.)
    reloaded = Parameters(tmp_path, 'roki1')
    assert reloaded.values['walk.step_height_mm'] == 30.
    reloaded.set('walk.step_height_mm', reloaded.describe('walk.step_height_mm')['default'])
    assert Parameters(tmp_path, 'roki1').values['walk.step_height_mm'] == 32.
    with pytest.raises(Fault, match='another robot model'):
        Parameters(tmp_path, 'roki2')


def test_roki2_keeps_agreed_default(tmp_path):
    p = Parameters(tmp_path, 'roki2')
    assert p.values['walk.step_height_mm'] == 40.
    assert p.describe('walk.step_height_mm')['default'] == 40.
