"""Built-in body backends, selected once at process startup."""
from importlib import import_module
from pathlib import Path

from roki_ng.wire import Fault

NAMES = ('roki2', 'roki1')


def available():
    root = Path(__file__).parent
    return tuple(name for name in NAMES if (root / name / 'backend.py').is_file())


def select(name=None):
    installed = available()
    if name is not None and name not in NAMES:
        raise Fault('configuration', f'Unknown robot model: {name}')
    if name is not None:
        if name not in installed:
            raise Fault('configuration', f'Robot model {name} is not installed; installed: {", ".join(installed) or "none"}')
        return name
    if not installed:
        raise Fault('configuration', 'No robot body backend installed (roki1 or roki2 required)')
    return 'roki2' if 'roki2' in installed else installed[0]


def load(name=None):
    name = select(name)
    try:
        return import_module(f'{__name__}.{name}.backend')
    except (ImportError, OSError) as exc:
        raise Fault('configuration', f'Cannot load robot model {name}: {exc}') from exc
