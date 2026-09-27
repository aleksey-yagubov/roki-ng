import sys
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

from roki_ng import platform


@pytest.fixture
def gpio(monkeypatch):
    chips = {
        "/dev/gpiochip0": ["userspace-bluecoin-reset", None],
        "/dev/gpiochip7": [None, platform.MOTHERBOARD_RESET],
    }

    def open_chip(path):
        chip = MagicMock()
        chip.__enter__.return_value = chip
        chip.get_info.return_value = SimpleNamespace(num_lines=len(chips[path]))
        chip.get_line_info.side_effect = lambda offset: SimpleNamespace(name=chips[path][offset])
        return chip

    module = SimpleNamespace(Chip=open_chip, request_lines=MagicMock(),
                             LineSettings=lambda **kw: kw)
    monkeypatch.setitem(sys.modules, "gpiod", module)
    monkeypatch.setitem(sys.modules, "gpiod.line", SimpleNamespace(
        Direction=SimpleNamespace(OUTPUT="output"),
        Value=SimpleNamespace(ACTIVE=1, INACTIVE=0)))
    monkeypatch.setattr(platform.Path, "glob", lambda *args: [platform.Path(p) for p in chips])
    return module, chips


def test_resolve_named_reset(gpio):
    assert platform.find_reset_chip() == "/dev/gpiochip7"
    assert platform.find_reset_chip("/dev/gpiochip7") == "/dev/gpiochip7"
    with pytest.raises(RuntimeError, match="found 0"):
        platform.find_reset_chip("/dev/gpiochip0")


def test_duplicate_reset_name_rejected(gpio):
    module, chips = gpio
    chips["/dev/gpiochip0"].append(platform.MOTHERBOARD_RESET)
    with pytest.raises(RuntimeError, match="found 2"):
        platform.find_reset_chip()
    module.request_lines.assert_not_called()


def test_bootstrap_never_requests_bluecoin(gpio, monkeypatch):
    module, chips = gpio
    serial = MagicMock()
    port = serial.Serial.return_value.__enter__.return_value
    port.read_until.side_effect = [b"PCB_NAME = MOTHERBOARD-V.1.0\r\n",
                                   b"CONNECTION OK\r\n", b"Starting firmware\r\n"]
    monkeypatch.setitem(sys.modules, "serial", serial)
    monkeypatch.setattr(platform.time, "sleep", lambda _: None)
    bootstrap = platform.Bootstrap({"uart": "/dev/ttyAMA5"})
    bootstrap.start()
    module.request_lines.assert_called_once_with(
        "/dev/gpiochip7", consumer="roki-ng-motherboard-reset",
        config={platform.MOTHERBOARD_RESET: {"direction": "output", "output_value": 1}})
    module.request_lines.return_value.set_value.assert_called_once_with(platform.MOTHERBOARD_RESET, 0)
    bootstrap.close()
    module.request_lines.return_value.release.assert_called_once()
