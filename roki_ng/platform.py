"""Reset and UART bootloader phase; runtime handoff uses USB ACM."""

from pathlib import Path
import time


MOTHERBOARD_RESET = "userspace-motherboard-reset"


def find_reset_chip(chip_path=None):
    import gpiod

    paths = [Path(chip_path)] if chip_path else sorted(Path("/dev").glob("gpiochip*"))
    matches = []
    for path in paths:
        with gpiod.Chip(str(path)) as chip:
            for offset in range(chip.get_info().num_lines):
                if chip.get_line_info(offset).name == MOTHERBOARD_RESET:
                    matches.append(str(path))
    if len(matches) != 1:
        raise RuntimeError(
            f"Expected exactly one GPIO named {MOTHERBOARD_RESET!r}; found {len(matches)}")
    return matches[0]


def expect_line(port, expected, timeout=2):
    deadline = time.monotonic() + timeout
    seen = []
    while time.monotonic() < deadline:
        line = port.read_until(b"\n", 256).strip()
        if line == expected:
            return
        if line:
            seen.append(line)
    raise RuntimeError(f"Motherboard expected {expected!r}, received {seen!r}")


class Bootstrap:
    def __init__(self, config):
        self.config = config
        self.lines = None

    def start(self):
        import gpiod
        import serial
        from gpiod.line import Direction, Value

        cfg = self.config
        chip_path = find_reset_chip(cfg.get("gpiochip"))
        self.lines = gpiod.request_lines(
            chip_path, consumer="roki-ng-motherboard-reset",
            config={MOTHERBOARD_RESET: gpiod.LineSettings(
                direction=Direction.OUTPUT, output_value=Value.ACTIVE)})
        time.sleep(0.2)
        self.lines.set_value(MOTHERBOARD_RESET, Value.INACTIVE)
        with serial.Serial(cfg["uart"], 921600, timeout=0.1, write_timeout=0.5,
                           exclusive=True) as port:
            deadline, next_query = time.monotonic() + 5, 0
            while time.monotonic() < deadline:
                if time.monotonic() >= next_query:
                    port.write(b"CMD:GET_PCB_NAME\r\n")
                    next_query = time.monotonic() + 0.25
                if port.read_until(b"\n", 256).strip() == b"PCB_NAME = MOTHERBOARD-V.1.0":
                    break
            else:
                raise RuntimeError("Motherboard bootloader did not identify")
            port.reset_input_buffer()
            port.write(b"CMD:GET_CONNECTION_STATE\r\n")
            expect_line(port, b"CONNECTION OK")
            port.write(b"CMD:START_FW\r\n")
            port.flush()
            expect_line(port, b"Starting firmware")
            # The bootloader reply precedes application initialization. On CM4,
            # querying Roki after only 0.3 s leaves the firmware unresponsive.
            time.sleep(2)
        # Keep reset released. Only the motherboard worker opens runtime ACM.

    def close(self):
        if self.lines is not None:
            self.lines.release()
            self.lines = None
