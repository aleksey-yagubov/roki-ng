import argparse
import asyncio
import fcntl
from pathlib import Path
import signal

from .supervisor import Supervisor


async def serve(config):
    runtime = Supervisor(config)
    stop = asyncio.Event()
    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, stop.set)
    try:
        await runtime.start()
        await stop.wait()
    finally:
        await runtime.close()


def main():
    parser = argparse.ArgumentParser(description="ROKI NG supervisor")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8093)
    parser.add_argument("--state-dir", default="/var/lib/roki-ng")
    parser.add_argument("--uart", default="/dev/ttyAMA5")
    parser.add_argument("--gpiochip", help="Only search this chip for userspace-motherboard-reset")
    parser.add_argument("--mixing-slot", type=int, default=3)
    parser.add_argument("--skip-mixing", action="store_true", help="Do not start the controller mixing slot")
    parser.add_argument("--body-disabled", action="store_true", help="Head only: do not probe or command the body")
    parser.add_argument("--simulate", action="store_true")
    parser.add_argument("--no-head-menu", action="store_true", help="Disable head buttons and voice menu")
    parser.add_argument("--test-video", action="store_true", help="GStreamer videotestsrc and software encoder")
    parser.add_argument("--skip-bootstrap", action="store_true", help="Firmware is already running; skip GPIO/bootloader")
    config = vars(parser.parse_args())
    Path(config["state_dir"]).mkdir(parents=True, exist_ok=True)
    with open(Path(config["state_dir"]) / "supervisor.lock", "w") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        asyncio.run(serve(config))


if __name__ == "__main__":
    main()
