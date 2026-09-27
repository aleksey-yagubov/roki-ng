"""STM bootstrap, read queries and strobe reset; no Rcb4 or body commands."""
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from roki_ng.platform import Bootstrap


def main():
    import Roki
    bootstrap = Bootstrap({"uart": "/dev/ttyAMA5"})
    try:
        if "--skip-bootstrap" not in sys.argv:
            bootstrap.start()
        mb = Roki.Motherboard()
        assert mb.ConfigureACM(), mb.GetError()
        print("ACM", Roki.FindMotherboard(), mb.GetVersion(), flush=True)
        for name in ("GetBodyQueueInfo", "GetIMUContainerInfo", "GetIMULatest", "GetStatus"):
            before = time.monotonic()
            try:
                result = getattr(mb, name)()
                print(name, result, "elapsed", time.monotonic() - before, "error", mb.GetError(), flush=True)
                if isinstance(result, tuple) and result[0]:
                    value = result[1]
                    for key in ("First", "NumAv", "MaxFrames", "Size", "Capacity", "SensorID"):
                        if hasattr(value, key):
                            print(key, getattr(value, key), flush=True)
            except Exception as exc:
                print(name, repr(exc), flush=True)
            time.sleep(0.1)
        for name, args in (("StopStrobeCapture", ()), ("ConfigureStrobeFilterUs", (16667, 4000)),
                           ("StartStrobeCapture", ()), ("GetIMUContainerInfo", ()),
                           ("StopStrobeCapture", ())):
            before = time.monotonic()
            print(name, getattr(mb, name)(*args), "elapsed", time.monotonic() - before,
                  "error", mb.GetError(), flush=True)
        mb.Close()
    finally:
        bootstrap.close()


if __name__ == "__main__":
    main()
