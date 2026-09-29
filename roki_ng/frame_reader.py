"""Bounded, event-driven latest-frame reader. iceoryx2 objects stay on one thread."""

import os
import threading

from .dataplane import Channel, FRAME_TOPIC


class FrameReader:
    def __init__(self, consume, topic=FRAME_TOPIC):
        self.consume = consume
        self.topic = topic
        self.error = None
        self.skipped = 0
        self.ready = threading.Event()
        self.stopping = threading.Event()
        self.read_fd, self.write_fd = os.pipe2(os.O_NONBLOCK | os.O_CLOEXEC)
        self.thread = threading.Thread(target=self._run, name="frame-reader", daemon=True)
        self.thread.start()
        if not self.ready.wait(3) or self.error:
            self.close()
            raise RuntimeError(self.error or "Frame subscriber failed to start")

    def _run(self):
        channel = waitset = guard = stop_guard = stop_fd = None
        try:
            channel = Channel(self.topic)
            iox = channel.iox
            waitset = iox.WaitSetBuilder.new().create(iox.ServiceType.Ipc)
            guard = waitset.attach_notification(channel.listener)
            stop_fd = iox.FileDescriptor.non_owning_new(self.read_fd)
            stop_guard = waitset.attach_notification_fd(stop_fd)
            self.ready.set()
            while not self.stopping.is_set():
                waitset.wait_and_process()
                if self.stopping.is_set():
                    break
                channel.listener.try_wait()
                latest = None
                for _ in range(4):
                    sample = channel.receive()
                    if sample is None:
                        break
                    if latest is not None:
                        self.skipped += 1
                    latest = sample
                if latest is not None:
                    payload = latest.payload()
                    view = payload.as_memory_view().cast("B")
                    try:
                        self.consume(view)
                    finally:
                        view.release()
                        del view, payload, latest, sample
        except Exception as exc:
            self.error = str(exc)
        finally:
            self.ready.set()
            stop_guard = guard = waitset = stop_fd = None
            if channel:
                channel.close()

    def close(self):
        self.stopping.set()
        if self.write_fd is not None:
            os.write(self.write_fd, b"x")
            self.thread.join(2)
            if self.thread.is_alive():
                raise RuntimeError("Frame subscriber did not stop")
            os.close(self.write_fd)
            os.close(self.read_fd)
            self.write_fd = self.read_fd = None
