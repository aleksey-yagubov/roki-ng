"""Bounded iceoryx2 loans and event notifications. No serialized image copies."""

import ctypes
import struct
from contextlib import contextmanager

FRAME_TOPIC = "roki/camera/frame/v1"
LOCALISATION_TOPIC = "roki/localisation/frame/v1"
IMU_TOPIC = "roki/motherboard/imu/v1"
DETECTIONS_TOPIC = "roki/detection/blobs/v1"
FRAME_HEADER = struct.Struct("<IQIII")  # Unicam u32, sensor ns, width, height, stride
IMU_RECORD = struct.Struct("<IQffffI")  # STM sequence, Bosch sensor ns, quaternion xyzw, sensor ID
FRAME_BYTES = FRAME_HEADER.size + 800 * 650 * 3
TOPICS = {FRAME_TOPIC: (4, FRAME_BYTES), LOCALISATION_TOPIC: (4, FRAME_BYTES), IMU_TOPIC: (128, IMU_RECORD.size),
          DETECTIONS_TOPIC: (4, 4096)}


class Channel:
    def __init__(self, name, *, publisher=False):
        import iceoryx2 as iox
        self.iox = iox
        self.node = iox.NodeBuilder.new().create(iox.ServiceType.Ipc)
        capacity, size = TOPICS[name]
        self.events = (self.node.service_builder(iox.ServiceName.new(name))
                       .event().max_notifiers(1).max_listeners(8).max_nodes(9)
                       .open_or_create())
        # Subscribe to notifications before data: a concurrent send cannot be missed.
        self.listener = None if publisher else self.events.listener_builder().create()
        self.service = (self.node.service_builder(iox.ServiceName.new(name))
                        .publish_subscribe(iox.Slice[ctypes.c_uint8])
                        .max_publishers(1).max_subscribers(8).max_nodes(9)
                        .subscriber_max_buffer_size(capacity)
                        .subscriber_max_borrowed_samples(4).history_size(0)
                        .enable_safe_overflow(True).open_or_create())
        self.publisher = self.subscriber = self.notifier = None
        if publisher:
            self.publisher = (self.service.publisher_builder().initial_max_slice_len(size)
                              .max_loaned_samples(1)
                              .backpressure_strategy(iox.BackpressureStrategy.DiscardData).create())
            self.notifier = self.events.notifier_builder().create()
        else:
            self.subscriber = self.service.subscriber_builder().create()

    @contextmanager
    def loan(self, size):
        """Fill the entire view; publishing occurs only on successful context exit."""
        sample = self.publisher.loan_slice_uninit(size)
        payload = sample.payload()
        view = payload.as_memory_view().cast("B")
        try:
            yield view
        except BaseException:
            raise
        else:
            view.release()
            del payload
            sample = sample.assume_init()
            sample.send()
            self.notifier.notify_with_custom_event_id(self.iox.EventId.new(0))
        finally:
            view.release()

    def receive(self):
        """Caller must retain sample for the entire lifetime of any payload view."""
        return self.subscriber.receive()

    def has_subscribers(self):
        """Current demand, independent of whether any frames have been sent."""
        return self.service.dynamic_config().number_of_subscribers() > 0

    def drain(self):
        self.listener.try_wait()
        while (sample := self.receive()) is not None:
            del sample

    def close(self):
        self.subscriber = self.publisher = self.notifier = self.listener = None
        self.service = self.events = self.node = None
