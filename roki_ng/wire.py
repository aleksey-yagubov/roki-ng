"""Bounded MessagePack framing shared by operator and local IPC."""

import math
import socket
import time

import msgpack

UDP_LIMIT = 1400  # Entire UDP payload, including the MessagePack envelope.
IPC_LIMIT = 65536


class Fault(Exception):
    def __init__(self, code, message, retryable=False):
        super().__init__(message)
        self.code = code
        self.retryable = retryable

    def as_dict(self):
        return {"code": self.code, "message": str(self)[:240], "retryable": self.retryable}


def pack(message, limit=UDP_LIMIT):
    data = msgpack.packb(message, use_bin_type=True)
    if len(data) > limit:
        raise Fault("too_large", f"Message exceeds {limit} bytes; use pagination")
    return data


def unpack(data, limit=UDP_LIMIT):
    if len(data) > limit:
        raise Fault("too_large", "Datagram too large")
    try:
        value = msgpack.unpackb(data, raw=False, strict_map_key=True,
                               max_array_len=256, max_map_len=128,
                               max_str_len=limit, max_bin_len=limit, max_ext_len=0)
    except (ValueError, TypeError, msgpack.UnpackException) as exc:
        raise Fault("bad_message", "Invalid MessagePack") from exc
    if not isinstance(value, dict) or not all(isinstance(k, str) for k in value):
        raise Fault("bad_message", "Expected a map with string keys")
    return value


def number(body, key, default, low, high, integer=False):
    value = body.get(key, default)
    if (isinstance(value, bool) or not isinstance(value, (int, float))
            or not math.isfinite(value) or not low <= value <= high
            or (integer and not isinstance(value, int))):
        raise Fault("invalid_argument", f"{key}: expected {'integer' if integer else 'number'} in [{low}, {high}]")
    return value


def boolean(body, key, default=False):
    value = body.get(key, default)
    if not isinstance(value, bool):
        raise Fault("invalid_argument", f"{key}: expected boolean")
    return value


def choice(body, key, default, choices):
    value = body.get(key, default)
    if value not in choices:
        raise Fault("invalid_argument", f"{key}: expected one of {list(choices)}")
    return value


def page(items, body, max_limit=8):
    offset = number(body, "offset", 0, 0, 100000, True)
    limit = number(body, "limit", max_limit, 1, max_limit, True)
    end = min(len(items), offset + limit)
    return {"items": items[offset:end], "next_offset": end if end < len(items) else None,
            "total": len(items)}


def udp_socket():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # Linux constants are not exported by every CPython build.
    sock.setsockopt(socket.IPPROTO_IP, getattr(socket, "IP_MTU_DISCOVER", 10), 2)
    sock.setblocking(False)
    return sock


def envelope(kind, op="", body=None, *, session=0, token=0, id=0, sequence=0):
    return {"v": 1, "kind": kind, "session": session, "token": token,
            "id": id, "sequence": sequence, "robot_mono_ns": time.monotonic_ns(),
            "op": op, "body": {} if body is None else body}
