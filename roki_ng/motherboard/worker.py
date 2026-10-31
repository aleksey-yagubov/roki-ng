"""Current combined process: one ACM owner for motherboard and body traffic.

The body runtime remains in-process until a separate physical body connection
is implemented. No second owner may open this ACM connection.
"""
from roki_ng.body.runtime import Body


def create(config, emit, log):
    return Body(config, emit, log)
