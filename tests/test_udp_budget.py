import msgpack
import asyncio
import pytest

from roki_ng.wire import Fault, UDP_LIMIT, pack, unpack


@pytest.mark.parametrize('size', [1399, 1400, 1401])
def test_complete_datagram_boundary(size):
    assert UDP_LIMIT == 1400
    overhead = len(msgpack.packb({'data': b'x' * 1300}, use_bin_type=True)) - 1300
    message = {'data': b'x' * (size - overhead)}
    raw = msgpack.packb(message, use_bin_type=True)
    assert len(raw) == size
    if size <= UDP_LIMIT:
        assert pack(message) == raw
        assert unpack(raw) == message
    else:
        for operation, argument in ((pack, message), (unpack, raw)):
            with pytest.raises(Fault) as caught:
                operation(argument)
            assert caught.value.code == 'too_large'


def test_reference_client_receives_full_size_sample():
    from roki_ng.client import Client
    from roki_ng.wire import envelope, udp_socket

    async def run():
        source = udp_socket()
        source.bind(('127.0.0.1', 0))
        client = Client(*source.getsockname())
        reader = asyncio.create_task(client._read())
        try:
            message = envelope('sample', 'data.sample', {'data': b'x' * 1200})
            overhead = len(pack(message)) - 1200
            message['body']['data'] = b'x' * (UDP_LIMIT - overhead)
            raw = pack(message)
            assert len(raw) == UDP_LIMIT
            await asyncio.get_running_loop().sock_sendto(
                source, raw, ('127.0.0.1', client.sock.getsockname()[1]))
            received = await asyncio.wait_for(client.events.get(), 1)
            assert received == message
        finally:
            reader.cancel()
            await asyncio.gather(reader, return_exceptions=True)
            await client.close()
            source.close()

    asyncio.run(run())
