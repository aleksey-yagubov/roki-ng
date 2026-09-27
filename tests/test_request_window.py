import asyncio
from collections import Counter
from contextlib import suppress

from roki_ng.supervisor import REQUEST_WINDOW, Session, Supervisor
from roki_ng.wire import envelope, pack, udp_socket, unpack


def test_reordered_requests_and_retries_execute_once(tmp_path):
    async def exercise():
        server = Supervisor({"state_dir": str(tmp_path)})
        server.sock = udp_socket()
        server.sock.bind(("127.0.0.1", 0))
        sock = udp_socket()
        sock.bind(("127.0.0.1", 0))
        session = Session(1, 2, sock.getsockname(), "test")
        server.sessions[session.id] = session
        calls = Counter()
        entered, release = asyncio.Event(), asyncio.Event()

        async def dispatch(_session, op, body):
            calls[op] += 1
            if op == "slow":
                entered.set()
                await release.wait()
            return {"op": op, "calls": calls[op]}

        server.dispatch = dispatch
        receiver = asyncio.create_task(server._receive())
        loop = asyncio.get_running_loop()

        async def send(ident, op, body=None):
            await loop.sock_sendto(sock, pack(envelope(
                "request", op, body or {}, id=ident, session=1, token=2)),
                server.sock.getsockname())

        async def receive():
            raw, _ = await asyncio.wait_for(loop.sock_recvfrom(sock, 1201), 1)
            return unpack(raw)

        async def request(ident, op, body=None):
            await send(ident, op, body)
            reply = await receive()
            assert reply["id"] == ident
            return reply["body"]

        try:
            # The first command datagram was lost/delayed; heartbeat arrived first.
            await request(20, "session.heartbeat")
            original = await request(19, "detection.start")
            assert original["result"]["calls"] == 1
            assert await request(19, "detection.start") == original
            assert (await request(19, "detection.start", {"profile": "other"}))["error"]["code"] == "id_conflict"
            assert (await request(19, "different"))["error"]["code"] == "id_conflict"

            await send(18, "slow")
            await asyncio.wait_for(entered.wait(), 1)
            await send(18, "slow")  # Pending duplicate must not start another job.
            assert (await request(18, "different"))["error"]["code"] == "id_conflict"
            # Move past the pending request and completed response retention window.
            await request(200, "session.heartbeat")
            assert (await request(19, "detection.start"))["error"]["code"] == "stale_request"
            assert (await request(200 - REQUEST_WINDOW, "boundary"))["error"]["code"] == "stale_request"
            assert "result" in await request(201 - REQUEST_WINDOW, "boundary")
            await send(18, "slow")  # Still pending even outside the window.
            await request(201, "session.heartbeat")  # Ensure duplicate was handled.
            release.set()
            reply = await receive()
            assert reply["id"] == 18 and reply["body"]["result"]["calls"] == 1
            assert (await request(18, "slow"))["error"]["code"] == "stale_request"

            # Complete in reverse order: cache eviction cannot use completion order.
            for ident in range(200, 73, -1):
                await request(ident, "session.heartbeat")
            assert len(session.cache) == REQUEST_WINDOW
            await request(202, "session.heartbeat")
            assert len(session.cache) == REQUEST_WINDOW
            assert (await request(74, "session.heartbeat"))["error"]["code"] == "stale_request"
            assert calls["detection.start"] == calls["slow"] == 1
            assert calls["different"] == 0
        finally:
            release.set()
            receiver.cancel()
            with suppress(asyncio.CancelledError):
                await receiver
            await asyncio.gather(*server.tasks)
            sock.close()
            server.sock.close()

    asyncio.run(exercise())
