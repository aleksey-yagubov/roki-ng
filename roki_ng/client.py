"""Small reference client, suitable as transport foundation for a Qt adapter."""

import argparse
import asyncio
import ipaddress
import json
import uuid

from .wire import Fault, UDP_LIMIT, envelope, pack, unpack, udp_socket


class Client:
    def __init__(self, host, port=8093):
        self.address = (str(ipaddress.IPv4Address(host)), port)
        self.sock = udp_socket()
        self.sock.bind(("0.0.0.0", 0))
        self.session = self.token = self.lease_epoch = 0
        self.id = 0
        self.sequence = 0
        self.pending = {}
        self.events = asyncio.Queue(maxsize=128)
        self.tasks = []

    async def connect(self):
        self.tasks.append(asyncio.create_task(self._read()))
        welcome = await self._exchange("hello", "hello", {
            "versions": [1], "client_name": "roki-reference-client", "client_instance": uuid.uuid4().hex})
        self.session, self.token = welcome["session"], welcome["token"]
        self.tasks.append(asyncio.create_task(self._heartbeat()))
        return welcome["body"]

    async def _read(self):
        while True:
            raw, address = await asyncio.get_running_loop().sock_recvfrom(self.sock, UDP_LIMIT + 1)
            if address != self.address:
                continue
            try:
                msg = unpack(raw)
            except Fault:
                continue
            if msg.get("v") != 1:
                continue
            if self.session and (msg.get("session") != self.session or msg.get("token") != self.token):
                continue
            future = self.pending.get(msg.get("id"))
            if msg.get("kind") in ("response", "welcome") and future is not None:
                if not future.done():
                    future.set_result(msg)
            elif msg.get("kind") in ("event", "sample"):
                if self.events.full():
                    self.events.get_nowait()
                self.events.put_nowait(msg)

    async def _exchange(self, kind, op, body):
        # No await while assigning IDs. A slow camera request must not block heartbeat.
        self.id += 1
        ident = self.id
        future = asyncio.get_running_loop().create_future()
        self.pending[ident] = future
        data = pack(envelope(kind, op, body, session=self.session, token=self.token, id=ident))
        timeout = 5.0 if op in ("camera.start", "camera.stop", "control.acquire") else 0.25
        try:
            for _ in range(5):
                await asyncio.get_running_loop().sock_sendto(self.sock, data, self.address)
                try:
                    return await asyncio.wait_for(asyncio.shield(future), timeout)
                except TimeoutError:
                    pass
            raise Fault("timeout", op)
        finally:
            self.pending.pop(ident, None)
            if not future.done():
                future.cancel()

    async def request(self, op, body=None):
        body = dict(body or {})
        if self.lease_epoch:
            body.setdefault("lease_epoch", self.lease_epoch)
        reply = await self._exchange("request", op, body)
        if "error" in reply["body"]:
            error = reply["body"]["error"]
            raise Fault(error["code"], error["message"], error.get("retryable", False))
        result = reply["body"]["result"]
        if op == "control.acquire":
            self.lease_epoch = result["lease_epoch"]
        elif op == "control.release":
            self.lease_epoch = 0
        return result

    async def drive(self, x=0, y=0, yaw=0, speed=0.5, crouch="off", heading_hold=False):
        self.sequence += 1
        data = pack(envelope("sample", "motion.drive", {
            "lease_epoch": self.lease_epoch, "x": x, "y": y, "yaw": yaw,
            "speed": speed, "crouch": crouch, "heading_hold": heading_hold,
        }, session=self.session, token=self.token, sequence=self.sequence))
        await asyncio.get_running_loop().sock_sendto(self.sock, data, self.address)

    async def _heartbeat(self):
        while True:
            await asyncio.sleep(0.5)
            try:
                await self.request("session.heartbeat")
            except Fault as exc:
                if self.events.full():
                    self.events.get_nowait()
                self.events.put_nowait({"kind": "event", "op": "client.connection_error", "body": exc.as_dict()})

    async def close(self):
        if self.session:
            try:
                await self.request("session.close")
            except Fault:
                pass
        for task in self.tasks:
            task.cancel()
        await asyncio.gather(*self.tasks, return_exceptions=True)
        self.sock.close()


async def cli(args):
    client = Client(args.robot, args.port)
    event_task = None
    try:
        print(json.dumps(await client.connect(), ensure_ascii=False))
        if args.control:
            await client.request("control.acquire")
            await client.request("mode.set", {"mode": "MANUAL"})
        await client.request("log.subscribe")

        async def events():
            while True:
                print(json.dumps(await client.events.get(), ensure_ascii=False), flush=True)

        event_task = asyncio.create_task(events())
        print('Enter: operation {JSON arguments}; e.g. motion.pose {"name":"base_stand"}; quit to exit')
        while True:
            try:
                line = await asyncio.to_thread(input, "> ")
            except EOFError:
                break
            if line.strip() in ("quit", "exit"):
                break
            if not line.strip():
                continue
            op, _, body = line.partition(" ")
            try:
                print(json.dumps(await client.request(op, json.loads(body or "{}")), ensure_ascii=False))
            except (Fault, ValueError) as exc:
                print(f"Error: {exc}")
    finally:
        if event_task:
            event_task.cancel()
        await client.close()


def main():
    parser = argparse.ArgumentParser(description="Interactive ROKI NG protocol client")
    parser.add_argument("--robot", required=True)
    parser.add_argument("--port", type=int, default=8093)
    parser.add_argument("--control", action="store_true")
    try:
        asyncio.run(cli(parser.parse_args()))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
