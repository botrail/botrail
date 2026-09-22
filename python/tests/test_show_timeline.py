"""`Scene.show_timeline`: the bake a studio opened afterwards replays.

The demos bake a cycle, verify it, run a scenario matrix for the report,
and only then open the studio — which replays the *last* bake, so without
this the viewer opens on the last scenario. The server hands a joining
studio that bake in its websocket handshake, so that is what is read
here (server frames only, unmasked, RFC 6455 §5.2).
"""

import base64
import json
import os
import socket
import time

import botrail as bt
from botrail import _core


def _frames(sock: socket.socket, budget: float) -> list[str]:
    """Text frames the server sends within `budget` seconds of the upgrade."""
    sock.settimeout(budget)
    buf = b""
    deadline = time.time() + budget
    while time.time() < deadline:
        try:
            chunk = sock.recv(1 << 16)
        except socket.timeout:
            break
        if not chunk:
            break
        buf += chunk
    head, _, buf = buf.partition(b"\r\n\r\n")
    assert head.startswith(b"HTTP/1.1 101"), head[:80]
    texts = []
    while len(buf) >= 2:
        opcode, length = buf[0] & 0x0F, buf[1] & 0x7F
        offset = 2
        if length == 126:
            length, offset = int.from_bytes(buf[2:4], "big"), 4
        elif length == 127:
            length, offset = int.from_bytes(buf[2:10], "big"), 10
        if len(buf) < offset + length:
            break  # a frame still in flight past the budget
        if opcode == 1:
            texts.append(buf[offset:offset + length].decode())
        buf = buf[offset + length:]
    return texts


def handshake_bakes(url: str) -> list[tuple[str, float]]:
    host, port = url.removeprefix("http://").split(":")
    key = base64.b64encode(os.urandom(16)).decode()
    with socket.create_connection((host, int(port)), timeout=5) as sock:
        sock.sendall(
            f"GET /ws HTTP/1.1\r\nHost: {host}:{port}\r\nUpgrade: websocket\r\n"
            f"Connection: Upgrade\r\nSec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n".encode()
        )
        messages = [json.loads(text) for text in _frames(sock, 1.5)]
    return [(m["sequence"], round(m["timeline"]["duration"], 2))
            for m in messages if m.get("type") == "sequence_result" and m.get("timeline")]


def test_show_timeline_is_what_a_joining_studio_replays() -> None:
    scene = bt.Scene()
    scene.add_box("table", size=(1.0, 1.0, 0.7), position=(0, 0, 0.35))
    scene.add_box("part", size=(0.1, 0.1, 0.1), position=(0, 0, 1.0))
    scene.set_physics("part", dynamic=True, mass=0.2)
    short = scene.sequence("short")
    short.step("wait", transition=bt.seq.elapsed(0.5))
    long = scene.sequence("long")
    long.step("wait", transition=bt.seq.elapsed(1.5))

    nominal = scene.simulate_sequence("long", physics=True)
    scene.simulate_sequence("short", physics=True)  # the matrix's last run

    server = _core.serve_studio(scene, "/nonexistent-studio", "127.0.0.1", 0)
    try:
        assert handshake_bakes(server.url) == [("short", 0.5)]
        scene.show_timeline(nominal)
        assert handshake_bakes(server.url) == [("long", 1.5)]
    finally:
        server.stop()
