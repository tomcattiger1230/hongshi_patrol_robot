#!/usr/bin/env python3
"""Allowlisted client for the Robot320 lift service's local Unix socket."""

from __future__ import annotations

import argparse
import json
import os
import socket
import sys


ACTIONS = {"raise", "lower", "stop"}
DEFAULT_SOCKET = f"/run/user/{os.getuid()}/robot320-lift.sock"


def send_action(action: str, socket_path: str = DEFAULT_SOCKET) -> str:
    if action not in ACTIONS:
        raise ValueError(f"unsupported lift action: {action}")
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
        client.settimeout(6.0)
        client.connect(socket_path)
        client.sendall((action + "\n").encode("ascii"))
        client.shutdown(socket.SHUT_WR)
        response = client.recv(512).decode("utf-8", errors="replace").strip()
    if not response.startswith("ok "):
        raise RuntimeError(response or "empty response from lift service")
    return response


def stream(socket_path: str) -> int:
    print(json.dumps({"ready": True}), flush=True)
    for line in sys.stdin:
        action = line.strip()
        try:
            response = send_action(action, socket_path)
            print(json.dumps({"ok": True, "action": action, "response": response}), flush=True)
        except Exception as exc:
            print(json.dumps({"ok": False, "action": action, "error": str(exc)}), flush=True)
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Robot320 lift allowlisted control client")
    parser.add_argument("action", nargs="?", choices=sorted(ACTIONS))
    parser.add_argument("--stream", action="store_true")
    parser.add_argument("--socket-path", default=DEFAULT_SOCKET)
    args = parser.parse_args(argv)
    if args.stream:
        return stream(args.socket_path)
    if args.action is None:
        parser.error("action is required unless --stream is used")
    print(send_action(args.action, args.socket_path))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
