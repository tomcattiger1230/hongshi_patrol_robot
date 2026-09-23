#!/usr/bin/env python3
"""Prefer a verified Wi-Fi reverse tunnel and fall back to wired SIM."""

from __future__ import annotations

import logging
import selectors
import socket
import threading


LOG = logging.getLogger("robot320-tunnel-selector")
LISTEN = ("127.0.0.1", 12220)
BACKENDS = (("wifi", 12222), ("sim", 12226))


def verified_backend(timeout_s: float = 2.0) -> tuple[str, socket.socket, bytes]:
    errors = []
    for name, port in BACKENDS:
        candidate = None
        try:
            candidate = socket.create_connection(("127.0.0.1", port), timeout=timeout_s)
            candidate.settimeout(timeout_s)
            banner = candidate.recv(512)
            if not banner.startswith(b"SSH-"):
                raise OSError("backend did not return an SSH banner")
            return name, candidate, banner
        except OSError as exc:
            errors.append(f"{name}:{exc}")
            if candidate is not None:
                candidate.close()
    raise OSError("; ".join(errors))


def proxy(client: socket.socket) -> None:
    backend = None
    try:
        name, backend, banner = verified_backend()
        LOG.info("selected %s backend", name)
        client.sendall(banner)
        selector = selectors.DefaultSelector()
        selector.register(client, selectors.EVENT_READ, backend)
        selector.register(backend, selectors.EVENT_READ, client)
        while True:
            events = selector.select(timeout=30.0)
            if not events:
                continue
            for key, _ in events:
                source = key.fileobj
                target = key.data
                data = source.recv(65536)
                if not data:
                    return
                target.sendall(data)
    except OSError as exc:
        LOG.warning("connection failed: %s", exc)
    finally:
        client.close()
        if backend is not None:
            backend.close()


def main() -> int:
    logging.basicConfig(level=logging.INFO)
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(LISTEN)
        server.listen(64)
        LOG.info("selector listening on %s:%s", *LISTEN)
        while True:
            client, _ = server.accept()
            threading.Thread(target=proxy, args=(client,), daemon=True).start()


if __name__ == "__main__":
    raise SystemExit(main())
