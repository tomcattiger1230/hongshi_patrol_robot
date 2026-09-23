#!/usr/bin/env python3
"""Prefer healthy Wi-Fi and fail over Robot320 cloud traffic to wired SIM."""

from __future__ import annotations

import argparse
import logging
import subprocess
import time
from dataclasses import dataclass


LOG = logging.getLogger("robot320-network-failover")


@dataclass
class FailoverState:
    active: str = "wifi"
    failures: int = 0
    successes: int = 0

    def observe(self, wifi_healthy: bool, threshold: int) -> str | None:
        if wifi_healthy:
            self.failures = 0
            self.successes += 1
            if self.active == "sim" and self.successes >= threshold:
                self.active = "wifi"
                self.successes = 0
                return "wifi"
        else:
            self.successes = 0
            self.failures += 1
            if self.active == "wifi" and self.failures >= threshold:
                self.active = "sim"
                self.failures = 0
                return "sim"
        return None


class NetworkFailover:
    def __init__(
        self,
        wifi_connection: str = "HSJC",
        wifi_interface: str = "wlo1",
        probe_host: str = "8.163.54.201",
        threshold: int = 3,
        interval_s: float = 5.0,
    ) -> None:
        self.wifi_connection = wifi_connection
        self.wifi_interface = wifi_interface
        self.probe_host = probe_host
        self.threshold = threshold
        self.interval_s = interval_s
        self.state = FailoverState(active=self._configured_state())

    @staticmethod
    def _run(command: list[str], check: bool = True) -> subprocess.CompletedProcess:
        return subprocess.run(command, check=check, text=True, capture_output=True)

    def _configured_state(self) -> str:
        result = self._run(
            ["nmcli", "-g", "ipv4.route-metric", "connection", "show", self.wifi_connection],
            check=False,
        )
        try:
            metric = int(result.stdout.strip())
        except ValueError:
            metric = 100
        return "wifi" if metric < 600 else "sim"

    def wifi_healthy(self) -> bool:
        address = self._run(
            ["ip", "-4", "-o", "addr", "show", "dev", self.wifi_interface],
            check=False,
        )
        if address.returncode != 0 or "inet " not in address.stdout:
            return False
        probe = self._run(
            ["ping", "-I", self.wifi_interface, "-c", "1", "-W", "2", self.probe_host],
            check=False,
        )
        return probe.returncode == 0

    def switch(self, target: str) -> None:
        metric = "100" if target == "wifi" else "900"
        LOG.warning("switching preferred uplink to %s", target)
        self._restart_long_connections("stop")
        self._run(
            ["nmcli", "connection", "modify", self.wifi_connection, "ipv4.route-metric", metric]
        )
        self._run(["nmcli", "device", "reapply", self.wifi_interface], check=False)
        time.sleep(1.0)
        self._restart_long_connections("start")
        route = self._run(["ip", "route", "get", self.probe_host], check=False)
        LOG.warning("active route after switch: %s", route.stdout.strip())

    @staticmethod
    def _restart_long_connections(action: str) -> None:
        user_env = [
            "runuser", "-u", "hs", "--", "env",
            "XDG_RUNTIME_DIR=/run/user/1000",
            "DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/1000/bus",
            "systemctl", "--user", action,
            "hikvision-video-relay-second.service",
        ]
        subprocess.run(user_env, check=False)
        subprocess.run(
            ["systemctl", action, "hikvision-video-relay@hs.service"],
            check=False,
        )

    def run(self) -> None:
        LOG.info(
            "started: active=%s threshold=%s interval=%ss probe=%s",
            self.state.active,
            self.threshold,
            self.interval_s,
            self.probe_host,
        )
        while True:
            healthy = self.wifi_healthy()
            transition = self.state.observe(healthy, self.threshold)
            LOG.debug(
                "wifi_healthy=%s active=%s failures=%s successes=%s",
                healthy,
                self.state.active,
                self.state.failures,
                self.state.successes,
            )
            if transition:
                self.switch(transition)
            time.sleep(self.interval_s)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--wifi-connection", default="HSJC")
    parser.add_argument("--wifi-interface", default="wlo1")
    parser.add_argument("--probe-host", default="8.163.54.201")
    parser.add_argument("--threshold", type=int, default=3)
    parser.add_argument("--interval", type=float, default=5.0)
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO)
    NetworkFailover(
        wifi_connection=args.wifi_connection,
        wifi_interface=args.wifi_interface,
        probe_host=args.probe_host,
        threshold=max(1, args.threshold),
        interval_s=max(1.0, args.interval),
    ).run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
