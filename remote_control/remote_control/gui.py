#!/usr/bin/env python3
"""Simple remote control GUI prototype."""

from __future__ import annotations

import argparse
import queue
import threading
import tkinter as tk
from tkinter import ttk

from mobile_platform.transport import UdpEndpoint

from .dds_client import RobotRemoteClient


def parse_endpoint(value: str) -> UdpEndpoint:
    host, port = value.rsplit(":", 1)
    return UdpEndpoint(host, int(port))


class RemoteControlApp:
    def __init__(self, root: tk.Tk, client: RobotRemoteClient):
        self.root = root
        self.client = client
        self.telemetry_queue: queue.Queue[str] = queue.Queue()
        self.linear = tk.DoubleVar(value=0.25)
        self.angular = tk.DoubleVar(value=0.5)
        self.status = tk.StringVar(value="等待遥测")
        self._running = True

        self._build_ui()
        self._start_telemetry_thread()
        self.root.after(100, self._poll_telemetry)
        self.root.protocol("WM_DELETE_WINDOW", self.close)

    def _build_ui(self) -> None:
        self.root.title("Robot320 远程控制")
        self.root.geometry("520x360")

        main = ttk.Frame(self.root, padding=16)
        main.pack(fill=tk.BOTH, expand=True)

        ttk.Label(main, textvariable=self.status).pack(anchor=tk.W, pady=(0, 12))

        speed_frame = ttk.LabelFrame(main, text="手动控制")
        speed_frame.pack(fill=tk.X)

        ttk.Label(speed_frame, text="线速度 m/s").grid(row=0, column=0, sticky=tk.W, padx=8, pady=8)
        ttk.Scale(speed_frame, variable=self.linear, from_=0.0, to=0.8, orient=tk.HORIZONTAL).grid(
            row=0, column=1, sticky=tk.EW, padx=8, pady=8
        )
        ttk.Label(speed_frame, text="角速度 rad/s").grid(row=1, column=0, sticky=tk.W, padx=8, pady=8)
        ttk.Scale(speed_frame, variable=self.angular, from_=0.0, to=1.2, orient=tk.HORIZONTAL).grid(
            row=1, column=1, sticky=tk.EW, padx=8, pady=8
        )
        speed_frame.columnconfigure(1, weight=1)

        buttons = ttk.Frame(main)
        buttons.pack(fill=tk.BOTH, expand=True, pady=16)

        ttk.Button(buttons, text="前进", command=lambda: self._move(1, 0)).grid(row=0, column=1, padx=6, pady=6)
        ttk.Button(buttons, text="左转", command=lambda: self._move(1, 1)).grid(row=1, column=0, padx=6, pady=6)
        ttk.Button(buttons, text="停止", command=self.client.stop).grid(row=1, column=1, padx=6, pady=6)
        ttk.Button(buttons, text="右转", command=lambda: self._move(1, -1)).grid(row=1, column=2, padx=6, pady=6)
        ttk.Button(buttons, text="后退", command=lambda: self._move(-1, 0)).grid(row=2, column=1, padx=6, pady=6)

        safety = ttk.Frame(main)
        safety.pack(fill=tk.X)
        ttk.Button(safety, text="刹车", command=self.client.brake).pack(side=tk.LEFT, padx=(0, 8))
        ttk.Button(safety, text="急停", command=self.client.emergency_stop).pack(side=tk.LEFT, padx=(0, 8))
        ttk.Button(safety, text="解除急停/空闲", command=self.client.reset_idle).pack(side=tk.LEFT)

    def _move(self, linear_sign: int, angular_sign: int) -> None:
        self.client.send_manual_command(
            linear_speed_mps=linear_sign * self.linear.get(),
            angular_speed_radps=angular_sign * self.angular.get(),
        )

    def _start_telemetry_thread(self) -> None:
        thread = threading.Thread(target=self._telemetry_loop, daemon=True)
        thread.start()

    def _telemetry_loop(self) -> None:
        while self._running:
            telemetry = self.client.receive_telemetry(timeout_s=0.2)
            if telemetry:
                chassis = telemetry.chassis
                self.telemetry_queue.put(
                    f"连接:{chassis.connected}  使能:{chassis.enabled}  "
                    f"速度:{chassis.speed_kmh} km/h  转速:{chassis.commanded_rpm} RPM  "
                    f"转向:{chassis.steering_direction} {chassis.steering_angle_deg}°"
                )

    def _poll_telemetry(self) -> None:
        while not self.telemetry_queue.empty():
            self.status.set(self.telemetry_queue.get())
        if self._running:
            self.root.after(100, self._poll_telemetry)

    def close(self) -> None:
        self._running = False
        self.client.stop()
        self.client.close()
        self.root.destroy()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Robot320 remote GUI")
    parser.add_argument("--robot", default="127.0.0.1:15000")
    parser.add_argument("--telemetry-bind", default="0.0.0.0:15001")
    return parser


def main() -> None:
    args = build_parser().parse_args()
    client = RobotRemoteClient(parse_endpoint(args.robot), parse_endpoint(args.telemetry_bind))
    root = tk.Tk()
    RemoteControlApp(root, client)
    root.mainloop()


if __name__ == "__main__":
    main()
