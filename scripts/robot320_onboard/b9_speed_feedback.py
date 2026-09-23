#!/usr/bin/env python3
"""Query-only B9 longitudinal speed feedback; never sends vehicle controls."""

import ctypes as C
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32


class CanConfig(C.Structure):
    _fields_ = [
        ("AccCode", C.c_uint), ("AccMask", C.c_uint), ("Reserved", C.c_uint),
        ("Filter", C.c_ubyte), ("Timing0", C.c_ubyte), ("Timing1", C.c_ubyte),
        ("Mode", C.c_ubyte),
    ]


class CanFrame(C.Structure):
    _fields_ = [
        ("ID", C.c_uint), ("TimeStamp", C.c_uint), ("TimeFlag", C.c_ubyte),
        ("SendType", C.c_ubyte), ("RemoteFlag", C.c_ubyte), ("ExternFlag", C.c_ubyte),
        ("DataLen", C.c_ubyte), ("Data", C.c_ubyte * 8), ("Reserved", C.c_ubyte * 3),
    ]


class B9SpeedFeedback(Node):
    def __init__(self):
        super().__init__("b9_speed_feedback")
        self.speed_pub = self.create_publisher(Float32, "/can/actual_speed_b9", 10)
        self.valid_pub = self.create_publisher(Bool, "/can/actual_speed_b9_valid", 10)
        self.last_response = 0.0
        self.library = C.cdll.LoadLibrary(
            "/home/hs/roboracer_ws/src/robot320_bringup/libcontrolcan.so"
        )
        self.device_type, self.device_index, self.channel = 4, 0, 0
        config = CanConfig(0, 0xFFFFFFFF, 0, 0, 0, 0x1C, 0)
        for function, arguments in (
            (self.library.VCI_OpenDevice, (self.device_type, self.device_index, 0)),
            (self.library.VCI_InitCAN,
             (self.device_type, self.device_index, self.channel, C.byref(config))),
            (self.library.VCI_StartCAN, (self.device_type, self.device_index, self.channel)),
        ):
            if function(*arguments) != 1:
                raise RuntimeError("CAN B9 query initialization failed")
        self.create_timer(0.1, self.poll)
        self.get_logger().info("query-only B9 feedback active; no enable/speed/brake/steering frames")

    def poll(self):
        query = CanFrame()
        query.ID, query.SendType, query.ExternFlag, query.DataLen = 0x020110B9, 1, 1, 2
        self.library.VCI_Transmit(
            self.device_type, self.device_index, self.channel, C.byref(query), 1
        )
        deadline = time.monotonic() + 0.08
        while time.monotonic() < deadline:
            response = CanFrame()
            received = self.library.VCI_Receive(
                self.device_type, self.device_index, self.channel, C.byref(response), 1, 0
            )
            if received != 1:
                time.sleep(0.001)
                continue
            if response.ID == 0x000110B9 and response.DataLen == 2 and response.ExternFlag == 1:
                raw = int.from_bytes(bytes(response.Data[:2]), "little", signed=True)
                self.last_response = time.monotonic()
                self.speed_pub.publish(Float32(data=raw * 0.01 / 3.6))
        valid = time.monotonic() - self.last_response <= 0.5
        self.valid_pub.publish(Bool(data=valid))
        if not valid:
            self.speed_pub.publish(Float32(data=0.0))

    def close(self):
        self.library.VCI_ResetCAN(self.device_type, self.device_index, self.channel)
        self.library.VCI_CloseDevice(self.device_type, self.device_index)


def main():
    rclpy.init()
    node = B9SpeedFeedback()
    try:
        rclpy.spin(node)
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
