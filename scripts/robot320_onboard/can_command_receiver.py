#!/usr/bin/env python3
"""
CAN 命令接收节点 - 正确解析速度反馈
速度反馈: 扩展请求 0x020110B9 → 扩展响应 0x000110B9，raw×0.01km/h
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32
import time
import math
from ctypes import *

# ============ CAN 结构体 ============
class VCI_INIT_CONFIG(Structure):
    _fields_ = [
        ("AccCode", c_uint),
        ("AccMask", c_uint),
        ("Reserved", c_uint),
        ("Filter", c_ubyte),
        ("Timing0", c_ubyte),
        ("Timing1", c_ubyte),
        ("Mode", c_ubyte)
    ]

class VCI_CAN_OBJ(Structure):
    _fields_ = [
        ("ID", c_uint),
        ("TimeStamp", c_uint),
        ("TimeFlag", c_ubyte),
        ("SendType", c_ubyte),
        ("RemoteFlag", c_ubyte),
        ("ExternFlag", c_ubyte),
        ("DataLen", c_ubyte),
        ("Data", c_ubyte * 8),
        ("Reserved", c_ubyte * 3)
    ]

# ============ 常量 ============
VCI_USBCAN2 = 4
STATUS_OK = 1
DEVICE_INDEX = 0
CAN1_INDEX = 0

class CANCommandReceiver(Node):
    def __init__(self):
        super().__init__('can_receiver')

        self.canDLL = None
        self.device_ready = False
        self.brake_released = False
        self.declare_parameter('command_timeout', 0.6)
        self.command_timeout = float(self.get_parameter('command_timeout').value)
        if self.command_timeout <= 0.0:
            raise ValueError('command_timeout must be greater than zero')

        # 发布实际反馈
        self.actual_rpm_pub = self.create_publisher(Float32, '/can/actual_rpm', 10)
        self.actual_speed_pub = self.create_publisher(Float32, '/can/actual_speed', 10)
        self.b9_valid_pub = self.create_publisher(Bool, '/can/actual_speed_b9_valid', 10)
        self.actual_steering_pub = self.create_publisher(Float32, '/can/actual_steering', 10)

        # 订阅速度命令
        self.speed_sub = self.create_subscription(Float32, '/can/speed_cmd', self.speed_callback, 10)
        self.steering_sub = self.create_subscription(Float32, '/can/steering_cmd', self.steering_callback, 10)

        self.current_rpm = 0
        self.current_steering_deg = 0
        self.last_speed_command_time = time.monotonic()
        self.prev_rpm = 0
        self.frame_count = 0
        self.last_speed_raw_log_time = 0.0
        self.last_b9_response_time = 0.0
        self.last_b9_query_time = 0.0
        self.b9_query_interval = 0.1
        self.b9_feedback_timeout = 0.5
        self.b9_query_id = 0x020110B9
        self.b9_response_id = 0x000110B9
        self.last_frame_log_times = {}

        # 实际反馈状态
        self.actual_rpm = 0.0
        self.actual_speed = 0.0
        self.actual_steering = 0.0

        self.wheel_diameter = 0.43

        # ============================================================
        # 缩放因子: 反馈值 / 实际RPM
        # 根据测试: 指令4 → 反馈63~71
        # 所以 反馈值 ≈ RPM × 16
        # ============================================================
        self.rpm_scale_factor = 16.0

        self.init_device()
        self.timer = self.create_timer(0.02, self.send_and_receive)

        self.get_logger().info('='*50)
        self.get_logger().info('📡 Robot320 CAN 节点')
        self.get_logger().info('  速度反馈: B9 请求 0x020110B9 → 响应 0x000110B9')
        self.get_logger().info('  启动安全策略: 不自动释放刹车，等待非零速度命令')
        self.get_logger().info(f'  速度命令看门狗: {self.command_timeout:.2f}s')
        self.get_logger().info(f'  缩放因子: {self.rpm_scale_factor}')
        self.get_logger().info('='*50)

    def init_device(self):
        try:
            import os
            lib_path = os.path.expanduser('~/roboracer_ws/src/robot320_bringup/libcontrolcan.so')
            if not os.path.exists(lib_path):
                lib_path = './libcontrolcan.so'
            self.canDLL = cdll.LoadLibrary(lib_path)
            self.get_logger().info('✅ 驱动库加载成功')
        except Exception as e:
            self.get_logger().error(f'❌ 驱动库加载失败: {e}')
            return

        ret = self.canDLL.VCI_OpenDevice(VCI_USBCAN2, DEVICE_INDEX, 0)
        if ret != STATUS_OK:
            self.get_logger().error('❌ 打开设备失败!')
            self.canDLL = None
            return
        self.get_logger().info('✅ 设备打开成功')

        config = VCI_INIT_CONFIG()
        config.AccCode = 0
        config.AccMask = 0xFFFFFFFF
        config.Filter = 0
        config.Timing0 = 0x00
        config.Timing1 = 0x1C
        config.Mode = 0

        ret = self.canDLL.VCI_InitCAN(VCI_USBCAN2, DEVICE_INDEX, CAN1_INDEX, byref(config))
        if ret != STATUS_OK:
            self.get_logger().error('❌ Init CAN1 失败!')
            self.canDLL.VCI_CloseDevice(VCI_USBCAN2, DEVICE_INDEX)
            self.canDLL = None
            return
        self.get_logger().info('✅ CAN1 初始化成功 (500kbps)')

        ret = self.canDLL.VCI_StartCAN(VCI_USBCAN2, DEVICE_INDEX, CAN1_INDEX)
        if ret != STATUS_OK:
            self.get_logger().error('❌ Start CAN1 失败!')
            self.canDLL.VCI_CloseDevice(VCI_USBCAN2, DEVICE_INDEX)
            self.canDLL = None
            return
        self.get_logger().info('✅ CAN1 启动成功')

        self.device_ready = True
        self.get_logger().info('✅ CAN 设备就绪')
        # Do not release the brake during startup; only an explicit non-zero speed command may do so.

    def release_brake(self):
        if not self.device_ready or self.canDLL is None:
            return False
        self.get_logger().info('🔄 释放刹车...')
        # Robot320 protocol: 0x06 releases the electromagnetic brake.
        # The previous 0x02/0x02/0x58 payload kept the brake applied.
        data = [0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        ok = self.send_critical_frame(0x000007B9, data, 0, 'release brake')
        self.brake_released = ok
        if ok:
            self.get_logger().info('✅ 刹车已释放')
        return ok

    def send_can_frame(self, msg_id, data, is_extended=1):
        if not self.device_ready or self.canDLL is None:
            return False
        msg = VCI_CAN_OBJ()
        msg.ID = msg_id
        msg.SendType = 1
        msg.RemoteFlag = 0
        msg.ExternFlag = is_extended
        msg.DataLen = min(len(data), 8)
        for i in range(msg.DataLen):
            msg.Data[i] = data[i]
        ret = self.canDLL.VCI_Transmit(VCI_USBCAN2, DEVICE_INDEX, CAN1_INDEX, byref(msg), 1)
        return ret == STATUS_OK

    def send_critical_frame(self, msg_id, data, is_extended, description, attempts=3):
        """Retry safety-critical state changes and report hard failure."""
        for attempt in range(1, attempts + 1):
            if self.send_can_frame(msg_id, data, is_extended):
                return True
            self.get_logger().warning(
                f'{description} CAN transmit failed (attempt {attempt}/{attempts})'
            )
            time.sleep(0.02)
        self.get_logger().error(f'{description} CAN transmit failed; motion inhibited')
        return False

    def receive_can_frames(self):
        """读取并解析 CAN 帧"""
        if not self.device_ready or self.canDLL is None:
            return

        # The chassis bus carries roughly 800 frames/s. At a 50 Hz timer,
        # reading only five frames per cycle builds an ever-growing backlog
        # and makes B9 speed feedback appear several seconds late. Drain a
        # bounded burst so control feedback stays close to real time.
        for _ in range(100):
            msg = VCI_CAN_OBJ()
            ret = self.canDLL.VCI_Receive(VCI_USBCAN2, DEVICE_INDEX, CAN1_INDEX, byref(msg), 1, 0)

            if ret != STATUS_OK:
                break

            msg_id = msg.ID
            data = [msg.Data[i] for i in range(msg.DataLen)]

            # Diagnostic only: log every received CAN ID once per second with
            # the complete 8-byte payload. No parsing or behavior is changed.
            now = time.monotonic()
            last_log = self.last_frame_log_times.get(msg_id, 0.0)
            if now - last_log >= 1.0:
                full_data = data + [0] * (8 - len(data))
                raw_hex = ' '.join(f'{byte:02X}' for byte in full_data[:8])
                self.get_logger().info(
                    f'📡 CAN RX: id=0x{msg_id:08X}, len={msg.DataLen}, '
                    f'extended={getattr(msg, "ExternFlag", 0)}, data=[{raw_hex}]'
                )
                self.last_frame_log_times[msg_id] = now

            # B9 speed response: signed little-endian value in 0.01 km/h.
            if msg_id == self.b9_response_id and msg.DataLen == 2 and getattr(msg, "ExternFlag", 0) == 1:
                raw_value = int.from_bytes(bytes(data[0:2]), byteorder='little', signed=True)
                self.last_b9_response_time = time.monotonic()
                self.actual_speed = raw_value * 0.01 / 3.6
                self.actual_rpm = self.actual_speed * 60.0 / (math.pi * self.wheel_diameter)
                self.get_logger().debug(f'B9 speed: raw={raw_value}, speed={self.actual_speed:.4f}m/s')

            # Keep 0x6FA as diagnostic only; it must not update actual_speed.
            if msg_id == 0x000006FA and msg.DataLen >= 4:
                raw_value = int.from_bytes(bytes(data[1:3]), byteorder='little', signed=True)
                now = time.monotonic()
                if now - self.last_speed_raw_log_time >= 1.0:
                    raw_hex = ' '.join(f'{byte:02X}' for byte in data)
                    self.get_logger().info(f'🔎 CAN 0x6FA diagnostic: len={msg.DataLen}, data=[{raw_hex}], le_s16(data[1:3])={raw_value}')
                    self.last_speed_raw_log_time = now

            # ============================================================
            # 转向反馈: 0x000006FB, data[0-1] = 角度
            # ============================================================
            if msg_id == 0x000006FB and msg.DataLen >= 2:
                angle_raw = int.from_bytes(bytes(data[0:2]), byteorder='little', signed=True)
                if abs(angle_raw) < 500:
                    self.actual_steering = angle_raw * 0.1

    def speed_callback(self, msg):
        self.current_rpm = msg.data
        self.last_speed_command_time = time.monotonic()

    def steering_callback(self, msg):
        self.current_steering_deg = msg.data

    def send_and_receive(self):
        if (
            self.current_rpm != 0
            and time.monotonic() - self.last_speed_command_time > self.command_timeout
        ):
            self.get_logger().error(
                f'速度命令超过 {self.command_timeout:.2f}s 未更新，强制停车'
            )
            self.current_rpm = 0
            self.current_steering_deg = 0

        rpm = int(self.current_rpm)

        if rpm != 0 and self.prev_rpm == 0:
            self.get_logger().info(f'🚀 使能: RPM={rpm}')
            if not self.send_critical_frame(
                0x03011008, [0x0A, 0x00], 1, 'enable motor'
            ):
                self.current_rpm = 0
                return
            # Follow the vendor startup sequence: enable the drive first,
            # then release the brake after the controller has settled.
            time.sleep(0.2)
            if not self.release_brake():
                self.send_critical_frame(
                    0x03011008, [0x01, 0x00], 1, 'disable motor after brake failure'
                )
                self.current_rpm = 0
                return
            time.sleep(0.2)
        elif rpm == 0 and self.prev_rpm != 0:
            self.get_logger().info('🛑 失能')
            self.send_critical_frame(0x03011008, [0x01, 0x00], 1, 'disable motor')
            self.brake_released = False

        self.prev_rpm = rpm

        # 发送速度指令
        if rpm != 0:
            speed_bytes = rpm.to_bytes(2, byteorder='little', signed=True)
            data = [speed_bytes[0] & 0xFF, speed_bytes[1] & 0xFF]
            self.send_can_frame(0x030110BA, data, 1)

        # 发送转向指令
        angle_deg = self.current_steering_deg
        if angle_deg == 0:
            data = [0x02, 0x75, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00]
        else:
            if angle_deg > 0:
                val = (3000 + angle_deg) / 0.1
            else:
                val = (3000 - abs(angle_deg)) / 0.1
            val_int = int(val)
            byte1 = (val_int >> 8) & 0xFF
            byte2 = val_int & 0xFF
            data = [0x02, byte1, byte2, 0x00, 0x00, 0x00, 0x00, 0x00]
        self.send_can_frame(0x00000169, data, 0)

        # Query the validated B9 speed response at the controller's tested
        # 10 Hz rate. Sending this on every 50 Hz timer tick can suppress or
        # delay responses on the already busy chassis bus.
        now = time.monotonic()
        if now - self.last_b9_query_time >= self.b9_query_interval:
            self.send_can_frame(self.b9_query_id, [0x00, 0x00], 1)
            self.last_b9_query_time = now
        # 读取反馈
        self.receive_can_frames()
        b9_valid = time.monotonic() - self.last_b9_response_time <= self.b9_feedback_timeout
        if not b9_valid:
            self.actual_speed = 0.0
            self.actual_rpm = 0.0
        # 发布反馈
        self.actual_rpm_pub.publish(Float32(data=self.actual_rpm))
        self.actual_speed_pub.publish(Float32(data=self.actual_speed))
        self.b9_valid_pub.publish(Bool(data=b9_valid))
        self.actual_steering_pub.publish(Float32(data=self.actual_steering))

        self.frame_count += 1
        if self.frame_count % 50 == 0:
            self.get_logger().info(
                f'📊 指令: RPM={rpm}, Steering={self.current_steering_deg:.1f}° | '
                f'实际: RPM={self.actual_rpm:.1f}, Speed={self.actual_speed:.2f}m/s'
            )

    def __del__(self):
        if self.device_ready and self.canDLL is not None:
            self.canDLL.VCI_ResetCAN(VCI_USBCAN2, DEVICE_INDEX, CAN1_INDEX)
            time.sleep(0.1)
            self.canDLL.VCI_CloseDevice(VCI_USBCAN2, DEVICE_INDEX)
            self.get_logger().info('✅ CAN 设备已关闭')

def main(args=None):
    rclpy.init(args=args)
    node = CANCommandReceiver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
