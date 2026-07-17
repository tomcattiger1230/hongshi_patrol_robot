# mobile_platform

Robot320 移动底盘的车载端 ROS 2 包。负责：

- 封装周立功 ControlCAN 驱动（`libcontrolcan.so`，多架构 vendor）
- 解析 Robot320 CAN 协议并维护底盘状态
- 复用 `robot320_interfaces` 中与 ROS 2 无关的命令、状态和 Fast DDS 契约
- 提供 4 个 `ros2 run` console script 和底盘 launch，方便车载端启动
- 集成 Livox MID-360s + Cartographer，并把 SLAM 位姿加入上位机遥测

## 1. 包信息

| 项 | 值 |
|---|---|
| ROS 构建类型 | `ament_python`（`format="3"`） |
| 入口模块 | `mobile_platform`（对应 `mobile_platform/mobile_platform/__init__.py`） |
| console scripts | `robot320_onboard` / `robot320_ros2_bridge` / `robot320_fastdds_bridge` / `robot320_cli` |
| launch | `robot320_ros2.launch.py`（SLAM 统一启动位于 `robot320_localization_bringup`） |
| 运行时依赖 | `robot320_interfaces` `rclpy` `std_msgs` `geometry_msgs` `launch` `launch_ros` `ament_index_python` |

## 2. 目录结构

```text
mobile_platform/
├── package.xml
├── setup.py
├── setup.cfg
├── MANIFEST.in
├── resource/mobile_platform           # ament index 标记
├── launch/
│   └── robot320_ros2.launch.py        # ros2 launch mobile_platform ...
├── vendor/controlcan/                 # libcontrolcan.so 多架构
│   ├── linux-x86_64/libcontrolcan.so
│   ├── linux-x86/libcontrolcan.so
│   ├── linux-aarch64/libcontrolcan.so
│   ├── linux-armv7/libcontrolcan.so
│   ├── include/controlcan.h
│   └── README.md
├── README.md                          # 本文档
└── mobile_platform/                   # Python 子包（import 路径）
    ├── __init__.py
    ├── can_types.py                   # libcontrolcan.so ctypes 结构
    ├── controlcan.py                  # ControlCAN 驱动封装
    ├── protocol.py                    # Robot320 CAN 协议
    ├── robot320.py                    # 高层底盘 API
    ├── messages.py                    # ChassisCommand / RobotTelemetry
    ├── transport.py                   # 通讯抽象 + UDP JSON 调试
    ├── safety.py                      # 超时、限速、急停
    ├── onboard_node.py                # 车载 UDP JSON 入口
    ├── ros2_node.py                   # ROS 2 车载节点
    ├── fastdds_node.py                # FastDDS 入口骨架
    └── cli.py                         # 命令行调试入口
```

`import mobile_platform` 的路径不变；源码现在位于 `mobile_platform/mobile_platform/` 子目录，这是 ament_python 推荐的标准嵌套布局（参见 `BUILD_HISTORY.md`）。

## 3. 构建

```bash
cd /path/to/hongshi_patrol_ws
source /opt/ros/jazzy/setup.bash
./build.sh --packages-up-to mobile_platform
source install/setup.bash
```

## 4. 运行

### 4.1 ROS 2 车载节点（推荐）

```bash
ros2 launch mobile_platform robot320_ros2.launch.py \
    topic_prefix:=/robot320 \
    command_timeout:=0.6 \
    max_linear_speed:=0.8 \
    max_angular_speed:=1.2 \
    rpm_per_mps:=500 \
    steering_gain:=180
```

可用参数（`launch` 文件中的 `DeclareLaunchArgument`）：

| 参数 | 默认值 | 含义 |
|---|---|---|
| `lib` | （空） | 显式指定 `libcontrolcan.so` 绝对路径，覆盖自动探测 |
| `device_index` | `0` | ControlCAN 设备索引 |
| `can_index` | `0` | CAN 通道索引 |
| `topic_prefix` | `/robot320` | ROS 2 topic 命名空间前缀 |
| `telemetry_period` | `0.2` | 遥测发布周期（秒） |
| `command_timeout` | `0.6` | 指令超时停车阈值（秒） |
| `max_linear_speed` | `0.8` | 线速度限幅（m/s） |
| `max_angular_speed` | `1.2` | 角速度限幅（rad/s） |
| `rpm_per_mps` | `500` | m/s → RPM 比例 |
| `steering_gain` | `180` | 角速度 → 转向角增益（deg / rad/s） |

也可以绕过 launch 直接用 Python 模块：

```bash
python3 -m mobile_platform.ros2_node \
    --topic-prefix /robot320 \
    --command-timeout 0.6 \
    --max-linear-speed 0.8 \
    --max-angular-speed 1.2 \
    --rpm-per-mps 500 \
    --steering-gain 180
```

### 4.2 UDP JSON 调试入口（无 ROS 2 环境）

```bash
robot320_onboard --command-bind 0.0.0.0:15000 --telemetry-remote 192.168.1.20:15001
```

这条入口只需要 `mobile_platform` 自身，不要求 ROS 2，便于在没有 DDS 的笔记本上做联调。

### 4.2.1 MID-360s SLAM 定位（NUC）

```bash
ros2 launch robot320_localization_bringup robot320_slam.launch.py \
    mode:=localization \
    map_state_file:=/var/lib/robot320/maps/site.pbstream \
    host_ip:=192.168.1.50 \
    lidar_ip:=192.168.1.107
```

统一 launch 会同时启动底盘、Livox 驱动、点云预处理、静态 TF、Cartographer 和
栅格地图发布。建图、地图保存、雷达外参和依赖安装见
[`robot320_localization_bringup/README.md`](../robot320_localization_bringup/README.md)。

### 4.3 CAN 命令行调试

```bash
robot320_cli watch --seconds 30
robot320_cli --lib /path/to/libcontrolcan.so watch --seconds 30
```

### 4.4 Fast DDS 车载入口

```bash
robot320_fastdds_bridge --device-index 0 --can-index 0 --domain-id 20
```

该入口订阅 `robot320/command`，经过序列号、时间戳和 `SafetyController` 校验后控制
CAN 底盘，并发布 `robot320/state`、`robot320/reply` 和 `robot320/heartbeat`。手动运动
必须持续发送，默认超过 `0.6s` 没有新运动指令停车。

IDL、Python 类型生成及运行环境见
[`robot320_interfaces/README.md`](../robot320_interfaces/README.md)。导航目标需要 ROS 2
导航网关；未配置升降杆硬件适配器时，升降指令会收到明确的 rejected 应答。

## 5. ROS 2 Topic 约定

默认 topic 前缀 `/robot320`：

| Topic | 类型 | 方向 | 用途 |
|---|---|---|---|
| `/robot320/cmd_vel` | `geometry_msgs/Twist` | 订阅 | `linear.x` 线速度，`angular.z` 角速度 |
| `/robot320/brake` | `std_msgs/Bool` | 订阅 | 刹车 |
| `/robot320/emergency_stop` | `std_msgs/Bool` | 订阅 | 急停 |
| `/robot320/mode` | `std_msgs/String` | 订阅 | `idle` / `manual` / `navigation` |
| `/robot320/telemetry` | `std_msgs/String` | 发布 | JSON `RobotTelemetry` |
| `/robot320/chassis_status` | `std_msgs/String` | 发布 | JSON `ChassisStatus` |
| `/robot320/speed_kmh` | `std_msgs/Float32` | 发布 | 底盘速度 km/h |

`/tracked_pose`（`geometry_msgs/PoseStamped`）由 Cartographer 发布，车载节点订阅后
写入 `/robot320/telemetry` 的 `pose` 字段；定位数据超过 1 秒未更新时不再回传旧位姿。

可用 `--topic-prefix` / `topic_prefix:=` 修改命名空间，例如 `/robot320/front`。

## 6. 安全门控

`mobile_platform/safety.py` 的 `SafetyController` 当前提供：

- 指令超时停车，默认 `0.6s`（`--command-timeout`）
- 线速度限幅，默认 `0.8 m/s`
- 角速度限幅，默认 `1.2 rad/s`
- 急停保持，收到 `mode=idle` 后解除

真实联调前建议进一步增加：

- 硬件急停输入
- 底盘故障码解析
- 心跳 topic
- 导航 / 手动模式互锁
- 低电量 / 定位丢失时降级停车

## 7. ControlCAN vendor

`mobile_platform/vendor/controlcan/` 已整理出 Linux 常用架构的 `libcontrolcan.so`，车载 Linux 端会按 CPU 架构自动选择：

- `linux-x86_64`
- `linux-x86`
- `linux-aarch64`
- `linux-armv7`

也可以通过以下方式覆盖：

- CLI：`--lib /path/to/libcontrolcan.so`
- ROS 2 launch：`lib:=/path/to/libcontrolcan.so`
- 环境变量：`CONTROL_CAN_LIB=/path/to/libcontrolcan.so`

厂商资料包可放在 `mobile_platform/driver/`，该目录只用于本地解包和查阅，不纳入 git。

## 8. CAN 协议速查

| 功能 | CAN ID | 帧类型 | 数据 |
|---|---:|---|---|
| 电机使能 | `0x03011008` | 扩展帧 | `0A 00` |
| 电机关闭/停车 | `0x03011008` | 扩展帧 | `01 00` |
| 转速 | `0x030110BA` | 扩展帧 | 有符号 int16，小端 |
| 速度请求 | `0x020101B9` | 扩展帧 | `00 00` |
| 刹车/释放 | `0x000007B9` | 标准帧 | `06 00 xx xx 00 00 00 00` |
| 转向 | `0x00000169` | 标准帧 | `02 xx xx 00 00 00 00 00` |

速度反馈兼容 `0x000110B9` 与 `0x020101B9` 扩展帧，按原程序逻辑从 `data[2:4]` 解析，单位为 `raw / 100 km/h`。

## 9. Python API 速查

```python
from mobile_platform import Robot320Platform
from mobile_platform.protocol import Direction

robot = Robot320Platform()
try:
    robot.connect()
    robot.set_motor_speed(Direction.FORWARD, 200)
    robot.turn(330, Direction.LEFT)
    state = robot.snapshot()
    print(state.speed_kmh)
finally:
    robot.disconnect()
```

## 10. 手工测试

```bash
source /opt/ros/<distro>/setup.bash
export ROS_DOMAIN_ID=20
ros2 launch mobile_platform robot320_ros2.launch.py
```

另一个终端：

```bash
source /opt/ros/<distro>/setup.bash
export ROS_DOMAIN_ID=20

# 前进 0.2 m/s
ros2 topic pub --once /robot320/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2}, angular: {z: 0.0}}"

# 左转
ros2 topic pub --once /robot320/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1}, angular: {z: 0.4}}"

# 停车 / 刹车 / 急停 / 解除急停
ros2 topic pub --once /robot320/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}"
ros2 topic pub --once /robot320/brake std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /robot320/emergency_stop std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /robot320/mode std_msgs/msg/String "{data: idle}"

# 查看遥测
ros2 topic echo /robot320/telemetry
ros2 topic echo /robot320/chassis_status
ros2 topic echo /robot320/speed_kmh
```

## 11. 联机排查

```bash
ros2 node list
ros2 node info /robot320_onboard
ros2 topic list
ros2 topic info /robot320/cmd_vel
ros2 topic hz /robot320/telemetry
```

如果上位机看不到机器人 topic，优先检查：

- 两端是否在同一网络
- 两端 `ROS_DOMAIN_ID` 是否一致
- 防火墙是否拦截 DDS 发现和 UDP 通讯
- 是否有多网卡导致 DDS 选错网卡
- 容器环境是否启用了 host network

## 12. 设备权限

```bash
lsusb                                  # 应看到 Microchip Technology, Inc. (04d8:0053)
sudo ros2 launch mobile_platform ...   # 临时 sudo
# 或写 udev 规则（参考厂商文档）
```

## 13. 与 `remote_control` 配合

上位机侧的发布/订阅与本包对称，详见 `src/hongshi_agent/remote_control/README.md`。
