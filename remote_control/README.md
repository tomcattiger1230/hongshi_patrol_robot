# remote_control

Robot320 移动底盘的上位机 ROS 2 控制包。它不直接发送 CAN 帧，只发送语义控制命令，并接收机器人遥测。

## 1. 包信息

| 项 | 值 |
|---|---|
| ROS 构建类型 | `ament_python`（`format="3"`） |
| 入口模块 | `remote_control`（对应 `remote_control/remote_control/__init__.py`） |
| console scripts | `robot320_remote_cli` / `robot320_remote_ros2` / `robot320_remote_fastdds` / `robot320_remote_gui` |
| launch | `robot320_remote_watch.launch.py` |
| 核心依赖 | `robot320_interfaces`；Fast DDS 模式不依赖 ROS 2 |
| 可选 ROS 2 依赖 | `rclpy` / `geometry_msgs` / `std_msgs` / `launch` |

## 2. 目录结构

```text
remote_control/
├── package.xml
├── setup.py
├── setup.cfg
├── MANIFEST.in
├── resource/remote_control           # ament index 标记
├── launch/
│   └── robot320_remote_watch.launch.py  # ros2 launch remote_control ...
├── README.md
└── remote_control/                   # Python 子包（import 路径）
    ├── __init__.py
    ├── cli.py                        # UDP JSON 远程 CLI
    ├── dds_client.py                 # UDP JSON 调试客户端
    ├── ros2_client.py                # ROS 2 上位机入口
    ├── fastdds_client.py             # ROS-independent Fast DDS client/API
    └── gui.py                        # Tkinter GUI 原型
```

`import remote_control` 路径不变；源码现在位于 `remote_control/remote_control/` 子目录，这是 ament_python 标准嵌套布局。

## 3. 构建

```bash
cd /path/to/hongshi_patrol_ws
source /opt/ros/jazzy/setup.bash
./build.sh --packages-up-to remote_control
source install/setup.bash
```

## 4. 运行

### 4.1 ROS 2 上位机（Ubuntu 首选）

```bash
# 一键遥测监听（默认 1 小时，seconds:=30 控制时长）
ros2 launch remote_control robot320_remote_watch.launch.py seconds:=30
```

可用参数：

| 参数 | 默认值 | 含义 |
|---|---|---|
| `topic_prefix` | `/robot320` | 订阅 `/robot320/telemetry` 等 topic 时使用 |
| `seconds` | `3600` | 监听时长（秒） |

```bash
# 发送控制
robot320_remote_ros2 move --linear 0.2 --angular 0.0 --duration 2
robot320_remote_ros2 stop
robot320_remote_ros2 brake
robot320_remote_ros2 estop
robot320_remote_ros2 reset               # mode=idle 解除急停
robot320_remote_ros2 watch --seconds 30
```

### 4.2 UDP JSON 远程 CLI

```bash
robot320_remote_cli --robot 192.168.1.10:15000 move --linear 0.2 --duration 2
robot320_remote_cli move --linear 0.2 --angular 0.4
robot320_remote_cli stop
robot320_remote_cli brake
robot320_remote_cli estop
robot320_remote_cli watch --seconds 30
```

适合没有 ROS 2 / DDS 的笔记本做本地联调。

### 4.3 Tkinter GUI 原型

```bash
robot320_remote_gui --robot 192.168.1.10:15000 --telemetry-bind 0.0.0.0:15001
```

### 4.4 Fast DDS 主通信入口（不需要 ROS 2）

```bash
robot320_remote_fastdds --domain-id 20 move --linear 0.2 --duration 2
robot320_remote_fastdds --domain-id 20 goal --x 3.0 --y 1.5 --yaw 0.0
robot320_remote_fastdds --domain-id 20 lift move_to --height 1.2
robot320_remote_fastdds --domain-id 20 estop
robot320_remote_fastdds --domain-id 20 watch --seconds 30
```

这是上位机正式通信入口，只依赖 `robot320_interfaces`、Fast DDS Python bindings 和
目标平台生成的 `Robot320Dds` 模块，不导入 `rclpy`。运行环境和 IDL 生成方式见
[`robot320_interfaces/README.md`](../robot320_interfaces/README.md)。

## 5. 启动顺序（ROS 2 联调）

1. 机器人 Ubuntu 端启动车载节点：

    ```bash
    source /opt/ros/<distro>/setup.bash
    export ROS_DOMAIN_ID=20
    ros2 launch mobile_platform robot320_ros2.launch.py
    ```

2. 上位机 Ubuntu 端启动遥测监听：

    ```bash
    source /opt/ros/<distro>/setup.bash
    export ROS_DOMAIN_ID=20
    ros2 launch remote_control robot320_remote_watch.launch.py seconds:=30
    ```

3. 上位机发送控制：

    ```bash
    robot320_remote_ros2 move --linear 0.2 --angular 0.0 --duration 2
    robot320_remote_ros2 stop
    ```

## 6. ROS 2 网络检查

```bash
ros2 node list
ros2 topic list
ros2 topic echo /robot320/telemetry
ros2 topic pub --once /robot320/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1}, angular: {z: 0.0}}"
```

上位机看不到 topic 时，优先检查：

- 两端 `ROS_DOMAIN_ID` 是否一致
- 网络是否互通
- 防火墙是否拦截 DDS 发现与 UDP 通讯
- 是否有多网卡导致 DDS 选错网卡

车载端 topic 完整列表见 [`../mobile_platform/README.md` §5](../mobile_platform/README.md#5-ros-2-topic-约定)。

## 7. 数据范围

- 控制：线速度、角速度、刹车、急停、导航 / 手动模式
- 状态：底盘连接、使能、刹车、速度、转速、转向、急停状态
- 定位：`Pose2D`（已由 NUC 端 `/tracked_pose` 写入遥测，CLI/GUI 可显示）
- 导航：目标点、导航状态、进度、提示信息
- 地图：先以 `map_revision` 占位，后续按 DDS topic 承载栅格地图或矢量地图

## 8. Fast DDS

Ubuntu、Windows 或 macOS 上位机均按目标系统构建 Fast DDS Python bindings 和 IDL 类型。
正式契约位于 `robot320_interfaces/robot320_interfaces/dds/Robot320Dds.idl`：

- `robot320/command`：目标点、手动运动、急停和升降指令
- `robot320/state`：底盘、位置、导航、升降杆、电池和故障状态
- `robot320/reply`：指令接受、完成、拒绝或失败应答
- `robot320/heartbeat`：双端在线心跳

## 9. Python API 速查

笔记本正式程序直接使用 Fast DDS 客户端类，不需要启动 ROS 2，也不必通过命令行：

```python
from remote_control.fastdds_client import RobotRemoteFastDDSClient

client = RobotRemoteFastDDSClient(domain_id=20, client_id="operator-laptop")
try:
    client.send_manual_command(linear_speed_mps=0.2, angular_speed_radps=0.0)
    client.send_navigation_goal(x_m=3.0, y_m=1.5, yaw_rad=0.0)
    state = client.receive_telemetry(timeout_s=1.0)
    reply = client.receive_reply(timeout_s=1.0)
    client.cancel_navigation()
    client.control_lift("move_to", target_height_m=1.2)
finally:
    client.close()
```

下面两个接口仅用于已有 ROS 2 或 UDP 调试场景：

```python
from remote_control.ros2_client import RobotRemoteRosNode
import rclpy
rclpy.init()
node = RobotRemoteRosNode(topic_prefix="/robot320")
node.send_manual_command(linear_speed_mps=0.2, angular_speed_radps=0.0)
node.stop()
node.destroy_node()
rclpy.shutdown()
```

```python
from remote_control.cli import main as remote_cli_main
remote_cli_main(["--robot", "192.168.1.10:15000", "move", "--linear", "0.2", "--duration", "2"])
```

## 10. 故障排查清单

| 现象 | 优先检查 |
|---|---|
| `ros2 run remote_control robot320_remote_ros2 --help` 找不到 | 是否 `source install/setup.bash`？是否成功 `colcon build`？ |
| launch 文件找不到 | 同上，且 `ros2 launch <pkg> <name>.launch.py` 中的 launch 文件名要与 `launch/` 下文件一致 |
| 上位机无 topic | 第四节启动顺序是否走通；`ROS_DOMAIN_ID` 是否一致；`ros2 topic list` 是否能看到车载端 topic |
| GUI 启动失败 | 系统中是否安装 `python3-tk` |
| Fast DDS 抛 `FastDDSUnavailable` | 未安装/source Fast-DDS-python，或未生成并加载 `Robot320Dds` 模块 |
