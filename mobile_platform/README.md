# Robot320 NUC 车载端

`mobile_platform` 在 Ubuntu NUC 上负责 ControlCAN 底盘、安全门控和 ROS 2 通信网关。
正式部署时由 `robot320_localization_bringup` 统一启动。

```text
ROS 2 / DDS String command
      |
robot320_ros_gateway (rclpy)
      | ROS 2
Nav2 / Cartographer / lift adapter
      |
robot320_ros2_bridge -> ControlCAN -> chassis
      |
ROS 2 String state/reply/heartbeat
```

## 1. 构建和启动

```bash
./scripts/uv_setup.sh nuc
./scripts/uv_run.sh nuc ./build.sh
```

NUC 网关只使用 `rclpy` 和 `std_msgs/String`，DDS 由 ROS 2 RMW 负责，不安装或导入
Fast-DDS-python，也不生成 `Robot320String`。

定位模式：

```bash
./scripts/uv_run.sh nuc ros2 launch \
  robot320_localization_bringup robot320_slam.launch.py \
  mode:=localization \
  map_state_file:=/var/lib/robot320/maps/site.pbstream
```

该 launch 默认同时启动 CAN bridge 和 ROS 2 communication gateway。不要再启动
`robot320_fastdds_bridge`，否则两个进程会争用 CAN 设备。后者只保留作无 ROS 2 的底盘
调试入口，不是正式部署组成。

## 2. 外部指令到 ROS 2 的映射

| JSON 指令 | NUC 行为 |
|---|---|
| `manual_motion` | 发布 `/robot320/cmd_vel`，取消正在执行的导航 |
| `stop` / `brake` | 立即停止或刹车 |
| `emergency_stop` | 取消导航并触发底盘急停 |
| `reset_emergency_stop` | 进入 `idle` 并解除软件急停保持 |
| `navigation_goal` | 调用 `/navigate_to_pose` |
| `cancel_navigation` | 取消当前 Nav2 goal |
| `lift` | 发布 `/robot320/lift/command` JSON |

网关把 `/robot320/telemetry`、导航状态和 `/robot320/lift/status` 合并为
`RobotTelemetry`，发布到 `/robot320/state`。Ubuntu 上位机可直接加入 ROS domain；
Windows/macOS 则由 standalone Fast DDS 访问其底层 `rt/robot320/state` DDS Topic。

## 3. ROS 2 内部接口

| Topic / Action | 类型 | 方向 |
|---|---|---|
| `/robot320/cmd_vel` | `geometry_msgs/Twist` | 到底盘 |
| `/robot320/brake` | `std_msgs/Bool` | 到底盘 |
| `/robot320/emergency_stop` | `std_msgs/Bool` | 到底盘 |
| `/robot320/mode` | `std_msgs/String` | 到底盘 |
| `/robot320/telemetry` | `std_msgs/String` JSON | 底盘状态 |
| `/tracked_pose` | `geometry_msgs/PoseStamped` | Cartographer 位姿 |
| `/navigate_to_pose` | `nav2_msgs/NavigateToPose` | 导航 action |
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2 输出，网关转发到底盘 |
| `/robot320/lift/command` | `std_msgs/String` JSON | 到升降杆驱动 |
| `/robot320/lift/status` | `std_msgs/String` JSON | 来自升降杆驱动 |

导航目标只有在 Nav2 action server 已启动且现场参数正确时才能执行；否则通信网关
reply 会明确返回 `rejected`。

## 4. 安全行为

- 指令线速度默认限制为 `0.8 m/s`
- 角速度默认限制为 `1.2 rad/s`
- 手动运动超过 `0.6 s` 未续发时停车
- 手动接管、停止、刹车和急停会关闭 Nav2 速度转发
- 急停保持，直到收到 `reset_emergency_stop`

软件门控不能替代物理急停、驱动器保护和现场避障。真机运行前必须验证 CAN 故障、
定位丢失、低电量和网络中断时的停车行为。

## 5. ControlCAN

驱动按 `platform.machine()` 从 `vendor/controlcan` 自动选择 Linux runtime：

- `linux-x86_64`
- `linux-x86`
- `linux-aarch64`
- `linux-armv7`

可用 launch 参数 `lib:=/absolute/path/libcontrolcan.so` 或环境变量
`CONTROL_CAN_LIB` 覆盖。厂商文件说明见
[`vendor/controlcan/README.md`](./vendor/controlcan/README.md)。

## 6. 排查

| 现象 | 检查项 |
|---|---|
| CAN 未连接 | USB 权限、设备索引、CAN 通道、runtime 架构 |
| 有定位无导航 | Nav2 action server、TF `map -> base_link`、costmap 参数 |
| NUC 有状态而上位机收不到 | ROS domain、网卡、防火墙；非 ROS 上位机再检查 String TypeSupport |
| 升降杆始终 unavailable | 现场驱动是否实现 command/status 两个 topic |
| NUC 网关无法启动 | 是否 source ROS 2，`rclpy` 和 `std_msgs` 是否可导入 |

共享消息和 Fast DDS topic 见
[`robot320_interfaces/README.md`](../robot320_interfaces/README.md)，雷达定位见
[`robot320_localization_bringup/README.md`](../robot320_localization_bringup/README.md)。
