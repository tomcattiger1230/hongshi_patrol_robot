# Robot320 实车运行配置（2026-09-02）

本目录保存 2026-09-02 在 Robot320 onboard NUC 上完成实车联调后同步回仓库的运行文件。
远端环境为 Ubuntu 24.04 + ROS 2 Jazzy，NUC 地址为 `192.168.42.39`，MID-360 地址为
`192.168.1.107`，雷达网卡地址为 `192.168.1.50`。

> 安全提示：启动脚本只启动传感器、定位、Nav2 和 CAN 反馈，不会启动底盘控制发送节点。
> 启动 `cmd_vel_to_ackermann_can.py` 前，应确保车辆架空或周围有足够空间、急停可用，并有人
> 在现场观察。不要绕过 `/cmd_vel_safe` 直接把 Nav2 输出接到底盘。

## 今日验证结果

| 项目 | 结果 | 说明 |
|---|---|---|
| MID-360 点云 | 通过 | 车体包络内点在过滤后为 0；最近外部点约 1.946 m |
| 定位 TF | 通过 | `map -> odom -> base_link -> livox_frame` 连续可用 |
| B9 速度反馈 | 通过 | 约 10 Hz，支持正负方向和 0.5 s 有效性超时 |
| wheel odom + EKF | 通过 | B9 提供 `vx`，IMU 提供 `angular_velocity.z` |
| Nav2 生命周期 | 通过 | 冷启动后各 server active，安全速度链路可输出 |
| 前进 | 通过 | 75 RPM（约 0.15 m/s）可靠起步；B9 峰值约 0.1667 m/s |
| 倒车 | 通过 | -75 RPM 时 B9 和 wheel odom 均为负值 |
| 静态转向方向 | 通过 | `+250°` 为物理左转，`-250°` 为物理右转，`0°` 回正 |
| 实际转角反馈 | 未通过 | `/can/actual_steering` 仍为 0，不能作为测量值使用 |
| 完整自主导航目标 | 待验证 | 尚未执行带转弯的低速闭环 Nav2 目标 |

测试结束时速度命令和转向命令均已归零；底盘控制发送节点已停止。

## 数据与控制链路

```text
MID-360 -> /livox/lidar -> mid360_preprocess -> /filtered_points -> Cartographer

B9 query/response -> /can/actual_speed
                  -> can_to_odom -> /wheel/odom_raw(vx)
IMU -> imu_covariance_filter -> gyro_z
wheel odom + IMU -> EKF -> /odometry/filtered + odom->base_link
Cartographer -> map->odom

Nav2 /cmd_vel_nav
  -> velocity_smoother /cmd_vel_smoothed
  -> collision_monitor /cmd_vel_safe
  -> cmd_vel_to_ackermann_can.py
  -> /can/speed_cmd + /can/steering_cmd
  -> can_command_receiver.py -> CAN
```

## 文件对应关系

| 仓库文件 | onboard NUC 目标位置 |
|---|---|
| `can_command_receiver.py` | `/home/hs/roboracer_ws/src/robot320_bringup/scripts/` |
| `can_to_odom.py` | `/home/hs/roboracer_ws/src/robot320_bringup/scripts/` |
| `cmd_vel_to_ackermann_can.py` | `/home/hs/roboracer_ws/src/robot320_bringup/scripts/` |
| `imu_covariance_filter.py` | `/home/hs/roboracer_ws/src/robot320_bringup/scripts/` |
| `config/nav2_params_robot320.yaml` | `/home/hs/roboracer_ws/config/nav2/` |
| `config/robot_localization_params.yaml` | `/home/hs/robot_localization_params.yaml` |
| `config/mid360_2d.lua` | `/home/hs/cartographer_config/mid360_2d.lua` |
| `start_robot320_full_ekf.sh` | `/home/hs/script/` |
| `stop_robot320_full.sh` | `/home/hs/script/` |
| `lift_serial_bridge.py` | `/home/hs/robot320_lift/` |
| `robot320-lift-serial.service` | `/home/hs/.config/systemd/user/` |
| `network_failover.py` | `/usr/local/lib/robot320/` |
| `sim_policy_route.sh` | `/usr/local/sbin/robot320-sim-policy-route` |
| `robot320-network-failover.service` | `/etc/systemd/system/` |
| `robot320-sim-policy-route.service` | `/etc/systemd/system/` |

点云源码没有在本目录保留重复副本。远端新增的车体过滤逻辑已合并到仓库的
`mid360_preprocess/src/mid360_preprocess_node.cpp`，同时保留了本地已有的话题、输出坐标系、
高度和体素参数。实车默认参数为：

```text
lidar_x=0.365
self_min_x=-0.82  self_max_x=0.82
self_min_y=-0.485 self_max_y=0.485
self_filter_enabled=true
```

远端修改前的备份位于
`/home/hs/robot320_backups/codex_remote_nav_20260902_104234`。

## 启停与检查

在 NUC 图形桌面终端中启动定位导航栈：

```bash
bash /home/hs/script/start_robot320_full_ekf.sh
```

启动脚本会显式调用 `/collision_monitor/toggle`。只有返回 `success=True` 才算安全速度链路
启动成功；若调用失败，脚本退出且不应启动控制发送节点。

推荐先做只读检查：

```bash
source /opt/ros/jazzy/setup.bash
source /home/hs/roboracer_ws/install/setup.bash
ros2 topic hz /can/actual_speed
ros2 topic echo /can/actual_speed_b9_valid --once
ros2 topic echo /odometry/filtered --once
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
```

确认现场安全后，控制发送节点需单独启动：

```bash
source /opt/ros/jazzy/setup.bash
source /home/hs/roboracer_ws/install/setup.bash
python3 /home/hs/roboracer_ws/src/robot320_bringup/scripts/cmd_vel_to_ackermann_can.py
```

停止整个栈：

```bash
bash /home/hs/script/stop_robot320_full.sh
```

## 已确认的协议与标定

- B9 查询帧：扩展帧 `0x020110B9`，数据 `00 00`，约 10 Hz。
- B9 响应：仅接受扩展帧 `0x000110B9` 且 `DLC=2`。
- 速度换算：两字节有符号小端整数，`speed_mps = raw * 0.01 / 3.6`。
- 超过 0.5 s 未收到 B9 响应时，速度归零且 `/can/actual_speed_b9_valid=false`。
- 刹车释放：标准帧 ID `0x7B9`，数据 `06 00 00 00 00 00 00 00`。
- 速度比例：`500 RPM/(m/s)`；最大软件速度为 0.3 m/s。
- 轴距 0.89 m，最小转弯半径 2.35 m，车轮最大转角 20.75° 对应执行器 350°。
- ROS 正 `angular.z` 对应正执行器角度，即车辆左转；负值对应右转。
- CAN 总线约 800 frame/s，接收端每个 50 Hz 周期最多清空 100 帧，避免 B9 积压。
- 控制看门狗为 0.6 s；关键使能、松刹和失能帧检查返回值并最多重试 3 次。

`0x6FA` 仅保留诊断用途，不能再解释为可靠车速。当前没有验证通过的真实转角反馈，
因此 wheel odom 中只有纵向速度 `vx` 可视为实测量；转向角、wheel yaw 和转弯半径不能当作
反馈结果。

## 升降平台 RS-485

升降平台通过 FTDI `/dev/ttyUSB0`、9600 8N1 控制。现场确认有效的是单机地址 `01`：

| GUI 动作 | 串口帧 |
|---|---|
| 上升 | `AA 01 03 00` |
| 下降 | `AA 01 0C 00` |
| 停止 | `AA 01 0A 00` |

`robot320-lift-serial.service` 常驻打开串口并订阅 `/robot320/lift/command`。每收到一条
`raise`、`lower` 或 `stop` 消息只写一帧，不合并重复命令，也不自动定时停止。GUI 每次
点击都会生成一个新命令，因此链路不稳定时可以重复点击。上升或下降后平台会持续运动，
必须由用户点击停止或由机械限位终止；服务退出时会额外发送一次安全停止。

异网 GUI 不依赖 DDS 发现：服务同时创建权限 `0600` 的
`/run/user/1000/robot320-lift.sock`，`lift_control.py --stream` 只接受
`raise/lower/stop` 三个动作。Mac GUI 通过 NUC→云反向 SSH 的持久控制流调用该入口，
保持点击顺序并避免每次重新登录的延迟。返回 `ok` 只代表指令写入 FTDI 串口成功，不代表
平台已经产生机械动作；当前设备没有接入位置或限位反馈。

```bash
systemctl --user status robot320-lift-serial.service
journalctl --user -u robot320-lift-serial.service -f
```

## 有人 G809 有线 / SIM 网络

NUC 有线口 `enp85s0` 使用静态地址 `192.168.1.50/24`，有人 USR-G809 的 LAN 地址和
默认网关为 `192.168.1.1`。持久配置对应仓库中的 `config/99-static-ip.yaml`，部署到
`/etc/netplan/99-static-ip.yaml`：

- Wi-Fi 健康时使用 metric 100，作为首选公网出口；
- 有线 / SIM 默认路由固定为 metric 600，作为热备用；
- Wi-Fi 连续 3 次探测失败后改为 metric 900，使有线 / SIM 接管；连续 3 次恢复后自动
  回到 metric 100；探测周期为 5 秒；
- 有线 DNS 首选 G809 `192.168.1.1`，备用 `223.5.5.5`。

`robot320-sim-policy-route.service` 为源地址 `192.168.1.50` 安装独立路由表 102。这样即使
系统默认出口仍是 Wi-Fi，绑定 SIM 地址的备用反向隧道也能同时在线。两个出站隧道分别在
云主机 loopback 创建 `12222`（Wi-Fi）和 `12226`（SIM）；云端选择器监听 `12220`，先
验证并选用 Wi-Fi 的真实 SSH banner，Wi-Fi 不可用时自动回退到 SIM。三个端口均不对公网
监听。

2026-09-23 已完成完整断网验证：临时断开 `HSJC` 后，默认路由自动切到
`192.168.1.50 -> 192.168.1.1`；Mac 经云选择器发送升降“停止”、两台工业相机单帧抓图
均成功，两路监控视频也从 SIM 地址重新发布到云端。Wi-Fi 恢复并连续通过 3 次探测后，
默认路由和两路视频自动回到 Wi-Fi；SIM 隧道保持在线待命。

网络出口切换会使绑定旧源地址的长连接失效。故障切换服务会自动重启两路 FFmpeg 推流；
两个独立反向 SSH 隧道不依赖切换时临时重建。检查命令：

```bash
systemctl --user restart hikvision-video-relay-second.service
sudo systemctl restart hikvision-video-relay@hs.service
ss -ntp | grep '8.163.54.201:8554'
```

正常结果应为两条从 `192.168.1.50` 到 `8.163.54.201:8554` 的 `ESTAB` 连接。
Wi-Fi 健康时对应源地址为当前 `wlo1` DHCP 地址。

## 下一步

1. 在现场人员和急停保护下，先下发转向角并保持，确认到位后再给 0.10–0.15 m/s 的短时
   速度，完成左右低速圆弧测试。
2. 比较 B9、wheel odom、EKF、Cartographer 四条轨迹的方向和尺度。
3. 查明真实转角反馈 CAN 字段，替换当前恒为 0 的 `/can/actual_steering`。
4. 圆弧验证通过后，再执行距离短、速度受限且路线无遮挡的 Nav2 单点目标。
