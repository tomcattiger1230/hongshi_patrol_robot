# Hongshi Patrol Robot

Robot320 巡检机器人项目。当前主要仿真平台运行 ROS 2 Jazzy，同时保留 ROS 2 Lyrical
兼容配置；底盘驱动、MID-360s 定位并对接 Nav2，同网段上位机运行 PySide6 GUI。Ubuntu 上位机自动使用 ROS 2，
Windows/macOS 使用 standalone Fast DDS，两者共享 ROS 2 `std_msgs/String` JSON 协议。

```text
Windows/macOS GUI -> Fast DDS ---+
                                 +---- ROS 2 String topics <---- Ubuntu NUC
Ubuntu GUI -------> rclpy -------+                              ROS/Nav2/CAN
                                             |
                         ROS 2 / Nav2 / Cartographer / CAN / lift adapter
```

## 仓库组成

| 目录 | 用途 |
|---|---|
| `robot320_interfaces` | 公共 JSON 消息和 ROS 2 兼容 Fast DDS 类型 |
| `remote_control` | 自动选择 ROS 2/Fast DDS 的 PySide6 GUI |
| `mobile_platform` | NUC 的 CAN、ROS 2 和通信网关 |
| `livox_ros_driver2` | 项目内使用的 Livox MID-360s 驱动 |
| `mid360_preprocess` | 点云高度裁剪和体素降采样 |
| `robot320_localization_bringup` | 底盘、雷达、Cartographer 的统一 launch |
| `patrol_robot_description` | roboQ-320 三维模型、Gazebo/Isaac Sim 自行车模型仿真 |

Python 环境由根目录 `pyproject.toml`、`uv.lock` 和 `scripts/uv_*.sh` 管理；ROS 2 C++
包仍由 colcon 构建。

当前 `192.168.3.117`、ROS 2 Jazzy 平台的 Gazebo/Isaac Sim 建图、地图保存、GUI
导航和故障排查命令统一见
[`SIMULATION_SLAM_NAVIGATION_DEBUG_GUIDE.md`](./SIMULATION_SLAM_NAVIGATION_DEBUG_GUIDE.md)。

## 上位机快速开始

Ubuntu 上位机默认和 NUC 一样已安装 ROS 2。安装
[uv](https://docs.astral.sh/uv/getting-started/installation/) 后直接运行：

```bash
./scripts/uv_setup.sh desktop
./scripts/uv_run.sh desktop robot320_remote_gui --domain-id 20
```

Linux 上的 `desktop` profile 固定使用 `/usr/bin/python3` 和 system site packages，从而
读取 apt 安装的 `rclpy`、`std_msgs` 等 ROS 2 模块。

GUI 的 `--backend auto` 默认优先选择 ROS 2。只有 Windows、macOS 等非 Ubuntu 上位机
需要额外安装 Fast DDS Python binding 和生成 ROS 2 String TypeSupport。具体步骤见
[`remote_control/README.md`](./remote_control/README.md)。

非 Ubuntu 系统上的 Fast DDS Python 是 native 扩展。uv 环境、`fastdds` Python
binding 和生成的 `Robot320String` 必须使用相同操作系统、CPU 架构和 Python ABI。

## NUC 快速开始

NUC 的系统镜像默认已安装 ROS 2。通信网关使用 `rclpy`，由 ROS 2 RMW 处理 DDS，
不需要 Fast-DDS-python 或项目生成类型。首次构建：

```bash
rosdep install --from-paths . --ignore-src -r -y
./scripts/uv_setup.sh nuc
./scripts/uv_run.sh nuc ./build.sh
```

定位运行：

```bash
./scripts/uv_run.sh nuc ros2 launch \
  robot320_localization_bringup robot320_slam.launch.py \
  mode:=localization \
  map_state_file:=/var/lib/robot320/maps/site.pbstream \
  host_ip:=192.168.1.50 lidar_ip:=192.168.1.107
```

`nuc` profile 固定使用 `/usr/bin/python3` 并允许 system site packages，使 uv 环境能读取
apt 安装的 `rclpy`。`uv_run.sh nuc` 会加载 ROS 2 和仓库的 `install/setup.bash`。

## Gazebo 仿真

在 ROS 2 Lyrical + Gazebo 设备上启动仿真和项目专用 RViz：

```bash
source /opt/ros/lyrical/setup.bash
source install/setup.bash
ros2 launch patrol_robot_description patrol_robot_sim.launch.py \
  gui:=true rviz:=true
```

模型、手动 `/cmd_vel` 控制和 `/livox/lidar` 仿真点云说明见
[`patrol_robot_description/README.md`](./patrol_robot_description/README.md)。

NVIDIA Isaac Sim 6 后端：

```bash
./patrol_robot_description/scripts/run_isaac_sim.sh \
  --gui --cmd-vel-topic /cmd_vel_isaac
```

### Isaac Sim 6 完整启动步骤（192.168.3.117）

工作空间为 `/home/arnold/Develop/ROS_ws/hongshi_patrol_ws`，仓库位于其
`src/hongshi_patrol_robot`。首次拉取代码或修改 ROS 包后先编译：

```bash
cd ~/Develop/ROS_ws/hongshi_patrol_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

使用三个终端启动。终端 1 启动 Isaac Sim 场景、车辆和 RTX MID-360：

```bash
cd ~/Develop/ROS_ws/hongshi_patrol_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 仅通过 SSH 启动图形界面时需要这两项：
export DISPLAY=:1
export XAUTHORITY=/run/user/1000/gdm/Xauthority

src/hongshi_patrol_robot/patrol_robot_description/scripts/run_isaac_sim.sh \
  --gui --cmd-vel-topic /cmd_vel_isaac
```

如果需要先检查驱动关节而不推进物理仿真，增加 `--start-paused`。Isaac Sim
窗口会保持打开；点击工具栏 Play 后开始仿真，之后点击 Pause/Stop 也不会退出程序：

```bash
src/hongshi_patrol_robot/patrol_robot_description/scripts/run_isaac_sim.sh \
  --gui --start-paused --cmd-vel-topic /cmd_vel_isaac
```

看到 `PATROL_ISAAC_READY` 后，终端 2 启动持续建图和 Nav2：

```bash
cd ~/Develop/ROS_ws/hongshi_patrol_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 launch robot320_localization_bringup robot320_simulation.launch.py \
  gazebo:=false mode:=mapping \
  persistent_map:=$HOME/robot320_maps/patrol_isaac_clean \
  navigation:=true exploration:=false rviz:=true gui:=false
```

终端 3 从仓库根目录通过 uv 启动地图导航 GUI：

```bash
cd ~/Develop/ROS_ws/hongshi_patrol_ws/src/hongshi_patrol_robot
source /opt/ros/jazzy/setup.bash
source ~/Develop/ROS_ws/hongshi_patrol_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
./scripts/uv_run.sh desktop robot320_navigation_gui \
  --domain-id 0 --use-sim-time
```

不要直接用系统 Python 执行 GUI，否则可能提示缺少 `PySide6`。停止时依次在 GUI、
Nav2 和 Isaac Sim 三个终端按 `Ctrl+C`，并确认 `/cmd_vel_isaac` 已归零。

### 2026-07-28 Isaac Sim 开发记录

今天已经完成并验证：

- Isaac 车辆支持自行车模型 `/cmd_vel` 控制，零速停车保持稳定。
- 增加速度断流 watchdog；Nav2 被取消、退出或通信中断后，Isaac 会自动制动。
- 直控隔离测试通过：前轮回正后直行约 `0.181 m`，航向漂移约 `0.73°`。
- Hybrid-A*（配置 ID 为 `GridBased`）能够为前方目标生成有效路径。
- RTX MID-360 保持完整 360° 点云，实测约 `3.1 Hz`；雷达 prim 已从静态
  articulation 容器移动到动态 `base_footprint` 下。
- GUI 会拒绝空 frame、非有限目标坐标和未完成旧任务时的新目标，避免
  `bt_navigator` 因无效/重叠 goal 异常退出。
- 原地图已备份到
  `$HOME/robot320_maps/backups/20260728_163708_before_moving_lidar`。

尚未通过、下次继续：

- 原 `$HOME/robot320_maps/patrol_current.posegraph` 是在雷达未随车运动时生成的，
  已包含错误约束，不应继续用于验证导航。
- 使用干净 posegraph 时，短距离 SLAM 与里程计可以一致；较长运动后
  `map→odom` 仍会出现明显平移和转角修正，导航会越过目标后倒车恢复。
- 当前验收结论是：底盘直控通过、规划通过、Isaac Sim 中的 SLAM 闭环导航未通过。
  下一步需要核对 RTX 点云输出参考系、雷达 prim 的局部/世界变换，以及
  `lidar_link` 静态 TF 三者是否逐帧一致，再调整 Nav2 控制参数。

带障碍物的 Gazebo SLAM/导航仿真：

```bash
cd ~/Develop/ROS2_ws/patrol_ws
source src/hongshi_patrol_robot/scripts/source_lyrical_sim.sh

ros2 launch robot320_localization_bringup robot320_simulation.launch.py \
  mode:=continuing navigation:=true exploration:=true rviz:=true gui:=true
```

`continuing` 是默认模式：从 `$HOME/robot320_maps/patrol_current` 加载完整 SLAM pose
graph，持续扩建 `/map`，并每 30 秒自动更新 pose graph、YAML 和 PGM。首次找不到
pose graph 时会创建新图；传统 `mode:=localization` 只加载静态地图。

通过地图鼠标点击发布 Nav2 目标：

```bash
./scripts/uv_run.sh desktop robot320_navigation_gui \
  --domain-id 20 --use-sim-time
```

具体操作见 [`remote_control/README.md`](remote_control/README.md)。

## 测试

```bash
./scripts/uv_setup.sh desktop --dev
./scripts/uv_run.sh desktop --dev pytest -q
./scripts/uv_run.sh desktop --dev ruff check \
  robot320_interfaces mobile_platform remote_control
```

## 详细文档

- [Gazebo/Isaac Sim SLAM、导航与 GUI 调试手册](./SIMULATION_SLAM_NAVIGATION_DEBUG_GUIDE.md)
- [上位机与 Windows/macOS Fast DDS 安装](./remote_control/README.md)
- [NUC 底盘和 ROS 2 通信网关](./mobile_platform/README.md)
- [MID-360s 建图与定位](./robot320_localization_bringup/README.md)
- [共享消息和 ROS 2 String 类型](./robot320_interfaces/README.md)
- [Livox 驱动集成](./livox_ros_driver2/README.md)
- [基础模型与 Gazebo 仿真](./patrol_robot_description/README.md)
