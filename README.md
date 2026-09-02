# Hongshi Patrol Robot

Robot320 巡检机器人项目。当前主要开发平台为 Ubuntu 26.04 + ROS 2 Lyrical，
Ubuntu 24.04 + ROS 2 Jazzy 保留为稳定基线和 Isaac Sim 6 Bridge 环境。项目包含自行车
模型底盘、MID-360s、SLAM Toolbox、Nav2 和 PySide6 地图导航 GUI。Ubuntu 上位机
直接使用 ROS 2，Windows/macOS 使用 standalone Fast DDS，两者共享 ROS 2
`std_msgs/String` JSON 协议。

## 2026-09-02 实车远程联调结果

已在 `192.168.42.39` 的 Robot320 onboard NUC 上完成定位导航栈冷启动、MID-360
车体自反射过滤、B9 有符号速度反馈、wheel odom、EKF、Cartographer、Nav2 安全速度链路
以及低速前进/倒车和静态左右转向测试。

- 定位链路 `map -> odom -> base_link -> livox_frame` 工作正常。
- B9 速度反馈约 10 Hz，并带 0.5 s 有效性超时；旧 `0x6FA` 只保留诊断用途。
- 75 RPM（约 0.15 m/s）可以可靠前进，-75 RPM 的反馈和里程计方向为负。
- 转向执行器正角为物理左转、负角为物理右转、0 为回正，与 ROS 正 `angular.z` 一致。
- Nav2 采用 `/cmd_vel_nav -> /cmd_vel_smoothed -> /cmd_vel_safe`，启动时必须显式启用
  collision monitor；控制发送节点只订阅安全输出。
- 真实转角反馈尚未确认，完整带转弯 Nav2 目标尚未执行，不能把 wheel yaw 当作实测结果。

本次从 NUC 同步的实车脚本、配置、部署位置、安全步骤和完整测试记录见
[`scripts/robot320_onboard/README.md`](./scripts/robot320_onboard/README.md)。

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

Gazebo/Isaac Sim 建图、地图保存、GUI 导航和故障排查命令统一见
[`SIMULATION_SLAM_NAVIGATION_DEBUG_GUIDE.md`](./SIMULATION_SLAM_NAVIGATION_DEBUG_GUIDE.md)。

## ROS 2 版本与兼容性

不要只根据 `package.xml` 判断兼容性。ROS 2 发行版同时决定 Ubuntu、Python、Nav2、
Gazebo 和 DDS 的版本；跨发行版复用二进制扩展或参数文件很容易出现“可以编译但运行
异常”的情况。

| 组合 | 项目状态 | 说明 |
|---|---|---|
| Ubuntu 26.04 Resolute + ROS 2 Lyrical + Gazebo Jetty | 当前主要平台，已验证 | 2026-08-28 在 `192.168.3.133` 验证 `gz-sim 10`、MID-360 点云、SLAM、Nav2 和 RViz |
| Ubuntu 24.04 Noble + ROS 2 Jazzy + Gazebo Harmonic | 稳定基线 | 适合已有 Jazzy 工作区和 Gazebo Harmonic；不要把 Jazzy 的 `build/`、`install/` 带到 Lyrical |
| Isaac Sim 6 + Jazzy Bridge + 外部 Lyrical 节点 | 实验性 | 标准 ROS 消息可以通过 DDS 互通，但 Python ABI、DDS TypeObject 和仿真时钟仍需逐项验证 |
| Jazzy 二进制包直接运行在 Ubuntu 26.04 | 不支持 | Jazzy 的 Tier 1 平台是 Ubuntu 24.04，不能通过复制 `/opt/ros/jazzy` 代替正确安装 |

ROS 2 官方目标平台见 [REP 2000](https://www.ros.org/reps/rep-2000.html)，ROS/Gazebo
推荐配对见 [Gazebo 官方兼容表](https://gazebosim.org/docs/latest/ros_installation/)。本项目
在 Lyrical 上使用其 vendor package 提供的 Gazebo Jetty；不要另外安装另一套 Gazebo
再让 `ros_gz_bridge` 链接不同主版本的库。

### Lyrical 平台的实际差异

- Ubuntu 26.04 的系统 Python 是 3.14。ROS 的 `rclpy`、Qt binding 和其他 native
  extension 必须使用匹配的 Python ABI。请通过 `scripts/uv_setup.sh` 和
  `scripts/uv_run.sh` 使用允许 system site packages 的 `desktop`/`nuc` profile；只在
  普通 uv 虚拟环境中执行可能出现 `No module named yaml`、`rclpy` 或 Qt 插件错误。
- `slam_toolbox`、`pointcloud_to_laserscan`、`ros_gz_sim`、`gz_sim_vendor` 和 Cyclone DDS
  已可从 Lyrical 系统包使用。当前 Nav2 使用仓库锁定的源码 overlay；每次新终端必须
  按“ROS underlay → Nav2 underlay（如有）→ 机器人工作区”的顺序 source。
- Kilted 及更新版本的 Nav2 默认使用 `geometry_msgs/msg/TwistStamped`。项目的 Gazebo
  Ackermann 插件和 Isaac 控制器仍接收 `geometry_msgs/msg/Twist`，因此
  `nav2_ackermann.yaml` 显式设置 `enable_stamped_cmd_vel: false`。不要让同一个
  `/cmd_vel` 同时出现 `Twist` 和 `TwistStamped`，可用
  `ros2 topic type /cmd_vel` 检查。
- 新版 Nav2 增加了 route、docking 等 lifecycle server，并有参数更名或废弃提示。
  从 Jazzy 复制参数后必须重新检查 `ros2 param dump` 和启动日志，不能默认旧参数仍被
  使用。
- 仿真图中只能存在一个 `/clock` 发布者。切换 Gazebo/Isaac Sim 前必须停止旧 simulator
  和 `ros_gz_bridge`；`ros2 topic info /clock --verbose` 的 `Publisher count` 必须为 1。
- 推荐所有外部 ROS 2 进程统一使用 `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`。不要在同一
  启动链中临时切换 RMW，也不要把不同发行版生成的 `build/`、`install/`、Python
  native module 或 TypeSupport 文件混用。

### Isaac Sim 6 的边界

Isaac Sim 6 内置 Python 3.12 和 Jazzy ROS Bridge，而 Lyrical 主机使用 Python 3.14。
不得把 `/opt/ros/lyrical` 的 `rclpy` 或其他 native library 加入 Isaac Kit 的
`PYTHONPATH`。Isaac 内只运行其自带 Bridge/仿真脚本，SLAM Toolbox、Nav2、RViz 和 GUI
运行在外部 ROS 环境。目前 Lyrical 与 Isaac Bridge 联调中仍观察到过 Cyclone DDS
TypeObject 错误，因此 Lyrical 上的完整闭环优先使用 Gazebo，Isaac Sim 暂按实验后端
处理。

Gazebo 与 Isaac Sim 的 pose graph 也不能混用。两者的雷达采样、出生位姿、障碍物和
坐标误差不同，混用旧图可能让 Nav2 报 `Start occupied` 或造成地图旋转。建议至少分开
保存：

```text
$HOME/robot320_maps/patrol_gazebo
$HOME/robot320_maps/patrol_isaac
```

升级或切换发行版后先彻底清理并重新构建：

```bash
cd ~/Develop/ROS_ws/hongshi_patrol_ws
rm -rf build log install   # 仅在确认当前目录是该 colcon 工作区后执行
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install
source install/setup.bash
```

如果使用 zsh，请对应 source `setup.zsh`。不要在已经 source Jazzy 的终端上继续 source
Lyrical；应打开全新终端。

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

当前 Lyrical 平台的推荐启动方式是从空白 Gazebo 专用地图开始，同时启动
SLAM Toolbox、Nav2、RViz 和 Gazebo GUI：

```bash
cd ~/Develop/ROS_ws/hongshi_patrol_ws
source /opt/ros/lyrical/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 launch robot320_localization_bringup robot320_simulation.launch.py \
  gazebo:=true mode:=mapping \
  persistent_map:=$HOME/robot320_maps/patrol_gazebo \
  navigation:=true exploration:=false rviz:=true gui:=true
```

确认新图有效并保存后，才切换为 `mode:=continuing`。不要用 Isaac Sim 生成的
`patrol_current.posegraph` 验证 Gazebo；2026-08-28 实测这种混用会让规划器报
`Start occupied`。完整测试中 `/livox/lidar`、`/filtered_points` 和 `/scan` 均约为
10 Hz，1 m 前向 Nav2 目标执行成功。

只检查模型、关节和手动 `/cmd_vel` 时可使用较小的 description launch：

```bash
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

### Isaac Sim 6 完整启动步骤（Lyrical 主机，实验性）

工作空间为 `/home/arnold/Develop/ROS_ws/hongshi_patrol_ws`，仓库位于其
`src/hongshi_patrol_robot`。首次拉取代码或修改 ROS 包后先编译：

```bash
cd ~/Develop/ROS_ws/hongshi_patrol_ws
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install
source install/setup.bash
```

使用三个终端启动。终端 1 启动 Isaac Sim 场景、车辆和 RTX MID-360：

```bash
cd ~/Develop/ROS_ws/hongshi_patrol_ws
source /opt/ros/lyrical/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# 通过 SSH 启动时，还需设置当前桌面会话的 DISPLAY、XAUTHORITY、
# XDG_RUNTIME_DIR 和 DBUS_SESSION_BUS_ADDRESS；见下方远程启动章节。

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
source /opt/ros/lyrical/setup.bash
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
source /opt/ros/lyrical/setup.bash
source ~/Develop/ROS_ws/hongshi_patrol_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
./scripts/uv_run.sh desktop robot320_navigation_gui \
  --domain-id 0 --use-sim-time
```

不要直接用系统 Python 执行 GUI，否则可能提示缺少 `PySide6`。停止时依次在 GUI、
Nav2 和 Isaac Sim 三个终端按 `Ctrl+C`，并确认 `/cmd_vel_isaac` 已归零。

#### 通过 SSH 远程启动 Isaac Sim、RViz2 和导航 GUI

下面是开发机 `192.168.3.133` 上使用过的后台启动方式。远程桌面会话必须已经登录；
SSH 本身不会创建可供 Qt/Isaac Sim 使用的图形会话。先在 SSH 终端中确认显示参数：

```bash
export XDG_RUNTIME_DIR=/run/user/$(id -u)
export DBUS_SESSION_BUS_ADDRESS=unix:path=$XDG_RUNTIME_DIR/bus
export DISPLAY=:0

# GNOME Wayland/Xwayland 通常使用这个文件；先确认只返回一个当前会话文件。
find "$XDG_RUNTIME_DIR" -maxdepth 1 -name '.mutter-Xwaylandauth.*' -print
export XAUTHORITY=/run/user/1000/.mutter-Xwaylandauth.TVEWU3

test -S "$XDG_RUNTIME_DIR/bus"
test -r "$XAUTHORITY"
```

`XAUTHORITY` 后缀会在重新登录后变化，不能永久照抄示例值。若机器使用 GDM/Xorg，
它也可能是 `/run/user/1000/gdm/Xauthority`。应以当前桌面会话的实际文件为准。

所有后台进程必须继承同一套 ROS 环境：

```bash
cd ~/Develop/ROS_ws/hongshi_patrol_ws
source /opt/ros/lyrical/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

开始前确认没有旧 Gazebo、Isaac Sim 或 ROS bridge。特别是 `/clock` 只能有一个发布者：

```bash
pgrep -af 'patrol_robot_isaac_sim.py|gz-sim-main|parameter_bridge|robot320_simulation.launch.py'
ros2 topic info /clock --verbose 2>/dev/null || true
```

以下命令均从同一个已经 source Lyrical 的 SSH shell 启动。Isaac Sim 6 与 Lyrical 的
组合目前仍按实验环境处理，因此继续使用已经验证过的 Cyclone DDS；Gazebo 上通过的
Fast DDS 结果不能自动等同于 Isaac Sim Bridge 也通过。

第一步，后台启动 Isaac Sim 6 场景。PID 和日志分别保存在 `/tmp`：

```bash
nohup env \
  DISPLAY="$DISPLAY" \
  XAUTHORITY="$XAUTHORITY" \
  XDG_RUNTIME_DIR="$XDG_RUNTIME_DIR" \
  DBUS_SESSION_BUS_ADDRESS="$DBUS_SESSION_BUS_ADDRESS" \
  ROS_DOMAIN_ID="$ROS_DOMAIN_ID" \
  RMW_IMPLEMENTATION="$RMW_IMPLEMENTATION" \
  src/hongshi_patrol_robot/patrol_robot_description/scripts/run_isaac_sim.sh \
    --gui --cmd-vel-topic /cmd_vel_isaac \
  > /tmp/robot320_isaac.log 2>&1 &
echo $! | tee /tmp/robot320_isaac.pid
```

等待场景、车辆、MID-360 和 ROS Bridge 初始化完成。未看到就绪标志前不要启动 SLAM：

```bash
tail -f /tmp/robot320_isaac.log
# 出现 PATROL_ISAAC_READY 后按 Ctrl+C，只退出 tail，不会关闭 Isaac Sim。
```

第二步，启动外部 SLAM Toolbox、Nav2 和项目 RViz2。这里必须使用 Isaac 专用地图名，
不能加载 Gazebo pose graph：

```bash
nohup env \
  DISPLAY="$DISPLAY" \
  XAUTHORITY="$XAUTHORITY" \
  XDG_RUNTIME_DIR="$XDG_RUNTIME_DIR" \
  DBUS_SESSION_BUS_ADDRESS="$DBUS_SESSION_BUS_ADDRESS" \
  ROS_DOMAIN_ID="$ROS_DOMAIN_ID" \
  RMW_IMPLEMENTATION="$RMW_IMPLEMENTATION" \
  ros2 launch robot320_localization_bringup robot320_simulation.launch.py \
    gazebo:=false mode:=mapping \
    persistent_map:=$HOME/robot320_maps/patrol_isaac \
    navigation:=true exploration:=false rviz:=true gui:=false \
  > /tmp/robot320_isaac_nav.log 2>&1 &
echo $! | tee /tmp/robot320_isaac_nav.pid
```

第三步，从仓库根目录通过 uv 启动 Python 地图导航 GUI：

```bash
cd ~/Develop/ROS_ws/hongshi_patrol_ws/src/hongshi_patrol_robot
nohup env \
  DISPLAY="$DISPLAY" \
  XAUTHORITY="$XAUTHORITY" \
  XDG_RUNTIME_DIR="$XDG_RUNTIME_DIR" \
  DBUS_SESSION_BUS_ADDRESS="$DBUS_SESSION_BUS_ADDRESS" \
  ROS_DOMAIN_ID="$ROS_DOMAIN_ID" \
  RMW_IMPLEMENTATION="$RMW_IMPLEMENTATION" \
  ./scripts/uv_run.sh desktop robot320_navigation_gui \
    --domain-id 0 --use-sim-time \
  > /tmp/robot320_isaac_gui.log 2>&1 &
echo $! | tee /tmp/robot320_isaac_gui.pid
```

三个窗口正常出现后做最小健康检查：

```bash
ros2 topic info /clock --verbose       # Publisher count 必须为 1
ros2 topic hz /livox/lidar             # Ctrl+C 结束统计
ros2 lifecycle get /slam_toolbox       # 应为 active [3]
ros2 lifecycle get /planner_server     # 应为 active [3]
ros2 lifecycle get /controller_server  # 应为 active [3]
ros2 run tf2_ros tf2_echo map base_footprint
```

如果 Isaac Sim 窗口出现但 RViz2/GUI 没出现，优先查看对应日志，而不是重复执行启动命令：

```bash
tail -n 100 /tmp/robot320_isaac.log
tail -n 100 /tmp/robot320_isaac_nav.log
tail -n 100 /tmp/robot320_isaac_gui.log
```

停止时先取消导航并发送零速，再按 GUI → Nav2/RViz2 → Isaac Sim 的顺序结束。PID 文件
可能来自旧进程，执行 `kill` 前必须核对命令行：

```bash
ros2 topic pub --once /cmd_vel_isaac geometry_msgs/msg/Twist \
  '{linear: {x: 0.0}, angular: {z: 0.0}}'

for file in /tmp/robot320_isaac_gui.pid \
            /tmp/robot320_isaac_nav.pid \
            /tmp/robot320_isaac.pid; do
  pid=$(cat "$file")
  ps -p "$pid" -o pid=,cmd=
done

# 确认上面三个 PID 与命令正确后，再依次执行：
kill -TERM "$(cat /tmp/robot320_isaac_gui.pid)"
kill -TERM "$(cat /tmp/robot320_isaac_nav.pid)"
kill -TERM "$(cat /tmp/robot320_isaac.pid)"
```

停止后再次运行 `pgrep -af` 和 `ros2 topic info /clock --verbose`，确认没有孤儿仿真、
点云预处理或时钟发布进程。残留节点会造成重复点云、TF 时间回跳和 DDS TypeObject
错误。

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
cd ~/Develop/ROS_ws/hongshi_patrol_ws
source src/hongshi_patrol_robot/scripts/source_lyrical_sim.sh

ros2 launch robot320_localization_bringup robot320_simulation.launch.py \
  gazebo:=true mode:=continuing \
  persistent_map:=$HOME/robot320_maps/patrol_gazebo \
  navigation:=true exploration:=true rviz:=true gui:=true
```

`continuing` 是默认模式：从指定的 `persistent_map` 加载完整 SLAM pose
graph，持续扩建 `/map`，并每 30 秒自动更新 pose graph、YAML 和 PGM。首次找不到
pose graph 时会创建新图；传统 `mode:=localization` 只加载静态地图。

通过地图鼠标点击发布 Nav2 目标：

```bash
./scripts/uv_run.sh desktop robot320_navigation_gui \
  --domain-id 0 --use-sim-time
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
