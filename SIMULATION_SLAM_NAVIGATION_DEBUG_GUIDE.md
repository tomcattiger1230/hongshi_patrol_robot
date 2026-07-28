# Robot320 双仿真 SLAM 与导航调试手册

本文是当前开发平台的专用执行手册，覆盖 Gazebo 和 NVIDIA Isaac Sim 两条仿真链路：

1. 自动探索并使用 SLAM Toolbox 建图；
2. 以约 0.5 Hz 的仿真时间周期更新 `/map`；
3. 保存并重新载入地图；
4. 使用 AMCL、Nav2 和 Python GUI 完成重定位及自主导航。

本文的命令以以下环境为准：

| 项目 | 当前值 |
|---|---|
| 远程主机 | `192.168.3.113` |
| 用户 | `arnold` |
| ROS 2 | Jazzy |
| 工作区 | `/home/arnold/Develop/ROS_ws/partrol_ws` |
| 仓库 | `/home/arnold/Develop/ROS_ws/partrol_ws/src/hongshi_patrol_robot` |
| Isaac Sim | `/home/arnold/isaacsim` |
| 默认仿真 ROS Domain | `0` |

> 注意：工作区目录确实拼写为 `partrol_ws`。不要使用旧平台的
> `/home/arnold/Develop/ROS2_ws/patrol_ws`，也不要 source
> `/opt/ros/lyrical/setup.bash`。

## 1. 登录、构建和 Python 环境

从开发机登录：

```bash
ssh arnold@192.168.3.113
```

首次构建或清除了 `build/ install/ log/` 后：

```bash
cd /home/arnold/Develop/ROS_ws/partrol_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

检查关键 ROS 包：

```bash
ros2 pkg prefix patrol_robot_description
ros2 pkg prefix robot320_localization_bringup
ros2 pkg prefix slam_toolbox
ros2 pkg prefix nav2_bringup
```

如果 `slam_toolbox` 或 Nav2 没找到，先确认本终端已经 source Jazzy 和工作区。若
`ros2 pkg prefix slam_toolbox` 仍失败，才需要安装缺少的 Jazzy 二进制包；不要用其他
ROS 发行版的环境覆盖 Jazzy。

初始化 GUI 的 uv 环境：

```bash
cd /home/arnold/Develop/ROS_ws/partrol_ws/src/hongshi_patrol_robot
./scripts/uv_setup.sh desktop --dev
```

Linux `desktop` profile 使用 `/usr/bin/python3` 和 system site packages，因此 uv
环境可以读取 apt 安装的 `rclpy`。不要在普通 venv 中单独 `pip install rclpy`。

## 2. 每次调试前的统一检查

每个新终端先执行：

```bash
cd /home/arnold/Develop/ROS_ws/partrol_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0
```

确认没有旧仿真仍在发布时钟：

```bash
ros2 topic info /clock --verbose
```

未启动仿真时应没有 `/clock`；启动后 `Publisher count` 必须为 `1`。Gazebo 和 Isaac
Sim 不得在同一 ROS Domain 中同时运行，否则会出现：

- `moved backwards in time`；
- `tf2 buffer detected jump back in time`；
- RViz 地图、点云或模型频闪；
- SLAM/AMCL 丢失 TF。

优先在原 launch 终端按 `Ctrl-C` 正常退出。只有遗留进程无法退出时，才执行：

```bash
pkill -f robot320_simulation.launch.py || true
pkill -f 'gz sim' || true
pkill -f parameter_bridge || true
pkill -f patrol_robot_isaac_sim.py || true
pkill -f robot320_navigation_gui || true
```

再次检查：

```bash
pgrep -af 'gz sim|parameter_bridge|patrol_robot_isaac_sim|robot320_simulation'
```

### 2.1 默认持久续建模式

`robot320_simulation.launch.py` 默认使用 `mode:=continuing`。默认文件前缀是：

```text
$HOME/robot320_maps/patrol_current
```

存在 `.posegraph` 和 `.data` 时，SLAM Toolbox 会加载旧图并继续添加扫描、闭环优化和
扩建 `/map`；不存在时会从空白地图建立第一份持久图。运行期间每 30 秒自动更新：

```text
patrol_current.posegraph
patrol_current.data
patrol_current.yaml
patrol_current.pgm
```

默认持续建图和导航：

```bash
ros2 launch robot320_localization_bringup robot320_simulation.launch.py \
  gazebo:=true mode:=continuing navigation:=true exploration:=true \
  rviz:=true gui:=true
```

需要立即保存时：

```bash
ros2 service call /robot320/save_persistent_map std_srvs/srv/Trigger '{}'
```

可以用 `persistent_map:=/absolute/path/map_prefix` 更换文件前缀。YAML/PGM 只有栅格
图像，无法恢复历史雷达扫描和图优化约束；续建必须同时保留 `.posegraph` 和 `.data`。

## 3. Gazebo 自动建图

### 3.1 启动

带本地桌面时：

```bash
cd /home/arnold/Develop/ROS_ws/partrol_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0

ros2 launch robot320_localization_bringup robot320_simulation.launch.py \
  gazebo:=true mode:=mapping navigation:=true exploration:=true \
  rviz:=true gui:=true
```

纯 SSH 或无显示环境使用：

```bash
ros2 launch robot320_localization_bringup robot320_simulation.launch.py \
  gazebo:=true mode:=mapping navigation:=true exploration:=true \
  rviz:=false gui:=false
```

自动探索器会从 `/map` 选取前沿目标，并通过 Nav2 驱动车辆。自行车底盘最小转弯半径为
`2.35 m`，不能原地旋转；探索轨迹出现大弧线和短距离倒车属于预期行为。

### 3.2 建图链路验收

在另一个已 source 的终端执行：

```bash
ros2 topic list | grep -E '^/(clock|odom|livox/lidar|scan|map|cmd_vel)$'
ros2 topic info /clock --verbose
ros2 topic echo /scan --once --field header
ros2 topic hz /map
ros2 lifecycle get /bt_navigator
ros2 action info /navigate_to_pose
```

预期结果：

- `/clock` 只有一个发布者；
- `/scan` 的 `frame_id` 为 `lidar_link`；
- `/map` 约每 `2.0` 个仿真秒更新一次，即约 `0.5 Hz`；
- `/bt_navigator` 为 `active [3]`；
- `/navigate_to_pose` 存在 action server。

`mapping` 和默认 `continuing` 模式下，地图随最新雷达数据持续更新。`continuing`
还会每 30 秒自动持久化；显式 `mapping` 模式仍需按下一节手动保存。

### 3.3 保存地图

保持建图 launch 运行，在另一个终端执行：

```bash
cd /home/arnold/Develop/ROS_ws/partrol_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

mkdir -p maps
MAP_PREFIX="$PWD/maps/gazebo_map"
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "{name: {data: '${MAP_PREFIX}'}}"
ls -lh maps/gazebo_map.yaml maps/gazebo_map.pgm
```

服务返回 `result=0` 且 YAML、PGM 均存在才算保存成功。仿真时
`nav2_map_server map_saver_cli` 偶尔受 QoS 或仿真时间影响而超时，调试时优先使用上面的
SLAM Toolbox 服务。

## 4. Gazebo 载图、定位和自主导航

先对建图 launch 按 `Ctrl-C`，确认旧进程退出，再启动定位：

```bash
cd /home/arnold/Develop/ROS_ws/partrol_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0

ros2 launch robot320_localization_bringup robot320_simulation.launch.py \
  gazebo:=true mode:=localization \
  map:="$PWD/maps/gazebo_map.yaml" \
  navigation:=true exploration:=false rviz:=true gui:=true
```

仿真出生点与建图原点一致时 launch 会给 AMCL 初始位姿。若机器人被移动、粒子不收敛或
位置未知，可使用 GUI 的初始位姿和全局重定位功能。

不通过 GUI 时，可直接发送一个测试目标：

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 3.0, y: 0.0}, orientation: {w: 1.0}}}}"
```

目标应落在已知自由区，并为阿克曼转向预留足够空间。距离过短、目标紧贴墙面或要求车辆
原地掉头时，Nav2 可能正确地判定无可行路径。

### 4.1 使用键盘手动引导建图

需要人工控制扫描路线时，必须关闭 Nav2 和自动探索，避免多个节点同时发布
`/cmd_vel`：

```bash
ros2 launch robot320_localization_bringup robot320_simulation.launch.py \
  gazebo:=true mode:=mapping navigation:=false exploration:=false \
  rviz:=true gui:=true
```

在另一个已 source 的交互终端运行：

```bash
ros2 run robot320_localization_bringup keyboard_teleop
```

键位为 `W/S` 前进和倒车、`A/D` 左右转向、`R` 回正、空格停车、`Q` 退出，也支持
方向键。松开运动键 `0.8 s` 后节点自动停车。按 `Ctrl-C` 或终端异常退出时也会连续
发布停车指令。自行车模型不能原地转向，推荐沿场景外围走大环线，再逐步覆盖内部通道，
避免驶入没有足够空间退出的死角。

## 5. Isaac Sim 自动建图

Isaac Sim 和外部 ROS 节点分别在两个终端运行。必须先看到 Isaac 端就绪，再启动 SLAM。

### 5.1 终端 1：启动 Isaac Sim

```bash
cd /home/arnold/Develop/ROS_ws/partrol_ws/src/hongshi_patrol_robot
export ROS_DOMAIN_ID=0
./patrol_robot_description/scripts/run_isaac_sim.sh \
  --cmd-vel-topic /cmd_vel_isaac
```

脚本默认使用 `~/isaacsim`、宿主 ROS 2 Jazzy 和 Isaac 内置 Jazzy ROS bridge。不要提前
source Lyrical，也不要把宿主机的 `PYTHONPATH` 注入 Isaac Python。

需要 Isaac 图形界面时：

```bash
./patrol_robot_description/scripts/run_isaac_sim.sh --gui \
  --cmd-vel-topic /cmd_vel_isaac
```

`/cmd_vel_isaac` 是集成仿真内部的最终控制话题。键盘遥控仍向 `/cmd_vel` 发布，
`isaac_cmd_vel_relay` 会在手动指令到达时暂时屏蔽 Nav2 的 `/cmd_vel_smoothed`，
避免非零与零速度交替造成车体剧烈抖动。

### 5.2 终端 2：启动自动 SLAM

```bash
cd /home/arnold/Develop/ROS_ws/partrol_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0

ros2 launch robot320_localization_bringup robot320_simulation.launch.py \
  gazebo:=false mode:=mapping navigation:=true exploration:=true \
  rviz:=true gui:=false
```

链路检查和保存方式与 Gazebo 相同，仅更换地图文件名：

```bash
ros2 topic info /clock --verbose
ros2 topic echo /odom --once --field pose.pose
ros2 topic echo /scan --once --field header
ros2 topic hz /map

mkdir -p maps
MAP_PREFIX="$PWD/maps/isaac_map"
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "{name: {data: '${MAP_PREFIX}'}}"
ls -lh maps/isaac_map.yaml maps/isaac_map.pgm
```

RTX 雷达在 NVIDIA Spark 上可能明显慢于真实时间。`map_update_interval=2.0` 按仿真时间
计算，不保证墙钟时间每两秒更新；判断是否卡死时应同时观察 `/clock` 和 `/odom`，不要
只看终端等待时间。

## 6. Isaac Sim 载图、定位和自主导航

1. 对外部建图 launch 按 `Ctrl-C`；
2. 建议同时重启 Isaac Sim，使机器人重新出生在建图原点；
3. 等 Isaac 发布 `/clock`、`/odom` 和雷达后启动定位。

```bash
cd /home/arnold/Develop/ROS_ws/partrol_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0

ros2 launch robot320_localization_bringup robot320_simulation.launch.py \
  gazebo:=false mode:=localization \
  map:="$PWD/maps/isaac_map.yaml" \
  navigation:=true exploration:=false rviz:=true gui:=false
```

Isaac 的 RTX scan 较稀疏。当前仿真路径让全局/局部代价地图继续使用过滤后的 `/scan`
避障，同时把 Nav2 平滑速度转发给 Isaac 底盘，避免 collision monitor 因稀疏扫描长时间
不输出速度。测试时选择大于最小转弯半径、周围开阔的目标，并预留比 Gazebo 更长的墙钟
时间。不可把这项仿真侧旁路直接照搬到真实车。

## 7. Python 地图导航 GUI

GUI 可在 Gazebo 或 Isaac Sim 定位 launch 运行时单独启动：

```bash
cd /home/arnold/Develop/ROS_ws/partrol_ws/src/hongshi_patrol_robot
./scripts/uv_run.sh desktop robot320_navigation_gui \
  --domain-id 0 --use-sim-time
```

GUI 的主要操作顺序：

1. 等待地图和机器人位姿显示；
2. 位置已知时，点击“设置初始位姿”，在地图上点击并拖动设置朝向；
3. 位置完全未知时，点击“全局重定位”，让车辆低速走大弧线或 S 形获取不同雷达视角；
4. 粒子收敛后，在自由区域点击并拖动发布导航目标；
5. 观察路径、机器人位姿和导航状态，必要时取消目标。

相关接口：

| 功能 | ROS 接口 |
|---|---|
| 地图显示 | `/map` |
| 定位结果 | `/amcl_pose` |
| 初始位姿 | `/initialpose` |
| 导航目标 | `/navigate_to_pose` |
| 全局重定位 | `/reinitialize_global_localization` |
| 静止更新 | `/request_nomotion_update` |

GUI 无地图但 RViz 正常时，首先检查 Domain ID。GUI 默认真实系统 Domain 为 `20`，本地
仿真必须显式传 `--domain-id 0`，或者让所有仿真、Nav2 和 GUI 统一使用另一个相同值。

也可从命令行触发 AMCL 恢复：

```bash
ros2 service call /reinitialize_global_localization std_srvs/srv/Empty '{}'
ros2 service call /request_nomotion_update std_srvs/srv/Empty '{}'
```

## 8. 常见故障

### 8.1 `moved backwards in time`、TF 时间跳变或 RViz 频闪

通常是旧 Gazebo/Isaac 仍在发布 `/clock`。检查：

```bash
ros2 topic info /clock --verbose
```

发布者必须只有一个。停止所有仿真，等待旧节点从 ROS 图消失，再只启动一种仿真。RViz
应由项目 launch 启动，以加载正确的 `use_sim_time`、Fixed Frame 和点云 QoS。

### 8.2 `/map` 不更新

按数据链逐段检查：

```bash
ros2 topic hz /livox/lidar
ros2 topic hz /scan
ros2 topic echo /scan --once --field header
ros2 run tf2_ros tf2_echo base_link lidar_link
ros2 topic hz /map
```

`/scan` 必须使用 `lidar_link`，且 `base_link -> lidar_link` TF 可查询。当前 MID-360
外参为前方约 `0.40 m`、左右居中、高 `1.50 m`。

### 8.3 机器人撞墙后找不到路径

这是自行车模型，不是双轮差速。车轮直径 `430 mm`，最小转弯半径 `2350 mm`，不能靠
原地旋转脱困。先取消目标，确认车后方安全并让车辆退出贴墙状态，然后清理代价地图：

```bash
ros2 service call /global_costmap/clear_entirely_global_costmap \
  nav2_msgs/srv/ClearEntireCostmap '{}'
ros2 service call /local_costmap/clear_entirely_local_costmap \
  nav2_msgs/srv/ClearEntireCostmap '{}'
```

重新选择更远、更开阔且朝向可达的目标。目标点在障碍膨胀区、车辆出生时与障碍重叠，或
通道宽度不足时，清理代价地图也不会产生物理上不可行的路径。

### 8.4 Nav2 action 存在但车辆不动

```bash
ros2 lifecycle get /bt_navigator
ros2 topic hz /cmd_vel
ros2 topic echo /odom --once --field pose.pose
```

确认 `bt_navigator` 为 active，目标被接受，并且 `/cmd_vel` 与 `/odom` 都在变化。
ROS 2 Jazzy 本项目的速度消息类型统一为 `geometry_msgs/msg/Twist`；同一个 ROS 图中不要
混用 `TwistStamped`。

### 8.5 Isaac Sim 看似停止

先比较两个时刻的仿真状态：

```bash
ros2 topic echo /clock --once
ros2 topic echo /odom --once --field pose.pose
```

若仿真时间仍前进，只是 RTX 计算慢，应继续等待。若 `/clock` 完全不前进，再检查 Isaac
终端是否报错、GPU 驱动是否正常以及 Isaac ROS bridge 是否加载成功。

## 9. 推荐验收顺序

每次修改仿真、SLAM、Nav2 或 GUI 后按以下顺序验证，便于定位故障层级：

1. `colcon build --symlink-install` 成功；
2. `/clock` 只有一个发布者；
3. `/odom`、TF、`/livox/lidar`、`/scan` 连续存在；
4. mapping 模式的 `/map` 持续更新；
5. 自动探索能接受目标并改变 `/odom`；
6. 保存得到非空 YAML 和 PGM；
7. localization 模式成功加载该 YAML；
8. AMCL、planner、controller、BT navigator 均 active；
9. GUI 显示地图和位姿；
10. GUI 初始位姿、全局重定位和导航目标均能到达对应 ROS 接口；
11. 机器人对开阔、可达目标产生运动并在目标附近结束。

修改 Python 代码后可补充运行仓库测试：

```bash
cd /home/arnold/Develop/ROS_ws/partrol_ws/src/hongshi_patrol_robot
./scripts/uv_run.sh desktop --dev pytest -q
```

需要保存 launch 日志时，可以将命令输出复制到文件：

```bash
mkdir -p /home/arnold/Develop/ROS_ws/partrol_ws/debug_logs
ros2 launch robot320_localization_bringup robot320_simulation.launch.py \
  gazebo:=true mode:=mapping navigation:=true exploration:=true \
  rviz:=false gui:=false 2>&1 | \
  tee /home/arnold/Develop/ROS_ws/partrol_ws/debug_logs/gazebo_mapping.log
```

日志中应重点搜索 `ERROR`、`TF_OLD_DATA`、`extrapolation`、`timeout`、生命周期状态变化和
导航 action 的拒绝原因。
