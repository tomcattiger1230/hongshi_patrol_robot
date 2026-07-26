# MID-360s SLAM、定位与导航

## 推荐方案

室内平整路面的巡检导航采用以下链路：

```text
MID-360 -> /livox/lidar --+-> pointcloud_to_laserscan -> /scan
                          |       +-> SLAM Toolbox（建图）
                          |       +-> AMCL + 静态地图（运行时定位）
                          |
                          +-> mid360_preprocess -> /filtered_points
                                  +-> Nav2 2D/3D obstacle costmaps

odom -> base_footprint -----> SLAM/AMCL -> map -> odom
Nav2 Smac Hybrid + MPPI Ackermann -> /cmd_vel -> EPS/后桥控制
```

该组合保留 MID-360 点云给障碍物层，同时生成 `/scan` 供成熟的 2D SLAM 和 AMCL
使用。全局规划使用 `SmacPlannerHybrid`，局部控制使用带 2.35 m 最小转弯半径约束的
`MPPI Ackermann`。仓库原有 Cartographer 配置保留为备选。

方案选择：

| 方案 | 建议 |
|---|---|
| SLAM Toolbox + AMCL + Nav2 | 当前首选；适合室内平地、二维巡检地图和长期复用 |
| Cartographer 2D | 保留备选；当 MID-360 投影后的单帧 `/scan` 过稀时对比测试 |
| FAST-LIO2/Point-LIO | 二期三维定位候选；适合坡道、室外和明显三维运动，但仍需单独生成 Nav2 二维地图与长期重定位链路 |

## 1. 依赖与构建

- Ubuntu 26.04、ROS 2 Lyrical
- `slam_toolbox`、`pointcloud_to_laserscan`、Navigation2
- PCL、`pcl_conversions`、`tf2_ros`
- Livox SDK2
- NUC 系统镜像自带的 ROS 2 通讯环境

Lyrical 当前没有完整的 Navigation2 二进制包，先用仓库内锁定的源码清单构建独立
underlay，再构建机器人工作区。安装脚本同时安装 Cyclone DDS；仿真 launch 会自动选择
`rmw_cyclonedds_cpp`，避开该版本 Fast DDS 在多生命周期节点启动时的进程继承卡顿：

```bash
source /opt/ros/lyrical/setup.bash
cd ~/Develop/ROS2_ws/patrol_ws/src/hongshi_patrol_robot
./scripts/setup_lyrical_navigation.sh \
  ~/Develop/ROS2_ws/navigation_lyrical_ws

source ~/Develop/ROS2_ws/navigation_lyrical_ws/install/setup.bash
cd ~/Develop/ROS2_ws/patrol_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

NUC uv profile 使用 `/usr/bin/python3` 和 system site packages，以读取 apt 安装的 ROS 2
模块。Livox SDK2 安装见 [`livox_ros_driver2/README.md`](../livox_ros_driver2/README.md)。

## 2. 障碍物仿真场景

Gazebo 和 Isaac Sim 使用一致的约 28 × 24 m 可探索区域，包含外围墙、多段宽通道、
箱体、托盘堆、低矮障碍和圆柱罐，面积约为旧场景的四倍。通道为 2.35 m 最小转弯
半径的自行车底盘保留了转弯空间。

### 自动 SLAM 建图

启动 Gazebo、MID-360 点云投影、SLAM Toolbox、Nav2 和前沿探索器：

```bash
cd ~/Develop/ROS2_ws/patrol_ws
source src/hongshi_patrol_robot/scripts/source_lyrical_sim.sh
ros2 pkg prefix slam_toolbox

ros2 launch robot320_localization_bringup robot320_simulation.launch.py \
  mode:=mapping navigation:=true exploration:=true rviz:=true gui:=true
```

检查命令应输出
`~/Develop/ROS2_ws/navigation_lyrical_ws/install/slam_toolbox`。只 source
`/opt/ros/lyrical` 或项目工作区并不能保证源码构建的 `slam_toolbox` 进入包索引。

前沿探索器每两秒检查 `/map` 的“已知自由区/未知区”边界，过滤不满足车体安全间距
的候选点，然后通过 `NavigateToPose` 让 Smac Hybrid + MPPI Ackermann 自动驶向下一个
候选点。规划器使用 Reeds-Shepp 曲线，必要时允许最高 0.20 m/s 的短距离倒车，以满足
2.35 m 最小转弯半径。连续没有候选点时表示自动探索完成。

保持建图 launch 运行，在第二个终端保存地图；确认 YAML 和 PGM 都已生成后，再回到
第一个终端按 `Ctrl-C` 停车：

```bash
mkdir -p maps
ros2 run nav2_map_server map_saver_cli -f maps/patrol_test
```

如果要人工遥控建图，使用 `exploration:=false navigation:=false`，再通过 `/cmd_vel`
低速绕场；不要同时启用 `demo:=true`，它会与 Nav2 竞争控制命令。

### 自动导航

使用保存的地图启动 AMCL、Nav2 和专用 RViz：

```bash
ros2 launch robot320_localization_bringup robot320_simulation.launch.py \
  mode:=localization \
  map:=$PWD/maps/patrol_test.yaml \
  navigation:=true exploration:=false rviz:=true gui:=true
```

在 RViz 中先用 “2D Pose Estimate” 给出粗略初始位姿；粒子云收敛后用
“Nav2 Goal” 点击目标点并拖动指定最终朝向。也可以启动项目 GUI：

```bash
cd ~/Develop/ROS2_ws/patrol_ws/src/hongshi_patrol_robot
./scripts/uv_run.sh desktop robot320_navigation_gui \
  --domain-id 20 --use-sim-time
```

定位启动后可通过地图导航 GUI、RViz 的 “2D Pose Estimate” 或 `/initialpose` 设置粗略
初始位姿。完全不知道位置时，可在 GUI 中启动 AMCL 全局重定位；机器人随后应低速走
大弧线或 S 形路径，使 MID-360 获得不同视角并让粒子分布收敛。当前 AMCL 配置启用了
动态障碍波束跳过、最多 5000 粒子和随机位姿恢复注入，以改善复杂场景和“机器人被搬动”
后的恢复能力。
不要同时启用 `demo:=true` 和 `navigation:=true`，否则两个节点会竞争 `/cmd_vel`。

阿克曼底盘不能使用 Nav2 默认的原地旋转脱困。项目会自动加载专用行为树：碰撞预测
使用比车身前后各大 0.28 m 的安全包络；控制失败时先以 0.12 m/s 后退 0.60 m，再清理
代价地图并重新规划。速度平滑器允许的倒车上限为 0.20 m/s。若车身已经在物理上顶住
墙面，应先人工急停并确认车后方安全，再恢复自动导航。

### RViz 频闪

不要直接运行裸 `rviz2`，它会加载 `~/.rviz2/default.rviz` 中旧的 Seer 配置。该配置
使用 `/lidar/points`、`base_footprint` 固定坐标系，并将点云 `Decay Time` 设为零，
10 Hz 的 MID-360 帧会被逐帧替换，看起来像高频闪烁。上述 launch 会加载项目专用配置：

- 建图/导航固定坐标系为 `map`，纯仿真模型为 `odom`；
- 点云话题为 `/livox/lidar`，QoS 为 Best Effort；
- 点云保留 0.5–0.75 秒，并给 RViz 设置 `use_sim_time:=true`。

只看模型与雷达时可运行：

```bash
ros2 launch patrol_robot_description patrol_robot_sim.launch.py \
  rviz:=true gui:=true
```

若仍频闪，先确认只有一个仿真时钟：

```bash
ros2 topic info /clock --verbose
```

`Publisher count` 必须为 1。切换仿真 launch 前应先 `Ctrl-C` 关闭旧的 Gazebo。
项目 launch 直接托管 `gz-sim-main`，正常退出时会一并终止 Gazebo，避免旧世界继续
发布较大的 `/clock` 后导致新世界出现 `moved backwards in time` 和 `TF_OLD_DATA`。

ROS 2 Lyrical 的 Nav2 使用 `geometry_msgs/msg/TwistStamped` 发布 `/cmd_vel`。Gazebo、
Isaac Sim 和实车通信网关已经统一适配该类型；若手工接入其他控制节点，不要再在同名
话题上发布旧的 `geometry_msgs/msg/Twist`。

## 3. 网络与雷达外参

默认示例地址：

- NUC 雷达网口：`192.168.1.50`
- MID-360s：`192.168.1.107`

启动前确认能从 NUC ping 雷达。`lidar_x/y/z` 单位为米，
`lidar_roll/pitch/yaw` 单位为弧度，表示 `base_link -> livox_frame`。仓库按当前测量值
预设 `x=0.40 m、y=0、z=1.50 m`；仍应在装车后复测，否则地图、定位和导航都会产生
系统误差。

## 4. Cartographer 备选建图

```bash
./scripts/uv_run.sh nuc ros2 launch \
  robot320_localization_bringup robot320_slam.launch.py \
  mode:=mapping \
  host_ip:=192.168.1.50 lidar_ip:=192.168.1.107 \
  lidar_x:=0.40 lidar_y:=0.0 lidar_z:=1.50 \
  lidar_roll:=0.0 lidar_pitch:=0.0 lidar_yaw:=0.0
```

完成后保存 Cartographer 状态：

```bash
mkdir -p /var/lib/robot320/maps
./scripts/uv_run.sh nuc ros2 service call \
  /write_state cartographer_ros_msgs/srv/WriteState \
  "{filename: '/var/lib/robot320/maps/site.pbstream'}"
```

## 5. Cartographer 备选定位

```bash
./scripts/uv_run.sh nuc ros2 launch \
  robot320_localization_bringup robot320_slam.launch.py \
  mode:=localization \
  map_state_file:=/var/lib/robot320/maps/site.pbstream \
  host_ip:=192.168.1.50 lidar_ip:=192.168.1.107
```

定位模式要求 `.pbstream` 已存在。只调雷达时可传 `enable_chassis:=false`；排查通讯网关时
可临时传 `enable_fastdds_gateway:=false`。

该 launch 提供定位、ROS 2 String 指令到 Nav2 action 的网关，以及一套待现场标定的
`nav2_ackermann.yaml` 初始参数。默认不启动 Nav2；
在 `/odom` 和 `map -> odom -> base_link` 验证完成后可传 `enable_nav2:=true`。否则即使 action
server 启动，控制器也无法获得连续可靠的底盘速度。

Robot320 初始阿克曼参数为轴距 `0.700 m`、最小转弯半径 `2.350 m`、等效前轮最大转角
`16.59 deg`，CAN 单一转向执行器命令范围为 `0..350`。这些参数均可通过 launch 参数覆盖。

## 6. 验证

```bash
./scripts/uv_run.sh nuc ros2 topic hz /livox/lidar
./scripts/uv_run.sh nuc ros2 topic hz /filtered_points
./scripts/uv_run.sh nuc ros2 topic hz /scan
./scripts/uv_run.sh nuc ros2 topic echo /map --once
./scripts/uv_run.sh nuc ros2 topic echo /tracked_pose
./scripts/uv_run.sh nuc ros2 run tf2_ros tf2_echo map base_footprint
./scripts/uv_run.sh nuc ros2 topic echo /robot320/telemetry
```

位姿超过 1 秒未更新时不会继续作为有效遥测回传。

## 7. 主要参数

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `mode` | `localization` | `mapping` 或 `localization` |
| `map_state_file` | 空 | 定位使用的 `.pbstream` |
| `host_ip` | `192.168.1.50` | NUC 雷达网口 |
| `lidar_ip` | `192.168.1.107` | MID-360s 地址 |
| `min_z` / `max_z` | `-1.35` / `-0.10` | 雷达坐标系内点云高度范围（米） |
| `voxel_size` | `0.05` | 体素尺寸（米） |
| `map_resolution` | `0.05` | 栅格分辨率（米） |
| `enable_chassis` | `true` | 启动 CAN bridge |
| `enable_fastdds_gateway` | `true` | 启动 ROS 2 通信网关（参数名为兼容旧配置保留） |
| `fastdds_domain_id` | `20` | `ROS_DOMAIN_ID`（参数名为兼容旧配置保留） |
| `nav_action` | `/navigate_to_pose` | Nav2 action |
| `nav_cmd_vel_topic` | `/cmd_vel` | Nav2 速度输出 |
| `enable_nav2` | `false` | 启动仓库内的 Nav2 Ackermann 配置 |
| `nav2_params_file` | 包内配置 | Nav2 参数文件 |
| `wheelbase` | `0.700` | 前后轴距（米） |
| `min_turning_radius` | `2.350` | 最小转弯半径（米） |
| `max_wheel_angle` | `16.59` | 等效前轮最大转角（度） |
| `max_steering_command` | `350` | CAN 转向执行器最大命令量 |

定位质量主要取决于雷达外参、时间戳、环境几何特征和地图一致性。

MID-360 安装高度为 1.50 m，向下视场有限。点云投影和导航滤波默认选择雷达坐标系
`z=-1.35..-0.10 m`，对应车体上方约 0.15–1.40 m 的障碍物。低于该范围或进入雷达
近距离盲区的物体可能无法稳定检测；实车安全层应增加低位安全雷达、超声波或深度相机，
不能只依赖高位 MID-360。
