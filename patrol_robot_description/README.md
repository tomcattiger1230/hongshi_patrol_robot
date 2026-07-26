# Patrol Robot Description

这是一个面向 ROS 2 Jazzy 和 Gazebo Harmonic 的最小巡检机器人仿真包。机器人只使用
box、cylinder 和 sphere 构造，采用前轮 EPS 转向、后桥驱动的 Ackermann/自行车模型。

模型依据 roboQ-320 配置清单设置：轴距 700 mm、前后轮距 825 mm、车轮直径
430 mm、最小转弯半径 2350 mm、机械最大转角 35°。仿真控制转角限制为约
16.6°，以满足 2.35 m 的自行车模型最小转弯半径。

## 编译

```bash
cd /home/arnold/ROS_ws/partrol_ws
source /opt/ros/jazzy/setup.bash
export PYTHONPATH=/usr/lib/python3/dist-packages${PYTHONPATH:+:$PYTHONPATH}
colcon build --symlink-install --packages-select patrol_robot_description
source install/setup.bash
```

## 启动

DGX Spark 默认使用无界面模式：

```bash
ros2 launch patrol_robot_description patrol_robot_sim.launch.py
```

启动自动巡航演示：

```bash
ros2 launch patrol_robot_description patrol_robot_sim.launch.py demo:=true
```

有桌面显示时可启用 Gazebo GUI：

```bash
ros2 launch patrol_robot_description patrol_robot_sim.launch.py gui:=true
```

场景可探索区域约为 28 × 24 m，包含外围墙、多段宽通道、高箱体、托盘堆和圆柱罐，
面积约为原场景的四倍。所有主要障碍物高度均不低于 1.8 m，能够覆盖安装高度为
1.5 m 的 MID-360s 水平扫描面。机器人默认从场景左下方
`(-10.5, -8.5)` 的无碰撞区域生成，也可以通过 `spawn_x`、`spawn_y` 和
`spawn_yaw` launch 参数覆盖。需要同时验证 SLAM Toolbox、AMCL 和 Nav2 时，使用
`robot320_localization_bringup/launch/robot320_simulation.launch.py`，具体命令见
该包 README。

手动发送速度控制（启动时不要设置 `demo:=true`）：

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.3}, angular: {z: 0.12}}"
```

`angular.z` 是期望横摆角速度。车辆不能原地旋转；控制器按
`steering = atan(wheel_base * angular.z / linear.x)` 换算前轮转角，并按
2.35 m 最小转弯半径限幅。当 `linear.x` 为零时，`angular.z` 不会产生运动。

停止：

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
```

主要话题：

| 话题 | 类型 | 方向 |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | ROS 2 → Gazebo |
| `/odom` | `nav_msgs/msg/Odometry` | Gazebo → ROS 2 |
| `/joint_states` | `sensor_msgs/msg/JointState` | Gazebo → ROS 2 |
| `/tf` | `tf2_msgs/msg/TFMessage` | Gazebo → ROS 2 |
| `/clock` | `rosgraph_msgs/msg/Clock` | Gazebo → ROS 2 |
| `/livox/lidar` | `sensor_msgs/msg/PointCloud2` | 仿真 MID-360s → ROS 2 |

## MID-360s 雷达仿真

Gazebo 和 Isaac Sim 都在 `lidar_link` 上安装仿真雷达，并将点云统一发布为
`/livox/lidar`（`sensor_msgs/msg/PointCloud2`）。Gazebo 使用 GPU LiDAR，设置为
360° 水平视场、约 59° 垂直视场、0.1–70 m 量程和 10 Hz。

雷达中心在 `base_footprint` 坐标系中的安装位置为
`xyz="0.40 0.00 1.50"`：高度 150 cm，距车体前端 36.5 cm、后端 116.5 cm，
左右居中并距两侧各约 39 cm。

Isaac Sim 6 没有内置 Livox MID-360 扫描模板，因此使用 RTX `Example_Rotary`
模板，并覆盖量程、安装位置和输出坐标系。这适合验证避障、点云管线及 ROS 2
接口，但扫描线分布和 Livox 非重复扫描模式并不等同于真实 MID-360。需要验证
Livox 特有算法时，应继续使用实机录制的 rosbag。

## Isaac Sim 6

Isaac Sim 安装在默认的 `~/isaacsim` 时，启动无界面仿真：

```bash
./patrol_robot_description/scripts/run_isaac_sim.sh
```

脚本使用 Isaac Sim 6 内置的 Python 3.12 Jazzy ROS 库和 Cyclone DDS，不应在同一终端
预先 source ROS 2 Lyrical。Lyrical 的 SLAM/Nav2 应在另一个终端启动。

启动自动巡航：

```bash
./patrol_robot_description/scripts/run_isaac_sim.sh --demo
```

有桌面或远程流式显示环境时：

```bash
./patrol_robot_description/scripts/run_isaac_sim.sh --gui
```

若 Isaac Sim 位于其他目录，可设置 `ISAAC_SIM_ROOT`。Isaac Sim 后端复用与 Gazebo
相同的 `/cmd_vel`、`/odom`、`/joint_states`、`/tf`、`/clock` 和
`/livox/lidar` 话题，因此手动控制及点云消费端无需改变。
