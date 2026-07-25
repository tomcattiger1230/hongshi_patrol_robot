# Patrol Robot Description

这是一个面向 ROS 2 Jazzy 和 Gazebo Harmonic 的最小巡检机器人仿真包。机器人只使用
box、cylinder 和 sphere 构造，采用 Gazebo `DiffDrive` 插件控制。

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

手动发送速度控制（启动时不要设置 `demo:=true`）：

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.3}, angular: {z: 0.4}}"
```

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

## Isaac Sim 6

Isaac Sim 安装在默认的 `~/isaacsim` 时，启动无界面仿真：

```bash
./patrol_robot_description/scripts/run_isaac_sim.sh
```

启动自动巡航：

```bash
./patrol_robot_description/scripts/run_isaac_sim.sh --demo
```

有桌面或远程流式显示环境时：

```bash
./patrol_robot_description/scripts/run_isaac_sim.sh --gui
```

若 Isaac Sim 位于其他目录，可设置 `ISAAC_SIM_ROOT`。Isaac Sim 后端复用与 Gazebo
相同的 `/cmd_vel`、`/odom`、`/joint_states`、`/tf` 和 `/clock` 话题，因此手动控制
命令无需改变。
