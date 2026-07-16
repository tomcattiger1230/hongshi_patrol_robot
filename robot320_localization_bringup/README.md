# Robot320 MID-360s SLAM 定位

NUC 端统一 launch 将以下节点组成一条定位链路：

```text
Livox MID-360s
  -> /livox/lidar
  -> mid360_preprocess
  -> /filtered_points
  -> Cartographer
  -> /tracked_pose + map -> base_link TF
  -> mobile_platform
  -> RobotTelemetry.pose
```

## 1. NUC 依赖

- Ubuntu + ROS 2
- Livox SDK2，且 `/usr/local/lib/liblivox_lidar_sdk_shared.so` 可用
- Cartographer ROS、PCL、`pcl_conversions`、`tf2_ros`
- NUC 网口与雷达位于同一网段

ZIP 中的 `livox-sdk_20240719-1_arm64.deb` 只适用于 ARM64，不能安装到常见的
x86_64 Intel NUC。NUC 上应安装对应架构的 Livox SDK2，再执行：

```bash
rosdep install --from-paths . --ignore-src -r -y
colcon build --symlink-install --packages-select \
  livox_ros_driver2 mid360_preprocess mobile_platform \
  robot320_localization_bringup remote_control
source install/setup.bash
```

## 2. 网络和安装外参

原 ZIP 中实际配置使用雷达 `192.168.1.107`、主机 `192.168.1.50`；另一个旧 launch
写的是 `192.168.1.111`。启动时必须传入现场真实地址：

```bash
ping 192.168.1.107
```

`lidar_x/y/z` 单位为米，`lidar_roll/pitch/yaw` 单位为弧度，表示
`base_link -> livox_frame` 的固定变换。默认全部为零只适合台架验证，装车后必须测量。

## 3. 建图

```bash
export ROS_DOMAIN_ID=20
ros2 launch robot320_localization_bringup robot320_slam.launch.py \
  mode:=mapping \
  host_ip:=192.168.1.50 \
  lidar_ip:=192.168.1.107 \
  lidar_x:=0.0 lidar_y:=0.0 lidar_z:=0.0 \
  lidar_roll:=0.0 lidar_pitch:=0.0 lidar_yaw:=0.0
```

移动机器人完成建图后保存 Cartographer 状态：

```bash
mkdir -p /var/lib/robot320/maps
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState \
  "{filename: '/var/lib/robot320/maps/site.pbstream'}"
```

## 4. 定位

定位时机器人应尽量从建图时已知位置启动：

```bash
export ROS_DOMAIN_ID=20
ros2 launch robot320_localization_bringup robot320_slam.launch.py \
  mode:=localization \
  map_state_file:=/var/lib/robot320/maps/site.pbstream \
  host_ip:=192.168.1.50 \
  lidar_ip:=192.168.1.107 \
  lidar_x:=0.0 lidar_y:=0.0 lidar_z:=0.0 \
  lidar_roll:=0.0 lidar_pitch:=0.0 lidar_yaw:=0.0
```

只调试雷达和定位、不连接 CAN 时增加 `enable_chassis:=false`。

## 5. 验证

```bash
ros2 topic hz /livox/lidar
ros2 topic hz /filtered_points
ros2 topic echo /tracked_pose
ros2 run tf2_ros tf2_echo map base_link
ros2 topic echo /robot320/telemetry
```

笔记本与 NUC 使用相同 `ROS_DOMAIN_ID` 后，执行：

```bash
robot320_remote_ros2 watch --seconds 30
```

输出中的 `pose=(x,y,yaw)` 即 NUC 回传的 SLAM 位姿。超过 1 秒没有收到新的
`/tracked_pose` 时，遥测会把 pose 标记为不可用，避免上位机误用旧定位。

## 6. 常用参数

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `mode` | `localization` | `mapping` 或 `localization` |
| `map_state_file` | 空 | 定位模式必需的 `.pbstream` |
| `host_ip` | `192.168.1.50` | NUC 雷达网口地址 |
| `lidar_ip` | `192.168.1.107` | MID-360s 地址 |
| `min_z` / `max_z` | `-0.2` / `2.5` | 点云高度裁剪范围 |
| `voxel_size` | `0.05` | 体素降采样尺寸（米） |
| `map_resolution` | `0.05` | 栅格地图分辨率（米） |
| `enable_chassis` | `true` | 是否同时启动 Robot320 CAN 桥 |

定位质量首先取决于准确的雷达外参、稳定的时间戳、足够的环境几何特征和匹配的地图。
