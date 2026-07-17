# MID-360s 建图与定位

NUC 统一 launch 组成以下链路：

```text
MID-360s -> /livox/lidar -> mid360_preprocess -> /filtered_points
         -> Cartographer -> /tracked_pose + map/base_link TF
         -> chassis telemetry -> Fast DDS robot320/state
```

## 1. 依赖与构建

- Ubuntu 24.04、ROS 2 Jazzy
- `ros-jazzy-cartographer-ros`、PCL、`pcl_conversions`、`tf2_ros`
- Livox SDK2
- NUC 系统镜像自带的 ROS 2/Fast DDS 通讯环境

```bash
rosdep install --from-paths . --ignore-src -r -y
./scripts/uv_setup.sh nuc
./scripts/uv_run.sh nuc ./build.sh
```

NUC uv profile 使用 `/usr/bin/python3` 和 system site packages，以读取 apt 安装的 ROS 2
模块。Livox SDK2 安装见 [`livox_ros_driver2/README.md`](../livox_ros_driver2/README.md)。

## 2. 网络与雷达外参

默认示例地址：

- NUC 雷达网口：`192.168.1.50`
- MID-360s：`192.168.1.107`

启动前确认能从 NUC ping 雷达。`lidar_x/y/z` 单位为米，
`lidar_roll/pitch/yaw` 单位为弧度，表示 `base_link -> livox_frame`。默认全零只适合台架；
装车后必须测量，否则地图、定位和导航都会产生系统误差。

## 3. 建图

```bash
./scripts/uv_run.sh nuc ros2 launch \
  robot320_localization_bringup robot320_slam.launch.py \
  mode:=mapping \
  host_ip:=192.168.1.50 lidar_ip:=192.168.1.107 \
  lidar_x:=0.0 lidar_y:=0.0 lidar_z:=0.0 \
  lidar_roll:=0.0 lidar_pitch:=0.0 lidar_yaw:=0.0
```

完成后保存 Cartographer 状态：

```bash
mkdir -p /var/lib/robot320/maps
./scripts/uv_run.sh nuc ros2 service call \
  /write_state cartographer_ros_msgs/srv/WriteState \
  "{filename: '/var/lib/robot320/maps/site.pbstream'}"
```

## 4. 定位

```bash
./scripts/uv_run.sh nuc ros2 launch \
  robot320_localization_bringup robot320_slam.launch.py \
  mode:=localization \
  map_state_file:=/var/lib/robot320/maps/site.pbstream \
  host_ip:=192.168.1.50 lidar_ip:=192.168.1.107
```

定位模式要求 `.pbstream` 已存在。只调雷达时可传 `enable_chassis:=false`；排查通讯网关时
可临时传 `enable_fastdds_gateway:=false`。

该 launch 提供定位和 Fast DDS 到 Nav2 action 的网关，但不包含现场 Nav2 planner、
controller、costmap 参数。发送导航目标前必须另行启动 `/navigate_to_pose` action server；
否则目标会收到 `rejected` reply。

## 5. 验证

```bash
./scripts/uv_run.sh nuc ros2 topic hz /livox/lidar
./scripts/uv_run.sh nuc ros2 topic hz /filtered_points
./scripts/uv_run.sh nuc ros2 topic echo /tracked_pose
./scripts/uv_run.sh nuc ros2 run tf2_ros tf2_echo map base_link
./scripts/uv_run.sh nuc ros2 topic echo /robot320/telemetry
```

位姿超过 1 秒未更新时不会继续作为有效遥测回传。

## 6. 主要参数

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `mode` | `localization` | `mapping` 或 `localization` |
| `map_state_file` | 空 | 定位使用的 `.pbstream` |
| `host_ip` | `192.168.1.50` | NUC 雷达网口 |
| `lidar_ip` | `192.168.1.107` | MID-360s 地址 |
| `min_z` / `max_z` | `-0.2` / `2.5` | 点云高度范围（米） |
| `voxel_size` | `0.05` | 体素尺寸（米） |
| `map_resolution` | `0.05` | 栅格分辨率（米） |
| `enable_chassis` | `true` | 启动 CAN bridge |
| `enable_fastdds_gateway` | `true` | 启动 Fast DDS ROS gateway |
| `fastdds_domain_id` | `20` | NUC/上位机共同 domain |
| `nav_action` | `/navigate_to_pose` | Nav2 action |
| `nav_cmd_vel_topic` | `/cmd_vel` | Nav2 速度输出 |

定位质量主要取决于雷达外参、时间戳、环境几何特征和地图一致性。
