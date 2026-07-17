# Livox ROS Driver 2（项目集成）

本目录是项目内使用的 Livox ROS Driver 2 源码，目标设备为 MID-360s，目标环境为
Ubuntu 24.04 + ROS 2 Jazzy。不要使用上游面向 ROS1、Foxy 或 Humble 的独立 build
脚本；本仓库通过根目录 `build.sh` 和统一 launch 构建、启动。

上游项目、完整参数和许可证：
[Livox-SDK/livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2)。

## 依赖

先安装 [Livox SDK2](https://github.com/Livox-SDK/Livox-SDK2)：

```bash
git clone https://github.com/Livox-SDK/Livox-SDK2.git /tmp/Livox-SDK2
cmake -S /tmp/Livox-SDK2 -B /tmp/Livox-SDK2/build \
  -DCMAKE_INSTALL_PREFIX=/usr/local
cmake --build /tmp/Livox-SDK2/build --parallel
sudo cmake --install /tmp/Livox-SDK2/build
```

若安装到自定义 prefix，需要把其 `lib` 加入 `LD_LIBRARY_PATH`。

## 构建与运行

```bash
./scripts/uv_setup.sh nuc
./scripts/uv_run.sh nuc ./build.sh --packages-up-to livox_ros_driver2
FASTDDS_SETUP=/path/to/Fast-DDS-python/install/setup.bash \
  ./scripts/uv_run.sh nuc ros2 launch \
  robot320_localization_bringup robot320_slam.launch.py \
  mode:=mapping host_ip:=192.168.1.50 lidar_ip:=192.168.1.107
```

统一 launch 会根据 `host_ip` 和 `lidar_ip` 生成 SDK JSON，并启动驱动。常用输出：

| Topic | 类型 | 用途 |
|---|---|---|
| `/livox/lidar` | Livox PointCloud2 | 原始点云 |
| `/livox/imu` | IMU | 雷达 IMU（当前 2D 配置未使用） |

点云随后由 `mid360_preprocess` 写入 `/filtered_points`，再交给 Cartographer。网络、外参、
建图和定位参数统一见
[`robot320_localization_bringup/README.md`](../robot320_localization_bringup/README.md)。

## 排查

- NUC 雷达网口和 MID-360s 必须在同一子网。
- `host_ip` 必须是 NUC 雷达网口地址，不能填其他网卡地址。
- 无点云时先检查 ping、UDP 防火墙和 SDK JSON 中的地址/端口。
- `liblivox_sdk_shared.so` 找不到时检查安装 prefix 和 `LD_LIBRARY_PATH`。
