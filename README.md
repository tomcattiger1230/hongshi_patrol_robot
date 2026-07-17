# Hongshi Patrol Robot

Robot320 巡检机器人项目。NUC 运行 Ubuntu 24.04、ROS 2 Jazzy、底盘驱动、MID-360s
定位并对接 Nav2；同网段上位机运行 PySide6 GUI。Ubuntu 上位机自动使用 ROS 2，
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

Python 环境由根目录 `pyproject.toml`、`uv.lock` 和 `scripts/uv_*.sh` 管理；ROS 2 C++
包仍由 colcon 构建。

## 上位机快速开始

Ubuntu 上位机默认和 NUC 一样已安装 ROS 2。安装
[uv](https://docs.astral.sh/uv/getting-started/installation/) 后直接运行：

```bash
./scripts/uv_setup.sh desktop
./scripts/uv_run.sh desktop robot320_remote_gui --domain-id 20
```

GUI 的 `--backend auto` 默认优先选择 ROS 2。只有 Windows、macOS 等非 Ubuntu 上位机
需要额外安装 Fast DDS Python binding 和生成 ROS 2 String TypeSupport。具体步骤见
[`remote_control/README.md`](./remote_control/README.md)。

非 Ubuntu 系统上的 Fast DDS Python 是 native 扩展。uv 环境、`fastdds` Python
binding 和生成的 `Robot320String` 必须使用相同操作系统、CPU 架构和 Python ABI。

## NUC 快速开始

NUC 的系统镜像默认已安装 ROS 2 Jazzy。通信网关使用 `rclpy`，由 ROS 2 RMW 处理 DDS，
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

## 测试

```bash
./scripts/uv_setup.sh desktop --dev
./scripts/uv_run.sh desktop --dev pytest -q
./scripts/uv_run.sh desktop --dev ruff check \
  robot320_interfaces mobile_platform remote_control
```

## 详细文档

- [上位机与 Windows/macOS Fast DDS 安装](./remote_control/README.md)
- [NUC 底盘和 ROS 2 通信网关](./mobile_platform/README.md)
- [MID-360s 建图与定位](./robot320_localization_bringup/README.md)
- [共享消息和 ROS 2 String 类型](./robot320_interfaces/README.md)
- [Livox 驱动集成](./livox_ros_driver2/README.md)
