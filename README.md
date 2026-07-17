# Hongshi Patrol Robot

Robot320 巡检机器人项目。NUC 运行 Ubuntu 24.04、ROS 2 Jazzy、底盘驱动、MID-360s
定位并对接 Nav2；同网段上位机运行 PySide6 GUI，通过 Fast DDS 直接收发命令与状态，
上位机不需要 ROS 2。

```text
Windows / macOS 上位机                    Ubuntu NUC
Qt GUI -> Fast DDS command/state  <----> Fast DDS ROS gateway
                                             |
                         ROS 2 / Nav2 / Cartographer / CAN / lift adapter
```

## 仓库组成

| 目录 | 用途 |
|---|---|
| `robot320_interfaces` | ROS-independent 消息、Fast DDS IDL 和传输实现 |
| `remote_control` | Fast DDS 客户端和 PySide6 GUI |
| `mobile_platform` | NUC 的 CAN、ROS 2 和 Fast DDS 网关 |
| `livox_ros_driver2` | 项目内使用的 Livox MID-360s 驱动 |
| `mid360_preprocess` | 点云高度裁剪和体素降采样 |
| `robot320_localization_bringup` | 底盘、雷达、Cartographer 的统一 launch |

Python 环境由根目录 `pyproject.toml`、`uv.lock` 和 `scripts/uv_*.sh` 管理；ROS 2 C++
包仍由 colcon 构建。

## 上位机快速开始

安装 [uv](https://docs.astral.sh/uv/getting-started/installation/) 后：

```bash
./scripts/uv_setup.sh desktop
FASTDDS_PREFIX=/path/to/Fast-DDS/install \
FASTDDS_PYTHON_SOURCE=/path/to/Fast-DDS-python/fastdds_python \
FASTDDSGEN_SOURCE=/path/to/Fast-DDS/src/fastddsgen \
  ./scripts/setup_fastdds.sh
./scripts/uv_run.sh desktop robot320_remote_gui --domain-id 20
```

如果三个源码/安装目录按 `Fast-DDS`、`Fast-DDS-python` 与本仓库并列放置，也可直接执行
`./scripts/uv_setup.sh desktop --fastdds` 一次完成 Python 环境和 native binding 初始化。

Windows 使用相同的 `uv.lock`，但需要先调用 Fast DDS 的 `setup.bat`，再执行
`uv run --locked --extra desktop --no-default-groups robot320_remote_gui --domain-id 20`。
Windows、macOS 的 Fast DDS 安装和生成类型步骤见
[`remote_control/README.md`](./remote_control/README.md)。

Fast DDS Python 是 native 扩展。uv 环境、`fastdds` Python binding 和生成的
`Robot320Dds` 必须使用相同操作系统、CPU 架构和 Python ABI。

## NUC 快速开始

NUC 需预装 ROS 2 Jazzy、Cartographer ROS、Livox SDK2、Fast DDS Python binding 和
Fast DDS-Gen。首次构建：

```bash
rosdep install --from-paths . --ignore-src -r -y
./scripts/uv_setup.sh nuc
FASTDDS_SETUP=/path/to/Fast-DDS-python/install/setup.bash \
  ./scripts/uv_run.sh nuc \
  ./robot320_interfaces/scripts/generate_fastdds_types.sh
FASTDDS_SETUP=/path/to/Fast-DDS-python/install/setup.bash \
  ./scripts/uv_run.sh nuc ./build.sh
```

定位运行：

```bash
FASTDDS_SETUP=/path/to/Fast-DDS-python/install/setup.bash \
  ./scripts/uv_run.sh nuc ros2 launch \
  robot320_localization_bringup robot320_slam.launch.py \
  mode:=localization \
  map_state_file:=/var/lib/robot320/maps/site.pbstream \
  host_ip:=192.168.1.50 lidar_ip:=192.168.1.107
```

`nuc` profile 固定使用 `/usr/bin/python3` 并允许 system site packages，使 uv 环境能读取
apt 安装的 `rclpy`。`uv_run.sh nuc` 会依次加载 ROS 2、可选 Fast DDS overlay 和仓库的
`install/setup.bash`。

## 测试

```bash
./scripts/uv_setup.sh desktop --dev
./scripts/uv_run.sh desktop --dev pytest -q
./scripts/uv_run.sh desktop --dev ruff check \
  robot320_interfaces mobile_platform remote_control
```

## 详细文档

- [上位机与 Windows/macOS Fast DDS 安装](./remote_control/README.md)
- [NUC 底盘和 Fast DDS ROS 网关](./mobile_platform/README.md)
- [MID-360s 建图与定位](./robot320_localization_bringup/README.md)
- [共享消息和 IDL](./robot320_interfaces/README.md)
- [Livox 驱动集成](./livox_ros_driver2/README.md)
