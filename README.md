# Hongshi Patrol Robot

Robot320 移动底盘的 ROS 2 包，独立仓库。

仓库现在包含一个共享接口包、一个上位机包和四个 NUC 端相关包：

- `mobile_platform/` — 车载端 ROS 2 包（CAN + 安全门控 + 车载节点）
- `remote_control/` — 上位机控制 ROS 2 包（CLI / GUI / ROS 2 客户端 / FastDDS 入口）
- `livox_ros_driver2/` — ZIP 导入的 Livox MID-360s ROS 2 驱动
- `mid360_preprocess/` — MID-360s 点云裁剪与降采样
- `robot320_localization_bringup/` — NUC 端底盘、雷达与 Cartographer 统一启动
- `robot320_interfaces/` — 不依赖 ROS 2 的共享语义消息与 Fast DDS IDL

## 1. 仓库结构

```text
hongshi_patrol_robot/
├── README.md                     # 本文档
├── pyproject.toml                # uv 的 desktop / nuc / dev 依赖定义
├── uv.lock                       # 两端共用的锁文件，必须提交
├── build.sh                      # 默认构建全部 ROS 2 包，可透传 colcon 参数
├── scripts/uv_setup.sh           # 创建 desktop 或 NUC 环境
├── scripts/uv_run.sh             # 带 native overlay 运行 uv 命令
├── .gitignore
├── robot320_interfaces/          # ROS-independent 消息与 DDS 契约
├── mobile_platform/              # 车载端 ROS 2 包
│   ├── package.xml
│   ├── setup.py
│   ├── setup.cfg
│   ├── MANIFEST.in
│   ├── resource/mobile_platform
│   ├── launch/robot320_ros2.launch.py
│   ├── vendor/controlcan/        # libcontrolcan.so 多架构
│   ├── README.md
│   └── mobile_platform/          # Python 子包（import 路径）
│       ├── __init__.py
│       ├── can_types.py
│       ├── controlcan.py
│       ├── protocol.py
│       ├── robot320.py
│       ├── messages.py
│       ├── transport.py
│       ├── safety.py
│       ├── onboard_node.py
│       ├── ros2_node.py
│       ├── fastdds_node.py
│       └── cli.py
├── remote_control/               # 上位机 ROS 2 包
    ├── package.xml
    ├── setup.py
    ├── setup.cfg
    ├── MANIFEST.in
    ├── resource/remote_control
    ├── launch/robot320_remote_watch.launch.py
    ├── README.md
    └── remote_control/           # Python 子包（import 路径）
        ├── __init__.py
        ├── cli.py
        ├── dds_client.py
        ├── fastdds_client.py
        ├── gui.py
        └── ros2_client.py
├── livox_ros_driver2/            # Livox MID-360s 驱动（ament_cmake）
├── mid360_preprocess/            # 点云滤波（ament_cmake）
└── robot320_localization_bringup/ # NUC 统一启动（ament_python）
    ├── launch/robot320_slam.launch.py
    ├── config/mid360_2d.lua
    ├── config/mid360_localization.lua
    └── README.md
```

Python 包采用 ament_python 标准嵌套布局；Livox 驱动和点云预处理采用 ament_cmake。

## 2. 前置依赖

### 2.1 uv

本仓库统一使用 uv 管理 Python 版本、虚拟环境、仓库内可编辑包和 PyPI 依赖。安装 uv 后
不要再分别执行 `pip install`：

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
uv --version
```

`desktop` profile 安装上位机、PySide6 和共享接口；`nuc` profile 安装车载包并使用
`--system-site-packages` 继承 Ubuntu 中由 apt 安装的 ROS 2 Python 模块。开发工具通过
可选的 `--dev` 安装。两端始终使用仓库提交的同一个 `uv.lock`。

### 2.2 ROS 2

仓库当前在 ROS 2 **Jazzy**（Ubuntu 24.04）下测试。需要安装：

```bash
sudo apt install ros-jazzy-cartographer-ros    # SLAM 定位（运行 robot320_localization_bringup 必需）
```

### 2.3 Livox SDK2（编译 livox_ros_driver2 必需）

```bash
git clone https://github.com/Livox-SDK/Livox-SDK2.git /tmp/Livox-SDK2
cd /tmp/Livox-SDK2 && mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=$HOME/.local \
         -DCMAKE_CXX_FLAGS="-include cstdint -Wno-error"
make -j$(nproc) && make install
```

编译时需 `LD_LIBRARY_PATH`，运行时也需：
```bash
export LD_LIBRARY_PATH=$HOME/.local/lib:$LD_LIBRARY_PATH
```

## 3. 构建

### 3.1 上位机

```bash
./scripts/uv_setup.sh desktop
./scripts/uv_run.sh desktop robot320_remote_gui --domain-id 20
```

Fast DDS Python 是 native 扩展，创建 desktop 环境时选择的 Python 必须与它的编译 ABI
一致；例如绑定使用 Python 3.12 构建时执行
`./scripts/uv_setup.sh desktop --python 3.12`。NUC profile 默认使用 ROS 2 Jazzy 的
`/usr/bin/python3`，避免 ABI 分叉。

如需运行测试，首次创建环境时增加 `--dev`：

```bash
./scripts/uv_setup.sh desktop --dev
./scripts/uv_run.sh desktop --dev pytest -q
```

### 3.2 NUC

NUC 上使用 ROS 2 Jazzy 的系统 Python 创建 uv 环境，再通过同一个 wrapper 调用 colcon：

```bash
cd /path/to/hongshi_patrol_ws
./scripts/uv_setup.sh nuc
./scripts/uv_run.sh nuc ./build.sh
```

`uv_run.sh nuc` 会依次加载 `/opt/ros/jazzy/setup.bash`、可选的 Fast DDS overlay 和已经
生成的 `install/setup.bash`，因此构建完成后可直接启动：

```bash
./scripts/uv_run.sh nuc ros2 launch \
  robot320_localization_bringup robot320_slam.launch.py \
  mode:=localization map_state_file:=/path/to/site.pbstream
```

默认构建仓库内全部 6 个包；也可以把 colcon 参数直接传给脚本：

```bash
./scripts/uv_run.sh nuc ./build.sh --packages-select mobile_platform remote_control
```

脚本会自动把 `/usr/lib/python3/dist-packages` 加入 `PYTHONPATH`，确保 colcon 的
CMake 子进程能导入 Jazzy 生成 ROS 消息 IDL 所需的 `lark` 模块。

ROS 2 或 Fast DDS Python bindings 是针对操作系统和 Python ABI 编译的 native overlay，
不属于 PyPI 依赖。若 Fast DDS 安装在独立 colcon 工作区，通过环境变量交给 wrapper：

```bash
FASTDDS_SETUP=/path/to/Fast-DDS-python/install/setup.bash \
  ./scripts/uv_run.sh desktop robot320_remote_gui --domain-id 20
```

生成的 `Robot320Dds` 默认从
`robot320_interfaces/generated/Robot320Dds/build` 自动加入 `PYTHONPATH`；也可用
`ROBOT320_DDS_TYPES=/other/build` 覆盖。NUC 的 ROS 路径可用 `ROS_SETUP` 覆盖，仓库
colcon overlay 可用 `ROBOT320_SETUP` 覆盖。

## 4. 运行入口

构建后 `ros2 run` 可用的 console scripts：

```text
mobile_platform:
  robot320_onboard           UDP JSON 调试入口（无需 ROS 2）
  robot320_ros2_bridge       ROS 2 车载 CAN 桥
  robot320_fastdds_bridge    Fast DDS → CAN 车载桥
  robot320_cli               CAN 命令行调试

remote_control:
  robot320_remote_cli        UDP JSON 远程 CLI
  robot320_remote_ros2       ROS 2 上位机入口
  robot320_remote_fastdds    不依赖 ROS 2 的 Fast DDS 上位机
  robot320_remote_gui        PySide6 + Fast DDS 图形控制台（无需 ROS 2）
```

启动 launch：

```bash
./scripts/uv_run.sh nuc ros2 launch mobile_platform robot320_ros2.launch.py
./scripts/uv_run.sh nuc ros2 launch robot320_localization_bringup robot320_slam.launch.py \
  mode:=localization map_state_file:=/path/to/site.pbstream
./scripts/uv_run.sh desktop robot320_remote_gui --domain-id 20
```

## 5. 文档

- [`mobile_platform/README.md`](./mobile_platform/README.md)：车载端完整说明
- [`robot320_localization_bringup/README.md`](./robot320_localization_bringup/README.md)：MID-360s 建图与定位部署
- [`remote_control/README.md`](./remote_control/README.md)：上位机完整说明
- 跨仓库的协议参考（消息字段、FastDDS IDL、ROS 2 topic、安全策略）见原仓库 `hongshi_agent/docs/mobile-platform-architecture.md`

## 6. 迁移历史

2026-07-15：从 `hongshi_agent` 仓库迁出，作为独立仓库首发。`import` 路径保持 `import mobile_platform` / `import remote_control` 不变。
