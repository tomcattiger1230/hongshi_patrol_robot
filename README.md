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
├── build.sh                      # 默认构建全部 ROS 2 包，可透传 colcon 参数
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

### 2.1 ROS 2

仓库当前在 ROS 2 **Jazzy**（Ubuntu 24.04）下测试。需要安装：

```bash
sudo apt install ros-jazzy-cartographer-ros    # SLAM 定位（运行 robot320_localization_bringup 必需）
```

### 2.2 Livox SDK2（编译 livox_ros_driver2 必需）

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

推荐把仓库放在 colcon 工作区中构建：

```bash
cd /path/to/hongshi_patrol_ws
source /opt/ros/jazzy/setup.bash
./build.sh
source install/setup.bash
```

默认构建仓库内全部 6 个包；也可以把 colcon 参数直接传给脚本：

```bash
./build.sh --packages-select mobile_platform remote_control
```

脚本会自动把 `/usr/lib/python3/dist-packages` 加入 `PYTHONPATH`，确保 colcon 的
CMake 子进程能导入 Jazzy 生成 ROS 消息 IDL 所需的 `lark` 模块。

如果单独构建某一个包（不通过工作区）：

```bash
cd /path/to/hongshi_patrol_robot/mobile_platform
python3 -m pip install --user --no-deps .
```

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
ros2 launch mobile_platform robot320_ros2.launch.py
ros2 launch robot320_localization_bringup robot320_slam.launch.py \
  mode:=localization map_state_file:=/path/to/site.pbstream
ros2 launch remote_control robot320_remote_watch.launch.py
```

## 5. 文档

- [`mobile_platform/README.md`](./mobile_platform/README.md)：车载端完整说明
- [`robot320_localization_bringup/README.md`](./robot320_localization_bringup/README.md)：MID-360s 建图与定位部署
- [`remote_control/README.md`](./remote_control/README.md)：上位机完整说明
- 跨仓库的协议参考（消息字段、FastDDS IDL、ROS 2 topic、安全策略）见原仓库 `hongshi_agent/docs/mobile-platform-architecture.md`

## 6. 迁移历史

2026-07-15：从 `hongshi_agent` 仓库迁出，作为独立仓库首发。`import` 路径保持 `import mobile_platform` / `import remote_control` 不变。
