# Hongshi Patrol Robot

Robot320 移动底盘的 ROS 2 包，独立仓库。

仓库现在包含一个上位机包和四个 NUC 端相关包：

- `mobile_platform/` — 车载端 ROS 2 包（CAN + 安全门控 + 车载节点）
- `remote_control/` — 上位机控制 ROS 2 包（CLI / GUI / ROS 2 客户端 / FastDDS 入口）
- `livox_ros_driver2/` — ZIP 导入的 Livox MID-360s ROS 2 驱动
- `mid360_preprocess/` — MID-360s 点云裁剪与降采样
- `robot320_localization_bringup/` — NUC 端底盘、雷达与 Cartographer 统一启动

## 1. 仓库结构

```text
hongshi_patrol_robot/
├── README.md                     # 本文档
├── .gitignore
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

## 2. 构建

推荐把仓库放在 colcon 工作区中构建：

```bash
cd /path/to/hongshi_patrol_ws
source /opt/ros/<distro>/setup.bash      # 仓库当前在 /opt/ros/lyrical 下测试
colcon build --symlink-install --packages-select \
  livox_ros_driver2 mid360_preprocess mobile_platform \
  robot320_localization_bringup remote_control
source install/setup.bash
```

如果单独构建某一个包（不通过工作区）：

```bash
cd /path/to/hongshi_patrol_robot/mobile_platform
python3 -m pip install --user --no-deps .
```

## 3. 运行入口

构建后 `ros2 run` 可用的 console scripts：

```text
mobile_platform:
  robot320_onboard           UDP JSON 调试入口（无需 ROS 2）
  robot320_ros2_bridge       ROS 2 车载 CAN 桥
  robot320_fastdds_bridge    FastDDS 入口骨架
  robot320_cli               CAN 命令行调试

remote_control:
  robot320_remote_cli        UDP JSON 远程 CLI
  robot320_remote_ros2       ROS 2 上位机入口
  robot320_remote_fastdds    FastDDS 入口骨架
  robot320_remote_gui        Tkinter GUI 原型
```

启动 launch：

```bash
ros2 launch mobile_platform robot320_ros2.launch.py
ros2 launch robot320_localization_bringup robot320_slam.launch.py \
  mode:=localization map_state_file:=/path/to/site.pbstream
ros2 launch remote_control robot320_remote_watch.launch.py
```

## 4. 文档

- [`mobile_platform/README.md`](./mobile_platform/README.md)：车载端完整说明
- [`robot320_localization_bringup/README.md`](./robot320_localization_bringup/README.md)：MID-360s 建图与定位部署
- [`remote_control/README.md`](./remote_control/README.md)：上位机完整说明
- 跨仓库的协议参考（消息字段、FastDDS IDL、ROS 2 topic、安全策略）见原仓库 `hongshi_agent/docs/mobile-platform-architecture.md`

## 5. 迁移历史

2026-07-15：从 `hongshi_agent` 仓库迁出，作为独立仓库首发。`import` 路径保持 `import mobile_platform` / `import remote_control` 不变。
