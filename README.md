# Hongshi Patrol Robot

Robot320 移动底盘的 ROS 2 包，独立仓库。

仓库内包含两个 ament_python 包：

- `mobile_platform/` — 车载端 ROS 2 包（CAN + 安全门控 + 车载节点）
- `remote_control/` — 上位机控制 ROS 2 包（CLI / GUI / ROS 2 客户端 / FastDDS 入口）

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
└── remote_control/               # 上位机 ROS 2 包
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
```

每个包都使用 ament_python 标准嵌套布局：`setup.py` 与 `<package-name>/<package-name>/` 同级，`import` 路径不变。

## 2. 构建

推荐放在 colcon 工作区 `hongshi_patrol_ws` 下，根 `build.sh` 已切换为只构建本仓库的这两个包：

```bash
cd /path/to/hongshi_patrol_ws
source /opt/ros/<distro>/setup.bash      # 仓库当前在 /opt/ros/lyrical 下测试
./build.sh --packages-select mobile_platform remote_control
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
ros2 launch remote_control robot320_remote_watch.launch.py
```

## 4. 文档

- [`mobile_platform/README.md`](./mobile_platform/README.md)：车载端完整说明
- [`remote_control/README.md`](./remote_control/README.md)：上位机完整说明
- 跨仓库的协议参考（消息字段、FastDDS IDL、ROS 2 topic、安全策略）见原仓库 `hongshi_agent/docs/mobile-platform-architecture.md`

## 5. 迁移历史

2026-07-15：从 `hongshi_agent` 仓库迁出，作为独立仓库首发。`import` 路径保持 `import mobile_platform` / `import remote_control` 不变。
