# Robot320 上位机

上位机正式入口是 PySide6 GUI。Ubuntu 自动使用 `rclpy`；Windows/macOS 使用 standalone
Fast DDS。两种后端都连接 NUC 的 ROS 2 `std_msgs/String` JSON Topic。

GUI 支持：

- 按住持续发送的前进、后退和转向，松开立即停车
- 停止、刹车、急停和解除急停
- Nav2 目标发送、取消、状态和进度
- ROS 2 地图显示、机器人实时位置和鼠标拖拽目标位姿
- 升降杆动作和目标高度
- 底盘、SLAM 位姿、升降杆、电池、故障和指令应答

## 1. Python 环境

仓库的 `desktop` profile 安装 `robot320_interfaces`、`remote_control` 和 PySide6：

```bash
./scripts/uv_setup.sh desktop --python 3.12
```

Ubuntu 上位机由 `uv_run.sh` 自动 source `/opt/ros/jazzy/setup.bash`，GUI 的默认
`--backend auto` 会强制选择 ROS 2。Linux desktop uv 环境使用 `/usr/bin/python3` 和
system site packages，以读取 apt 安装的 ROS 2 Python 模块：

```bash
./scripts/uv_run.sh desktop robot320_remote_gui --domain-id 20
```

Windows/macOS 才需要下面三个 native 层，并且 Python ABI 必须完全一致：

1. Fast DDS / Fast CDR C++ runtime
2. 提供 `import fastdds` 的 Fast-DDS-python binding
3. 由项目 IDL 生成的 ROS 2 String `Robot320String` Python module

Windows 和 macOS 没有项目预装的 Ubuntu/ROS 2 环境，才需要按下面章节额外准备三个
native 层。Fast DDS 本身不是 PyPI 包，不能只靠 `uv sync` 安装。

## 2. Windows 安装 Fast DDS

### 2.1 前置条件

- 64 位 Python（建议 3.12，并确认所选 Fast-DDS-python 版本支持）
- Visual Studio，勾选 **Desktop development with C++**
- CMake、Git、Java、SWIG 4.1 和 uv

先在仓库中固定 Python：

```bat
cd /d C:\path\to\hongshi_patrol_robot
uv venv --python 3.12 .venv
uv sync --locked --extra desktop --no-default-groups
set ROBOT320_PYTHON=%CD%\.venv\Scripts\python.exe
```

Fast DDS C++ runtime/Gen 有两条官方路径：

- 使用 [eProsima Windows 二进制安装器](https://fast-dds.docs.eprosima.com/en/stable/installation/binaries/binaries_windows.html)，安装时选择匹配的 Visual Studio 和 x64 架构；
- 按 [Windows 源码安装](https://fast-dds.docs.eprosima.com/en/stable/installation/sources/sources_windows.html) 编译 Fast DDS、Fast CDR 和 Fast DDS-Gen。

二进制安装器不等于 Python binding。GUI 仍需在 **Developer Command Prompt for VS**
中构建 [Fast-DDS-python](https://github.com/eProsima/Fast-DDS-python)：

```bat
mkdir C:\fastdds-python
cd /d C:\fastdds-python
curl.exe -L https://raw.githubusercontent.com/eProsima/Fast-DDS-python/master/fastdds_python.repos -o fastdds_python.repos
mkdir src
uvx --from vcstool vcs import src --input fastdds_python.repos
uvx --from colcon-common-extensions colcon build --packages-up-to fastdds_python --cmake-args -DPython3_EXECUTABLE="%ROBOT320_PYTHON%"
cd src\fastddsgen
gradlew.bat assemble
set PATH=%CD%\scripts;%PATH%
cd ..\..
call install\setup.bat
```

如果已经用安装器装好了 C++ runtime，也可以按官方 Windows 源码文档的 CMake 路径只
构建 Python binding，并通过 `CMAKE_PREFIX_PATH` 指向安装器目录。

### 2.2 Windows 生成项目类型并运行

保持上一步 `install\setup.bat` 已调用，然后从仓库根目录生成 IDL 类型：

```bat
mkdir robot320_interfaces\generated\Robot320String
cd robot320_interfaces\generated\Robot320String
fastddsgen.bat -python -replace ..\..\robot320_interfaces\dds\Robot320String.idl
cmake -S . -B build -DPython3_EXECUTABLE="%CD%\..\..\..\.venv\Scripts\python.exe"
cmake --build build --config Release
set PYTHONPATH=%CD%;%CD%\build\Release;%CD%\build;%PYTHONPATH%
cd /d ..\..\..
uv run --locked --extra desktop --no-default-groups robot320_remote_gui --domain-id 20
```

若 Windows 防火墙弹出网络请求，应允许专用网络访问；否则 DDS discovery 可能无法找到
NUC。官方文档也提示 Windows 可能需要单独的防火墙规则。

## 3. macOS 安装 Fast DDS

macOS 没有官方二进制安装器。Fast DDS C++ runtime 和 Fast DDS-Gen 应按
[官方 macOS 源码安装](https://fast-dds.docs.eprosima.com/en/stable/installation/sources/sources_mac.html)
构建，前置条件包括 Homebrew、Xcode Command Line Tools、CMake、Asio、TinyXML2、
OpenSSL 和 Java。

```bash
xcode-select --install
brew install cmake asio tinyxml2 openssl wget openjdk
```

Fast-DDS-python 要求 SWIG 低于 4.2（推荐 4.1）。Homebrew 当前默认版本可能更高，必须
先用 `swig -version` 核对，并按 SWIG/Homebrew 的版本化安装方式准备 4.1。

随后可使用 Fast-DDS-python 官方仓库的 colcon workspace 方式构建 binding：

```bash
export ROBOT320_REPO=/path/to/hongshi_patrol_robot
cd "$ROBOT320_REPO"
./scripts/uv_setup.sh desktop --python 3.12
export ROBOT320_PYTHON="$ROBOT320_REPO/.venv/bin/python"

mkdir -p ~/fastdds-python/src
cd ~/fastdds-python
curl -L https://raw.githubusercontent.com/eProsima/Fast-DDS-python/master/fastdds_python.repos \
  -o fastdds_python.repos
uvx --from vcstool vcs import src --input fastdds_python.repos
uvx --from colcon-common-extensions colcon build --packages-up-to fastdds_python \
  --cmake-args -DPython3_EXECUTABLE="$ROBOT320_PYTHON"
cd src/fastddsgen
./gradlew assemble
export PATH="$PWD/scripts:$PATH"
cd ../..
source install/setup.bash
```

重要限制：Fast-DDS-python 上游当前公开 CI 只标明 Ubuntu 和 Windows，官方安装手册也
没有单独的 macOS Python binding 章节。因此 macOS binding 属于源码构建路径，必须在
目标 Mac 和目标 Python 上实际验证；若构建失败，正式可支持方案是 Windows 上位机或
Linux 虚拟机，而不是复用其他操作系统生成的 `.so`/`.dylib`。

构建完成后回到仓库。以下脚本会像参考项目一样，把 `fastdds` binding 与项目的
ROS 2 String TypeSupport 直接安装/编译到当前 `.venv`，运行时不再依赖手工设置
`PYTHONPATH`：

```bash
cd "$ROBOT320_REPO"
FASTDDS_PREFIX="$HOME/fastdds-python/install" \
FASTDDS_PYTHON_SOURCE="$HOME/fastdds-python/src/fastdds_python" \
FASTDDSGEN_SOURCE="$HOME/fastdds-python/src/fastddsgen" \
  ./scripts/setup_fastdds.sh
./scripts/uv_run.sh desktop robot320_remote_gui --domain-id 20
```

## 4. 使用 GUI

### 4.1 地图点击导航

地图导航窗口需要直接连接 ROS 2，订阅持久化 `/map`，通过 TF 获取
`map -> base_footprint`，并调用 Nav2 `/navigate_to_pose` action。因此应在 NVIDIA
Spark 本机、带 X11 转发的 SSH 会话，或能够加入同一 ROS 2 Domain 的 Ubuntu 上位机运行。

仿真时先启动定位和 Nav2：

```bash
ros2 launch robot320_localization_bringup robot320_simulation.launch.py \
  mode:=localization \
  map:=$PWD/maps/patrol_test.yaml \
  navigation:=true gui:=false
```

然后启动地图导航 GUI：

```bash
./scripts/uv_run.sh desktop robot320_navigation_gui \
  --domain-id 20 --use-sim-time
```

GUI 默认立即读取
`~/robot320_maps/patrol_current.yaml` 和同目录的 PGM，因此即使 ROS 后端尚未启动也能
预览上次地图。后端发布 `/map` 后会自动切换到实时地图。需要载入其他地图或 pose
graph 时：

```bash
./scripts/uv_run.sh desktop robot320_navigation_gui \
  --map-file /path/to/site.yaml \
  --pose-graph /path/to/site
```

地图中的红点是 `/scan` 根据当前 `map -> lidar_link` 变换投影后的 MID-360 匹配结果。
“贴墙率”统计红点落在已知障碍边缘 0.15 m 范围内的比例。红点贴合黑色墙面且该比例
稳定表示当前雷达匹配合理；整体错位通常表示初始位姿或里程计存在偏差。

GUI 使用与 RViz 相同的 `/plan` 显示青色 Nav2 全局规划路径，同时订阅
`/lookahead_collision_arc` 显示黄色控制器局部前视轨迹。右侧导航状态列出两条路径的
点数和累计长度；路径不在 `map` frame 时会先通过 TF 转换后再绘制。

灰色栅格是占用值 `-1` 的未知区域，不是已经确认可通行的地面。在 `mode:=continuing`
下，GUI 允许把灰色边界附近设为探索目标，Smac Hybrid 的 `allow_unknown` 会规划进入，
SLAM Toolbox 随扫描把可见区域更新为白色或黑色。`mode:=localization` 使用只读静态
地图，GUI 会拒绝灰色目标并提示切换模式。为了保持碰撞监测余量，应优先选择白色与灰色
交界处，不要直接把目标放到大片未知区域深处。

本机仿真默认 ROS domain 为 0，此时把上述参数改为 `--domain-id 0`。GUI、仿真器和
Nav2 必须使用同一个 domain ID。

实车运行时去掉 `--use-sim-time`。操作顺序：

1. 等待右上角显示“Nav2 已连接”。
2. 如果大致知道机器人位置，在地图上点击并拖动朝向，然后选择“将选中位姿设为初始位置”。
   AMCL 会接收 `/initialpose`；持续建图模式会重新载入序列化 pose graph，并让下一帧
   MID-360 扫描在所选区域附近匹配。
3. AMCL 静态定位模式中，完全不知道位置时可选择“不知道位置：全局重定位”。持续建图
   模式不提供全地图粒子搜索，按钮会改成“回到建图起点：雷达重匹配”，且车辆必须确实
   位于原始建图起点附近。
4. 重定位后使用综合遥控面板，让自行车底盘低速走一段大弧线或 S 形路径；不能要求
   它像差速底盘一样原地旋转。
5. 等待“定位置信度”由“不确定”变为“正在收敛”或“良好”，并确认红色雷达点贴合
   地图墙面。AMCL 可选择“使用当前扫描强制更新”；SLAM Toolbox 需移动至少 0.10 m
   或转向 0.05 rad 才会处理下一帧。
6. 在空闲区域点击并拖动目标朝向，确认坐标后选择“发送目标，开始自动导航”。
7. GUI 持续显示剩余距离、预计时间和最终结果；“取消当前导航”可随时终止。

重定位会先取消正在执行的导航。AMCL 全局重定位调用
`/reinitialize_global_localization`，强制更新调用 `/request_nomotion_update`。
GUI 在发布 AMCL 粗略位姿或启动全局搜索后会自动触发三次无运动扫描更新，使车辆静止时
也能利用连续 MID-360 扫描开始收敛。
持续建图的选区重定位调用 SLAM Toolbox `/slam_toolbox/deserialize_map` 的
`START_AT_GIVEN_POSE`，建图起点重匹配使用 `START_AT_FIRST_NODE`。GUI 同时读取
`/amcl_pose` 和 SLAM Toolbox `/pose` 的协方差；在置信度尚未收敛时不要发送导航目标。

地图采用 ROS `OccupancyGrid` 坐标原点、分辨率和旋转信息进行换算，目标消息的
`frame_id` 使用地图实际 frame。GUI 不会自行绕过 Nav2 安全检查：目标能否接受及能否
到达仍由全局代价地图、规划器、控制器和行为树决定。

如果地图可见但机器人不显示，检查：

```bash
ros2 run tf2_ros tf2_echo map base_footprint
ros2 action info /navigate_to_pose
ros2 topic echo /map --once
ros2 service type /reinitialize_global_localization
ros2 topic echo /amcl_pose --once
```

地图导航功能目前要求 ROS 2 后端。Windows/macOS 的 standalone Fast DDS 模式仍可使用
原有坐标输入和遥控界面，但不会传输体积较大的完整栅格地图。

### 4.2 综合遥控面板

```bash
./scripts/uv_run.sh desktop robot320_remote_gui \
  --domain-id 20 --client-id operator-laptop --backend auto
```

Windows 中先 `call install\setup.bat`，再直接执行对应的 `uv run --locked ...` 命令。
NUC 的 `ROS_DOMAIN_ID` 与上位机 domain ID 必须一致，默认均为 `20`。可用
`--backend ros2` 或 `--backend fastdds` 强制选择，通常保留 `auto` 即可。

## 5. Python API

GUI 和其他应用复用同一个自动后端客户端：

```python
from remote_control.fastdds_client import RobotRemoteFastDDSClient

client = RobotRemoteFastDDSClient(domain_id=20, client_id="operator-laptop")
try:
    client.send_navigation_goal(x_m=3.0, y_m=1.5, yaw_rad=0.0)
    telemetry = client.receive_telemetry(timeout_s=1.0)
    reply = client.receive_reply(timeout_s=1.0)
finally:
    client.close()
```

## 6. 排查

| 现象 | 检查项 |
|---|---|
| Ubuntu 启动后端失败 | `source /opt/ros/jazzy/setup.bash` 后能否导入 `rclpy`、`std_msgs` |
| `FastDDSUnavailable` | Windows/macOS 确认 `import fastdds, Robot320String` 在同一个 uv Python 中成功 |
| GUI 启动但无遥测 | `ROS_DOMAIN_ID`/domain ID、同网段、防火墙、NUC gateway、多网卡路由 |
| Windows 找不到 DLL | 是否在同一终端调用 Fast DDS `setup.bat` |
| macOS 找不到 dylib | Fast DDS prefix 是否已 source，架构是否与 Python 一致 |
| 生成类型导入失败 | 重新用当前 uv Python 运行 Fast DDS-Gen 和 CMake |
