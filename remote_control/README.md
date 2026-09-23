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

### 离线开发模式（不连接机器人）

macOS、Windows 和 Linux 均可使用 `demo` 后端独立开发综合 GUI。该后端不会创建 DDS
participant、不会连接 ROS 2，也不会访问车辆网络；所有遥测、导航进度、急停和升降杆
状态只在本机内存中模拟。界面标题和黄色横幅会持续标明“离线演示”。

```bash
./scripts/uv_run.sh desktop robot320_remote_gui --backend demo
```

可以在此模式下验证按钮布局、持续按压运动、停止/刹车/急停、坐标导航任务、进度显示、
取消任务、升降杆和指令应答。地图点击导航窗口仍是独立的 ROS 2 GUI；后续接入 DDS 地图
数据时，不应移除离线演示后端。

### 海康视频与云台页

原生综合 GUI 可复用现有 `video_relay/local/web_viewer.py`，无需把摄像头密码放进 Qt
代码或命令行。视频保留 `NUC -> 8.163.54.201:8554/robot -> Mac` 的云端中继架构，
也支持同网时通过 NUC 的 go2rtc 代理低延迟读流。云台仍使用带 token 的 NUC PTZ Agent；
云端视频中继不提供云台控制，也不应把 NUC 的 HTTP PTZ 接口直接暴露到公网。

在 Mac 上分别运行以下两个终端（当前底盘保持离线）：

```bash
# 终端 1：读取忽略版本控制的 video_relay/local/.env
./scripts/uv_run.sh desktop python video_relay/local/web_viewer.py

# 终端 2：只启用真实摄像头，车辆按钮仍操作本地演示后端
./scripts/uv_run.sh desktop robot320_remote_gui --backend demo \
  --camera-url http://127.0.0.1:8081
```

`--camera-url` 默认为空，不主动连接摄像头；启用后横幅会明确提示云台按钮操作真实设备。
页面使用异步网络请求显示 JPEG 画面，网络失败时清除旧画面；方向键按住持续移动，
松开、切换页面或应用失去焦点时请求停止，并依靠 NUC deadman timeout 做第二层保护。
`/health` 仅表示 PTZ Agent 存活，不代表摄像头 ONVIF 链路已经验证。

2026-09-18 联调：车上缺失的 `local_rtsp_proxy.py` 和 `ptz_agent.py` 已补齐。当前 NUC
Wi-Fi 地址为 `192.168.88.108`；Mac 私有 `.env` 需与现场地址一致。另发现旧云端推流
脚本使用 FFmpeg 不支持的 `-rw_timeout`；备份后替换为本地修正版 `-timeout` 并单独
重启视频推流进程，保留云端地址和账号。旧脚本备份位于
`/home/hs/robot320_backups/video_relay_20260918/push_stream.py.before`。
局域网和云端均通过 RTSP 验证：H.264、640×360、12.5 FPS；Mac web viewer 实测约
12.36 FPS，并成功获取 JPEG。认证 PTZ 接口和 ONVIF 停止请求返回成功；实际左右/上下
移动和变焦仍待现场观察确认。三项视频服务均恢复运行。
`lsusb` 已识别两台 `Hikrobot MV-CH100-60UC`（`2bdf:0001`）。这类工业相机使用海康
MVS SDK，没有 `/dev/video*` 并不表示设备未连接。机上的 `hik_camera_node` 已调用
`MvCameraControl`，但只打开枚举列表中的第一台；双路录像需要扩展为按设备序列号选择、
分别启动/停止、文件落盘以及磁盘空间保护，不能直接套用 V4L2 录像命令。

### 四项整合顺序

双摄显示：NUC 的 `CAMERA_SECOND_RTSP_URL` 配置第二路 `/Streaming/Channels/202`，
`SECOND_STREAM_PATH=robot2`；Mac 私有配置增加 `SECOND_LOCAL_STREAM_PATH=robot2`。
原生摄像头页同时显示通道 1、2；Web 页面也提供双画面，第二路接口为 `/video2`、
`/snapshot2.jpg`、`/api/status2`。两路采集独立，第二路失败不会伪装成第一路画面。
第二路已部署独立云端推流服务，发布到 `/robot2`；Mac 配置
`SECOND_CLOUD_STREAM_PATH=robot2`。为改善放大清晰度，现场已切换通道 1 `/101`、
通道 2 `/201` 主码流；两路实测 2560×1440，第二路通过云端约 12.5 FPS。
随后已将两路云端独立转码为 H.264、960×540、12 FPS、每路 600 kbps 目标码率，
减少公网带宽并解决第二路云端 OpenCV H.265 灰屏。局域网仍为 2560×1440 原始画面。
摄像头页使用等宽视频卡片及对称状态栏，画面按比例缩放，不由原始分辨率决定控件大小。
两路实际全景/细节对应关系待现场确认，
目前保留中性的“通道 1/通道 2”标识，不改变现有 PTZ profile。

原生摄像头页每路提供“自动选择 / 局域网 / 云端 / 关闭视频”下拉开关，并从桥接状态
显示实际链路。自动模式可回退，手选局域网或云端时不跨链路回退；关闭只停止 Mac
读取该路视频，不关闭相机、NUC 推流或 PTZ。未配置的链路选项禁用；当前两路均已配置局域网和云端。
切换经本地 `POST /api/link` 完成，独立更换读取器并清除旧帧，不会混入上一链路的数据；
发生网络超时时旧读取器最多等待当前连接/读超时后释放。

| 阶段 | 内容 | 当前状态 |
|---|---|---|
| 1 | Mac 海康监控视频 + PTZ | 局域网/云端视频与 Mac 解码通过；ONVIF 停止通过，移动/变焦待现场确认 |
| 2 | MID-360 点云、轨迹、实时地图 | 车端采集与定位已有；Mac DDS 可视化数据接口待整合 |
| 3 | 鼠标操作底盘 | GUI 与离线后端已有；接入实车安全门控及动作验收待完成 |
| 4 | 两路海康 USB 录像 | USB 设备已识别、MVS 单路采集已有；双路录像与远程任务接口待开发 |

视频采用 RTSP 传输，不将高带宽视频塞进 JSON/DDS 状态消息。DDS 负责车辆指令、
遥测和任务状态；雷达点云需单独限频降采样，地图需版本化。录像计划优先在 NUC 落盘，
Mac 发送开始/停止任务并接收录制状态，避免 Wi-Fi 丢包破坏录像文件。

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

右侧“地图文件”区域提供两个交互操作：

- “保存当前地图…”选择目标 YAML。GUI 立即写出 YAML/PGM；SLAM Toolbox 可用时还会
  依次调用 `serialize_map` 和 `save_map`，写出同名 `.posegraph/.data` 完整会话。
- “载入已有地图…”先预览所选 YAML。AMCL 模式通过 `/map_server/load_map` 热加载；
  continuing 模式要求存在同名 `.posegraph/.data`，切换持久化管理器的自动保存前缀后
  调用 `/slam_toolbox/deserialize_map`。载入按原始建图起点匹配；如果车辆不在起点，
  应在地图上选择粗略位置并使用“将选中位姿设为初始位置”重新匹配。

载入操作会取消当前导航并替换后端地图，因此 GUI 会在执行前要求确认。保存到新文件属于
导出，不会改变当前自动保存目标；载入新会话则会把后续 30 秒周期保存切换到新前缀。

地图中的红点是 `/scan` 根据当前 `map -> lidar_link` 变换投影后的 MID-360 匹配结果。
“贴墙率”统计红点落在已知障碍边缘 0.15 m 范围内的比例。红点贴合黑色墙面且该比例
稳定表示当前雷达匹配合理；整体错位通常表示初始位姿或里程计存在偏差。

GUI 使用与 RViz 相同的 `/plan` 显示青色 Nav2 全局规划路径，同时订阅
`/lookahead_collision_arc` 显示黄色控制器局部前视轨迹。右侧导航状态列出两条路径的
点数和累计长度；路径不在 `map` frame 时会先通过 TF 转换后再绘制。

“路径点任务”支持依次规划和执行多个带方向的目标：

1. 在地图上按下并拖动鼠标，选择路径点的位置和车头方向。
2. 选择“添加当前选中位姿”，地图会显示紫色编号标记。
3. 使用“上移”“下移”“删除”“清空”编辑执行顺序。
4. 选择“依次执行全部路径点”。GUI 调用 Nav2 `/follow_waypoints` action；
   `waypoint_follower` 会逐点调用 `NavigateToPose`，每到一个点后再规划下一个点。
5. 列表会自动选中当前执行点，状态区显示 `当前点/总点数`；取消导航按钮也会取消整个
   路径点任务。任务结束时会列出未到达的点号。

单次任务只执行一遍，不循环。每个路径点仍经过 GUI 栅格检查、Nav2 全局规划、局部
碰撞检测和自行车模型控制；continuing 模式可使用灰色边界点，静态定位模式只允许已知
空闲区域。载入另一张地图会自动清空旧路径点，避免坐标误用。

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
`--backend ros2` 或 `--backend fastdds` 强制选择真实通信，通常保留 `auto` 即可；
`--backend demo` 始终只运行本地模拟。

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

## 6. MID-360 只读空间数据（2026-09-18）

GUI 新增“雷达与地图”页，点击“连接只读 DDS 数据”订阅 Domain 20 的真实数据。
与底盘演示后端独立，不创建车辆命令 writer；仅允许独立建图服务的 start/stop 管理。
现场已验证 NUC→Mac 的实际点云与 Qt 绘制，
默认每帧最多 1200 个二维投影点、最多 5 Hz，保留原始采集数据在车上。

| DDS String 话题 | 来源 | 当前状态 |
|---|---|---|
| `/robot320/spatial_scan` | `/filtered_points` | 已实测真实 MID-360 点云，坐标 `livox_frame` |
| `/robot320/spatial_pose` | TF `map/odom → base_link` | 等待定位栈提供；GUI 不伪造车辆轨迹 |
| `/robot320/spatial_map` | `/map` OccupancyGrid | 接口已准备，等待 SLAM 提供；压缩后传输，最多 100 万格 |

地图、点云和轨迹只在坐标系一致时叠加。无有效 TF 时保留雷达坐标，
不把原始点云标成定位或建图结果；扫描超过 2 秒未更新明确提示超时。
地图按 2 秒周期发送缓存，便于后连接客户端接收。

NUC 专用文件在 `/home/hs/robot320_remote_spatial`，来源为仓库
`scripts/robot320_onboard/spatial_bridge.py`、`spatial_sensors.launch.py`、
`start_spatial_sensors.sh`、`robot320-spatial-sensors.service`。
用户服务只启动 Livox、点云预处理、只读桥接，不启动 CAN 控制或 Nav2：

```bash
systemctl --user start robot320-spatial-sensors.service
systemctl --user stop robot320-spatial-sensors.service
journalctl --user -u robot320-spatial-sensors.service -n 50
```

传感器与只读 DDS 桥接已启用用户级开机启动，建图服务保持 disabled，仅在 GUI 请求时启动，
避免重启后自动创建无意的新地图会话。已处理旧工作空间的 `launch` Python 包遮蔽：
启动脚本将 ROS Jazzy Python 路径置前，避免更改或删除车上既有代码。
### 建图与本机保存

“雷达与地图”页已提供开始/停止独立雷达建图、保存地图到本机、打开本机地图。
建图按钮仅在车上桥接确认安装了建图管理服务之后启用；通过专用 DDS String 话题
`/robot320/mapping_control`（仅 start/stop）和 `/robot320/mapping_status` 管理，
没有车辆命令写入。请求超过 5 秒作废，服务启动脚本拒绝与已有 SLAM/EKF/URDF 节点竞争。

该独立模式复用 `robot320_localization_bringup/config/mid360_2d.lua`：
Cartographer 用雷达匹配生成 `map → base_link`，URDF 提供雷达安装 TF，
不依赖未经整合的轮式里程计，也不建立假的静态 `odom → base_link`。
它是雷达独立建图模式，不等同于之前 B9+IMU+EKF 的完整定位导航栈。

2026-09-23 已部署到新地址 `192.168.88.108`：`spatial_mapping.launch.py`、
`start_spatial_mapping.sh`、Lua 配置位于 `/home/hs/robot320_remote_spatial`，
用户服务 `robot320-spatial-mapping.service` 已安装。通过 GUI 所用的 DDS 管理话题
成功启动 Cartographer，实测 `/filtered_points` 10 Hz、地图与 pose 均到达 Mac；
地图约 611×571、0.05 m/格（地图边界会随建图更新）。

2026-09-23 长时间建图检查时，Cartographer 仍以约 10 Hz 接收点云，但不断扩大的地图已
超过桥接器 1,000,000 栅格的预览上限，桥接器会拒绝发送而不是占满 DDS 带宽。因此当前
会话的“车上建图进程运行”正常，但不能等同于“Mac 已收到最新地图”；需增加裁剪/降采样
或开始新的建图会话后再验收地图保存。

保存地图使用运行 GUI 本机的文件选择框，默认建议位置为
`~/Documents/Robot320Maps/map_日期_时间.yaml`，写出 `.yaml + .pgm`。
复用既有地图导出函数，包含分辨率、原点和占据栅格，支持重新打开本机预览；
打开本机地图不会上传机器人或改变车上定位状态。
同名 YAML/PGM 覆盖需确认；没有真实栅格地图时不能保存。
这不是 Cartographer `.pbstream` 状态导出，不能用于恢复完整 SLAM pose graph。

本地导出、回读、地图无点云时显示以及专用建图请求测试通过；GUI 进入页面后自动连接
DDS，无需再点击连接按钮。标准 trinary PGM 会将 Cartographer 概率值归类为占用、空闲、
未知，适用于 map_server/Nav2，但不是概率栅格或 pose graph 的逐值无损备份。
完整 B9+IMU+EKF 定位导航、手动底盘控制与双 USB 摄像头录像尚未完成整合。

## 7. 两台海康 USB 工业相机：公网按需静态抓图

这两台设备不是前面的海康监控/PTZ 通道。NUC 实测枚举出两台
`MV-CH100-60UC`（序列号 `DB0168357`、`DB0168290`）。GUI 新增“海康工业相机”页，
每台相机只有“获取当前图像”按钮：未点击时不打开设备、不采集帧、不上传图像；
点击后使用 MVS SDK 独占打开指定相机，自动曝光预热，编码一张 JPEG 后立即释放设备。
实测两路均返回 4096×2460、约 193 KiB 的 JPEG。当前实拍内容接近全黑，链路和 JPEG
格式正常。进一步读取原始 BayerGR8 后，两台原始像素采样均值约 1/255；自动曝光已工作，
将上限从出厂约 27.8 ms 提高到 200 ms 后仍为黑场。因此不是 Qt 缩放或公网传输造成，
需现场检查镜头盖、是否安装镜头、光圈、照明及相机安装方向。GUI 会明确显示黑场警告。

异网传输不使用 DDS 发现或局域网地址：NUC 的 Wi-Fi、SIM 两个反向 SSH 服务分别主动
连接 `8.163.54.201`，只在云主机 loopback 创建 `12222` 和 `12226`。云端 `12220`
选择器优先使用健康 Wi-Fi，失败时自动选择 SIM；Mac 再通过已有 `hsjc_ecs` SSH 配置建立
本地转发。机器人 SSH 没有暴露到公网，NUC 专用密钥在云端只允许端口转发。GUI 调用固定
脚本和相机编号，不接受任意远程命令。

相关文件：

- `scripts/robot320_onboard/hik_usb_snapshot.py`
- `scripts/robot320_onboard/robot320-cloud-reverse-tunnel.service`
- `scripts/robot320_onboard/robot320-cloud-reverse-tunnel-sim.service`
- `scripts/robot320_onboard/cloud/reverse_tunnel_selector.py`
- `remote_control/remote_control/industrial_camera_panel.py`

### GUI 异网能力边界

后续所有 GUI 功能必须声明和实现 WAN 路径，不能因“DDS 按钮存在”就认为公网可用：

| 功能 | 当前异网路径 | 状态 |
|---|---|---|
| 海康监控视频 | 公网 MediaMTX；也可手选 LAN | 已实现 |
| 海康监控 PTZ | 当前 HTTP Agent 仍偏向 LAN | 待迁移到 SSH/WAN 通道 |
| 两台 USB 工业相机静态抓图 | NUC→云双反向 SSH→选择器→Mac | Wi-Fi/SIM 均已实测 |
| 雷达、地图、轨迹 | Fast DDS Domain 20 | 同网已实测，跨公网待迁移 |
| 建图启停 | DDS 管理话题 | 同网已实测，跨公网待迁移 |
| 底盘/导航 | 当前 GUI 仍为 demo；DDS 后端仅适合同网 | 跨公网控制未授权上线 |
| 升降平台 | NUC→云双反向 SSH→选择器→Mac 持久控制流 | Wi-Fi/SIM 均以停止指令实测 |

在 WAN 通道未完成的功能上，GUI 应显示不可达并禁用操作，不自动回退为未经认证的公网接口。
计划将状态、地图、建图管理及以后授权的控制统一迁到这个出站 SSH 通道或等价的 VPN/受认证
中继；控制命令仍需保留心跳、时效、急停和服务端 allowlist。

升降页已简化为“上升、下降、停止”三个直接按钮。每次点击都通过持久 SSH 控制流发送一条
独立命令，连续点击不会合并；不再提供未接入传感器的目标高度功能。车载
`robot320-lift-serial.service` 常驻打开 `/dev/ttyUSB0`，其权限为 `0600` 的本地 Unix
Socket 只接受 `raise/lower/stop`，并将每条请求转换为一帧 RS-485 数据。GUI 经独立的
Mac 本地端口 `12224` 复用 NUC→云反向 SSH 路径，局域网和异网使用同一路径。界面上的
确认仅表示 NUC 已成功写入串口；升降设备没有位置或限位反馈，不能把它解释为机械动作确认。

## 8. 排查

| 现象 | 检查项 |
|---|---|
| Ubuntu 启动后端失败 | `source /opt/ros/jazzy/setup.bash` 后能否导入 `rclpy`、`std_msgs` |
| `FastDDSUnavailable` | Windows/macOS 确认 `import fastdds, Robot320String` 在同一个 uv Python 中成功 |
| GUI 启动但无遥测 | `ROS_DOMAIN_ID`/domain ID、同网段、防火墙、NUC gateway、多网卡路由 |
| Windows 找不到 DLL | 是否在同一终端调用 Fast DDS `setup.bat` |
| macOS 找不到 dylib | Fast DDS prefix 是否已 source，架构是否与 Python 一致 |
| 生成类型导入失败 | 重新用当前 uv Python 运行 Fast DDS-Gen 和 CMake |
