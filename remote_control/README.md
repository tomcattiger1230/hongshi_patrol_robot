# Robot320 上位机

上位机正式入口是 PySide6 GUI。它通过 Fast DDS 与 NUC 通信，不导入 `rclpy`，可运行在
Windows、macOS 或 Linux。

GUI 支持：

- 按住持续发送的前进、后退和转向，松开立即停车
- 停止、刹车、急停和解除急停
- Nav2 目标发送、取消、状态和进度
- 升降杆动作和目标高度
- 底盘、SLAM 位姿、升降杆、电池、故障和指令应答

## 1. Python 环境

仓库的 `desktop` profile 安装 `robot320_interfaces`、`remote_control` 和 PySide6：

```bash
./scripts/uv_setup.sh desktop --python 3.12
```

这里的 Python 版本只是示例，必须与随后构建 Fast DDS Python binding 时使用的解释器
完全一致。项目需要三个 native 层全部匹配：

1. Fast DDS / Fast CDR C++ runtime
2. 提供 `import fastdds` 的 Fast-DDS-python binding
3. 由项目 IDL 生成的 `Robot320Dds` Python module

Fast DDS 本身不是 PyPI 包，不能只靠 `uv sync` 安装。

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
mkdir robot320_interfaces\generated\Robot320Dds
cd robot320_interfaces\generated\Robot320Dds
fastddsgen.bat -python -replace ..\..\robot320_interfaces\dds\Robot320Dds.idl
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

构建完成后回到仓库：

```bash
cd "$ROBOT320_REPO"
FASTDDS_SETUP=/path/to/fastdds-python/install/setup.bash \
  ./scripts/uv_run.sh desktop \
  ./robot320_interfaces/scripts/generate_fastdds_types.sh
FASTDDS_SETUP=/path/to/fastdds-python/install/setup.bash \
  ./scripts/uv_run.sh desktop robot320_remote_gui --domain-id 20
```

## 4. 使用 GUI

```bash
FASTDDS_SETUP=/path/to/Fast-DDS-python/install/setup.bash \
  ./scripts/uv_run.sh desktop robot320_remote_gui \
  --domain-id 20 --client-id operator-laptop
```

Windows 中先 `call install\setup.bat`，再直接执行对应的 `uv run --locked ...` 命令。
NUC 与上位机的 domain ID 必须一致，默认均为 `20`。

## 5. Python API

GUI 和其他应用复用同一个 ROS-independent 客户端：

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
| `FastDDSUnavailable` | `import fastdds` 和 `import Robot320Dds` 是否在同一个 uv Python 中成功 |
| GUI 启动但无遥测 | domain ID、同网段、防火墙、NUC gateway、多网卡路由 |
| Windows 找不到 DLL | 是否在同一终端调用 Fast DDS `setup.bat` |
| macOS 找不到 dylib | Fast DDS prefix 是否已 source，架构是否与 Python 一致 |
| 生成类型导入失败 | 重新用当前 uv Python 运行 Fast DDS-Gen 和 CMake |
