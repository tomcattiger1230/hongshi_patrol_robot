# robot320_interfaces

机器人 NUC 与 `remote_control` 共用的 ROS-independent 语义模型和 Fast DDS IDL。
该包本身不导入 `rclpy`，可通过 colcon 安装，也可在无 ROS 2 的电脑上使用 pip 安装。

正式 DDS topic：

| Topic | 类型 | 方向 |
|---|---|---|
| `robot320/command` | `Robot320CommandEnvelope` | 上位机 → NUC |
| `robot320/state` | `Robot320StateEnvelope` | NUC → 上位机 |
| `robot320/reply` | `Robot320ReplyEnvelope` | NUC → 上位机 |
| `robot320/heartbeat` | `Robot320HeartbeatEnvelope` | 双向 |

IDL envelope 保存身份、序列号和时间戳；具体业务字段使用版本兼容的 JSON 语义消息。
这允许后续增加升降杆或电池字段，而不必每次破坏 DDS 二进制类型兼容性。

生成目标平台的 Python 类型：

```bash
sudo apt install swig libpython3-dev
./robot320_interfaces/scripts/generate_fastdds_types.sh
export PYTHONPATH="$PWD/robot320_interfaces/generated/Robot320Dds/build:$PYTHONPATH"
```

生成类型与 Fast DDS Python bindings 都必须针对运行电脑的操作系统和 Python 版本构建。
