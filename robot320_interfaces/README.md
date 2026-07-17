# robot320_interfaces

机器人 NUC 与 `remote_control` 共用的 ROS-independent 语义模型和 Fast DDS IDL。
该包本身不导入 `rclpy`，由根目录 uv 项目在 NUC 和无 ROS 2 的上位机中统一安装。

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
FASTDDS_SETUP=/path/to/Fast-DDS-python/install/setup.bash \
  ./scripts/uv_run.sh desktop ./robot320_interfaces/scripts/generate_fastdds_types.sh
```

`uv_run.sh` 会自动把默认生成目录加入 `PYTHONPATH`；自定义输出目录时设置
`ROBOT320_DDS_TYPES=/path/to/build`。NUC 上把 profile 从 `desktop` 改成 `nuc`。

生成类型与 Fast DDS Python bindings 都必须针对运行电脑的操作系统和 Python 版本构建；
`uv_setup.sh --python` 选择的解释器也必须使用相同 ABI。

实现依据：

- [Fast DDS Python publisher/subscriber](https://fast-dds.docs.eprosima.com/en/stable/fastdds/getting_started/simple_python_app/simple_python_app.html)
- [Fast DDS-Gen Python bindings](https://fast-dds.docs.eprosima.com/en/2.x/fastddsgen/python_bindings/python_bindings.html)
