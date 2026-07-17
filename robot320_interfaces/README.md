# robot320_interfaces

NUC 和上位机共用的 JSON 消息与 ROS 2 兼容 wire contract。本包本身不导入 `rclpy`，
由根目录 uv 项目以 editable 方式安装。

## ROS 2 / DDS topic

| ROS 2 Topic | DDS Topic | 内容 | 方向 |
|---|---|---|---|
| `/robot320/command` | `rt/robot320/command` | 手动、导航、安全和升降杆指令 | 上位机 → NUC |
| `/robot320/state` | `rt/robot320/state` | 底盘、位姿、导航、升降杆、电池和故障 | NUC → 上位机 |
| `/robot320/reply` | `rt/robot320/reply` | 指令处理结果 | NUC → 上位机 |
| `/robot320/heartbeat` | `rt/robot320/heartbeat` | 节点身份、角色、序列号和时间戳 | 双向 |

所有 Topic 类型均为 ROS 2 `std_msgs/msg/String`，业务字段为 JSON。NUC 直接用 `rclpy`；
standalone Fast DDS 上位机使用相同的 DDS 类型名 `std_msgs::msg::dds_::String_`。

## 生成 Python 类型

只有 Windows/macOS 等非 Ubuntu 上位机需要额外安装 Fast DDS C++ runtime、
Fast-DDS-python 和 Fast DDS-Gen，具体路径见
[`remote_control/README.md`](../remote_control/README.md)。Ubuntu 设备直接使用 ROS 2，
不执行本节。非 Ubuntu 上位机生成命令：

```bash
./scripts/setup_fastdds.sh
```

默认输出为 `robot320_interfaces/generated/Robot320String/build`，`uv_run.sh` 会自动加入
`PYTHONPATH`。自定义目录时设置 `ROBOT320_DDS_TYPES=/path/to/build`。

以下三者必须使用相同操作系统、CPU 架构和 Python ABI：

- `fastdds` Python binding
- `Robot320String` 生成 module
- uv 环境的 Python

IDL 位于 `robot320_interfaces/robot320_interfaces/dds/Robot320String.idl`。实现参考
[Fast DDS Python 示例](https://fast-dds.docs.eprosima.com/en/stable/fastdds/getting_started/simple_python_app/simple_python_app.html)
和 [Fast DDS-Gen](https://fast-dds.docs.eprosima.com/en/stable/fastddsgen/introduction/introduction.html)。
