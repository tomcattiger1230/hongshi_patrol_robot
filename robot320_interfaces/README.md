# robot320_interfaces

NUC 和上位机共用的 ROS-independent 消息与 Fast DDS wire contract。本包不导入
`rclpy`，由根目录 uv 项目在两端以 editable 方式安装。

## Fast DDS topic

| Topic | 内容 | 方向 |
|---|---|---|
| `robot320/command` | 手动、导航、安全和升降杆指令 | 上位机 → NUC |
| `robot320/state` | 底盘、位姿、导航、升降杆、电池和故障 | NUC → 上位机 |
| `robot320/reply` | accepted/completed/rejected/failed | NUC → 上位机 |
| `robot320/heartbeat` | 节点身份、角色、序列号和时间戳 | 双向 |

IDL envelope 保存标识、序列号和时间戳；业务字段使用 JSON，使新增遥测字段时不必破坏
DDS envelope 的二进制类型。

## 生成 Python 类型

上位机需要安装 Fast DDS C++ runtime、Fast-DDS-python 和 Fast DDS-Gen，Windows/macOS
具体路径见 [`remote_control/README.md`](../remote_control/README.md)。NUC 镜像默认随
ROS 2 提供完整通讯环境，不执行本节安装步骤。上位机生成命令：

```bash
./scripts/setup_fastdds.sh
```

默认输出为 `robot320_interfaces/generated/Robot320Dds/build`，`uv_run.sh` 会自动加入
`PYTHONPATH`。自定义目录时设置 `ROBOT320_DDS_TYPES=/path/to/build`。

以下三者必须使用相同操作系统、CPU 架构和 Python ABI：

- `fastdds` Python binding
- `Robot320Dds` 生成 module
- uv 环境的 Python

IDL 位于 `robot320_interfaces/robot320_interfaces/dds/Robot320Dds.idl`。实现参考
[Fast DDS Python 示例](https://fast-dds.docs.eprosima.com/en/stable/fastdds/getting_started/simple_python_app/simple_python_app.html)
和 [Fast DDS-Gen](https://fast-dds.docs.eprosima.com/en/stable/fastddsgen/introduction/introduction.html)。
