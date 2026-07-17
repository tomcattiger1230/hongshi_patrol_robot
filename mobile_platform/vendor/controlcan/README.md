# ControlCAN Linux runtime

此目录只保存 Robot320 运行所需的最小厂商 runtime 和头文件。完整安装包、工具和手册不
进入 git。

| 目录 | 目标平台 |
|---|---|
| `linux-x86_64` | x86_64 / amd64 Linux |
| `linux-x86` | 32 位 x86 Linux |
| `linux-aarch64` | 64 位 ARM Linux |
| `linux-armv7` | 32 位 ARM Linux |
| `include` | `controlcan.h` |

`ControlCANAdapter` 的查找顺序：显式 `library_path`、`CONTROL_CAN_LIB`、当前目录的
`libcontrolcan.so`、最后是与 `platform.machine()` 匹配的本目录 runtime。

这些库仅用于 Linux NUC，不用于 Windows/macOS 上位机。设备权限、固件和 CAN 波特率
以厂商手册及现场硬件为准。
