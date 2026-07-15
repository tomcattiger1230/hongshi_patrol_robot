# ControlCAN Linux Runtime Libraries

This directory contains the small runtime subset extracted from the Linux CAN
driver package in `mobile_platform/driver/can_linux.rar`.

`ControlCANAdapter` selects the library automatically on Linux:

| Directory | Target |
|---|---|
| `linux-x86_64/` | x86_64 / amd64 Linux |
| `linux-x86/` | 32-bit x86 Linux |
| `linux-aarch64/` | 64-bit ARM Linux |
| `linux-armv7/` | 32-bit ARM Linux |
| `include/` | vendor `controlcan.h` |

Search order at runtime:

1. Explicit `--lib` / `CANAdapterConfig.library_path`
2. `CONTROL_CAN_LIB`
3. `./libcontrolcan.so`
4. Bundled library matching `platform.machine()`

The full vendor archive and manuals are intentionally kept under
`mobile_platform/driver/`, which is ignored by git because it is large.
