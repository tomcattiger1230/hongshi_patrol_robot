# Mac–Ubuntu DDS LAN setup

## Chosen topology

The Ubuntu robot/simulator runs normal ROS 2 nodes with `rmw_fastrtps_cpp`. The macOS
controller uses standalone Fast DDS and a generated wire-compatible
`std_msgs/msg/String` type. Both sides use ROS domain `20` and the standard ROS DDS topic
names (`rt/robot320/command`, `rt/robot320/state`, `rt/robot320/reply`, and
`rt/robot320/heartbeat`). No custom UDP bridge is involved.

This keeps ROS native on Ubuntu while avoiding an unsupported ROS 2 installation on macOS.
The macOS standalone backend carries Robot320 JSON control and telemetry; full map/TF/Nav2
action support remains an Ubuntu ROS 2 feature.

## Ubuntu

```bash
cd ~/Develop/ROS_ws/patrol_robot/src/hongshi_patrol_robot
source ./scripts/source_robot320.sh lyrical
source ./scripts/source_dds_lan.sh 192.168.0.114
```

The first command selects Fast DDS RMW, domain 20, and the correct Lyrical overlays. The
second restricts Fast DDS to the LAN interface selected by the route to the Mac.

## macOS

```bash
cd ~/Develop/github_ws/hongshi_patrol_robot
./scripts/setup_macos_fastdds.sh
source ./scripts/source_dds_lan.sh 192.168.0.218
./scripts/uv_run.sh desktop robot320_remote_gui --backend fastdds --domain-id 20
```

The Fast DDS toolchain and runtime are stored under `~/Develop/fastdds-python`. The generated
Robot320 type module is stored under the project and added to `PYTHONPATH` automatically by
`uv_run.sh`.

## Discovery checklist

- Both computers must be on the same LAN and use the same `ROS_DOMAIN_ID`.
- Disable VPNs during diagnosis; a VPN can change multicast routing or become the advertised
  DDS interface.
- Allow UDP multicast and the DDS/RTPS UDP ports in the Ubuntu firewall. For domain 20 the
  well-known discovery ports start around UDP 12400; allowing UDP 12400–12500 between the
  two LAN addresses is a practical narrow diagnostic rule.
- Do not run the simulator with a different RMW/domain in a previously sourced terminal.
- `FASTDDS_DEFAULT_PROFILES_FILE` should point to the generated profile and contain the
  current LAN address, not a Docker/VPN address.

Fast DDS discovery and payloads are not authenticated or encrypted by this configuration.
Use it only on a trusted LAN; use a VPN or DDS Security/SROS2 before crossing an untrusted
network.
