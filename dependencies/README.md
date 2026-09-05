# ROS 2 dependency strategy

Robot320 keeps application code on one shared branch. ROS distribution-specific source
dependencies live in pinned `*.repos` manifests, while system packages remain managed by
`rosdep`/APT. This is preferable to Git submodules for a ROS workspace because:

- one upstream package can require a different branch or commit for each ROS distribution;
- `vcs import` is idempotent and understands the standard ROS repository manifest format;
- generated `build/`, `install/`, and native Python artifacts never cross distro or OS boundaries.

For Ubuntu 26.04 and ROS 2 Lyrical, run from the repository:

```bash
./scripts/setup_ros_workspace.sh lyrical
source ./scripts/source_robot320.sh lyrical
```

The pinned navigation sources are imported into
`<workspace>/.ros-deps/lyrical/navigation_ws`. The directory is intentionally ignored by Git;
the reproducible inputs are `lyrical_navigation.repos` and the patches in this directory.
Livox SDK2 is pinned separately in `lyrical_native.repos` and installed without `sudo` into
`<workspace>/.ros-deps/lyrical/native_ws/install`.

For another supported distribution, add a separate pinned manifest and a matching case in
`scripts/setup_ros_workspace.sh`. Do not reuse an `install/` tree produced by another ROS
distribution, Ubuntu release, CPU architecture, or Python ABI.

The macOS standalone DDS toolchain follows the same model: `macos_fastdds.repos` pins the
eProsima repositories and `setup_macos_fastdds.sh` keeps their source/build/install trees in
`~/Develop/fastdds-python`. They are not ROS workspace packages and must not be copied to
Ubuntu.
