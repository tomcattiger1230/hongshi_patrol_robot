#!/bin/bash

# ============================================================
# Robot320 完整导航系统 + EKF（第一阶段）
# 启动顺序：URDF → Livox → 点云 → CAN反馈 → wheel odom → EKF → Cartographer → Nav2 → RViz
# TF：Cartographer 发布 map→odom，EKF 发布 odom→base_link，URDF 发布 base_link→livox_frame
# 安全：本脚本只启动 CAN 反馈，不启动 cmd_vel_to_ackermann_can.py 控制发送节点。
# ============================================================

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

WORKSPACE="/home/hs/roboracer_ws"
CONFIG_DIR="/home/hs/cartographer_config"
MAP_PBSTREAM="/home/hs/script/my_map_outdoor-all.pbstream"
MAP_FILE="/home/hs/my_map_outdoor-all.yaml"
NAV2_CONFIG="/home/hs/roboracer_ws/config/nav2/nav2_params_robot320.yaml"
EKF_CONFIG="/home/hs/robot_localization_params.yaml"

# gnome-terminal starts child shells; export paths so every child receives them.
export WORKSPACE CONFIG_DIR MAP_PBSTREAM MAP_FILE NAV2_CONFIG EKF_CONFIG

print_header() { echo ""; echo "=========================================="; echo "  $1"; echo "=========================================="; }
print_step() { echo -e "${CYAN}▶ $1${NC}"; }
print_success() { echo -e "${GREEN}✅ $1${NC}"; }

source /opt/ros/jazzy/setup.bash
source "$WORKSPACE/install/setup.bash" 2>/dev/null || true
export PYTHONPATH=/opt/ros/jazzy/lib/python3.12/site-packages:$PYTHONPATH

print_header "Robot320 完整系统（EKF 第一阶段）"

print_step "清理旧进程..."
pkill -9 -f static_transform_publisher 2>/dev/null || true
pkill -9 -f cartographer_node 2>/dev/null || true
pkill -9 -f cartographer_occupancy_grid_node 2>/dev/null || true
pkill -9 -f mid360_preprocess 2>/dev/null || true
pkill -9 -f livox_ros_driver2 2>/dev/null || true
pkill -9 -f cmd_vel_to_can 2>/dev/null || true
pkill -9 -f cmd_vel_to_ackermann_can 2>/dev/null || true
pkill -9 -f can_receiver 2>/dev/null || true
pkill -9 -f can_command_receiver 2>/dev/null || true
pkill -9 -f robot_state_publisher 2>/dev/null || true
pkill -9 -f ekf_node 2>/dev/null || true
pkill -9 -f can_to_odom 2>/dev/null || true
pkill -9 -f rviz2 2>/dev/null || true
pkill -9 -f nav2 2>/dev/null || true
pkill -9 -f lidar_remap 2>/dev/null || true
sleep 3
print_success "清理完成"

print_step "1/11 启动 URDF..."
gnome-terminal --title="1-URDF" -- bash -c "
  source /opt/ros/jazzy/setup.bash; source \$WORKSPACE/install/setup.bash 2>/dev/null || true
  ros2 launch robot320_description robot_state_publisher.launch.py; exec bash
" &
sleep 3

print_step "2/11 启动 Livox 雷达..."
gnome-terminal --title="2-Livox" -- bash -c "
  source /opt/ros/jazzy/setup.bash; source \$WORKSPACE/install/setup.bash 2>/dev/null || true
  ros2 launch livox_ros_driver2 msg_MID360s_launch.py; exec bash
" &
sleep 5

print_step "3/11 启动点云预处理..."
gnome-terminal --title="3-点云预处理" -- bash -c "
  source /opt/ros/jazzy/setup.bash; source \$WORKSPACE/install/setup.bash 2>/dev/null || true
  ros2 run mid360_preprocess mid360_preprocess_node; exec bash
" &
sleep 3

print_step "4/11 启动雷达话题转发..."
gnome-terminal --title="4-雷达转发" -- bash -c "
  source /opt/ros/jazzy/setup.bash; source \$WORKSPACE/install/setup.bash 2>/dev/null || true
  ros2 run robot320_utils lidar_remap; exec bash
" &
sleep 2

print_step "5/11 启动 CAN 反馈节点（不发送控制）..."
gnome-terminal --title="5-CAN反馈" -- bash -c "
  source /opt/ros/jazzy/setup.bash; source \$WORKSPACE/install/setup.bash 2>/dev/null || true
  export PYTHONPATH=/opt/ros/jazzy/lib/python3.12/site-packages:\$PYTHONPATH
  python3 \$WORKSPACE/src/robot320_bringup/scripts/can_command_receiver.py; exec bash
" &
sleep 5

print_step "6/11 启动原始轮式里程计..."
gnome-terminal --title="6-轮式里程计" -- bash -c "
  source /opt/ros/jazzy/setup.bash; source \$WORKSPACE/install/setup.bash 2>/dev/null || true
  export PYTHONPATH=/opt/ros/jazzy/lib/python3.12/site-packages:\$PYTHONPATH
  echo '输入: /can/actual_speed, /can/actual_steering'
  echo '输出: /wheel/odom_raw（不发布TF）'
  python3 \$WORKSPACE/src/robot320_bringup/scripts/can_to_odom.py; exec bash
" &
sleep 5

print_step "7/12 启动 IMU 修正节点..."
gnome-terminal --title="7-IMU修正" -- bash -c "
  source /opt/ros/jazzy/setup.bash; source \$WORKSPACE/install/setup.bash 2>/dev/null || true
  export PYTHONPATH=/opt/ros/jazzy/lib/python3.12/site-packages:\$PYTHONPATH
  python3 \$WORKSPACE/src/robot320_bringup/scripts/imu_covariance_filter.py; exec bash
" &
sleep 3

print_step "8/12 启动 EKF（融合 /wheel/odom_raw + 修正后的 gyro_z）..."
gnome-terminal --title="7-EKF" -- bash -c "
  source /opt/ros/jazzy/setup.bash; source \$WORKSPACE/install/setup.bash 2>/dev/null || true
  export PYTHONPATH=/opt/ros/jazzy/lib/python3.12/site-packages:\$PYTHONPATH
  echo '输入: /wheel/odom_raw'
  echo '输出: /odometry/filtered，odom→base_link'
  ros2 run robot_localization ekf_node --ros-args --params-file \$EKF_CONFIG; exec bash
" &
sleep 8

print_step "9/12 启动 Cartographer..."
gnome-terminal --title="8-Cartographer" -- bash -c "
  source /opt/ros/jazzy/setup.bash; source \$WORKSPACE/install/setup.bash 2>/dev/null || true
  export PYTHONPATH=/opt/ros/jazzy/lib/python3.12/site-packages:\$PYTHONPATH
  ros2 run cartographer_ros cartographer_node \
    -configuration_directory \$CONFIG_DIR \
    -configuration_basename mid360_2d.lua \
    -load_state_filename \$MAP_PBSTREAM \
    --ros-args -r points2:=/filtered_points; exec bash
" &
sleep 12

print_step "10/12 启动栅格地图..."
gnome-terminal --title="9-地图" -- bash -c "
  source /opt/ros/jazzy/setup.bash; source \$WORKSPACE/install/setup.bash 2>/dev/null || true
  ros2 run cartographer_ros cartographer_occupancy_grid_node -resolution 0.05; exec bash
" &
sleep 3

print_step "11/12 启动 Nav2..."
gnome-terminal --title="10-Nav2" -- bash -c "
  source /opt/ros/jazzy/setup.bash; source \$WORKSPACE/install/setup.bash 2>/dev/null || true
  ros2 launch nav2_bringup navigation_launch.py \
    map:=\$MAP_FILE params_file:=\$NAV2_CONFIG use_sim_time:=False autostart:=True; exec bash
" &
sleep 8

print_step "启用 collision monitor 安全输出..."
# The lifecycle node can report active while its global output gate remains
# disabled. Explicitly enable it and fail the startup if the service cannot be
# reached, so /cmd_vel_safe is never silently absent.
COLLISION_TOGGLE_OUTPUT=$(timeout 30 ros2 service call \
  /collision_monitor/toggle nav2_msgs/srv/Toggle "{enable: true}")
echo "$COLLISION_TOGGLE_OUTPUT"
if ! grep -q "success=True" <<<"$COLLISION_TOGGLE_OUTPUT"; then
  echo -e "${RED}❌ collision monitor 未成功启用，不允许启动底盘控制${NC}"
  exit 1
fi
print_success "collision monitor 安全输出已启用"

print_step "12/12 启动 RViz..."
gnome-terminal --title="11-RViz" -- bash -c "
  source /opt/ros/jazzy/setup.bash; source \$WORKSPACE/install/setup.bash 2>/dev/null || true
  rviz2; exec bash
" &
sleep 3

print_header "系统启动命令已发出"
echo ""
echo "📌 验证命令："
echo "  ros2 topic hz /wheel/odom_raw"
echo "  ros2 topic echo /odometry/filtered --once"
echo "  ros2 run tf2_ros tf2_echo odom base_link"
echo "  ros2 run tf2_ros tf2_echo map odom"
echo "  ros2 topic echo /tracked_pose --once"
echo ""
echo "⚠️ 注意：当前未启动 cmd_vel_to_ackermann_can.py（控制发送节点）"
echo "=========================================="
