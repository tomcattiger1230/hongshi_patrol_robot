#!/bin/bash
echo "🛑 停止所有 Robot320 节点..."
pkill -9 -f cartographer_node 2>/dev/null
pkill -9 -f cartographer_occupancy_grid_node 2>/dev/null
pkill -9 -f livox_ros_driver2 2>/dev/null
pkill -9 -f mid360_preprocess 2>/dev/null
pkill -9 -f robot_state_publisher 2>/dev/null
pkill -9 -f ekf_node 2>/dev/null
pkill -9 -f can_to_odom 2>/dev/null
pkill -9 -f cmd_vel_to_can 2>/dev/null
pkill -9 -f cmd_vel_to_ackermann_can 2>/dev/null
pkill -9 -f can_receiver 2>/dev/null
pkill -9 -f can_command_receiver 2>/dev/null
pkill -9 -f lidar_remap 2>/dev/null
pkill -9 -f nav2 2>/dev/null
pkill -9 -f rviz2 2>/dev/null
echo "✅ 所有节点已停止"
