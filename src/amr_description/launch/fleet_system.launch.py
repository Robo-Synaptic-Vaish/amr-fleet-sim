#!/usr/bin/env python3
"""
Fleet System Launch Description (Stage 3.3 & Dashboard)
-------------------------------------------------------
Launches:
  1. AMR 1 Decentralized Safety & State Node (Priority 1)
  2. AMR 2 Decentralized Safety & State Node (Priority 2 / Yielding)
  3. Live Web Dashboard & SSE Bridge Node on http://localhost:8080
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # AMR 1 Safety & State Node (Priority 1: Right-of-Way)
        Node(
            package='amr_description',
            executable='fleet_safety_node.py',
            name='fleet_safety_amr',
            parameters=[{
                'robot_id': 'amr',
                'peer_id': 'amr2',
                'odom_topic': '/model/amr/odometry',
                'offset_x': -2.0,
                'offset_y': 0.8,
                'cmd_vel_in': '/model/amr/cmd_vel_raw',
                'cmd_vel_out': '/model/amr/cmd_vel',
                'priority': 1,
                'caution_distance': 2.0,
                'stop_distance': 1.35,
                'recovery_distance': 1.65,
                'comm_timeout': 1.5
            }],
            output='screen'
        ),

        # AMR 2 Safety & State Node (Priority 2: Yielding)
        Node(
            package='amr_description',
            executable='fleet_safety_node.py',
            name='fleet_safety_amr2',
            parameters=[{
                'robot_id': 'amr2',
                'peer_id': 'amr',
                'odom_topic': '/model/amr2/odometry',
                'offset_x': -2.0,
                'offset_y': -0.8,
                'cmd_vel_in': '/model/amr2/cmd_vel_raw',
                'cmd_vel_out': '/model/amr2/cmd_vel',
                'priority': 2,
                'caution_distance': 2.0,
                'stop_distance': 1.35,
                'recovery_distance': 1.65,
                'comm_timeout': 1.5
            }],
            output='screen'
        ),

        # Real-time Web Dashboard & SSE Bridge (http://localhost:8080)
        Node(
            package='amr_description',
            executable='fleet_dashboard_node.py',
            name='fleet_dashboard_node',
            parameters=[{
                'port': 8080,
                'host': '0.0.0.0',
                'stale_timeout': 1.5
            }],
            output='screen'
        )
    ])
