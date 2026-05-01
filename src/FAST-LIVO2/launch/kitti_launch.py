#!/usr/bin/env python3
"""
FAST-LIVO2 Launch File for KITTI Dataset - FIXED VERSION
Forces lidar_type parameter to ensure Velodyne PointCloud2 subscription

This launch file starts:
1. Parameter blackboard node (provides camera calibration parameters)
2. FAST-LIVO2 mapping node with EXPLICIT lidar_type override
3. RViz2 for visualization

Usage:
    ros2 launch fast_livo kitti_launch_fixed.py
    
Then in another terminal:
    ros2 bag play ~/Downloads/kitti_drive_0071.mcap --clock
"""
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
import os


def generate_launch_description():
    
    # Path to config files
    config_file = os.path.expanduser('~/fast_ws/src/FAST-LIVO2/config/kitti.yaml')
    rviz_config = os.path.expanduser('~/fast_ws/src/FAST-LIVO2/rviz_cfg/kitti.rviz')
    
    # 1. Start parameter blackboard node (provides camera calibration)
    # KITTI cam2 (left color camera) parameters
    param_server = Node(
        package='demo_nodes_cpp',
        executable='parameter_blackboard',
        name='parameter_blackboard',
        output='screen',
        parameters=[{
            'cam_model': 'Pinhole',
            'cam_width': 1238,
            'cam_height': 374,
            'scale': 1.0,
            'cam_fx': 721.5377,
            'cam_fy': 721.5377,
            'cam_cx': 609.5593,
            'cam_cy': 172.854,
            'cam_d0': 0.0,  # KITTI images are rectified
            'cam_d1': 0.0,
            'cam_d2': 0.0,
            'cam_d3': 0.0,
        }]
    )
    
    # 2. FAST-LIVO2 mapping node with EXPLICIT parameter override
    # This forces lidar_type to be 1 (Velodyne) even if config file doesn't work
    fast_livo_node = Node(
        package='fast_livo',
        executable='fastlivo_mapping',
        name='fastlivo_mapping',
        output='screen',
        parameters=[
            config_file,
            {
                'use_sim_time': True,
                'preprocess.lidar_type': 3,      
                'common.lid_topic': '/kitti/velodyne_points', 
                'common.imu_topic': '/kitti/imu',
                'common.img_topic': '/kitti/camera_color_left/image_raw',
            }
        ],
    )
    
    # 3. RViz2 for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        parameters=[{'use_sim_time': True}],
    )
    
    # Use TimerAction to delay FAST-LIVO2 start (gives parameter server time to initialize)
    delayed_fast_livo = TimerAction(
        period=2.0,  # 2 second delay
        actions=[fast_livo_node]
    )
    
    return LaunchDescription([
        # Start parameter server immediately
        param_server,
        
        # Start FAST-LIVO2 after 2 second delay
        delayed_fast_livo,
        
        # Start RViz2 after 2 second delay  
        TimerAction(
            period=2.0,
            actions=[rviz_node]
        ),
    ])