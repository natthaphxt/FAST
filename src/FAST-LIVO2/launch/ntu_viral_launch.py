#!/usr/bin/env python3
"""
FAST-LIVO2 Launch File for NTU-VIRAL Dataset

This launch file starts:
1. Parameter blackboard node (provides camera calibration parameters)
2. FAST-LIVO2 mapping node
3. RViz2 for visualization

Usage:
    ros2 launch fast_livo ntu_viral_complete.launch.py
    
Then in another terminal:
    ros2 bag play ~/NTU_dataset/eee_01/eee_01_ros2/ --clock --rate 0.5
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
import os

def generate_launch_description():
    
    # Path to config files
    config_file = os.path.expanduser('~/fast_ws/src/FAST-LIVO2/config/NTU_VIRAL.yaml')
    rviz_config = os.path.expanduser('~/fast_ws/src/FAST-LIVO2/rviz_cfg/ntu_viral.rviz')
    
    # Alternative: use installed config if available
    # from ament_index_python.packages import get_package_share_directory
    # pkg_share = get_package_share_directory('fast_livo')
    # config_file = os.path.join(pkg_share, 'config', 'NTU_VIRAL.yaml')
    
    # 1. Start parameter blackboard node (provides camera calibration)
    param_server = Node(
        package='demo_nodes_cpp',
        executable='parameter_blackboard',
        name='parameter_blackboard',
        output='screen',
        parameters=[{
            'cam_model': 'Pinhole',
            'cam_width': 752,
            'cam_height': 480,
            'scale': 1.0,
            'cam_fx': 425.0258563372763,
            'cam_fy': 426.7976260903337,
            'cam_cx': 386.0151866550880,
            'cam_cy': 241.9130336743440,
            'cam_d0': -0.288105327549552,
            'cam_d1': 0.074578284234601,
            'cam_d2': 0.0007784489598138802,
            'cam_d3': -0.0002277853975035461,
        }]
    )
    
    # 2. FAST-LIVO2 mapping node (starts after parameter server)
    fast_livo_node = Node(
        package='fast_livo',
        executable='fastlivo_mapping',
        name='fastlivo_mapping',
        output='screen',
        parameters=[config_file],
    )
    
    # 3. RViz2 for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
    )
    
    # Use TimerAction to delay FAST-LIVO2 start (gives parameter server time to initialize)
    delayed_fast_livo = TimerAction(
        period=5.0,  # 5 second delay — gives parameter_blackboard time on slow start
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