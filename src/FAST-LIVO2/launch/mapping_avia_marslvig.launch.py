#!/usr/bin/python3
# -- coding: utf-8 --
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    
    config_file_dir = os.path.join(get_package_share_directory("fast_livo"), "config")
    rviz_config_file = os.path.join(get_package_share_directory("fast_livo"), "rviz_cfg", "M300.rviz")
    
    avia_config_cmd = os.path.join(config_file_dir, "MARS_LVIG.yaml")
    camera_config_cmd = os.path.join(config_file_dir, "camera_MARS_LVIG.yaml")

    # Map save directory
    map_save_dir = os.path.join(os.path.expanduser('~'), 'fast_livo_maps')
    os.makedirs(map_save_dir, exist_ok=True)
    
    use_rviz_arg = DeclareLaunchArgument("use_rviz", default_value="True")
    avia_config_arg = DeclareLaunchArgument('avia_params_file', default_value=avia_config_cmd)
    camera_config_arg = DeclareLaunchArgument('camera_params_file', default_value=camera_config_cmd)
    
    avia_params_file = LaunchConfiguration('avia_params_file')
    camera_params_file = LaunchConfiguration('camera_params_file')
    
    return LaunchDescription([
        use_rviz_arg,
        avia_config_arg,
        camera_config_arg,
        
        Node(
            package='demo_nodes_cpp',
            executable='parameter_blackboard',
            name='parameter_blackboard',
            parameters=[camera_params_file, {'use_sim_time': True}],
            output='screen'
        ),
        
        Node(
            package="image_transport",
            executable="republish",
            name="republish",
            arguments=['compressed', 'raw'],
            remappings=[
                ('in/compressed', '/left_camera/image/compressed'),
                ('out', '/left_camera/image_decompressed'),
            ],
            parameters=[{'use_sim_time': True}],
            output="screen",
        ),
        
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package="fast_livo",
                    executable="fastlivo_mapping",
                    name="laserMapping",
                    parameters=[
                        avia_params_file,
                        {
                            'use_sim_time': True,
                            # Enable PCD map saving
                            'pcd_save.pcd_save_en': True,
                            'pcd_save.interval': -1,        # -1 = one single merged file
                            'pcd_save.filter_size_pcd': 0.15,
                            'pcd_save.type': 0,             # 0 = colored point cloud
                            # Enable trajectory output
                            'evo.pose_output_en': True,
                        }
                    ],
                    output="screen",
                    respawn=False,
                ),
            ]
        ),
        
        Node(
            condition=IfCondition(LaunchConfiguration("use_rviz")),
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config_file],
            output="screen"
        ),
    ])