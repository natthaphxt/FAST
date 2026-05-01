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
    rviz_config_file = os.path.join(get_package_share_directory("fast_livo"), "rviz_cfg", "fast_livo2.rviz")

    main_config = os.path.join(config_file_dir, "HKU_landmark.yaml")
    camera_config = os.path.join(config_file_dir, "camera_pinhole.yaml")

    use_rviz_arg = DeclareLaunchArgument("use_rviz", default_value="True")
    main_config_arg = DeclareLaunchArgument("main_params_file", default_value=main_config)
    camera_config_arg = DeclareLaunchArgument("camera_params_file", default_value=camera_config)

    main_params_file = LaunchConfiguration("main_params_file")
    camera_params_file = LaunchConfiguration("camera_params_file")

    return LaunchDescription([
        use_rviz_arg,
        main_config_arg,
        camera_config_arg,

        Node(
            package="demo_nodes_cpp",
            executable="parameter_blackboard",
            name="parameter_blackboard",
            parameters=[camera_params_file, {"use_sim_time": True}],
            output="screen",
        ),

        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package="fast_livo",
                    executable="fastlivo_mapping",
                    name="laserMapping",
                    parameters=[
                        main_params_file,
                        {"use_sim_time": True},
                    ],
                    output="screen",
                    respawn=False,
                ),
            ],
        ),

        Node(
            condition=IfCondition(LaunchConfiguration("use_rviz")),
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config_file],
            output="screen",
        ),
    ])
