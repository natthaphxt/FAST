from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    publish_rate = LaunchConfiguration('publish_rate')
    max_age_sec = LaunchConfiguration('max_age_sec')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('publish_rate', default_value='10.0'),
        DeclareLaunchArgument('max_age_sec', default_value='0.5'),
        Node(
            package='lidar_concat',
            executable='concat_node',
            name='lidar_concat',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'publish_rate': publish_rate,
                'max_age_sec': max_age_sec,
            }],
        ),
    ])
