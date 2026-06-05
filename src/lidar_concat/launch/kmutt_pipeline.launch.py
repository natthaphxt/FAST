from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    config_file = LaunchConfiguration('config_file')
    publish_rate = LaunchConfiguration('publish_rate')
    max_age_sec = LaunchConfiguration('max_age_sec')

    fast_lio_launch = PathJoinSubstitution([
        FindPackageShare('fast_lio'), 'launch', 'mapping.launch.py',
    ])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('config_file', default_value='kmutt.yaml'),
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

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(fast_lio_launch),
            launch_arguments={
                'config_file': config_file,
                'use_sim_time': use_sim_time,
            }.items(),
        ),
    ])
