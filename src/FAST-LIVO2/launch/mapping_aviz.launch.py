import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    
    # 1. ค้นหา Path ของไฟล์ Config
    package_path = get_package_share_directory("fast_livo")
    config_file_dir = os.path.join(package_path, "config")
    rviz_config_file = os.path.join(package_path, "rviz_cfg", "fast_livo2.rviz")

    # 2. กำหนดไฟล์ Parameter เริ่มต้น (สำหรับ Hilti)
    avia_config_cmd = os.path.join(config_file_dir, "avia.yaml")
    camera_config_cmd = os.path.join(config_file_dir, "camera_hilti.yaml")

    # 3. ประกาศ Arguments สำหรับ Launch
    use_rviz_arg = DeclareLaunchArgument("use_rviz", default_value="False")
    avia_config_arg = DeclareLaunchArgument('avia_params_file', default_value=avia_config_cmd)
    camera_config_arg = DeclareLaunchArgument('camera_params_file', default_value=camera_config_cmd)
    use_respawn_arg = DeclareLaunchArgument('use_respawn', default_value='True')

    # 4. ดึงค่าจาก Configuration
    avia_params_file = LaunchConfiguration('avia_params_file')
    camera_params_file = LaunchConfiguration('camera_params_file')
    use_rviz = LaunchConfiguration('use_rviz')
    use_respawn = LaunchConfiguration('use_respawn')

    return LaunchDescription([
        use_rviz_arg,
        avia_config_arg,
        camera_config_arg,
        use_respawn_arg,

        # Node 1: Parameter Blackboard
        Node(
            package='demo_nodes_cpp',
            executable='parameter_blackboard',
            name='parameter_blackboard',
            parameters=[
                camera_params_file,
                {'use_sim_time': True}
            ],
            output='screen'
        ),
        
        # Node 2: FAST-LIVO2 Mapping
        Node(
            package="fast_livo",
            executable="fastlivo_mapping",
            name="laserMapping",
            parameters=[
                avia_params_file,
                camera_params_file,
                {
                    'use_sim_time': True,
                    'common.img_topic': '/alphasense/cam0/image_raw',
                    'common.lid_topic': '/hesai/pandar',
                    'common.imu_topic': '/alphasense/imu',
                    'preprocess.lidar_type': 5,
                    'preprocess.scan_line': 32,
                    'preprocess.point_filter_num': 1,
                    'preprocess.blind': 1.0,
                    'vio.grid_size': 400,
                    'vio.max_num_features': 15,
                }
            ],
            output="screen"
        ),

        # Node 3: RViz2
        Node(
            condition=IfCondition(use_rviz),
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config_file],
            parameters=[{'use_sim_time': True}],
            output='screen'
        ),
    ])