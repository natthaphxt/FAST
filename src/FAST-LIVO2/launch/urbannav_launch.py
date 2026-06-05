from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, ExecuteProcess
import os

def generate_launch_description():
    config_file = os.path.expanduser('~/fast_ws/src/FAST-LIVO2/config/urbannav.yaml')
    rviz_config = os.path.expanduser('~/fast_ws/src/FAST-LIVO2/rviz_cfg/fast_livo2.rviz')

    # ZED2 left camera intrinsics (from /zed2/camera/left/camera_info in bag)
    camera_params = {
        'cam_model': 'Pinhole',
        'cam_width': 672,
        'cam_height': 376,
        'scale': 1.0,
        'cam_fx': 264.9425,
        'cam_fy': 264.7900,
        'cam_cx': 334.3975,
        'cam_cy': 183.1620,
        'cam_d0': -0.0442856,
        'cam_d1': 0.0133574,
        'cam_d2': 0.0,
        'cam_d3': 0.0
    }

    fix_image_script = os.path.expanduser('~/fast_ws/src/FAST-LIVO2/scripts/fix_image.py')

    return LaunchDescription([
        Node(
            package='demo_nodes_cpp',
            executable='parameter_blackboard',
            name='parameter_blackboard',
            output='screen',
            parameters=[camera_params]
        ),

        ExecuteProcess(
            cmd=['python3', fix_image_script],
            output='screen'
        ),

        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='fast_livo',
                    executable='fastlivo_mapping',
                    name='fastlivo_mapping',
                    output='screen',
                    parameters=[config_file]
                )
            ]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config]
        )

        # NOTE: Play rosbag in separate terminal:
        # ros2 bag play ~/fast_ws/urbannav_bag --clock --rate 0.5
    ])
