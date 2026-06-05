from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
import os

def generate_launch_description():
    config_file = os.path.expanduser('~/fast_ws/src/FAST-LIVO2/config/carla.yaml')
    rviz_config = os.path.expanduser('~/fast_ws/src/FAST-LIVO2/rviz_cfg/fast_livo2.rviz')
    
    camera_params = {
        'cam_model': 'Pinhole',
        'cam_width': 640,
        'cam_height': 480,
        'scale': 1.0,
        'cam_fx': 320.0,   # 400 * (640/800)
        'cam_fy': 320.0,
        'cam_cx': 320.0,   # 400 * (640/800)
        'cam_cy': 240.0,   # 300 * (480/600)
        'cam_d0': 0.0,
        'cam_d1': 0.0,
        'cam_d2': 0.0,
        'cam_d3': 0.0,
        'use_sim_time': False
    }
    
    return LaunchDescription([

        Node(
            package='demo_nodes_cpp',
            executable='parameter_blackboard',
            name='parameter_blackboard',
            output='screen',
            parameters=[camera_params]
        ),
        
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='fast_livo',
                    executable='fastlivo_mapping',
                    name='fastlivo_mapping',
                    output='screen',
                    parameters=[
                        config_file,
                        {
                            'use_sim_time': False,
                            'tf_buffer_duration': 300.0,
                        }
                    ]
                )
            ]
        ),
        
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    output='screen',
                    arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
                    parameters=[{'use_sim_time': True}]
                )
            ]
        )
    ])