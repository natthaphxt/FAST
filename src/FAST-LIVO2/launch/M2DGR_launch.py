from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction
import os

def generate_launch_description():
    # Paths
    config_file = os.path.expanduser('~/fast_ws/src/FAST-LIVO2/config/M2DGR.yaml')
    rviz_config = os.path.expanduser('~/fast_ws/src/FAST-LIVO2/rviz_cfg/M2DGR.rviz')
    
    # Camera parameters (from M2DGR calibration)
    camera_params = {
        'cam_model': 'Pinhole',
        'cam_width': 1280,
        'cam_height': 1024,
        'scale': 1.0,
        'cam_fx': 540.645056202188,
        'cam_fy': 539.8545023658869,
        'cam_cx': 626.4125666883942,
        'cam_cy': 523.947634226782,
        'cam_d0': -0.07015146608431883,
        'cam_d1': 0.008586142263125124,
        'cam_d2': -0.021968993685891842,
        'cam_d3': 0.007442211946112636
    }
    
    return LaunchDescription([
        # 1. Parameter blackboard (for camera parameters)
        Node(
            package='demo_nodes_cpp',
            executable='parameter_blackboard',
            name='parameter_blackboard',
            output='screen',
            parameters=[camera_params]
        ),
        
        # 2. FAST-LIVO2 (wait 2 seconds for parameter server)
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
        
        # 3. RViz2 for visualization (with M2DGR config)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config]
        )
        
        # NOTE: Play rosbag manually in separate terminal:
        # cd ~/Downloads/street_08_ros2
        # ros2 bag play street_08_ros2.db3 --clock --rate 0.5 --topics /velodyne_points /handsfree/imu
    ])