from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch_ros.actions import PushRosNamespace, SetRemap
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():

    lidar_launch_path = '/home/sunrise/ros2_ws/install/rplidar_ros/share/rplidar_ros/launch/rplidar_c1_launch.py'
    rf2o_launch_path = '/home/sunrise/slam_nav/odom_clean/src/rf2o_laser_odometry/launch/rf2o_laser_odometry.launch.py'
    nav_launch_path = '/home/sunrise/slam_nav/nav/launch/navigation.launch.py'

    return LaunchDescription([
        GroupAction(
            actions=[

                PushRosNamespace('robot1'),

                SetRemap(src='/tf', dst='tf'),
                SetRemap(src='/tf_static', dst='tf_static'),
                SetRemap(src='/scan', dst='scan'),
                SetRemap(src='/odom', dst='odom'),
                SetRemap(src='/cmd_vel', dst='cmd_vel'),
                SetRemap(src='/amcl_pose', dst='amcl_pose'),
                SetRemap(src='/initialpose', dst='initialpose'),

                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(lidar_launch_path)
                ),
                
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(rf2o_launch_path)
                ),

                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(nav_launch_path),
                    launch_arguments={
                        'use_sim_time': 'False',
                        'params_file': '/home/sunrise/slam_nav/nav/config/nav2_params.yaml' # 建议显式指定你的参数文件
                    }.items()
                ),
            ]
        )
    ])