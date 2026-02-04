"""
  Copyright 2018 The Cartographer Authors
  Copyright 2022 Wyca Robotics (for the ros2 conversion)

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import Shutdown
import os

def generate_launch_description():
    ## ***** Launch arguments *****
#    bag_filename_arg = DeclareLaunchArgument('bag_filename')

  ## ***** File paths ******
#    pkg_share = FindPackageShare('carto_ws').find('')
#    urdf_dir = os.path.join(pkg_share, 'urdf')
#    urdf_file = os.path.join(urdf_dir, 'backpack_2d.urdf')
#    with open(urdf_file, 'r') as infp:
#        robot_desc = infp.read()
    config_dir = '/home/sunrise/carto_ws/configuration_files'
    use_sim_time = LaunchConfiguration('use_sim_time',default='false')
    resolution = LaunchConfiguration('resolution',default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec',default='1.0')
    ## ***** Nodes *****
    tf2_node = Node(package='tf2_ros',
                   executable='static_transform_publisher',
                   name='static_tf_pub_laser',
                   arguments=['0.0','0.0','0.0','0','0','0','base_link','laser'],
                   )

    cartographer_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_node',
        name='cartographer_node',
        parameters = [{'use_sim_time': False}],
        arguments = [
            '-configuration_directory', config_dir,
            '-configuration_basename', 'c1_2d.lua'],
        remappings = [
            ('scan', 'scan'),
            ('odom','odom_rf2o')],
        output = 'screen'
        )

    cartographer_occupancy_grid_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_occupancy_grid_node',
        parameters = [
            {'use_sim_time': False},
            {'resolution': 0.05}],
        arguments=['-resolution',resolution,'-publish_period_sec',publish_period_sec]
        )

    rviz_node = Node(
        package = 'rviz2',
        executable = 'rviz2',
        name='rviz2',
        # arguments = ['-d', FindPackageShare('cartographer_ros').find('cartographer_ros') + '/configuration_files/demo_2d.rviz'],
        parameters = [{'use_sim_time': use_sim_time}],
        output='screen'
        )

#    ros2_bag_play_cmd = ExecuteProcess(
#        cmd = ['ros2', 'bag', 'play', LaunchConfiguration('bag_filename'), '--clock'],
#        name = 'rosbag_play',
#    )

    return LaunchDescription([
        # Launch arguments
        #bag_filename_arg,
        # Nodes
        #robot_state_publisher_node,
        tf2_node,
        cartographer_node,
        cartographer_occupancy_grid_node,
        rviz_node,
        #ros2_bag_play_cmd
    ])
