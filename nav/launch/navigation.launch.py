import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    map_yaml_path = os.path.expanduser('~/map2.yaml') 
    nav2_params_path = os.path.expanduser('~/slam_nav/nav/config/nav2_params.yaml')

    # rviz_config_path = os.path.expanduser('~/config/nav2_view.rviz')
    
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    default_bt_xml_path = os.path.join(
        get_package_share_directory('nav2_bt_navigator'),
        'behavior_trees','navigate_to_pose_w_replanning_and_recovery.xml'
    )
    start_nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_yaml_path,
            'params_file': nav2_params_path,
            'use_sim_time': 'false',
            'autostart': 'true',
            'default_bt_xml_filename': default_bt_xml_path,
            'use_composition': 'False',
            'base_frame_id': 'base_link'
        }.items()
    )

    start_static_tf_cmd = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser']
    )

    start_rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    ld = LaunchDescription()

    ld.add_action(start_nav2_cmd)
    ld.add_action(start_static_tf_cmd)
    ld.add_action(start_rviz_cmd)

    return ld
