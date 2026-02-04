terminal1:
cd slam_nav/microros_ws
source install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200


terminal2:
ros2 run teleop_twist_keyboard teleop_twist_keyboard


terminal3:
ros2 launch rplidar_ros rplidar_c1_launch.py


terminal4:
cd slam_nav/odom_clean
source ~/slam_nav/odom_clean/install/setup.bash
ros2 launch rf2o_laser_odometry rf2o_laser_odometry.launch.py


terminal5:
cd slam_nav/carto_ws/configuration_files
ros2 launch c1_2d.launch.py

// build rplidar_ros from github instead of apt and run rplidar_c1_launch.py


