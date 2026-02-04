pc端：
# 创建工作空间并下载源码
mkdir -p ~/microros_ws/src
cd ~/microros_ws/src
git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git
cd ..
rosdep update && rosdep install --from-paths src --ignore-src -y
colcon build
source install/local_setup.bash

# 下载并创建 Agent
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash

#启动 Agent： 通过串口连接 ESP32（假设是 /dev/ttyUSB0，波特率通常设为 115200）：
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200

esp32端：
active_move.ino