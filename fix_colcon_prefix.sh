cd ros2_workspace
export AMENT_PREFIX_PATH=/home/yueqian/Documents/DRL-Autopilot/ros2_workspace/install/training_room:/home/yueqian/Documents/DRL-Autopilot/ros2_workspace/install/px4_ros_com:/home/yueqian/Documents/DRL-Autopilot/ros2_workspace/install/drl_controller:/home/yueqian/Documents/DRL-Autopilot/ros2_workspace/install/px4_msgs:/home/yueqian/Documents/DRL-Autopilot/ros2_workspace/install/custom_gazebo_plugins:/opt/ros/foxy
export CMAKE_PREFIX_PATH=/home/yueqian/Documents/DRL-Autopilot/ros2_workspace/install/training_room:/home/yueqian/Documents/DRL-Autopilot/ros2_workspace/install/px4_ros_com:/home/yueqian/Documents/DRL-Autopilot/ros2_workspace/install/drl_controller:/home/yueqian/Documents/DRL-Autopilot/ros2_workspace/install/px4_msgs:/home/yueqian/Documents/DRL-Autopilot/ros2_workspace/install/custom_gazebo_plugins:/opt/ros/foxy
cd ..

# WARNING:colcon.colcon_ros.prefix_path.ament:The path xxx AMENT_PREFIX_PATH doesn’t exist
# WARNING:colcon.colcon_ros.prefix_path.catkin:The path xxx CMAKE_PREFIX_PATH doesn’t exist

# ROS2 删除自定义包后 编译警告
# 在终端内：使用命令列出编译路径

# printenv  AMENT_PREFIX_PATH CMAKE_PREFIX_PATH

# 然后根据自己删除的包删除对应的路径后再赋值回去
