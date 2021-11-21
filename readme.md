# Deep Reinforcement Learning Autopilot

![overview](./overview.jpg)

## branch information

1. ROOT
   1. master (ROS2, **always switch px4 back to ROS2 branch before commiting**)
   3. other dev branches (dynamically created and merged)
2. PX4
   1. master: upstream
   2. DRL-Autopilot: ros1 based DRL method
   3. DRL2-Autopilot: ros2 based, derived from master at the time of clone
   4. modified repos should be merged with future stable releases
3. mavlink C library in PX4
   1. master: upstream
   2. DRL-Autopilot: to cooperate with ros1 mavros system
4. sitl_gazebo
   1. master: upstream
   2. DRL-Autopilot: for DRL training, ros1 and ros2 use the same branch
5. mavros and mavlink (ROS1)
   1. master: upstream
   2. DRL-Autopilot: ros1 packages for px4 DRL-Autopilot branch

## usage

1. with ROS1
   1. clone this repo recursively, **checkout the correct branches**
   2. build ros packages with `catkin build`, remember to source this workspace and launch mavros by `roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"`
   3. launch QGC and build PX4 by `make px4_sitl gazebo`
2. with ROS2
   1. clone the repo recursively, **checkout the correct branches DRL2-Autopilot**
   2. build ros packages with the script `buid_ros2_px4_clean.sh`
   3. launch QGC and build PX4 by `make px4_sitl_rtps gazebo`
   4. `micrortps_agent -t UDP` to start daemon process (built within the px4_ros_com package)
   5. `ros2 launch px4_ros_com sensor_combined_listener.launch.py` (demo) or use rqt to access topics
   6. create other ros2 applications and use the topic

## REPO structure

1. `dev_notes`
   * about recent development progress and interesting paper related to this project
2. `ros*_workspace`
   * ros workspaces as they are
3. `*.sh`
   * useful lazy buttons
4. `PX4-Autopilot`
   * sensor, RC, PWM driver interface

## TODOs

1. learn to use ros2-python interface
2. check motor speed consistency with PWM pipeline
3. formulate the problem and construct gym env
4. export the network to CPP
5. write a trajectory generator from stick input
