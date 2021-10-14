# Deep Reinforcement Learning Autopilot

## branch information

1. ROOT
   1. master (ROS2)
   2. master-ROS1 (for ROS1)
   3. other dev branches (dynamically created and merged)
2. PX4
   1. master: should be updated regularly to be consistent with upstream
   2. DRL-Autopilot: ros1 based DRL method
   3. DRL-Autopilot-ROS2: ros2 based, derived from master
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
6. 

## usage

1. with ROS1
   1. clone this repo recursively, checkout the correct branches
   2. build ros packages with `catkin build`, remember to source this workspace and launch mavros by `roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"`
   3. launch QGC and build PX4 by `make px4_sitl gazebo`
2. with ROS2
   1. 

