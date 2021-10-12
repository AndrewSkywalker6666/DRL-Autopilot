# Deep Reinforcement Learning Autopilot

## usage

1. clone this repo recursively

2. build ros packages with `catkin build`, remember to source this workspace and launch mavros by `roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"`

3. launch QGC and build PX4 by `make px4_sitl gazebo`