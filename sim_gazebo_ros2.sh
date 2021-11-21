# https://github.com/ros-simulation/gazebo_ros_pkgs/wiki

# build plugins provided by sitl_gazebo
cd PX4-Autopilot
DONT_RUN=1 make px4_sitl_default gazebo
cd ..

# export plugin and library path
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:./PX4-Autopilot/build/px4_sitl_default/build_gazebo:./ros2_workspace/install/custom_gazebo_plugins/lib
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:./PX4-Autopilot/Tools/sitl_gazebo/models
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:./PX4-Autopilot/build/px4_sitl_default/build_gazebo:./ros2_workspace/install/custom_gazebo_plugins/lib

# launch world for training, check if .gazebo folder contains files with the same name
# available: omni_hex_ros2, iris_ros2
gazebo ./PX4-Autopilot/Tools/sitl_gazebo/worlds/iris_ros2.world -s libgazebo_ros_init.so -s libgazebo_ros_factory.so -s libgazebo_ros_force_system.so --verbose
