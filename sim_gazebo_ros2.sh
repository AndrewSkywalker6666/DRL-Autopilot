# https://github.com/ros-simulation/gazebo_ros_pkgs/wiki

# build plugins provided by sitl_gazebo
echo -e "\033[32m === build plugins provided by sitl_gazebo === \033[0m"
cd PX4-Autopilot
DONT_RUN=1 make px4_sitl_default gazebo
cd ..

# build custom plugins
echo -e "\033[32m === build custom plugins === \033[0m"
cd ros2_workspace
colcon build --packages-select custom_gazebo_plugins
cd ..

# export plugin and library path
echo -e "\033[32m === export path === \033[0m"
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:./PX4-Autopilot/build/px4_sitl_default/build_gazebo:./ros2_workspace/install/custom_gazebo_plugins/lib
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:./PX4-Autopilot/Tools/sitl_gazebo/models
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:./PX4-Autopilot/build/px4_sitl_default/build_gazebo:./ros2_workspace/install/custom_gazebo_plugins/lib
echo GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH
echo GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH
echo LD_LIBRARY_PATH=$LD_LIBRARY_PATH

# launch world for training, check if .gazebo folder contains files with the same name
# available: omni_hex_ros2, iris_ros2
echo -e "\033[32m === launching sim === \033[0m"
gazebo ./PX4-Autopilot/Tools/sitl_gazebo/worlds/omni_hex_ros2.world -s libgazebo_ros_init.so -s libgazebo_ros_factory.so -s libgazebo_ros_force_system.so --verbose
