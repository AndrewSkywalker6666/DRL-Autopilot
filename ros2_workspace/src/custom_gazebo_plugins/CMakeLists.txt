cmake_minimum_required(VERSION 3.5)
project(custom_gazebo_plugins)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  #add_compile_options(-Wall -Wextra -Wpedantic)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# link dir
link_directories(${gazebo_dev_LIBRARY_DIRS})

# gazebo_ros_joint_pid_ctrl START
add_library(gazebo_ros_joint_pid_ctrl SHARED src/gazebo_ros_joint_pid_ctrl.cpp)
target_include_directories(gazebo_ros_joint_pid_ctrl PUBLIC include)
ament_target_dependencies(gazebo_ros_joint_pid_ctrl
  "gazebo_dev"
  "gazebo_ros"
  "std_msgs"
  "rclcpp"
)
ament_export_libraries(gazebo_ros_joint_pid_ctrl)
# gazebo_ros_joint_pid_ctrl END

# gazebo_ros_motor_model START
add_library(gazebo_ros_motor_model SHARED src/gazebo_ros_motor_model.cpp)
target_include_directories(gazebo_ros_motor_model PUBLIC include)
ament_target_dependencies(gazebo_ros_motor_model
  "gazebo_dev"
  "gazebo_ros"
  "std_msgs"
  "rclcpp"
)
ament_export_libraries(gazebo_ros_motor_model)
# gazebo_ros_motor_model END

# gazebo_ros_joint_motor START
add_library(gazebo_ros_joint_motor SHARED src/gazebo_ros_joint_motor.cpp)
target_include_directories(gazebo_ros_joint_motor PUBLIC include)
ament_target_dependencies(gazebo_ros_joint_motor
        "gazebo_dev"
        "gazebo_ros"
        "std_msgs"
        "rclcpp"
        )
ament_export_libraries(gazebo_ros_joint_motor)
# gazebo_ros_joint_motor END


ament_export_include_directories(include)
ament_export_dependencies(rclcpp)
ament_export_dependencies(gazebo_dev)
ament_export_dependencies(gazebo_msgs)
ament_export_dependencies(gazebo_ros)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

install(TARGETS
    gazebo_ros_joint_pid_ctrl
    gazebo_ros_motor_model
    gazebo_ros_joint_motor
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
