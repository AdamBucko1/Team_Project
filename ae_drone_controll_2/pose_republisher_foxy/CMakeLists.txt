cmake_minimum_required(VERSION 3.5)
project(tp_local_odom)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(interfaces REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)


#add_executable(local_odom_node src/local_odom_node.cpp)
add_executable(pose_republisher_foxy src/pose_republisher_foxy.cpp)
add_executable(reset_odom_srv src/reset_odom_srv.cpp)

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

#ament_target_dependencies(local_odom_node rclcpp sensor_msgs nav_msgs geometry_msgs tf2_geometry_msgs tf2 tf2_ros) 
ament_target_dependencies(pose_republisher_foxy rclcpp  sensor_msgs nav_msgs geometry_msgs tf2_geometry_msgs tf2 tf2_ros interfaces Eigen3) 
ament_target_dependencies(reset_odom_srv rclcpp  sensor_msgs nav_msgs geometry_msgs tf2_geometry_msgs tf2 tf2_ros interfaces Eigen3) 
include_directories(
  ${EIGEN3_INCLUDE_DIR}
  include
)

install(TARGETS
 # local_odom_node
  pose_republisher_foxy
  reset_odom_srv
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
