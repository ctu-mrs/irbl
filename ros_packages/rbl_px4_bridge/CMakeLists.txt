cmake_minimum_required(VERSION 3.8)
project(rbl_px4_bridge)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options("-Wall" "-Wextra" "-Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(px4_ros_com REQUIRED)
find_package(rbl_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
# filter_reflective_uavs's exported CMake target links mrs_lib and ouster_sensor_msgs; this
# package's own code uses neither, but they still need to be find_package()-d here to resolve
# that exported target.
find_package(mrs_lib REQUIRED)
find_package(ouster_sensor_msgs REQUIRED)
find_package(ouster_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(filter_reflective_uavs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(px4_bridge_node src/px4_bridge_node.cpp)

target_include_directories(px4_bridge_node PRIVATE ${EIGEN3_INCLUDE_DIR})

ament_target_dependencies(px4_bridge_node
  rclcpp
  px4_msgs
  rbl_msgs
  nav_msgs
  geometry_msgs
  std_srvs
)

target_link_libraries(px4_bridge_node
  px4_ros_com::frame_transforms
)

add_executable(group_state_relay_node src/group_state_relay_node.cpp)

ament_target_dependencies(group_state_relay_node
  rclcpp
  nav_msgs
  filter_reflective_uavs
)

install(TARGETS px4_bridge_node group_state_relay_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
