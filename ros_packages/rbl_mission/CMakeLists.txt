cmake_minimum_required(VERSION 3.8)
project(rbl_mission)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options("-Wall" "-Wextra" "-Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rbl_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tinyxml2_vendor REQUIRED)
find_package(TinyXML2 REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(visualization_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  srv/Deploy.srv
  srv/DeploySequence.srv
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

add_executable(warehouse_inspection_node src/warehouse_inspection_node.cpp)
target_include_directories(warehouse_inspection_node PRIVATE include)
ament_target_dependencies(warehouse_inspection_node
  rclcpp
  rbl_msgs
  nav_msgs
  geometry_msgs
  std_msgs
  std_srvs
  visualization_msgs
)
target_link_libraries(warehouse_inspection_node tinyxml2::tinyxml2)

# offline check of the coverage planner (same code as the node), see src/coverage_plan_check.cpp
add_executable(coverage_plan_check src/coverage_plan_check.cpp)
target_include_directories(coverage_plan_check PRIVATE include)
ament_target_dependencies(coverage_plan_check rclcpp)
target_link_libraries(coverage_plan_check tinyxml2::tinyxml2)

# UGV + UAV deployment coordinator, see src/ugv_uav_mission_node.cpp
add_executable(ugv_uav_mission_node src/ugv_uav_mission_node.cpp)
target_include_directories(ugv_uav_mission_node PRIVATE include)
ament_target_dependencies(ugv_uav_mission_node rclcpp rbl_msgs nav_msgs geometry_msgs std_msgs std_srvs visualization_msgs)
target_link_libraries(ugv_uav_mission_node tinyxml2::tinyxml2 "${cpp_typesupport_target}")

# simplified rviz picture of the mission (vehicles + warehouse), see src/mission_viz_node.cpp
add_executable(mission_viz_node src/mission_viz_node.cpp)
target_include_directories(mission_viz_node PRIVATE include)
ament_target_dependencies(mission_viz_node rclcpp nav_msgs std_msgs visualization_msgs geometry_msgs)
target_link_libraries(mission_viz_node tinyxml2::tinyxml2)

install(TARGETS warehouse_inspection_node coverage_plan_check ugv_uav_mission_node mission_viz_node
  DESTINATION lib/${PROJECT_NAME}
)

# ArUco detector -> PX4 TARGET_RELATIVE (built-in precision landing), see the script header
install(PROGRAMS scripts/aruco_landing_target.py DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()
