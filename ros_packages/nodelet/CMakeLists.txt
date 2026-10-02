cmake_minimum_required(VERSION 3.8)
project(rbl_controller_node)

# set the correct standards
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# set the compile options to show code warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options("-Wall" "-Wextra" "-Wpedantic")
endif()

if(ENABLE_COVERAGE)
  message(WARNING "building for coverage, the performance might be limited")

  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --coverage -fprofile-arcs")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage -fprofile-arcs")

  add_definitions("-O1")
  add_definitions("-g")
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# Optional: only used for the "~/reflective_group_states_in" input (the filter_reflective_uavs
# detector's output). Without it RBL builds and runs normally; group states then come only from
# "~/group_states_in" (rbl_msgs/PoseVelocityArray). Its exported CMake target links mrs_lib and
# ouster_ros, so those are needed only in that case.
find_package(filter_reflective_uavs QUIET)
if(filter_reflective_uavs_FOUND)
  find_package(mrs_lib REQUIRED)
  find_package(ouster_sensor_msgs REQUIRED)
  message(STATUS "filter_reflective_uavs found: enabling ~/reflective_group_states_in")
else()
  message(STATUS "filter_reflective_uavs not found: building without ~/reflective_group_states_in")
endif()
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rbl_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rbl_replanner REQUIRED)
find_package(ciri REQUIRED)
find_package(rbl_controller_core REQUIRED)

# Define the shared library created by this package
set(RBL_CONTROLLER_NODE_TARGET ${PROJECT_NAME}_component)

add_library(${RBL_CONTROLLER_NODE_TARGET} SHARED)

target_sources(${RBL_CONTROLLER_NODE_TARGET}
  PRIVATE
    "src/nodelet.cpp"
    "src/local_static_map.cpp"
)

ament_target_dependencies(
  ${RBL_CONTROLLER_NODE_TARGET}
  rclcpp
  rclcpp_components
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_sensor_msgs
  octomap_msgs
  pcl_conversions
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  rbl_msgs
  std_srvs
  std_msgs
)

if(filter_reflective_uavs_FOUND)
  ament_target_dependencies(${RBL_CONTROLLER_NODE_TARGET} filter_reflective_uavs)
  target_compile_definitions(${RBL_CONTROLLER_NODE_TARGET} PRIVATE RBL_WITH_FILTER_REFLECTIVE_UAVS)
endif()

target_link_libraries(
  ${RBL_CONTROLLER_NODE_TARGET}
  Eigen3::Eigen
  rbl_replanner::rbl_replanner
  ciri::ciri
  rbl_controller_core::rbl_controller_core
  ${OCTOMAP_LIBRARIES}
  ${PCL_LIBRARIES}
)

# target_link_libraries(rbl_nodelet
#   PRIVATE
#     rclcpp::rclcpp
#     rclcpp_components::component
#     mrs_lib::mrs_lib
#     Eigen3::Eigen
#     rbl_replanner::rbl_replanner
#     ciri::ciri
#     rbl_controller_core::rbl_controller_core
#     ${geometry_msgs_TARGETS}
#     ${nav_msgs_TARGETS}
#     ${mrs_msgs_TARGETS}
#     ${std_srvs_TARGETS}
#     ${PCL_LIBRARIES}
# )

# Register components defined in this library.
# rclcpp_components_register_nodes(<library> <names of components>...)
rclcpp_components_register_nodes(${RBL_CONTROLLER_NODE_TARGET}
  "rbl_controller::WrapperRosRBL"
)

## --------------------------------------------------------------
## |                           testing                          |
## --------------------------------------------------------------

if(ENABLE_TESTS)

  add_subdirectory(test)

  find_package(ament_cmake_clang_format REQUIRED)
  ament_clang_format(CONFIG_FILE "${PROJECT_SOURCE_DIR}/.clang-format")
  find_package(ament_cmake_xmllint REQUIRED)
  ament_xmllint()

endif()

install(
  TARGETS ${RBL_CONTROLLER_NODE_TARGET}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY launch
  DESTINATION "share/${PROJECT_NAME}"
)

install(
  DIRECTORY config
  DESTINATION "share/${PROJECT_NAME}"
)

# install(
#   DIRECTORY tmux
#   USE_SOURCE_PERMISSIONS
#   DESTINATION "share/${PROJECT_NAME}"
# )

ament_export_targets(
  export_${PROJECT_NAME} HAS_LIBRARY_TARGET
)

ament_package()
