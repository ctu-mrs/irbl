cmake_minimum_required(VERSION 3.16)
project(gz_greenhouse_dynamics)

# gz-sim (Harmonic) system plugins for the dynamic greenhouse world: PlantSway (wind-blown foliage),
# PathFollower (static models driven along a looping path: forklifts).
# Installed to lib/; the tmux session adds that folder to GZ_SIM_SYSTEM_PLUGIN_PATH.
find_package(ament_cmake REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)

add_library(gz_greenhouse_dynamics SHARED src/PlantSway.cc src/PathFollower.cc)
target_link_libraries(gz_greenhouse_dynamics PRIVATE gz-sim8::gz-sim8 gz-plugin2::register)

install(TARGETS gz_greenhouse_dynamics LIBRARY DESTINATION lib)
ament_package()
