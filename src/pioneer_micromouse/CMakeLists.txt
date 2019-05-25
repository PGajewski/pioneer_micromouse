cmake_minimum_required(VERSION 2.8.3)
project(pioneer_micromouse)
add_compile_options(-std=c++17)

# Catkin components
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  tf
)


# Catkin
catkin_package(
#  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs tf
#  DEPENDS system_lib
)

# Includes
include_directories(
  ${catkin_INCLUDE_DIRS}
  include/
)

file( GLOB dijkstra_SRC src/algorithm/dijkstra/*.cpp )
# Exe & linking (pioneer_control)
add_executable(pioneer_control src/pioneer_control.cpp src/map.cpp ${dijkstra_SRC} )
target_link_libraries(pioneer_control ${catkin_LIBRARIES})
