cmake_minimum_required(VERSION 2.8.3)
project(pioneer_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  joy
  tf
  hokuyo_node
  nav_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pioneer_driver
#  CATKIN_DEPENDS geometry_msgs joy roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

# include dirs
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# qtcreator hack
file(GLOB_RECURSE SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.hpp include/*.h include/*.hpp)

# driver exe
add_executable(pioneer_manual src/main.cpp ${SOURCES})
target_link_libraries(pioneer_manual ${catkin_LIBRARIES})

# pose_with_cov
add_executable(pose_with_cov src/add_covariance.cpp ${SOURCES})
target_link_libraries(pose_with_cov ${catkin_LIBRARIES})
