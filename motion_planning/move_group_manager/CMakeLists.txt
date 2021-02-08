cmake_minimum_required(VERSION 2.8.12)
project(move_group_manager)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  eigen_conversions

  moveit_ros_planning_interface
)

find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES move_group_manager
  CATKIN_DEPENDS roscpp
# DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_library(move_group_manager src/move_group_manager.cpp)
target_link_libraries(move_group_manager ${catkin_LIBRARIES})