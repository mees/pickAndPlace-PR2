cmake_minimum_required(VERSION 2.8.12)
project(planning_scene_manager)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  std_msgs
  moveit_msgs
  geometry_msgs
  shape_msgs
  eigen_conversions
)

find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES planning_scene_manager
  CATKIN_DEPENDS roscpp
  DEPENDS Eigen
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_library(planning_scene_manager src/planning_scene_manager.cpp)
target_link_libraries(planning_scene_manager ${catkin_LIBRARIES})

add_executable(add_collision_box_node src/add_collision_box_node.cpp)
target_link_libraries(add_collision_box_node ${catkin_LIBRARIES} planning_scene_manager)