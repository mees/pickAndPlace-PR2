cmake_minimum_required(VERSION 2.8.12)
project(table_detection)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2_ros
  tf2_sensor_msgs
  sensor_msgs
  geometry_msgs
  pcl_conversions
  planning_scene_manager
  rviz_visualizer
)

find_package(PCL REQUIRED)
add_definitions(${PCL_DEFINITIONS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES table_detection
  CATKIN_DEPENDS roscpp planning_scene_manager rviz_visualizer
  DEPENDS PCL
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_library(table_detection src/table_detection.cpp)
target_link_libraries(table_detection ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(table_detection_node src/table_detection_node.cpp)
target_link_libraries(table_detection_node ${catkin_LIBRARIES} table_detection)
