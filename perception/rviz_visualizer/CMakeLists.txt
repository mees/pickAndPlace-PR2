cmake_minimum_required(VERSION 2.8.12)
project(rviz_visualizer)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  std_msgs
  geometry_msgs
  visualization_msgs
  eigen_conversions
)

find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rviz_visualizer
  CATKIN_DEPENDS roscpp
#  DEPENDS Eigen
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIR}
)

add_library(rviz_visualizer src/rviz_visualizer.cpp)
target_link_libraries(rviz_visualizer ${catkin_LIBRARIES})

add_executable(rviz_visualizer_node src/rviz_visualizer_node.cpp)
target_link_libraries(rviz_visualizer_node ${catkin_LIBRARIES} rviz_visualizer)