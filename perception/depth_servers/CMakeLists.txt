cmake_minimum_required(VERSION 2.8.3)
project(depth_servers)

SET(CMAKE_BUILD_TYPE Release)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  tf_conversions
  cv_bridge
  sensor_msgs
  geometry_msgs
  message_generation
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  pluginlib
  geometric_shapes
  moveit_visual_tools
  planning_scene_manager
  move_group_manager
  moveit_ros_warehouse
  planning_scene_manager
  pcl_conversions
)  

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

find_package(PCL  REQUIRED)
include_directories(${PCL_INCLUDE_DIRS}  ${PROJECT_SOURCE_DIR}/include)
link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   depth_server.srv
   mask2pcl_server.srv
   #point2pcl_server.srv
 )
## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs
   geometry_msgs
 )

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs message_runtime moveit_core
    moveit_ros_planning_interface planning_scene_manager move_group_manager moveit_ros_warehouse
)

###########
## Build ##
###########

include_directories(include ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(depth_server  src/depth_server.cpp)
add_executable(mask2pcl_server  src/mask2pcl_server.cpp)
add_executable(point2pcl_server  src/point2pcl_server.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(depth_server   ${catkin_LIBRARIES} ${boost_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(mask2pcl_server   ${catkin_LIBRARIES} ${boost_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(point2pcl_server   ${catkin_LIBRARIES} ${boost_LIBRARIES} ${PCL_LIBRARIES})

