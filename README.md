# PR2 Robot Pick-and-Place
[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)

This repository enables to pick-and-place real-world tabletop objects by controlling a PR2 robot via [ROS](https://www.ros.org/) topics. 
Concretely, the current implementation was used for picking-and-placing tabletops objects from natural language. 
The node in this repository assumes that a segmented pointcloud of the object to be picked and the object placing location are being published on their respective ROS topics.
Grasping of objects is done with [GPG](https://arxiv.org/abs/1603.01564).
More information at the [project page](http://speechrobot.cs.uni-freiburg.de/).

<p align="center">
  <img src="http://speechrobot.cs.uni-freiburg.de/images/motivation.png" width="65%"/>
</p>

## Reference
If you find the code helpful please consider citing our work
```
@inproceedings{mees21iser,
  author = {Oier Mees and Wolfram Burgard},
  title = {Composing Pick-and-Place Tasks By Grounding Language},
  booktitle = {Proceedings of the International Symposium on Experimental Robotics (ISER)},
  year = 2021,
  address = {La Valletta, Malta}
}
```
```
@inproceedings{mees20icra_placements,
author = {Oier Mees and Alp Emek and Johan Vertens and Wolfram Burgard},
title = {Learning Object Placements For Relational Instructions by Hallucinating Scene Representations},
booktitle = {Proceedings of the IEEE International Conference on Robotics and Automation  (ICRA)},
year = 2020,
address = {Paris, France}
}
```

## Usage
```
roslaunch pr2_moveit_config warehouse.launch moveit_warehouse_database_path:=~/warehouse_db
roslaunch pr2_moveit_config move_group.launch
roslaunch pr2_gripper_sensor_action pr2_gripper_sensor_actions.launch
rosrun table_detection table_detection_node _topic_in:=your_cloud
roslaunch pick_and_place pick_and_place.launch
```
#### Subscribed topics
The `pick_and_place` node subscribes to the following topics:

*  `/refexp_object (sensor_msgs::PointCloud2)`: segmented point cloud of the object to be picked in the camera frame
* `/placing_pose_pub (geometry_msgs::PointStamped)`: location on which the picked object should be placed in the camera frame

#### Published topics
The `pick_and_place` node publishes to the following topics:

*  `/placing_pose (geometry_msgs::PointStamped>)`: location on which the picked object will be placed on the global frame, for visualization purposes
*  `/placing_pose_reached (std_msgs::Bool>)`: boolean indicating if the placing pose was reached
*  `/picking_reached_pub (std_msgs::Bool>)`: boolean indicating if the grasping pose was reached