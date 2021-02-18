# Motion Planning
This folder contains ROS packages that handle motion planning of a PR2 robot with [MoveIt!](https://moveit.ros.org/).

## Planning Scene Manager
Contains methods for adding and removing collision objects from the planning scene.

## Move Group Manager
Does the actual motion planning given a planning scene. We leverage [PR2's fingertip pressure sensors](http://wiki.ros.org/pr2_gripper_sensor_action) for gently opening and closing the gripper.