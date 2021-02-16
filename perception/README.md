# Perception packages
This folder contains ROS packages that help a robot with perception tasks.

## Table detection
This package takes a point cloud as input and publishes the plane coefficients and the detected table as a collision object.
#### Subscribed topics
The `table_detection` node subscribes to the following topics:

*  `/topic_in (sensor_msgs::PointCloud2)` point cloud of a scene containing a table

#### Published topics
The `table_detection` node publishes to the following topics:

* `/model_coeff_pub (pcl_msgs::ModelCoefficients)` model coefficients of the detected plane
* `/markers (visualization_msgs::MarkerArray)` rviz marker array visualization of the detetcted table

Besides it adds the table as a collision object in the moveit planning scene.

## Depth servers
This package contains ROS services used to query the depth of pixels in a point cloud. If the queried point has a nan or inf, we return the value of the closest point in the point cloud.

### get_kinect_depth
Expects as a query 2D pixel int32 u, int32 v.
#### Subscribed topics
The `get_kinect_depth` service subscribes to the following topics:
*  `/camera/depth_registered/points (sensor_msgs::PointCloud2)` point cloud of a scene

#### Published topics
The `get_kinect_depth` service publishes to the following topics:
* `placing_pose_pub (geometry_msgs/PointStamped)` 3D point of the queried pixel


### convert_mask2pcl
Expects as a query an array os 2D pixel float32[] data, in our case pixel values of an image mask.
#### Subscribed topics
The `convert_mask2pcl` service subscribes to the following topics:
*  `/camera/depth_registered/points (sensor_msgs::PointCloud2)` point cloud of a scene

#### Published topics
The `convert_mask2pcl` service publishes to the following topics:
* `refexp_object (sensor_msgs::PointCloud2)` point cloud of the corresponding image mask
