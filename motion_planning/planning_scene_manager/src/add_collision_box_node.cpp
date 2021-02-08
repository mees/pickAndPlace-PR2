#include <planning_scene_manager/planning_scene_manager.h>
#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_collision_box");
  ros::NodeHandle nh_public, nh_private("~");

  std::string object_id, frame_id;
  nh_private.param<std::string>("frame_id", frame_id, "odom_combined");
  nh_private.param<std::string>("object_id", object_id, "collision_box");

  Eigen::Affine3d pose = Eigen::Affine3d::Identity();

  std::vector<double> dimensions_tmp;
  nh_private.param("dimensions", dimensions_tmp, std::vector<double>(3));
  Eigen::Vector3d dimensions(dimensions_tmp.data());

  planning_scene_manager::PlanningSceneManager scene_mgr;
  scene_mgr.addBoxCollisionObject(frame_id, object_id, pose, dimensions);

  ROS_INFO("Added collision box with parameters ...");
  ROS_INFO("frame_id: %s", frame_id.c_str());
  ROS_INFO("object_id: %s", object_id.c_str());
  ROS_INFO("position: %.2f, %.2f, %.2f", pose.translation().x(), pose.translation().y(), pose.translation().z());
  ROS_INFO("dimensions: %.2f, %.2f, %.2f", dimensions.x(), dimensions.y(), dimensions.z());

  return EXIT_SUCCESS;
}