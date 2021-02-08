#ifndef PLANNING_SCENE_MANAGER_PLANNING_SCENE_MANAGER_H
#define PLANNING_SCENE_MANAGER_PLANNING_SCENE_MANAGER_H

#include <ros/ros.h>
#include <Eigen/Geometry>
#include <std_msgs/Header.h>
#include <geometry_msgs/Pose.h>
#include <shape_msgs/SolidPrimitive.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/AllowedCollisionMatrix.h>

namespace planning_scene_manager
{

class PlanningSceneManager
{

public:

  PlanningSceneManager();

  void allowCollision(const std::string& id);

  bool getCollisionObject(const std::string& object_id, moveit_msgs::CollisionObject& object);

  void addBoxCollisionObject(const std::string& frame_id, const std::string& object_id,
    const Eigen::Affine3d& pose, const Eigen::Vector3d& dimensions);

  void addSphereCollisionObject(const std::string& frame_id, const std::string& object_id,
    const Eigen::Affine3d& pose, double radius);

  void removeCollisionObject(const std::string& frame_id, const std::string& object_id);

private:

  bool getAllowedCollisionMatrix(moveit_msgs::AllowedCollisionMatrix& acm);

  bool getCollisionObjects(std::vector<moveit_msgs::CollisionObject>& objects);

  void applyPlanningScene(const moveit_msgs::PlanningScene& scene);

  void expandAllowedCollisionMatrix(moveit_msgs::AllowedCollisionMatrix& acm,
    const std::string& entry, bool value = false);

  void addCollisionObject(const moveit_msgs::CollisionObject& object);

  void initHeader(const ros::Time& stamp, const std::string& frame_id, std_msgs::Header& header);

  void initSolidPrimitive(const int& type, const std::vector<double>& dimensions,
    shape_msgs::SolidPrimitive& primitive);

  size_t find(const std::vector<std::string>& vector, const std::string& string);

  std::string ns() { return "planning_scene_manager"; }
  std::string name() { return "PlanningSceneManager"; }

  ros::NodeHandle nh_;
  ros::ServiceClient client_get_scene_;
  ros::ServiceClient client_apply_scene_;

};

}

#endif