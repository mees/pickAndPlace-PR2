#ifndef MOVE_GROUP_MANAGER_MOVE_GROUP_MANAGER_H
#define MOVE_GROUP_MANAGER_MOVE_GROUP_MANAGER_H

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <Eigen/Geometry>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Pose.h>
#include <moveit/move_group_interface/move_group.h>

namespace move_group_manager
{

class MoveGroupManager
{

public:

  MoveGroupManager();

  bool planCartesianPath(const std::vector<geometry_msgs::Pose>& poses,
    moveit::planning_interface::MoveGroup::Plan& plan);

  bool plan(const std::string& frame_id, const std::string& eef_link,
    const geometry_msgs::Pose& pose, moveit::planning_interface::MoveGroup::Plan& plan);

  bool plan(moveit::planning_interface::MoveGroup::Plan& plan);

  bool execute(const moveit::planning_interface::MoveGroup::Plan& plan);

  int pick(const geometry_msgs::Pose& grasp_pose, const geometry_msgs::Vector3& approach);

  int place(const geometry_msgs::Pose& pose);

  void openGripper();

private:

  ros::NodeHandle nh_;
  ros::CallbackQueue call_back_queue_;
  boost::shared_ptr<ros::AsyncSpinner> async_spinner_ptr_;
  boost::shared_ptr<moveit::planning_interface::MoveGroup> group_ptr_;
  boost::shared_ptr<moveit::planning_interface::MoveGroup> right_gripper_ptr_;

};

}

#endif