#ifndef MOVE_GROUP_MANAGER_MOVE_GROUP_MANAGER_H
#define MOVE_GROUP_MANAGER_MOVE_GROUP_MANAGER_H

#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <moveit/move_group_interface/move_group.h>
#include <pr2_gripper_sensor_msgs/PR2GripperGrabAction.h>
#include <pr2_gripper_sensor_msgs/PR2GripperReleaseAction.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <Eigen/Geometry>

typedef actionlib::SimpleActionClient<pr2_gripper_sensor_msgs::PR2GripperGrabAction> GrabClient;
typedef actionlib::SimpleActionClient<pr2_gripper_sensor_msgs::PR2GripperReleaseAction> ReleaseClient;

namespace move_group_manager {

class MoveGroupManager {
 public:
  MoveGroupManager();

  bool planCartesianPath(const std::vector<geometry_msgs::Pose>& poses,
                         moveit::planning_interface::MoveGroup::Plan& plan);
  bool plan(const std::string& frame_id, const std::string& eef_link, const geometry_msgs::Pose& pose,
            moveit::planning_interface::MoveGroup::Plan& plan);
  bool plan(moveit::planning_interface::MoveGroup::Plan& plan);
  bool execute(const moveit::planning_interface::MoveGroup::Plan& plan);
  int pick(const geometry_msgs::Pose& grasp_pose, const geometry_msgs::Vector3& approach);
  bool pickGently(const geometry_msgs::Pose& grasp_pose, const geometry_msgs::Vector3& approach);
  int place(const geometry_msgs::Pose& pose);
  void openGripper();
  bool openGripperGently();
  bool closeGripperGently();

 private:
  ros::NodeHandle nh_;
  ros::CallbackQueue call_back_queue_;
  boost::shared_ptr<ros::AsyncSpinner> async_spinner_ptr_;
  boost::shared_ptr<moveit::planning_interface::MoveGroup> group_ptr_;
  boost::shared_ptr<moveit::planning_interface::MoveGroup> right_gripper_ptr_;
  boost::shared_ptr<GrabClient> grab_client_ptr_;
  boost::shared_ptr<ReleaseClient> release_client_ptr_;
};

}  // namespace move_group_manager

#endif