#include <move_group_manager/move_group_manager.h>
#include <moveit_msgs/RobotTrajectory.h>

namespace move_group_manager
{

MoveGroupManager::MoveGroupManager()
{
  nh_.setCallbackQueue(&call_back_queue_);
  async_spinner_ptr_.reset(new ros::AsyncSpinner(0, &call_back_queue_));

  moveit::planning_interface::MoveGroup::Options options_arm("right_arm", "robot_description", nh_);
  group_ptr_.reset(new moveit::planning_interface::MoveGroup(options_arm));

  moveit::planning_interface::MoveGroup::Options options_right_gripper("right_gripper", "robot_description", nh_);
  right_gripper_ptr_.reset(new moveit::planning_interface::MoveGroup(options_right_gripper));

  group_ptr_->setGoalTolerance(0.01);
  //group_ptr_->setNumPlanningAttempts(3);
  group_ptr_->setPlanningTime(10.0);
}

bool MoveGroupManager::planCartesianPath(const std::vector<geometry_msgs::Pose>& poses,
  moveit::planning_interface::MoveGroup::Plan& plan)
{
  moveit_msgs::RobotTrajectory trajectory;
  double fraction = group_ptr_->computeCartesianPath(poses, 0.01, 0.0, trajectory);
  plan.trajectory_ = trajectory;

  bool result = this->plan(plan);

  return result;
}

bool MoveGroupManager::plan(const std::string& frame_id, const std::string& eef_link,
  const geometry_msgs::Pose& pose, moveit::planning_interface::MoveGroup::Plan& plan)
{
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.frame_id = frame_id;
  pose_stamped.pose = pose;

  group_ptr_->setPoseTarget(pose_stamped, eef_link);
  bool result = this->plan(plan);

  return result;
}

bool MoveGroupManager::plan(moveit::planning_interface::MoveGroup::Plan& plan)
{
  async_spinner_ptr_->start();
  bool result = group_ptr_->plan(plan);
  async_spinner_ptr_->stop();

  return result;
}

bool MoveGroupManager::execute(const moveit::planning_interface::MoveGroup::Plan& plan)
{
  async_spinner_ptr_->start();
  bool result = group_ptr_->execute(plan);
  async_spinner_ptr_->stop();

  return result;
}

int MoveGroupManager::pick(const geometry_msgs::Pose& grasp_pose, const geometry_msgs::Vector3& approach)
{
  moveit_msgs::Grasp grasp;

  grasp.grasp_pose.header.frame_id = "odom_combined";
  grasp.grasp_pose.pose = grasp_pose;

  grasp.pre_grasp_approach.direction.header.frame_id = "odom_combined";
  grasp.pre_grasp_approach.direction.vector = approach;
  grasp.pre_grasp_approach.min_distance = 0.1;
  grasp.pre_grasp_approach.desired_distance = 0.25;

  grasp.post_grasp_retreat.direction.header.frame_id = "odom_combined";
  grasp.post_grasp_retreat.direction.vector.z = 1.0;
  grasp.post_grasp_retreat.min_distance = 0.1;
  grasp.post_grasp_retreat.desired_distance = 0.25;

  // Open gripper
  grasp.pre_grasp_posture.joint_names.resize(6);
  grasp.pre_grasp_posture.joint_names[0] = "r_gripper_joint";
  grasp.pre_grasp_posture.joint_names[1] = "r_gripper_motor_screw_joint";
  grasp.pre_grasp_posture.joint_names[2] = "r_gripper_l_finger_joint";
  grasp.pre_grasp_posture.joint_names[3] = "r_gripper_r_finger_joint";
  grasp.pre_grasp_posture.joint_names[4] = "r_gripper_r_finger_tip_joint";
  grasp.pre_grasp_posture.joint_names[5] = "r_gripper_l_finger_tip_joint";

  grasp.pre_grasp_posture.points.resize(1);
  grasp.pre_grasp_posture.points[0].positions.resize(6);
  grasp.pre_grasp_posture.points[0].positions[0] = 1;
  grasp.pre_grasp_posture.points[0].positions[1] = 1.0;
  grasp.pre_grasp_posture.points[0].positions[2] = 0.477;
  grasp.pre_grasp_posture.points[0].positions[3] = 0.477;
  grasp.pre_grasp_posture.points[0].positions[4] = 0.477;
  grasp.pre_grasp_posture.points[0].positions[5] = 0.477;

  // Close gripper
  grasp.grasp_posture.joint_names.resize(6);
  grasp.grasp_posture.joint_names[0] = "r_gripper_joint";
  grasp.grasp_posture.joint_names[1] = "r_gripper_motor_screw_joint";
  grasp.grasp_posture.joint_names[2] = "r_gripper_l_finger_joint";
  grasp.grasp_posture.joint_names[3] = "r_gripper_r_finger_joint";
  grasp.grasp_posture.joint_names[4] = "r_gripper_r_finger_tip_joint";
  grasp.grasp_posture.joint_names[5] = "r_gripper_l_finger_tip_joint";

  grasp.grasp_posture.points.resize(1);
  grasp.grasp_posture.points[0].positions.resize(6);
  grasp.grasp_posture.points[0].positions[0] = 0;
  grasp.grasp_posture.points[0].positions[1] = 0;
  grasp.grasp_posture.points[0].positions[2] = 0.002;
  grasp.grasp_posture.points[0].positions[3] = 0.002;
  grasp.grasp_posture.points[0].positions[4] = 0.002;
  grasp.grasp_posture.points[0].positions[5] = 0.002;

  grasp.max_contact_force = 0.001;

  async_spinner_ptr_->start();
  group_ptr_->setSupportSurfaceName("table");
  moveit_msgs::MoveItErrorCodes result = group_ptr_->pick("object", grasp);
  async_spinner_ptr_->stop();

  return result.val;
}

int MoveGroupManager::place(const geometry_msgs::Pose& pose)
{
  moveit_msgs::PlaceLocation location;

  location.place_pose.header.frame_id = "odom_combined";
  location.place_pose.pose = pose;

  location.pre_place_approach.direction.header.frame_id = "odom_combined";
  location.pre_place_approach.direction.vector.z = -1.0;
  location.pre_place_approach.min_distance = 0.1;
  location.pre_place_approach.desired_distance = 0.25;

  location.post_place_retreat.direction.header.frame_id = "odom_combined";
  location.post_place_retreat.direction.vector.x = -1.0;
  location.post_place_retreat.min_distance = 0.1;
  location.post_place_retreat.desired_distance = 0.25;

  // Open gripper
  location.post_place_posture.joint_names.resize(6);
  location.post_place_posture.joint_names[0] = "r_gripper_joint";
  location.post_place_posture.joint_names[1] = "r_gripper_motor_screw_joint";
  location.post_place_posture.joint_names[2] = "r_gripper_l_finger_joint";
  location.post_place_posture.joint_names[3] = "r_gripper_r_finger_joint";
  location.post_place_posture.joint_names[4] = "r_gripper_r_finger_tip_joint";
  location.post_place_posture.joint_names[5] = "r_gripper_l_finger_tip_joint";

  location.post_place_posture.points.resize(1);
  location.post_place_posture.points[0].positions.resize(6);
  location.post_place_posture.points[0].positions[0] = 1;
  location.post_place_posture.points[0].positions[1] = 1.0;
  location.post_place_posture.points[0].positions[2] = 0.477;
  location.post_place_posture.points[0].positions[3] = 0.477;
  location.post_place_posture.points[0].positions[4] = 0.477;
  location.post_place_posture.points[0].positions[5] = 0.477;

  std::vector<moveit_msgs::PlaceLocation> locations;
  locations.push_back(location);

  geometry_msgs::PoseStamped ps;
  ps.header.frame_id = "odom_combined";
  ps.pose = pose;

  async_spinner_ptr_->start();
  group_ptr_->setSupportSurfaceName("box");
  moveit_msgs::MoveItErrorCodes result = group_ptr_->place("object", ps);
  async_spinner_ptr_->stop();

  return result.val;
}

void MoveGroupManager::openGripper()
{
  std::map<std::string, double> values;
  values["r_gripper_joint"] = 1;
  values["r_gripper_motor_screw_joint"] = 1;
  values["r_gripper_l_finger_joint"] = 0.477;
  values["r_gripper_r_finger_joint"] = 0.477;
  values["r_gripper_r_finger_tip_joint"] = 0.477;
  values["r_gripper_l_finger_tip_joint"] = 0.477;

  async_spinner_ptr_->start();
  right_gripper_ptr_->setJointValueTarget(values);
  right_gripper_ptr_->move();
  async_spinner_ptr_->stop();

  group_ptr_->detachObject("object");
}

}