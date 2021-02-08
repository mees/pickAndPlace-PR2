#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

// MoveIt!
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/move_group_interface/move_group.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include <geometry_msgs/PointStamped.h>

#include <planning_scene_manager/planning_scene_manager.h>
#include <move_group_manager/move_group_manager.h>
#include <moveit/warehouse/state_storage.h>
#include <std_msgs/Bool.h>

#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>

#include <gpg/cloud_camera.h>
#include <gpd/grasp_detector.h>
#include <gpd/grasp_plotter.h>
#include <eigen_conversions/eigen_msg.h>

#include <pr2_controllers_msgs/PointHeadAction.h>

// Our Action interface type, provided as a typedef for convenience
typedef actionlib::SimpleActionClient <pr2_controllers_msgs::PointHeadAction> PointHeadClient;
PointHeadClient *point_head_client_;

using namespace std;
boost::shared_ptr <moveit::planning_interface::MoveGroup> right_gripper_ptr_;

void pCallback(const geometry_msgs::PointStampedConstPtr &p_msg, geometry_msgs::PointStamped &point_out) {
    point_out = *p_msg;
}

bool place(planning_scene_manager::PlanningSceneManager scene_mgr,
           move_group_manager::MoveGroupManager &group_mgr, geometry_msgs::PointStamped placing_point,
           moveit_msgs::DisplayTrajectory display_trajectory, ros::Publisher display_publisher) {
    geometry_msgs::Pose place_pose_msg;
    place_pose_msg.position.x = placing_point.point.x;
    place_pose_msg.position.y = placing_point.point.y;
    place_pose_msg.position.z = placing_point.point.z;
    place_pose_msg.orientation = tf::createQuaternionMsgFromRollPitchYaw(M_PI / 2, M_PI / 2, 0); //top down grasp
    moveit::planning_interface::MoveGroup::Plan my_plan;
    bool success = false;
    if (group_mgr.plan("odom_combined", "r_wrist_roll_link", place_pose_msg, my_plan)) {
        group_mgr.execute(my_plan);
        display_trajectory.trajectory_start = my_plan.start_state_;
        display_trajectory.trajectory.push_back(my_plan.trajectory_);
        display_publisher.publish(display_trajectory);
        success = true;
        ROS_INFO("Opening gripper...");
        group_mgr.openGripperGently();
    } else
        ROS_WARN("Place object failed.");
    scene_mgr.removeCollisionObject("r_wist_roll_link", "object");
    scene_mgr.removeCollisionObject("odom_combined", "object");
    return success;
}

bool LoadRobotStateFromStorage(std::string &state_id, moveit::core::RobotState &state) {
    moveit_warehouse::RobotStateWithMetadata state_msg;
    moveit_warehouse::RobotStateStorage state_storage;
    if (!state_storage.getRobotState(state_msg, state_id, "")) {
        ROS_ERROR("Could not find state %s in storage.", state_id.c_str());
        return false;
    }
    moveit::core::robotStateMsgToRobotState(*state_msg, state);
    return true;
}

//! Points the high-def camera frame at a point in a given frame  
void lookAt(std::string frame_id, double x, double y, double z) {
    //the goal message we will be sending
    pr2_controllers_msgs::PointHeadGoal goal;

    //the target point, expressed in the requested frame
    geometry_msgs::PointStamped point;
    point.header.frame_id = frame_id;
    point.point.x = x;
    point.point.y = y;
    point.point.z = z;
    goal.target = point;

    //we are pointing the high-def camera frame 
    //(pointing_axis defaults to X-axis)
    goal.pointing_frame = "high_def_frame";
    goal.pointing_axis.x = 1;
    goal.pointing_axis.y = 0;
    goal.pointing_axis.z = 0;

    //take at least 0.5 seconds to get there
    goal.min_duration = ros::Duration(0.5);

    //and go no faster than 1 rad/s
    goal.max_velocity = 1.0;

    //send the goal
    point_head_client_->sendGoal(goal);

    //wait for it to get there (abort after 2 secs to prevent getting stuck)
    point_head_client_->waitForResult(ros::Duration(2));
}

bool moveToInitState(moveit::planning_interface::MoveGroup &arm_group, moveit::core::RobotState &robot_state) {
    std::string state_id = "twoTableView4";
    if (!LoadRobotStateFromStorage(state_id, robot_state)) {
        ROS_ERROR("Can't load robot state");
        return false;
    } else {
        ROS_INFO("Loaded init robot state");
    }
    ROS_INFO("Moving robot head");
    lookAt("odom_combined", 3.0, -3.0, -7);
    usleep(50000);

// Move the arm.
    arm_group.setJointValueTarget(robot_state);
    moveit::planning_interface::MoveGroup::Plan my_plan;
    arm_group.setNumPlanningAttempts(10.0);
    arm_group.setPlanningTime(5.0);
    arm_group.setGoalOrientationTolerance(0.18);
    arm_group.setGoalPositionTolerance(0.08);
    ROS_INFO_STREAM("Try to move to state " << state_id);
    moveit_msgs::MoveItErrorCodes success = arm_group.plan(my_plan);
    std::cout << "success: " << success << std::endl;
    if (success.val == moveit_msgs::MoveItErrorCodes::SUCCESS) {
        ROS_INFO("Success! Now move");
        success = arm_group.move();
    }
    return true;
}

bool moveToPlacingState(moveit::planning_interface::MoveGroup &arm_group, moveit::core::RobotState &robot_state) {
    std::string state_id = "twoTableView4";

    if (!LoadRobotStateFromStorage(state_id, robot_state)) {
        ROS_ERROR("Can't load robot state");
        return false;
    } else {
        ROS_INFO("Loaded init robot state");
    }
// Move the arm.
    arm_group.setJointValueTarget(robot_state);
    moveit::planning_interface::MoveGroup::Plan my_plan;
    arm_group.setNumPlanningAttempts(10.0);
    arm_group.setPlanningTime(5.0);
    arm_group.setGoalOrientationTolerance(0.18);
    arm_group.setGoalPositionTolerance(0.08);
    ROS_INFO_STREAM("Try to move to state " << state_id);
    moveit_msgs::MoveItErrorCodes success = arm_group.plan(my_plan);
    std::cout << "success: " << success << std::endl;
    if (success.val == moveit_msgs::MoveItErrorCodes::SUCCESS) {
        ROS_INFO("Success! Now move");
        success = arm_group.move();
    }
    return true;
}


void set_gpd_params(ros::NodeHandle &nh, const moveit_msgs::CollisionObject &table) {
    const double THRESH = 0.02;
    geometry_msgs::Pose table_pose = table.primitive_poses.at(0);
    std::vector<double> table_dims = table.primitives.at(0).dimensions;
    std::vector<double> workspace(6);
    workspace[0] = table_pose.position.x - 0.5 * table_dims[0];
    workspace[1] = table_pose.position.x + 0.5 * table_dims[0];
    workspace[2] = table_pose.position.y - 0.5 * table_dims[1] + THRESH;
    workspace[3] = table_pose.position.y + 0.5 * table_dims[1];
    workspace[4] = table_pose.position.z - 0.5 * table_dims[2] - THRESH;
    workspace[5] = table_pose.position.z + 1.0;
    nh.setParam("workspace", workspace);
    nh.setParam("workspace_grasps", workspace);
    nh.setParam("table_height", table.primitive_poses.at(0).position.z);
}


bool transform_cloud(const pcl::PointCloud <pcl::PointXYZRGB> &cloud_in,
                     const tf2_ros::Buffer &tf_buffer, const std::string &frame_id, Eigen::Affine3d &transform,
                     pcl::PointCloud <pcl::PointXYZRGB> &cloud_out) {
    try {
        geometry_msgs::TransformStamped transform_msg = tf_buffer.lookupTransform(
                frame_id, cloud_in.header.frame_id, pcl_conversions::fromPCL(cloud_in.header.stamp));
        transform = tf2::transformToEigen(transform_msg);
        pcl::transformPointCloud(cloud_in, cloud_out, transform);
    }
    catch (tf2::TransformException &ex) {
        std::cout << "frame_id: " << frame_id << std::endl;
        ROS_WARN("%s", ex.what());
        return false;
    }
    return true;
}

void callback_cloud(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg_ptr,
                    pcl::PointCloud <pcl::PointXYZRGB> &cloud_out) {
    pcl::fromROSMsg(*cloud_msg_ptr, cloud_out);
}

bool detect_grasps2Cams(const Eigen::Vector3d &view_point, const Eigen::Vector3d &view_point2,
                        const pcl::PointCloud <pcl::PointXYZRGB> cloud,
                        GraspDetector &grasp_detector, std::vector <Grasp> &grasps)//, int cloud_size)
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ptr(new pcl::PointCloud <pcl::PointXYZRGBA>);
    pcl::copyPointCloud(cloud, *cloud_ptr);
    Eigen::Matrix3Xd view_points(3, 2);
    view_points.col(0) = view_point;
    view_points.col(1) = view_point2;
    int cloud_size = 0;
    CloudCamera cloud_camera(cloud_ptr, cloud_size, view_points);
    grasp_detector.preprocessPointCloud(cloud_camera);
    grasps = grasp_detector.detectGrasps(cloud_camera);
    if (grasps.empty()) {
        ROS_WARN("Could not detect any grasps.");
        return false;
    }
    return true;
}

bool detect_grasps(const Eigen::Vector3d &view_point, const pcl::PointCloud <pcl::PointXYZRGB> cloud,
                   GraspDetector &grasp_detector, std::vector <Grasp> &grasps) {
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_ptr(new pcl::PointCloud <pcl::PointXYZRGBA>);
    pcl::copyPointCloud(cloud, *cloud_ptr);
    Eigen::Matrix3Xd view_points(3, 1);
    view_points.col(0) = view_point;
    CloudCamera cloud_camera(cloud_ptr, 0, view_points);
    grasp_detector.preprocessPointCloud(cloud_camera);
    grasps = grasp_detector.detectGrasps(cloud_camera);
    if (grasps.empty()) {
        ROS_WARN("Could not detect any grasps.");
        return false;
    }

    return true;
}

bool grasp(planning_scene_manager::PlanningSceneManager scene_mgr,
           move_group_manager::MoveGroupManager &group_mgr, const std::vector <Grasp> grasps) {
    for (size_t i = 0; i < grasps.size(); i++) {
        Eigen::Affine3d object_pose = Eigen::Affine3d::Identity();
        object_pose.translation() = grasps[i].getGraspBottom();

        Eigen::Affine3d grasp_pose = Eigen::Affine3d::Identity();
        grasp_pose.translation() = -0.14 * grasps[i].getApproach() + grasps[i].getGraspBottom();
        grasp_pose.linear() << grasps[i].getApproach(), grasps[i].getBinormal(), grasps[i].getAxis();
        geometry_msgs::Pose grasp_pose_msg;
        tf::poseEigenToMsg(grasp_pose, grasp_pose_msg);
        geometry_msgs::Vector3 approach_msg;
        tf::vectorEigenToMsg(grasps[i].getApproach(), approach_msg);
        scene_mgr.addSphereCollisionObject("odom_combined", "object", object_pose, 0.10);
        scene_mgr.allowCollision("object");
        if (group_mgr.pickGently(grasp_pose_msg, approach_msg)) {
            return true;
        } else {
            scene_mgr.removeCollisionObject("r_wist_roll_link", "object");
            scene_mgr.removeCollisionObject("odom_combined", "object");
        }
    }

    ROS_WARN("Tried all grasps without success.");
    return false;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "move_group_placing");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::NodeHandle node_handle, nh_private("~");;
    geometry_msgs::PointStamped placing_point;
    geometry_msgs::PointStamped placing_point_transformed;
    ros::Subscriber p_sub = node_handle.subscribe<geometry_msgs::PointStamped>("placing_pose_pub", 1,
                                                                               boost::bind(pCallback, _1,
                                                                                           boost::ref(placing_point)));
    pcl::PointCloud <pcl::PointXYZRGB> cloud;
    ros::Subscriber sub_cloud = node_handle.subscribe<sensor_msgs::PointCloud2>(
            "refexp_object", 1, boost::bind(callback_cloud, _1, boost::ref(cloud)));
    ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>(
            "/move_group/display_planned_path", 1, true);
    ros::Publisher placing_pose_pub = node_handle.advertise<geometry_msgs::PointStamped>("placing_pose", 1);
    ros::Publisher goal_reached_pub = node_handle.advertise<std_msgs::Bool>("placing_pose_reached", 1);
    ros::Publisher picking_reached_pub = node_handle.advertise<std_msgs::Bool>("picking_reached", 1);
    moveit_msgs::DisplayTrajectory display_trajectory;
    const std::string cam_frame_id = "head_mount_asus_rgb_optical_frame";
    const std::string base_frame_id = "odom_combined";
    tf::TransformListener _tf_listener;
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);
    bool use_gpd = true;
    moveit_msgs::CollisionObject table;
    planning_scene_manager::PlanningSceneManager scene_mgr;
    move_group_manager::MoveGroupManager group_mgr;
    planning_scene_monitor::PlanningSceneMonitorPtr psm_ptr = planning_scene_monitor::PlanningSceneMonitorPtr(
            new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
    boost::shared_ptr <moveit::planning_interface::MoveGroup> arm_group(
            new moveit::planning_interface::MoveGroup("arms"));
    arm_group->setPoseReferenceFrame(base_frame_id);
    if (!scene_mgr.getCollisionObject("table", table))
        return EXIT_FAILURE;
    geometry_msgs::Pose table_pose = table.primitive_poses.at(0);
    std::vector<double> table_dims = table.primitives.at(0).dimensions;
    node_handle.setParam("table_height", table.primitive_poses.at(0).position.z);
    cout << "table height: " << table.primitive_poses.at(0).position.z << endl;

    set_gpd_params(nh_private, table);
    GraspDetector grasp_detector(nh_private);
    GraspPlotter grasp_plotter(nh_private, grasp_detector.getHandSearchParameters());
    point_head_client_ = new PointHeadClient("/head_traj_controller/point_head_action", true);
    //wait for head controller action server to come up
    while (!point_head_client_->waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for the point_head_action server to come up");
    }
    ros::Duration(1.0).sleep();
    bool grasp_cmd_recv = false;
    bool place_cmd_recv = false;
    geometry_msgs::TransformStamped transformStamped;
    Eigen::Affine3d transformPR2, transformCam2;
    ros::Rate rate(1);
    psm_ptr->requestPlanningSceneState();
    moveit::core::RobotState robot_state(psm_ptr->getRobotModel());
    moveToInitState(*arm_group, robot_state);
    group_mgr.openGripperGently();
    while (ros::ok()) {
        ROS_INFO("Wait for new picking object cloud ...");
        ros::Time time = ros::Time::now();
        while (ros::ok()) {
            ros::spinOnce();

            if (cloud.header.stamp >= pcl_conversions::toPCL(time)) {
                grasp_cmd_recv = true;
                break;
            }
            if (placing_point.header.stamp >= time) {
                place_cmd_recv = true;
                break;
            }
            rate.sleep();
        }
        if (grasp_cmd_recv) {
            group_mgr.openGripperGently();
            ROS_INFO("Transform point cloud to frame '%s'.", base_frame_id.c_str());
            Eigen::Affine3d transform;
            if (!transform_cloud(cloud, tf_buffer, base_frame_id, transform, cloud))
                continue;
            std::vector<int> indices;
            pcl::removeNaNFromPointCloud(cloud, cloud, indices);
            try {
                transformStamped = tf_buffer.lookupTransform(base_frame_id, "head_mount_asus_rgb_optical_frame",
                                                             ros::Time(0));
                transformPR2 = tf2::transformToEigen(transformStamped);

                transformStamped = tf_buffer.lookupTransform(base_frame_id, "camera2_link",
                                                             ros::Time(0));
                transformCam2 = tf2::transformToEigen(transformStamped);
            }
            catch (tf2::TransformException &ex) {
                ROS_WARN("could not find transforms");
                ROS_WARN("%s", ex.what());
                ros::Duration(1.0).sleep();
                continue;
            }
            if (use_gpd) {
                std_msgs::Bool msg_pick;
                msg_pick.data = false;
                ROS_INFO("Detect grasps.");
                std::vector <Grasp> grasps;
                if (!detect_grasps2Cams(transformPR2.translation(), transformCam2.translation(), cloud, grasp_detector,
                                        grasps))
                    // if (!detect_grasps(transform.translation(), cloud, grasp_detector, grasps))
                {
                    picking_reached_pub.publish(msg_pick);
                    continue;
                }

                ROS_INFO("Publish grasps to Rviz.");
                grasp_plotter.drawGrasps(grasps, base_frame_id);

                ROS_INFO("Grasp object.");
                if (!grasp(scene_mgr, group_mgr, grasps)) {
                    moveit::core::RobotState robot_state(psm_ptr->getRobotModel());
                    moveToInitState(*arm_group, robot_state);
                    ROS_INFO("Grasp failed, moving to init state");
                    picking_reached_pub.publish(msg_pick);
                    continue;
                }
                ROS_INFO("Grasped object, moving to placing state");
                msg_pick.data = true;
                picking_reached_pub.publish(msg_pick);
                ROS_INFO("test head movement");
                lookAt("odom_combined", 3.0, 0, -7);
                usleep(50000);
                psm_ptr->requestPlanningSceneState();
                moveit::core::RobotState robot_state(psm_ptr->getRobotModel());
                if (!moveToPlacingState(*arm_group, robot_state)) {
                    cout << "trying to move to placing state one more time..." << endl;
                    moveToPlacingState(*arm_group, robot_state);
                }
            }
            grasp_cmd_recv = false;
        }

        if (place_cmd_recv) {
            ROS_INFO("Wait for new placing pose ...");
            time = ros::Time::now();
            while (ros::ok()) {
                ros::spinOnce();

                if (placing_point.header.stamp >= time)
                    break;

                rate.sleep();
            }
            std_msgs::Bool msg1;
            msg1.data = false;
            //transform points to odom_combined
            _tf_listener.waitForTransform(base_frame_id, cam_frame_id,
                                          placing_point.header.stamp, ros::Duration(10.0));
            _tf_listener.transformPoint(base_frame_id, placing_point, placing_point_transformed);
            if (placing_point_transformed.point.x != placing_point_transformed.point.x) //if nan
            {
                goal_reached_pub.publish(msg1);
                continue;
            }
            if (placing_point_transformed.point.z < table.primitive_poses.at(0).position.z &&
                placing_point_transformed.point.z > 0.9) {//if point is under the table
                goal_reached_pub.publish(msg1);
                continue;
            }
            placing_point_transformed.point.z += 0.30;  //so we dont place inside the table and account for the gripper
            float x1 = table_pose.position.x - 0.5 * table_dims[0];
            float x2 = table_pose.position.x + 0.5 * table_dims[0];
            float y1 = table_pose.position.y - 0.5 * table_dims[1];
            float y2 = table_pose.position.y + 0.5 * table_dims[1];
            cout << placing_point_transformed.point.x << ", " << placing_point_transformed.point.y << ", "
                 << placing_point_transformed.point.z << ", " << endl;
            placing_pose_pub.publish(placing_point_transformed);
            if (placing_point_transformed.point.x > x1 && placing_point_transformed.point.x < x2) {
                if (placing_point_transformed.point.y > y1 && placing_point_transformed.point.y < y2) {
                    cout << "point lies on the table" << endl;
                    ROS_INFO("Place object.");
                    bool success = place(scene_mgr, group_mgr, placing_point_transformed, display_trajectory,
                                         display_publisher);
                    psm_ptr->requestPlanningSceneState();
                    if (success) {
                        moveit::core::RobotState robot_state(psm_ptr->getRobotModel());
                        if (!moveToInitState(*arm_group, robot_state)) {
                            cout << "trying to move to init state one more time..." << endl;
                            moveToInitState(*arm_group, robot_state);
                        }

                        msg1.data = true;
                        goal_reached_pub.publish(msg1);

                    }
                } else {
                    goal_reached_pub.publish(msg1);
                }
            } else {
                goal_reached_pub.publish(msg1);
            }
            place_cmd_recv = false;
        }
        rate.sleep();
    }
}
