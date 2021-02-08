#include <ros/ros.h>
#include <rviz_visualizer/rviz_visualizer.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "rviz_visualizer");
  ros::NodeHandle nh_public, nh_private("~");
  rviz_visualizer::RvizVisualizer rviz_visualizer("odom_combined", "markers", nh_private);
  rviz_visualizer.setAlpha(0.5);
  std::vector<Eigen::Vector3d> points;
  points.push_back(Eigen::Vector3d(1, 0, 0));
  points.push_back(Eigen::Vector3d(2, 1, 0));
  points.push_back(Eigen::Vector3d(1, 2, 0));
  points.push_back(Eigen::Vector3d(2, 3, 0));
  while (ros::ok()) {
    rviz_visualizer.publishCube(Eigen::Vector3d(0, 0, 0), Eigen::Quaterniond::Identity(), Eigen::Vector3d(1, 1, 1),
                                rviz_visualizer::GREEN);
    rviz_visualizer.publishLineStrip(points, Eigen::Quaterniond::Identity(), 0.1, rviz_visualizer::RED);
    rviz_visualizer.publish();
    ros::Rate(1).sleep();
  }
}