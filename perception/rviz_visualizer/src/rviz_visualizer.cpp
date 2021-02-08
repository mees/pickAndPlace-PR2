#include <eigen_conversions/eigen_msg.h>
#include <rviz_visualizer/rviz_visualizer.h>
#include <visualization_msgs/Marker.h>

namespace rviz_visualizer {

RvizVisualizer::RvizVisualizer(const std::string& frame_id, const std::string& topic, ros::NodeHandle& nh) {
  frame_id_ = frame_id;
  topic_ = topic;
  lifetime_ = 0;
  alpha_ = 1;
  pub_markers_ = nh.advertise<visualization_msgs::MarkerArray>(topic_, 10);
}

void RvizVisualizer::publish() {
  pub_markers_.publish(markers_);
  markers_.markers.clear();
  last_id_.clear();
}

void RvizVisualizer::publishCube(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation,
                                 const Eigen::Vector3d& scale, const Colors& color, const std::string& ns) {
  visualization_msgs::Marker marker;
  initMarker(marker, ns, visualization_msgs::Marker::CUBE, visualization_msgs::Marker::ADD);
  tf::pointEigenToMsg(position, marker.pose.position);
  tf::quaternionEigenToMsg(orientation, marker.pose.orientation);
  tf::vectorEigenToMsg(scale, marker.scale);
  colorToColorMsg(color, marker.color);
  markers_.markers.push_back(marker);
}

void RvizVisualizer::publishLineStrip(const std::vector<Eigen::Vector3d>& points, const Eigen::Quaterniond& orientation,
                                      double scale, const Colors& color, const std::string& ns) {
  visualization_msgs::Marker marker;
  initMarker(marker, ns, visualization_msgs::Marker::LINE_STRIP, visualization_msgs::Marker::ADD);
  pointListEigenToMsg(points, marker.points);
  tf::quaternionEigenToMsg(orientation, marker.pose.orientation);
  marker.scale.x = scale;
  colorToColorMsg(color, marker.color);
  markers_.markers.push_back(marker);
}

void RvizVisualizer::publishPolygon(const std::vector<Eigen::Vector3d>& points, const Eigen::Quaterniond& orientation,
                                    double scale, const Colors& color, const std::string& ns) {
  std::vector<Eigen::Vector3d> polygon = points;
  if (polygon.size() > 0) polygon.push_back(polygon.front());
  publishLineStrip(polygon, orientation, scale, color, ns);
}

void RvizVisualizer::initMarker(visualization_msgs::Marker& marker, const std::string& ns, int type, int action) {
  marker.header.frame_id = frame_id_;
  marker.header.stamp = ros::Time::now();
  marker.ns = ns;
  marker.id = last_id_[ns]++;
  marker.type = type;
  marker.action = action;
  marker.lifetime = ros::Duration(lifetime_);
}

void RvizVisualizer::colorToColorMsg(const Colors& color, std_msgs::ColorRGBA& color_msg) {
  switch (color) {
    case BLACK:
      color_msg.r = 0.0;
      color_msg.g = 0.0;
      color_msg.b = 0.0;
      color_msg.a = alpha_;
      break;

    case BLUE:
      color_msg.r = 0.0;
      color_msg.g = 0.0;
      color_msg.b = 1.0;
      color_msg.a = alpha_;
      break;

    case GREEN:
      color_msg.r = 0.0;
      color_msg.g = 1.0;
      color_msg.b = 0.0;
      color_msg.a = alpha_;
      break;

    case RED:
      color_msg.r = 1.0;
      color_msg.g = 0.0;
      color_msg.b = 0.0;
      color_msg.a = alpha_;
      break;

    case WHITE:
      color_msg.r = 1.0;
      color_msg.g = 1.0;
      color_msg.b = 1.0;
      color_msg.a = alpha_;
      break;
  }
}

void RvizVisualizer::pointListEigenToMsg(const std::vector<Eigen::Vector3d>& points,
                                         std::vector<geometry_msgs::Point>& points_msg) {
  points_msg.resize(points.size());

  for (size_t i = 0; i < points.size(); i++) tf::pointEigenToMsg(points[i], points_msg[i]);
}

}  // namespace rviz_visualizer