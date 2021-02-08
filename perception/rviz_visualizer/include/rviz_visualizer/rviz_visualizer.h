#ifndef RVIZ_VISUALIZER_RVIZ_VISUALIZER_H
#define RVIZ_VISUALIZER_RVIZ_VISUALIZER_H

#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Geometry>

namespace rviz_visualizer
{

enum Colors
{
  BLACK,
  BLUE,
  GREEN,
  RED,
  WHITE
};

class RvizVisualizer
{

public:

  typedef boost::shared_ptr<RvizVisualizer> Ptr;
  typedef boost::shared_ptr<const RvizVisualizer> ConstPtr;

  RvizVisualizer(const std::string& frame_id, const std::string& topic, ros::NodeHandle& nh);

  void setLifetime(double lifetime) { lifetime_ = lifetime; }

  void setAlpha(double alpha) { alpha_ = alpha; }

  void publish();

  void publishCube(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation,
    const Eigen::Vector3d& scale, const Colors& color, const std::string& ns = "Cube");

  void publishLineStrip(const std::vector<Eigen::Vector3d>& points,
    const Eigen::Quaterniond& orientation, double scale, const Colors& color,
    const std::string& ns = "Line Strip");

  void publishPolygon(const std::vector<Eigen::Vector3d>& points,
    const Eigen::Quaterniond& orientation, double scale, const Colors& color,
    const std::string& ns = "Polygon");

private:

  void initMarker(visualization_msgs::Marker& marker, const std::string& ns, int type, int action);

  void colorToColorMsg(const Colors& color, std_msgs::ColorRGBA& color_msg);

  void pointListEigenToMsg(const std::vector<Eigen::Vector3d>& points,
    std::vector<geometry_msgs::Point>& points_msg);

  std::string getNamespace() { return "rviz_visualizer"; }

  std::string getName() { return "RvizVisualizer"; }


  std::string frame_id_;
  std::string topic_;
  double lifetime_;
  double alpha_;

  std::map<std::string, size_t> last_id_;
  visualization_msgs::MarkerArray markers_;
  ros::Publisher pub_markers_;

};

}

#endif
