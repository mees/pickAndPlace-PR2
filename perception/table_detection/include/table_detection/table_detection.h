#ifndef TABLE_DETECTION_TABLE_DETECTION_H
#define TABLE_DETECTION_TABLE_DETECTION_H

#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/convex_hull.h>
#include <pcl_ros/point_cloud.h>
#include <planning_scene_manager/planning_scene_manager.h>
#include <ros/ros.h>
#include <rviz_visualizer/rviz_visualizer.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>

namespace table_detection {

struct Table {
  std::string frame_id;
  Eigen::Affine3d pose;
  Eigen::Vector3d dimensions;
};

class TableDetection {
 public:
  TableDetection();

  Table getTable() { return table_; }
  void run(const std::string& topic);
  void stop();
  void detect(const sensor_msgs::PointCloud2& cloud_msg);

 private:
  void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr);
  bool transform(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr, sensor_msgs::PointCloud2& cloud_msg);
  void downsample();
  void cropBox();
  void estimatePlaneCoeffs();
  void extractCluster();
  void projectPointCloudToModel();
  void computeConvexHull();
  void computeBounds();
  void publish();
  void extractOutliers();
  std::string nameSpace() { return "table_detection"; };
  std::string className() { return "TableDetection"; };
  std::string name() { return nameSpace() + "::" + className(); };
  std::string frame_id_;
  bool detect_continuous_;
  float voxel_grid_size_;
  std::vector<float> crop_box_;
  std::vector<float> up_vector_;
  float up_vector_thresh_;
  float inlier_thresh_;
  int min_cluster_size_;
  float cluster_tolerance_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_ptr_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_bounds_ptr_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_cloud;
  pcl::PointIndices::Ptr inlier_ptr_;
  // pcl::PointIndices::Ptr outlier_ptr_;
  pcl::ModelCoefficients model_coeffs_;
  Table table_;
  ros::NodeHandle nh_public_;
  ros::NodeHandle nh_private_;
  ros::Subscriber sub_cloud_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  planning_scene_manager::PlanningSceneManager scene_mgr_;
  rviz_visualizer::RvizVisualizer::Ptr rviz_visualizer_ptr_;
  ros::Publisher pub_cloud_;
  ros::Publisher model_coeff_pub;
};

}  // namespace table_detection

#endif