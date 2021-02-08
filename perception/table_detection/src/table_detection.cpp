#include <geometry_msgs/TransformStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <table_detection/table_detection.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

namespace table_detection {

TableDetection::TableDetection() : nh_private_("~"), tf_listener_(tf_buffer_) {
  float cb_arr[] = {0.0, 2.0, -1.0, 1.0, 0.1, 1.0};
  std::vector<float> crop_box_default(cb_arr, cb_arr + sizeof(cb_arr) / sizeof(float));
  float uv_arr[] = {0.0, 0.0, 1.0};
  std::vector<float> up_vector_default(uv_arr, uv_arr + sizeof(uv_arr) / sizeof(float));
  nh_private_.param<std::string>("frame_id", frame_id_, "odom_combined");
  nh_private_.param<bool>("detect_continuous", detect_continuous_, false);
  nh_private_.param<float>("voxel_grid_size", voxel_grid_size_, 0.01);
  nh_private_.param<std::vector<float> >("crop_box", crop_box_, crop_box_default);
  nh_private_.param<std::vector<float> >("up_vector", up_vector_, up_vector_default);
  nh_private_.param<float>("up_vector_thresh", up_vector_thresh_, 0.087);
  nh_private_.param<float>("inlier_thresh", inlier_thresh_, 0.02);
  nh_private_.param<int>("min_cluster_size", min_cluster_size_, 100);
  nh_private_.param<float>("cluster_tolerance", cluster_tolerance_, 0.04);
  cloud_ptr_.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
  cloud_filtered_ptr_.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
  cloud_bounds_ptr_.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
  inlier_ptr_.reset(new pcl::PointIndices());
  object_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
  rviz_visualizer_ptr_.reset(new rviz_visualizer::RvizVisualizer(frame_id_, "markers", nh_private_));
  rviz_visualizer_ptr_->setAlpha(0.5);
  pub_cloud_ = nh_public_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("debug", 10);
  model_coeff_pub = nh_public_.advertise<pcl_msgs::ModelCoefficients>("model_coeff_pub", 1);
}

void TableDetection::run(const std::string& topic) {
  ROS_INFO("[%s]: Subscribe to topic '%s'.", name().c_str(), topic.c_str());
  sub_cloud_ = nh_public_.subscribe<sensor_msgs::PointCloud2>(topic, 1, &TableDetection::cloudCallback, this);
  if (sub_cloud_.getNumPublishers() == 0)
    ROS_WARN("[%s]: No publishers for topic '%s' detected.", name().c_str(), topic.c_str());
}

void TableDetection::stop() { sub_cloud_.shutdown(); }

void TableDetection::detect(const sensor_msgs::PointCloud2& cloud_msg) {
  pcl::fromROSMsg(cloud_msg, *cloud_ptr_);
  if (cloud_ptr_->empty()) {
    ROS_WARN("[%s]: Point cloud is empty.", name().c_str());
    return;
  }
  inlier_ptr_->indices.clear();
  inlier_ptr_->header = cloud_ptr_->header;
  // outlier_ptr_->indices.clear();
  // outlier_ptr_->header = cloud_ptr_->header;
  cloud_bounds_ptr_->clear();
  cloud_bounds_ptr_->header = cloud_ptr_->header;
  object_cloud->clear();
  object_cloud->header = cloud_ptr_->header;
  pcl::copyPointCloud(*cloud_ptr_, *cloud_filtered_ptr_);
  cloud_filtered_ptr_->header = cloud_ptr_->header;

  downsample();
  cropBox();
  estimatePlaneCoeffs();
  extractCluster();
  projectPointCloudToModel();
  computeConvexHull();
  computeBounds();
  extractOutliers();
  publish();
}

void TableDetection::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr) {
  ROS_INFO("[%s]: Recived point cloud with frame id '%s'.", name().c_str(), cloud_msg_ptr->header.frame_id.c_str());

  sensor_msgs::PointCloud2 cloud_msg;
  if (transform(cloud_msg_ptr, cloud_msg)) detect(cloud_msg);
}

bool TableDetection::transform(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr,
                               sensor_msgs::PointCloud2& cloud_msg) {
  try {
    geometry_msgs::TransformStamped transform =
        tf_buffer_.lookupTransform(frame_id_, cloud_msg_ptr->header.frame_id, cloud_msg_ptr->header.stamp);
    tf2::doTransform(*cloud_msg_ptr, cloud_msg, transform);
  } catch (tf2::TransformException& ex) {
    ROS_WARN("[%s]: %s", name().c_str(), ex.what());
    return false;
  }

  return true;
}

void TableDetection::downsample() {
  if (cloud_filtered_ptr_->empty()) return;

  pcl::VoxelGrid<pcl::PointXYZRGB> vg;
  vg.setInputCloud(cloud_filtered_ptr_->makeShared());
  vg.setLeafSize(voxel_grid_size_, voxel_grid_size_, voxel_grid_size_);
  vg.filter(*cloud_filtered_ptr_);

  if (cloud_filtered_ptr_->empty()) ROS_WARN("[%s]: Point cloud is empty after downsample.", name().c_str());
}

void TableDetection::cropBox() {
  if (cloud_filtered_ptr_->empty()) return;

  pcl::CropBox<pcl::PointXYZRGB> cb;
  cb.setInputCloud(cloud_filtered_ptr_);
  cb.setMin(Eigen::Vector4f(crop_box_[0], crop_box_[2], crop_box_[4], 0));
  cb.setMax(Eigen::Vector4f(crop_box_[1], crop_box_[3], crop_box_[5], 0));
  cb.filter(inlier_ptr_->indices);

  if (inlier_ptr_->indices.empty()) ROS_WARN("[%s]: Point cloud is empty after crop.", name().c_str());
}

void TableDetection::estimatePlaneCoeffs() {
  if (cloud_filtered_ptr_->empty() || inlier_ptr_->indices.empty()) return;

  pcl::SACSegmentation<pcl::PointXYZRGB> sac;
  sac.setInputCloud(cloud_filtered_ptr_);
  sac.setIndices(inlier_ptr_);
  sac.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
  sac.setMethodType(pcl::SAC_RANSAC);
  sac.setOptimizeCoefficients(true);
  sac.setDistanceThreshold(inlier_thresh_);
  sac.setAxis(Eigen::Vector3f(up_vector_.at(0), up_vector_.at(1), up_vector_.at(2)));
  sac.setEpsAngle(up_vector_thresh_);
  sac.segment(*inlier_ptr_, model_coeffs_);

  if (inlier_ptr_->indices.empty()) ROS_WARN("[%s]: Point cloud is empty after segmentation.", name().c_str());
}

void TableDetection::extractCluster() {
  if (cloud_filtered_ptr_->empty() || inlier_ptr_->indices.empty()) return;

  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ece;
  ece.setInputCloud(cloud_filtered_ptr_);
  ece.setIndices(inlier_ptr_);
  ece.setClusterTolerance(cluster_tolerance_);
  ece.setMinClusterSize(min_cluster_size_);
  std::vector<pcl::PointIndices> clusters;
  ece.extract(clusters);

  inlier_ptr_->indices.clear();
  for (size_t i = 0; i < clusters.size(); i++) {
    inlier_ptr_->indices.insert(inlier_ptr_->indices.end(), clusters[i].indices.begin(), clusters[i].indices.end());
  }

  if (inlier_ptr_->indices.empty()) ROS_WARN("[%s]: Point cloud is empty after cluster extraction.", name().c_str());
}

void TableDetection::projectPointCloudToModel() {
  if (cloud_filtered_ptr_->empty() || inlier_ptr_->indices.empty()) return;

  pcl::ProjectInliers<pcl::PointXYZRGB> pi;
  pi.setInputCloud(cloud_filtered_ptr_);
  pi.setIndices(inlier_ptr_);
  pi.setModelType(pcl::SACMODEL_PLANE);
  pi.setModelCoefficients(boost::make_shared<pcl::ModelCoefficients>(model_coeffs_));
  pi.filter(*cloud_bounds_ptr_);

  if (cloud_bounds_ptr_->empty()) ROS_WARN("[%s]: Point cloud is empty after projection.", name().c_str());
}

void TableDetection::computeConvexHull() {
  if (cloud_bounds_ptr_->empty()) return;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr tmp_ptr(cloud_bounds_ptr_);
  pcl::ConvexHull<pcl::PointXYZRGB> ch;
  ch.setInputCloud(tmp_ptr);
  ch.reconstruct(*cloud_bounds_ptr_);

  if (cloud_bounds_ptr_->empty()) ROS_WARN("[%s]: Point cloud is empty after convex hull.", name().c_str());
}

void TableDetection::computeBounds() {
  if (cloud_filtered_ptr_->empty() || inlier_ptr_->indices.empty()) return;

  Eigen::Vector4f min, max;
  pcl::getMinMax3D(*cloud_filtered_ptr_, *inlier_ptr_, min, max);
  table_.frame_id = cloud_filtered_ptr_->header.frame_id;
  table_.pose = Eigen::Affine3d::Identity();
  table_.pose.translation() = ((min + max) / 2).head(3).cast<double>();
  table_.dimensions = (max - min).head(3).cast<double>();
  table_.dimensions.z() = 0.01;
}

void TableDetection::extractOutliers() {
  // pcl::PointCloud<pcl::PointXYZRGB> cloud_outliers;
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;
  extract.setInputCloud(cloud_filtered_ptr_);  // cloud_ptr_   cloud_filtered_ptr_
  extract.setIndices(inlier_ptr_);
  extract.setNegative(true);  // Extract the outliers
  extract.filter(*object_cloud);
  pub_cloud_.publish(object_cloud);
}

void TableDetection::publish() {
  if (cloud_filtered_ptr_->empty() || inlier_ptr_->indices.empty() || cloud_bounds_ptr_->empty()) return;

  // pcl::PointCloud<pcl::PointXYZRGB> tmp;
  // pcl::copyPointCloud(*cloud_filtered_ptr_, *inlier_ptr_, tmp);
  // pub_cloud_.publish(tmp);
  pcl_msgs::ModelCoefficients ros_coefficients;
  pcl_conversions::fromPCL(model_coeffs_, ros_coefficients);
  model_coeff_pub.publish(ros_coefficients);

  // Publish collision object.
  scene_mgr_.addBoxCollisionObject(table_.frame_id, "table", table_.pose, table_.dimensions);

  // // Publish rviz markers.
  std::vector<Eigen::Vector3d> bounds(cloud_bounds_ptr_->size());
  for (size_t i = 0; i < cloud_bounds_ptr_->size(); i++) {
    bounds[i] << cloud_bounds_ptr_->at(i).x, cloud_bounds_ptr_->at(i).y, cloud_bounds_ptr_->at(i).z;
  }
  rviz_visualizer_ptr_->publishPolygon(bounds, Eigen::Quaterniond::Identity(), 0.02, rviz_visualizer::GREEN, "Bounds");
  rviz_visualizer_ptr_->publish();
  ROS_INFO("[%s]: Published table with frame id '%s'.", name().c_str(), table_.frame_id.c_str());
}

}  // namespace table_detection
