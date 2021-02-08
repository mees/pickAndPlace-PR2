#include "depth_servers/depth_server.h"
#include <geometry_msgs/PointStamped.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include "ros/ros.h"

#define SQ(x) ((x) * (x))

using namespace std;

bool pc_called = false;
pcl::PointCloud<pcl::PointXYZ> cur_pc;
bool use_closest_pixel = true;
uint32_t img_width = 0;
uint32_t img_height = 0;
ros::Publisher placing_pose_pub;

void pcCallback(const sensor_msgs::PointCloud2ConstPtr& pc_msg) {
  pcl::fromROSMsg(*pc_msg, cur_pc);
  pc_called = true;
  img_width = pc_msg->width;
  img_height = pc_msg->height;
}

bool get_depth(placing::depth_server::Request& req, placing::depth_server::Response& resp) {
  cout << req.u << ", " << req.v << endl;
  int pixel_u = req.u;
  int pixel_v = req.v;
  if (pixel_u < 0 || pixel_v < 0 || pixel_u >= (int32_t)img_width || pixel_v >= (int32_t)img_height) {
    ROS_WARN("[pixel_2_3d] Pixel requested is outside image size.");
  }
  int64_t pc_ind = pixel_u + pixel_v * img_width;
  geometry_msgs::PointStamped pt3d;
  pt3d.header.frame_id = cur_pc.header.frame_id;
  pt3d.header.stamp = ros::Time::now();
  if (cur_pc.points[pc_ind].x != cur_pc.points[pc_ind].x) {
    if (use_closest_pixel) {
      // find the closest pixel that has a point in the PC
      std::vector<double> dists(img_width * img_height, 1e30);
      int64_t cur_pc_ind;
      for (int64_t i = 0; i < img_height; i++) {
        for (int64_t j = 0; j < img_width; j++) {
          cur_pc_ind = j + i * img_width;
          if (cur_pc.points[cur_pc_ind].x == cur_pc.points[cur_pc_ind].x)
            dists[cur_pc_ind] = SQ(pixel_u - j) + SQ(pixel_v - i);
        }
      }
      pc_ind = std::min_element(dists.begin(), dists.end()) - dists.begin();
    } else {
      ROS_WARN("[pixel_2_3d] Point cloud not defined for this region.");
    }
  }
  cout << cur_pc.points[pc_ind].x << ", " << cur_pc.points[pc_ind].y << ", " << cur_pc.points[pc_ind].z << ", " << endl;
  pt3d.point.x = cur_pc.points[pc_ind].x;
  pt3d.point.y = cur_pc.points[pc_ind].y;
  pt3d.point.z = cur_pc.points[pc_ind].z;
  resp.pixel3d = pt3d;
  placing_pose_pub.publish(pt3d);
  return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "pixel_2_3d");
  ros::NodeHandle nh;
  ros::Subscriber pc_sub = nh.subscribe("/camera/depth_registered/points", 1, pcCallback);
  ros::ServiceServer service = nh.advertiseService("get_kinect_depth", get_depth);
  placing_pose_pub = nh.advertise<geometry_msgs::PointStamped>("placing_pose_pub", 10);
  ros::spin();
  return 0;
}