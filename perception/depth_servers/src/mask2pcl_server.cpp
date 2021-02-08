#include "depth_servers/mask2pcl_server.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include "ros/ros.h"
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>

#define SQ(x) ((x) * (x))

using namespace std;

bool pc_called = false;
pcl::PointCloud<pcl::PointXYZ> cur_pc;
bool use_closest_pixel = true;
uint32_t img_width = 0;
uint32_t img_height = 0;
ros::Publisher object_pub;

void pcCallback(const sensor_msgs::PointCloud2ConstPtr& pc_msg) {
  pcl::fromROSMsg(*pc_msg, cur_pc);
  pc_called = true;
  img_width = pc_msg->width;
  img_height = pc_msg->height;
}

bool mask2depth(placing::mask2pcl_server::Request& req, placing::mask2pcl_server::Response& resp) {
  sensor_msgs::PointCloud2 object_cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  for (int idx = 0; idx < req.data.size() - 1; idx = idx + 2) {
    int pixel_u = req.data[idx];
    int pixel_v = req.data[idx + 1];
    int64_t pc_ind = pixel_u + pixel_v * img_width;
    cout << "pixel_u: " << pixel_u << " pixel_v: " << pixel_v << " pc_ind: " << pc_ind << endl;
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
      }
    } else {
      ROS_WARN("[mask2pcl] Point cloud not defined for this region.");
      cout << "pixel_u: " << pixel_u << " pixel_v: "
           << " pc_ind: " << pc_ind << endl;
    }
    cout << cur_pc.points[pc_ind].x << ", " << cur_pc.points[pc_ind].y << ", " << cur_pc.points[pc_ind].z << ", "
         << endl;
    pcl::PointXYZ p;
    p.x = cur_pc.points[pc_ind].x;
    p.y = cur_pc.points[pc_ind].y;
    p.z = cur_pc.points[pc_ind].z;
    cloud->push_back(p);

    pcl::toROSMsg(*cloud, object_cloud);
    object_cloud.header.frame_id = cur_pc.header.frame_id;
    object_cloud.header.stamp = ros::Time::now();
    object_pub.publish(object_cloud);
    return true;
  }

  int main(int argc, char** argv) {
    ros::init(argc, argv, "mask2pcl");
    ros::NodeHandle nh;
    ros::Subscriber pc_sub  = nh.subscribe("/camera/depth_registered/points", 1, pcCallback);
    ros::ServiceServer service = nh.advertiseService("convert_mask2pcl", mask2depth);
    object_pub = nh.advertise<sensor_msgs::PointCloud2>("refexp_object", 10);
    ros::spin();
    return 0;
  }