#include <ros/ros.h>
#include <table_detection/table_detection.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "table_detection");
  ros::NodeHandle nh_public, nh_private("~");

  std::string topic_in;
  nh_private.param<std::string>("topic_in", topic_in, "");

  if (topic_in.empty()) {
    ROS_ERROR("Parameter missing: 'topic_in'");
    return EXIT_FAILURE;
  }

  table_detection::TableDetection table_detection;
  table_detection.run(topic_in);
  ros::spin();

  return EXIT_SUCCESS;
}
