#ifndef PC_UTILS_H
#define PC_UTILS_H

#include <fstream>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "opencv2/core/types.hpp"

class PCUtils {
public:
  // delete the default constructor
  PCUtils() = delete;

  // utility function

  static void save_pointcloud_to_txt(const sensor_msgs::msg::PointCloud2::SharedPtr &cloud_msg, const std::string &filename);

  static void save_pointcloud_to_txt(const std::vector<cv::Point3i> &points, const std::string &filename);
};

#endif