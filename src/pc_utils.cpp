#include "dstereo_occnet/pc_utils.h"

void PCUtils::save_pointcloud_to_txt(const sensor_msgs::msg::PointCloud2::SharedPtr &cloud_msg, const std::string &filename) {
  std::ofstream ofs(filename);
  if (!ofs.is_open()) {
    return;
  }

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud_msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud_msg, "z");

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    ofs << *iter_x << " " << *iter_y << " " << *iter_z << "\n";
  }

  ofs.close();
}

void PCUtils::save_pointcloud_to_txt(const std::vector<cv::Point3i> &points, const std::string &filename) {
  std::string buffer;
  buffer.reserve(points.size() * 20);

  for (const auto &pt : points) {
    buffer += std::to_string(pt.x) + " " + std::to_string(pt.y) + " " + std::to_string(pt.z) + "\n";
  }

  std::ofstream ofs(filename, std::ios::out | std::ios::binary);
  if (!ofs.is_open()) {
    return;
  }

  ofs.write(buffer.c_str(), buffer.size());
  ofs.close();
}