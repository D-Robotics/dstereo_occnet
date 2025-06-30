#ifndef DSTEREO_OCCNET_NODE_H
#define DSTEREO_OCCNET_NODE_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "opencv2/opencv.hpp"
#include "dstereo_occnet/dstereo_occnet_infer.h"
#include "dstereo_occnet/dstereo_occnet_infer.h"
#include "dstereo_occnet/img_convert_utils.h"
#include "dstereo_occnet/pc_utils.h"
#include <filesystem>

namespace fs = std::filesystem;

/**
 * @brief DStereoOccNetNode class for occupancy network ros node
 * This class is used to initialize the occupancy network ros node and handle the inference process.
 */
class DStereoOccNetNode : public rclcpp::Node {
public:
  explicit DStereoOccNetNode(const std::string &node_name = "dstereo_occnet_node", const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions());
  ~DStereoOccNetNode() = default;

private:
  // ===================================== callback fun ===============================
  /**
   * @brief occupancy network online infer fun
   * @param stereo_msg stereo images ros message
   */
  void infer_online(const sensor_msgs::msg::Image::ConstSharedPtr &stereo_msg);

  /**
   * @brief occupancy network offline infer fun
   * This function processes images from a local directory and performs inference using the occupancy network.
   */
  void infer_offline();

  // ===================================== member =====================================
  /* sub */
  std::string stereo_msg_topic_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr stereo_msg_sub_ = nullptr;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr voxel_pub_ = nullptr;

  /* occ model */
  std::string occ_model_file_path_;
  DStereoOccNetInfer dstereo_occnet_infer_;

  /* offline */
  bool use_local_image_;
  std::string local_image_dir_;

  /* save img */
  bool save_img_flag_;
  std::string save_img_dir_;
};

#endif