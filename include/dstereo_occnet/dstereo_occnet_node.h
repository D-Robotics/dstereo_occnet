#ifndef DSTEREO_OCCNET_NODE_H
#define DSTEREO_OCCNET_NODE_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "opencv2/opencv.hpp"
#include "dstereo_occnet/dstereo_occnet_infer.h"
#include "dstereo_occnet/dstereo_occnet_infer.h"
#include "dstereo_occnet/img_convert_utils.h"
#include "dstereo_occnet/pc_utils.h"
#include "dstereo_occnet/timer_utils.h"
#include "dstereo_occnet/file_utils.h"

/**
 * @brief DStereoOccNetNode class for occupancy network ros node
 * This class is used to initialize the occupancy network ros node and handle the inference process.
 */
class DStereoOccNetNode : public rclcpp::Node {
public:
  explicit DStereoOccNetNode(const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions(), const std::string &node_name = "dstereo_occnet_node");
  ~DStereoOccNetNode() = default;

private:
  // ===================================== callback fun ===============================
  /**
   * @brief occupancy network online infer fun
   * @param stereo_msg stereo images ros message
   * This function processes stereo images and performs inference using the occupancy network.
   */
  void infer_online(const sensor_msgs::msg::Image::ConstSharedPtr &stereo_msg);

  /**
   * @brief occupancy network offline infer fun
   * This function processes images from a local directory and performs inference using the occupancy network.
   */
  void infer_offline();

  /**
   * @brief callback for camera info message
   * @param camera_info_msg camera info ros message
   * This function updates camera intrinsic parameters based on the received camera info message.
   */
  void camera_info_cb(const sensor_msgs::msg::CameraInfo::ConstSharedPtr &camera_info_msg);

  // ===================================== member =====================================
  /* sub */
  std::string stereo_msg_topic_;
  std::string camera_info_topic_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr stereo_msg_sub_ = nullptr;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_ = nullptr;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr voxel_pub_ = nullptr;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr stereo_msg_pub_ = nullptr;

  /* occ model */
  std::string occ_model_file_path_;
  std::shared_ptr<DStereoOccNetInfer> dstereo_occnet_infer_;

  /* offline */
  bool use_local_image_;
  std::string local_image_dir_;
  rclcpp::TimerBase::SharedPtr timer_ = nullptr;

  /* save result */
  bool save_occ_flag_;
  std::string save_occ_dir_;
  int save_freq_;
  int save_total_;

  /* occ grid */
  float voxel_size_;
};

#endif