#include "dstereo_occnet/dstereo_occnet_node.h"

DStereoOccNetNode::DStereoOccNetNode(const std::string &node_name, const rclcpp::NodeOptions &node_options) : Node(node_name, node_options), dstereo_occnet_infer_(this->get_logger()) {
  // =================================================================================================================================
  /* param */
  this->declare_parameter("stereo_msg_topic", "/image_combine_raw");
  this->get_parameter("stereo_msg_topic", stereo_msg_topic_);
  this->declare_parameter("occ_model_file_path", "");
  this->get_parameter("occ_model_file_path", occ_model_file_path_);

  this->declare_parameter("use_local_image", false);
  this->get_parameter("use_local_image", use_local_image_);
  this->declare_parameter("local_image_dir", "");
  this->get_parameter("local_image_dir", local_image_dir_);

  this->declare_parameter("save_img_flag", false);
  this->get_parameter("save_img_flag", save_img_flag_);
  this->declare_parameter("save_img_dir", "");
  this->get_parameter("save_img_dir", save_img_dir_);

  this->declare_parameter("voxel_size", 0.02);
  this->get_parameter("voxel_size", voxel_size_);

  RCLCPP_INFO_STREAM(this->get_logger(), "\033[31m " << std::endl
                                                     << node_name << " param: " << std::endl
                                                     << "=> occ_model_file_path: " << occ_model_file_path_ << std::endl
                                                     << "=> stereo_msg_topic: " << stereo_msg_topic_ << std::endl
                                                     << "=> use_local_image: " << (use_local_image_ ? "true" : "false") << std::endl
                                                     << "=> local_image_dir: " << local_image_dir_ << std::endl
                                                     << "=> save_img_flag: " << (save_img_flag_ ? "true" : "false") << std::endl
                                                     << "=> save_img_dir: " << save_img_dir_ << std::endl
                                                      << "=> voxel_size: " << voxel_size_  << "m" << std::endl
                                                     << "\033[0m");

  // =================================================================================================================================
  // pub & sub
  stereo_msg_sub_ = this->create_subscription<sensor_msgs::msg::Image>(stereo_msg_topic_, rclcpp::SensorDataQoS(), std::bind(&DStereoOccNetNode::infer_online, this, std::placeholders::_1));
  voxel_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/voxel", 10);

  // =================================================================================================================================
  /* init infer class */
  int ret_code = 0;
  ret_code = dstereo_occnet_infer_.init(occ_model_file_path_);
  if (ret_code == -1) {
    RCLCPP_ERROR(this->get_logger(), "=> Failed to initialize dstereo_occnet Model, shutting down node.");
    rclcpp::shutdown();
    return;
  }

  if (use_local_image_) {
    infer_offline();
  }
}

void DStereoOccNetNode::infer_online(const sensor_msgs::msg::Image::ConstSharedPtr &stereo_msg) {
  if (stereo_msg->encoding != "nv12") {
    RCLCPP_ERROR(this->get_logger(), "=> Unsupported image encoding: %s", stereo_msg->encoding.c_str());
    return;
  }
  RCLCPP_INFO_ONCE(this->get_logger(), "=> Image width: %d, height: %d", stereo_msg->width, stereo_msg->height);

  int single_img_w = stereo_msg->width;
  int single_img_h = stereo_msg->height / 2;
  // auto occ_grid_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
  // occ_grid_msg->header.frame_id = stereo_msg->header.frame_id;
  // occ_grid_msg->header.stamp = stereo_msg->header.stamp;
  // stereo_msg->data.data(), stereo_msg->data.data() + (single_img_w * single_img_h * 3 / 2);
  uint8_t *left_img_data = new uint8_t[single_img_w * single_img_h * 3 / 2];
  uint8_t *right_img_data = new uint8_t[single_img_w * single_img_h * 3 / 2];
  // Copy the left and right images from the stereo
  std::memcpy(left_img_data, stereo_msg->data.data(), single_img_w * single_img_h);
  std::memcpy(left_img_data + single_img_w * single_img_h, stereo_msg->data.data() + stereo_msg->width * stereo_msg->height, single_img_w * single_img_h / 2);
  std::memcpy(right_img_data, stereo_msg->data.data() + single_img_w * single_img_h, single_img_w * single_img_h);
  std::memcpy(right_img_data + single_img_w * single_img_h, stereo_msg->data.data() + stereo_msg->width * stereo_msg->height + single_img_w * single_img_h / 2, single_img_w * single_img_h / 2);
  int ret_code = dstereo_occnet_infer_.forward(left_img_data, right_img_data, single_img_w, single_img_h, stereo_msg->header, voxel_pub_, voxel_size_);

  // if (ret_code == 0) {
  //   voxel_pub_->publish(*occ_grid_msg);

  //   if (save_img_flag_) {
  //     if (!save_img_dir_.empty() && !fs::exists(save_img_dir_)) {
  //       bool success = fs::create_directories(save_img_dir_);
  //       if (success) {
  //         RCLCPP_INFO_STREAM(this->get_logger(), "=> Created directory: " << save_img_dir_);
  //       } else {
  //         RCLCPP_ERROR_STREAM(this->get_logger(), "=> Failed to create directory: " << save_img_dir_);
  //         return;
  //       }
  //     }
  //     std::string left_img_path = save_img_dir_ + "/left_" + std::to_string(stereo_msg->header.stamp.sec) + "_" + std::to_string(stereo_msg->header.stamp.nanosec) + ".png";
  //     std::string right_img_path = save_img_dir_ + "/right_" + std::to_string(stereo_msg->header.stamp.sec) + "_" + std::to_string(stereo_msg->header.stamp.nanosec) + ".png";
  //     std::string pointcloud_path = save_img_dir_ + "/occgrid_" + std::to_string(stereo_msg->header.stamp.sec) + "_" + std::to_string(stereo_msg->header.stamp.nanosec) + ".txt";
  //     cv::Mat left_img, right_img;
  //     ImgConvertUtils::nv12_to_bgr_mat(left_img_data, left_img, single_img_w, single_img_h);
  //     ImgConvertUtils::nv12_to_bgr_mat(right_img_data, right_img, single_img_w, single_img_h);
  //     cv::imwrite(left_img_path, left_img);
  //     cv::imwrite(right_img_path, right_img);
  //     PCUtils::save_pointcloud_to_txt(occ_grid_msg, pointcloud_path);
  //     RCLCPP_INFO_STREAM(this->get_logger(), "=> Saved Occ Result to: " << save_img_dir_);
  //   }
  // }
  delete[] left_img_data;
  delete[] right_img_data;
}

void DStereoOccNetNode::infer_offline() {

  std::string left_img_path = "./180_left.npy.png";
  std::string right_img_path = "./180_right.npy.png";
  RCLCPP_INFO_STREAM(this->get_logger(), "=> left_img_path: " << left_img_path << " , right_img_path: " << right_img_path);
  cv::Mat left_img_bgr = cv::imread(left_img_path, cv::IMREAD_COLOR);
  cv::Mat right_img_bgr = cv::imread(right_img_path, cv::IMREAD_COLOR);
  if (left_img_bgr.empty() || right_img_bgr.empty()) {
    RCLCPP_ERROR(this->get_logger(), "=> failed to read image!");
  }
  cv::Mat left_img_nv12, right_img_nv12;
  ImgConvertUtils::bgr_to_nv12_mat(left_img_bgr, left_img_nv12);
  ImgConvertUtils::bgr_to_nv12_mat(right_img_bgr, right_img_nv12);

  auto occ_grid_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
  occ_grid_msg->header.frame_id = "pcl_link";
  // int ret_code = dstereo_occnet_infer_.forward(left_img_nv12.data, right_img_nv12.data, left_img_bgr.cols, left_img_bgr.rows, occ_grid_msg);
  // if (ret_code == 0) {
  //   while (rclcpp::ok()) {
  //     occ_grid_msg->header.stamp = this->get_clock()->now();
  //     voxel_pub_->publish(*occ_grid_msg);
  //     std::this_thread::sleep_for(std::chrono::milliseconds(100));
  //     RCLCPP_INFO(this->get_logger(), "=> Published occupancy grid point cloud.");
  //   }
  // }
}
