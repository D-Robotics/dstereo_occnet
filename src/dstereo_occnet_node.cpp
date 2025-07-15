#include "dstereo_occnet/dstereo_occnet_node.h"

DStereoOccNetNode::DStereoOccNetNode(const std::string &node_name, const rclcpp::NodeOptions &node_options) : Node(node_name, node_options), dstereo_occnet_infer_(this->get_logger()) {
  // =================================================================================================================================
  /* param */
  this->declare_parameter("stereo_msg_topic", "/image_combine_raw");
  this->get_parameter("stereo_msg_topic", stereo_msg_topic_);
  this->declare_parameter("camera_info_topic", "/image_combine_raw/camera_info");
  this->get_parameter("camera_info_topic", camera_info_topic_);
  this->declare_parameter("occ_model_file_path", "");
  this->get_parameter("occ_model_file_path", occ_model_file_path_);

  this->declare_parameter("use_local_image", false);
  this->get_parameter("use_local_image", use_local_image_);
  this->declare_parameter("local_image_dir", "./occ_offline");
  this->get_parameter("local_image_dir", local_image_dir_);

  this->declare_parameter("save_occ_flag", false);
  this->get_parameter("save_occ_flag", save_occ_flag_);
  this->declare_parameter("save_occ_dir", "./occ_results");
  this->get_parameter("save_occ_dir", save_occ_dir_);
  this->declare_parameter("save_freq", 1);
  this->get_parameter("save_freq", save_freq_);
  this->declare_parameter("save_total", -1);
  this->get_parameter("save_total", save_total_);
  if (save_freq_ < 0) save_freq_ = 1; // Ensure save frequency is at least 1

  this->declare_parameter("voxel_size", 0.02);
  this->get_parameter("voxel_size", voxel_size_);

  RCLCPP_INFO_STREAM(this->get_logger(), "\033[31m " << std::endl
                                                     << node_name << " param: " << std::endl
                                                     << "=> occ_model_file_path: " << occ_model_file_path_ << std::endl
                                                     << "=> stereo_msg_topic: " << stereo_msg_topic_ << std::endl
                                                     << "=> use_local_image: " << (use_local_image_ ? "true" : "false") << std::endl
                                                     << "=> local_image_dir: " << local_image_dir_ << std::endl
                                                     << "=> save_occ_flag: " << (save_occ_flag_ ? "true" : "false") << std::endl
                                                     << "=> save_occ_dir: " << save_occ_dir_ << std::endl
                                                     << "=> save_freq: " << save_freq_ << std::endl
                                                     << "=> save_total: " << save_total_ << std::endl
                                                     << "=> voxel_size: " << voxel_size_ << "m" << std::endl
                                                     << "\033[0m");

  // =================================================================================================================================
  // pub & sub
  // stereo_msg_sub_ = this->create_subscription<sensor_msgs::msg::Image>(stereo_msg_topic_, rclcpp::SensorDataQoS(), std::bind(&DStereoOccNetNode::infer_online, this, std::placeholders::_1));
  rclcpp::QoS qos = rclcpp::QoS(1).best_effort().durability_volatile();
  stereo_msg_sub_ = this->create_subscription<sensor_msgs::msg::Image>(stereo_msg_topic_, qos, std::bind(&DStereoOccNetNode::infer_online, this, std::placeholders::_1));
  camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(camera_info_topic_, 10, std::bind(&DStereoOccNetNode::camera_info_cb, this, std::placeholders::_1));
  voxel_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/voxel", 10);

  // =================================================================================================================================
  /* init infer class */
  int ret_code = 0;
  ret_code = dstereo_occnet_infer_.init(occ_model_file_path_, save_occ_flag_, save_occ_dir_, save_freq_, save_total_);
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

  rclcpp::Time msg_time = stereo_msg->header.stamp;
  rclcpp::Time now = this->get_clock()->now();
  double latency_ms = (now - msg_time).seconds() * 1000.0;
  RCLCPP_INFO(this->get_logger(), "=> before latency: %.2f ms", latency_ms);

  int single_img_w = stereo_msg->width;
  int single_img_h = stereo_msg->height / 2;
  size_t single_nv12_size = single_img_w * single_img_h * 3 / 2;

  auto left_img_data = std::shared_ptr<uint8_t>(new uint8_t[single_nv12_size], std::default_delete<uint8_t[]>());
  auto right_img_data = std::shared_ptr<uint8_t>(new uint8_t[single_nv12_size], std::default_delete<uint8_t[]>());
  {
    ScopeProcessTime t(this->get_logger(), "mem alloc");
    std::memcpy(left_img_data.get(), stereo_msg->data.data(), single_img_w * single_img_h);
    std::memcpy(left_img_data.get() + single_img_w * single_img_h, stereo_msg->data.data() + stereo_msg->width * stereo_msg->height, single_img_w * single_img_h / 2);
    std::memcpy(right_img_data.get(), stereo_msg->data.data() + single_img_w * single_img_h, single_img_w * single_img_h);
    std::memcpy(right_img_data.get() + single_img_w * single_img_h, stereo_msg->data.data() + stereo_msg->width * stereo_msg->height + single_img_w * single_img_h / 2,
                single_img_w * single_img_h / 2);
  }
  dstereo_occnet_infer_.forward(left_img_data, right_img_data, single_img_w, single_img_h, stereo_msg->header, voxel_pub_, voxel_size_);

  now = this->get_clock()->now();
  latency_ms = (now - msg_time).seconds() * 1000.0;
  RCLCPP_INFO(this->get_logger(), "=> after latency: %.2f ms", latency_ms);
}

void DStereoOccNetNode::infer_offline() {

  auto img_paths = FileUtils::find_pairs(local_image_dir_);

  for (auto &img_pair : img_paths) {
    RCLCPP_INFO_STREAM(this->get_logger(), "=> processing image pair: [" << img_pair.first << ", " << img_pair.second << "]");
    cv::Mat left_img_bgr = cv::imread(img_pair.first, cv::IMREAD_COLOR);
    cv::Mat right_img_bgr = cv::imread(img_pair.second, cv::IMREAD_COLOR);
    if (left_img_bgr.empty() || right_img_bgr.empty()) {
      RCLCPP_ERROR(this->get_logger(), "=> failed to read image pair: %s and %s", img_pair.first.c_str(), img_pair.second.c_str());
      continue;
    }
    cv::Mat left_img_nv12, right_img_nv12;
    ImgConvertUtils::bgr_to_nv12_mat(left_img_bgr, left_img_nv12);
    ImgConvertUtils::bgr_to_nv12_mat(right_img_bgr, right_img_nv12);

    size_t img_size = left_img_nv12.cols * left_img_nv12.rows;
    auto left_img_data = std::shared_ptr<uint8_t>(new uint8_t[img_size], std::default_delete<uint8_t[]>());
    auto right_img_data = std::shared_ptr<uint8_t>(new uint8_t[img_size], std::default_delete<uint8_t[]>());
    std::memcpy(left_img_data.get(), left_img_nv12.data, img_size);
    std::memcpy(right_img_data.get(), right_img_nv12.data, img_size);

    std_msgs::msg::Header header;
    header.stamp = rclcpp::Clock().now();
    header.frame_id = "pcl_link";
    dstereo_occnet_infer_.forward(left_img_data, right_img_data, left_img_bgr.cols, left_img_bgr.rows, header, voxel_pub_, voxel_size_);
  }
}

void DStereoOccNetNode::camera_info_cb(const sensor_msgs::msg::CameraInfo::ConstSharedPtr &camera_info_msg) {
  double camera_fx = camera_info_msg->p[0];
  double camera_fy = camera_info_msg->p[5];
  double camera_cx = camera_info_msg->p[2];
  double camera_cy = camera_info_msg->p[6];
  double baseline = camera_info_msg->p[3] / camera_fx;
  dstereo_occnet_infer_.set_cam_intr(camera_fx, camera_fy, camera_cx, camera_cy, baseline);
  RCLCPP_INFO_ONCE(this->get_logger(), "\033[31m=> sub cam intr : fx=%.4f, fy=%.4f, cx=%.4f, cy=%.4f, baseline=%.2f\033[0m", camera_fx, camera_fy, camera_cx, camera_cy, baseline);
}
