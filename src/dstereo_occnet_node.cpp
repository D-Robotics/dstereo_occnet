#include "dstereo_occnet/dstereo_occnet_node.h"
#include <cstdint>

DStereoOccNetNode::DStereoOccNetNode(const std::string &node_name, const rclcpp::NodeOptions &node_options) : Node(node_name, node_options), dstereo_occnet_infer_(this->get_logger())
{
    // =================================================================================================================================
    /* param */
    this->declare_parameter("stereo_msg_topic", "/image_combine_raw");
    this->get_parameter("stereo_msg_topic", stereo_msg_topic_);
    this->declare_parameter("occ_model_file_path", "");
    this->get_parameter("occ_model_file_path", occ_model_file_path_);

    RCLCPP_INFO_STREAM(this->get_logger(), "\033[31m " << std::endl
                                                       << node_name << " param: " << std::endl
                                                       << "=> occ_model_file_path: " << occ_model_file_path_ << std::endl
                                                       << "=> stereo_msg_topic: " << stereo_msg_topic_ << std::endl
                                                       << "\033[0m");

    // =================================================================================================================================
    // pub & sub
    stereo_msg_sub_ = this->create_subscription<sensor_msgs::msg::Image>(stereo_msg_topic_, rclcpp::SensorDataQoS(), std::bind(&DStereoOccNetNode::infer_online, this, std::placeholders::_1));
    voxel_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/voxel", 10);

    // =================================================================================================================================
    /* init infer class */
    int ret_code = 0;
    ret_code = dstereo_occnet_infer_.init(occ_model_file_path_);
    if (ret_code == -1)
    {
        RCLCPP_ERROR(this->get_logger(), "=> Failed to initialize dstereo_occnet Model, shutting down node.");
        rclcpp::shutdown();
        return;
    }
}

void DStereoOccNetNode::infer_online(const sensor_msgs::msg::Image::ConstSharedPtr &stereo_msg)
{
    if (stereo_msg->encoding != "nv12")
    {
        RCLCPP_ERROR(this->get_logger(), "=> Unsupported image encoding: %s", stereo_msg->encoding.c_str());
        return;
    }
    RCLCPP_INFO_ONCE(this->get_logger(), "=> Image width: %d, height: %d", stereo_msg->width, stereo_msg->height);

    int single_img_w = stereo_msg->width;
    int single_img_h = stereo_msg->height / 2;
    auto occ_grid_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
    occ_grid_msg->header.frame_id = stereo_msg->header.frame_id;
    occ_grid_msg->header.stamp = stereo_msg->header.stamp;

    int ret_code = dstereo_occnet_infer_.forward(stereo_msg->data.data(), stereo_msg->data.data() + (single_img_w * single_img_h * 3 / 2), stereo_msg->width, single_img_h, occ_grid_msg);

    if (ret_code == 0)
    {
        voxel_pub_->publish(*occ_grid_msg);
    }
}
