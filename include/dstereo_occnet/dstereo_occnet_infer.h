#ifndef DSTEREO_OCCNET_INFER_H
#define DSTEREO_OCCNET_INFER_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "dnn/hb_dnn.h"
#include "opencv2/opencv.hpp"
#include "dstereo_occnet/bpu_utils.h"

// =================================================================================================================================
#define HB_CHECK_SUCCESS(logger, ret_code, errmsg)                                                                                                                                                     \
  do {                                                                                                                                                                                                 \
    /*value can be call of function*/                                                                                                                                                                  \
    if (ret_code != 0) {                                                                                                                                                                               \
      RCLCPP_ERROR_STREAM(logger, "=> [BPU ERROR]: " << errmsg << ", error code: " << ret_code);                                                                                                       \
    }                                                                                                                                                                                                  \
  } while (0);

// =================================================================================================================================

/**
 * @brief DStereoOccNetInfer class for occupancy network inference
 * This class is used to initialize and manage the occupancy network inference process.
 */
class DStereoOccNetInfer {
public:
  explicit DStereoOccNetInfer(const rclcpp::Logger &logger);
  ~DStereoOccNetInfer() = default;

  // ===================================== func =======================================
  /**
   * @brief init occnet infer class
   * @param occ_model_file_path occupancy model file path
   * @return 0 on success, -1 on failure
   */
  int init(std::string &occ_model_file_path);

  /**
   * @brief infer by occ model
   */
  int forward(const uint8_t *left_img_data, const uint8_t *right_img_data, const int &img_w, const int &img_h, sensor_msgs::msg::PointCloud2::SharedPtr &occ_grid_msg);

private:
  // ===================================== func =======================================
  /**
   * @brief allocate memory for input tensors, nv12 format
   * @return 0 on success, -1 on failure
   */
  int prepare_input_tensor();

  /**
   * @brief allocate memory for output tensors
   * @return 0 on success, -1 on failure
   */
  int prepare_output_tensor();

  /**
   * @brief fill nv12 image to input tensor
   * @param left_img_data left image data in nv12 format
   * @param right_img_data right image data in nv12 format
   * @param img_w image width
   * @param img_h image height
   * @return 0 on success, -1 on failure
   */
  int fill_nv12_img_to_input_tensor(const uint8_t *left_img_data, const uint8_t *right_img_data);

  /**
   * @brief postprocess the output tensors
   * @param occ_grid_msg output occupancy grid message
   * @return 0 on success, -1 on failure
   */
  int postprocess(sensor_msgs::msg::PointCloud2::SharedPtr &occ_grid_msg);

  // ===================================== member =====================================
  /** init */
  rclcpp::Logger logger_;
  std::string occ_model_file_path_;
  hbPackedDNNHandle_t packed_dnn_handle_;
  const char **model_name_list_;
  int model_count_ = 0;
  hbDNNHandle_t dnn_handle_;
  int input_count_ = 0;
  int output_count_ = 0;

  /** tensor */
  std::vector<hbDNNTensor> input_tensors_;
  std::vector<hbDNNTensor> output_tensors_;
  int model_input_h_;
  int model_input_w_;
  int32_t input_tensor_type_;
};

#endif