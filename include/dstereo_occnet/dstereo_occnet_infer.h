#ifndef DSTEREO_OCCNET_INFER_H
#define DSTEREO_OCCNET_INFER_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "opencv2/opencv.hpp"
#include "magic_enum/magic_enum.hpp"
#include "dstereo_occnet/timer_utils.h"
#include "dstereo_occnet/thread_pool.h"
#include "dstereo_occnet/img_convert_utils.h"
#include "dstereo_occnet/pc_utils.h"
#include <filesystem>
#include "dstereo_occnet/dnn_platform.h"
//#include "dstereo_occnet/bpu_utils.h"

namespace fs = std::filesystem;

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
   * @param save_occ_flag flag to save occupancy grid result
   * @param save_occ_dir directory to save occupancy grid result
   * @param save_freq frequency to save occupancy grid result
   * @return 0 on success, -1 on failure
   */
  int init(std::string &occ_model_file_path, bool save_occ_flag, std::string &save_occ_dir, const int &save_freq, const int &save_total = -1);

  /**
   * @brief infer by occ model
   */
  int forward(std::shared_ptr<uint8_t> left_img_data, std::shared_ptr<uint8_t> right_img_data, const int &img_w, const int &img_h, const std_msgs::msg::Header &header,
              const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &voxel_pub, const float &voxel_size);

  /**
   * @brief set camera intrinsic parameters
   * @param fx focal length in x direction
   * @param fy focal length in y direction
   * @param cx optical center in x direction
   * @param cy optical center in y direction
   * @param baseline baseline distance between two cameras
   */
  void set_cam_intr(const double &fx, const double &fy, const double &cx, const double &cy, const double &baseline);

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
   * @param header message header
   * @param voxel_pub publisher for voxel grid
   * @param voxel_size voxel size for occupancy grid
   * @param occ_points output occupancy points
   * @return 0 on success, -1 on failure
   */
  int postprocess(const std_msgs::msg::Header &header, const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &voxel_pub, const float &voxel_size,
                  std::vector<cv::Point3i> &occ_points /* out */);

  /**
   * @brief save occupancy grid result
   * @param left_img_data left image data in nv12 format
   * @param right_img_data right image data in nv12 format
   * @param occ_points occupancy points
   * @param header message header
   * @param img_w image width
   * @param img_h image height
   * @return void
   */
  void save_occ_result(const std::shared_ptr<uint8_t> &left_img_data, const std::shared_ptr<uint8_t> &right_img_data, const std::vector<cv::Point3i> &occ_points, const std_msgs::msg::Header &header,
                       int img_w, int img_h);

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

  /** thread pool */
  std::unique_ptr<ThreadPool> postprocess_thread_pool_;
  std::unique_ptr<ThreadPool> save_thread_pool_;

  /* save result */
  bool save_occ_flag_;
  std::string save_occ_dir_;
  int save_freq_;
  int save_total_;
  int save_count_ = 0;
  std::mutex mtx_;

  /* cam intr */
  double camera_fx_ = 0.0;
  double camera_fy_ = 0.0;
  double camera_cx_ = 0.0;
  double camera_cy_ = 0.0;
  double baseline_ = 0.0;
};

#endif