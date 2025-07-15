#include "dstereo_occnet/dstereo_occnet_infer.h"

DStereoOccNetInfer::DStereoOccNetInfer(const rclcpp::Logger &logger) : logger_(logger), thread_pool_(std::make_unique<ThreadPool>(5)), save_count_(0) {
  RCLCPP_INFO(logger_, "=> DStereoOccNetInfer initialized");
}

int DStereoOccNetInfer::init(std::string &occ_model_file_path, bool save_occ_flag, std::string &save_occ_dir, const int &save_freq, const int &save_total) {
  int ret_code = 0;
  RCLCPP_INFO(logger_, "=> ==================== init occ model start ====================");
  // load model
  occ_model_file_path_ = occ_model_file_path;
  const char *occ_model_file_path_cstr = occ_model_file_path_.c_str();
  ret_code = hbDNNInitializeFromFiles(&packed_dnn_handle_, &occ_model_file_path_cstr, 1);
  HB_CHECK_SUCCESS(logger_, ret_code, "hbDNNInitializeFromFiles failed");

  // get model name
  ret_code = hbDNNGetModelNameList(&model_name_list_, &model_count_, packed_dnn_handle_);
  HB_CHECK_SUCCESS(logger_, ret_code, "hbDNNGetModelNameList failed");

  // get model handle
  ret_code = hbDNNGetModelHandle(&dnn_handle_, packed_dnn_handle_, model_name_list_[0]);
  HB_CHECK_SUCCESS(logger_, ret_code, "hbDNNGetModelHandle failed");

  // get input count and output count
  ret_code = hbDNNGetInputCount(&input_count_, dnn_handle_);
  HB_CHECK_SUCCESS(logger_, ret_code, "hbDNNGetInputCount failed");
  ret_code = hbDNNGetOutputCount(&output_count_, dnn_handle_);
  HB_CHECK_SUCCESS(logger_, ret_code, "hbDNNGetOutputCount failed");
  RCLCPP_INFO_STREAM(logger_, "=> model name: " << model_name_list_[0]);
  RCLCPP_INFO_STREAM(logger_, "=> input_count: " << input_count_);
  RCLCPP_INFO_STREAM(logger_, "=> output_count: " << output_count_);

  // allocate memory for input/output tensor
  ret_code = prepare_input_tensor();
  HB_CHECK_SUCCESS(logger_, ret_code, "prepare_input_tensor failed");
  ret_code = prepare_output_tensor();
  HB_CHECK_SUCCESS(logger_, ret_code, "prepare_output_tensor failed");

  save_occ_flag_ = save_occ_flag;
  save_occ_dir_ = save_occ_dir;
  save_freq_ = save_freq;
  save_total_ = save_total;
  // if (save_occ_flag_ && (!fs::exists(save_occ_dir_) || !fs::is_directory(save_occ_dir_))) {
  //   RCLCPP_ERROR_STREAM(logger_, "\033[31m=> save_occ_dir: " << save_occ_dir_ << " does not exist, please create it manually.\033[0m");
  //   save_occ_flag_ = false;
  // }
  if (save_occ_flag_) {
    if (fs::exists(save_occ_dir_) && !fs::is_directory(save_occ_dir_)) {
      RCLCPP_ERROR_STREAM(logger_, "\033[31m=> save_occ_dir: " << save_occ_dir_ << " is not a directory, please check it.\033[0m");
      save_occ_flag_ = false;
    } else if (!fs::exists(save_occ_dir_)) {
      RCLCPP_INFO_STREAM(logger_, "\033[31m=> save_occ_dir: " << save_occ_dir_ << " does not exist, creating it.\033[0m");
      try {
        if (!fs::create_directories(save_occ_dir_)) {
          RCLCPP_ERROR_STREAM(logger_, "\033[31m=> create save_occ_dir failed: " << save_occ_dir_ << "\033[0m");
          save_occ_flag_ = false;
        } else {
          RCLCPP_INFO_STREAM(logger_, "\033[32m=> create save_occ_dir success: " << save_occ_dir_ << "\033[0m");
        }
      } catch (const fs::filesystem_error &e) {
        RCLCPP_ERROR_STREAM(logger_, "\033[31m=> create save_occ_dir failed: " << save_occ_dir_ << "\033[0m");
        save_occ_flag_ = false;
      }
    } else {
      RCLCPP_INFO_STREAM(logger_, "\033[31m=> save_occ_dir: " << save_occ_dir_ << " exists.\033[0m");
    }
  }
  RCLCPP_INFO(logger_, "=> ==================== init occ model end   ====================");

  return ret_code;
}

int DStereoOccNetInfer::prepare_input_tensor() {
  int ret_code = 0;
  RCLCPP_INFO(logger_, "=> ----- prepare_input_tensor_nv12 -----");
  // check the type of input tensor
  hbDNNTensorProperties properties;
  ret_code = hbDNNGetInputTensorProperties(&properties, dnn_handle_, 0);
  HB_CHECK_SUCCESS(logger_, ret_code, "hbDNNGetInputTensorProperties failed");
  RCLCPP_INFO_STREAM(logger_, "=> input tensor type is " << BPUUtils::tensor_type_to_str(properties.tensorType));
  if ((properties.tensorType != HB_DNN_IMG_TYPE_NV12) && (properties.tensorType != HB_DNN_IMG_TYPE_NV12_SEPARATE)) {
    RCLCPP_ERROR(logger_, "=> input tensor type is not in [HB_DNN_IMG_TYPE_NV12, HB_DNN_IMG_TYPE_NV12_SEPARATE]");
    return -1;
  }
  RCLCPP_INFO_STREAM(logger_, "=> input tensor memsize: " << properties.alignedByteSize);
  input_tensor_type_ = properties.tensorType;
  // int dims = properties.validShape.numDimensions;
  // int *shape = properties.validShape.dimensionSize;
  // RCLCPP_INFO(logger_, "=> input tensor dims: %d", dims);
  // RCLCPP_INFO(logger_, "=> input tensor shape: [%d, %d, %d, %d]", shape[0], shape[1], shape[2], shape[3]);

  // allocate memory for input tensor
  input_tensors_.resize(2);
  for (auto &tensor : input_tensors_) {
    tensor.properties = properties;
    tensor.properties.tensorType = properties.tensorType;
    switch (properties.tensorLayout) {
    case HB_DNN_LAYOUT_NHWC:
      model_input_h_ = properties.validShape.dimensionSize[1];
      model_input_w_ = properties.validShape.dimensionSize[2];
      break;
    case HB_DNN_LAYOUT_NCHW:
      model_input_h_ = properties.validShape.dimensionSize[2];
      model_input_w_ = properties.validShape.dimensionSize[3];
      break;
    default: RCLCPP_ERROR(logger_, "=> input tensor layout is not in [HB_DNN_LAYOUT_NHWC, HB_DNN_LAYOUT_NCHW]"); return -1;
    }
    tensor.properties.validShape.numDimensions = 4;
    tensor.properties.validShape.dimensionSize[0] = 1;
    tensor.properties.validShape.dimensionSize[1] = 3;
    tensor.properties.validShape.dimensionSize[2] = model_input_h_;
    tensor.properties.validShape.dimensionSize[3] = model_input_w_;
    tensor.properties.alignedShape = tensor.properties.validShape;

    RCLCPP_INFO_STREAM(logger_, "=> model_input_h: " << model_input_h_ << ", model_input_w: " << model_input_w_);

    if (properties.tensorType == HB_DNN_IMG_TYPE_NV12) {
      RCLCPP_INFO(logger_, "=> allocate memory HB_DNN_IMG_TYPE_NV12");
      ret_code = hbSysAllocCachedMem(&tensor.sysMem[0], (3 * model_input_h_ * model_input_w_) / 2);
      HB_CHECK_SUCCESS(logger_, ret_code, "hbSysAllocCachedMem failed");
      tensor.sysMem[0].memSize = (3 * model_input_h_ * model_input_w_) / 2;
    } else if (properties.tensorType == HB_DNN_IMG_TYPE_NV12_SEPARATE) {
      RCLCPP_INFO(logger_, "=> allocate memory HB_DNN_IMG_TYPE_NV12_SEPARATE");
      ret_code = hbSysAllocCachedMem(&tensor.sysMem[0], model_input_h_ * model_input_w_);
      HB_CHECK_SUCCESS(logger_, ret_code, "hbSysAllocCachedMem failed");
      tensor.sysMem[0].memSize = model_input_h_ * model_input_w_;

      ret_code = hbSysAllocCachedMem(&tensor.sysMem[1], model_input_h_ * model_input_w_ / 2);
      HB_CHECK_SUCCESS(logger_, ret_code, "hbSysAllocCachedMem failed");
      tensor.sysMem[1].memSize = model_input_h_ * model_input_w_ / 2;
    } else {
      return -1;
    }
  }
  return ret_code;
}

int DStereoOccNetInfer::prepare_output_tensor() {
  int ret_code = 0;
  RCLCPP_INFO(logger_, "=> ----- prepare_output_tensor -----");
  output_tensors_.resize(output_count_);
  for (int i = 0; i < output_count_; ++i) {
    ret_code = hbDNNGetOutputTensorProperties(&output_tensors_[i].properties, dnn_handle_, i);
    HB_CHECK_SUCCESS(logger_, ret_code, "hbDNNGetOutputTensorProperties failed");
    ret_code = hbSysAllocCachedMem(&output_tensors_[i].sysMem[0], output_tensors_[i].properties.alignedByteSize);
    HB_CHECK_SUCCESS(logger_, ret_code, "hbSysAllocCachedMem failed");
    RCLCPP_INFO_STREAM(logger_, "=> output[" << i << "].memsize: " << output_tensors_[i].properties.alignedByteSize);
  }
  return ret_code;
}

int DStereoOccNetInfer::forward(std::shared_ptr<uint8_t> left_img_data, std::shared_ptr<uint8_t> right_img_data, const int &img_w, const int &img_h, const std_msgs::msg::Header &header,
                                const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &voxel_pub, const float &voxel_size) {
  RCLCPP_INFO_STREAM(logger_, "=> ==================== infer by model =======================");
  int ret_code = 0;
  if (img_w != model_input_w_ || img_h != model_input_h_) {
    RCLCPP_ERROR_STREAM(logger_, "=> input image size does not match model input size, expected: " << model_input_w_ << "x" << model_input_h_ << ", got: " << img_w << "x" << img_h);
    return -1;
  }

  {
    ScopeProcessTime t(logger_, "preprocess");
    RCLCPP_INFO(logger_, "=> ----- fill_nv12_img_to_input_tensor -----");
    ret_code = fill_nv12_img_to_input_tensor(left_img_data.get(), right_img_data.get());
  }

  {
    ScopeProcessTime t(logger_, "infer");
    RCLCPP_INFO(logger_, "=> ----- infer -----");
    hbDNNTensor *output = output_tensors_.data();
    hbDNNInferCtrlParam infer_ctrl_param;
    HB_DNN_INITIALIZE_INFER_CTRL_PARAM(&infer_ctrl_param);
    hbDNNTaskHandle_t task_handle = nullptr;
    ret_code = hbDNNInfer(&task_handle, &output, input_tensors_.data(), dnn_handle_, &infer_ctrl_param);
    HB_CHECK_SUCCESS(logger_, ret_code, "hbDNNInfer failed");
    // wait task done
    ret_code = hbDNNWaitTaskDone(task_handle, 0);
    HB_CHECK_SUCCESS(logger_, ret_code, "hbDNNWaitTaskDone failed");
    ret_code = hbDNNReleaseTask(task_handle);
    HB_CHECK_SUCCESS(logger_, ret_code, "hbDNNReleaseTask failed");
  }

  thread_pool_->enqueue([=]() {
    std::vector<cv::Point3i> occ_points;
    {
      ScopeProcessTime t(logger_, "postprocess");
      postprocess(header, voxel_pub, voxel_size, occ_points);
    }

    {
      if (save_occ_flag_ && fs::exists(save_occ_dir_) && fs::is_directory(save_occ_dir_)) {
        std::unique_lock<std::mutex> lock(mtx_);
        ScopeProcessTime t(logger_, "save occ");

        if (save_count_ == 0) {
          std::string cam_intr_file = save_occ_dir_ + "/cam_intr.txt";
          PCUtils::save_cam_intr_to_txt(cam_intr_file, camera_fx_, camera_fy_, camera_cx_, camera_cy_, baseline_);
          RCLCPP_INFO_STREAM(logger_, "\033[32m=> saved cam intr to: " << cam_intr_file << "\033[0m");
        }

        if (save_count_ % save_freq_ == 0) {
          std::ostringstream ss_nsec;
          std::ostringstream ss_cnt;
          ss_nsec << std::setfill('0') << std::setw(9) << header.stamp.nanosec;
          ss_cnt << std::setfill('0') << std::setw(6) << save_count_;
          std::string nsec_str = ss_nsec.str();
          std::string cnt_str = ss_cnt.str();
          std::string pointcloud_path = save_occ_dir_ + "/" + cnt_str + "_" + std::to_string(header.stamp.sec) + "_" + nsec_str + "_occgrid.txt";
          std::string left_img_path = save_occ_dir_ + "/" + cnt_str + "_" + std::to_string(header.stamp.sec) + "_" + nsec_str + "_left.png";
          std::string right_img_path = save_occ_dir_ + "/" + cnt_str + "_" + std::to_string(header.stamp.sec) + "_" + nsec_str + "_right.png";
          cv::Mat left_img, right_img;

          ImgConvertUtils::nv12_to_bgr_mat(left_img_data.get(), left_img, img_w, img_h);
          ImgConvertUtils::nv12_to_bgr_mat(right_img_data.get(), right_img, img_w, img_h);
          cv::imwrite(left_img_path, left_img);
          cv::imwrite(right_img_path, right_img);
          PCUtils::save_pointcloud_to_txt(occ_points, pointcloud_path);
          RCLCPP_INFO_STREAM(logger_, "\033[32m=> saved occ to: " << save_occ_dir_ << ", saved count: " << save_count_ << "\033[0m");
        }

        if (save_total_ > 0 && (save_count_ / save_freq_ + 1) >= save_total_) {
          RCLCPP_INFO(logger_, "\033[32m=> save total count reached, stopping saving occupancy grid.\033[0m");
          save_occ_flag_ = false;
        }
        save_count_++;
      }
    }
  });

  return ret_code;
}

int DStereoOccNetInfer::fill_nv12_img_to_input_tensor(const uint8_t *left_img_data, const uint8_t *right_img_data) {
  int ret_code = 0;
  hbDNNTensor &left_input_tensor = input_tensors_[0];
  hbDNNTensor &right_input_tensor = input_tensors_[1];

  if (input_tensor_type_ == HB_DNN_IMG_TYPE_NV12) {
    // RCLCPP_INFO(logger_, "=> fill image data into memory HB_DNN_IMG_TYPE_NV12");
    // fill image data into memory
    ret_code = hbSysWriteMem(&left_input_tensor.sysMem[0], (char *)left_img_data, left_input_tensor.sysMem[0].memSize);
    HB_CHECK_SUCCESS(logger_, ret_code, "hbSysWriteMem failed");
    ret_code = hbSysWriteMem(&right_input_tensor.sysMem[0], (char *)right_img_data, right_input_tensor.sysMem[0].memSize);
    HB_CHECK_SUCCESS(logger_, ret_code, "hbSysWriteMem failed");

    // make sure memory data is flushed to DDR before inference
    ret_code = hbSysFlushMem(&left_input_tensor.sysMem[0], HB_SYS_MEM_CACHE_CLEAN);
    HB_CHECK_SUCCESS(logger_, ret_code, "hbSysFlushMem failed");
    ret_code = hbSysFlushMem(&right_input_tensor.sysMem[0], HB_SYS_MEM_CACHE_CLEAN);
    HB_CHECK_SUCCESS(logger_, ret_code, "hbSysFlushMem failed");
  } else if (input_tensor_type_ == HB_DNN_IMG_TYPE_NV12_SEPARATE) {
    // RCLCPP_INFO(logger_, "=>fill image data into memory HB_DNN_IMG_TYPE_NV12_SEPARATE");
    // fill image data into memory
    ret_code = hbSysWriteMem(&left_input_tensor.sysMem[0], (char *)left_img_data, left_input_tensor.sysMem[0].memSize);
    HB_CHECK_SUCCESS(logger_, ret_code, "hbSysWriteMem failed");
    ret_code = hbSysWriteMem(&left_input_tensor.sysMem[1], (char *)left_img_data + left_input_tensor.sysMem[0].memSize, left_input_tensor.sysMem[1].memSize);
    HB_CHECK_SUCCESS(logger_, ret_code, "hbSysWriteMem failed");
    ret_code = hbSysWriteMem(&right_input_tensor.sysMem[0], (char *)right_img_data, right_input_tensor.sysMem[0].memSize);
    HB_CHECK_SUCCESS(logger_, ret_code, "hbSysWriteMem failed");
    ret_code = hbSysWriteMem(&right_input_tensor.sysMem[1], (char *)right_img_data + right_input_tensor.sysMem[0].memSize, right_input_tensor.sysMem[1].memSize);
    HB_CHECK_SUCCESS(logger_, ret_code, "hbSysWriteMem failed");

    // make sure memory data is flushed to DDR before inference
    ret_code = hbSysFlushMem(&left_input_tensor.sysMem[0], HB_SYS_MEM_CACHE_CLEAN);
    HB_CHECK_SUCCESS(logger_, ret_code, "hbSysFlushMem failed");
    ret_code = hbSysFlushMem(&left_input_tensor.sysMem[1], HB_SYS_MEM_CACHE_CLEAN);
    HB_CHECK_SUCCESS(logger_, ret_code, "hbSysFlushMem failed");
    ret_code = hbSysFlushMem(&right_input_tensor.sysMem[0], HB_SYS_MEM_CACHE_CLEAN);
    HB_CHECK_SUCCESS(logger_, ret_code, "hbSysFlushMem failed");
    ret_code = hbSysFlushMem(&right_input_tensor.sysMem[1], HB_SYS_MEM_CACHE_CLEAN);
    HB_CHECK_SUCCESS(logger_, ret_code, "hbSysFlushMem failed");
  } else {
    RCLCPP_ERROR(logger_, "=> input_tensor_type is not in [HB_DNN_IMG_TYPE_NV12, HB_DNN_IMG_TYPE_NV12_SEPARATE]");
    return -1;
  }

  return ret_code;
}

/*
int DStereoOccNetInfer::postprocess(sensor_msgs::msg::PointCloud2::SharedPtr &occ_grid_msg) {
  int ret_code = 0;
  float voxel_size = 0.02; // voxel size in meters
  // make sure CPU read data from DDR before using output tensor data
  for (size_t i = 0; i < output_tensors_.size(); i++) {
    ret_code = hbSysFlushMem(&(output_tensors_[i].sysMem[0]), HB_SYS_MEM_CACHE_INVALIDATE);
    HB_CHECK_SUCCESS(logger_, ret_code, "hbSysFlushMem failed");
  }

  hbDNNTensor output_tensor = output_tensors_[0];
  if (output_tensor.properties.tensorType != HB_DNN_TENSOR_TYPE_S32) {
    return -1;
  }

  auto output_tensor_data = reinterpret_cast<int32_t *>(output_tensor.sysMem[0].virAddr);
  int dims = output_tensor.properties.validShape.numDimensions;
  RCLCPP_INFO(logger_, "=> output tensor dims: %d", dims);
  if (dims != 4) {
    return -1;
  }
  int *shape = output_tensor.properties.validShape.dimensionSize;
  int B = shape[0], X = shape[1], Y = shape[2], Z = shape[3];
  RCLCPP_INFO(logger_, "=> output tensor shape: [%d, %d, %d, %d]", B, X, Y, Z);

  std::vector<cv::Point3f> occ_points;
  occ_points.reserve(X * Y * (Z / 2));
  for (int x = 0; x < X; ++x) {
    for (int y = 0; y < Y; ++y) {
      for (int z = 0; z < Z; z += 2) {
        int index1 = x * Y * Z + y * Z + z;
        int index2 = x * Y * Z + y * Z + z + 1;
        int32_t val1 = output_tensor_data[index1];
        int32_t val2 = output_tensor_data[index2];
        if (output_tensor.properties.quantiType == SCALE) {
          float occ_val1 = BPUUtils::quanti_scale(val1, output_tensor.properties.scale.scaleData[z]);
          float occ_val2 = BPUUtils::quanti_scale(val2, output_tensor.properties.scale.scaleData[z + 1]);
          if (occ_val2 > occ_val1) {
            occ_points.emplace_back((x - X / 2) * voxel_size, y * voxel_size, (-z / 2 + Z / 2) * voxel_size);
            // occ_points.emplace_back(x, y, z / 2);
          }
        } else {
          RCLCPP_ERROR(logger_, "=> output tensor quantiType is not SCALE");
          return -1;
        }
      }
    }
  }

  occ_grid_msg->height = 1;
  occ_grid_msg->is_dense = false;
  occ_grid_msg->is_bigendian = false;

  sensor_msgs::PointCloud2Modifier modifier(*occ_grid_msg);
  modifier.setPointCloud2Fields(3, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1, sensor_msgs::msg::PointField::FLOAT32, "z", 1, sensor_msgs::msg::PointField::FLOAT32);
  occ_grid_msg->width = occ_points.size();
  modifier.resize(occ_points.size());

  sensor_msgs::PointCloud2Iterator<float> iter_x(*occ_grid_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*occ_grid_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*occ_grid_msg, "z");

  for (const auto &point : occ_points) {
    *iter_x = point.x;
    *iter_y = point.y;
    *iter_z = point.z;
    ++iter_x;
    ++iter_y;
    ++iter_z;
  }

  return ret_code;
}
*/

int DStereoOccNetInfer::postprocess(const std_msgs::msg::Header &header, const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &voxel_pub, const float &voxel_size,
                                    std::vector<cv::Point3i> &occ_points /* out */) {
  int ret_code = 0;
  // make sure CPU read data from DDR before using output tensor data
  for (size_t i = 0; i < output_tensors_.size(); i++) {
    ret_code = hbSysFlushMem(&(output_tensors_[i].sysMem[0]), HB_SYS_MEM_CACHE_INVALIDATE);
    HB_CHECK_SUCCESS(logger_, ret_code, "hbSysFlushMem failed");
  }

  hbDNNTensor output_tensor = output_tensors_[0];
  if (output_tensor.properties.tensorType != HB_DNN_TENSOR_TYPE_S32) {
    return -1;
  }

  auto output_tensor_data = reinterpret_cast<int32_t *>(output_tensor.sysMem[0].virAddr);
  int dims = output_tensor.properties.validShape.numDimensions;
  RCLCPP_INFO(logger_, "=> output tensor dims: %d", dims);
  if (dims != 4) {
    return -1;
  }
  int *shape = output_tensor.properties.validShape.dimensionSize;
  int B = shape[0], X = shape[1], Y = shape[2], Z = shape[3];
  RCLCPP_INFO(logger_, "=> output tensor shape: [%d, %d, %d, %d]", B, X, Y, Z);

  occ_points.reserve(X * Y * (Z / 2));
  for (int z = 0; z < Z; z += 2) {
    float scale1 = output_tensor.properties.scale.scaleData[z];
    float scale2 = output_tensor.properties.scale.scaleData[z + 1];

    for (int x = 0; x < X; ++x) {
      int row_base = x * Y * Z;

      int y = 0;
      for (; y <= Y - 4; y += 4) {
        // val1
        int32x4_t val1_i32 = {output_tensor_data[row_base + y * Z + z], output_tensor_data[row_base + (y + 1) * Z + z], output_tensor_data[row_base + (y + 2) * Z + z],
                              output_tensor_data[row_base + (y + 3) * Z + z]};

        // val2
        int32x4_t val2_i32 = {output_tensor_data[row_base + y * Z + z + 1], output_tensor_data[row_base + (y + 1) * Z + z + 1], output_tensor_data[row_base + (y + 2) * Z + z + 1],
                              output_tensor_data[row_base + (y + 3) * Z + z + 1]};

        float32x4_t val1_f32 = vcvtq_f32_s32(val1_i32);
        float32x4_t val2_f32 = vcvtq_f32_s32(val2_i32);

        float32x4_t occ_val1 = vmulq_n_f32(val1_f32, scale1);
        float32x4_t occ_val2 = vmulq_n_f32(val2_f32, scale2);

        uint32x4_t mask = vcgeq_f32(occ_val2, occ_val1);

        uint32_t mask_array[4];
        vst1q_u32(mask_array, mask);

        for (int i = 0; i < 4; ++i) {
          if (mask_array[i]) {
            occ_points.emplace_back(x, y + i, z / 2);
          }
        }
      }

      // handle the remaining rows (if Y is not a multiple of 4)
      for (; y < Y; ++y) {
        int index1 = row_base + y * Z + z;
        int index2 = index1 + 1;
        int32_t val1 = output_tensor_data[index1];
        int32_t val2 = output_tensor_data[index2];
        float occ_val1 = val1 * scale1;
        float occ_val2 = val2 * scale2;
        if (occ_val2 > occ_val1) {
          occ_points.emplace_back(x, y, z / 2);
        }
      }
    }
  }
  auto occ_grid_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
  occ_grid_msg->header = header;
  occ_grid_msg->height = 1;
  occ_grid_msg->is_dense = false;
  occ_grid_msg->is_bigendian = false;

  sensor_msgs::PointCloud2Modifier modifier(*occ_grid_msg);
  modifier.setPointCloud2Fields(3, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1, sensor_msgs::msg::PointField::FLOAT32, "z", 1, sensor_msgs::msg::PointField::FLOAT32);
  occ_grid_msg->width = occ_points.size();
  modifier.resize(occ_points.size());

  sensor_msgs::PointCloud2Iterator<float> iter_x(*occ_grid_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*occ_grid_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*occ_grid_msg, "z");

  for (const auto &point : occ_points) {
    *iter_x = (point.x - X / 2) * voxel_size;
    *iter_y = point.y * voxel_size;
    *iter_z = (-point.z + Z / 2) * voxel_size;
    ++iter_x;
    ++iter_y;
    ++iter_z;
  }

  voxel_pub->publish(*occ_grid_msg);

  return ret_code;
}

void DStereoOccNetInfer::set_cam_intr(const double &fx, const double &fy, const double &cx, const double &cy, const double &baseline) {
  camera_fx_ = fx;
  camera_fy_ = fy;
  camera_cx_ = cx;
  camera_cy_ = cy;
  baseline_ = baseline;
}