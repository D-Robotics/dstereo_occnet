#include "dstereo_occnet/bpu_utils.h"

std::string BPUUtils::tensor_type_to_str(const int32_t &tensor_type) {
  switch (tensor_type) {
  case HB_DNN_IMG_TYPE_Y:
    return "HB_DNN_IMG_TYPE_Y";
  case HB_DNN_IMG_TYPE_NV12:
    return "HB_DNN_IMG_TYPE_NV12";
  case HB_DNN_IMG_TYPE_NV12_SEPARATE:
    return "HB_DNN_IMG_TYPE_NV12_SEPARATE";
  case HB_DNN_IMG_TYPE_YUV444:
    return "HB_DNN_IMG_TYPE_YUV444";
  case HB_DNN_IMG_TYPE_RGB:
    return "HB_DNN_IMG_TYPE_RGB";
  case HB_DNN_IMG_TYPE_BGR:
    return "HB_DNN_IMG_TYPE_BGR";
  case HB_DNN_TENSOR_TYPE_S4:
    return "HB_DNN_TENSOR_TYPE_S4";
  case HB_DNN_TENSOR_TYPE_U4:
    return "HB_DNN_TENSOR_TYPE_U4";
  case HB_DNN_TENSOR_TYPE_S8:
    return "HB_DNN_TENSOR_TYPE_S8";
  case HB_DNN_TENSOR_TYPE_U8:
    return "HB_DNN_TENSOR_TYPE_U8";
  case HB_DNN_TENSOR_TYPE_F16:
    return "HB_DNN_TENSOR_TYPE_F16";
  case HB_DNN_TENSOR_TYPE_S16:
    return "HB_DNN_TENSOR_TYPE_S16";
  case HB_DNN_TENSOR_TYPE_U16:
    return "HB_DNN_TENSOR_TYPE_U16";
  case HB_DNN_TENSOR_TYPE_F32:
    return "HB_DNN_TENSOR_TYPE_F32";
  case HB_DNN_TENSOR_TYPE_S32:
    return "HB_DNN_TENSOR_TYPE_S32";
  case HB_DNN_TENSOR_TYPE_U32:
    return "HB_DNN_TENSOR_TYPE_U32";
  case HB_DNN_TENSOR_TYPE_F64:
    return "HB_DNN_TENSOR_TYPE_F64";
  case HB_DNN_TENSOR_TYPE_S64:
    return "HB_DNN_TENSOR_TYPE_S64";
  case HB_DNN_TENSOR_TYPE_U64:
    return "HB_DNN_TENSOR_TYPE_U64";
  case HB_DNN_TENSOR_TYPE_MAX:
    return "HB_DNN_TENSOR_TYPE_MAX";
  default:
    return "Unknown";
  }
}

float BPUUtils::quanti_shift(int32_t data, uint32_t shift) {
  return static_cast<float>(data) / static_cast<float>(1 << shift);
}

float BPUUtils::quanti_scale(int32_t data, float scale) {
  return data * scale;
}
