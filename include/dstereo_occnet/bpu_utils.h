#ifndef BPU_UTILS_H
#define BPU_UTILS_H

#include <string>
#include "dnn/hb_dnn.h"

class BPUUtils {
public:
  // delete the default constructor
  BPUUtils() = delete;

  // utility function
  static std::string tensor_type_to_str(const int32_t &tensor_type);
  static float quanti_shift(int32_t data, uint32_t shift);
  static float quanti_scale(int32_t data, float scale);
};

#endif