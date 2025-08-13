#include "dstereo_occnet/bpu_utils.h"

float BPUUtils::quanti_shift(int32_t data, uint32_t shift) {
  return static_cast<float>(data) / static_cast<float>(1 << shift);
}

float BPUUtils::quanti_scale(int32_t data, float scale) {
  return data * scale;
}
