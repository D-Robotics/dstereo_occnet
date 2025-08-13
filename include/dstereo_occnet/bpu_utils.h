#ifndef BPU_UTILS_H
#define BPU_UTILS_H

#include <string>
#ifdef PLATFORM_S100
#include "hobot/dnn/hb_dnn.h"
#include "hobot/dnn/hb_dnn_status.h"
#include "hobot/hb_ucp.h"
#include "hobot/hb_ucp_sys.h"
#endif

#ifdef PLATFORM_X5
#include "dnn/hb_dnn.h"
#endif

class BPUUtils {
public:
  // delete the default constructor
  BPUUtils() = delete;

  // utility function
  static float quanti_shift(int32_t data, uint32_t shift);
  static float quanti_scale(int32_t data, float scale);
};

#endif