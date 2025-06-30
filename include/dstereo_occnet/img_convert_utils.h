#ifndef IMG_CONVERT_UTILS_H
#define IMG_CONVERT_UTILS_H

#include <arm_neon.h>
#include "opencv2/opencv.hpp"

class ImgConvertUtils
{
public:
    // delete the default constructor
    ImgConvertUtils() = delete;

    // utility functions
    static void nv12_to_bgr24_neon(uint8_t *nv12, uint8_t *bgr24, int width, int height);
    static void bgr24_to_nv12_neon(uint8_t *bgr24, uint8_t *nv12, int width, int height);
    static void bgr_to_nv12_mat(const cv::Mat &bgr, cv::Mat &nv12);
    static void nv12_to_bgr_mat(const uint8_t *nv12, cv::Mat &bgr24, int width, int height);
};

#endif