#pragma once

#include "opencv2/core/types_c.h"

static inline std::vector<std::vector<cv::Point>> get_contours_points(
    cv::InputOutputArray _image, cv::InputArrayOfArrays _contours, int contourIdx,
    cv::InputArray _hierarchy = cv::noArray(), int maxLevel = std::numeric_limits<int>::max());

