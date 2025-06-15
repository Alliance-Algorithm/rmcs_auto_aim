#pragma once

#include <opencv2/core/mat.hpp>

namespace rmcs_auto_aim {
struct ArmorInfo {
public:
    cv::Rect rect_;
    float landmarks_[8];    // 4个关键点
    int label_;
    int color_;             // blue:1 , red:0
};
}