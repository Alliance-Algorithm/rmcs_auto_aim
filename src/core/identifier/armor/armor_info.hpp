#pragma once

#include <opencv2/core/mat.hpp>
#include "rmcs_msgs/robot_color.hpp"
#include "rmcs_msgs/robot_id.hpp"

namespace rmcs_auto_aim {
struct ArmorInfo {
public:
    cv::Rect rect_;
    rmcs_msgs::ArmorID robot_id_;
    rmcs_msgs::RobotColor color_;
};
}