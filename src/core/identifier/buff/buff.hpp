#pragma once

#include <cassert>
#include <opencv2/opencv.hpp>

#include <rmcs_description/tf_description.hpp>
#include <rmcs_msgs/msg/buff_plate.hpp>
#include <rmcs_msgs/msg/detail/buff_plate__struct.hpp>

namespace rmcs_auto_aim {
struct BuffPlate {
    // bl br tr tl
    std::vector<cv::Point2f> points;

    explicit operator rmcs_msgs::msg::BuffPlate() const {
        assert(points.size() == 4);
        rmcs_msgs::msg::BuffPlate buff;
        buff.bottom_left.x  = points[0].x;
        buff.bottom_left.y  = points[0].y;
        buff.bottom_right.x = points[1].x;
        buff.bottom_right.y = points[1].y;
        buff.top_right.x    = points[2].x;
        buff.top_right.y    = points[2].y;
        buff.top_left.x     = points[3].x;
        buff.top_left.y     = points[3].y;
        return buff;
    }
};
} // namespace rmcs_auto_aim