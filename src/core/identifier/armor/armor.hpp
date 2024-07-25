/**
 * @file Armor.hpp
 * @author Lorenzo Feng (lorenzo.feng@njust.edu.cn), Qzh
 * @brief
 * @version 0.1
 * @date 2024-06-02
 *
 * (C)Copyright: NJUST.Alliance - All rights reserved
 *
 */
#pragma once

#include <stdexcept>
#include <utility>

#include <opencv2/core/types.hpp>

#include <rmcs_msgs/robot_id.hpp>

namespace rmcs_auto_aim {

struct LightBar {
    cv::Point2f top, bottom;
    float angle;

    LightBar(cv::Point2f _top, cv::Point2f _bottom, float angle)
        : top(std::move(_top))
        , bottom(std::move(_bottom))
        , angle(angle) {}
};

struct ArmorPlate {
    ArmorPlate(
        const LightBar& left, const LightBar& right,
        rmcs_msgs::ArmorID armorId = rmcs_msgs::ArmorID::Unknown, bool isLargeArmor = false)
        : id(armorId)
        , is_large_armor(isLargeArmor) {
        points.push_back(left.top);
        points.push_back(left.bottom);
        points.push_back(right.bottom);
        points.push_back(right.top);
    }

    [[nodiscard]] cv::Point2f center() const {
        if (points.size() != 4) {
            throw std::runtime_error("Invalid ArmorPlate object");
        }
        return (points[0] + points[1] + points[2] + points[3]) / 4;
    }

    std::vector<cv::Point2f> points;
    rmcs_msgs::ArmorID id;
    bool is_large_armor;
};

} // namespace rmcs_auto_aim
