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

#include <cassert>
#include <cstdint>
#include <stdexcept>
#include <utility>

#include <opencv2/core/types.hpp>

#include <rmcs_msgs/msg/armor_plate.hpp>
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

    explicit ArmorPlate(
        std::vector<cv::Point2f>&& points, rmcs_msgs::ArmorID armorId = rmcs_msgs::ArmorID::Unknown,
        bool isLargeArmor = false)
        : points(points)
        , id(armorId)
        , is_large_armor(isLargeArmor) {}

    explicit operator rmcs_msgs::msg::ArmorPlate() const {
        assert(points.size() == 4);
        rmcs_msgs::msg::ArmorPlate armor;
        armor.id             = static_cast<uint16_t>(id);
        armor.left_top.x     = points[0].x;
        armor.left_top.y     = points[0].y;
        armor.left_bottom.x  = points[1].x;
        armor.left_bottom.y  = points[1].y;
        armor.right_bottom.x = points[2].x;
        armor.right_bottom.y = points[2].y;
        armor.right_top.x    = points[3].x;
        armor.right_top.y    = points[3].y;
        return armor;
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

namespace whitelist_code {
constexpr uint8_t Hero        = 0x1;
constexpr uint8_t Engineer    = 0x2;
constexpr uint8_t InfantryIII = 0x4;
constexpr uint8_t InfantryIV  = 0x8;
constexpr uint8_t InfantryV   = 0x10;
constexpr uint8_t Sentry      = 0x20;
constexpr uint8_t Outpost     = 0x40;
constexpr uint8_t Base        = 0x80;
} // namespace whitelist_code

} // namespace rmcs_auto_aim