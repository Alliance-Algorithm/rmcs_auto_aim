/**
 * @file armor_plate_3d.hpp
 * @author Lorenzo Feng (lorenzo.feng@njust.edu.cn)
 * @brief
 * @version 0.1
 * @date 2024-06-02
 *
 * (C)Copyright: NJUST.Alliance - All rights reserved
 *
 */
#pragma once

#include <geometry_msgs/msg/pose.hpp>

#include <rmcs_description/tf_description.hpp>

#include "core/identifier/armor/armor.hpp"

namespace rmcs_auto_aim {
struct ArmorPlate3d {
    ArmorID id;
    rmcs_description::OdomImu::Position position;
    rmcs_description::OdomImu::Rotation rotation;

    explicit ArmorPlate3d(
        ArmorID id, rmcs_description::OdomImu::Position position,
        rmcs_description::OdomImu::Rotation rotation)
        : id(id)
        , position(std::move(position))
        , rotation(std::move(rotation)) {}
};

struct ArmorPlate3dWithNoFrame {
    ArmorID id;
    geometry_msgs::msg::Pose pose;

    ArmorPlate3dWithNoFrame() = default;
    ArmorPlate3dWithNoFrame(ArmorID id, Eigen::Vector3d position, Eigen::Quaterniond rotation)
        : id(id) {
        pose.position.x    = position.x();
        pose.position.y    = position.y();
        pose.position.z    = position.z();
        pose.orientation.x = rotation.x();
        pose.orientation.y = rotation.y();
        pose.orientation.z = rotation.z();
        pose.orientation.w = rotation.w();
    }
};
} // namespace rmcs_auto_aim
