#pragma once

#include <cassert>
#include <cmath>
#include <numbers>
#include <rmcs_description/tf_description.hpp>
#include <vector>

#include "core/transform_optimizer/armor/squad.hpp"
#include "core/transform_optimizer/optimizer/fibonacci.hpp"

#include "core/identifier/armor/armor.hpp"
#include "core/pnpsolver/armor/armor3d.hpp"

#include <Eigen/Eigen>

namespace rmcs_auto_aim::transform_optimizer {

constexpr inline static double epsilone = 0.001;

constexpr inline auto set_armor3d_angle(const auto& inOutArmor3d, const double& angle) {
    return rmcs_description::OdomImu::Rotation(
        Eigen::AngleAxis(angle, Eigen::Vector3d::UnitZ()) * inOutArmor3d);
}

static inline double get_yaw_from_quaternion(const Eigen::Quaterniond& quaternion) {

    double yaw = atan2(
        2.0 * (quaternion.w() * quaternion.z() + quaternion.x() * quaternion.y()),
        1.0 - 2.0 * (quaternion.y() * quaternion.y() + quaternion.z() * quaternion.z()));

    return yaw;
}

static inline void transform_optimize(
    const std::vector<ArmorPlate>& inArmor2d, std::vector<ArmorPlate3d>& inOutArmor3d,
    const rmcs_description::Tf& tf, const double& fx, const double& fy, const double& cx,
    const double& cy, const double& k1, const double& k2, const double& k3) {
    if (inArmor2d.size() != inOutArmor3d.size())
        return;

    for (int i = 0, len = (int)inArmor2d.size(); i < len; i++) {
        auto squad2d = Squad(inArmor2d[i]);
        auto armor3d = inOutArmor3d[i];

        auto rotation = Eigen::AngleAxis(
            165. / 180.0 * std::numbers::pi,
            *rmcs_description::OdomImu::DirectionVector(Eigen::Vector3d::UnitY()));

        auto cameraMatrix = (cv::Mat)(cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
        auto distCoeffs   = (cv::Mat)(cv::Mat_<double>(1, 5) << k1, k2, 0, 0, k3);
        double yaw        = get_yaw_from_quaternion(*armor3d.rotation);
        auto angle        = optimizer::Fibonacci::optimizer(
            yaw - std::numbers::pi / 6, yaw + std::numbers::pi / 6, epsilone,
            [&squad2d, &armor3d, &rotation, &cameraMatrix, &distCoeffs,
             &tf](double angle) -> double {
                armor3d.rotation = set_armor3d_angle(rotation, angle);
                auto squad3d     = Squad3d(armor3d).ToSquad(
                    cameraMatrix, distCoeffs, tf, squad2d.is_large_armor());
                return squad3d - squad2d;
            });

        inOutArmor3d[i].rotation = set_armor3d_angle(rotation, angle);
    }
}

} // namespace rmcs_auto_aim::transform_optimizer