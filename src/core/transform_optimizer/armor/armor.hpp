#pragma once

#include <cassert>
#include <cmath>
#include <numbers>
#include <vector>

#include <Eigen/Eigen>

#include <rmcs_description/tf_description.hpp>

#include "core/identifier/armor/armor.hpp"
#include "core/pnpsolver/armor/armor3d.hpp"
#include "core/transform_optimizer/armor/quadrilateral/quadrilateral.hpp"
#include "util/math.hpp"
#include "util/optimizer/fibonacci.hpp"

namespace rmcs_auto_aim::transform_optimizer {

constexpr inline static double epsilone = 0.001;

inline rmcs_description::OdomImu::Rotation
    set_armor3d_angle(const Eigen::AngleAxis<double>& inOutArmor3d, const double& angle) {
    return rmcs_description::OdomImu::Rotation(
        Eigen::AngleAxis(angle, Eigen::Vector3d::UnitZ()) * inOutArmor3d);
}
static inline void armor_transform_optimize(
    const std::vector<ArmorPlate>& inArmor2d, std::vector<ArmorPlate3d>& inOutArmor3d,
    const rmcs_description::Tf& tf) {
    if (inArmor2d.size() != inOutArmor3d.size())
        return;

    for (int i = 0, len = (int)inArmor2d.size(); i < len; i++) {
        auto squad2d = Quadrilateral(inArmor2d[i]);
        auto armor3d = inOutArmor3d[i];

        Eigen::AngleAxis rotation = Eigen::AngleAxis(
            165. / 180.0 * std::numbers::pi,
            *rmcs_description::OdomImu::DirectionVector(Eigen::Vector3d::UnitY()));

        double yaw = util::math::get_yaw_from_quaternion(*armor3d.rotation);
        auto angle = util::optimizer::fibonacci(
            yaw - std::numbers::pi / 5, yaw + std::numbers::pi / 5, epsilone,
            [&squad2d, &armor3d, &rotation, &tf](double angle) -> double {
                armor3d.rotation = set_armor3d_angle(rotation, angle);
                auto squad3d =
                    Quadrilateral3d(armor3d).ToQuadrilateral(tf, squad2d.is_large_armor());
                return squad3d - squad2d;
            });

        inOutArmor3d[i].rotation = set_armor3d_angle(rotation, angle);
        *inOutArmor3d[i].rotation =
            Eigen::AngleAxisd(
                std::numbers::pi, *inOutArmor3d[i].rotation * Eigen::Vector3d::UnitX())
            * *inOutArmor3d[i].rotation;
    }
}

} // namespace rmcs_auto_aim::transform_optimizer