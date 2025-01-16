#pragma once

#include <Eigen/Eigen>

namespace rmcs_auto_aim::util::math {

static inline double get_yaw_from_quaternion(const Eigen::Quaterniond& quaternion) {

    double yaw = atan2(
        2.0 * (quaternion.w() * quaternion.z() + quaternion.x() * quaternion.y()),
        1.0 - 2.0 * (quaternion.y() * quaternion.y() + quaternion.z() * quaternion.z()));

    return yaw;
}
} // namespace rmcs_auto_aim::util::math