#pragma once

#include <Eigen/Eigen>

namespace rmcs_auto_aim::util::math {

static inline double get_yaw_from_quaternion(const Eigen::Quaterniond& quaternion) {

    double yaw = atan2(
        2.0 * (quaternion.w() * quaternion.z() + quaternion.x() * quaternion.y()),
        1.0 - 2.0 * (quaternion.y() * quaternion.y() + quaternion.z() * quaternion.z()));

    return yaw;
}

static inline double
    get_angle_err_rad_from_quaternion(const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2) {
    double dot_product   = q1.dot(q2);
    double magnitude_q1  = q1.norm();
    double magnitude_q2  = q2.norm();
    double cos_theta     = dot_product / (magnitude_q1 * magnitude_q2);
    double angle_radians = std::acos(cos_theta);
    return angle_radians;
}
} // namespace rmcs_auto_aim::util::math