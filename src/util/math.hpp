#pragma once

#include <Eigen/Eigen>

namespace rmcs_auto_aim::util::math {

static inline double get_yaw_from_quaternion(const Eigen::Quaterniond& quaternion) {

    const double yaw = atan2(
        2.0 * (quaternion.w() * quaternion.z() + quaternion.x() * quaternion.y()),
        1.0 - 2.0 * (quaternion.y() * quaternion.y() + quaternion.z() * quaternion.z()));

    return yaw;
}

static inline double get_pitch_from_quaternion(const Eigen::Quaterniond& quaternion) {
    const double pitch =
        std::asin(2.0 * (quaternion.w() * quaternion.y() - quaternion.x() * quaternion.z()));

    return pitch;
}

static inline Eigen::Quaterniond
    euler_to_quaternion(const double& yaw_rad, const double& pitch_rad, const double& roll_rad) {
    Eigen::AngleAxisd rollAngle(roll_rad, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch_rad, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw_rad, Eigen::Vector3d::UnitZ());

    Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
    return q;
}

static inline double
    get_angle_err_rad_from_quaternion(const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2) {
    double yaw1  = get_yaw_from_quaternion(q1);
    double yaw2  = get_yaw_from_quaternion(q2);
    auto yaw_err = abs(yaw1 - yaw2);

    while (yaw_err > 2 * std::numbers::pi)
        yaw_err -= 2 * std::numbers::pi;
    if (yaw_err > std::numbers::pi)
        yaw_err = 2 * std::numbers::pi - yaw_err;
    return yaw_err;
}
static inline double
    get_distance_err_rad_from_vector3d(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2) {
    double d1 = v1.norm();
    double d2 = v2.norm();
    auto derr = abs(d1 - d2);

    return derr;
}
static constexpr double ratio(const auto& point) { return atan2(point.y, point.x); }
static constexpr double clamp_pm_pi(auto&& angle) {
    while (angle >= std::numbers::pi)
        angle -= std::numbers::pi;
    while (angle <= -std::numbers::pi)
        angle += std::numbers::pi;

    return angle;
}
static constexpr double clamp_pm_tau(auto&& angle) {
    while (angle >= 2 * std::numbers::pi)
        angle -= 2 * std::numbers::pi;
    while (angle <= -2 * std::numbers::pi)
        angle += 2 * std::numbers::pi;

    return angle;
}
} // namespace rmcs_auto_aim::util::math