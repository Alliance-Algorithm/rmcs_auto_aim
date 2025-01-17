#pragma once

#include <map>
#include <vector>

#include <robot_id.hpp>

#include "core/pnpsolver/armor/armor3d.hpp"
#include "core/tracker/fling_tracker/armor_tracker/armor_ekf.hpp"
#include "util/math.hpp"

namespace rmcs_auto_aim::tracker::armor::prcessor {

static inline void get_grouped_armor(
    std::map<rmcs_msgs::ArmorID, std::vector<ArmorPlate3d>>& grouped_armor,
    const std::vector<ArmorPlate3d>& armors) {

    grouped_armor[rmcs_msgs::ArmorID::Hero].clear();
    grouped_armor[rmcs_msgs::ArmorID::Engineer].clear();
    grouped_armor[rmcs_msgs::ArmorID::InfantryIII].clear();
    grouped_armor[rmcs_msgs::ArmorID::InfantryIV].clear();
    grouped_armor[rmcs_msgs::ArmorID::InfantryV].clear();
    grouped_armor[rmcs_msgs::ArmorID::Sentry].clear();
    grouped_armor[rmcs_msgs::ArmorID::Outpost].clear();
    for (const auto& armor : armors)
        grouped_armor[armor.id].push_back(armor);
}

constexpr static int distance_in_four(int a, int b) {
    auto err1 = a - b;
    auto err2 = b - a;
    while (err1 < 0)
        err1 += 4;
    while (err2 < 0)
        err2 += 4;
    return err1 > err2 ? err2 : err1;
}

static inline double calculate_similarity(
    const Eigen::Vector3d& t1, const Eigen::Quaterniond& q1, const Eigen::Vector3d& t2,
    const Eigen::Quaterniond& q2) {
    Eigen::Vector3d vec{};
    vec << t1 - t2;

    double rotationDifference =
        -cos(util::math::get_yaw_from_quaternion(q1) - util::math::get_yaw_from_quaternion(q2));

    auto z                       = vec.z();
    vec.z()                      = 0;
    double translationDifference = vec.norm();
    double similarity =
        translationDifference * 1 + std::clamp(z, -0.1, 0.1) + rotationDifference * 5;
    return similarity;
}

static inline rmcs_description::OdomImu::Position
    armor_to_car(const Eigen::Vector<double, 4>& z1, const double& l) {

    return rmcs_description::OdomImu::Position(
        *rmcs_description::OdomImu::Position(Eigen::Vector3d{z1(0), z1(1), z1(2)})
        + rmcs_description::OdomImu::Rotation(Eigen::AngleAxisd(z1(3), Eigen::Vector3d::UnitZ()))
                  ->toRotationMatrix()
              * *rmcs_description::OdomImu::DirectionVector(Eigen::Vector3d::UnitX() * l));
}
static inline double armor_get_odom_z(ArmorEKF& ekf, const rmcs_description::Tf& tf) {
    auto zVec     = ekf.h(ekf.OutPut(), ArmorEKF::v_zero);
    auto odom_pos = fast_tf::cast<rmcs_description::OdomImu>(
        rmcs_description::CameraLink::Position(zVec.x(), zVec.y(), zVec.z()), tf);
    return odom_pos->z();
};

} // namespace rmcs_auto_aim::tracker::armor::prcessor