#pragma once

#include <chrono>
#include <map>
#include <memory>
#include <numbers>
#include <rmcs_description/tf_description.hpp>
#include <tuple>
#include <utility>
#include <vector>

#include <Eigen/Eigen>
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/AngleAxis.h>

#include "core/pnpsolver/armor/armor3d.hpp"
#include "core/tracker_v2/armor/armor_ekf.hpp"
#include "core/tracker_v2/armor/car_tracker.hpp"
#include "core/tracker_v2/armor/i_tracker.hpp"
#include "core/transform_optimizer/armor/armor.hpp"
#include "core/transform_optimizer/armor/squad.hpp"
#include <fast_tf/impl/cast.hpp>
#include <robot_id.hpp>

namespace rmcs_auto_aim::tracker2::armor {
class ArmorTracker : ITracker {
    ArmorTracker() {
        car_trackers[rmcs_msgs::ArmorID::Hero]          = std::make_shared<CarTracker>();
        car_trackers[rmcs_msgs::ArmorID::Engineer]      = std::make_shared<CarTracker>();
        car_trackers[rmcs_msgs::ArmorID::InfantryIII]   = std::make_shared<CarTracker>();
        car_trackers[rmcs_msgs::ArmorID::InfantryIV]    = std::make_shared<CarTracker>();
        car_trackers[rmcs_msgs::ArmorID::InfantryV]     = std::make_shared<CarTracker>();
        car_trackers[rmcs_msgs::ArmorID::Sentry]        = std::make_shared<CarTracker>();
        car_trackers[rmcs_msgs::ArmorID::Outpost]       = std::make_shared<CarTracker>();
        armor_trackers[rmcs_msgs::ArmorID::Hero]        = {};
        armor_trackers[rmcs_msgs::ArmorID::Engineer]    = {};
        armor_trackers[rmcs_msgs::ArmorID::InfantryIII] = {};
        armor_trackers[rmcs_msgs::ArmorID::InfantryIV]  = {};
        armor_trackers[rmcs_msgs::ArmorID::InfantryV]   = {};
        armor_trackers[rmcs_msgs::ArmorID::Sentry]      = {};
        armor_trackers[rmcs_msgs::ArmorID::Outpost]     = {};
        for (int i = 0; i < 4; i++)
            for (auto& [_, armors] : armor_trackers)
                armors.emplace_back();
    }

    std::shared_ptr<ArmorTarget> Update(
        const std::vector<ArmorPlate3d>& armors,
        const std::chrono::steady_clock::time_point& timestamp,
        const rmcs_description::Tf& tf) override {

        double dt          = std::chrono::duration<double>(timestamp - last_update_).count();
        last_update_       = timestamp;
        auto grouped_armor = get_grouped_armor(armors);
        for (const auto& [armorID, car] : car_trackers) {
            auto len = (*grouped_armor)[armorID].size();
            Eigen::MatrixXd z_car;
            if (len == 1) {
                auto [z, armor_ekf, offset] =
                    single_armor_update((*grouped_armor)[armorID], car, armorID, tf);
                armor_ekf.Update(z, {}, dt);
                z_car = armor_get_car_input({armor_ekf, armor_ekf}, {offset, offset});

            } else if (len == 2) {
                auto [z1, armor_ekf1, offset1, z2, armor_ekf2, offset2] =
                    double_armor_update((*grouped_armor)[armorID], car, armorID, tf);
                armor_ekf1.Update(z1, {}, dt);
                armor_ekf2.Update(z2, {}, dt);
                z_car = armor_get_car_input({armor_ekf1, armor_ekf2}, {offset1, offset2});
            } else
                continue;
            auto pos = fast_tf::cast<rmcs_description::OdomImu>(
                rmcs_description::CameraLink::DirectionVector{z_car(0), z_car(1), z_car(2)}, tf);
            z_car(0) = pos->x();
            z_car(1) = pos->y();
            z_car(2) = pos->z();
            car->update_car(z_car, dt);
            car->update_z(
                cos(armor_trackers[armorID][0].OutPut()(3)),
                cos(armor_trackers[armorID][1].OutPut()(3)),
                cos(armor_trackers[armorID][2].OutPut()(3)),
                cos(armor_trackers[armorID][3].OutPut()(3)));

            last_armors = car->get_armor();
        }
        return nullptr;
    }

    void draw_armors(
        const double& fx, const double& fy, const double& cx, const double& cy, const double& k1,
        const double& k2, const double& k3, const rmcs_description::Tf& tf,
        cv::InputOutputArray image, const cv::Scalar& color, int thickness = 1,
        int lineType = cv::LINE_8, int shift = 0) {
        for (const auto& armor3d : *last_armors)
            transform_optimizer::Squad3d::darw_squad(
                fx, fy, cx, cy, k1, k2, k3, tf, image, transform_optimizer::Squad3d(armor3d), false,
                color, thickness, lineType, shift);
    }

private:
    static std::shared_ptr<std::map<rmcs_msgs::ArmorID, std::vector<ArmorPlate3d>>>
        get_grouped_armor(const std::vector<ArmorPlate3d>& armors) {
        std::shared_ptr<std::map<rmcs_msgs::ArmorID, std::vector<ArmorPlate3d>>> grouped_armor =
            std::make_shared<std::map<rmcs_msgs::ArmorID, std::vector<ArmorPlate3d>>>();

        (*grouped_armor)[rmcs_msgs::ArmorID::Hero]        = {};
        (*grouped_armor)[rmcs_msgs::ArmorID::Engineer]    = {};
        (*grouped_armor)[rmcs_msgs::ArmorID::InfantryIII] = {};
        (*grouped_armor)[rmcs_msgs::ArmorID::InfantryIV]  = {};
        (*grouped_armor)[rmcs_msgs::ArmorID::InfantryV]   = {};
        (*grouped_armor)[rmcs_msgs::ArmorID::Sentry]      = {};
        (*grouped_armor)[rmcs_msgs::ArmorID::Outpost]     = {};
        for (const auto& armor : armors)
            (*grouped_armor)[armor.id].push_back(armor);
        return grouped_armor;
    }

    std::tuple<Eigen::MatrixXd, ArmorEKF&, double> single_armor_update(
        const std::vector<ArmorPlate3d>& armors, const std::shared_ptr<CarTracker>& car,
        const rmcs_msgs::ArmorID& id, const rmcs_description::Tf& tf) {
        auto car_armors = car->get_armor();
        int index       = 0;
        for (auto& armor : armors) {
            double min = 1e7;
            for (int i = 0; i < 4; i++) {
                auto distance = (*(*car_armors)[i].position - *armor.position).norm();
                if (distance < min)
                    index = i;
            }
        }
        auto& tracker     = armor_trackers[id][index];
        Eigen::MatrixXd z = Eigen::VectorXd::Zero(6);
        auto a0_position  = fast_tf::cast<rmcs_description::CameraLink>(armors[0].position, tf);
        z << a0_position->x(), a0_position->y(), a0_position->z(),
            transform_optimizer::get_yaw_from_quaternion(*armors[0].rotation);
        return {z, tracker, index * std::numbers::pi / 2};
    }
    std::tuple<Eigen::MatrixXd, ArmorEKF&, double, Eigen::MatrixXd, ArmorEKF&, double>
        double_armor_update(
            std::vector<ArmorPlate3d>& armors, const std::shared_ptr<CarTracker>& car,
            const rmcs_msgs::ArmorID& id, const rmcs_description::Tf& tf) {
        auto& armor1 = armors[0];
        auto& armor2 = armors[1];

        Eigen::Vector2d p1 = {armor1.position->x(), armor1.position->y()};
        Eigen::Vector2d p2 = {armor2.position->x(), armor2.position->y()};

        auto diffp = p1 - p2;
        auto theta1 =
            abs(atan2(-diffp.y(), -diffp.x())
                - transform_optimizer::get_yaw_from_quaternion(*armor1.rotation));
        auto theta2 =
            abs(atan2(diffp.y(), diffp.x())
                - transform_optimizer::get_yaw_from_quaternion(*armor2.rotation));

        auto theta3 = std::numbers::pi * 2 - theta1 - theta2;

        auto l3 = diffp.norm();
        auto l1 = l3 / sin(theta3) * sin(theta2);
        auto l2 = l3 / sin(theta3) * sin(theta1);
        if (l1 < l2) {
            std::swap(l1, l2);
            std::swap(armor1, armor2);
        }
        car->update_frame(l1, l2);

        auto car_armors = car->get_armor();
        int index1 = 0, index2 = 1;
        double min = 1e7;
        for (int i = 0; i < 4; i += 2) {
            auto distance = (*(*car_armors)[i].position - *armor1.position).norm()
                          + (*(*car_armors)[i + 1].position - *armor2.position).norm();
            if (distance < min) {
                index1 = i;
                index2 = i + 1;
                min    = distance;
            }
            distance = (*(*car_armors)[i].position - *armor1.position).norm()
                     + (*(*car_armors)[((i - 1) + 4) % 4].position - *armor2.position).norm();
            if (distance < min) {
                index1 = i;
                index2 = ((i - 1) + 4) % 4;
                min    = distance;
            }
        }
        auto& tracker1     = armor_trackers[id][index1];
        Eigen::MatrixXd z1 = Eigen::VectorXd::Zero(6);
        auto a1_position   = fast_tf::cast<rmcs_description::CameraLink>(armor1.position, tf);
        z1 << a1_position->x(), a1_position->y(), a1_position->z(),
            transform_optimizer::get_yaw_from_quaternion(*armor1.rotation);

        auto& tracker2     = armor_trackers[id][index2];
        Eigen::MatrixXd z2 = Eigen::VectorXd::Zero(6);
        auto a2_position   = fast_tf::cast<rmcs_description::CameraLink>(armor2.position, tf);
        z2 << a2_position->x(), a2_position->y(), a2_position->z(),
            transform_optimizer::get_yaw_from_quaternion(*armor2.rotation);

        return {z1, tracker1, index1 * std::numbers::pi / 2,
                z2, tracker2, index2 * std::numbers::pi / 2};
    }

    static Eigen::MatrixXd armor_get_car_input(
        std::tuple<ArmorEKF&, ArmorEKF&> armors, std::tuple<double, double> offset) {
        const auto& [a1, a2]           = armors;
        const auto& [offset1, offset2] = offset;

        auto z1 = a1.h(a1.OutPut(), {0, 0, 0, 0, 0, 0});
        auto z2 = a2.h(a2.OutPut(), {0, 0, 0, 0, 0, 0});

        auto p1 = armor_to_car(z1);
        auto p2 = armor_to_car(z2);

        return {
            (p1.x() + p2.x()) / 2, (p1.y() + p2.y()) / 2, (p1.z() + p2.z()) / 2,
            (z1(3) + z2(3) + offset1 + offset2) / 2 + std::numbers::pi};
    }

    static Eigen::VectorXd armor_to_car(const Eigen::VectorXd& z1) {
        return Eigen::Vector3d{z1(0), z1(1), z1(2) - z1(4)}
             + Eigen::AngleAxisd(15, Eigen::Vector3d::UnitY())
                   * Eigen::AngleAxisd(z1(3), Eigen::Vector3d::UnitZ()) * Eigen::Vector3d::UnitX()
                   * z1(5);
    }

    std::map<rmcs_msgs::ArmorID, std::shared_ptr<CarTracker>> car_trackers;
    std::map<rmcs_msgs::ArmorID, std::vector<ArmorEKF>> armor_trackers;
    std::chrono::steady_clock::time_point last_update_;

    bool draw = false;

    std::shared_ptr<std::vector<ArmorPlate3d>> last_armors;
};
} // namespace rmcs_auto_aim::tracker2::armor
