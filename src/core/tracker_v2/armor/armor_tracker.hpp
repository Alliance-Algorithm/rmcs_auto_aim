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
#include "core/tracker_v2/armor/armor_target.hpp"
#include "core/tracker_v2/armor/car_tracker.hpp"
#include "core/tracker_v2/tracker_interface.hpp"
#include "core/transform_optimizer/armor/armor.hpp"
#include "core/transform_optimizer/armor/squad.hpp"
#include <fast_tf/impl/cast.hpp>
#include <robot_id.hpp>

namespace rmcs_auto_aim::tracker2::armor {
class ArmorTracker : ITracker {
public:
    ArmorTracker() {
        car_trackers[rmcs_msgs::ArmorID::Hero]          = std::make_shared<CarTracker>();
        car_trackers[rmcs_msgs::ArmorID::Engineer]      = std::make_shared<CarTracker>();
        car_trackers[rmcs_msgs::ArmorID::InfantryIII]   = std::make_shared<CarTracker>();
        car_trackers[rmcs_msgs::ArmorID::InfantryIV]    = std::make_shared<CarTracker>();
        car_trackers[rmcs_msgs::ArmorID::InfantryV]     = std::make_shared<CarTracker>();
        car_trackers[rmcs_msgs::ArmorID::Sentry]        = std::make_shared<CarTracker>();
        car_trackers[rmcs_msgs::ArmorID::Outpost]       = std::make_shared<CarTracker>();
        armor_trackers[rmcs_msgs::ArmorID::Hero]        = std::vector<ArmorEKF>{};
        armor_trackers[rmcs_msgs::ArmorID::Engineer]    = std::vector<ArmorEKF>{};
        armor_trackers[rmcs_msgs::ArmorID::InfantryIII] = std::vector<ArmorEKF>{};
        armor_trackers[rmcs_msgs::ArmorID::InfantryIV]  = std::vector<ArmorEKF>{};
        armor_trackers[rmcs_msgs::ArmorID::InfantryV]   = std::vector<ArmorEKF>{};
        armor_trackers[rmcs_msgs::ArmorID::Sentry]      = std::vector<ArmorEKF>{};
        armor_trackers[rmcs_msgs::ArmorID::Outpost]     = std::vector<ArmorEKF>{};
        for (int i = 0; i < 4; i++)
            for (auto& [_, armors] : armor_trackers)
                armors.emplace_back();
        grouped_armor[rmcs_msgs::ArmorID::Hero]        = {};
        grouped_armor[rmcs_msgs::ArmorID::Engineer]    = {};
        grouped_armor[rmcs_msgs::ArmorID::InfantryIII] = {};
        grouped_armor[rmcs_msgs::ArmorID::InfantryIV]  = {};
        grouped_armor[rmcs_msgs::ArmorID::InfantryV]   = {};
        grouped_armor[rmcs_msgs::ArmorID::Sentry]      = {};
        grouped_armor[rmcs_msgs::ArmorID::Outpost]     = {};
    }

    std::shared_ptr<ITarget> Update(
        const std::vector<ArmorPlate3d>& armors,
        const std::chrono::steady_clock::time_point& timestamp,
        const rmcs_description::Tf& tf) override {

        std::shared_ptr<ITarget> target = nullptr;
        auto target_min                 = 1e7;

        double dt    = std::chrono::duration<double>(timestamp - last_update_).count();
        last_update_ = timestamp;
        dt           = std::clamp(dt, 0., .5);
        get_grouped_armor(grouped_armor, armors);
        for (const auto& [armorID, car] : car_trackers) {
            auto len = grouped_armor[armorID].size();
            ArmorEKF::ZVec z_armor{};
            double l1_ = 0;
            double l2_ = 0;
            z_armor.setIdentity();
            if (len == 1) {
                auto [z, armor_ekf, l, offset] =
                    single_armor_update(grouped_armor[armorID], car, armorID, tf, dt);
                // auto x        = armor_ekf.h(armor_ekf.OutPut(), {});
                // auto odom_pos = fast_tf::cast<rmcs_description::OdomImu>(
                //     rmcs_description::CameraLink::Position(x.x(), x.y(), x.z()), tf, dt);

                l1_ = l;
                l2_ = l;
                ArmorEKF::XVec out;
                out << armor_ekf.OutPut();
                armor_ekf.Update(z, {}, dt);
                z_armor =
                    combine_armor_out({armor_ekf, armor_ekf}, {offset, offset}, {l1_, l2_}, tf);

            } else if (len == 2) {
                auto [z1, armor_ekf1, l1, offset1, z2, armor_ekf2, l2, offset2] =
                    double_armor_update(grouped_armor[armorID], car, armorID, tf, dt);
                armor_ekf1.Update(z1, {}, dt);
                armor_ekf2.Update(z2, {}, dt);
                l1_ = l1;
                l2_ = l1;
                z_armor =
                    combine_armor_out({armor_ekf1, armor_ekf1}, {offset1, offset1}, {l1, l2}, tf);
            } else {
                car->update_self(dt);
                continue;
            }

            CarKF::ZVec z_car{};
            z_car << z_armor(0), z_armor(1), z_armor(3);

            car->update_car(z_car, dt);
            car->update_z(
                armor_get_odom_z(armor_trackers[armorID][0], tf),
                armor_get_odom_z(armor_trackers[armorID][1], tf),
                armor_get_odom_z(armor_trackers[armorID][2], tf),
                armor_get_odom_z(armor_trackers[armorID][3], tf));

            last_armors = car->get_armor();
            auto min    = 1e7;
            auto index  = 0;
            for (int i = 0; i < 4; i++) {
                auto vec =
                    Eigen::Vector2d{last_armors[i].position->x(), last_armors[i].position->y()};
                auto norm = vec.norm();
                if (norm < min) {
                    index = i;
                    min   = norm;
                }
            }

            if (target == nullptr) {
                target     = std::make_shared<ArmorTarget>(CarTracker{*car}, index);
                target_min = min;
            } else if (target_min > min) {
                target     = std::make_shared<ArmorTarget>(CarTracker{*car}, index);
                target_min = min;
            }
        }

        return target;
    }

    void draw_armors(
        const double& fx, const double& fy, const double& cx, const double& cy, const double& k1,
        const double& k2, const double& k3, const rmcs_description::Tf& tf,
        cv::InputOutputArray image, const cv::Scalar& color, int thickness = 1,
        int lineType = cv::LINE_8, int shift = 0) {
        if (last_armors.empty())
            return;
        auto color_ = color;
        for (const auto& armor3d : last_armors) {
            transform_optimizer::Squad3d::darw_squad(
                fx, fy, cx, cy, k1, k2, k3, tf, image, transform_optimizer::Squad3d(armor3d), false,
                color_, thickness, lineType, shift);
            color_ = color_ / 1.2;
        }
    }

private:
    static void get_grouped_armor(
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

    static double calculateSimilarity(
        const Eigen::Vector3d& t1, const Eigen::Quaterniond& q1, const Eigen::Vector3d& t2,
        const Eigen::Quaterniond& q2) {
        // 计算位移差异
        Eigen::Vector3d vec = t1 - t2;

        // 计算旋转差异
        double rotationDifference = -cos(
            transform_optimizer::get_yaw_from_quaternion(q1)
            - transform_optimizer::get_yaw_from_quaternion(q2));
        // 综合相似度（可以根据需要调整权重）

        // RCLCPP_INFO(
        //     rclcpp::get_logger("armor"), "x:%lf,y:%lf,z:%lf,w:%lf,px:%lf,py:%lf,pz:%lf", vec.x(),
        //     vec.y(), vec.z(), transform_optimizer::get_yaw_from_quaternion(q1),
        //     transform_optimizer::get_yaw_from_quaternion(q2));
        vec.z()                      = 0;
        double translationDifference = vec.norm();
        double similarity            = translationDifference + rotationDifference * 10;
        return similarity;
    }
    constexpr static int distance_in_four(int a, int b) {
        auto err1 = a - b;
        auto err2 = b - a;
        if (err1 < 0)
            err1 += 4;
        if (err2 < 0)
            err2 += 4;
        return err1 > err2 ? err2 : err1;
    }
    std::tuple<ArmorEKF::ZVec, ArmorEKF&, double, double> single_armor_update(
        const std::vector<ArmorPlate3d>& armors, const std::shared_ptr<CarTracker>& car,
        const rmcs_msgs::ArmorID& id, const rmcs_description::Tf& tf, const double& dt) {
        auto car_armors = car->get_armor(dt);
        int index       = index1;
        for (auto& armor : armors) {
            double min = 1e7;
            for (int i = 0; i < 4; i++) {
                if (distance_in_four(i, index1) > 1 && distance_in_four(i, index2) > 1)
                    continue;
                auto distance = calculateSimilarity(
                    *car_armors[i].position, Eigen::Quaterniond{*car_armors[i].rotation},
                    *armor.position, Eigen::Quaterniond{*armor.rotation});
                if (distance < min) {
                    index = i;
                    min   = distance;
                }
            }
        }
        // RCLCPP_INFO(rclcpp::get_logger("l"), "l1:%d", index);

        index1 = index2 = index;

        auto& tracker = armor_trackers[id][index];
        // RCLCPP_INFO(
        //     rclcpp::get_logger("armor"), "x:%lf,y:%lf,z:%lf,w:%lf,px:%lf,py:%lf,pz:%lf",
        //     armors[0].rotation->x(), armors[0].rotation->y(), armors[0].rotation->z(),
        //     armors[0].rotation->w(), armors[0].position->x(), armors[0].position->y(),
        //     armors[0].position->z());
        ArmorEKF::ZVec z{};
        z.setZero();
        auto a0_position = fast_tf::cast<rmcs_description::CameraLink>(armors[0].position, tf);

        auto [l1, l2] = car->get_frame();
        auto l        = l1;
        if (index % 2 != 0)
            l = l2;
        z << a0_position->x(), a0_position->y(), a0_position->z(),
            transform_optimizer::get_yaw_from_quaternion(*armors[0].rotation);

        return {z, tracker, l, -index * std::numbers::pi / 2};
    }

    std::tuple<ArmorEKF::ZVec, ArmorEKF&, double, double, ArmorEKF::ZVec, ArmorEKF&, double, double>
        double_armor_update(
            std::vector<ArmorPlate3d>& armors, const std::shared_ptr<CarTracker>& car,
            const rmcs_msgs::ArmorID& id, const rmcs_description::Tf& tf, const double& dt) {
        auto armor1 = armors[0];
        auto armor2 = armors[1];

        Eigen::Vector2d p1 = {armor1.position->x(), armor1.position->y()};
        Eigen::Vector2d p2 = {armor2.position->x(), armor2.position->y()};
        auto f1            = armor1.rotation->toRotationMatrix() * Eigen::Vector3d::UnitX();
        auto f2            = armor2.rotation->toRotationMatrix() * Eigen::Vector3d::UnitX();
        auto diffp         = p1 - p2;
        auto l1            = abs(diffp.dot(Eigen::Vector2d{f1.x(), f1.y()}.normalized()));
        auto l2            = abs(diffp.dot(Eigen::Vector2d{f2.x(), f2.y()}.normalized()));

        auto car_armors = car->get_armor(dt);
        int index1 = this->index1, index2 = this->index2;
        double min1 = 1e7;
        double min2 = 1e7;
        do {
            for (int i = 0; i < 4; i++) {
                if (distance_in_four(i, this->index1) > 1)
                    continue;
                auto distance = calculateSimilarity(
                    *car_armors[i].position, Eigen::Quaterniond{*car_armors[i].rotation},
                    *armor1.position, Eigen::Quaterniond{*armor1.rotation});
                if (distance < min1) {
                    index1 = i;
                    min1   = distance;
                }
            }
            for (int i = 0; i < 4; i++) {
                if (distance_in_four(i, this->index2) > 1)
                    continue;
                auto distance = calculateSimilarity(
                    *car_armors[i].position, Eigen::Quaterniond{*car_armors[i].rotation},
                    *armor2.position, Eigen::Quaterniond{*armor2.rotation});
                if (distance < min2) {
                    index2 = i;
                    min2   = distance;
                }
            }

        } while (false);
        this->index1 = index1;
        this->index2 = index2;
        if (index1 % 2 != 0)
            std::swap(l1, l2);
        car->update_frame(l1, l2);
        auto [l1_, l2_] = car->get_frame();
        l1              = l1_;
        l2              = l2_;

        if (index1 % 2 != 0)
            std::swap(l1, l2);

        // RCLCPP_INFO(rclcpp::get_logger("l"), "l1:%d,l2: %d", index1, index2);
        auto& tracker1 = armor_trackers[id][index1];
        ArmorEKF::ZVec z1{};
        z1.setZero();
        auto a1_position = fast_tf::cast<rmcs_description::CameraLink>(armor1.position, tf);
        z1 << a1_position->x(), a1_position->y(), a1_position->z(),
            transform_optimizer::get_yaw_from_quaternion(*armor1.rotation);

        auto& tracker2 = armor_trackers[id][index2];
        ArmorEKF::ZVec z2{};
        z2.setZero();
        auto a2_position = fast_tf::cast<rmcs_description::CameraLink>(armor2.position, tf);
        z2 << a2_position->x(), a2_position->y(), a2_position->z(),
            transform_optimizer::get_yaw_from_quaternion(*armor2.rotation);

        return {z1, tracker1, l1, -index1 * std::numbers::pi / 2,
                z2, tracker2, l2, -index2 * std::numbers::pi / 2};
    }

    static ArmorEKF::ZVec combine_armor_out(
        std::tuple<ArmorEKF&, ArmorEKF&> armors, std::tuple<double, double> offset,
        std::tuple<double, double> l, const rmcs_description::Tf& tf) {
        const auto& [a1, a2]           = armors;
        const auto& [offset1, offset2] = offset;
        const auto& [l1, l2]           = l;

        auto z1 = a1.h(a1.OutPut(), {});
        auto z2 = a2.h(a2.OutPut(), {});

        auto z1pos = fast_tf::cast<rmcs_description::OdomImu>(
            rmcs_description::CameraLink::Position{z1(0), z1(1), z1(2)}, tf);
        auto z2pos = fast_tf::cast<rmcs_description::OdomImu>(
            rmcs_description::CameraLink::Position{z2(0), z2(1), z2(2)}, tf);
        z1 << z1pos->x(), z1pos->y(), z1pos->z(), z1(3);
        z2 << z2pos->x(), z2pos->y(), z2pos->z(), z2(3);
        auto p1 = armor_to_car(z1, l1);
        auto p2 = armor_to_car(z2, l2);
        return ArmorEKF::ZVec{
            (p1->x() + p2->x()) / 2, (p1->y() + p2->y()) / 2, (p1->z() + p2->z()) / 2,
            (z1(3) + z2(3) + offset1 + offset2) / 2};
    }

    static rmcs_description::OdomImu::Position
        armor_to_car(const ArmorEKF::ZVec& z1, const double& l) {

        return rmcs_description::OdomImu::Position(
            *rmcs_description::OdomImu::Position(Eigen::Vector3d{z1(0), z1(1), z1(2)})
            + rmcs_description::OdomImu::Rotation(
                  Eigen::AngleAxisd(z1(3), Eigen::Vector3d::UnitZ()))
                      ->toRotationMatrix()
                  * *rmcs_description::OdomImu::DirectionVector(Eigen::Vector3d::UnitX() * l));
    }

    static double armor_get_odom_z(ArmorEKF& ekf, const rmcs_description::Tf& tf) {
        auto zVec     = ekf.h(ekf.OutPut(), ArmorEKF::v_zero);
        auto odom_pos = fast_tf::cast<rmcs_description::OdomImu>(
            rmcs_description::CameraLink::Position(zVec.x(), zVec.y(), zVec.z()), tf);
        return odom_pos->z();
    };

    std::map<rmcs_msgs::ArmorID, std::shared_ptr<CarTracker>> car_trackers;
    std::map<rmcs_msgs::ArmorID, std::vector<ArmorEKF>> armor_trackers;
    std::chrono::steady_clock::time_point last_update_;

    std::vector<ArmorPlate3d> last_armors;
    std::map<rmcs_msgs::ArmorID, std::vector<ArmorPlate3d>> grouped_armor;

    int index1 = 0, index2 = 0;
};
} // namespace rmcs_auto_aim::tracker2::armor
