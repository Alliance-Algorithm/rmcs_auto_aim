#pragma once

#include <chrono>
#include <map>
#include <memory>
#include <numbers>
#include <tuple>
#include <utility>
#include <vector>

#include <Eigen/Eigen>

#include <fast_tf/impl/cast.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_msgs/robot_id.hpp>

#include "core/pnpsolver/armor/armor3d.hpp"

#include "core/tracker/armor/armor_target.hpp"
#include "core/tracker/armor/filter/armor_ekf.hpp"

#include "core/tracker/car/car_tracker.hpp"
#include "core/tracker/tracker_interface.hpp"
#include "core/transform_optimizer/armor/quadrilateral/quadrilateral.hpp"

#include "util/image_viewer/image_viewer.hpp"
#include "util/math.hpp"

namespace rmcs_auto_aim::tracker::armor {
class ArmorTracker : public ITracker {
public:
    ArmorTracker() {
        car_trackers_[rmcs_msgs::ArmorID::Hero]          = std::make_shared<CarTracker>();
        car_trackers_[rmcs_msgs::ArmorID::Engineer]      = std::make_shared<CarTracker>();
        car_trackers_[rmcs_msgs::ArmorID::InfantryIII]   = std::make_shared<CarTracker>();
        car_trackers_[rmcs_msgs::ArmorID::InfantryIV]    = std::make_shared<CarTracker>();
        car_trackers_[rmcs_msgs::ArmorID::InfantryV]     = std::make_shared<CarTracker>();
        car_trackers_[rmcs_msgs::ArmorID::Sentry]        = std::make_shared<CarTracker>();
        car_trackers_[rmcs_msgs::ArmorID::Outpost]       = std::make_shared<CarTracker>();
        armor_trackers_[rmcs_msgs::ArmorID::Hero]        = std::vector<ArmorEKF>{};
        armor_trackers_[rmcs_msgs::ArmorID::Engineer]    = std::vector<ArmorEKF>{};
        armor_trackers_[rmcs_msgs::ArmorID::InfantryIII] = std::vector<ArmorEKF>{};
        armor_trackers_[rmcs_msgs::ArmorID::InfantryIV]  = std::vector<ArmorEKF>{};
        armor_trackers_[rmcs_msgs::ArmorID::InfantryV]   = std::vector<ArmorEKF>{};
        armor_trackers_[rmcs_msgs::ArmorID::Sentry]      = std::vector<ArmorEKF>{};
        armor_trackers_[rmcs_msgs::ArmorID::Outpost]     = std::vector<ArmorEKF>{};
        for (int i = 0; i < 4; i++)
            for (auto& [_, armors] : armor_trackers_)
                armors.emplace_back();
        grouped_armor_[rmcs_msgs::ArmorID::Hero]        = {};
        grouped_armor_[rmcs_msgs::ArmorID::Engineer]    = {};
        grouped_armor_[rmcs_msgs::ArmorID::InfantryIII] = {};
        grouped_armor_[rmcs_msgs::ArmorID::InfantryIV]  = {};
        grouped_armor_[rmcs_msgs::ArmorID::InfantryV]   = {};
        grouped_armor_[rmcs_msgs::ArmorID::Sentry]      = {};
        grouped_armor_[rmcs_msgs::ArmorID::Outpost]     = {};
    }

    std::shared_ptr<ITarget> Update(
        const std::vector<ArmorPlate3d>& armors,
        const std::chrono::steady_clock::time_point& timestamp,
        const rmcs_description::Tf& tf) override {

        std::shared_ptr<ITarget> target = nullptr;

        double dt    = std::chrono::duration<double>(timestamp - last_update_).count();
        last_update_ = timestamp;
        dt           = std::clamp(dt, 0., .5);

        get_grouped_armor(grouped_armor_, armors);

        for (const auto& [armorID, car] : car_trackers_) {
            auto len = grouped_armor_[armorID].size();

            int nearest_armor_index_in_detected;

            if (len > 0) {
                auto armor_id = calculate_armor_id(
                    grouped_armor_[armorID], car->get_armor(dt), tf,
                    nearest_armor_index_in_detected);
                if (grouped_armor_[armorID].size() > 1)
                    update_car_frame(
                        grouped_armor_[armorID][0], grouped_armor_[armorID][0], armor_id[0], car);

                Eigen::Vector<double, 4> armor_z{};

                Eigen::Vector3d armor_in_camera = *fast_tf::cast<rmcs_description::CameraLink>(
                    grouped_armor_[armorID][nearest_armor_index_in_detected].position, tf);

                armor_z << armor_in_camera,
                    util::math::get_yaw_from_quaternion(
                        *grouped_armor_[armorID][nearest_armor_index_in_detected].rotation);
                armor_trackers_[armorID][armor_id[nearest_armor_index_in_detected]].Update(
                    armor_z, {}, dt);

                const auto& [l1, l2]                        = car->get_frame();
                rmcs_description::OdomImu::Position car_pos = armor_to_car(
                    armor_trackers_[armorID][armor_id[nearest_armor_index_in_detected]],
                    armor_id[nearest_armor_index_in_detected] % 2 ? l2 : l1);

                car->update_car(
                    {car_pos->x(), car_pos->y(),
                     armor_trackers_[armorID][armor_id[nearest_armor_index_in_detected]].OutPut()(
                         3)},
                    dt);

                Eigen::Vector<double, 4> car_armor_height = car->get_armor_height();
                for (int i = 0; i < 4; i++)
                    if (i == armor_id[0] || armor_id.size() > 1 ? (i == armor_id[1]) : false)
                        car_armor_height(i) = armor_get_odom_z(armor_trackers_[armorID][i], tf);
                car->update_z(
                    car_armor_height(0), car_armor_height(1), car_armor_height(2),
                    car_armor_height(3));
                target = std::make_shared<ArmorTarget>(CarTracker{*car});

                last_car_id_ = armorID;
            } else {
                car->update_self(dt);
                continue;
            }
        }

        last_armors1_ = car_trackers_[last_car_id_]->get_armor(dt);
        last_armors2_ = car_trackers_[last_car_id_]->get_armor();
        return target;
    }

    void draw_armors(const rmcs_description::Tf& tf, const cv::Scalar& color) {
        if (last_armors1_.empty())
            return;
        auto color_ = color;
        for (const auto& armor3d : last_armors1_) {
            util::ImageViewer::draw(
                transform_optimizer::Quadrilateral3d(armor3d).ToQuadrilateral(tf, false), color_);
            color_ = color_ / 1.2;
        }
        for (const auto& armor3d : last_armors2_) {
            util::ImageViewer::draw(
                transform_optimizer::Quadrilateral3d(armor3d).ToQuadrilateral(tf, false),
                {0, 0, 255});
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

    constexpr static int distance_in_four(int a, int b) {
        auto err1 = a - b;
        auto err2 = b - a;
        while (err1 < 0)
            err1 += 4;
        while (err2 < 0)
            err2 += 4;
        return err1 > err2 ? err2 : err1;
    }

    ///
    /// return {index_detected, index_predicted};
    ///
    static std::tuple<int, int> calculate_nearest_armor_id(
        const std::vector<ArmorPlate3d>& armors_detected,
        const std::vector<ArmorPlate3d>& armors_predicted,
        rmcs_description::OdomImu::DirectionVector camera_forward) {

        double min;
        int index_detected  = 0;
        int index_predicted = 0;

        min = 1e7;
        for (int i = 0; i < (int)armors_detected.size(); i++) {
            Eigen::Vector3d armor_plate_normal =
                *armors_detected[i].rotation * Eigen::Vector3d::UnitX();

            double dot_val = armor_plate_normal.normalized().dot(*camera_forward);
            if (dot_val < min) {
                min            = dot_val;
                index_detected = i;
            }
        }

        min = 1e7;
        for (int i = 0; i < 4; i++) {
            Eigen::Vector3d armor_plate_normal =
                *armors_predicted[i].rotation * Eigen::Vector3d::UnitX();

            double dot_val = armor_plate_normal.normalized().dot(*camera_forward);
            if (dot_val < min) {
                min             = dot_val;
                index_predicted = i;
            }
        }
        return {index_detected, index_predicted};
    }

    ///
    /// return index_predicted;
    ///
    static int calculate_nearest_armor_id(
        ArmorPlate3d armors_detected, const std::vector<ArmorPlate3d>& armors_predicted,
        int index_predicted) {

        double min;
        double armor_angular_distance;

        min                    = 1e7;
        armor_angular_distance = armors_detected.rotation->angularDistance(
            *armors_predicted[(index_predicted + 1) % 4].rotation);
        if (armor_angular_distance < min) {
            min             = armor_angular_distance;
            index_predicted = (index_predicted + 1) % 4;
        }
        armor_angular_distance = armors_detected.rotation->angularDistance(
            *armors_predicted[(index_predicted + 3) % 4].rotation);
        if (armor_angular_distance < min) {
            min             = armor_angular_distance;
            index_predicted = (index_predicted + 3) % 4;
        }

        return index_predicted;
    }

    static std::vector<int> calculate_armor_id(
        const std::vector<ArmorPlate3d>& armors_detected,
        const std::vector<ArmorPlate3d>& armors_predicted, const rmcs_description::Tf& tf,
        int& nearest_armor_id) {

        rmcs_description::OdomImu::DirectionVector camera_forward =
            fast_tf::cast<rmcs_description::OdomImu>(
                rmcs_description::CameraLink::DirectionVector(Eigen::Vector3d::UnitX()), tf);

        const auto [detected_index, predicted_index] =
            calculate_nearest_armor_id(armors_detected, armors_predicted, camera_forward);

        nearest_armor_id = detected_index;

        if (armors_detected.size() == 1)
            return {predicted_index};

        const int predicted_index_2nd = calculate_nearest_armor_id(
            armors_predicted[1 - detected_index], armors_predicted, predicted_index);

        if (detected_index == 0)
            return {predicted_index, predicted_index_2nd};
        else
            return {predicted_index_2nd, predicted_index};
    }

    static void update_car_frame(
        const ArmorPlate3d& armor1, const ArmorPlate3d& armor2, const int& armor1_index,
        const std::shared_ptr<CarTracker>& car_tracker) {
        auto angle_err =
            util::math::get_angle_err_rad_from_quaternion(*armor1.rotation, *armor2.rotation);
        angle_err = abs(angle_err - std::numbers::pi / 2);

        if (angle_err > armor_tuple_angular_epsilon)
            return;

        Eigen::Vector3d diffp          = *armor1.position - *armor2.position;
        Eigen::Vector3d armor1_forward = *armor1.rotation * Eigen::Vector3d::UnitX();
        Eigen::Vector3d armor2_forward = *armor2.rotation * Eigen::Vector3d::UnitX();

        Eigen::Vector2d diffp_2d{diffp.x(), diffp.y()};
        Eigen::Vector2d armor1_forward2d{armor1_forward.x(), armor1_forward.y()};
        Eigen::Vector2d armor2_forward2d{armor2_forward.x(), armor2_forward.y()};
        armor1_forward2d << armor1_forward2d.normalized();
        armor2_forward2d << armor2_forward2d.normalized();

        double l1 = abs(diffp_2d.dot(armor1_forward2d));
        double l2 = abs(diffp_2d.dot(armor2_forward2d));

        if (armor1_index % 2)
            std::swap(l1, l2);

        car_tracker->update_frame(l1, l2);
    }

    static rmcs_description::OdomImu::Position armor_to_car(ArmorEKF& ekf, const double& l) {
        ArmorEKF::ZVec z1 = ekf.h(ekf.OutPut(), {});
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

    std::map<rmcs_msgs::ArmorID, std::shared_ptr<CarTracker>> car_trackers_;
    std::map<rmcs_msgs::ArmorID, std::vector<ArmorEKF>> armor_trackers_;
    std::chrono::steady_clock::time_point last_update_;

    std::vector<ArmorPlate3d> last_armors1_;
    std::vector<ArmorPlate3d> last_armors2_;
    std::map<rmcs_msgs::ArmorID, std::vector<ArmorPlate3d>> grouped_armor_;

    static constexpr double armor_tuple_angular_epsilon = 1e-1;

    // 改为多车 这是测试版本
    rmcs_msgs::ArmorID last_car_id_ = rmcs_msgs::ArmorID::Hero;
};
} // namespace rmcs_auto_aim::tracker::armor
