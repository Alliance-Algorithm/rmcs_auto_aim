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

#include "core/fire_controller/fire_controller.hpp"
#include "core/fire_controller/noname_controller.hpp"
#include "core/fire_controller/tracker_test_controller.hpp"
#include "core/tracker/armor/filter/armor_ekf.hpp"

#include "core/tracker/car/car_tracker.hpp"
#include "core/tracker/outpost/filter/outpost_ekf.hpp"
#include "core/tracker/outpost/outpost_tracker.hpp"
#include "core/tracker/tracker_interface.hpp"
#include "core/transform_optimizer/armor/quadrilateral/quadrilateral.hpp"

#include "util/image_viewer/image_viewer.hpp"
#include "util/math.hpp"

namespace rmcs_auto_aim::tracker::armor {
class ArmorTracker : public ITracker {
    using TFireController = rmcs_auto_aim::fire_controller::TrackerTestController;

public:
    ArmorTracker()
        : target_() {}

    std::shared_ptr<IFireController> Update(
        const std::vector<ArmorPlate3d>& armors,
        const std::chrono::steady_clock::time_point& timestamp,
        const rmcs_description::Tf& tf) override {
        target_.SetTracker(nullptr);


        return target_.check() ? std::make_shared<TFireController>(target_) : nullptr;
    }

    void draw_armors(const rmcs_description::Tf& tf, const cv::Scalar& color) {
        if (last_armors1_.empty())
            return;
        auto color_ = color;

        for (const auto& armor3d : last_armors1_) {
            util::ImageViewer::draw(
                transform_optimizer::Quadrilateral3d(armor3d).ToQuadrilateral(tf, false), color_);
            color_ = color_ / 1.5;
        }
        color_ = {255 - color(0), 255 - color(1), 255 - color(2)};
        for (const auto& armor3d : last_armors2_) {
            util::ImageViewer::draw(
                transform_optimizer::Quadrilateral3d(armor3d).ToQuadrilateral(tf, false), color_);
            color_ = color_ / 1.5;
        }
    }

private:
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

        min = -1e7;
        for (int i = 0; i < (int)armors_detected.size(); i++) {
            Eigen::Vector3d armor_plate_normal =
                *armors_detected[i].rotation * Eigen::Vector3d::UnitX();

            double dot_val = armor_plate_normal.normalized().dot(*camera_forward);
            if (dot_val > min) {
                min            = dot_val;
                index_detected = i;
            }
        }

        min = 1e7;
        for (int i = 0; i < 4; i++) {
            double dot_val =
                abs(util::math::get_yaw_from_quaternion(*armors_detected[index_detected].rotation)
                    - util::math::get_yaw_from_quaternion(*armors_predicted[i].rotation));
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
        int index = index_predicted;
        double armor_angular_distance;

        min = 1e7;
        armor_angular_distance =
            abs(util::math::get_yaw_from_quaternion(*armors_detected.rotation)
                - util::math::get_yaw_from_quaternion(
                    *armors_predicted[(index_predicted + 1) % 4].rotation));
        if (armor_angular_distance < min) {
            min   = armor_angular_distance;
            index = (index_predicted + 1) % 4;
        }
        armor_angular_distance =
            abs(util::math::get_yaw_from_quaternion(*armors_detected.rotation)
                - util::math::get_yaw_from_quaternion(
                    *armors_predicted[(index_predicted + 3) % 4].rotation));
        if (armor_angular_distance < min) {
            index = (index_predicted + 3) % 4;
        }
        return index;
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
        // 这里如果没有扫描到可能出问题吧
        nearest_armor_id = detected_index;

        if (armors_detected.size() == 1)
            return {predicted_index};

        // 这里（1 - detected_index）可能出问题吧
        const int predicted_index_2nd = calculate_nearest_armor_id(
            armors_detected[1 - detected_index], armors_predicted, predicted_index);

        if (detected_index == 0)
            return {predicted_index, predicted_index_2nd};
        else
            return {predicted_index_2nd, predicted_index};
    }

    static rmcs_description::OdomImu::Position
        armor_to_car(ArmorEKF& ekf, const double& l, const auto& tf) {
        ArmorEKF::ZVec z1 = ekf.h(ekf.OutPut(), {});
        return rmcs_description::OdomImu::Position(
            *fast_tf::cast<rmcs_description::OdomImu>(
                rmcs_description::CameraLink::Position(Eigen::Vector3d{z1(0), z1(1), z1(2)}), tf)
            + rmcs_description::OdomImu::Rotation(
                  Eigen::AngleAxisd(z1(3), Eigen::Vector3d::UnitZ()))
                      ->toRotationMatrix()
                  * *rmcs_description::OdomImu::DirectionVector(Eigen::Vector3d::UnitX() * l));
    }

    static double armor_get_odom_z(ArmorEKF& ekf, const rmcs_description::Tf& tf) {
        auto zVec     = ekf.h(ekf.OutPut(), ArmorEKF::v_zero);
        auto odom_pos = fast_tf::cast<rmcs_description::OdomImu>(
            rmcs_description::CameraLink::Position(Eigen::Vector3d{zVec.x(), zVec.y(), zVec.z()}),
            tf);
        return odom_pos->z();
    };

    OutPostTracker outpost_tracker_;
    std::vector<OutPostEKF> armor_trackers_;
    std::chrono::steady_clock::time_point last_update_;

    std::vector<ArmorPlate3d> last_armors1_;
    std::vector<ArmorPlate3d> last_armors2_;
    std::vector<ArmorPlate3d> grouped_armor_;

    static constexpr double armor_tuple_angular_epsilon  = 1e-1;
    static constexpr double armor_tuple_distance_epsilon = 1e-1;

    TFireController target_;
    double nearest_distance_ = 0.0;
};
} // namespace rmcs_auto_aim::tracker::armor
