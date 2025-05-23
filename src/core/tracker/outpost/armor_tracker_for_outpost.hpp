#pragma once

#include <chrono>
#include <memory>
#include <numbers>
#include <tuple>
#include <vector>

#include <Eigen/Eigen>

#include <fast_tf/impl/cast.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_msgs/robot_id.hpp>

#include "core/fire_controller/outpost_controller.hpp"
#include "core/pnpsolver/armor/armor3d.hpp"

#include "core/fire_controller/tracker_test_controller.hpp"
#include "core/tracker/armor/filter/armor_ekf.hpp"

#include "core/fire_controller/fire_controller.hpp"
#include "core/tracker/outpost/outpost_tracker.hpp"
#include "core/tracker/tracker_interface.hpp"
#include "core/transform_optimizer/armor/quadrilateral/quadrilateral.hpp"

#include "util/image_viewer/image_viewer.hpp"
#include "util/math.hpp"

namespace rmcs_auto_aim::tracker::armor {
class OutPostArmorTracker : public ITracker {
    using TFireController = rmcs_auto_aim::fire_controller::TrackerTestController;

public:
    OutPostArmorTracker()
        : target_() {}

    std::shared_ptr<IFireController> Update(
        const std::vector<ArmorPlate3d>& armors,
        const std::chrono::steady_clock::time_point& timestamp,
        const rmcs_description::Tf& tf) override {
        target_.SetTracker(nullptr);

        last_armors1_ = outpost_tracker_.get_armor();
        last_update_  = timestamp;

        const auto outpost_armors = get_outpost_armor(armors);
        // std::cerr << "hello"
        if (outpost_armors.size() > 0) {
            nearest_distance_ = 1e7;

            double dt                = outpost_tracker_.get_dt(timestamp);
            last_armors2_            = outpost_tracker_.get_armor(dt);
            auto last_detected_armor = outpost_tracker_.get_armor();
            auto armor_id            = calculate_armor_id(outpost_armors, last_detected_armor, tf);

            Eigen::Vector<double, 4> armor_z{};

            armor_z(0) = outpost_armors[0].position->x();
            armor_z(1) = outpost_armors[0].position->y();
            armor_z(2) = outpost_armors[0].position->z();
            armor_z(3) = util::math::get_yaw_from_quaternion(*outpost_armors[0].rotation);

            rmcs_description::OdomImu::Position outpost_pos =
                armor_to_outpost(armor_z, outpost_radius, tf);
            // std::cerr << armor_z << std::endl << *outpost_pos << std::endl << std::endl;
            outpost_tracker_.update_outpost(
                {outpost_pos->x(), outpost_pos->y(), outpost_pos->z(),
                 armor_z(3) - armor_id * std::numbers::pi * 2 / 3},
                dt);
            // outpost_tracker_.update_outpost(
            //     {armor_z(0), armor_z(1), armor_z(2),
            //      armor_z(3) - armor_id * std::numbers::pi * 2 / 3},
            //     dt);

            target_.SetTracker(std::make_shared<OutPostTracker>(outpost_tracker_));
        } else {
            // outpost_tracker_.update_self(outpost_tracker_.get_dt(timestamp));
        }

        return target_.check() ? std::make_shared<fire_controller::OutPostController>(target_)
                               : nullptr;
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
    static inline std::vector<ArmorPlate3d>
        get_outpost_armor(const std::vector<ArmorPlate3d>& armors) {
        std::vector<ArmorPlate3d> outpost_armors;
        for (const auto& armor : armors) {
            if (armor.id == rmcs_msgs::ArmorID::Outpost) {
                outpost_armors.emplace_back(armor);
            }
        }
        return outpost_armors;
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

        std::vector<double> angular_distance{};
        for (int i = 0; i < 3; i++) {
            angular_distance.emplace_back(
                armors_detected[index_detected].rotation->angularDistance(
                    *armors_predicted[i].rotation));
        }
        for (int i = 0; i < 3; i++) {
            if (angular_distance[i] > angular_distance[(i + 1) % 3]
                && angular_distance[i] > angular_distance[(i + 2) % 3])
                continue;

            if (angular_distance[i] < angular_distance[(i + 1) % 3]
                && angular_distance[i] < angular_distance[(i + 2) % 3])
                continue;
            index_predicted = i;
        }
        return {index_detected, index_predicted};
    }

    ///
    /// return index_predicted;
    ///

    // static int calculate_nearest_armor_id(
    //     ArmorPlate3d armors_detected, const std::vector<ArmorPlate3d>& armors_predicted,
    //     int index_predicted) {

    //     double min;
    //     int index = index_predicted;
    //     double armor_angular_distance;

    //     min = 1e7;
    //     armor_angular_distance =
    //         abs(util::math::get_yaw_from_quaternion(*armors_detected.rotation)
    //             - util::math::get_yaw_from_quaternion(
    //                 *armors_predicted[(index_predicted + 1) % 3].rotation));
    //     if (armor_angular_distance < min) {
    //         min   = armor_angular_distance;
    //         index = (index_predicted + 1) % 3;
    //     }
    //     armor_angular_distance =
    //         abs(util::math::get_yaw_from_quaternion(*armors_detected.rotation)
    //             - util::math::get_yaw_from_quaternion(
    //                 *armors_predicted[(index_predicted + 2) % 3].rotation));
    //     if (armor_angular_distance < min) {
    //         index = (index_predicted + 2) % 3;
    //     }
    //     return index;
    // }

    static int calculate_armor_id(
        const std::vector<ArmorPlate3d>& armors_detected,
        const std::vector<ArmorPlate3d>& armors_predicted, const rmcs_description::Tf& tf) {

        const rmcs_description::OdomImu::DirectionVector camera_forward =
            fast_tf::cast<rmcs_description::OdomImu>(
                rmcs_description::CameraLink::DirectionVector(Eigen::Vector3d::UnitX()), tf);

        const auto [detected_index, predicted_index] =
            calculate_nearest_armor_id(armors_detected, armors_predicted, camera_forward);

        return predicted_index;
    }

    static rmcs_description::OdomImu::Position
        armor_to_outpost(const Eigen::Vector4d& z, const double& l, const auto& tf) {
        return rmcs_description::OdomImu::Position(
            *rmcs_description::OdomImu::Position(Eigen::Vector3d{z(0), z(1), z(2)})
            + rmcs_description::OdomImu::Rotation(Eigen::AngleAxisd(z(3), Eigen::Vector3d::UnitZ()))
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
    std::chrono::steady_clock::time_point last_update_;

    std::vector<ArmorPlate3d> last_armors1_;
    std::vector<ArmorPlate3d> last_armors2_;

    static constexpr double armor_tuple_angular_epsilon  = 1e-1;
    static constexpr double armor_tuple_distance_epsilon = 1e-1;

    static constexpr double outpost_radius = 0.2765;

    fire_controller::OutPostController target_;
    double nearest_distance_ = 0.0;
};
} // namespace rmcs_auto_aim::tracker::armor
