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
#include "core/tracker/tracker_interface.hpp"
#include "core/transform_optimizer/armor/quadrilateral/quadrilateral.hpp"

#include "core/fire_controller/outpost_firecontroller.hpp"
#include "util/image_viewer/image_viewer.hpp"
#include "util/math.hpp"

namespace rmcs_auto_aim::tracker::outpost {
class OutPostTracker : public armor::ITracker {
    using TFireController = rmcs_auto_aim::fire_controller::TrackerTestController;

public:
    OutPostTracker()
        : target_() {}

    std::shared_ptr<IFireController> Update(
        const std::vector<ArmorPlate3d>& armors,
        const std::chrono::steady_clock::time_point& timestamp,
        const rmcs_description::Tf& tf) override {

        rmcs_description::OdomImu::DirectionVector camera_forward =
            fast_tf::cast<rmcs_description::OdomImu>(
                rmcs_description::CameraLink::DirectionVector(Eigen::Vector3d::UnitX()), tf);
        if (armors.empty()) {
            update_self(
                std::chrono::duration_cast<std::chrono::milliseconds>(timestamp - last_update_)
                        .count()
                    / 1000.0,
                tf);
        } else {
            last_armor_ = armors[0];
        }

        armors_.clear();
        armors_.emplace_back(last_armor_);
        add_armor(
            util::math::get_yaw_from_quaternion(*last_armor_.rotation) + std::numbers::pi * 2 / 3,
            last_armor_.position->z(), {last_armor_.position->x(), last_armor_.position->y(), 0});
        add_armor(
            util::math::get_yaw_from_quaternion(*last_armor_.rotation) + std::numbers::pi * 4 / 3,
            last_armor_.position->z(), {last_armor_.position->x(), last_armor_.position->y(), 0});

        draw_armors(tf, armors, {0, 0, 255});

        update_self(0.15, tf);

        std::sort(armors_.begin(), armors_.end(), [](const ArmorPlate3d& a, const ArmorPlate3d& b) {
            return a.position->norm() < b.position->norm();
        });
        draw_armors(tf, armors, {255, 0, 0});

        target_.in(armors_[0]);

        last_update_ = timestamp;
        return std::make_shared<TFireController>(target_);
    }

    static inline double omega() { return rotate_velocity; };

private:
    void update_self(const double& dt, const rmcs_description::Tf& tf) {
        *last_armor_.rotation = *last_armor_.rotation
                              * Eigen::AngleAxisd(-dt * rotate_velocity, Eigen::Vector3d::UnitZ());

        const auto last_z = last_armor_.position->z();
        *last_armor_.position =
            *rmcs_description::OdomImu::Position(last_armor_.position)
            - Eigen::AngleAxisd(
                  util::math::get_yaw_from_quaternion(*last_armor_.rotation),
                  Eigen::Vector3d::UnitZ())
                      .toRotationMatrix()
                  * *rmcs_description::OdomImu::DirectionVector(Eigen::Vector3d::UnitX()) * radius;
        last_armor_.position->z() = last_z;
    }

    void add_armor(double angle, double z, const Eigen::Vector3d& center) {
        Eigen::Quaterniond forward_armor =
            Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ())
            * Eigen::AngleAxisd(
                -15. / 180. * std::numbers::pi,
                *rmcs_description::OdomImu::DirectionVector(Eigen::Vector3d::UnitY()));

        Eigen::Vector3d ccenter_{};
        ccenter_ << *rmcs_description::OdomImu::Position(center)
                        - Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()).toRotationMatrix()
                              * *rmcs_description::OdomImu::DirectionVector(
                                  Eigen::Vector3d::UnitX())
                              * radius;
        ccenter_.z() = z;

        armors_.emplace_back(
            rmcs_msgs::ArmorID::Unknown, rmcs_description::OdomImu::Position(ccenter_),
            rmcs_description::OdomImu::Rotation(forward_armor));
    }

    static void draw_armors(
        const rmcs_description::Tf& tf, const std::vector<ArmorPlate3d>& armors,
        const cv::Scalar& color) {
        auto color_ = color;

        for (const auto& armor3d : armors) {
            util::ImageViewer::draw(
                transform_optimizer::Quadrilateral3d(armor3d).ToQuadrilateral(tf, false), color_);
            color_ = color_ / 1.5;
        }
    }

    static double armor_get_odom_z(ArmorEKF& ekf, const rmcs_description::Tf& tf) {
        auto zVec     = ekf.h(ekf.OutPut(), ArmorEKF::v_zero);
        auto odom_pos = fast_tf::cast<rmcs_description::OdomImu>(
            rmcs_description::CameraLink::Position(Eigen::Vector3d{zVec.x(), zVec.y(), zVec.z()}),
            tf);
        return odom_pos->z();
    };

    std::chrono::steady_clock::time_point last_update_;

    ArmorPlate3d last_armor_{rmcs_msgs::ArmorID::Unknown, {}, {}};
    std::vector<ArmorPlate3d> armors_;

    static constexpr double radius          = 0.2765;
    static constexpr double rotate_velocity = 0.8 * std::numbers::pi;

    fire_controller::OutPostFireController target_;
};
} // namespace rmcs_auto_aim::tracker::outpost
