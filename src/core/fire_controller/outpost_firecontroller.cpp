#include "outpost_firecontroller.hpp"
#include "core/tracker/armor/armor_target.hpp"
#include "core/tracker/car/car_tracker.hpp"
#include "util/math.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/AngleAxis.h>
#include <iostream>
#include <memory>
#include <numbers>
#include <rmcs_description/tf_description.hpp>
#include <utility>

using namespace rmcs_auto_aim::fire_controller;
class OutPostFireController::Impl {
public:
    Impl()
        : tracker_(nullptr) {};
    std::chrono::steady_clock::time_point get_timestamp() { return tracker_->get_timestamp(); }

    bool check() { return tracker_ != nullptr; };
    [[nodiscard]] std::tuple<bool, rmcs_description::OdomImu::Position>
        UpdateController(double sec, const rmcs_description::Tf& tf) {
        if (tracker_ == nullptr)
            return {false, rmcs_description::OdomImu::Position(0, 0, 0)};

        bool fire_permission = false;

        auto camera_x = fast_tf::cast<rmcs_description::OdomImu>(
            rmcs_description::CameraLink::DirectionVector(Eigen::Vector3d::UnitX()), tf);

        auto len = camera_x->dot(Eigen::Vector3d(*last_armor_.position));

        if (len < 0.97) {
            fire_permission = false;
        } else {
            fire_permission = true;
        }

        return {fire_permission, last_armor_.position};
    }

    void in(const ArmorPlate3d& armor) { last_armor_ = armor; }

    void SetTracker(std::shared_ptr<tracker::CarTracker> tracker) { tracker_ = std::move(tracker); }
    double get_omega() { return -0.8 * std::numbers::pi; }

private:
    void update_self(const double& dt, const rmcs_description::Tf& tf) {
        *last_armor_.rotation =
            *last_armor_.rotation
            * Eigen::AngleAxisd(dt * -0.8 * std::numbers::pi, Eigen::Vector3d::UnitZ());

        const auto last_z = last_armor_.position->z();
        *last_armor_.position =
            *rmcs_description::OdomImu::Position(last_armor_.position)
            - Eigen::AngleAxisd(
                  util::math::get_yaw_from_quaternion(*last_armor_.rotation),
                  Eigen::Vector3d::UnitZ())
                      .toRotationMatrix()
                  * *rmcs_description::OdomImu::DirectionVector(Eigen::Vector3d::UnitX()) * 0.2765;
        last_armor_.position->z() = last_z;
    }

    std::shared_ptr<tracker::CarTracker> tracker_;
    ArmorPlate3d last_armor_{rmcs_msgs::ArmorID::Unknown, {}, {}};
};

[[nodiscard]] std::tuple<bool, rmcs_description::OdomImu::Position>
    OutPostFireController::UpdateController(double sec, const rmcs_description::Tf& tf) {
    return pimpl_->UpdateController(sec, tf);
}

void OutPostFireController::SetTracker(const std::shared_ptr<tracker::CarTracker>& tracker) {
    pimpl_->SetTracker(tracker);
}

double OutPostFireController::get_omega() { return pimpl_->get_omega(); }
OutPostFireController::OutPostFireController(const OutPostFireController& car_tracker) {
    pimpl_ = std::make_unique<Impl>(*car_tracker.pimpl_);
}

bool OutPostFireController::check() { return pimpl_->check(); }

void OutPostFireController::in(const ArmorPlate3d& armor) { pimpl_->in(armor); }

std::chrono::steady_clock::time_point OutPostFireController::get_timestamp() {
    return pimpl_->get_timestamp();
}
OutPostFireController::OutPostFireController() { pimpl_ = std::make_unique<Impl>(); }
OutPostFireController::~OutPostFireController() = default;
