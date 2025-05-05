#include "outpost_controller.hpp"
#include "core/tracker/armor/armor_target.hpp"
#include "core/tracker/car/car_tracker.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/AngleAxis.h>
#include <iostream>
#include <memory>
#include <numbers>
#include <rmcs_description/tf_description.hpp>
#include <utility>

using namespace rmcs_auto_aim::fire_controller;
class OutPostController::Impl {
public:
    Impl()
        : tracker_(nullptr) {};
    std::chrono::steady_clock::time_point get_timestamp() { return tracker_->get_timestamp(); }

    bool check() { return tracker_ != nullptr; };
    [[nodiscard]] std::tuple<bool, rmcs_description::OdomImu::Position>
        UpdateController(double sec, const rmcs_description::Tf& tf) {
        if (tracker_ == nullptr)
            return {false, rmcs_description::OdomImu::Position(0, 0, 0)};

        rmcs_description::OdomImu::Position ret_pos = rmcs_description::OdomImu::Position(0, 0, 0);
        bool fire_permission                        = false;

        ret_pos = tracker::armor::ArmorTarget{tracker::CarTracker(*tracker_)}.Predict(sec, tf);

        auto camera_x = fast_tf::cast<rmcs_description::OdomImu>(
            rmcs_description::CameraLink::DirectionVector(Eigen::Vector3d::UnitX()), tf);

        auto armor_x = ret_pos;

        auto len = camera_x->dot(Eigen::Vector3d(*armor_x));

        if (len < 0.97) {
            fire_permission = false;
        } else {
            fire_permission = true;
        }

        return {fire_permission, ret_pos};
    }

    void SetTracker(std::shared_ptr<tracker::CarTracker> tracker) { tracker_ = std::move(tracker); }
    double get_omega() { return tracker_->omega(); }

private:
    std::shared_ptr<tracker::CarTracker> tracker_;
};

[[nodiscard]] std::tuple<bool, rmcs_description::OdomImu::Position>
    OutPostController::UpdateController(double sec, const rmcs_description::Tf& tf) {
    return pimpl_->UpdateController(sec, tf);
}

void OutPostController::SetTracker(const std::shared_ptr<tracker::CarTracker>& tracker) {
    pimpl_->SetTracker(tracker);
}

double OutPostController::get_omega() { return pimpl_->get_omega(); }
OutPostController::OutPostController(const OutPostController& car_tracker) {
    pimpl_ = std::make_unique<Impl>(*car_tracker.pimpl_);
}

bool OutPostController::check() { return pimpl_->check(); }

std::chrono::steady_clock::time_point OutPostController::get_timestamp() {
    return pimpl_->get_timestamp();
}
OutPostController::OutPostController() { pimpl_ = std::make_unique<Impl>(); }
OutPostController::~OutPostController() = default;
