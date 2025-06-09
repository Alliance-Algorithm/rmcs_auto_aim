#include "outpost_controller.hpp"
#include "core/tracker/armor/armor_target.hpp"
#include "core/tracker/armor/armor_target_for_outpost.hpp"
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
        // std::cerr << tracker_->omega() << std::endl;
        rmcs_description::OdomImu::Position ret_pos = rmcs_description::OdomImu::Position(0, 0, 0);
        bool fire_permission                        = false;
        auto [l1, l2]                               = tracker_->get_frame();
        double min_l                                = std::min(l1, l2);
        Eigen::Vector3d position                    = *tracker_->get_outpost_position();
        position                                    = position - position.normalized() * min_l;
        double max                                  = -1e7;
        int index                                   = 0;
        auto armors                                 = tracker_->get_armor(sec);
        auto pos_norm = Eigen::Vector2d(position.x(), position.y()).normalized();

        for (int i = 0; i < 3; i++) {

            auto armor_x = (*armors[i].rotation * Eigen::Vector3d::UnitX());
            auto len     = pos_norm.dot(Eigen::Vector2d(armor_x.x(), armor_x.y()).normalized());
            if (len > max) {
                index = i;
                max   = len;
            }
        }
        position.z() = armors[(index + 3) % 4].position->z();
        if (index % 2 == 0) {
            position.z() = armors[index].position->z();
            if (max > 0.99)
                fire_permission = true;
        }
        ret_pos = rmcs_description::OdomImu::Position(position);

        return {fire_permission, ret_pos};
    }

    void SetTracker(std::shared_ptr<tracker::OutPostTracker> tracker) {
        tracker_ = std::move(tracker);
    }
    double get_omega() { return tracker_->omega(); }

private:
    std::shared_ptr<tracker::OutPostTracker> tracker_;
};

[[nodiscard]] std::tuple<bool, rmcs_description::OdomImu::Position>
    OutPostController::UpdateController(double sec, const rmcs_description::Tf& tf) {
    return pimpl_->UpdateController(sec, tf);
}

void OutPostController::SetTracker(const std::shared_ptr<tracker::OutPostTracker>& tracker) {
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
