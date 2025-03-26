#include "tracker_test_controller.hpp"
#include "core/tracker/armor/armor_target.hpp"
#include "core/tracker/car/car_tracker.hpp"
#include "noname_controller.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/AngleAxis.h>
#include <memory>
#include <numbers>
#include <rmcs_description/tf_description.hpp>
#include <utility>

using namespace rmcs_auto_aim::fire_controller;
class TrackerTestController::Impl {
public:
    Impl()
        : tracker_(nullptr)
        , enemy_high_speed_mode(false) {};

    bool check() { return tracker_ != nullptr; };
    [[nodiscard]] std::tuple<bool, rmcs_description::OdomImu::Position>
        UpdateController(double sec, const rmcs_description::Tf&) {
        if (tracker_ == nullptr)
            return {false, rmcs_description::OdomImu::Position(0, 0, 0)};
        // std::cerr << tracker_->omega() << std::endl;
        if (!enemy_high_speed_mode && abs(tracker_->omega()) > 4 * std::numbers::pi)
            enemy_high_speed_mode = true;
        else if (enemy_high_speed_mode && abs(tracker_->omega()) < 3 * std::numbers::pi)
            enemy_high_speed_mode = false;

        rmcs_description::OdomImu::Position ret_pos = rmcs_description::OdomImu::Position(0, 0, 0);
        bool fire_permission                        = false;
        auto [l1, l2]                               = tracker_->get_frame();
        double min_l                                = std::min(l1, l2);
        Eigen::Vector3d position                    = *tracker_->get_car_position();
        position                                    = position - position.normalized() * min_l;
        double max                                  = -1e7;
        int index                                   = 0;
        auto armors                                 = tracker_->get_armor(sec);
        auto pos_norm = Eigen::Vector2d(position.x(), position.y()).normalized();

        for (int i = 0; i < 4; i++) {

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

    void SetTracker(std::shared_ptr<tracker::CarTracker> tracker) { tracker_ = std::move(tracker); }
    double get_omega() { return tracker_->omega(); }
    std::chrono::steady_clock::time_point get_timestamp() { return tracker_->get_timestamp(); }

private:
    std::shared_ptr<tracker::CarTracker> tracker_;
    bool enemy_high_speed_mode = false;
};

[[nodiscard]] std::tuple<bool, rmcs_description::OdomImu::Position>
    TrackerTestController::UpdateController(double sec, const rmcs_description::Tf& tf) {
    return pimpl_->UpdateController(sec, tf);
}

void TrackerTestController::SetTracker(const std::shared_ptr<tracker::CarTracker>& tracker) {
    pimpl_->SetTracker(tracker);
}

double TrackerTestController::get_omega() { return pimpl_->get_omega(); }
TrackerTestController::TrackerTestController(const TrackerTestController& car_tracker) {
    pimpl_ = std::make_unique<Impl>(*car_tracker.pimpl_);
}

bool TrackerTestController::check() { return pimpl_->check(); }

std::chrono::steady_clock::time_point TrackerTestController::get_timestamp() {
    return pimpl_->get_timestamp();
}

TrackerTestController::TrackerTestController() { pimpl_ = std::make_unique<Impl>(); }
TrackerTestController::~TrackerTestController() = default;
