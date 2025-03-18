#include "sixty_forty_controller.hpp"
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
class SixFortyController::Impl {
public:
    Impl()
        : tracker_(nullptr)
        , enemy_high_speed_mode(false) {};

    bool check() { return tracker_ != nullptr; };
    [[nodiscard]] std::tuple<bool, rmcs_description::OdomImu::Position>
        UpdateController(double sec, const rmcs_description::Tf& tf) {
        if (tracker_ == nullptr)
            return {false, rmcs_description::OdomImu::Position(0, 0, 0)};
        std::cerr << tracker_->omega() << std::endl;
        if (!enemy_high_speed_mode && abs(tracker_->omega()) > 2 * std::numbers::pi)
            enemy_high_speed_mode = true;
        else if (enemy_high_speed_mode && abs(tracker_->omega()) < 4.0 / 3.0 * std::numbers::pi)
            enemy_high_speed_mode = false;

        enemy_high_speed_mode                       = false;
        rmcs_description::OdomImu::Position ret_pos = rmcs_description::OdomImu::Position(0, 0, 0);
        bool fire_permission                        = false;
        if (enemy_high_speed_mode) {
            auto [l1, l2]            = tracker_->get_frame();
            double min_l             = std::min(l1, l2);
            Eigen::Vector3d position = *tracker_->get_car_position();
            position                 = position
                     + Eigen::AngleAxisd(
                           -std::numbers::pi / 6 * (tracker_->omega() > 0 ? 1 : -1),
                           Eigen::Vector3d::UnitZ())
                           * -position.normalized() * min_l;
            double max    = -1e7;
            int index     = 0;
            auto armors   = tracker_->get_armor(sec);
            auto pos_norm = Eigen::Vector2d(position.x(), position.x()).normalized();

            for (int i = 0; i < 4; i++) {

                auto armor_x = (*armors[i].rotation * Eigen::Vector3d::UnitZ());
                auto len     = pos_norm.dot(Eigen::Vector2d(armor_x.x(), armor_x.y()));
                if (len > max) {
                    index = i;
                    max   = len;
                }
            }
            position.z() = armors[index].position->z();
            if (max < 0.97) {
                fire_permission = false;
                ret_pos         = rmcs_description::OdomImu::Position(position);
            } else {
                fire_permission = true;

                ret_pos = armors[index].position;
            }
        } else {
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
        }

        return {fire_permission, ret_pos};
    }

    void SetTracker(std::shared_ptr<tracker::CarTracker> tracker) { tracker_ = std::move(tracker); }

private:
    std::shared_ptr<tracker::CarTracker> tracker_;
    bool enemy_high_speed_mode = false;
};

[[nodiscard]] std::tuple<bool, rmcs_description::OdomImu::Position>
    SixFortyController::UpdateController(double sec, const rmcs_description::Tf& tf) {
    return pimpl_->UpdateController(sec, tf);
}

void SixFortyController::SetTracker(const std::shared_ptr<tracker::CarTracker>& tracker) {
    pimpl_->SetTracker(tracker);
}

SixFortyController::SixFortyController(const SixFortyController& car_tracker) {
    pimpl_ = std::make_unique<Impl>(*car_tracker.pimpl_);
}

bool SixFortyController::check() { return pimpl_->check(); }

SixFortyController::SixFortyController() { pimpl_ = std::make_unique<Impl>(); }
SixFortyController::~SixFortyController() = default;
