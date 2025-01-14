#pragma once

#include <utility>

#include "core/tracker_v2/armor/car_tracker.hpp"
#include "core/tracker_v2/target_interface.hpp"

namespace rmcs_auto_aim::tracker2::armor {
class ArmorTarget : public tracker2::ITarget {

public:
    explicit ArmorTarget(rmcs_auto_aim::tracker2::CarTracker car, int index)
        : tracker2::ITarget{}
        , car(std::move(car))
        , index_(index) {};
    rmcs_description::OdomImu::Position Predict(double sec, rmcs_description::Tf tf) override {
        double max    = -1e7;
        int index     = 0;
        auto armors   = car.get_armor(sec);
        auto camera_x = fast_tf::cast<rmcs_description::OdomImu>(
            rmcs_description::CameraLink::DirectionVector(Eigen::Vector3d::UnitX()), tf);
        bool enable = false;
        for (int i = 0; i < 4; i++) {
            auto armor_x = fast_tf::cast<rmcs_description::OdomImu>(
                rmcs_description::OdomImu::DirectionVector(
                    armors[i].rotation->toRotationMatrix() * Eigen::Vector3d::UnitX()),
                tf);
            Eigen::Vector2d vec{};
            vec << Eigen::Vector2d{armors[i].position->x(), armors[i].position->y()};

            auto len = camera_x->dot(Eigen::Vector3d(*armor_x));
            if (index_ == i)
                enable = len > 0;
            if (len > max) {
                index = i;
                max   = len;
            }
        }
        enable = false;
        if (enable)
            return armors[index_].position;
        else {
            return armors[index].position;
        }
    }

private:
    rmcs_auto_aim::tracker2::CarTracker car;
    int index_;
};
} // namespace rmcs_auto_aim::tracker2::armor