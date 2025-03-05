#pragma once

#include "core/tracker/car/car_tracker.hpp"
#include "core/tracker/target_interface.hpp"

namespace rmcs_auto_aim::tracker::armor {
class ArmorTarget : public tracker::ITarget {

public:
    explicit ArmorTarget(const rmcs_auto_aim::tracker::CarTracker& car)
        : tracker::ITarget{}
        , car(car) {};
    rmcs_description::OdomImu::Position Predict(double sec, rmcs_description::Tf tf) override {
        double max    = -1e7;
        int index     = 0;
        auto armors   = car.get_armor(sec);
        auto camera_x = fast_tf::cast<rmcs_description::OdomImu>(
            rmcs_description::CameraLink::DirectionVector(Eigen::Vector3d::UnitX()), tf);
        for (int i = 0; i < 4; i++) {

            Eigen::Vector3d armor_plate_normal = *armors[i].rotation * Eigen::Vector3d::UnitX();
            Eigen::Vector2d vec{};
            vec << Eigen::Vector2d{armors[i].position->x(), armors[i].position->y()};

            auto len = camera_x->dot(armor_plate_normal.normalized());
            if (len > max) {
                index = i;
                max   = len;
            }
        }
        return armors[index].position;
    }

private:
    rmcs_auto_aim::tracker::CarTracker car;
};
} // namespace rmcs_auto_aim::tracker::armor