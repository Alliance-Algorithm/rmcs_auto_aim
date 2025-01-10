#pragma once

#include <utility>

#include "core/tracker_v2/armor/car_tracker.hpp"
#include "core/tracker_v2/target_interface.hpp"

namespace rmcs_auto_aim::tracker2::armor {
class ArmorTarget : public tracker2::ITarget {

public:
    ArmorTarget(rmcs_auto_aim::tracker2::CarTracker car, int armor_index)
        : tracker2::ITarget{}
        , car(std::move(car))
        , armor_index(armor_index){};
    rmcs_description::OdomImu::Position Predict(double sec) override {
        return car.get_armor(sec)[armor_index].position;
    }

private:
    rmcs_auto_aim::tracker2::CarTracker car;
    const int armor_index;
};
} // namespace rmcs_auto_aim::tracker2::armor