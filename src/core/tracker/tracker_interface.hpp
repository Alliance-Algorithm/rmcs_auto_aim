#pragma once

#include <chrono>
#include <memory>
#include <vector>

#include "core/fire_controller/fire_controller.hpp"
#include "core/pnpsolver/armor/armor3d.hpp"

namespace rmcs_auto_aim::tracker::armor {
class ITracker {
public:
    virtual std::shared_ptr<IFireController> Update(
        const std::vector<ArmorPlate3d>& armors,
        const std::chrono::steady_clock::time_point& timestamp, const rmcs_description::Tf& tf) = 0;
};
} // namespace rmcs_auto_aim::tracker::armor