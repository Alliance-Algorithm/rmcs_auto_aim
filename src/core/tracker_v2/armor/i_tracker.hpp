#pragma once

#include "core/pnpsolver/armor/armor3d.hpp"
#include "core/tracker/armor/target.hpp"

#include <memory>
#include <vector>

namespace rmcs_auto_aim::tracker2::armor {
class ITracker {
    virtual std::shared_ptr<ArmorTarget> Update(
        const std::vector<ArmorPlate3d>& armors,
        const std::chrono::steady_clock::time_point& timestamp, const rmcs_description::Tf& tf) = 0;
};
} // namespace rmcs_auto_aim::tracker2::armor