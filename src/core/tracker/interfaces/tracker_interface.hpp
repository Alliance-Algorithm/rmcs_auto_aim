#pragma once

#include <chrono>
#include <memory>
#include <opencv2/core/mat.hpp>
#include <vector>

#include "core/pnpsolver/armor/armor3d.hpp"
#include "core/tracker/interfaces/target_interface.hpp"

namespace rmcs_auto_aim::tracker::armor {
class ITracker {
public:
    virtual std::shared_ptr<ITarget> Update(
        const std::vector<ArmorPlate3d>& armors,
        const std::chrono::steady_clock::time_point& timestamp, const rmcs_description::Tf& tf,
        const cv::Mat& image) = 0;
};
} // namespace rmcs_auto_aim::tracker::armor