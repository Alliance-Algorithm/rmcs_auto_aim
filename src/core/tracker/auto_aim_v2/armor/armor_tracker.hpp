#pragma once

#include <chrono>
#include <memory>
#include <vector>

#include <Eigen/Eigen>

#include <opencv2/core/types.hpp>

#include <fast_tf/impl/cast.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_msgs/robot_id.hpp>

#include "core/pnpsolver/armor/armor3d.hpp"

#include "core/tracker/interfaces/target_interface.hpp"

namespace rmcs_auto_aim::tracker::armor {
class ArmorTracker final {
public:
    ArmorTracker();
    ~ArmorTracker();

    std::shared_ptr<ITarget> Update(
        const std::vector<ArmorPlate3d>& armors,
        const std::chrono::steady_clock::time_point& timestamp, const rmcs_description::Tf& tf);

    void draw_armors(const rmcs_description::Tf& tf, const cv::Scalar& color);

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};
} // namespace rmcs_auto_aim::tracker::armor
