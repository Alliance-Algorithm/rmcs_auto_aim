#pragma once

#include "core/tracker/interfaces/tracker_interface.hpp"
#include <memory>

namespace rmcs_auto_aim::tracker {

class FLiNGTracker final : public rmcs_auto_aim::tracker::armor::ITracker {
public:
    FLiNGTracker();
    ~FLiNGTracker();

    std::shared_ptr<ITarget> Update(
        const std::vector<ArmorPlate3d>& armors,
        const std::chrono::steady_clock::time_point& timestamp, const rmcs_description::Tf& tf,
        const cv::Mat& image) final;

    void draw_armors(const rmcs_description::Tf& tf, const cv::Scalar& color) const;

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};
} // namespace rmcs_auto_aim::tracker