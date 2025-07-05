#pragma once
#include "opencv2/core/core.hpp"
#include "tracker_interface.hpp"

namespace rmcs_auto_aim::tracker {
class NewTracker : public armor::ITracker {
public:
    std::shared_ptr<IFireController> Update(
        const std::vector<ArmorPlate3d>& armors,
        const std::chrono::steady_clock::time_point& timestamp,
        const rmcs_description::Tf& tf) override;

    inline void draw_armors(const cv::Scalar& color, const rmcs_description::Tf& tf);

    inline void initial_timestamp(const std::chrono::steady_clock::time_point& timestamp);

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};
} // namespace rmcs_auto_aim::tracker