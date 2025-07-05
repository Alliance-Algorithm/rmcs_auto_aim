#pragma once
#include "core/tracker/tracker_model.hpp"
#include "opencv2/core/core.hpp"
#include "tracker_interface.hpp"

namespace rmcs_auto_aim::tracker {
class NewTracker : public armor::ITracker {
public:
    NewTracker();
    ~NewTracker();
    NewTracker(const NewTracker&)            = delete;
    NewTracker& operator=(const NewTracker&) = delete;

    std::shared_ptr<IFireController> Update(
        const std::vector<ArmorPlate3d>& armors,
        const std::chrono::steady_clock::time_point& timestamp,
        const rmcs_description::Tf& tf) override;

    void draw_armors(const cv::Scalar& color, const rmcs_description::Tf& tf);

    std::array<ArmorPlate3d, 4> get_armors();

    TrackerModel::XVec get_model_output();

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};
} // namespace rmcs_auto_aim::tracker