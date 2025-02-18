#include "profile.hpp"
#include <opencv2/core/types.hpp>

namespace rmcs_auto_aim::util {

struct Profile::Impl {
    Impl(
        const double& fx, const double& fy, const double& cx, const double& cy, const double& k1,
        const double& k2, const double& k3)
        : intrinsic_parameters((cv::Mat)(cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1))
        , distortion_parameters((cv::Mat)(cv::Mat_<double>(1, 5) << k1, k2, 0, 0, k3)) {}

    const cv::Mat intrinsic_parameters;
    const cv::Mat distortion_parameters;
    std::tuple<int, int> width_height;
};

Profile::Profile(
    const double& fx, const double& fy, const double& cx, const double& cy, const double& k1,
    const double& k2, const double& k3) {
    impl_ = std::make_unique<Impl>(fx, fy, cx, cy, k1, k2, k3);
}

void Profile::set_width_height(const int& width, const int& height) {
    impl_->width_height = std::make_tuple(width, height);
}

const cv::Mat& Profile::get_intrinsic_parameters() { return impl_->intrinsic_parameters; }
const cv::Mat& Profile::get_distortion_parameters() { return impl_->distortion_parameters; }
const std::tuple<int, int>& Profile::get_width_height() { return impl_->width_height; }
} // namespace rmcs_auto_aim::util

std::unique_ptr<rmcs_auto_aim::util::Profile::Impl> rmcs_auto_aim::util::Profile::impl_;