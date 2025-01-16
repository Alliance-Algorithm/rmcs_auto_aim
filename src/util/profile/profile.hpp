#pragma once

#include <memory>
#include <opencv2/core/mat.hpp>
namespace rmcs_auto_aim::util {
class Profile {
public:
    Profile(
        const double& fx, const double& fy, const double& cx, const double& cy, const double& k1,
        const double& k2, const double& k3) {
        pimpl_ = std::make_unique<Profile_>(fx, fy, cx, cy, k1, k2, k3);
    }

    static inline const cv::Mat& get_intrinsic_parameters() { return pimpl_->intrinsic_parameters; }
    static inline const cv::Mat& get_distortion_parameters() {
        return pimpl_->distortion_parameters;
    }

private:
    struct Profile_ {
        Profile_(
            const double& fx, const double& fy, const double& cx, const double& cy,
            const double& k1, const double& k2, const double& k3)
            : intrinsic_parameters(
                  (cv::Mat)(cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1))
            , distortion_parameters((cv::Mat)(cv::Mat_<double>(1, 5) << k1, k2, 0, 0, k3)) {}

        const cv::Mat intrinsic_parameters;
        const cv::Mat distortion_parameters;
    };

    static std::unique_ptr<Profile_> pimpl_;
};
} // namespace rmcs_auto_aim::util