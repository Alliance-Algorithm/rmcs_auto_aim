#pragma once
#include "util/image_viewer/image_viewer.hpp"
#include "util/math.hpp"
#include "util/profile/profile.hpp"
#include <Eigen/Eigen>
#include <Eigen/src/Core/Matrix.h>
#include <fast_tf/impl/cast.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <rmcs_description/tf_description.hpp>
#include <utility>
#include <vector>

namespace rmcs_auto_aim {
class Line : public util::IAutoAimDrawable {
public:
    Line(cv::Point2f top, cv::Point2f button)
        : top_(std::move(top))
        , button_(std::move(button)) {}

    void draw(cv::InputOutputArray image, const cv::Scalar& color) const final {
        cv::line(image, top_, button_, color);
        cv::circle(image, top_, 3, color, -1);
    };
    double angle_distance(const Line& other) const {
        double ratio_err = util::math::clamp_pm_pi(util::math::ratio(top_ - button_))
                         - util::math::clamp_pm_pi(util::math::ratio(other.top_ - other.button_));
        return abs(ratio_err);
    }
    double length_distance(const Line& other) const {
        double ratio_err = cv::norm(top_ - button_) - cv::norm(other.top_ - other.button_);
        return abs(ratio_err);
    }
    Line() = delete;

    const cv::Point2f top_;
    const cv::Point2f button_;
};
class Line3d {
public:
    explicit Line3d(Eigen::Quaterniond rotation, Eigen::Vector3d position, double length)
        : rotation_(std::move(rotation))
        , position(std::move(position))
        , length_(length) {}

    Line to_line_2d(const rmcs_description::Tf& tf) {
        Eigen::Vector3d up_eigen   = Eigen::Vector3d::UnitZ() * length_ / 2.0;
        auto intrinsic_parameters  = util::Profile::get_intrinsic_parameters();
        auto distortion_parameters = util::Profile::get_distortion_parameters();

        auto rotationInCamera = fast_tf::cast<rmcs_description::CameraLink>(
            rmcs_description::OdomImu::DirectionVector(rotation_ * up_eigen), tf);

        auto pos1 = (*rotationInCamera + position) * 1000;
        auto pos2 = (-*rotationInCamera + position) * 1000;
        std::vector<cv::Point3f> object_points{
            {static_cast<float>(-pos1.y()), static_cast<float>(-pos1.z()),
             static_cast<float>(pos1.x())},
            {static_cast<float>(-pos2.y()), static_cast<float>(-pos2.z()),
             static_cast<float>(pos2.x())}
        };
        std::vector<cv::Point2f> imagePoints{};
        cv::Mat t = cv::Mat::zeros(3, 1, CV_32F), r = cv::Mat::zeros(3, 1, CV_32F);
        cv::projectPoints(
            object_points, t, r, intrinsic_parameters, distortion_parameters, imagePoints);
        return {imagePoints[0], imagePoints[1]};
    };

private:
    Eigen::Quaterniond rotation_;
    Eigen::Vector3d position;
    double length_;
};

class Line3dBottom {
public:
    explicit Line3dBottom(Eigen::Quaterniond rotation, Eigen::Vector3d position, double length)
        : rotation_(std::move(rotation))
        , position(std::move(position))
        , length_(length) {}

    Line to_line_2d(const rmcs_description::Tf& tf) {
        Eigen::Vector3d up_eigen   = Eigen::Vector3d::UnitY() * length_ / 2.0;
        auto intrinsic_parameters  = util::Profile::get_intrinsic_parameters();
        auto distortion_parameters = util::Profile::get_distortion_parameters();

        auto rotationInCamera = fast_tf::cast<rmcs_description::CameraLink>(
            rmcs_description::OdomImu::DirectionVector(rotation_ * up_eigen), tf);

        auto pos1 = (*rotationInCamera + position) * 1000;
        auto pos2 = (-*rotationInCamera + position) * 1000;
        std::vector<cv::Point3f> object_points{
            {static_cast<float>(-pos1.y()), static_cast<float>(-pos1.z()),
             static_cast<float>(pos1.x())},
            {static_cast<float>(-pos2.y()), static_cast<float>(-pos2.z()),
             static_cast<float>(pos2.x())}
        };
        std::vector<cv::Point2f> imagePoints{};
        cv::Mat t = cv::Mat::zeros(3, 1, CV_32F), r = cv::Mat::zeros(3, 1, CV_32F);
        cv::projectPoints(
            object_points, t, r, intrinsic_parameters, distortion_parameters, imagePoints);
        return {imagePoints[0], imagePoints[1]};
    };

private:
    Eigen::Quaterniond rotation_;
    Eigen::Vector3d position;
    double length_;
};
} // namespace rmcs_auto_aim