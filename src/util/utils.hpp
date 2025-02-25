/**
 * @file fps_counter.hpp
 * @author Lorenzo Feng (lorenzo.feng@njust.edu.cn)
 * @brief
 * @version 0.1
 * @date 2024-10-22
 *
 * (C)Copyright: NJUST.Alliance - All rights reserved
 *
 */

#pragma once
#include <chrono>
#include <numbers>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/types.hpp>

#include "core/pnpsolver/armor/armor3d.hpp"

namespace rmcs_auto_aim::util {
class FPSCounter {
public:
    bool Count() {
        if (_count == 0) {
            _count       = 1;
            _timingStart = std::chrono::steady_clock::now();
        } else {
            ++_count;
            if (std::chrono::steady_clock::now() - _timingStart >= std::chrono::seconds(1)) {
                _lastFPS = _count;
                _count   = 0;
                return true;
            }
        }
        return false;
    }

    int GetFPS() const { return _lastFPS; }

private:
    int _count = 0, _lastFPS;
    std::chrono::steady_clock::time_point _timingStart;
};

static inline constexpr double Pi = std::numbers::pi;

static inline double GetArmorYaw(const ArmorPlate3d& armor) {
    Eigen::Vector3d normal = (*armor.rotation) * Eigen::Vector3d{1, 0, 0};
    return atan2(normal.y(), normal.x());
}

static inline double GetMinimumAngleDiff(double a, double b) {
    double diff = std::fmod(a - b, 2 * Pi);
    if (diff < -Pi) {
        diff += 2 * Pi;
    } else if (diff > Pi) {
        diff -= 2 * Pi;
    }
    return diff;
}
static inline bool compute_yaw_pitch_from_point(
    const cv::Point2f& imagePoint, const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs,
    Eigen::Vector3d& direction_vec, double assumedDepth = 1.0) {
    // 1. 去畸变
    std::vector<cv::Point2f> distortedPoints = {imagePoint};
    std::vector<cv::Point2f> undistortedPoints;
    cv::undistortPoints(
        distortedPoints, undistortedPoints, cameraMatrix, distCoeffs, cv::noArray(), cameraMatrix);
    cv::Point2f undistortedPoint = undistortedPoints[0];

    // 2. 归一化相机坐标
    double fx = cameraMatrix.at<double>(0, 0);
    double fy = cameraMatrix.at<double>(1, 1);
    double cx = cameraMatrix.at<double>(0, 2);
    double cy = cameraMatrix.at<double>(1, 2);

    double x_normalized = (undistortedPoint.x - cx) / fx;
    double y_normalized = (undistortedPoint.y - cy) / fy;

    // 3. 构建方向向量
    cv::Point3d directionVec(
        x_normalized * assumedDepth, y_normalized * assumedDepth, assumedDepth);
    // 4. 计算方向向量的模长并归一化
    double norm = cv::norm(directionVec);
    directionVec /= norm;

    direction_vec = Eigen::Vector3d{directionVec.z, -directionVec.x, -directionVec.y};

    direction_vec = direction_vec.normalized();

    return true;
}
} // namespace rmcs_auto_aim::util