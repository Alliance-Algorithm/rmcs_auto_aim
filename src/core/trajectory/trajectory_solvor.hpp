#pragma once

#include <cmath>

#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>

#include <fast_tf/rcl.hpp>
#include <rmcs_description/tf_description.hpp>

namespace rmcs_auto_aim {
class TrajectorySolver {
public:
    [[nodiscard]] static rmcs_description::OdomImu::DirectionVector GetShotVector(
        const rmcs_description::OdomImu::Position& target_pos, const double& speed,
        double& fly_time);

private:
    class StaticImpl;
};
} // namespace rmcs_auto_aim