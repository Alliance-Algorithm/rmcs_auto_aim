#pragma once
#include "track_unit.hpp"

namespace rmcs_auto_aim {

class ArmorTarget {
public:
    explicit ArmorTarget(const TrackerUnit& tracker)
        : tracker_(tracker) {}

    ~ArmorTarget() {}

    [[nodiscard]] rmcs_description::OdomImu::Position Predict(double sec) const {
        if (tracker_.collision) {
            return rmcs_description::OdomImu::Position{tracker_.measurement_pos};
        }
        // xc0  v_xc1  yc2  v_yc3  za4  v_za5  yaw6  v_yaw7  r8
        Eigen::VectorXd x = tracker_.ekf.PredictConst(sec);
        const double &xc = x(0), &yc = x(2), &za = x(4), &v_yaw = x(7);
        const double &vc = x(1), &vy = x(3);
        const double sigma_v = sqrt(pow(vc, 2) + pow(vy, 2));
        if (sigma_v > 2.0) {
            return rmcs_description::OdomImu::Position{tracker_.measurement_pos};
        }
        double& model_yaw = x(6);
        double camera_yaw = std::atan2(-yc, -xc);
        if (fabs(v_yaw) < 12.0) {
            double shift                 = 0;
            constexpr double legal_range = rmcs_auto_aim::util::Pi / 6;
            constexpr double step        = 2 * rmcs_auto_aim::util::Pi / TrackerUnit::armor_count;
            size_t i;
            for (i = 0; i < TrackerUnit::armor_count; ++i) {
                double diff =
                    rmcs_auto_aim::util::GetMinimumAngleDiff(camera_yaw, model_yaw + shift);
                if (-legal_range < diff && diff < legal_range)
                    break;
                else
                    shift += step;
            }
            if (i < TrackerUnit::armor_count) {
                double r    = tracker_.r_list[i];
                auto offset = 0.0; // To solve gimbal delay
                double yaw  = model_yaw + shift + offset;
                // TODO: Dynamic z
                auto pos = Eigen::Vector3d{xc - r * cos(yaw), yc - r * sin(yaw), za};
                return rmcs_description::OdomImu::Position(pos);
            }
        }
        // return GimbalGyro::Position(0, 0, 0);
        model_yaw = camera_yaw;
        double r  = (tracker_.r_list[0] + tracker_.r_list[1]) / 2;
        // auto [pos, yaw] = TrackerUnit::GetArmorState(x, 0);
        auto pos = Eigen::Vector3d{xc - r * cos(camera_yaw), yc - r * sin(camera_yaw), za};

        return rmcs_description::OdomImu::Position(pos);
    }

private:
    const TrackerUnit& tracker_;
};
} // namespace rmcs_auto_aim