#pragma once
#include <chrono>
#include <random>

#include "core/pnpsolver/armor/armor3d.hpp"
#include "core/tracker/ekf.hpp"

#include "util/utils.hpp"

namespace rmcs_auto_aim {

// Complete Vehicle Solution
struct TrackerUnit {
    TrackerUnit(const ArmorPlate3d& armor, const std::chrono::steady_clock::time_point& timestamp)
        : last_update(timestamp)
        , measurement_pos(armor.position->x(), armor.position->y(), armor.position->z()) {
        double& r        = r_list[0];
        double yaw       = rmcs_auto_aim::util::GetArmorYaw(armor);
        double xc        = armor.position->x() + r * cos(yaw);
        double yc        = armor.position->y() + r * sin(yaw);
        const double& za = armor.position->z();
        // xc  v_xc  yc  v_yc  za  v_za  yaw  v_yaw  r
        ekf.x_ << xc, 0, yc, 0, za, 0, yaw, 0, r;

        std::random_device rd;
        auto gen = std::default_random_engine(rd());
        std::uniform_real_distribution<float> dis(0.0, 1.0);

        color_r = dis(gen);
        color_g = dis(gen);
        color_b = dis(gen);
    }

    void Predict(double dt) {
        ekf.Predict(dt);
        tracked_duration += dt;
        for (bool& updated : armor_newly_updated) {
            updated = false;
        }
    }

    void Update(const ArmorPlate3d& armor, const std::chrono::steady_clock::time_point& timestamp) {

        measurement_pos[0] = armor.position->x();
        measurement_pos[1] = armor.position->y();
        measurement_pos[2] = armor.position->z();

        if (armor.position->norm() <= 0.8) {
            collision = true;
        } else {
            collision = false;
        }

        double &v_za = ekf.x_(5), &model_yaw = ekf.x_(6), &r = ekf.x_(8);
        double yaw = rmcs_auto_aim::util::GetArmorYaw(armor);

        constexpr double legal_range = rmcs_auto_aim::util::Pi / 4;
        constexpr double step        = 2 * rmcs_auto_aim::util::Pi / armor_count;

        double shift = 0;

        size_t i;
        for (i = 0; i < armor_count; ++i) {
            double diff = rmcs_auto_aim::util::GetMinimumAngleDiff(yaw, model_yaw + shift);
            if (-legal_range < diff && diff < legal_range) {
                yaw = model_yaw + shift + diff;
                break;
            } else {
                shift += step;
            }
        }

        if (i < 4) {
            model_yaw += shift;
            r = r_list[i];

            Eigen::Vector4d measurement = {
                armor.position->x(), armor.position->y(), armor.position->z(), yaw};
            ekf.Update(measurement);
            last_update = timestamp;

            v_za = 0;
            if (r > -0.12) {
                r = -0.12;
            } else if (r < -0.4) {
                r = -0.4;
            }
            if (i == 0 || i == 2) {
                r_list[0] = r_list[2] = r;
            } else if (i == 1 || i == 3) {
                r_list[1] = r_list[3] = r;
            }
            model_yaw -= shift;

            tracked_times += 1;
            if (tracked_duration > 0.5) {
                tracking_density = tracked_times / tracked_duration;
            }

            armor_newly_updated[i] = true;
        }
    }

    [[nodiscard]] static std::tuple<Eigen::Vector3d, double>
        GetArmorState(const Eigen::VectorXd& x, size_t index) {
        const double &xc = x(0), &yc = x(2), &za = x(4), &r = x(8);
        constexpr double yaw_step = 2 * rmcs_auto_aim::util::Pi / armor_count;
        double yaw                = x(6) + yaw_step * (double)index;

        double xa = xc - r * cos(yaw);
        double ya = yc - r * sin(yaw);

        return {Eigen::Vector3d(xa, ya, za), yaw};
    }

    [[nodiscard]] std::tuple<Eigen::Vector3d, double> GetArmorState(size_t index) const {
        Eigen::VectorXd x = ekf.x_;
        x(8)              = r_list[index];
        return GetArmorState(x, index);
    }

    EKF ekf;
    static constexpr size_t armor_count = 4;
    std::chrono::steady_clock::time_point last_update;
    double r_list[armor_count]{-0.26, -0.26, -0.26, -0.26};
    bool armor_newly_updated[armor_count]{false, false, false, false};

    double tracked_duration = 0, tracked_times = 0, tracking_density = 0;

    int ros_marker_id;
    float color_r, color_g, color_b;

    bool collision = false;

    Eigen::Vector3d measurement_pos;
};
} // namespace rmcs_auto_aim