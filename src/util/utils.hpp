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

} // namespace rmcs_auto_aim::util