#include <rmcs_description/tf_description.hpp>

#include "trajectory_solvor.hpp"

using namespace rmcs_auto_aim;

class TrajectorySolver::StaticImpl {
public:
    StaticImpl() = default;

    [[nodiscard]] static rmcs_description::OdomImu::DirectionVector GetShotVector(
        const rmcs_description::OdomImu::Position& target_pos, const double& speed,
        double& fly_time) {

        const double& x = target_pos->x();
        const double& y = target_pos->y();
        const double& z = target_pos->z();

        double yaw   = atan2(y, x);
        double pitch = 0;

        double a = speed * speed; // v0 ^ 2
        double b = a * a;         // v0 ^ 4
        double c = x * x + y * y; // xt ^ 2
        double d = c * c;         // xt ^ 4
        double e = G * G;         // g ^ 2

        double xt = sqrt(c);      // target horizontal distance

        double f = b * d * (b - e * c - 2 * G * a * z);
        if (f >= 0) {
            pitch = -atan((b * c - sqrt(f)) / (G * a * c * xt));
        }

        auto result = rmcs_description::OdomImu::DirectionVector{
            cos(pitch) * cos(yaw), cos(pitch) * sin(yaw), -sin(pitch)};

        fly_time = xt / (cos(pitch) * speed);

        return result;
    }

private:
    constexpr const static double G = 9.80665;
};

[[nodiscard]] rmcs_description::OdomImu::DirectionVector TrajectorySolver::GetShotVector(
    const rmcs_description::OdomImu::Position& target_pos, const double& speed, double& fly_time) {
    return StaticImpl::GetShotVector(target_pos, speed, fly_time);
}