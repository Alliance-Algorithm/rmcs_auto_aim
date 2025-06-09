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

    [[nodiscard]] static rmcs_description::OdomImu::DirectionVector GetShotVectorConsiderAirResistance(
        const rmcs_description::OdomImu::Position& target_pos, const double& speed,
        double& fly_time, const double& bullet_diameter) {
        
        constexpr double air_density = 1.225;   // 空气密度 kg/m³
        constexpr double drag_coeff = 0.47;     // 球体阻力系数
        constexpr double mass = 0.003;          // 弹丸质量 kg
        const double area = std::numbers::pi * std::pow(bullet_diameter/2, 2); // 截面积 m²
        
        const double k = 0.5 * air_density * drag_coeff * area / mass;

        const double& x = target_pos->x();
        const double& y = target_pos->y();
        const double& z = target_pos->z();
        const double horizontal_dist = sqrt(x*x + y*y);
        const double yaw = atan2(y, x);

        // 二分法迭代求解俯仰角
        double low = -std::numbers::pi / 2, high =  std::numbers::pi / 2;
        double pitch = 0.0;
        const double tolerance = 1e-3;
        
        for (int i = 0; i < 20; ++i) {
            pitch = (low + high) / 2;
            
            // 初始化运动参数
            double vx = speed * cos(pitch) * cos(yaw);
            double vy = speed * cos(pitch) * sin(yaw);
            double vz = speed * sin(pitch);
            double px = 0.0, py = 0.0, pz = 0.0;
            fly_time = 0.0;
            
            // 数值积分计算弹道
            const double dt = 0.001;
            while (true) {
                // 计算空气阻力加速度
                const double speed_sq = vx*vx + vy*vy + vz*vz;
                const double drag     = k * speed_sq;

                // 分解加速度分量
                const double ax = -drag * vx;
                const double ay = -drag * vy;
                const double az = -drag * vz - G;

                // 更新速度和位置
                vx += ax * dt;
                vy += ay * dt;
                vz += az * dt;
                
                px += vx * dt;
                py += vy * dt;
                pz += vz * dt;
                fly_time += dt;
                
                // 终止条件检查
                const double current_h = std::sqrt(px * px + py * py);
                if (current_h > horizontal_dist || pz < -10) {
                    break;
                }
            }
            
            // 计算高度偏差
            double height_error = pz - z;
            
            // 调整俯仰角
            if (height_error > tolerance) {
                low = pitch;
            } else if (height_error < -tolerance) {
                high = pitch;
            } else {
                break;
            }
        }

        return rmcs_description::OdomImu::DirectionVector{
            std::cos(pitch) * std::cos(yaw), std::cos(pitch) * std::sin(yaw), -std::sin(pitch)};
    }

private:
    constexpr const static double G = 9.80665;
};

[[nodiscard]] rmcs_description::OdomImu::DirectionVector TrajectorySolver::GetShotVector(
    const rmcs_description::OdomImu::Position& target_pos, const double& speed, double& fly_time) {
    return StaticImpl::GetShotVector(target_pos, speed, fly_time);
}
