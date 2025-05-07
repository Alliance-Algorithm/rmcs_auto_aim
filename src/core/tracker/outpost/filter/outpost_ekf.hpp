#pragma once

#include <Eigen/Eigen>
#include <cmath>

#include "util/ekf.hpp"

namespace rmcs_auto_aim::tracker {

// x,y,yaw,rotate_velocity
class OutPostKF final : public util::EKF<4, 3> {
public:
    OutPostKF()
        : EKF() {
        P_k <<  0.1, 0.0, 0.0, 0.0,   // x (0.5m误差)
                0.0, 0.1, 0.0, 0.0,   // y 
                0.0, 0.0, 0.1, 1.,   // yaw (0.2rad ~11度) 与 omega的协方差
                0.0, 0.0, 1. , 10.;   // omega (0.3rad/s)
        // clang-format on

        x_.setZero();
        z_.setZero();

        a_.setIdentity();

        w_.setIdentity();

        h_.setZero();

        v_.setIdentity();
    };

    [[nodiscard]] ZVec process_z(const ZVec& z_k) override {
        auto err = z_k(2) - X_k(2);
        while (err > std::numbers::pi)
            err -= std::numbers::pi * 2;
        while (err < -std::numbers::pi)
            err += std::numbers::pi * 2;
        ZVec z_new{};
        z_new << z_k;
        z_new(2) = err + X_k(2);
        return z_new;
    }
    [[nodiscard]] XVec normalize_x(const XVec& x_k) override { return x_k; }

    [[nodiscard]] XVec f(const XVec& X_k, const UVec&, const WVec&, const double& dt) override {
        x_ = X_k;
        x_(2) += X_k(3) * dt;
        return X_k;
    }

    [[nodiscard]] ZVec h(const XVec& X_k, const VVec&) override {
        z_ << X_k(0), X_k(1), X_k(2) + X_k(3) * dt_;
        return z_;
    }

    [[nodiscard]] AMat A(const XVec&, const UVec&, const WVec&, const double& dt) override {
        a_.setIdentity();
        a_(2, 3) = dt;
        return a_;
    }

    [[nodiscard]] WMat W(const XVec&, const UVec&, const WVec&) override { return w_; }

    [[nodiscard]] HMat H(const XVec&, const VVec&) override {
        h_.setZero();
        h_(0, 0) = 1.0;
        h_(1, 1) = 1.0;
        h_(2, 2) = 1.0;
        return h_;
    }

    [[nodiscard]] VMat V(const XVec&, const VVec&) override { return v_; }
    [[nodiscard]] QMat Q(const double& dt) override {
        const double t = dt, y = sigma2_q_yaw_;
        const double q_y_y = pow(t, 4) / 4 * y, q_y_vy = pow(t, 3) / 2 * y, q_vy_vy = pow(t, 2) * y;

        q_ << 1e-3, 0.0, 0.0, 0.0, 0.0, 1e-3, 0.0, 0.0, 0.0, 0.0, q_y_y, q_y_vy, 0.0, 0.0, q_y_vy,
            q_vy_vy;
        return q_;
    }
    [[nodiscard]] RMat R(const ZVec&) override {
        r_.diagonal() << 0.1, // x观测
            0.1,              // y观测
            5;                // yaw观测
        return r_;
    }

protected:
private:
    static constexpr double sigma2_q_yaw_ = 20;

    XVec x_{};
    ZVec z_{};
    AMat a_{};
    WMat w_{};
    HMat h_{};
    VMat v_{};
    QMat q_{};
    RMat r_{};
};

} // namespace rmcs_auto_aim::tracker
