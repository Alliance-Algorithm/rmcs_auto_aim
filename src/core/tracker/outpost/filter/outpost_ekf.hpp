

#pragma once

#include <Eigen/Eigen>

#include "util/ekf.hpp"

namespace rmcs_auto_aim ::tracker {

class OutPostKF : public util::EKF<5, 4> {
public:
    OutPostKF()
        : EKF() {
        // clang-format off
        P_k.setIdentity();
        // clang-format on

        P_k *= 0.1;
        x_.setZero();
        z_.setZero();

        a_.setIdentity();

        w_.setIdentity();

        h_.setZero();
        h_(0, 0) = 1;
        h_(1, 1) = 1;
        h_(2, 2) = 1;
        h_(3, 3) = 1;

        v_.setIdentity();
        q_.setIdentity();
        r_.setIdentity();
        r_ *= 0.1;
    };

    [[nodiscard]] ZVec h(const XVec& X_k, const VVec&) override {
        z_ << X_k(0), X_k(1), X_k(2), X_k(3);
        return z_;
    }

protected:
    [[nodiscard]] XVec f(const XVec& X_k, const UVec&, const WVec&, const double& dt) override {
        x_ = X_k;
        x_(3) += dt * x_(4);
        return x_;
    }

    [[nodiscard]] AMat A(const XVec&, const UVec&, const WVec&, const double& dt) override {
        a_(3, 4) = dt;
        return a_;
    }
    [[nodiscard]] ZVec process_z(const ZVec& z_k) override {
        auto err = z_k(3) - X_k(3);
        while (err > std::numbers::pi)
            err -= std::numbers::pi * 2;
        while (err < -std::numbers::pi)
            err += std::numbers::pi * 2;
        ZVec z_new{};
        z_new << z_k;
        z_new(3) = err + X_k(3);
        return z_new;
    }
    [[nodiscard]] WMat W(const XVec&, const UVec&, const WVec&) override { return w_; }

    [[nodiscard]] HMat H(const XVec&, const VVec&) override { return h_; }

    [[nodiscard]] VMat V(const XVec&, const VVec&) override { return v_; }
    [[nodiscard]] QMat Q(const double& dt) override {

        double t = dt, x = sigma2_q_xy_, y = sigma2_q_yaw_;
        double q_x_x = pow(t, 4) / 4 * x;
        double q_y_y = pow(t, 4) / 4 * y, q_y_vy = pow(t, 3) / 2 * y, q_vy_vy = pow(t, 2) * y;
        // clang-format off
        //      xc         ,yc     ,zc      ,theta  ,omega
        q_ <<   q_x_x      ,0      ,0          ,0      ,0,
                0          ,q_x_x  ,0          ,0      ,0,
                0          ,0      ,q_x_x      ,0      ,0,
                0          ,0      ,0          ,q_y_y  ,q_y_vy,
                0          ,0      ,0          ,q_y_vy ,q_vy_vy;
        // clang-format on
        return q_;
    }
    RMat R(const ZVec&) override {
        double x = r_xyz_factor_, y = r_ywq_factor_;

        r_.diagonal() << x, x, x, y;
        return r_;
    };

private:
    static constexpr double sigma2_q_xy_  = 1e1;
    static constexpr double sigma2_q_yaw_ = 1e2;
    static constexpr double r_xyz_factor_ = 1e-1;
    static constexpr double r_ywq_factor_ = 1e3;

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
