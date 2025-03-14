

#pragma once

#include <Eigen/Eigen>

#include "util/ekf.hpp"

namespace rmcs_auto_aim ::tracker {

class CarPosKF : public util::EKF<3, 3> {
public:
    CarPosKF()
        : EKF() {
        // clang-format off
        P_k .setIdentity();
        // clang-format on

        P_k *= 0.1;
        x_.setZero();
        z_.setZero();

        a_.setIdentity();

        w_.setIdentity();

        h_.setIdentity();

        v_.setIdentity();
        // q_ = Eigen::MatrixXd::Identity(8, 8) * 0.01;
        r_.setIdentity();
        r_ *= 0.1;
    };

    [[nodiscard]] ZVec h(const XVec& x_k, const VVec&) override { return x_k; }

protected:
    [[nodiscard]] XVec f(const XVec& x_k, const UVec&, const WVec&, const double&) override {

        return x_k;
    }

    [[nodiscard]] AMat A(const XVec&, const UVec&, const WVec&, const double&) override {

        return a_;
    }
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
    [[nodiscard]] WMat W(const XVec&, const UVec&, const WVec&) override { return w_; }

    [[nodiscard]] HMat H(const XVec&, const VVec&) override { return h_; }

    [[nodiscard]] VMat V(const XVec&, const VVec&) override { return v_; }
    [[nodiscard]] QMat Q(const double&) override {

        // clang-format off
        q_ .setIdentity();
        q_*= sigma2_q_xy_;
        q_(2,2) = sigma2_q_yaw_;
        // clang-format on
        return q_;
    }
    RMat R(const ZVec&) override {
        double x = r_xyz_factor_;
        r_.diagonal() << x, x, r_ywq_factor_;
        return r_;
    };

private:
    static constexpr double sigma2_q_xy_  = 5e-3;
    static constexpr double sigma2_q_yaw_ = 5e-3;
    static constexpr double r_xyz_factor_ = 5e-2;
    static constexpr double r_ywq_factor_ = 5e-5;

    static constexpr inline const double conv_y     = 0.01;
    static constexpr inline const double conv_p     = 0.01;
    static constexpr inline const double conv_d     = 0.5;
    static constexpr inline const double conv_theta = 0.1;

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