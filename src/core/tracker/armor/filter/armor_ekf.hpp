

#pragma once

#include <Eigen/Eigen>
#include <cmath>

#include "util/ekf.hpp"

namespace rmcs_auto_aim::tracker {

class ArmorEKF final : public util::EKF<4, 4> {
public:
    ArmorEKF()
        : EKF() {
        // clang-format off
        P_k.setIdentity();
        P_k *= 0.1;
        // clang-format on
        x_.setZero();
        z_.setZero();

        a_.setIdentity();

        w_.setIdentity();

        h_.setZero();

        v_.setIdentity();
        r_.setIdentity();
        q_.setIdentity();

        r_ *= 0.01;
    };

    [[nodiscard]] ZVec process_z(const ZVec& z_k) override {
        auto err = z_k(3) - X_k(3);
        while (err >= std::numbers::pi)
            err -= std::numbers::pi * 2;
        while (err < -std::numbers::pi)
            err += std::numbers::pi * 2;
        ZVec z_new{};
        z_new << z_k;
        z_new(3) = err + X_k(3);
        return z_new;
    }
    [[nodiscard]] XVec normalize_x(const XVec& x_k) override { return x_k; }
    //     XVec x_new = x_k;

    //     for (int i = 0; i < 8; i += 2) {
    //         std::clamp(x_new(i + 1), -10., 10.);
    //     }
    //     return x_new;
    // }

    [[nodiscard]] XVec f(const XVec& X_k, const UVec&, const WVec&, const double&) override {
        return X_k;
    }

    [[nodiscard]] ZVec h(const XVec& X_k, const VVec&) override {
        z_(0) = cos(X_k(0)) * cos(X_k(1)) * X_k(2);
        z_(1) = sin(X_k(0)) * cos(X_k(1)) * X_k(2);
        z_(2) = sin(X_k(1)) * X_k(2);
        z_(3) = X_k(3);

        return z_;
    }

    [[nodiscard]] AMat A(const XVec&, const UVec&, const WVec&, const double&) override {
        return a_;
    }

    [[nodiscard]] WMat W(const XVec&, const UVec&, const WVec&) override { return w_; }

    [[nodiscard]] HMat H(const XVec&, const VVec&) override {
        // clang-format off
        h_ << -cos(X_k(1)) * sin(X_k(0)) * X_k(2)  , -sin(X_k(1)) * cos(X_k(0))* X_k(2) , cos(X_k(0)) * cos(X_k(1))  , 0,    //1
             cos(X_k(1)) * cos(X_k(0)) * X_k(2)    , -sin(X_k(1)) * sin(X_k(0))* X_k(2) , sin(X_k(0)) * cos(X_k(1))  , 0,    //2
             0                                     , cos(X_k(1))* X_k(2)                , sin(X_k(1))                , 0,    //3
             0                                     , 0                                  , 0                          , 1;    //4

        return h_;
        // clang-format on
    }

    [[nodiscard]] VMat V(const XVec&, const VVec&) override { return v_; }
    [[nodiscard]] QMat Q(const double&) override {
        // clang-format on
        q_.diagonal() << 1, 1, 1, 1;
        return q_;
    }
    [[nodiscard]]
    RMat R(const ZVec&) override {
        r_.diagonal() << 1e-5, 1e-5, 1e-2, 1;
        return r_;
    }

protected:
private:
    static constexpr inline const double conv_x     = 0.01;
    static constexpr inline const double conv_y     = 0.01;
    static constexpr inline const double conv_z     = 0.01;
    static constexpr inline const double conv_theta = 0.01;
    static constexpr inline const double conv_r     = 0.1;

    static constexpr double sigma2_q_xyz_ = 200;
    static constexpr double sigma2_q_yaw_ = 100.0;
    static constexpr double sigma2_q_r_   = 800.0;

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