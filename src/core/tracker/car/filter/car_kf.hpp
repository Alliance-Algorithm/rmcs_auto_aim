

#pragma once

#include <Eigen/Eigen>

#include "util/ekf.hpp"

namespace rmcs_auto_aim ::tracker {

class CarKF : public util::EKF<6, 3> {
public:
    CarKF()
        : EKF() {
        // clang-format off
        P_k <<  .1, 1. , 0., 0. , 0., 0. ,  // 1
                1., 10., 0., 0. , 0., 0. ,  // 2
                0., 0. , .1, 1. , 0., 0. ,  // 3
                0., 0. , 1., 10., 0., 0. ,  // 4
                0., 0. , 0., 0. , .1, 1. ,  // 5
                0., 0. , 0., 0. , 1., 10.;  // 6
        // clang-format on

        P_k *= 0.1;
        x_.setZero();
        z_.setZero();

        a_.setIdentity();

        w_.setIdentity();

        h_.setZero();
        h_(0, 0) = 1;
        h_(1, 2) = 1;
        h_(2, 4) = 1;

        v_.setIdentity();
        // q_ = Eigen::MatrixXd::Identity(8, 8) * 0.01;
        r_.setIdentity();
        r_ *= 0.1;
    };

    [[nodiscard]] ZVec h(const XVec& X_k, const VVec&) override {
        z_ << X_k(0) + X_k(1) * dt_, X_k(2) + X_k(3) * dt_, X_k(4) + X_k(5) * dt_;
        return z_;
    }

protected:
    [[nodiscard]] XVec f(const XVec& X_k, const UVec&, const WVec&, const double& dt) override {
        for (int i = 0; i < 6; i += 2) {
            x_(i)     = X_k(i) + X_k(i + 1) * dt;
            x_(i + 1) = X_k(i + 1);
        }
        return x_;
    }

    [[nodiscard]] AMat A(const XVec&, const UVec&, const WVec&, const double& dt) override {

        for (int i = 0; i < 6; i += 2)
            a_(i, i + 1) = dt;
        return a_;
    }
    [[nodiscard]] ZVec process_z(const ZVec& z_k) override {
        auto err = z_k(2) - X_k(4);
        while (err > std::numbers::pi)
            err -= std::numbers::pi * 2;
        while (err < -std::numbers::pi)
            err += std::numbers::pi * 2;
        ZVec z_new{};
        z_new << z_k;
        z_new(2) = err + X_k(4);
        return z_new;
    }
    [[nodiscard]] WMat W(const XVec&, const UVec&, const WVec&) override { return w_; }

    [[nodiscard]] HMat H(const XVec&, const VVec&) override {
        for (int i = 0; i < 3; i += 1)
            h_(i, i * 2 + 1) = dt_;
        return h_;
    }

    [[nodiscard]] VMat V(const XVec&, const VVec&) override { return v_; }
    [[nodiscard]] QMat Q(const double& dt) override {

        double t = dt, x = sigma2_q_xy_, y = sigma2_q_yaw_;
        double q_x_x = pow(t, 4) / 4 * x, q_x_vx = pow(t, 3) / 2 * x, q_vx_vx = pow(t, 2) * x;
        double q_y_y = pow(t, 4) / 4 * y, q_y_vy = pow(t, 3) / 2 * y, q_vy_vy = pow(t, 2) * y;
        // clang-format off
        //      xc      ,vxc        ,yc     ,vyc        ,theta  ,omega
        q_ <<   q_x_x   ,q_x_vx     ,0      ,0          ,0      ,0      ,
                q_x_vx  ,q_vx_vx    ,0      ,0          ,0      ,0      ,
                0       ,0          ,q_x_x  ,q_x_vx     ,0      ,0      ,
                0       ,0          ,q_x_vx ,q_vx_vx    ,0      ,0      ,
                0       ,0          ,0      ,0          ,q_y_y  ,q_y_vy ,
                0       ,0          ,0      ,0          ,q_y_vy ,q_vy_vy;
        // clang-format on
        return q_;
    }
    RMat R(const ZVec&) override {
        double x = r_xyz_factor_;
        r_.diagonal() << x, x, r_ywq_factor_;
        return r_;
    };

private:
    static constexpr double sigma2_q_xy_            = 20;
    static constexpr double sigma2_q_yaw_           = 20;
    static constexpr double r_xyz_factor_           = 1e-2;
    static constexpr double r_ywq_factor_           = 10;
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