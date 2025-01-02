

#pragma once

#include "core/tracker_v2/ekf.hpp"
#include <Eigen/Eigen>
#include <Eigen/src/Core/DiagonalMatrix.h>
#include <Eigen/src/Core/Matrix.h>

namespace rmcs_auto_aim ::tracker2 {

class CarKF : public EKF<8, 4> {
public:
    CarKF()
        : EKF() {
        // clang-format off
        P_k <<  .1, 1. , 0., 0. , 0., 0. , 0., 0., // 1
                1., 10., 0., 0. , 0., 0. , 0., 0.,   // 2
                0., 0. , .1, 1. , 0., 0. , 0., 0.,    // 3
                0., 0. , 1., 10., 0., 0. , 0., 0.,   // 4
                0., 0. , 0., 0. , .1, 1. , 0., 0.,   // 5
                0., 0. , 0., 0. , 1., 10., 0., 0.,   // 6
                0., 0. , 0., 0. , 0., 0. , .1, 1.,     // 7
                0., 0. , 0., 0. , 0., 0. , 1., 10.;     // 8
        // clang format on
        P_k *= 0.1;
        x_ = Eigen::VectorXd::Zero(8);
        z_ = Eigen::VectorXd::Zero(4);

        a_ = Eigen::MatrixXd::Identity(8, 8);

        w_ = Eigen::MatrixXd::Identity(8, 8);

        h_       = Eigen::MatrixXd::Zero(4, 8);
        h_(0, 0) = 1;
        h_(1, 2) = 1;
        h_(2, 4) = 1;
        h_(3, 6) = 1;

        v_ = Eigen::MatrixXd::Identity(4, 4);

        // q_ = Eigen::MatrixXd::Identity(8, 8) * 0.01;
        r_ = Eigen::MatrixXd::Identity(4, 4) * 0.01;
    };

protected:
    [[nodiscard]] XVec f(const XVec& X_k, const UVec&, const WVec& w_k, const double& dt) override {
        for (int i = 0; i < 8; i += 2) {
            x_(i)     = X_k(i) + X_k(i + 1) * dt + w_k(i);
            x_(i + 1) = X_k(i + 1) + w_k(i + 1);
        }
        return x_;
    }

    [[nodiscard]] ZVec h(const XVec& X_k, const VVec&) override {
        z_ << X_k(0), X_k(2), X_k(4), X_k(6);
        return z_;
    }

    [[nodiscard]] AMat A(const XVec&, const UVec&, const WVec&, const double& dt) override {

        for (int i = 0; i < 8; i += 2)
            a_(i, i + 1) = dt;
        return a_;
    }
    [[nodiscard]] ZVec process_z(const ZVec& z_k) override {
        auto err = z_k(3) - X_k(6);
        while (err > std::numbers::pi)
            err -= std::numbers::pi * 2;
        while (err < -std::numbers::pi)
            err += std::numbers::pi * 2;
        ZVec z_new {};
        z_new << z_k;
        z_new(3)   = err + X_k(6);
        return z_new;
    }
    [[nodiscard]] WMat W(const XVec&, const UVec&, const WVec&) override { return w_; }

    [[nodiscard]] HMat H(const XVec&, const VVec&) override { return h_; }

    [[nodiscard]] VMat V(const XVec&, const VVec&) override { return v_; }
    [[nodiscard]] QMat Q(const double& dt) override {

        double t = dt, x = sigma2_q_xyz_, y = sigma2_q_yaw_;
        double q_x_x = pow(t, 4) / 4 * x, q_x_vx = pow(t, 3) / 2 * x, q_vx_vx = pow(t, 2) * x;
        double q_y_y = pow(t, 4) / 4 * y, q_y_vy = pow(t, 3) / 2 * x, q_vy_vy = pow(t, 2) * y;
        // clang-format off
        //    xc      v_xc    yc      v_yc    za      v_za    yaw     v_yaw   r
        q_ << q_x_x, q_x_vx, 0, 0,  0, 0, 0, 0,
                q_x_vx, q_vx_vx, 0, 0, 0, 0, 0, 0,
                0, 0, q_x_x, q_x_vx, 0, 0, 0, 0,
                0, 0, q_x_vx, q_vx_vx, 0, 0, 0, 0,
                0, 0, 0, 0, q_x_x , q_x_vx  , 0, 0,
                0, 0, 0, 0, q_x_vx  , q_vx_vx  ,  0, 0,
                0, 0, 0, 0, 0, 0, q_y_y, q_y_vy, 
                0, 0, 0, 0, 0, 0, q_y_vy, q_vy_vy;
        // clang-format on
        return q_;
    }
    [[nodiscard]] RMat R(const double&) override { return r_; }

private:
    static constexpr double sigma2_q_xyz_ = 300;
    static constexpr double sigma2_q_yaw_ = 100.0;
    static constexpr double sigma2_q_r_   = 800.0;

    static constexpr inline const double conv_y     = 0.01;
    static constexpr inline const double conv_p     = 0.01;
    static constexpr inline const double conv_d     = 0.5;
    static constexpr inline const double conv_theta = 0.1;

    XVec x_;
    ZVec z_;
    AMat a_;
    WMat w_;
    HMat h_;
    VMat v_;
    QMat q_;
    RMat r_;
};

} // namespace rmcs_auto_aim::tracker2