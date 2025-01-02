

#pragma once

#include "core/tracker_v2/ekf.hpp"
#include <Eigen/Eigen>
#include <Eigen/src/Core/DiagonalMatrix.h>
#include <Eigen/src/Core/Matrix.h>
#include <cmath>

namespace rmcs_auto_aim::tracker2 {

class ArmorEKF : public EKF<9, 5> {
public:
    ArmorEKF()
        : EKF() {
        P_k = Eigen::MatrixXd::Identity(9, 9);
        // clang-format off
        P_k <<  .1, 1. , 0., 0. , 0., 0. , 0., 0. , 0., // 1
                1., 10., 0., 0. , 0., 0. , 0., 0. , 0.,    // 2
                0., 0. , .1, 1. , 0., 0. , 0., 0. , 0.,     // 3
                0., 0. , 1., 10., 0., 0. , 0., 0. , 0.,    // 4
                0., 0. , 0., 0. , .1, 1. , 0., 0. , 0.,     // 5
                0., 0. , 0., 0. , 1., 10., 0., 0. , 0.,    // 6
                0., 0. , 0., 0. , 0., 0. , .1, 1. , 0.,     // 7
                0., 0. , 0., 0. , 0., 0. , 1., 10., 0.,    // 8
                0., 0. ,0. , 0. , 0., 0. , 0., 0. , .1;     // 9
        P_k *= 0.1;
        // clang-format on
        x_ = x_.Zero();
        z_ = Eigen::VectorXd::Zero(5);
        a_ = Eigen::MatrixXd::Identity(9, 9);
        w_ = Eigen::MatrixXd::Identity(9, 9);
        h_ = Eigen::MatrixXd::Zero(5, 9);
        v_ = Eigen::MatrixXd::Identity(5, 5);
        q_ = Eigen::MatrixXd::Identity(9, 9) * 0.01;

        r_ = Eigen::MatrixXd::Zero(5, 5);
        r_ << conv_x, 0, 0, 0, 0,   //
            0, conv_y, 0, 0, 0,     //
            0, 0, conv_z, 0, 0,     //
            0, 0, 0, conv_theta, 0, //
            0, 0, 0, 0, conv_r;
    };

    // [[nodiscard]] ZVec process_z(const ZVec& z_k) override {
    //     auto err = z_k(3) - X_k(6);
    //     while (err >= std::numbers::pi)
    //         err -= std::numbers::pi * 2;
    //     while (err < -std::numbers::pi)
    //         err += std::numbers::pi * 2;
    //     ZVec z_new{};
    //     z_new << z_k;
    //     z_new(3) = err + X_k(6);
    //     return z_new;
    // }
    [[nodiscard]] XVec normalize_x(const XVec& x_k) override { return x_k; }
    //     XVec x_new = x_k;

    //     for (int i = 0; i < 8; i += 2) {
    //         std::clamp(x_new(i + 1), -10., 10.);
    //     }
    //     return x_new;
    // }

    [[nodiscard]] XVec f(const XVec& X_k, const UVec&, const WVec& w_k, const double& dt) override {
        for (int i = 0; i < 8; i += 2) {
            x_(i)     = X_k(i) + X_k(i + 1) * dt + w_k(i);
            x_(i + 1) = X_k(i + 1) + w_k(i + 1);
        }
        x_(8) = X_k(8);
        return x_;
    }

    [[nodiscard]] ZVec h(const XVec& X_k, const VVec&) override {
        z_(2) = X_k(4);
        z_(1) = X_k(2);
        z_(0) = X_k(0);
        z_(3) = X_k(6);
        z_(4) = X_k(8);

        return z_;
    }

    [[nodiscard]] AMat A(const XVec&, const UVec&, const WVec&, const double& dt) override {

        for (int i = 0; i < 8; i += 2)
            a_(i, i + 1) = dt;

        return a_;
    }

    [[nodiscard]] WMat W(const XVec&, const UVec&, const WVec&) override { return w_; }

    [[nodiscard]] HMat H(const XVec&, const VVec&) override {
        // clang-format off
        // h_ << -cos(X_k(2)) * sin(X_k(0)) * X_k(4)   , 0, -sin(X_k(2)) * cos(X_k(0))* X_k(4) , 0, cos(X_k(0)) * cos(X_k(2))  , 0, 0, 0, 0,   //1
        //      cos(X_k(2)) * cos(X_k(0)) * X_k(4)     , 0, -sin(X_k(2)) * sin(X_k(0))* X_k(4) , 0, sin(X_k(0)) * cos(X_k(2))  , 0, 0, 0, 0,   //2
        //      0                                      , 0, cos(X_k(2))* X_k(4)                , 0, sin(X_k(2))                , 0, 0, 0, 0,   //3
        //      0                                      , 0, 0                                  , 0, 0                          , 0, 1, 0, 0,   //4
        //      0                                      , 0, 0                                  , 0, 0                          , 0, 0, 0, 1;   //5
        
        h_ <<1 , 0, 0 , 0, 0 , 0, 0, 0, 0,   //1
             0 , 0, 1 , 0, 0 , 0, 0, 0, 0,   //2
             0 , 0, 0 , 0, 1 , 0, 0, 0, 0,   //3
             0 , 0, 0 , 0, 0 , 0, 1, 0, 0,   //4
             0 , 0, 0 , 0, 0 , 0, 0, 0, 1;   //5
        return h_;
        // clang-format on
    }

    [[nodiscard]] VMat V(const XVec&, const VVec&) override { return v_; }
    [[nodiscard]] QMat Q(const double& dt) override {

        double t = dt, x = sigma2_q_xyz_, y = sigma2_q_yaw_, r = sigma2_q_r_;
        double q_x_x = pow(t, 4) / 4 * x, q_x_vx = pow(t, 3) / 2 * x, q_vx_vx = pow(t, 2) * x;
        double q_y_y = pow(t, 4) / 4 * y, q_y_vy = pow(t, 3) / 2 * x, q_vy_vy = pow(t, 2) * y;
        double q_r = pow(t, 4) / 4 * r;
        // clang-format off
        //    xc      v_xc    yc      v_yc    za      v_za    yaw     v_yaw   r
        q_ << q_x_x, q_x_vx, 0, 0, 0, 0, 0, 0, 0,
                q_x_vx, q_vx_vx, 0, 0, 0, 0, 0, 0, 0,
                0, 0, q_x_x, q_x_vx, 0, 0, 0, 0, 0,
                0, 0, q_x_vx, q_vx_vx, 0, 0, 0, 0, 0,
                0, 0, 0, 0, q_x_x , q_x_vx  , 0, 0, 0,
                0, 0, 0, 0, q_x_vx  , q_vx_vx  , 0, 0, 0,
                0, 0, 0, 0, 0, 0, q_y_y, q_y_vy, 0,
                0, 0, 0, 0, 0, 0, q_y_vy, q_vy_vy, 0,
                0, 0, 0, 0, 0, 0, 0, 0, q_r;
        // clang-format on
        return q_;
    }
    [[nodiscard]] RMat R(const double&) override { return r_; }

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