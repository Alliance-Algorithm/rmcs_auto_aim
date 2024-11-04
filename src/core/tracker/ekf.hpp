/**
 * @file ekf.hpp
 * @author Lorenzo Feng (lorenzo.feng@njust.edu.cn)
 * @brief
 * @version 0.1
 * @date 2024-07-17
 *
 * (C)Copyright: NJUST.Alliance - All rights reserved
 *
 */

#pragma once

#include <Eigen/Eigen>

class EKF {
public:
    EKF() {
        Eigen::DiagonalMatrix<double, 9> p;
        p.setIdentity();
        P_ = p;
    };

    void Predict(double t) {
        Eigen::MatrixXd F = jacobian_f(x_, t), Q = get_Q(t);

        x_ = f(x_, t);
        P_ = F * P_ * F.transpose() + Q;
    }

    [[nodiscard]] Eigen::VectorXd PredictConst(double t) const { return f(x_, t); }

    void Update(const Eigen::VectorXd& z) {
        Eigen::MatrixXd H = jacobian_h(x_), R = get_R(z);
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(9, 9);

        Eigen::MatrixXd K = P_ * H.transpose() * (H * P_ * H.transpose() + R).inverse();
        x_                = x_ + K * (z - h(x_));
        P_                = (I - K * H) * P_;
    }

    Eigen::Matrix<double, 9, 1> x_;
    Eigen::Matrix<double, 9, 9> P_;

private:
    static constexpr double sigma2_q_xyz_ = 20.0;
    static constexpr double sigma2_q_yaw_ = 100.0;
    static constexpr double sigma2_q_r_   = 800.0;
    static constexpr double r_xyz_factor_ = 0.05;
    static constexpr double r_yaw_        = 0.02;

    // f - Process function
    static Eigen::VectorXd f(const Eigen::VectorXd& x, double dt) {
        Eigen::VectorXd x_new = x;
        x_new(0) += x(1) * dt;
        x_new(2) += x(3) * dt;
        x_new(4) += x(5) * dt;
        x_new(6) += x(7) * dt;
        return x_new;
    }

    // J_f - Jacobian of process function
    static Eigen::MatrixXd jacobian_f(const Eigen::VectorXd& x, double dt) {
        (void)x;
        Eigen::MatrixXd f(9, 9);
        f << 1, dt, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, dt, 0, 0, 0, 0, 0, 0,
            0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, dt, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
            0, 0, 0, 1, dt, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;
        return f;
    };

    // h - Observation function
    static Eigen::VectorXd h(const Eigen::VectorXd& x) {
        Eigen::VectorXd z(4);
        double xc = x(0), yc = x(2), yaw = x(6), r = x(8);
        z(0) = xc - r * cos(yaw); // xa
        z(1) = yc - r * sin(yaw); // ya
        z(2) = x(4);              // za
        z(3) = x(6);              // yaw
        return z;
    };

    // jacobian_h - Jacobian of process function
    static Eigen::MatrixXd jacobian_h(const Eigen::VectorXd& x) {
        Eigen::MatrixXd h(4, 9);
        double yaw = x(6), r = x(8);
        // clang-format off
        //    xc   v_xc yc   v_yc za   v_za yaw         v_yaw r
        h << 1, 0, 0, 0, 0, 0, r* sin(yaw), 0, -cos(yaw),
                0, 0, 1, 0, 0, 0, -r * cos(yaw), 0, -sin(yaw),
                0, 0, 0, 0, 1, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0, 1, 0, 0;
        // clang-format on
        return h;
    };

    // Q - process noise covariance matrixEigen::Vector3d
    static Eigen::MatrixXd get_Q(double dt) {
        Eigen::MatrixXd q(9, 9);
        double t = dt, x = sigma2_q_xyz_, y = sigma2_q_yaw_, r = sigma2_q_r_;
        double q_x_x = pow(t, 4) / 4 * x, q_x_vx = pow(t, 3) / 2 * x, q_vx_vx = pow(t, 2) * x;
        double q_y_y = pow(t, 4) / 4 * y, q_y_vy = pow(t, 3) / 2 * x, q_vy_vy = pow(t, 2) * y;
        double q_r = pow(t, 4) / 4 * r;
        // clang-format off
        //    xc      v_xc    yc      v_yc    za      v_za    yaw     v_yaw   r
        q << q_x_x, q_x_vx, 0, 0, 0, 0, 0, 0, 0,
                q_x_vx, q_vx_vx, 0, 0, 0, 0, 0, 0, 0,
                0, 0, q_x_x, q_x_vx, 0, 0, 0, 0, 0,
                0, 0, q_x_vx, q_vx_vx, 0, 0, 0, 0, 0,
                0, 0, 0, 0, q_x_x, q_x_vx, 0, 0, 0,
                0, 0, 0, 0, q_x_vx, q_vx_vx, 0, 0, 0,
                0, 0, 0, 0, 0, 0, q_y_y, q_y_vy, 0,
                0, 0, 0, 0, 0, 0, q_y_vy, q_vy_vy, 0,
                0, 0, 0, 0, 0, 0, 0, 0, q_r;
        // clang-format on
        return q;
    };

    // R - measurement noise covariance matrix
    static Eigen::DiagonalMatrix<double, 4> get_R(const Eigen::VectorXd& z) {
        Eigen::DiagonalMatrix<double, 4> r;
        double x = r_xyz_factor_;
        r.diagonal() << abs(x * z[0]), abs(x * z[1]), abs(x * z[2]), r_yaw_;
        return r;
    };

    // P - error estimate covariance matrix
    // static Eigen::DiagonalMatrix<double, 9> get_P() {
    //}
};