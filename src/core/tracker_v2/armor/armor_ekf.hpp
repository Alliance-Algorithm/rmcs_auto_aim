

#pragma once

#include "core/tracker_v2/ekf.hpp"
#include <Eigen/Eigen>
#include <Eigen/src/Core/DiagonalMatrix.h>
#include <Eigen/src/Core/Matrix.h>
#include <cmath>

namespace rmcs_auto_aim ::tracker2 {

class ArmorEKF : public EKF {
public:
    ArmorEKF()
        : EKF(X_k, P_k, w_zero, v_zero, R, Eye_K) {
        x_ = Eigen::VectorXd::Zero(9);
        P_k << 1, 1, 1, 1, 1, 1, 1, 1, 0.1,                                                //
            1, 1, 1, 1, 1, 1, 1, 1, 0.1,                                                   //
            1, 1, 1, 1, 1, 1, 1, 1, 0.1,                                                   //
            1, 1, 1, 1, 1, 1, 1, 1, 0.1,                                                   //
            1, 1, 1, 1, 1, 1, 5, 1, 0.1,                                                   //
            1, 1, 1, 1, 1, 1, 1, 5, 0.1,                                                   //
            .1, .1, .1, .1, .1, .1, .1, .1, .1;                                            //
        P_k *= 0.1;
        z_ = Eigen::VectorXd::Zero(6);
        a_ = Eigen::MatrixXd::Identity(9, 9);
        w_ = Eigen::MatrixXd::Identity(9, 9);
        h_ = Eigen::MatrixXd::Identity(5, 9);
        v_ = Eigen::MatrixXd::Identity(5, 6);
        q_ = Eigen::VectorXd::Identity(9, 9) * 0.01;
    };

    [[nodiscard]] const Eigen::MatrixXd&
        f(const Eigen::VectorXd& X_k, const Eigen::VectorXd&, const Eigen::VectorXd& w_k,
          const double& dt) override {
        for (int i = 0; i < 8; i += 2) {
            x_(i)     = X_k(i) + X_k(i + 1) * dt + w_k(i);
            x_(i + 1) = X_k(i + 1) + w_k(i + 1);
        }
        x_(8) = X_k(8);
        return x_;
    }

    [[nodiscard]] const Eigen::MatrixXd&
        h(const Eigen::VectorXd& X_k, const Eigen::VectorXd& v_k) override {
        z_(2) = sin(X_k(2));
        z_(1) = sin(X_k(0)) * cos(X_k(2));
        z_(0) = cos(X_k(0)) * cos(X_k(2));
        z_(3) = X_k(6);
        z_(4) = X_k(8);
        z_ += v_k;
        return z_;
    }

    [[nodiscard]] const Eigen::MatrixXd&
        A(const Eigen::VectorXd&, const Eigen::VectorXd&, const Eigen::VectorXd&,
          const double& dt) override {

        for (int i = 0; i < 8; i += 2)
            a_(i, i + 1) = dt;
        a_(8) = 1;
        return a_;
    }

    [[nodiscard]] const Eigen::MatrixXd&
        W(const Eigen::VectorXd&, const Eigen::VectorXd&, const Eigen::VectorXd&) override {
        return w_;
    }

    [[nodiscard]] const Eigen::MatrixXd&
        H(const Eigen::VectorXd& X_k, const Eigen::VectorXd&) override {
        h_ << -cos(X_k(2)) * sin(X_k(0)), 0, -sin(X_k(2)) * cos(X_k(0)), 0, 0, 0, 0, 0, 0, //
            cos(X_k(2)) * cos(X_k(0)), 0, -sin(X_k(2)) * sin(X_k(0)), 0, 0, 0, 0, 0, 0,    //
            0, 0, cos(X_k(2)), 0, 0, 0, 0, 0, 0,                                           //
            0, 0, 0, 0, 0, 0, 1, 0, 0,                                                     //
            0, 0, 0, 0, 0, 0, 0, 0, 1;
        return h_;
    }

    [[nodiscard]] const Eigen::MatrixXd&
        V(const Eigen::VectorXd&, const Eigen::VectorXd&) override {
        return v_;
    }
    [[nodiscard]] const Eigen::MatrixXd& Q(const double&) override { return q_; }

protected:
private:
    Eigen::VectorXd X_k = Eigen::VectorXd::Zero(9);
    Eigen::MatrixXd P_k = Eigen::MatrixXd::Identity(9, 9);

    static constexpr inline const double& conv_x     = 0.01;
    static constexpr inline const double& conv_y     = 0.01;
    static constexpr inline const double& conv_z     = 0.5;
    static constexpr inline const double& conv_theta = 0.5;
    static constexpr inline const double& conv_r     = 0.1;

    static inline const Eigen::VectorXd& w_zero = Eigen::VectorXd::Zero(9);
    static inline const Eigen::MatrixXd& v_zero = Eigen::VectorXd::Zero(5);
    static inline const Eigen::MatrixXd& Eye_K  = Eigen::MatrixXd::Identity(9, 9);

    Eigen::MatrixXd x_;
    Eigen::MatrixXd z_;
    Eigen::MatrixXd a_;
    Eigen::MatrixXd w_;
    Eigen::MatrixXd h_;
    Eigen::MatrixXd v_;
    Eigen::MatrixXd q_;
    Eigen::DiagonalMatrix<double, 5, 5> R{
        (Eigen::VectorXd(6) << conv_x, conv_y, conv_z, conv_theta, conv_r).finished()};
};

} // namespace rmcs_auto_aim::tracker2