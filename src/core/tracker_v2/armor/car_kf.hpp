

#pragma once

#include "core/tracker_v2/ekf.hpp"
#include <Eigen/Eigen>
#include <Eigen/src/Core/DiagonalMatrix.h>
#include <Eigen/src/Core/Matrix.h>

namespace rmcs_auto_aim ::tracker2 {

class CarKF : public EKF {
public:
    CarKF()
        : EKF(X_k, P_k, w_zero, v_zero, R, Eye_K) {
        P_k << 10, 10, 10, 10, 1, 1, 0, 0, //
            10, 10, 10, 10, 1, 1, 0, 0,    //
            10, 10, 10, 10, 1, 1, 0, 0,    //
            10, 10, 10, 10, 1, 1, 0, 0,    //
            1, 1, 1, 1, 0.1, 0, 0, 0,      //
            1, 1, 1, 1, 0, 0.1, 0, 0,      //
            0, 0, 0, 0, 0, 0, 1e-4, 0,     //
            0, 0, 0, 0, 0, 0, 0, 1e-4;

        x_ = Eigen::VectorXd::Zero(8);
        z_ = Eigen::VectorXd::Zero(4);

        a_ = Eigen::MatrixXd::Identity(8, 8);

        w_ = Eigen::MatrixXd::Identity(8, 8);

        h_       = Eigen::MatrixXd::Identity(4, 8);
        h_(0, 0) = 1;
        h_(1, 2) = 1;
        h_(2, 4) = 1;
        h_(3, 6) = 1;

        v_ = Eigen::MatrixXd::Identity(4, 4);

        q_ = Eigen::VectorXd::Identity(8, 8) * 0.01;
    };

protected:
    [[nodiscard]] const Eigen::MatrixXd&
        f(const Eigen::VectorXd& X_k, const Eigen::VectorXd&, const Eigen::VectorXd& w_k,
          const double& dt) override {
        for (int i = 0; i < 8; i += 2) {
            x_(i)     = X_k(i) + X_k(i + 1) * dt + w_k(i);
            x_(i + 1) = X_k(i + 1) + w_k(i + 1);
        }
        return x_;
    }

    [[nodiscard]] const Eigen::MatrixXd&
        h(const Eigen::VectorXd& X_k, const Eigen::VectorXd& v_k) override {
        z_ << X_k(0) + v_k(0), X_k(2) + v_k(1), X_k(4) + v_k(2), X_k(6) + v_k(3);
        return z_;
    }

    [[nodiscard]] const Eigen::MatrixXd&
        A(const Eigen::VectorXd&, const Eigen::VectorXd&, const Eigen::VectorXd&,
          const double& dt) override {

        for (int i = 0; i < 8; i += 2)
            a_(i, i + 1) = dt;
        return a_;
    }

    [[nodiscard]] const Eigen::MatrixXd&
        W(const Eigen::VectorXd&, const Eigen::VectorXd&, const Eigen::VectorXd&) override {
        return w_;
    }

    [[nodiscard]] const Eigen::MatrixXd&
        H(const Eigen::VectorXd&, const Eigen::VectorXd&) override {
        return h_;
    }

    [[nodiscard]] const Eigen::MatrixXd&
        V(const Eigen::VectorXd&, const Eigen::VectorXd&) override {
        return v_;
    }
    [[nodiscard]] const Eigen::MatrixXd& Q(const double&) override { return q_; }

private:
    Eigen::VectorXd X_k = Eigen::VectorXd::Zero(8);
    Eigen::MatrixXd P_k = Eigen::MatrixXd::Identity(8, 8);

    static constexpr inline const double& conv_y     = 0.01;
    static constexpr inline const double& conv_p     = 0.01;
    static constexpr inline const double& conv_d     = 0.5;
    static constexpr inline const double& conv_theta = 0.1;

    static inline const Eigen::VectorXd& w_zero = Eigen::VectorXd::Zero(8);
    static inline const Eigen::MatrixXd& v_zero = Eigen::VectorXd::Zero(8);
    static inline const Eigen::MatrixXd& R      = Eigen::VectorXd::Identity(4, 4) * 0.1;
    static inline const Eigen::MatrixXd& Eye_K  = Eigen::MatrixXd::Identity(8, 8);

    Eigen::MatrixXd x_;
    Eigen::MatrixXd z_;
    Eigen::MatrixXd a_;
    Eigen::MatrixXd w_;
    Eigen::MatrixXd h_;
    Eigen::MatrixXd v_;
    Eigen::MatrixXd q_;
};

} // namespace rmcs_auto_aim::tracker2