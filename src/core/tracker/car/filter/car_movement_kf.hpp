

#pragma once

#include <Eigen/Eigen>

#include "util/ekf.hpp"

namespace rmcs_auto_aim ::tracker {

class CarMovementKF : public util::EKF<3, 3> {
public:
    CarMovementKF()
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
        h_.setIdentity();

        v_.setIdentity();
        // q_ = Eigen::MatrixXd::Identity(8, 8) * 0.01;
        r_.setIdentity();
        r_ *= 0.1;
    };

    [[nodiscard]] ZVec h(const XVec& X_k, const VVec&) override {
        z_ << X_k(0), X_k(1), X_k(2);
        return z_;
    }

protected:
    [[nodiscard]] XVec f(const XVec& X_k, const UVec&, const WVec&, const double&) override {
        x_ << X_k;
        return x_;
    }

    [[nodiscard]] AMat A(const XVec&, const UVec&, const WVec&, const double&) override {

        return a_;
    }
    [[nodiscard]] ZVec process_z(const ZVec& z_k) override { return z_k; }
    [[nodiscard]] WMat W(const XVec&, const UVec&, const WVec&) override { return w_; }

    [[nodiscard]] HMat H(const XVec&, const VVec&) override {
        h_.setIdentity();
        // h_ *= dt_;
        return h_;
    }

    [[nodiscard]] VMat V(const XVec&, const VVec&) override { return v_; }
    [[nodiscard]] QMat Q(const double&) override {

        double x = sigma2_q_xy_;
        double y = sigma2_q_yaw_;
        // clang-format off
        //      ,vxc        ,vyc        ,theta  ,omega
        q_ <<   x       ,0          ,0      ,
                0           ,x     ,0      ,
                0           ,0          ,y ;
        // clang-format on
        return q_;
    }
    RMat R(const ZVec&) override {
        r_.diagonal() << r_xyz_factor_, r_xyz_factor_, r_ywq_factor_;
        return r_;
    };

private:
    static constexpr double sigma2_q_xy_  = 1e0;
    static constexpr double sigma2_q_yaw_ = 1e0;
    static constexpr double r_xyz_factor_ = 1e1;
    static constexpr double r_ywq_factor_ = 1e-9;

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