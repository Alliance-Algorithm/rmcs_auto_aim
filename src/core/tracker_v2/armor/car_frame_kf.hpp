

#pragma once

#include "core/tracker_v2/ekf.hpp"
#include <Eigen/Eigen>
#include <Eigen/src/Core/DiagonalMatrix.h>
#include <Eigen/src/Core/Matrix.h>

namespace rmcs_auto_aim ::tracker2 {

class CarFrameKF : public EKF<2, 2> {
public:
    CarFrameKF()
        : EKF() {
        P_k.setIdentity();
        P_k *= 0.01;
        x_.setZero();
        z_.setZero();

        a_.setIdentity();

        w_.setZero();

        h_.setIdentity();

        v_.setZero();
        q_.setIdentity();
        q_ *= 0.01;
        r_.setIdentity();
        r_ *= 0.01;
    };

protected:
    [[nodiscard]] XVec f(const XVec& X_k, const UVec&, const WVec&, const double&) override {
        return X_k;
    }

    [[nodiscard]] ZVec h(const XVec& X_k, const VVec&) override { return X_k; }

    [[nodiscard]] AMat A(const XVec&, const UVec&, const WVec&, const double&) override {
        return a_;
    }
    [[nodiscard]] WMat W(const XVec&, const UVec&, const WVec&) override { return w_; }

    [[nodiscard]] HMat H(const XVec&, const VVec&) override { return h_; }

    [[nodiscard]] VMat V(const XVec&, const VVec&) override { return v_; }
    [[nodiscard]] QMat Q(const double&) override { return q_; }
    [[nodiscard]] RMat R(const double&) override { return r_; }

private:
    static constexpr double sigma2_q_xy_  = 300;
    static constexpr double sigma2_q_yaw_ = 100.0;

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