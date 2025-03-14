

#pragma once

#include <Eigen/Eigen>

#include "util/ekf.hpp"

namespace rmcs_auto_aim::tracker {

class CarFrameZKF : public util::EKF<4, 4> {
public:
    CarFrameZKF()
        : EKF() {
        P_k.setIdentity();
        P_k *= 0.01;
        x_.setZero();
        z_.setZero();

        a_.setIdentity();

        w_.setIdentity();

        h_.setIdentity();

        v_.setIdentity();
        q_.setIdentity();
        q_ *= 10;
        r_.setIdentity();
        r_ *= 0.001;
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
    [[nodiscard]]
    RMat R(const ZVec&) override {
        return r_;
    }

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

} // namespace rmcs_auto_aim::tracker