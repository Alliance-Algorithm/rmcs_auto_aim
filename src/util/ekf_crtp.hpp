#pragma once

#include "eigen3/Eigen/Eigen"

namespace rmcs_auto_aim::util {

template <int xn, int zn, typename Derived>
class EkfCrtp {
public:
    typedef Eigen::Vector<double, xn> XVec;
    typedef XVec UVec;
    typedef XVec WVec;

    typedef Eigen::Vector<double, zn> ZVec;
    typedef ZVec VVec;

    typedef Eigen::Matrix<double, xn, xn> PMat;
    typedef Eigen::Matrix<double, zn, zn> RMat;
    typedef Eigen::Matrix<double, xn, xn> AMat;
    typedef Eigen::Matrix<double, xn, xn> WMat;
    typedef Eigen::Matrix<double, zn, zn> VMat;
    typedef Eigen::Matrix<double, xn, xn> QMat;
    typedef Eigen::Matrix<double, zn, xn> HMat;
    typedef Eigen::Matrix<double, xn, zn> KMat;

    [[nodiscard]] inline XVec OutPut() const { return X_k; }

    inline void Update(const ZVec& z_k, const UVec& u_k, const double& dt) {
        dt_   = dt;
        P_k_n = P_k_n.Zero();
        S_k   = S_k.Zero();
        y_k   = y_k.Zero();
        K_t   = K_t.Zero();
        tmpK  = tmpK.Zero();

        const auto processed_z = static_cast<Derived*>(this)->process_z(z_k);

        auto x_k_n = static_cast<Derived*>(this)->f(X_k, u_k, w_zero, dt);
        auto A_k   = static_cast<Derived*>(this)->A(X_k, u_k, w_zero, dt);
        auto W_k   = static_cast<Derived*>(this)->W(X_k, u_k, w_zero);
        auto H_k   = static_cast<Derived*>(this)->H(x_k_n, v_zero);
        auto V_k   = static_cast<Derived*>(this)->V(x_k_n, v_zero);

        P_k_n << A_k * P_k * A_k.transpose()
                     + W_k * static_cast<Derived*>(this)->Q(dt) * W_k.transpose();
        y_k << processed_z - static_cast<Derived*>(this)->h(x_k_n, v_zero);
        S_k << H_k * P_k_n * H_k.transpose()
                   + V_k * static_cast<Derived*>(this)->R(z_k) * V_k.transpose();
        K_t << P_k_n * H_k.transpose() * S_k.inverse();
        X_k << x_k_n + K_t * y_k;
        X_k << static_cast<Derived*>(this)->normalize_x(X_k);
        tmpK << Eye_K - K_t * H_k;
        P_k << tmpK * P_k_n;
    }

    // 派生类实现以下函数
    // [[nodiscard]] virtual ZVec process_z(const ZVec& z_k) { return z_k; };
    // [[nodiscard]] virtual XVec normalize_x(const XVec& X_k) { return X_k; };
    // [[nodiscard]] virtual XVec f(const XVec&, const UVec&, const WVec&, const double&) = 0;
    // [[nodiscard]] virtual ZVec h(const XVec&, const VVec&)                             = 0;

    // [[nodiscard]] virtual AMat A(const XVec&, const XVec&, const XVec&, const double&) = 0;

    // [[nodiscard]] virtual WMat W(const XVec&, const XVec&, const XVec&) = 0;

    // [[nodiscard]] virtual HMat H(const XVec&, const VVec&) = 0;

    // [[nodiscard]] virtual VMat V(const XVec&, const VVec&) = 0;

    // [[nodiscard]] virtual QMat Q(const double& t) = 0;
    // [[nodiscard]] virtual RMat R(const ZVec& z)   = 0;

    static inline const WVec w_zero = Eigen::VectorXd::Zero(xn);
    static inline const VVec v_zero = Eigen::VectorXd::Zero(zn);
    static inline const PMat Eye_K  = Eigen::MatrixXd::Identity(xn, xn);

protected:
    EkfCrtp()
        : X_k{XVec::Zero()}
        , P_k{PMat::Identity()} {};
    XVec X_k;
    PMat P_k;

    double dt_;

private:
    PMat P_k_n{};
    RMat S_k{};
    ZVec y_k{};
    KMat K_t{};
    PMat tmpK{};
};
} // namespace rmcs_auto_aim::util