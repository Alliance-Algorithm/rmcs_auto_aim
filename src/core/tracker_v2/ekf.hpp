#pragma once

#include <Eigen/Eigen>
#include <Eigen/src/Core/Matrix.h>

#define unuse(x) (void)(x);

namespace rmcs_auto_aim::tracker2 {
class EKF {
public:
    [[nodiscard]] inline Eigen::MatrixXd OutPut() { return X_k; }

    void Update(const Eigen::VectorXd& z_k, const Eigen::VectorXd& u_k, const double& dt) {
        auto& x_k_n = f(X_k, u_k, w_zero, dt);
        auto& A_k   = A(X_k, u_k, w_zero, dt);
        auto& W_k   = W(X_k, u_k, w_zero);
        auto& H_k   = H(x_k_n, v_zero);
        auto& V_k   = V(x_k_n, v_zero);
        auto& P_k_n = A_k * P_k * A_k.transpose() + W_k * Q(dt) * W_k.transpose();
        auto& y_k   = z_k - h(x_k_n, v_zero);
        auto& S_k   = H_k * P_k_n * H_k.transpose() + V_k * R * V_k.transpose();
        auto& K_t   = P_k_n * H_k.transpose() * S_k.inverse();
        X_k         = x_k_n + K_t * y_k;
        P_k         = (Eye_K - K_t * H_k) * P_k_n;
    }

    [[nodiscard]] virtual const Eigen::MatrixXd&
        f(const Eigen::VectorXd& X_k, const Eigen::VectorXd& u_k, const Eigen::VectorXd& w_k,
          const double& dt) = 0;
    [[nodiscard]] virtual const Eigen::MatrixXd&
        h(const Eigen::VectorXd& X_k, const Eigen::VectorXd& v_k) = 0;

    [[nodiscard]] virtual const Eigen::MatrixXd&
        A(const Eigen::VectorXd& X_k, const Eigen::VectorXd& u_k, const Eigen::VectorXd& w_k,
          const double& dt) = 0;

    [[nodiscard]] virtual const Eigen::MatrixXd&
        W(const Eigen::VectorXd& X_k, const Eigen::VectorXd& u_k, const Eigen::VectorXd& w_k) = 0;

    [[nodiscard]] virtual const Eigen::MatrixXd&
        H(const Eigen::VectorXd& X_k, const Eigen::VectorXd& v_k) = 0;

    [[nodiscard]] virtual const Eigen::MatrixXd&
        V(const Eigen::VectorXd& X_k, const Eigen::VectorXd& v_k) = 0;

    [[nodiscard]] virtual const Eigen::MatrixXd& Q(const double& t) = 0;

protected:
    EKF(Eigen::VectorXd& X_k, Eigen::MatrixXd& P_k, const Eigen::VectorXd& w_zero,
        const Eigen::VectorXd& v_zero, const Eigen::MatrixXd& R, const Eigen::MatrixXd& Eye_K)
        : X_k(X_k)
        , P_k(P_k)
        , w_zero(w_zero)
        , v_zero(v_zero)
        , R(R)
        , Eye_K(Eye_K) {}

private:
    Eigen::VectorXd& X_k;
    Eigen::MatrixXd& P_k;

    const Eigen::VectorXd& w_zero;
    const Eigen::VectorXd& v_zero;
    const Eigen::MatrixXd& R;
    const Eigen::MatrixXd& Eye_K;
};
} // namespace rmcs_auto_aim::tracker2