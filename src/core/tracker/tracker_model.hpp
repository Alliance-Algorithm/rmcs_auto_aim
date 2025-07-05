#pragma once

#include "util/ekf_crtp.hpp"
#include <iostream>

namespace rmcs_auto_aim::tracker {
class TrackerModel : public util::EkfCrtp<13, 4, TrackerModel> {
public:
    TrackerModel() {
        X_k << 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.05, 0.20, 0.20, std::numbers::pi / 2.,
            0.0;

        // 13x13
        P_k.diagonal() << 10.0, 0.1, 0.02, 10.0, 0.05, 0.01, 0.005, 0.005, 0.1, 0.01, 0.01, 0.1, 0.1;
    }
    inline bool get_side_flag_() const { return side_flag_; }

private:
    friend class util::EkfCrtp<13, 4, TrackerModel>;

    //           0,   1,   2, 3,   4,   5, 6,   7,   8,  9, 10,11,12
    // x分别代表[ x, v_x, a_x, y, v_y, a_y, z, z_1, z_2, r1, r2, φ, ω ]
    // z分别代表[ φ, yaw, pitch, distance ]
    // side_flag_为true时更新 r1, z1，为false时更新 r2, z2 , 每当输入的 yaw 有一个跳变 side_flag_
    // 翻转一次

    // WARNING: yaw 一定在  pi/2 - pi 和 -pi - -pi/2 之间，外部要做好处理,尽量在 PNP
    // 解出来之后立马将不合理的丢掉，不在的话舍掉异常数据
    inline ZVec process_z(const ZVec& z_k) {
        ZVec processed_z_k{z_k};
        // 将yaw的角度值从 pi/2 - pi 和 -pi - -pi/2 映射到 0 - pi
        if (z_k(0) >= std::numbers::pi / 2. && z_k(0) <= std::numbers::pi)
            processed_z_k(0) -= std::numbers::pi / 2;
        else
            processed_z_k(0) += 3 * std::numbers::pi / 2;

        if (std::abs(last_yaw_ - processed_z_k(0)) >= switch_angle_difference_) {
            side_flag_ = !side_flag_;
            offset_    = processed_z_k(0) > last_yaw_ ? 1.0 : -1.0;
            std::cerr << "trigger" << processed_z_k(0) * 180. / std::numbers::pi << " "
                      << z_k(0) * 180. / std::numbers::pi << " last:" << last_yaw_ << std::endl;
        } else
            offset_ = 0.0;

        last_yaw_ = processed_z_k(0);
        return processed_z_k;
    };

    static inline XVec normalize_x(const XVec& X_k) { return X_k; };

    inline XVec f(const XVec& X_k, const UVec&, const WVec&, const double& dt) {
        const double x   = X_k(0) + X_k(1) * dt;
        const double y   = X_k(3) + X_k(4) * dt;
        const double v_x = X_k(1) + X_k(2) * dt;
        const double v_y = X_k(4) + X_k(5) * dt;
        const double z   = (X_k(7) + X_k(8)) / 2.;
        const double yaw = X_k(11) + X_k(12) * dt + offset_ * std::numbers::pi / 2.0;

        f_ << x, v_x, X_k(2), y, v_y, X_k(5), z, X_k(7), X_k(8), X_k(9), X_k(10), yaw, X_k(12);

        return f_;
    };

    inline ZVec h(const XVec& x_k_n, const VVec&) {
        double r, z;
        if (side_flag_) {
            r = x_k_n(9);
            z = x_k_n(7);
        } else {
            r = x_k_n(10);
            z = x_k_n(8);
        }

        const double armor_x = x_k_n(0) - r * std::sin(x_k_n(11));
        const double armor_y = x_k_n(3) - r * std::cos(x_k_n(11));
        const double yaw     = std::atan(armor_y / armor_x);
        // Warning: Pitch的观测误差要给大一点，因为这种观测方法本身一定的稳态误差,可以考虑根据
        // x_k_n的 z 进行简单的补偿
        const double pitch    = -std::atan(z / armor_x);
        const double distance = std::sqrt(armor_x * armor_x + armor_y * armor_y + z * z);
        h_ << x_k_n(11), yaw, pitch, distance;
        return h_;
    };

    inline AMat A(const XVec&, const XVec&, const XVec&, const double& dt) {
        // clang-format off
            A_ << 1., dt, 0., 0., 0., 0., 0.,   0.,   0., 0., 0., 0., 0.,
                  0., 1., dt, 0., 0., 0., 0.,   0.,   0., 0., 0., 0., 0.,
                  0., 0., 1., 0., 0., 0., 0.,   0.,   0., 0., 0., 0., 0.,
                  0., 0., 0., 1., dt, 0., 0.,   0.,   0., 0., 0., 0., 0.,
                  0., 0., 0., 0., 1., dt, 0.,   0.,   0., 0., 0., 0., 0.,
                  0., 0., 0., 0., 0., 1., 0.,   0.,   0., 0., 0., 0., 0.,
                  0., 0., 0., 0., 0., 0., 0.,  0.5,  0.5, 0., 0., 0., 0., 
                  0., 0., 0., 0., 0., 0., 0.,   1.,   0., 0., 0., 0., 0., 
                  0., 0., 0., 0., 0., 0., 0.,   0.,   1., 0., 0., 0., 0.,
                  0., 0., 0., 0., 0., 0., 0.,   0.,   0., 0.1, 0., 0., 0.,
                  0., 0., 0., 0., 0., 0., 0.,   0.,   0., 0., 0.1, 0., 0., 
                  0., 0., 0., 0., 0., 0., 0.,   0.,   0., 0., 0., 1., dt, 
                  0., 0., 0., 0., 0., 0., 0.,   0.,   0., 0., 0., 0., 1.;
        // clang-format on
            return A_;
    };

    inline WMat W(const XVec&, const XVec&, const XVec&) const { return W_; };

    inline HMat H(const XVec& x_k_n, const VVec&) {
        if (side_flag_) {
            const double d_x_2_0  = ;
            const double d_x_2_3  = ;
            const double d_x_2_9  = ;
            const double d_x_2_11 = ;

            const double d_x_3_0  = ;
            const double d_x_3_7  = ;
            const double d_x_3_9  = ;
            const double d_x_3_11 = ;

            const double d_x_4_0  = ;
            const double d_x_4_3  = ;
            const double d_x_4_7  = ;
            const double d_x_4_9  = ;
            const double d_x_4_11 = ;

            // clang-format off
            H_ <<      0., 0., 0.,      0., 0., 0.,      0.,      0., 0.,      0., 0.,       1., 0.,
                  d_x_2_0, 0., 0., d_x_2_3, 0., 0.,      0.,      0., 0., d_x_2_9, 0., d_x_2_11, 0., 
                  d_x_3_0, 0., 0.,      0., 0., 0.,      0., d_x_3_7, 0., d_x_3_9, 0., d_x_3_11, 0.,
                  d_x_4_0, 0., 0., d_x_4_3, 0., 0.,      0., d_x_4_7, 0., d_x_4_9, 0., d_x_4_11, 0.;
            // clang-format on
        } else {
            const double d_x_2_6 =
                -1.0 / (1.0 + (x_k_n(6) - x_k_n(8)) * (x_k_n(6) - x_k_n(8)) / x_k_n(10) / x_k_n(10))
                / x_k_n(10);
            const double d_x_2_8  = -d_x_2_6;
            const double d_x_2_10 = d_x_2_8 * (x_k_n(6) - x_k_n(8)) / x_k_n(10);

            const double delta_x = x_k_n(10) * std::sin(x_k_n(11));
            const double delta_y = x_k_n(10) * std::cos(x_k_n(11));
            const double armor_x = x_k_n(0) - delta_x;
            const double armor_y = x_k_n(3) - delta_y;
            const double d_x_3_base =
                1.0 / std::sqrt(armor_x * armor_x + armor_y * armor_y + x_k_n(8) * x_k_n(8));
            const double d_x_3_0 = d_x_3_base * armor_x;
            const double d_x_3_3 = d_x_3_base * armor_y;
            const double d_x_3_8 = d_x_3_base * x_k_n(8);
            const double d_x_3_10 =
                d_x_3_base * (-std::sin(x_k_n(11)) * armor_x - std::cos(x_k_n(11)) * armor_y);
            const double d_x_3_11 = d_x_3_base * (-armor_x * delta_y + armor_y * delta_x);
            // clang-format off
            H_ <<      0., 0., 0.,      0., 0., 0.,      0., 0.,      0., 0.,       0.,       1., 0.,
                       0., 0., 0.,      0., 0., 0., d_x_2_6, 0., d_x_2_8, 0., d_x_2_10,       0., 0., 
                  d_x_3_0, 0., 0., d_x_3_3, 0., 0.,      0., 0., d_x_3_8, 0., d_x_3_10, d_x_3_11, 0.,
                       0., 0., 0.,      0., 0., 0.,      0., 0.,      1., 0.,       0.,       0., 0.;
            // clang-format on
        }
        return H_;
    };

    inline VMat V(const XVec&, const VVec&) const { return V_; };

    // 待测
    inline QMat Q(const double& dt) {
        // Q_.setIdentity();
        // clang-format off
        Q_ << 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
              0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
              0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
              0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
              0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0.,
              0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0.,
              0., 0., 0., 0., 0., 0., 0.01, 0., 0., 0., 0., 0., 0., 
              0., 0., 0., 0., 0., 0., 0., 0.1, 0., 0., 0., 0., 0., 
              0., 0., 0., 0., 0., 0., 0., 0., 0.1, 0., 0., 0., 0.,
              0., 0., 0., 0., 0., 0., 0., 0., 0., 0.1, 0., 0., 0.,
              0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.1, 0., 0., 
              0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.01, 0., 
              0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.01;
        // clang-format on
        return Q_;
    };

    inline RMat R(const ZVec& z) {
        const double yaw_center_difference_ = std::abs(z(0) - std::numbers::pi / 2);
        R_.diagonal() << yaw_center_difference_ * r_yaw_yaw_,
            yaw_center_difference_ * r_yaw_pitch_ + std::abs(X_k(7) - X_k(8)) * r_z_pitch_,
            yaw_center_difference_ * r_yaw_distance_, 0.;
        return R_;
    };

    XVec f_{};
    ZVec h_{};
    AMat A_{};
    WMat W_{WMat::Identity()};
    HMat H_{};
    VMat V_{VMat::Identity()};
    QMat Q_{};
    RMat R_{};

    static constexpr double switch_angle_difference_ = std::numbers::pi * 2. / 5.;
    static constexpr double r_yaw_yaw_               = 0.;
    static constexpr double r_yaw_pitch_             = 0.;
    static constexpr double r_z_pitch_               = 0.;
    static constexpr double r_yaw_distance_          = 0.;

    double last_yaw_{std::numbers::pi / 2.};
    bool side_flag_{true};
    double offset_{0.0};
};
} // namespace rmcs_auto_aim::tracker