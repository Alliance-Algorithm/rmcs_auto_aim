// #pragma once

// #include "util/ekf_crtp.hpp"

// namespace rmcs_auto_aim::tracker {
// class TrackerModel : public util::EkfCrtp<11, 5, TrackerModel> {
// public:
//     TrackerModel() {
//         X_k << 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.05, 0.2, 0.2, std::numbers::pi / 2., 0.0;

//         P_k.diagonal() << 1.0, 0.01, 1.0, 0.01, 0.005, 0.005, 0.1, 0.05, 0.05, 0.1, 0.1;
//     }
//     inline bool get_side_flag_() const { return side_flag_; }

// private:
//     friend class util::EkfCrtp<11, 5, TrackerModel>;

//     //           0,   1, 2,   3, 4,   5,   6,  7,  8, 9,10
//     // x分别代表[ x, v_x, y, v_y, z, z_1, z_2, r1, r2, θ, ω ]
//     // z分别代表[ θ, yaw, pitch, distance, z ]
//     // side_flag_为true时更新 r1, z1，为false时更新 r2, z2 , 每当输入的 yaw 有一个跳变 side_flag_
//     // 翻转一次

//     std::size_t flag{0};

//     // WARNING: yaw 一定在  pi/2 - pi 和 -pi - -pi/2 之间，外部要做好处理,尽量在 PNP
//     // 解出来之后立马将不合理的丢掉，不在的话舍掉异常数据
//     inline ZVec process_z(const ZVec& z_k) {
//         ZVec processed_z_k{z_k};
//         // 将yaw的角度值从 pi/2 - pi 和 -pi - -pi/2 映射到 0 - pi
//         if (z_k(0) >= std::numbers::pi / 2. && z_k(0) <= std::numbers::pi)
//             processed_z_k(0) -= std::numbers::pi / 2.;
//         else
//             processed_z_k(0) += 3 * std::numbers::pi / 2.;

//         if (std::abs(last_theta_ - processed_z_k(0)) >= switch_angle_difference_) {
//             side_flag_ = !side_flag_;
//             if (processed_z_k(0) > last_theta_)
//                 X_k(9) += std::numbers::pi / 2.;
//             else
//                 X_k(9) -= std::numbers::pi / 2.;
//         }
//         ++flag;
//         if (flag <= 10) {
//             std::cerr << "processed_z_k" << flag << ": " << processed_z_k(0) << std::endl;
//             std::cerr << "theta" << flag << ": " << X_k(9) << std::endl;
//             std::cerr << "omega" << flag << ": " << X_k(10) << std::endl;
//         }

//         last_theta_ = processed_z_k(0);
//         return processed_z_k;
//     };

//     static inline XVec normalize_x(const XVec& X_k) { return X_k; };

//     inline XVec f(const XVec& X_k, const UVec&, const WVec&, const double& dt) {
//         const double x     = X_k(0) + X_k(1) * dt;
//         const double y     = X_k(2) + X_k(3) * dt;
//         const double z     = (X_k(5) + X_k(6)) / 2.;
//         const double theta = X_k(9) + X_k(10) * dt;

//         f_ << x, X_k(1), y, X_k(3), z, X_k(5), X_k(6), X_k(7), X_k(8), theta, X_k(10);

//         return f_;
//     };

//     inline ZVec h(const XVec& x_k_n, const VVec&) {
//         double r, z;
//         if (side_flag_) {
//             r = x_k_n(7);
//             z = x_k_n(5);
//         } else {
//             r = x_k_n(8);
//             z = x_k_n(6);
//         }

//         const double theta    = x_k_n(9);
//         const double armor_x  = x_k_n(0) - r * std::sin(theta);
//         const double armor_y  = x_k_n(2) + r * std::cos(theta);
//         const double yaw      = std::atan(armor_y / armor_x);
//         const double pitch    = -std::atan(z / armor_x);
//         const double distance = std::sqrt(armor_x * armor_x + armor_y * armor_y + z * z);
//         h_ << x_k_n(9), yaw, pitch, distance, z;
//         return h_;
//     };

//     inline AMat A(const XVec&, const XVec&, const XVec&, const double& dt) {
//         // clang-format off
//             A_ << 1., dt, 0., 0., 0.,  0.,   0., 0., 0., 0., 0.,
//                   0., 1., 0., 0., 0.,  0.,   0., 0., 0., 0., 0.,
//                   0., 0., 1., dt, 0.,  0.,   0., 0., 0., 0., 0.,
//                   0., 0., 0., 1., 0.,  0.,   0., 0., 0., 0., 0.,
//                   0., 0., 0., 0., 0., 0.5,  0.5, 0., 0., 0., 0., 
//                   0., 0., 0., 0., 0.,  1.,   0., 0., 0., 0., 0., 
//                   0., 0., 0., 0., 0.,  0.,   1., 0., 0., 0., 0.,
//                   0., 0., 0., 0., 0.,  0.,   0., 1., 0., 0., 0.,
//                   0., 0., 0., 0., 0.,  0.,   0., 0., 1., 0., 0., 
//                   0., 0., 0., 0., 0.,  0.,   0., 0., 0., 1., dt, 
//                   0., 0., 0., 0., 0.,  0.,   0., 0., 0., 0., 1.;
//         // clang-format on
//             return A_;
//     };

//     inline WMat W(const XVec&, const XVec&, const XVec&) const { return W_; };

//     inline HMat H(const XVec& x_k_n, const VVec&) {
//         const double theta = x_k_n(9);
//         if (side_flag_) {
//             const double r         = x_k_n(7);
//             const double z         = x_k_n(5);
//             const double armor_x   = x_k_n(0) - r * std::sin(theta);
//             const double armor_y   = x_k_n(2) + r * std::cos(theta);
//             const double armor_x_2 = armor_x * armor_x;
//             const double armor_y_2 = armor_y * armor_y;

//             const double d_x_2_base = 1.0 / (1.0 + armor_y_2 / armor_x_2);
//             const double d_x_2_0    = -d_x_2_base * armor_y / armor_x_2;
//             const double d_x_2_2    = d_x_2_base / armor_x;
//             const double d_x_2_7 =
//                 d_x_2_base * (armor_y / armor_x_2 * std::sin(theta) + std::cos(theta) / armor_x);
//             const double d_x_2_9 =
//                 d_x_2_base * r
//                 * (std::cos(theta) * armor_y / armor_x_2 - std::sin(theta) / armor_x);

//             const double d_x_3_base = -1.0 / armor_x_2 / (1.0 + z * z / armor_x_2);
//             const double d_x_3_0    = -d_x_3_base * z;
//             const double d_x_3_5    = d_x_3_base * armor_x;
//             const double d_x_3_7    = -d_x_3_0 * sin(theta);
//             const double d_x_3_9    = -d_x_3_0 * r * std::cos(theta);

//             const double d_x_4_base = 1.0 / std::sqrt(z * z + armor_x_2 + armor_y_2);
//             const double d_x_4_0    = d_x_4_base * armor_x;
//             const double d_x_4_2    = d_x_4_base * armor_y;
//             const double d_x_4_5    = d_x_4_base * z;
//             const double d_x_4_7 =
//                 d_x_4_base * (std::cos(theta) * armor_y - std::sin(theta) * armor_x);
//             const double d_x_4_9 =
//                 -d_x_4_base * r * (std::sin(theta) * armor_y + std::cos(theta) * armor_x);

//             // clang-format off
//             H_ <<      0., 0.,      0., 0., 0.,      0., 0.,      0., 0.,      1., 0.,
//                   d_x_2_0, 0., d_x_2_2, 0., 0.,      0., 0., d_x_2_7, 0., d_x_2_9, 0., 
//                   d_x_3_0, 0.,      0., 0., 0., d_x_3_5, 0., d_x_3_7, 0., d_x_3_9, 0.,
//                   d_x_4_0, 0., d_x_4_2, 0., 0., d_x_4_5, 0., d_x_4_7, 0., d_x_4_9, 0.,
//                        0., 0.,      0., 0., 0.,      1., 0.,      0., 0.,      0., 0.;
//             // clang-format on
//         } else {
//             const double r         = x_k_n(8);
//             const double z         = x_k_n(6);
//             const double armor_x   = x_k_n(0) - r * std::sin(theta);
//             const double armor_y   = x_k_n(2) + r * std::cos(theta);
//             const double armor_x_2 = armor_x * armor_x;
//             const double armor_y_2 = armor_y * armor_y;

//             const double d_x_2_base = 1.0 / (1.0 + armor_y_2 / armor_x_2);
//             const double d_x_2_0    = -d_x_2_base * armor_y / armor_x_2;
//             const double d_x_2_2    = d_x_2_base / armor_x;
//             const double d_x_2_8 =
//                 d_x_2_base * (armor_y / armor_x_2 * std::sin(theta) + std::cos(theta) / armor_x);
//             const double d_x_2_9 =
//                 d_x_2_base * r
//                 * (std::cos(theta) * armor_y / armor_x_2 - std::sin(theta) / armor_x);

//             const double d_x_3_base = -1.0 / armor_x_2 / (1.0 + z * z / armor_x_2);
//             const double d_x_3_0    = -d_x_3_base * z;
//             const double d_x_3_6    = d_x_3_base * armor_x;
//             const double d_x_3_8    = -d_x_3_0 * sin(theta);
//             const double d_x_3_9    = -d_x_3_0 * r * std::cos(theta);

//             const double d_x_4_base = 1.0 / std::sqrt(z * z + armor_x_2 + armor_y_2);
//             const double d_x_4_0    = d_x_4_base * armor_x;
//             const double d_x_4_2    = d_x_4_base * armor_y;
//             const double d_x_4_6    = d_x_4_base * z;
//             const double d_x_4_8 =
//                 d_x_4_base * (std::cos(theta) * armor_y - std::sin(theta) * armor_x);
//             const double d_x_4_9 =
//                 -d_x_4_base * r * (std::sin(theta) * armor_y + std::cos(theta) * armor_x);

//             // clang-format off
//             H_ <<      0., 0.,      0., 0., 0., 0.,      0., 0.,      0.,      1., 0.,
//                   d_x_2_0, 0., d_x_2_2, 0., 0., 0.,      0., 0., d_x_2_8, d_x_2_9, 0., 
//                   d_x_3_0, 0.,      0., 0., 0., 0., d_x_3_6, 0., d_x_3_8, d_x_3_9, 0.,
//                   d_x_4_0, 0., d_x_4_2, 0., 0., 0., d_x_4_6, 0., d_x_4_8, d_x_4_9, 0.,
//                        0., 0.,      0., 0., 0., 0.,      1., 0.,      0.,      0., 0.;
//             // clang-format on
//         }
//         return H_;
//     };

//     inline VMat V(const XVec&, const VVec&) const { return V_; };

//     inline QMat Q(const double& dt) {
//         // Q_.setIdentity();
//         if (side_flag_) {
//             // clang-format off
//             Q_ << 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
//                   0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
//                   0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0.,
//                   0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0.,
//                   0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0.,
//                   0., 0., 0., 0., 0., 1., 0., 0., 0., 0., 0.,
//                   0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
//                   0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0.,
//                   0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
//                   0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0.,
//                   0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1.;
//             // clang-format on
//         } else {
//             // clang-format off
//             Q_ << 1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
//                   0., 1., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
//                   0., 0., 1., 0., 0., 0., 0., 0., 0., 0., 0.,
//                   0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0.,
//                   0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0.,
//                   0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
//                   0., 0., 0., 0., 0., 0., 1., 0., 0., 0., 0.,
//                   0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
//                   0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0.,
//                   0., 0., 0., 0., 0., 0., 0., 0., 0., 1., 0.,
//                   0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 1.;
//             // clang-format on
//         }
//         return Q_;
//     };

//     inline RMat R(const ZVec& z) {
//         // 加 0.01 的偏置量主要是为了防止求不出逆矩阵
//         const double yaw_center_difference_ = std::abs(z(0) - std::numbers::pi / 2) + 0.01;
//         R_.diagonal() << 0.001, 0.001, 0.001, 1., 0.0;
//         return R_;
//     };

//     XVec f_{};
//     ZVec h_{};
//     AMat A_{};
//     WMat W_{WMat::Identity()};
//     HMat H_{};
//     VMat V_{VMat::Identity()};
//     QMat Q_{};
//     RMat R_{};

//     static constexpr double switch_angle_difference_ = std::numbers::pi / 4.;
//     static constexpr double r_theta_theta_           = 0.5;
//     static constexpr double r_theta_yaw_             = 0.5;
//     static constexpr double r_theta_pitch_           = 0.5;
//     static constexpr double r_theta_distance_        = 1.0;

//     bool debug_{true};

//     double last_theta_{std::numbers::pi / 2.};
//     bool side_flag_{true};
// };
// } // namespace rmcs_auto_aim::tracker