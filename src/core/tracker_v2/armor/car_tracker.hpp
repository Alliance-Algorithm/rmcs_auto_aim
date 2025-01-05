#pragma once

#include <numbers>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rmcs_description/tf_description.hpp>
#include <robot_id.hpp>
#include <tuple>
#include <vector>

#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/AngleAxis.h>
#include <Eigen/src/Geometry/Quaternion.h>

#include "core/pnpsolver/armor/armor3d.hpp"
#include "core/tracker_v2/armor/car_frame_kf.hpp"
#include "core/tracker_v2/armor/car_kf.hpp"

namespace rmcs_auto_aim::tracker2 {
class CarTracker {
public:
    CarTracker()
        : car_kf_()
        , car_frame_kf_()
        , armors_() {
        car_frame_kf_.Update({l1, l2}, {}, 0);
    }

    void update_self(const double& dt) { car_kf_.Update(car_kf_.h(car_kf_.OutPut(), {}), {}, dt); }

    void update_car(const CarKF::ZVec& zk, const double& dt) { car_kf_.Update(zk, {}, dt); }
    std::vector<ArmorPlate3d> get_armor(double dt = 0) {
        armors_.clear();
        auto X = car_kf_.OutPut();

        Eigen::Vector3d center{X(0) + X(1) * dt, X(2) + X(3) * dt, 0};

        auto angle = X(4) + dt * X(5);

        // RCLCPP_INFO(rclcpp::get_logger(""), "%lf", angle);
        add_armor(angle, z1, center, l1);
        angle += std::numbers::pi / 2;
        add_armor(angle, z2, center, l2);
        angle += std::numbers::pi / 2;
        add_armor(angle, z3, center, l1);
        angle += std::numbers::pi / 2;
        add_armor(angle, z4, center, l2);

        return armors_;
    }
    void add_armor(double angle, double z, const Eigen::Vector3d& center, const double& l) {
        auto forward_armor =
            Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ())
            * Eigen::AngleAxisd(
                15. / 180. * std::numbers::pi,
                *rmcs_description::OdomImu::DirectionVector(Eigen::Vector3d::UnitY()));

        Eigen::Vector3d ccenter_ =
            *rmcs_description::OdomImu::Position(center)
            - Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()).toRotationMatrix()
                  * *rmcs_description::OdomImu::DirectionVector(Eigen::Vector3d::UnitX()) * l;
        ccenter_.z() = z;

        // RCLCPP_INFO(
        //     rclcpp::get_logger("armor0"), "x:%lf,y:%lf,z:%lf,w:%lf,px:%lf,py:%lf,pz:%lf",
        //     forward_armor.x(), forward_armor.y(), forward_armor.z(), forward_armor.w(),
        //     ccenter_.x(), ccenter_.y(), ccenter_.z());
        armors_.emplace_back(
            rmcs_msgs::ArmorID::Unknown, rmcs_description::OdomImu::Position(ccenter_),
            rmcs_description::OdomImu::Rotation(forward_armor));
    }

    void update_frame(double l1, double l2) {
        car_frame_kf_.Update({l1, l2}, {}, 0);
        auto frame = car_frame_kf_.OutPut();
        this->l1   = std::clamp(frame(0), 0.1, 0.6);
        this->l2   = std::clamp(frame(1), 0.1, 0.6);
    };

    void update_z(const double& z1, const double& z2, const double& z3, const double& z4) {
        this->z1 = z1;
        this->z2 = z2;
        this->z3 = z3;
        this->z4 = z4;
    };
    std::tuple<double, double> get_frame() { return {l1, l2}; }

private:
    CarKF car_kf_;
    CarFrameKF car_frame_kf_;
    double l1 = 0.3, l2 = 0.3;
    double z1 = 0, z2 = 0, z3 = 0, z4 = 0;

    std::vector<ArmorPlate3d> armors_;
};
} // namespace rmcs_auto_aim::tracker2