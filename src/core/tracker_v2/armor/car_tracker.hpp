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

    void update_car(const CarKF::ZVec& zk, const double& dt) {
        CarKF::XVec out{};
        out << car_kf_.OutPut();
        RCLCPP_INFO(rclcpp::get_logger("out"), "x:%lf,y:%lf,z:%lf", out(0), out(2), out(4));
        RCLCPP_INFO(rclcpp::get_logger("in"), "x:%lf,y:%lf,z:%lf", zk(0), zk(1), zk(2));
        car_kf_.Update(zk, {}, dt);
    }
    std::vector<ArmorPlate3d> get_armor(double dt = 0) {
        armors_.clear();
        auto X = car_kf_.OutPut();

        Eigen::Vector3d center{X(0) + X(1) * dt, X(2) + X(3) * dt, 0};

        // RCLCPP_INFO(
        //     rclcpp::get_logger(""), "x:%lf,y:%lf,z:%lf,dt:%lf", center.x(), center.y(),
        //     center.z(), dt * 1000);

        auto angle       = X(4) + dt * X(5);
        auto forward_car = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ())
                         * Eigen::AngleAxisd(-15, Eigen::Vector3d::UnitY());
        auto forward_armor = Eigen::AngleAxisd(angle + std::numbers::pi, Eigen::Vector3d::UnitZ())
                           * Eigen::AngleAxisd(15, Eigen::Vector3d::UnitY());
        armors_.emplace_back(
            rmcs_msgs::ArmorID::Unknown,
            rmcs_description::OdomImu::Position(
                center + forward_car.toRotationMatrix() * Eigen::Vector3d::UnitX() * l1
                + Eigen::Vector3d::UnitZ() * z1),
            rmcs_description::OdomImu::Rotation(forward_armor));

        forward_car = Eigen::AngleAxisd(angle + std::numbers::pi * 1 / 2, Eigen::Vector3d::UnitZ())
                    * Eigen::AngleAxisd(-15, Eigen::Vector3d::UnitY());
        forward_armor =
            Eigen::AngleAxisd(angle + +std::numbers::pi * 3 / 2, Eigen::Vector3d::UnitZ())
            * Eigen::AngleAxisd(15, Eigen::Vector3d::UnitY());
        armors_.emplace_back(
            rmcs_msgs::ArmorID::Unknown,
            rmcs_description::OdomImu::Position(
                center + forward_car.toRotationMatrix() * Eigen::Vector3d::UnitX() * l2
                + Eigen::Vector3d::UnitZ() * z2),
            rmcs_description::OdomImu::Rotation(forward_armor));

        forward_car = Eigen::AngleAxisd(angle + std::numbers::pi * 1, Eigen::Vector3d::UnitZ())
                    * Eigen::AngleAxisd(-15, Eigen::Vector3d::UnitY());
        forward_armor = Eigen::AngleAxisd(angle + std::numbers::pi * 2, Eigen::Vector3d::UnitZ())
                      * Eigen::AngleAxisd(15, Eigen::Vector3d::UnitY());
        armors_.emplace_back(
            rmcs_msgs::ArmorID::Unknown,
            rmcs_description::OdomImu::Position(
                center + forward_car.toRotationMatrix() * Eigen::Vector3d::UnitX() * l1
                + Eigen::Vector3d::UnitZ() * z3),
            rmcs_description::OdomImu::Rotation(forward_armor));

        forward_car = Eigen::AngleAxisd(angle + std::numbers::pi * 3 / 2, Eigen::Vector3d::UnitZ())
                    * Eigen::AngleAxisd(-15, Eigen::Vector3d::UnitY());
        forward_armor =
            Eigen::AngleAxisd(angle + std::numbers::pi * 5 / 2, Eigen::Vector3d::UnitZ())
            * Eigen::AngleAxisd(15, Eigen::Vector3d::UnitY());
        armors_.emplace_back(
            rmcs_msgs::ArmorID::Unknown,
            rmcs_description::OdomImu::Position(
                center + forward_car.toRotationMatrix() * Eigen::Vector3d::UnitX() * l2
                + Eigen::Vector3d::UnitZ() * z4),
            rmcs_description::OdomImu::Rotation(forward_armor));

        return armors_;
    }

    void update_frame(double l1, double l2) {
        car_frame_kf_.Update({l1, l2}, {}, 0);
        auto frame = car_frame_kf_.OutPut();
        this->l1   = frame(0);
        this->l2   = frame(1);
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
    double l1 = 0.6, l2 = 0.6;
    double z1 = 0, z2 = 0, z3 = 0, z4 = 0;

    std::vector<ArmorPlate3d> armors_;
};
} // namespace rmcs_auto_aim::tracker2