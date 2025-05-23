#pragma once

#include <memory>
#include <pluginlib/class_loader.hpp>
#include <tuple>
#include <vector>

#include <Eigen/Eigen>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include <rmcs_description/tf_description.hpp>
#include <rmcs_msgs/robot_id.hpp>

#include "core/pnpsolver/armor/armor3d.hpp"

namespace rmcs_auto_aim::tracker {
class CarTracker {
public:
    CarTracker();
    CarTracker(const CarTracker&);

    ~CarTracker();

    void update_self(const double& dt);

    bool check_armor_tracked() const;
    double omega();
    Eigen::Vector2d velocity();
    void update_car(const Eigen::Vector<double, 3>& zk, const double& dt);

    std::vector<ArmorPlate3d> get_armor(double dt = 0);

    void update_frame(double l1, double l2);

    void update_z(const double& z1, const double& z2, const double& z3);
    Eigen::Vector<double, 3> get_armor_height() const;
    std::tuple<double, double> get_frame();
    std::chrono::steady_clock::time_point get_timestamp();

    [[nodiscard]] rmcs_description::OdomImu::Position get_car_position(double dt = 0);
    [[nodiscard]] double get_dt(const std::chrono::steady_clock::time_point& timestamp);

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};
} // namespace rmcs_auto_aim::tracker
