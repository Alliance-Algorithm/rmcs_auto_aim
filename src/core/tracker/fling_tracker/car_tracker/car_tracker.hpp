#pragma once

#include <Eigen/Eigen>

#include <memory>

#include "core/pnpsolver/armor/armor3d.hpp"

namespace rmcs_auto_aim::tracker::fling_tracker {
class CarTracker {
public:
    CarTracker();
    CarTracker(const CarTracker&);

    ~CarTracker();

    bool check_armor_tracked() const;

    void update_self(const double& dt);

    void update_car(const Eigen::Vector<double, 3>& zk, const double& dt);

    void update_frame(double l1, double l2);

    std::vector<ArmorPlate3d> get_armor(double dt = 0);

    void update_z(const Eigen::Vector<double, 4>& z);

    Eigen::Vector<double, 4> get_z() const;

    double get_omega() const;

    std::tuple<double, double> get_frame();

private:
    class Impl;

    std::unique_ptr<Impl> pimpl_;
};
} // namespace rmcs_auto_aim::tracker::fling_tracker
