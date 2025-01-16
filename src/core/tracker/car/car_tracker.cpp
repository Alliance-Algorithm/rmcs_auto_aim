

#include <memory>
#include <numbers>
#include <tuple>
#include <vector>

#include <Eigen/Eigen>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include <rmcs_description/tf_description.hpp>
#include <rmcs_msgs/robot_id.hpp>

#include "core/pnpsolver/armor/armor3d.hpp"

#include "core/tracker/car/filter/car_frame_kf.hpp"
#include "core/tracker/car/filter/car_frame_z_kf.hpp"
#include "core/tracker/car/filter/car_kf.hpp"
#include "core/tracker/car/filter/car_movement_kf.hpp"
#include "core/tracker/car/filter/car_pos_kf.hpp"

#include "car_tracker.hpp"

namespace rmcs_auto_aim::tracker {
class CarTracker::Impl {
public:
    Impl()
        : car_kf_()
        , car_frame_kf_()
        , car_pos_kf()
        , car_movement_kf_()
        , armors_() {
        car_frame_kf_.Update({l1, l2}, {}, 0);
    }

    void update_self(const double& dt) {
        if (self_update_time_ > 0.7)
            return;

        auto X  = car_pos_kf.OutPut();
        auto Vx = car_movement_kf_.OutPut();

        self_update_time_ += dt;
        last_acc_ << 0, 0;

        Eigen::Vector3d center{};
        center << X(0) + Vx(0) * dt, X(1) + Vx(1) * dt, 0;

        center(2) = X(2) + dt * Vx(2);

        car_pos_kf.Update(center, {}, dt);
    }

    bool check_armor_tracked() const { return self_update_time_ == 0; }

    double omega() { return car_movement_kf_.OutPut()(2); }

    void update_car(const CarPosKF::ZVec& zk, const double& dt) {

        car_pos_kf.Update(zk, {}, dt);

        CarKF ::ZVec car_z{};
        car_z << car_pos_kf.OutPut();

        last_acc_ << car_movement_kf_.OutPut()(0), car_movement_kf_.OutPut()(1);
        last_vel_ << last_acc_;

        car_kf_.Update(car_z, {}, dt);
        car_movement_kf_.Update(
            {car_kf_.OutPut()(1), car_kf_.OutPut()(3), car_kf_.OutPut()(5)}, {}, dt);

        last_acc_ << (car_movement_kf_.OutPut()(0) - last_acc_(0)) / dt,
            (car_movement_kf_.OutPut()(1) - last_acc_(1)) / dt;
        last_vel_ << (car_movement_kf_.OutPut()(0) + last_vel_(0)) / 2,
            (car_movement_kf_.OutPut()(1) + last_vel_(1)) / 2;

        self_update_time_ = 0;
    }
    std::vector<ArmorPlate3d> get_armor(double dt = 0) {
        armors_.clear();
        auto X  = car_pos_kf.OutPut();
        auto Vx = car_movement_kf_.OutPut();
        if (!check_armor_tracked())
            last_acc_ << 0, 0;

        Eigen::Vector3d center{
            X(0) + last_vel_(0) * dt + last_acc_(0) * dt * dt / 2.0,
            X(1) + last_vel_(1) * dt + last_acc_(1) * dt * dt / 2.0, 0};

        if (last_acc_.norm() < 0.5)
            center << X(0) + Vx(0) * dt, X(1) + Vx(1) * dt, 0;

        auto angle = X(2) + dt * Vx(2);

        add_armor(angle, z1, center, l1);
        angle += std::numbers::pi / 2;
        add_armor(angle, z2, center, l2);
        angle += std::numbers::pi / 2;
        add_armor(angle, z3, center, l1);
        angle += std::numbers::pi / 2;
        add_armor(angle, z4, center, l2);

        return armors_;
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
    Eigen::Vector<double, 4> get_z() const { return {z1, z2, z3, z4}; }
    std::tuple<double, double> get_frame() { return {l1, l2}; }

private:
    void add_armor(double angle, double z, const Eigen::Vector3d& center, const double& l) {
        Eigen::Quaterniond forward_armor =
            Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ())
            * Eigen::AngleAxisd(
                15. / 180. * std::numbers::pi,
                *rmcs_description::OdomImu::DirectionVector(Eigen::Vector3d::UnitY()));

        Eigen::Vector3d ccenter_{};
        ccenter_ << *rmcs_description::OdomImu::Position(center)
                        - Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()).toRotationMatrix()
                              * *rmcs_description::OdomImu::DirectionVector(
                                  Eigen::Vector3d::UnitX())
                              * l;
        ccenter_.z() = z;

        armors_.emplace_back(
            rmcs_msgs::ArmorID::Unknown, rmcs_description::OdomImu::Position(ccenter_),
            rmcs_description::OdomImu::Rotation(forward_armor));
    }
    Eigen::Vector2d last_acc_ = {0, 0};
    Eigen::Vector2d last_vel_ = {0, 0};
    CarKF car_kf_;
    CarFrameKF car_frame_kf_;
    CarFrameZKF car_frame_z_kf_;
    CarPosKF car_pos_kf;
    CarMovementKF car_movement_kf_;
    double l1 = 0.3, l2 = 0.3;
    double z1 = 0, z2 = 0, z3 = 0, z4 = 0;

    double self_update_time_ = 10086;

    constexpr static const double alpha_ = 1;

    std::vector<ArmorPlate3d> armors_;
};

CarTracker::CarTracker() { pimpl_ = std::make_unique<Impl>(); }

CarTracker::CarTracker(const CarTracker& car_tracker) {
    pimpl_ = std::make_unique<Impl>(*car_tracker.pimpl_);
}

void CarTracker::update_self(const double& dt) { pimpl_->update_self(dt); }

bool CarTracker::check_armor_tracked() const { return pimpl_->check_armor_tracked(); }

double CarTracker::omega() { return pimpl_->omega(); }

void CarTracker::update_car(const Eigen::Vector<double, 3>& zk, const double& dt) {
    pimpl_->update_car(zk, dt);
}

std::vector<ArmorPlate3d> CarTracker::get_armor(double dt) { return pimpl_->get_armor(dt); }

void CarTracker::update_frame(double l1, double l2) { return pimpl_->update_frame(l1, l2); }

void CarTracker::update_z(const double& z1, const double& z2, const double& z3, const double& z4) {
    return pimpl_->update_z(z1, z2, z3, z4);
}

Eigen::Vector<double, 4> CarTracker::get_z() const { return pimpl_->get_z(); }

std::tuple<double, double> CarTracker::get_frame() { return pimpl_->get_frame(); }
CarTracker::~CarTracker() = default;
} // namespace rmcs_auto_aim::tracker