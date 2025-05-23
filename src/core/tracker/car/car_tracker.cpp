
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
        , armors_() {}
    Eigen::Vector2d velocity() { return {car_kf_.OutPut()(1), car_kf_.OutPut()(3)}; }
    void update_self(const double& dt) {
        if (self_update_time_ > 0.3)
            return;

        auto X = Eigen::Vector3d{car_kf_.OutPut()(0), car_kf_.OutPut()(2), car_kf_.OutPut()(4)};

        self_update_time_ += dt;
        last_acc_ << 0, 0;

        Eigen::Vector3d center{};
        center << X(0) + 0. * dt, X(1) + 0. * dt, 0;

        center(2) = X(2) + dt * 0.8 * std::numbers::pi;
    }

    double get_dt(const std::chrono::steady_clock::time_point& timestamp) {
        auto ret          = std::chrono::duration<double>(timestamp - last_update_time_).count();
        last_update_time_ = timestamp;
        return ret;
    }
    std::chrono::steady_clock::time_point get_timestamp() { return last_update_time_; };

    bool check_armor_tracked() const { return self_update_time_ == 0; }

    double omega() { return 0.8 * std::numbers::pi; }

    void update_car(const CarPosKF::ZVec& zk, const double& dt) {
        detected_yaw = zk(2);
        last_acc_ << 0, 0;
        last_vel_ << 0, 0;

        car_kf_.Update(zk, {}, dt);

        self_update_time_ = 0;
    }
    std::vector<ArmorPlate3d> get_armor(double dt = 0) {
        armors_.clear();
        auto X = Eigen::Vector3d{car_kf_.OutPut()(0), car_kf_.OutPut()(2), car_kf_.OutPut()(4)};
        if (!check_armor_tracked())
            last_acc_ << 0, 0;

        Eigen::Vector3d center{
            X(0) + last_vel_(0) * dt + last_acc_(0) * dt * dt / 2.0,
            X(1) + last_vel_(1) * dt + last_acc_(1) * dt * dt / 2.0, 0};

        auto angle = X(2) + dt * 0.8 * std::numbers::pi;
        if (dt == 0)
            angle = detected_yaw;

        add_armor(angle, z1, center, l1);
        angle += std::numbers::pi * 2 / 3;
        add_armor(angle, z2, center, l2);
        angle += std::numbers::pi * 2 / 3;
        add_armor(angle, z3, center, l1);
        // angle += std::numbers::pi / 2;
        // add_armor(angle, z4, center, l2);

        return armors_;
    }

    void update_frame(double l1, double l2) {
        this->l1 = 0.2765;
        this->l2 = 0.2765;
    };

    void update_z(const double& z1, const double& z2, const double& z3) {

        this->z1 = z1;
        this->z2 = z2;
        this->z3 = z3;
    };
    Eigen::Vector<double, 3> get_z() const { return {z1, z2, z3}; }
    std::tuple<double, double> get_frame() { return {l1, l2}; }
    [[nodiscard]] rmcs_description::OdomImu::Position get_car_position(double dt = 0) {
        armors_.clear();
        auto X = Eigen::Vector3d{car_kf_.OutPut()(0), car_kf_.OutPut()(2), car_kf_.OutPut()(4)};
        if (!check_armor_tracked())
            last_acc_ << 0, 0;

        Eigen::Vector3d center{X(0), X(1), 0};

        return rmcs_description::OdomImu::Position(center);
    }

private:
    void add_armor(double angle, double z, const Eigen::Vector3d& center, const double& l) {
        Eigen::Quaterniond forward_armor =
            Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ())
            * Eigen::AngleAxisd(
                -15. / 180. * std::numbers::pi,
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
    double l1 = 0.2765, l2 = 0.2765;
    double z1 = 0, z2 = 0, z3 = 0;
    double detected_yaw = 0;
    std::chrono::steady_clock::time_point last_update_time_;
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

void CarTracker::update_z(const double& z1, const double& z2, const double& z3) {
    return pimpl_->update_z(z1, z2, z3);
}

Eigen::Vector<double, 3> CarTracker::get_armor_height() const { return pimpl_->get_z(); }

[[nodiscard]] rmcs_description::OdomImu::Position CarTracker::get_car_position(double dt) {
    return pimpl_->get_car_position(dt);
}

std::tuple<double, double> CarTracker::get_frame() { return pimpl_->get_frame(); }

Eigen::Vector2d CarTracker::velocity() { return pimpl_->velocity(); }
CarTracker::~CarTracker() = default;

double CarTracker::get_dt(const std::chrono::steady_clock::time_point& timestamp) {
    return pimpl_->get_dt(timestamp);
}
} // namespace rmcs_auto_aim::tracker
