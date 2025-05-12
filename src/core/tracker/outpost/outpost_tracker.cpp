
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

#include "outpost_tracker.hpp"

namespace rmcs_auto_aim::tracker {
class OutPostTracker::Impl {
public:
    Impl()
        : armors_() {}
    void update_self(const double& dt) {
        if (self_update_time_ > 0.3)
            return;

        self_update_time_ += dt;
    }

    double get_dt(const std::chrono::steady_clock::time_point& timestamp) {
        auto ret          = std::chrono::duration<double>(timestamp - last_update_time_).count();
        last_update_time_ = timestamp;
        return ret;
    }
    std::chrono::steady_clock::time_point get_timestamp() { return last_update_time_; };

    bool check_armor_tracked() const { return self_update_time_ == 0; }

    static inline double omega() { return rotate_velocity_; }

    void update_outpost(const OutPostKF::ZVec& zk, const double& dt) {
        detected_yaw = zk(2);
        outpost_kf_.Update(zk, {}, dt);

        self_update_time_ = 0;
    }
    
    std::vector<ArmorPlate3d> get_armor(double dt = 0) {
        armors_.clear();
        if (!check_armor_tracked()) {}

        const auto kf_output = outpost_kf_.OutPut();

        Eigen::Vector3d center{kf_output(0), kf_output(1), 0};

        auto angle = kf_output(2) + dt * rotate_velocity_;
        if (dt == 0)
            angle = detected_yaw;

        add_armor(angle, z1, center, radius_);
        angle += std::numbers::pi * 2 / 3;
        add_armor(angle, z2, center, radius_);
        angle += std::numbers::pi * 2 / 3;
        add_armor(angle, z3, center, radius_);

        return armors_;
    }

    void update_z(const double& z1, const double& z2, const double& z3) {

        this->z1 = z1;
        this->z2 = z2;
        this->z3 = z3;
    };
    Eigen::Vector<double, 3> get_z() const { return {z1, z2, z3}; }
    [[nodiscard]] rmcs_description::OdomImu::Position get_car_position(double dt = 0) {
        armors_.clear();
        if (!check_armor_tracked()) {}
        const auto kf_output = outpost_kf_.OutPut();

        return rmcs_description::OdomImu::Position{kf_output(0), kf_output(1), 0};
    }

    static std::tuple<double, double> get_frame() { return {radius_, radius_}; }

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

    OutPostKF outpost_kf_;
    double z1 = 0, z2 = 0, z3 = 0;
    double detected_yaw = 0;
    std::chrono::steady_clock::time_point last_update_time_;
    double self_update_time_ = 10086;

    constexpr static const double radius_          = 276.5;
    constexpr static const double rotate_velocity_ = 0.8 * std::numbers::pi;

    std::vector<ArmorPlate3d> armors_;
};

OutPostTracker::OutPostTracker() { pimpl_ = std::make_unique<Impl>(); }

OutPostTracker::OutPostTracker(const OutPostTracker& car_tracker) {
    pimpl_ = std::make_unique<Impl>(*car_tracker.pimpl_);
}

void OutPostTracker::update_self(const double& dt) { pimpl_->update_self(dt); }

bool OutPostTracker::check_armor_tracked() const { return pimpl_->check_armor_tracked(); }

double OutPostTracker::omega() { return pimpl_->omega(); }

void OutPostTracker::update_outpost(const OutPostKF::ZVec& zk, const double& dt) {
    pimpl_->update_outpost(zk, dt);
}

std::vector<ArmorPlate3d> OutPostTracker::get_armor(double dt) { return pimpl_->get_armor(dt); }

std::tuple<double, double> OutPostTracker::get_frame() { return pimpl_->get_frame(); }

void OutPostTracker::update_z(const double& z1, const double& z2, const double& z3) {
    return pimpl_->update_z(z1, z2, z3);
}

Eigen::Vector<double, 3> OutPostTracker::get_armor_height() const { return pimpl_->get_z(); }

[[nodiscard]] rmcs_description::OdomImu::Position OutPostTracker::get_car_position(double dt) {
    return pimpl_->get_car_position(dt);
}

OutPostTracker::~OutPostTracker() = default;

double OutPostTracker::get_dt(const std::chrono::steady_clock::time_point& timestamp) {
    return pimpl_->get_dt(timestamp);
}
} // namespace rmcs_auto_aim::tracker