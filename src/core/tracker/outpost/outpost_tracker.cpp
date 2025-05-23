
#include <iostream>
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

#include "./outpost_tracker.hpp"

namespace rmcs_auto_aim::tracker {
class OutPostTracker::Impl {
public:
    Impl()
        : armors_() {}
    void update_self(const double& dt) {
        if (self_update_time_ > 2)
            return;

        outpost_kf_.Update(outpost_kf_.h(outpost_kf_.OutPut(), {}), {}, dt);
        self_update_time_ += dt;
    }

    double get_dt(const std::chrono::steady_clock::time_point& timestamp) {
        auto ret          = std::chrono::duration<double>(timestamp - last_update_time_).count();
        last_update_time_ = timestamp;
        return ret;
    }
    std::chrono::steady_clock::time_point get_timestamp() { return last_update_time_; };

    bool check_armor_tracked() const { return self_update_time_ == 0; }

    double omega() { return outpost_kf_.OutPut()(4); }

    void update_outpost(const OutPostKF::ZVec& zk, const double& dt) {
        detected_yaw = zk(3);
        // std::cerr << zk << std::endl << std::endl;
        outpost_kf_.Update(zk, {}, dt);

        self_update_time_ = 0;
    }

    std::vector<ArmorPlate3d> get_armor(double dt = 0) {
        armors_.clear();
        if (!check_armor_tracked()) {}

        const auto kf_output = outpost_kf_.OutPut();

        // std::cerr << "output:\n" << kf_output << std::endl << std ::endl;

        Eigen::Vector3d center{kf_output(0), kf_output(1), kf_output(2)};

        auto angle = kf_output(3) + dt * kf_output(4);
        if (dt == 0)
            angle = detected_yaw;

        add_armor(angle, center, radius_);
        angle += std::numbers::pi * 2 / 3;
        add_armor(angle, center, radius_);
        angle += std::numbers::pi * 2 / 3;
        add_armor(angle, center, radius_);

        return armors_;
    }

    [[nodiscard]] rmcs_description::OdomImu::Position get_outpost_position() {
        armors_.clear();
        if (!check_armor_tracked()) {}
        const auto kf_output = outpost_kf_.OutPut();

        return rmcs_description::OdomImu::Position{kf_output(0), kf_output(1), kf_output(2)};
    }

    static std::tuple<double, double> get_frame() { return {radius_, radius_}; }

private:
    void add_armor(double angle, const Eigen::Vector3d& center, const double& l) {
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

        armors_.emplace_back(
            rmcs_msgs::ArmorID::Unknown, rmcs_description::OdomImu::Position(ccenter_),
            rmcs_description::OdomImu::Rotation(forward_armor));
    }

    OutPostKF outpost_kf_;
    double detected_yaw = 0;
    std::chrono::steady_clock::time_point last_update_time_;
    double self_update_time_ = 10086;

    constexpr static const double radius_          = 0.2765;
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

[[nodiscard]] rmcs_description::OdomImu::Position OutPostTracker::get_outpost_position(double) {
    return pimpl_->get_outpost_position();
}

OutPostTracker::~OutPostTracker() = default;

double OutPostTracker::get_dt(const std::chrono::steady_clock::time_point& timestamp) {
    return pimpl_->get_dt(timestamp);
}
} // namespace rmcs_auto_aim::tracker
