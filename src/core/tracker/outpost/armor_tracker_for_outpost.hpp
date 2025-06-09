#pragma once

#include <chrono>
#include <memory>
#include <numbers>
#include <vector>

#include <Eigen/Eigen>

#include <fast_tf/impl/cast.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_msgs/robot_id.hpp>

#include "core/fire_controller/outpost_controller.hpp"
#include "core/pnpsolver/armor/armor3d.hpp"

#include "core/fire_controller/fire_controller.hpp"
#include "core/fire_controller/tracker_test_controller.hpp"
#include "core/tracker/outpost/outpost_tracker.hpp"
#include "core/tracker/tracker_interface.hpp"
#include "core/transform_optimizer/armor/quadrilateral/quadrilateral.hpp"

#include "util/image_viewer/image_viewer.hpp"
#include "util/math.hpp"

namespace rmcs_auto_aim::tracker::armor {
class OutPostArmorTracker : public ITracker {
    using TFireController = rmcs_auto_aim::fire_controller::TrackerTestController;

public:
    OutPostArmorTracker()
        : target_() {}

    std::shared_ptr<IFireController> Update(
        const std::vector<ArmorPlate3d>& armors,
        const std::chrono::steady_clock::time_point& timestamp,
        const rmcs_description::Tf& tf) override {
        target_.SetTracker(nullptr);

        last_armors1_ = outpost_tracker_.get_armor();
        last_update_  = timestamp;

        const auto outpost_armors = get_outpost_armor(armors);
        // std::cerr << "hello"
        if (outpost_armors.size() > 0) {
            nearest_distance_ = 1e7;

            double dt                = outpost_tracker_.get_dt(timestamp);
            last_armors2_            = outpost_tracker_.get_armor(dt);
            auto last_detected_armor = outpost_tracker_.get_armor();

            Eigen::Vector<double, 4> armor_z{};

            armor_z << *outpost_armors[0].position,
                util::math::get_yaw_from_quaternion(*outpost_armors[0].rotation);

            rmcs_description::OdomImu::Position outpost_pos =
                armor_to_outpost(armor_z, outpost_radius, tf);
            // std::cerr << armor_z << std::endl << *outpost_pos << std::endl << std::endl;

            const auto cross_product =
                outpost_pos->normalized().cross(armors[0].position->normalized()).z();
            if (cross_product < -0.0003)
                last_cross_product_ = cross_product;

            if (last_cross_product_ < -0.0003 && cross_product > 0.0003) {
                last_cross_product_  = cross_product;
                const auto pos_error = (*armors[0].position - *last_detected_armor_pos_).norm();
                if (pos_error > 0.3) {
                    ++last_detected_armor_id_;
                    last_detected_armor_id_ %= 3;
                    // std::cout << "trigger" << std::endl;
                }
            }
            *last_detected_armor_pos_ << *armors[0].position;

            if (rotate_direction_) {
                outpost_tracker_.update_outpost(
                    {outpost_pos->x(), outpost_pos->y(), outpost_pos->z(),
                     armor_z(3) + last_detected_armor_id_ * std::numbers::pi * 2 / 3},
                    dt);
            } else {
                outpost_tracker_.update_outpost(
                    {outpost_pos->x(), outpost_pos->y(), outpost_pos->z(),
                     armor_z(3) - last_detected_armor_id_ * std::numbers::pi * 2 / 3},
                    dt);
            }

            target_.SetTracker(std::make_shared<OutPostTracker>(outpost_tracker_));
        } else {
            // outpost_tracker_.update_self(outpost_tracker_.get_dt(timestamp));
        }

        return target_.check() ? std::make_shared<fire_controller::OutPostController>(target_)
                               : nullptr;
    }

    void draw_armors(const rmcs_description::Tf& tf, const cv::Scalar& color) {
        if (last_armors1_.empty())
            return;
        auto color_ = color;
        for (const auto& armor3d : last_armors1_) {
            util::ImageViewer::draw(
                transform_optimizer::Quadrilateral3d(armor3d).ToQuadrilateral(tf, false), color_);
            color_ = color_ / 1.5;
        }
        color_ = {255 - color(0), 255 - color(1), 255 - color(2)};
        for (const auto& armor3d : last_armors2_) {
            util::ImageViewer::draw(
                transform_optimizer::Quadrilateral3d(armor3d).ToQuadrilateral(tf, false), color_);
            color_ = color_ / 1.5;
        }
    }

    double pos_error() const { return last_cross_product_; }

    rmcs_description::OdomImu::Position outpost_pos() {
        outpost_pos_.emplace_back(outpost_tracker_.get_outpost_position());
        if (outpost_pos_.size() >= 100) {
            last_outpost_pos_ = {0, 0, 0};
            for (const auto& pos : outpost_pos_)
                *last_outpost_pos_ += *pos;
            *last_outpost_pos_ /= outpost_pos_.size();
            outpost_pos_.clear();
        }
        return last_outpost_pos_;
    }

    bool rotate_direction() const { return rotate_direction_; }
    void set_rotate_direction(const bool& rotate_direction) {
        rotate_direction_ = rotate_direction;
    }

private:
    static inline std::vector<ArmorPlate3d>
        get_outpost_armor(const std::vector<ArmorPlate3d>& armors) {
        std::vector<ArmorPlate3d> outpost_armors;
        for (const auto& armor : armors) {
            if (armor.id == rmcs_msgs::ArmorID::Outpost) {
                outpost_armors.emplace_back(armor);
            }
        }
        return outpost_armors;
    }

    static rmcs_description::OdomImu::Position
        armor_to_outpost(const Eigen::Vector4d& z, const double& l, const auto& tf) {
        return rmcs_description::OdomImu::Position(
            *rmcs_description::OdomImu::Position(Eigen::Vector3d{z(0), z(1), z(2)})
            + rmcs_description::OdomImu::Rotation(Eigen::AngleAxisd(z(3), Eigen::Vector3d::UnitZ()))
                      ->toRotationMatrix()
                  * *rmcs_description::OdomImu::DirectionVector(Eigen::Vector3d::UnitX() * l));
    }

    OutPostTracker outpost_tracker_;
    std::vector<rmcs_description::OdomImu::Position> outpost_pos_;
    rmcs_description::OdomImu::Position last_outpost_pos_{0, 0, 0};
    std::chrono::steady_clock::time_point last_update_;

    double last_detected_armor_yaw_{0.};
    int last_detected_armor_id_{0};
    int test{0};
    double pos_error_{0.};
    double last_cross_product_{0.};

    rmcs_description::OdomImu::Position last_detected_armor_pos_{0, 0, 0};

    std::vector<ArmorPlate3d> last_armors1_;
    std::vector<ArmorPlate3d> last_armors2_;

    bool first_flag_{true};

    static constexpr double outpost_radius = 0.2765;

    // true代表逆时针，false代表顺时针
    bool rotate_direction_{true};

    fire_controller::OutPostController target_;
    double nearest_distance_ = 0.0;
};
} // namespace rmcs_auto_aim::tracker::armor
