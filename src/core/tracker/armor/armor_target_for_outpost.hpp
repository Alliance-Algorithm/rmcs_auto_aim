#pragma once

#include "core/tracker/car/car_tracker.hpp"
#include "core/tracker/outpost/outpost_tracker.hpp"
#include "core/tracker/target_interface.hpp"

namespace rmcs_auto_aim::tracker::armor {
class ArmorTargetForOutPost : public tracker::ITarget {

public:
    explicit ArmorTargetForOutPost(const rmcs_auto_aim::tracker::OutPostTracker& outpost)
        : outpost_(outpost) {};
    rmcs_description::OdomImu::Position
        Predict(double sec, const rmcs_description::Tf& tf) override {
        double max    = -1e7;
        int index     = 0;
        auto armors   = outpost_.get_armor(sec + 0.01);
        auto camera_x = fast_tf::cast<rmcs_description::OdomImu>(
            rmcs_description::CameraLink::DirectionVector(Eigen::Vector3d::UnitX()), tf);
        for (int i = 0; i < 3; i++) {
            auto armor_x = fast_tf::cast<rmcs_description::OdomImu>(
                rmcs_description::OdomImu::DirectionVector(
                    armors[i].rotation->toRotationMatrix() * Eigen::Vector3d::UnitX()),
                tf);
            Eigen::Vector2d vec{};
            vec << Eigen::Vector2d{armors[i].position->x(), armors[i].position->y()};

            auto len = camera_x->dot(Eigen::Vector3d(*armor_x));
            if (len > max) {
                index = i;
                max   = len;
            }
        }
        return outpost_.get_armor(sec)[index].position;
    }

    [[nodiscard]] double get_omega() final { return outpost_.omega(); }
    [[nodiscard]] std::tuple<double, double> get_frame() final { return outpost_.get_frame(); }
    [[nodiscard]] rmcs_description::OdomImu::Position get_car_position() final {
        return outpost_.get_outpost_position();
    }

private:
    rmcs_auto_aim::tracker::OutPostTracker outpost_;
};

} // namespace rmcs_auto_aim::tracker::armor