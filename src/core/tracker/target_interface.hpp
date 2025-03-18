#pragma once

#include <fast_tf/fast_tf.hpp>
#include <rmcs_description/tf_description.hpp>

namespace rmcs_auto_aim::tracker {
class ITarget {
public:
    [[nodiscard]] virtual rmcs_description::OdomImu::Position
        Predict(double sec, const rmcs_description::Tf&)                         = 0;
    [[nodiscard]] virtual double get_omega()                                     = 0;
    [[nodiscard]] virtual std::tuple<double, double> get_frame()                 = 0;
    [[nodiscard]] virtual rmcs_description::OdomImu::Position get_car_position() = 0;
};
} // namespace rmcs_auto_aim::tracker