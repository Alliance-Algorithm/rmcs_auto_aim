#pragma once

#include <chrono>
#include <fast_tf/fast_tf.hpp>
#include <rmcs_description/tf_description.hpp>
#include <tuple>

namespace rmcs_auto_aim::tracker {
class IFireController {

public:
    [[nodiscard]] virtual std::tuple<bool, rmcs_description::OdomImu::Position>
        UpdateController(double sec, const rmcs_description::Tf& tf) = 0;
    double virtual get_omega()                                       = 0;
    std::chrono::steady_clock::time_point virtual get_timestamp()    = 0;
};
} // namespace rmcs_auto_aim::tracker