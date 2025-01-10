#pragma once

#include <fast_tf/fast_tf.hpp>
#include <rmcs_description/tf_description.hpp>

namespace rmcs_auto_aim::tracker2 {
class ITarget {

public:
    [[nodiscard]] virtual rmcs_description::OdomImu::Position Predict(double sec) = 0;
};
} // namespace rmcs_auto_aim::tracker2