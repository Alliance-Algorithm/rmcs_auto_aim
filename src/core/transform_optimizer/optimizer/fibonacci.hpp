#pragma once

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <utility>
namespace rmcs_auto_aim::transform_optimizer::optimizer {
class Fibonacci {

public:
    template <typename Func>
    static double optimizer(double a, double b, double epsilone, Func concav_upward) {
        if (b < a)
            std::swap(a, b);

        double fn1 = 2, fn2 = 3;
        while ((b - a) / epsilone > fn2) {
            fn2 = fn1 + fn2;
            fn1 = fn2 - fn1;
        }

        double x1 = a + (fn2 - fn1) / fn2 * (b - a), x2 = a + fn1 / fn2 * (b - a);
        double cv1 = concav_upward(x1);
        double cv2 = concav_upward(x2);
        while ((b - a) > epsilone) {
            fn1 = fn2 - fn1;
            fn2 = fn2 - fn1;
            if (cv1 < cv2) {
                b   = x2;
                x2  = x1;
                x1  = a + (fn2 - fn1) / fn2 * (b - a);
                cv2 = cv1;
                cv1 = concav_upward(x1);
            } else {
                a   = x1;
                x1  = x2;
                x2  = a + fn1 / fn2 * (b - a);
                cv1 = cv2;
                cv2 = concav_upward(x2);
            }
            // RCLCPP_INFO(rclcpp::get_logger(""), "%lf,%lf", x1, x2);
        }
        return (a + b) / 2;
    }
};
} // namespace rmcs_auto_aim::transform_optimizer::optimizer