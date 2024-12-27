#pragma once

#include <utility>
namespace rmcs_auto_aim::transform_optimizer::optimizer {
class Fibonacci {
    [[nodiscard]] inline constexpr static double fibonacci_ratio(int fn, int k, bool first) {
        return first ? (fn - k - 1) / (fn - k + 1) : (fn - k) / (fn - k + 1);
    };

public:
    template <typename Func>
    static double optimizer(double a, double b, double epsilone, Func concav_upward) {
        int k = 1;
        if (a < b)
            std::swap(a, b);
        int fn1 = 2, fn2 = 3;
        double x1 = a + fibonacci_ratio(fn2, k, true) * (b - a),
               x2 = a + fibonacci_ratio(fn2, k, false) * (b - a);
        while ((b - a) > epsilone) {
            k++;
            if (concav_upward(x1) < concav_upward(x2)) {
                b  = x2;
                x2 = x1;
                x1 = a + fibonacci_ratio(fn2, k, true) * (b - a);
            } else {
                a  = x1;
                x1 = x2;
                x2 = a + fibonacci_ratio(fn2, k, false) * (b - a);
            }
            fn2 = fn1 + fn2;
            fn1 = fn2 - fn1;
        }
        return (a + b) / 2;
    }
};
} // namespace rmcs_auto_aim::transform_optimizer::optimizer