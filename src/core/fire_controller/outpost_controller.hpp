#pragma once
#include "./fire_controller.hpp"
#include "core/tracker/car/car_tracker.hpp"
#include "core/tracker/outpost/outpost_tracker.hpp"
#include <memory>

namespace rmcs_auto_aim::fire_controller {
class OutPostController {
public:
    [[nodiscard]] std::tuple<bool, rmcs_description::OdomImu::Position>
        UpdateController(double sec, const rmcs_description::Tf&);

    void SetTracker(const std::shared_ptr<tracker::OutPostTracker>& tracker);
    double get_omega();
    bool check();

    std::chrono::steady_clock::time_point get_timestamp();

    OutPostController();

    OutPostController(const OutPostController&);

    ~OutPostController();

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};
} // namespace rmcs_auto_aim::fire_controller