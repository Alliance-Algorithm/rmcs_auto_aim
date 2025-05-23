#pragma once
#include "./fire_controller.hpp"
#include "core/tracker/outpost/outpost_tracker.hpp"
#include <memory>

namespace rmcs_auto_aim::fire_controller {
class OutPostController final : public tracker::IFireController {
public:
    [[nodiscard]] std::tuple<bool, rmcs_description::OdomImu::Position>
        UpdateController(double sec, const rmcs_description::Tf&) final;

    void SetTracker(const std::shared_ptr<tracker::OutPostTracker>& tracker);
    double get_omega() final;
    bool check();

    std::chrono::steady_clock::time_point get_timestamp() final;

    OutPostController();

    OutPostController(const OutPostController&);

    ~OutPostController();

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};
} // namespace rmcs_auto_aim::fire_controller