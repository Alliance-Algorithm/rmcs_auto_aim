#include "./fire_controller.hpp"
#include "core/tracker/car/car_tracker.hpp"
#include <memory>

namespace rmcs_auto_aim::fire_controller {
class OutPostFireController final : public rmcs_auto_aim::tracker::IFireController {
public:
    [[nodiscard]] std::tuple<bool, rmcs_description::OdomImu::Position>
        UpdateController(double sec, const rmcs_description::Tf&) final;

    void SetTracker(const std::shared_ptr<tracker::CarTracker>& tracker) final;
    double get_omega() final;
    bool check();

    void in(const ArmorPlate3d& armor);

    std::chrono::steady_clock::time_point get_timestamp() final;

    OutPostFireController();

    OutPostFireController(const OutPostFireController&);

    ~OutPostFireController();

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};
} // namespace rmcs_auto_aim::fire_controller