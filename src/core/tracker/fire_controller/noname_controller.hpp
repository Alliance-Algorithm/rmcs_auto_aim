#include "core/tracker/car/car_tracker.hpp"
#include "core/tracker/fire_controller.hpp"
#include <memory>

namespace rmcs_auto_aim::fire_controller {
class NoNameController final : public rmcs_auto_aim::tracker::IFireController {
public:
    [[nodiscard]] std::tuple<bool, rmcs_description::OdomImu::Position>
        UpdateController(double sec, const rmcs_description::Tf&) final;

    void SetTracker(const std::shared_ptr<tracker::CarTracker>& tracker) final;
    double get_omega() final;
    bool check();

    NoNameController();

    NoNameController(const NoNameController&);

    ~NoNameController();

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
};
} // namespace rmcs_auto_aim::fire_controller