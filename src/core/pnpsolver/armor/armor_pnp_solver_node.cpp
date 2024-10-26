#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

namespace rmcs_auto_aim {
class ArmorPnpSolverNode
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    explicit ArmorPnpSolverNode()
        : rclcpp::Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {}

    void update() override {}

private:
};
} // namespace rmcs_auto_aim

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_auto_aim::ArmorPnpSolverNode, rmcs_executor::Component)