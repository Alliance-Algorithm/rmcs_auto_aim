#include "core/identifier/armor/armor.hpp"
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <robot_color.hpp>

namespace rmcs_auto_aim {
class AutoAimInitializer
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    AutoAimInitializer()
        : rclcpp::Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        register_output("/auto_aim/target_color", target_color_);
        register_output("/auto_aim/blacklist", blacklist_);
    }

    void update() override {}

private:
    InputInterface<std::vector<ArmorPlate>> armor_plates_;
    OutputInterface<rmcs_msgs::RobotColor> target_color_;
    OutputInterface<uint8_t> blacklist_;
};
} // namespace rmcs_auto_aim

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_auto_aim::AutoAimInitializer, rmcs_executor::Component)