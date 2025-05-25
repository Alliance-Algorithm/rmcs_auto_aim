#include <cstddef>

#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>

#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/robot_color.hpp>
#include <rmcs_msgs/robot_id.hpp>

#include "core/identifier/armor/armor.hpp"

namespace rmcs_auto_aim {

class AutoAimInitializer
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    AutoAimInitializer()
        : rclcpp::Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        register_input("/predefined/update_count", update_count_);
        register_input("/referee/id", robot_msg_, false);

        register_output("/auto_aim/target_color", target_color_);
        register_output("/auto_aim/whitelist", whitelist_);

        RCLCPP_INFO(this->get_logger(), "AutoAimInitializer initialized.");
    }

    void update() override {
        if (*update_count_ == 0) {
            // clang-format off
            *whitelist_ = // If target is not included in blacklist, it will be ignored.
                0;
            // clang-format on
        }
        if (robot_msg_.ready()) {
            auto my_color = robot_msg_->color();
            if (my_color == rmcs_msgs::RobotColor::RED) {
                *target_color_ = rmcs_msgs::RobotColor::BLUE;
            } else {
                *target_color_ = rmcs_msgs::RobotColor::RED;
            }
        } else {
            // RCLCPP_INFO(this->get_logger(), "Using information from the configuration file.");
            *target_color_ = rmcs_msgs::RobotColor::RED;
        }
    }

private:
    InputInterface<size_t> update_count_;
    InputInterface<rmcs_msgs::RobotId> robot_msg_;

    OutputInterface<rmcs_msgs::RobotColor> target_color_;
    OutputInterface<uint8_t> whitelist_;
};
} // namespace rmcs_auto_aim

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_auto_aim::AutoAimInitializer, rmcs_executor::Component)