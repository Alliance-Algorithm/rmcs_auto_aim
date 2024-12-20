
#include "core/parameters/parameters.hpp"
#include <rclcpp/rclcpp.hpp>

#include <rmcs_executor/component.hpp>
#include <string>
#include <vector>

namespace rmcs_auto_aim {
class ParametersNode
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    ParametersNode()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        param_names_ = get_parameter("param_names").as_string_array();
    }

    void update() override {
        for (const auto& name : param_names_) {
            if (Parameters::getInstance().getBoolParam(name) != get_parameter(name).as_bool()) {
                RCLCPP_INFO(get_logger(), "param %s changed", name.c_str());
                Parameters::getInstance().setBoolParam(name, get_parameter(name).as_bool());
            }
        }
    }

private:
    std::vector<std::string> param_names_;
};
} // namespace rmcs_auto_aim

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_auto_aim::ParametersNode, rmcs_executor::Component)