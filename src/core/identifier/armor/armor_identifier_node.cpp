#include "core/identifier/armor/armor.hpp"
#include "core/identifier/armor/armor_identifier.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cstdint>
#include <memory>
#include <opencv2/core/mat.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <sensor_msgs/msg/detail/image__struct.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <rmcs_executor/component.hpp>
#include <robot_color.hpp>
#include <vector>

namespace rmcs_auto_aim {
class ArmorIdentifierNode
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    explicit ArmorIdentifierNode()
        : rclcpp::Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        register_input("/predefined/update_count", update_count_);
        register_input("/auto_aim/target_color", target_color_);
        register_input("/auto_aim/blacklist", blacklist_);

        register_output("/auto_aim/armor_plates", armor_plates_);

        armor_identifier_ = std::make_unique<ArmorIdentifier>(
            ament_index_cpp::get_package_share_directory("rmcs_auto_aim")
            + get_parameter("armor_model_path").as_string());
    }

    void update() override {
        if (!img_->empty()) {
            auto result    = armor_identifier_->Identify(*img_, *target_color_, *blacklist_);
            *armor_plates_ = result;
        }
    }

private:
    // rclcpp::Publisher<>
    InputInterface<cv::Mat> img_;

    InputInterface<rmcs_msgs::RobotColor> target_color_;
    InputInterface<size_t> update_count_;
    InputInterface<uint8_t> blacklist_;

    OutputInterface<std::vector<ArmorPlate>> armor_plates_;

    std::unique_ptr<ArmorIdentifier> armor_identifier_;

    // clang-format off
    // uint8_t blacklist_{
    //     rmcs_auto_aim::blacklist_code::Base         | 
    //     rmcs_auto_aim::blacklist_code::Engineer     | 
    //     rmcs_auto_aim::blacklist_code::Hero         | 
    //     rmcs_auto_aim::blacklist_code::InfantryIII  | 
    //     rmcs_auto_aim::blacklist_code::InfantryIV   | 
    //     rmcs_auto_aim::blacklist_code::InfantryV    | 
    //     rmcs_auto_aim::blacklist_code::Outpost      | 
    //     rmcs_auto_aim::blacklist_code::Sentry       | 
    //     0
    // };
    // clang-format on
};
} // namespace rmcs_auto_aim

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_auto_aim::ArmorIdentifierNode, rmcs_executor::Component)