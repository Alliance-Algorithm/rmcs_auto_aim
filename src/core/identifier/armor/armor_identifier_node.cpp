#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv2/core/mat.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/msg/armor_plate_array.hpp>
#include <robot_color.hpp>

#include "core/capturer/frame.h"
#include "core/identifier/armor/armor.hpp"
#include "core/identifier/armor/armor_identifier.hpp"

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
        register_input("/auto_aim/whitelist", blacklist_);

        register_output("/auto_aim/armor_plates", armor_plates_);

        armor_identifier_ = std::make_unique<ArmorIdentifier>(
            ament_index_cpp::get_package_share_directory("rmcs_auto_aim")
            + get_parameter("armor_model_path").as_string());

        publish_topic = get_parameter("publish_topic").as_bool();

        armor_plates_pub_ = create_publisher<rmcs_msgs::msg::ArmorPlateArray>(
            get_parameter("topic_name").as_string(), 10);

        RCLCPP_INFO(get_logger(), "Armor Identifier Node Initialized");
    }

    void update() override {
        if (frame_->frame_id != last_frame_id_) {
            auto result    = armor_identifier_->Identify(frame_->img, *target_color_, *blacklist_);
            *armor_plates_ = result;
            if (publish_topic) {
                rmcs_msgs::msg::ArmorPlateArray msg;
                for (auto& plate : result) {
                    msg.armor_plate_array.push_back((rmcs_msgs::msg::ArmorPlate)plate);
                }
                armor_plates_pub_->publish(msg);
            }
            last_frame_id_ = frame_->frame_id;
        }
    }

private:
    bool publish_topic{false};
    size_t last_frame_id_{0};

    std::unique_ptr<ArmorIdentifier> armor_identifier_;

    rclcpp::Publisher<rmcs_msgs::msg::ArmorPlateArray>::SharedPtr armor_plates_pub_;

    InputInterface<struct rmcs_auto_aim::Frame> frame_;
    InputInterface<size_t> update_count_;
    InputInterface<uint8_t> blacklist_;
    InputInterface<rmcs_msgs::RobotColor> target_color_;

    OutputInterface<std::vector<ArmorPlate>> armor_plates_;
};
} // namespace rmcs_auto_aim

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_auto_aim::ArmorIdentifierNode, rmcs_executor::Component)