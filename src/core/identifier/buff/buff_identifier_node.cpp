#include <cstddef>
#include <memory>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv2/core/mat.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>

#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/msg/buff_plate_array.hpp>
#include <rmcs_msgs/msg/detail/buff_plate_array__struct.hpp>

#include "core/frame.hpp"
#include "core/identifier/buff/buff.hpp"
#include "core/identifier/buff/buff_identifier.hpp"

namespace rmcs_auto_aim {
class BuffIdentifierNode
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    explicit BuffIdentifierNode()
        : rclcpp::Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {

        register_input("/predefined/update_count", update_count_);
        register_input("/auto_aim/camera", frame_);

        register_output("/auto_aim/buff_plates", buff_plates_);

        buff_identifier_ = std::make_unique<BuffIdentifier>(
            ament_index_cpp::get_package_share_directory("rmcs_auto_aim")
            + get_parameter("buff_model_path").as_string());

        publish_topic = get_parameter("publish_topic").as_bool();

        buff_paltes_pub_ = create_publisher<rmcs_msgs::msg::BuffPlateArray>(
            get_parameter("topic_name").as_string(), 10);

        RCLCPP_INFO(get_logger(), "Buff Identifier Node Initialized");
    }

    void update() override {
        if (frame_.ready() && frame_->frame_id_ != last_frame_id_) {
            auto result             = buff_identifier_->Identify(frame_->data_);
            buff_plates_->data_     = result;
            buff_plates_->frame_id_ = frame_->frame_id_;
            if (publish_topic) {
                rmcs_msgs::msg::BuffPlateArray msg;
                if (result) {
                    msg.buff_plate_array.emplace_back((rmcs_msgs::msg::BuffPlate)*result);
                }
                buff_paltes_pub_->publish(msg);
            }
            last_frame_id_ = frame_->frame_id_;
        }
    }

private:
    bool publish_topic{false};
    size_t last_frame_id_{0};

    InputInterface<struct rmcs_auto_aim::Frame<cv::Mat>> frame_;
    InputInterface<size_t> update_count_;

    OutputInterface<struct Frame<std::optional<BuffPlate>>> buff_plates_;

    rclcpp::Publisher<rmcs_msgs::msg::BuffPlateArray>::SharedPtr buff_paltes_pub_;

    std::unique_ptr<BuffIdentifier> buff_identifier_;
};
} // namespace rmcs_auto_aim

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_auto_aim::BuffIdentifierNode, rmcs_executor::Component)