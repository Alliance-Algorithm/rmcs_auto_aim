#include <memory>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/parameter_event_handler.hpp>
#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <rmcs_msgs/msg/robot_pose.hpp>

#include "core/pnpsolver/armor/armor3d.hpp"
#include "debugger.hpp"

using namespace rmcs_auto_aim;

class Debugger::Impl : public rclcpp::Node {
public:
    Impl()
        : rclcpp::Node("debugger") {
        img_pub_  = this->create_publisher<sensor_msgs::msg::Image>("/raw_img", 10);
        pose_pub_ = this->create_publisher<rmcs_msgs::msg::RobotPose>("/armor_pose", 10);
        parameter_subscriber_ = std::make_unique<rclcpp::ParameterEventHandler>(this);

        parameter_callbacks_.push_back(parameter_subscriber_->add_parameter_callback(
            "test",
            [this](const rclcpp::Parameter& para) {
                RCLCPP_INFO(get_logger(), "test -> %f", para.as_double());
            },
            "rmcs_auto_aim"));
    }

    void publish_raw_image(const cv::Mat& img) {
        sensor_msgs::msg::Image msg;
        std_msgs::msg::Header header;
        cv_bridge::CvImage bridge;
        header.stamp = this->get_clock()->now();
        bridge       = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, img);
        bridge.toImageMsg(msg);

        img_pub_->publish(msg);
    }

    void publish_pnp_armor(const ArmorPlate3dWithNoFrame& armor) {
        rmcs_msgs::msg::RobotPose msg;
        msg.id   = static_cast<int64_t>(armor.id);
        msg.pose = armor.pose;
        pose_pub_->publish(msg);
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub_;
    rclcpp::Publisher<rmcs_msgs::msg::RobotPose>::SharedPtr pose_pub_;

    std::unique_ptr<rclcpp::ParameterEventHandler> parameter_subscriber_;
    std::vector<std::shared_ptr<rclcpp::ParameterCallbackHandle>> parameter_callbacks_;
};

Debugger::Debugger()
    : pImpl_(std::make_unique<Impl>()) {}

rclcpp::Node::SharedPtr Debugger::getNode() { return pImpl_; }

void Debugger::publish_raw_image(const cv::Mat& img) { return pImpl_->publish_raw_image(img); }

void Debugger::publish_pnp_armor(const ArmorPlate3dWithNoFrame& armor) {
    return pImpl_->publish_pnp_armor(armor);
}
