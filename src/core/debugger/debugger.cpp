#include <cstdlib>
#include <memory>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/parameter_event_handler.hpp>
#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <rmcs_msgs/msg/robot_pose.hpp>

#include "core/pnpsolver/armor/armor3d.hpp"
#include "debugger.hpp"

using namespace rmcs_auto_aim;

class Debugger::Impl : public rclcpp::Node {
public:
    Impl()
        : rclcpp::Node("debugger") {
        img_pub_        = this->create_publisher<sensor_msgs::msg::Image>("/raw_img", 10);
        pose_pub_       = this->create_publisher<rmcs_msgs::msg::RobotPose>("/armor_pose", 10);
        pnp_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/marker", 10);
        marker_pub_ =
            this->create_publisher<visualization_msgs::msg::MarkerArray>("/armor_plate_array", 10);

        parameter_subscriber_ = std::make_unique<rclcpp::ParameterEventHandler>(this);

        parameter_callbacks_.push_back(parameter_subscriber_->add_parameter_callback(
            "exposure_time",
            [this](const rclcpp::Parameter& para) {
                RCLCPP_INFO(get_logger(), "exposure -> %f", para.as_double());
            },
            "rmcs_auto_aim"));
    }

    void publish_raw_image(const cv::Mat& img, const rclcpp::Time& stamp) {
        sensor_msgs::msg::Image msg;
        std_msgs::msg::Header header;
        cv_bridge::CvImage bridge;
        header.stamp = stamp;
        bridge       = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, img);
        bridge.toImageMsg(msg);

        img_pub_->publish(msg);
    }

    void publish_pnp_armor(const ArmorPlate3dWithoutFrame& armor) {
        rmcs_msgs::msg::RobotPose msg;
        msg.id   = static_cast<int64_t>(armor.id);
        msg.pose = armor.pose;
        pose_pub_->publish(msg);

        auto marker               = visualization_msgs::msg::Marker();
        marker.header.frame_id    = "odom_imu";
        marker.header.stamp       = this->get_clock()->now();
        marker.ns                 = "basic_shapes";
        marker.id                 = 0;
        marker.type               = visualization_msgs::msg::Marker::CUBE;
        marker.action             = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x    = armor.pose.position.x;
        marker.pose.position.y    = armor.pose.position.y;
        marker.pose.position.z    = armor.pose.position.z;
        marker.pose.orientation.x = armor.pose.orientation.x;
        marker.pose.orientation.y = armor.pose.orientation.y;
        marker.pose.orientation.z = armor.pose.orientation.z;
        marker.pose.orientation.w = armor.pose.orientation.w;
        marker.scale.x            = 1.0;
        marker.scale.y            = 1.0;
        marker.scale.z            = 0.1; // 厚度非常薄
        marker.color.a            = 1.0; // 不要忘记设置alpha
        marker.color.r            = 0.0;
        marker.color.g            = 1.0;
        marker.color.b            = 0.0;

        pnp_marker_pub_->publish(marker);
    }

    void publish_armors(const visualization_msgs::msg::MarkerArray& msg) {
        marker_pub_->publish(msg);
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub_;
    rclcpp::Publisher<rmcs_msgs::msg::RobotPose>::SharedPtr pose_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pnp_marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    static inline int ros_marker_id_global_ = 0;

    std::unique_ptr<rclcpp::ParameterEventHandler> parameter_subscriber_;
    std::vector<std::shared_ptr<rclcpp::ParameterCallbackHandle>> parameter_callbacks_;

    float color_r = (float)((double)random() / ((double)RAND_MAX + 1));
    float color_g = (float)((double)random() / ((double)RAND_MAX + 1));
    float color_b = (float)((double)random() / ((double)RAND_MAX + 1));
};

Debugger::Debugger()
    : pImpl_(std::make_unique<Impl>()) {}

rclcpp::Node::SharedPtr Debugger::getNode() { return pImpl_; }

void Debugger::publish_raw_image(const cv::Mat& img, const rclcpp::Time& stamp) {
    return pImpl_->publish_raw_image(img, stamp);
}

void Debugger::publish_pnp_armor(const ArmorPlate3dWithoutFrame& armor) {
    return pImpl_->publish_pnp_armor(armor);
}

void Debugger::publish_armors(const visualization_msgs::msg::MarkerArray& msg) {
    return pImpl_->publish_armors(msg);
}
