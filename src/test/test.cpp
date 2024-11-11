
// #include "util/fps_counter.hpp"
#include <hikcamera/image_capturer.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/utilities.hpp>

#include <rmcs_executor/component.hpp>
#include <robot_color.hpp>

namespace rmcs_auto_aim {
class AutoAimTest
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    AutoAimTest()
        : rclcpp::Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        // register_output("/auto_aim/target_color", target_color_);
        // register_output("/auto_aim/blacklist", blacklist_);
        // register_input("/auto_aim/camera", img_);
        RCLCPP_INFO(get_logger(), "test init");
    }

    void update() override {
        // if (rclcpp::ok() && !img_->empty()) {
        //     // cv::imshow("test", *img_);
        //     // cv::waitKey();
        //     if (fps_counter_.Count()) {
        //         RCLCPP_INFO(get_logger(), "fps: %d", fps_counter_.GetFPS());
        //     }
        // }
        hikcamera::ImageCapturer::CameraProfile profile;
        RCLCPP_INFO(get_logger(), "%f", profile.exposure_time.count());
    }

private:
    // InputInterface<cv::Mat> img_;
    // FPSCounter fps_counter_;
};
} // namespace rmcs_auto_aim

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_auto_aim::AutoAimTest, rmcs_executor::Component)