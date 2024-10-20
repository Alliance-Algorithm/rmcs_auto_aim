#include <chrono>
#include <hikcamera/image_capturer.hpp>
#include <memory>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <thread>
namespace rmcs_auto_aim {
class Capturer
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    Capturer()
        : rclcpp::Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        RCLCPP_INFO(this->get_logger(), "Capturer init");

        register_input("/predefined/update_count", update_count_);

        hikcamera::ImageCapturer::CameraProfile profile;
        profile.invert_image  = get_parameter("invert_image").as_bool();
        profile.gain          = 16.9807;
        profile.exposure_time = std::chrono::milliseconds(get_parameter("exposure_time").as_int());

        capturer_ = std::make_unique<hikcamera::ImageCapturer>(profile);
    }

    ~Capturer() {
        if (thread_.joinable()) {
            thread_.join();
        }
    }

    void update() override {
        if (*update_count_ == 0) {
            thread_ = std::thread([this] {
                while (true) {
                    auto img = capturer_->read();
                }
            });
        }
    }

private:
    std::thread thread_;
    InputInterface<size_t> update_count_;
    std::unique_ptr<hikcamera::ImageCapturer> capturer_;
};
} // namespace rmcs_auto_aim

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_auto_aim::Capturer, rmcs_executor::Component);