#include <chrono>
#include <cstddef>
#include <memory>
#include <opencv2/videoio.hpp>
#include <thread>

#include <opencv2/core/mat.hpp>
#include <rclcpp/node.hpp>

#include <hikcamera/image_capturer.hpp>
#include <rmcs_executor/component.hpp>

#include "frame.h"

namespace rmcs_auto_aim {

class Capturer
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    Capturer()
        : rclcpp::Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {

        register_input("/predefined/update_count", update_count_);
        register_output("/auto_aim/camera", frame_);

        mode_ = CapturerMode{get_parameter("use_video").as_bool()};

        if (mode_ == CapturerMode::VideoMode) {
            RCLCPP_INFO(this->get_logger(), "Video mode is on.");

            video_capture_ = std::make_unique<cv::VideoCapture>(
                get_parameter("video_path").as_string(), cv::CAP_FFMPEG);

            if (!video_capture_->isOpened()) {
                RCLCPP_ERROR(this->get_logger(), "Video file not found.");
            }
        } else {
            RCLCPP_INFO(get_logger(), "Camera mode is on.");

            hikcamera::ImageCapturer::CameraProfile profile;
            profile.invert_image = get_parameter("invert_image").as_bool();
            profile.gain         = 16.9807;
            profile.exposure_time =
                std::chrono::milliseconds(get_parameter("exposure_time").as_int());

            camera_capturer_ = std::make_unique<hikcamera::ImageCapturer>(profile);
        }
        RCLCPP_INFO(this->get_logger(), "Capturer initialized");
    }

    ~Capturer() {
        if (thread_.joinable()) {
            thread_.join();
        }
        if (video_capture_->isOpened()) {
            video_capture_->release();
        }
    }

    void update() override {
        if (*update_count_ == 0) {
            thread_ = std::thread([this] {
                size_t frame_id = 0;
                while (rclcpp::ok()) {
                    cv::Mat img;

                    if (mode_ == CapturerMode::CameraMode) {
                        img = camera_capturer_->read();
                    } else if (video_capture_->isOpened()) {
                        video_capture_->read(img);
                    }

                    if (!img.empty()) {
                        buffer[!buffer_index_.load()].img      = img;
                        buffer[!buffer_index_.load()].frame_id = ++frame_id;
                        buffer_index_.store(!buffer_index_.load());
                    }
                }
            });
        }
        *frame_ = get_frame();
    }

    struct Frame get_frame() const { return buffer[buffer_index_.load()]; }

private:
    std::atomic<bool> buffer_index_{false};
    std::thread thread_;

    std::unique_ptr<hikcamera::ImageCapturer> camera_capturer_;
    std::unique_ptr<cv::VideoCapture> video_capture_;

    struct Frame buffer[2];

    InputInterface<size_t> update_count_;
    OutputInterface<struct Frame> frame_;

    enum class CapturerMode { CameraMode = false, VideoMode = true } mode_;
};
} // namespace rmcs_auto_aim

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_auto_aim::Capturer, rmcs_executor::Component);