#pragma once

#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <memory>
#include <mutex>
#include <pthread.h>
#include <queue>
#include <string>
#include <thread>
#include <unistd.h>
#include <utility>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cv_bridge/cv_bridge.h>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/msg/pose.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/utilities.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/int8.hpp>

#include <fast_tf/impl/cast.hpp>
#include <hikcamera/image_capturer.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/game_stage.hpp>
#include <rmcs_msgs/keyboard.hpp>
#include <rmcs_msgs/msg/robot_pose.hpp>
#include <rmcs_msgs/robot_color.hpp>
#include <rmcs_msgs/robot_id.hpp>

#include "core/debugger/debugger.hpp"
#include "core/recorder/recorder.hpp"
#include "core/tracker/target.hpp"
#include "core/trajectory/trajectory_solvor.hpp"

namespace rmcs_auto_aim {
using std::placeholders::_1;

class Controller
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    Controller()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {

        register_input("/predefined/update_count", update_count_);
        register_input("/tf", tf_);
        register_input("/referee/id", robot_msg_);
        register_input("/remote/keyboard", keyboard_);
        register_input("/referee/game/stage", stage_);

        register_output("/auto_aim/hero/position", enemies_hero_pose_);
        register_output("/auto_aim/engineer/position", enemies_engineer_pose_);
        register_output("/auto_aim/infantry_iii/position", enemies_infantry_iii_pose_);
        register_output("/auto_aim/infantry_iv/position", enemies_infantry_iv_pose_);
        register_output("/auto_aim/infantry_v/position", enemies_infantry_v_pose_);
        register_output("/auto_aim/sentry/position", enemies_sentry_pose_);
        register_output(
            "/gimbal/auto_aim/control_direction", control_direction_, Eigen::Vector3d::Zero());
        register_output("/auto_aim/ui_target", ui_target_, std::make_pair(0, 0));

        exposure_time_           = get_parameter("exposure_time").as_int();
        armor_predict_duration_  = get_parameter("armor_predict_duration").as_int();
        buff_predict_duration_   = get_parameter("buff_predict_duration").as_int();
        yaw_error_               = get_parameter("yaw_error").as_double();
        pitch_error_             = get_parameter("pitch_error").as_double();
        gimbal_predict_duration_ = get_parameter("gimbal_predict_duration").as_int();
        armor_model_path_        = get_parameter("armor_model_path").as_string();
        buff_model_path_         = get_parameter("buff_model_path").as_string();
        debug_mode_              = get_parameter("debug").as_bool();
        fx_                      = get_parameter("fx").as_double();
        fy_                      = get_parameter("fy").as_double();
        cx_                      = get_parameter("cx").as_double();
        cy_                      = get_parameter("cy").as_double();
        k1_                      = get_parameter("k1").as_double();
        k2_                      = get_parameter("k2").as_double();
        k3_                      = get_parameter("k3").as_double();
        record_mode_             = get_parameter("record").as_bool();
        raw_img_pub_mode_        = get_parameter("raw_img_pub").as_bool();
        shoot_velocity_          = get_parameter("shoot_velocity").as_double();
        predict_sec_             = get_parameter("predict_sec").as_double();

        try {
            record_fps_      = get_parameter("record_fps").as_int();
            debug_robot_id_  = get_parameter("debug_robot_id").as_int();
            debug_buff_mode_ = get_parameter("debug_buff_mode").as_bool();
            debug_color_     = get_parameter("debug_color").as_int();
        } catch (rclcpp::exceptions::InvalidParametersException& e) {
            RCLCPP_WARN(get_logger(), "Failed to read parameter: %s", e.what());
        }

        non_target_sub_ = this->create_subscription<std_msgs::msg::Int8>(
            "/sentry/lock_permit", 10,
            [this](std_msgs::msg::Int8::UniquePtr msg) { blacklist_update(std::move(msg)); });
    }

    ~Controller() {
        for (auto& thread : threads_) {
            if (thread.joinable()) {
                thread.join();
            }
        }
    }

    void update() override;

    void blacklist_update(std_msgs::msg::Int8::UniquePtr msg) {
        blacklist_.store(msg->data);
        RCLCPP_INFO(get_logger(), "%d", blacklist_.load());
    }

private:
    void communicate(
        const rmcs_msgs::ArmorID& id, const rmcs_description::OdomImu::Position& pos, uint8_t& ids);
    class FPSCounter {
    public:
        bool Count() {
            if (_count == 0) {
                _count       = 1;
                _timingStart = std::chrono::steady_clock::now();
            } else {
                ++_count;
                if (std::chrono::steady_clock::now() - _timingStart >= std::chrono::seconds(1)) {
                    _lastFPS = _count;
                    _count   = 0;
                    return true;
                }
            }
            return false;
        }

        int GetFPS() const { return _lastFPS; }

    private:
        int _count = 0, _lastFPS;
        std::chrono::steady_clock::time_point _timingStart;
    };

    void gimbal_process();

    template <typename Link>
    void omni_perception_process(const std::string& device);

    // Parameters of omni-direction perception
    double omni_fx, omni_fy, omni_cx, omni_cy, omni_k1, omni_k2, omni_k3;
    double omni_exposure_;

    OutputInterface<Eigen::Vector2d> enemies_infantry_iii_pose_;
    OutputInterface<Eigen::Vector2d> enemies_infantry_iv_pose_;
    OutputInterface<Eigen::Vector2d> enemies_infantry_v_pose_;
    OutputInterface<Eigen::Vector2d> enemies_engineer_pose_;
    OutputInterface<Eigen::Vector2d> enemies_sentry_pose_;
    OutputInterface<Eigen::Vector2d> enemies_hero_pose_;

    // Parameters of auto-aim
    double fx_, fy_, cx_, cy_, k1_, k2_, k3_;
    double pitch_error_;
    double yaw_error_;

    int64_t gimbal_predict_duration_;
    int64_t armor_predict_duration_;
    int64_t buff_predict_duration_;
    int64_t exposure_time_;
    int64_t record_fps_;

    std::string armor_model_path_;
    std::string buff_model_path_;

    std::atomic<uint8_t> blacklist_{0x3f};

    std::chrono::steady_clock::time_point timestamp_;

    std::shared_ptr<TargetInterface> target_{nullptr};
    rmcs_description::OdomImu::Position pnp_result_;
    OutputInterface<Eigen::Vector3d> control_direction_;

    TrajectorySolver trajectory_{};
    Recorder recorder_;

    // Parameters in debug mode
    bool debug_buff_mode_;
    bool record_mode_;
    bool debug_mode_;

    uint8_t debug_robot_id_;
    uint8_t debug_color_;

    // TF related
    InputInterface<rmcs_description::Tf> tf_;

    // Recorder related
    std::queue<std::pair<rclcpp::Time, std::shared_ptr<cv::Mat>>> image_queue_;
    std::condition_variable img_cv_;
    std::mutex img_mtx_;

    // Topic related
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr non_target_sub_;

    // IntputInterfaces
    InputInterface<rmcs_msgs::RobotId> robot_msg_;
    InputInterface<rmcs_msgs::Keyboard> keyboard_;
    InputInterface<rmcs_msgs::GameStage> stage_;
    InputInterface<size_t> update_count_;

    Debugger& debugger_ = Debugger::getInstance();
    bool raw_img_pub_mode_;

    std::vector<std::thread> threads_;

    OutputInterface<std::pair<uint16_t, uint16_t>> ui_target_;

    double shoot_velocity_;
    double predict_sec_;
};
} // namespace rmcs_auto_aim

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_auto_aim::Controller, rmcs_executor::Component)