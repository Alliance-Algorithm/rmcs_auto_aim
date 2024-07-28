
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <pthread.h>
#include <queue>
#include <std_msgs/msg/detail/int8__struct.hpp>
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

#include "core/identifier/armor/armor_identifier.hpp"
#include "core/identifier/buff/buff_identifier.hpp"
#include "core/pnpsolver/armor/armor_pnp_solver.hpp"
#include "core/pnpsolver/buff/buff_pnp_solver.hpp"
#include "core/recorder/recorder.hpp"
#include "core/tracker/armor/armor_tracker.hpp"
#include "core/tracker/buff/buff_tracker.hpp"
#include "core/tracker/target.hpp"
#include "core/trajectory/trajectory_solvor.hpp"

namespace rmcs_auto_aim {

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

        register_output("/referee/enemies/hero/position", enemies_hero_pose_);
        register_output("/referee/enemies/engineer/position", enemies_engineer_pose_);
        register_output("/referee/enemies/infantry_iii/position", enemies_infantry_iii_pose_);
        register_output("/referee/enemies/infantry_iv/position", enemies_infantry_iv_pose_);
        register_output("/referee/enemies/infantry_v/position", enemies_infantry_v_pose_);
        register_output("/referee/enemies/sentry/position", enemies_sentry_pose_);
        register_output(
            "/gimbal/auto_aim/control_direction", control_direction_, Eigen::Vector3d::Zero());

        exposure_time_           = get_parameter("exposure_time").as_int();
        armor_predict_duration_  = get_parameter("armor_predict_duration").as_int();
        buff_predict_duration_   = get_parameter("buff_predict_duration").as_int();
        yaw_error_               = get_parameter("yaw_error").as_double();
        pitch_error_             = get_parameter("pitch_error").as_double();
        gimbal_predict_duration_ = get_parameter("gimbal_predict_duration").as_int();
        armor_model_path_        = get_parameter("armor_model_path").as_string();
        buff_model_path_         = get_parameter("buff_model_path").as_string();
        debug_                   = get_parameter("debug").as_bool();
        fx                       = get_parameter("fx").as_double();
        fy                       = get_parameter("fy").as_double();
        cx                       = get_parameter("cx").as_double();
        cy                       = get_parameter("cy").as_double();
        k1                       = get_parameter("k1").as_double();
        k2                       = get_parameter("k2").as_double();
        k3                       = get_parameter("k3").as_double();
        record_                  = get_parameter("record").as_bool();

        try {
            record_fps_      = get_parameter("record_fps").as_int();
            debug_robot_id_  = get_parameter("debug_robot_id").as_int();
            debug_buff_mode_ = get_parameter("debug_buff_mode").as_bool();
            debug_color_     = get_parameter("debug_color").as_int();
        } catch (rclcpp::exceptions::InvalidParametersException& e) {
            RCLCPP_WARN(get_logger(), "Failed to read parameter: %s", e.what());
        }

        if ((debug_ && debug_robot_id_ == 7)
            || (!debug_ && robot_msg_->id() == rmcs_msgs::ArmorID::Sentry)) {
            try {
                omni_exposure_ = get_parameter("omni_exposure").as_double();
                omni_cx        = get_parameter("omni_cx").as_double();
                omni_cy        = get_parameter("omni_cy").as_double();
                omni_fx        = get_parameter("omni_fx").as_double();
                omni_fy        = get_parameter("omni_fy").as_double();
                omni_k1        = get_parameter("omni_k1").as_double();
                omni_k2        = get_parameter("omni_k2").as_double();
                omni_k3        = get_parameter("omni_k3").as_double();

            } catch (rclcpp::exceptions::InvalidParametersException& e) {
                RCLCPP_WARN(get_logger(), "Failed to read parameter: %s", e.what());
            }
        }

        pose_pub_ = this->create_publisher<rmcs_msgs::msg::RobotPose>("/armor_pose", 10);
        non_identifier_target_sub_ = this->create_subscription<std_msgs::msg::Int8>(
            "/sentry/lock_premit", 10,
            [this](auto&& PH1) { blacklist_update(std::forward<decltype(PH1)>(PH1)); });
    }

    ~Controller() {
        for (auto& thread : threads_) {
            if (thread.joinable()) {
                thread.join();
            }
        }
    }

    void update() override {
        if (*update_count_ == 0) {
            threads_.emplace_back([this]() {
                size_t attempt = 0;
                while (true) {
                    try {
                        gimbal_process();
                    } catch (std::exception& e) {
                        attempt++;
                        if (attempt < 10) {
                            RCLCPP_FATAL(get_logger(), "Gimbal Error: %s", e.what());
                        } else if (attempt == 10) {
                            RCLCPP_WARN(get_logger(), "Too many error. Diabled log...");
                        }
                    }
                    sleep(5);
                }
            });
            if (record_) {
                threads_.emplace_back([this]() {
                    if (recorder.is_opened()) {
                        RCLCPP_INFO(get_logger(), "RECORDING %s...", recorder.get_filename().c_str());
                    }
                    while (rclcpp::ok()) {
                        if (!recorder.is_opened()) {
                            continue;
                        }
                        std::unique_lock<std::mutex> lock(img_mtx_);
                        img_cv_.wait(lock, [this] { return !image_queue_.empty(); });
                        std::shared_ptr<cv::Mat> imgPtr = image_queue_.front();
                        image_queue_.pop();
                        lock.unlock();

                        cv::Mat img = *imgPtr;
                        recorder.record_frame(img);
                    }
                });
            }
            if (robot_msg_->id() == rmcs_msgs::ArmorID::Sentry) {
                threads_.emplace_back([this]() {
                    omni_perception_process<rmcs_description::OmniLinkLeftFront>("/dev/leftfront");
                });
                threads_.emplace_back([this]() {
                    omni_perception_process<rmcs_description::OmniLinkRightFront>(
                        "/dev/rightfront");
                });
                threads_.emplace_back([this]() {
                    omni_perception_process<rmcs_description::OmniLinkLeft>("/dev/left");
                });
                threads_.emplace_back([this]() {
                    omni_perception_process<rmcs_description::OmniLinkRight>("/dev/right");
                });
            }
        }

        auto local_target = target_.release();
        if (!local_target) {
            *control_direction_ = Eigen::Vector3d::Zero();
            return;
        }
        using namespace std::chrono_literals;
        auto diff = std::chrono::steady_clock::now() - timestamp_;
        if (diff > std::chrono::milliseconds(gimbal_predict_duration_)) {
            *control_direction_ = Eigen::Vector3d::Zero();
            return;
        }

        auto offset = fast_tf::cast<rmcs_description::OdomImu>(
            rmcs_description::MuzzleLink::Position{0, 0, 0}, *tf_);

        double fly_time = 0;
        for (int i = 5; i-- > 0;) {
            auto pos = local_target->Predict(
                static_cast<std::chrono::duration<double>>(diff).count() + fly_time + 0.05);
            auto aiming_direction = *trajectory_.GetShotVector(
                {pos->x() - offset->x(), pos->y() - offset->y(), pos->z() - offset->z()}, 27.0,
                fly_time);
            // auto pos = pnp_result_;
            // auto aiming_direction = *trajectory_.GetShotVector(pos, 27.0, fly_time);

            auto yaw_axis = fast_tf::cast<rmcs_description::PitchLink>(
                                rmcs_description::OdomImu::DirectionVector(0, 0, 1), *tf_)
                                ->normalized();
            auto pitch_axis = fast_tf::cast<rmcs_description::PitchLink>(
                                  rmcs_description::OdomImu::DirectionVector(0, 1, 0), *tf_)
                                  ->normalized();
            auto delta_yaw   = Eigen::AngleAxisd{yaw_error_, yaw_axis};
            auto delta_pitch = Eigen::AngleAxisd{pitch_error_, pitch_axis};
            aiming_direction = delta_pitch * (delta_yaw * (aiming_direction));

            if (i == 0) {
                *control_direction_ = aiming_direction;
                break;
            }
        }

        if (target_updated_.load()) {
            delete local_target;
            target_updated_.store(false);
        } else {
            target_.reset(local_target);
        }
    }

    void blacklist_update(const std_msgs::msg::Int8::SharedPtr& msg) {
        blacklist.store(msg->data);
        RCLCPP_INFO(get_logger(), "%d", blacklist.load());
    }

private:
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

    void gimbal_process() {

        hikcamera::ImageCapturer::CameraProfile camera_profile;
        camera_profile.exposure_time = std::chrono::milliseconds(exposure_time_);
        camera_profile.gain          = 16.9807;
        if ((debug_ ? debug_robot_id_ == 7 : robot_msg_->id() == rmcs_msgs::ArmorID::Sentry)) {
            camera_profile.invert_image = true;
        } else {
            camera_profile.invert_image = false;
        }

        hikcamera::ImageCapturer img_capture(camera_profile);

        auto package_share_directory =
            ament_index_cpp::get_package_share_directory("rmcs_auto_aim");

        auto armor_identifier = ArmorIdentifier(package_share_directory + armor_model_path_);
        auto buff_identifier  = BuffIdentifier(package_share_directory + buff_model_path_);

        auto armor_tracker = ArmorTracker(armor_predict_duration_, debug_);
        auto buff_tracker  = BuffTracker(buff_predict_duration_);

        auto buff_enabled = false;

        auto my_color =
            debug_ ? static_cast<rmcs_msgs::RobotColor>(debug_color_) : robot_msg_->color();

        auto target_color = static_cast<rmcs_msgs::RobotColor>(1 - static_cast<uint8_t>(my_color));

        FPSCounter fps;

        if (record_) {
            auto flag = true;
            recorder.setParam(static_cast<double>(record_fps_), [&img_capture, this, &flag]() {
                auto i   = 0;
                auto img = img_capture.read();
                while (i < 5) {
                    if (!img.empty()) {
                        break;
                    }
                    img = img_capture.read();
                    i++;
                }
                if (i == 5) {
                    RCLCPP_FATAL(get_logger(), "Failed to sample image size.");
                    flag = false;
                    return cv::Size(1, 1);
                }
                return cv::Size(img.cols, img.rows);
            }());

            if (!flag || !recorder.is_opened()) {
                RCLCPP_WARN(get_logger(), "Failed to open an VideoWriter.");
                record_ = false;
            }
        }

        while (rclcpp::ok()) {
            if (!debug_ && *stage_ == rmcs_msgs::GameStage::SETTLING) {
                continue;
            }

            auto img       = img_capture.read();
            auto timestamp = std::chrono::steady_clock::now();

            auto tf = *tf_;

            do {
                if (!buff_enabled && (debug_ ? debug_buff_mode_ : keyboard_->g == 1)) {

                    buff_tracker.ResetAll(tf);
                }
                buff_enabled = (debug_ ? debug_buff_mode_ : keyboard_->g == 1);

                if (!buff_enabled) {
                    auto armors = armor_identifier.Identify(img, target_color, blacklist.load());

                    auto armor3d = ArmorPnPSolver::SolveAll(armors, tf, fx, fy, cx, cy, k1, k2, k3);

                    if (auto target = armor_tracker.Update(armor3d, timestamp, tf)) {
                        timestamp_ = timestamp;
                        target_.swap(target);
                        target_updated_.store(true);
                        break;
                    }

                } else {
                    if (auto buff = buff_identifier.Identify(img)) {
                        if (auto buff3d =
                                BuffPnPSolver::Solve(*buff, tf, fx, fy, cx, cy, k1, k2, k3)) {
                            if (auto target = buff_tracker.Update(*buff3d, timestamp)) {
                                timestamp_ = timestamp;
                                target_.swap(target);
                                target_updated_.store(true);
                                break;
                            }
                        }
                    }
                }
            } while (false);

            if (record_ && (debug_ || *stage_ == rmcs_msgs::GameStage::STARTED)
                && recorder.is_opened() && !img.empty()) {

                std::shared_ptr<cv::Mat> imgPtr = std::make_shared<cv::Mat>(img);
                std::unique_lock<std::mutex> lock(img_mtx_);
                image_queue_.push(imgPtr);
                lock.unlock();
                img_cv_.notify_one();
            }

            if (fps.Count()) {
                RCLCPP_INFO(get_logger(), "Fps:%d", fps.GetFPS());
            }
        } // while rclcpp::ok end
    }

    template <typename Link>
    void omni_perception_process(const std::string& device) {

        // // omni dir
        auto camera = cv::VideoCapture(device);

        if (!camera.isOpened()) {
            RCLCPP_WARN(get_logger(), "Failed to open camera!");
            return;
        }
        camera.set(cv::CAP_PROP_EXPOSURE, omni_exposure_);

        auto package_share_directory =
            ament_index_cpp::get_package_share_directory("rmcs_auto_aim");

        auto armor_identifier = ArmorIdentifier(package_share_directory + armor_model_path_);

        auto my_color =
            debug_ ? static_cast<rmcs_msgs::RobotColor>(debug_color_) : robot_msg_->color();
        auto target_color = static_cast<rmcs_msgs::RobotColor>(1 - static_cast<uint8_t>(my_color));

        cv::Mat img;
        while (camera.isOpened()) {
            camera >> img;
            if (img.empty()) {
                continue;
            }
            auto armors = armor_identifier.Identify(img, target_color, blacklist.load());

            std::map<rmcs_msgs::ArmorID, typename Link::Position> targets_map;

            for (auto& armor : armors) {
                auto pnp_result = ArmorPnPSolver::Solve(
                    armor, omni_fx, omni_fy, omni_cx, omni_cy, omni_k1, omni_k2, omni_k3);
                typename Link::Position pos{
                    pnp_result.pose.position.x, pnp_result.pose.position.y,
                    pnp_result.pose.position.z};
                targets_map.insert(std::make_pair(pnp_result.id, pos));
            }

            for (auto& [id, target] : targets_map) {
                auto pos = fast_tf::cast<rmcs_description::OdomImu>(target, *tf_);
                Eigen::Vector2d plate_pos{pos->x(), pos->y()};
                switch (id) {
                case rmcs_msgs::ArmorID::Hero: {
                    *enemies_hero_pose_ = plate_pos;
                    break;
                }
                case rmcs_msgs::ArmorID::Engineer: {
                    *enemies_engineer_pose_ = plate_pos;
                    break;
                }
                case rmcs_msgs::ArmorID::InfantryIII: {
                    *enemies_infantry_iii_pose_ = plate_pos;
                    break;
                }
                case rmcs_msgs::ArmorID::InfantryIV: {
                    *enemies_infantry_iv_pose_ = plate_pos;
                    break;
                }
                case rmcs_msgs::ArmorID::InfantryV: {
                    *enemies_infantry_v_pose_ = plate_pos;
                    break;
                }
                case rmcs_msgs::ArmorID::Sentry: {
                    *enemies_sentry_pose_ = plate_pos;
                    break;
                }
                default: break;
                }
            }
        }
    }

    double omni_fx, omni_fy, omni_cx, omni_cy, omni_k1, omni_k2, omni_k3;
    double fx, fy, cx, cy, k1, k2, k3;
    double omni_exposure_;
    double pitch_error_;
    double yaw_error_;
    bool debug_;
    bool record_;

    int64_t gimbal_predict_duration_;
    int64_t armor_predict_duration_;
    int64_t buff_predict_duration_;
    uint8_t debug_robot_id_;
    int64_t exposure_time_;
    uint8_t debug_color_;
    int64_t record_fps_;
    bool debug_buff_mode_;

    std::atomic<int8_t> blacklist;

    std::unique_ptr<TargetInterface> target_{nullptr};
    std::atomic<bool> target_updated_{false};

    std::chrono::steady_clock::time_point timestamp_;
    std::queue<std::shared_ptr<cv::Mat>> image_queue_;
    std::vector<std::thread> threads_;
    std::string armor_model_path_;
    std::string buff_model_path_;
    std::condition_variable img_cv_;
    std::mutex img_mtx_;

    rclcpp::Publisher<rmcs_msgs::msg::RobotPose>::SharedPtr pose_pub_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr non_identifier_target_sub_;

    OutputInterface<Eigen::Vector2d> enemies_infantry_iii_pose_;
    OutputInterface<Eigen::Vector2d> enemies_infantry_iv_pose_;
    OutputInterface<Eigen::Vector2d> enemies_infantry_v_pose_;
    OutputInterface<Eigen::Vector2d> enemies_engineer_pose_;
    OutputInterface<Eigen::Vector2d> enemies_sentry_pose_;
    OutputInterface<Eigen::Vector2d> enemies_hero_pose_;
    OutputInterface<Eigen::Vector3d> control_direction_;

    InputInterface<rmcs_msgs::RobotId> robot_msg_;
    InputInterface<rmcs_msgs::Keyboard> keyboard_;
    InputInterface<rmcs_msgs::GameStage> stage_;
    InputInterface<rmcs_description::Tf> tf_;
    InputInterface<size_t> update_count_;

    rmcs_description::OdomImu::Position pnp_result_;

    TrajectorySolver trajectory_{};
    Recorder recorder;
};
} // namespace rmcs_auto_aim
  // namespace rmcs_auto_aim

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_auto_aim::Controller, rmcs_executor::Component)