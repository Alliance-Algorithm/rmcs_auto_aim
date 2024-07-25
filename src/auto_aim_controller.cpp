
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <map>
#include <string>
#include <thread>
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
#include <rclcpp/utilities.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

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

        register_output("/omni", omni_perception_);
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
            threads_.emplace_back([this]() { gimbal_process(); });
            if (record_) {
                threads_.emplace_back([this]() {
                    RCLCPP_INFO(get_logger(), "RECORDING...");
                    while (rclcpp::ok()) {
                        if (!recorder.is_opened()) {
                            continue;
                        }
                        std::unique_lock<std::mutex> lock(mtx);
                        cv.wait(lock, [this] { return !imageQueue.empty(); });
                        std::shared_ptr<cv::Mat> imgPtr = imageQueue.front();
                        imageQueue.pop();
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

        auto target = target_;
        if (!target_) {
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
            auto pos = target->Predict(
                static_cast<std::chrono::duration<double>>(diff).count() + fly_time + 0.05);
            // auto pos              = pnp_result_;
            auto aiming_direction = *trajectory_.GetShotVector(
                {pos->x() - offset->x(), pos->y() - offset->y(), pos->z() - offset->z()}, 27.0,
                fly_time);
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
            }
        }
    }

private:
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

        auto armor_tracker = ArmorTracker(armor_predict_duration_, debug);
        auto buff_tracker  = BuffTracker(buff_predict_duration_);

        auto buff_enabled = false;
        auto& debugger    = Debugger::getInstance();

        auto my_color =
            debug ? static_cast<rmcs_msgs::RobotColor>(debug_color_) : robot_msg_->color();

        auto target_color = static_cast<rmcs_msgs::RobotColor>(1 - static_cast<uint8_t>(my_color));

        // Recorder recorder(static_cast<double>(fps_), [&img_capture]() {
        //     auto img = img_capture.read();
        //     return cv::Size(img.cols, img.rows);
        // }());

        // if (!recorder.is_opened()) {
        //     RCLCPP_WARN(get_logger(), "Failed to create an VideoWriter().");
        // }

        while (rclcpp::ok()) {
            if (!debug_ && *stage_ == rmcs_msgs::GameStage::SETTLING) {
                continue;
            }

            auto img = img_capture.read();

            if (record_ && (debug_ || *stage_ == rmcs_msgs::GameStage::STARTED)
                && recorder.is_opened() && !img.empty()) {

                std::shared_ptr<cv::Mat> imgPtr = std::make_shared<cv::Mat>(img);
                std::unique_lock<std::mutex> lock(mtx);
                imageQueue.push(imgPtr);
                lock.unlock();
                cv.notify_one();
            }
            auto timestamp = std::chrono::steady_clock::now();

            auto tf = *tf_;

            do {
                if (!buff_enabled && (debug_ ? debug_buff_mode_ : keyboard_->g == 1)) {

                    buff_tracker.ResetAll(tf);
                }
                buff_enabled = (debug_ ? debug_buff_mode_ : keyboard_->g == 1);

                if (!buff_enabled) {
                    auto armors = armor_identifier.Identify(img, target_color);

                    auto armor3d = ArmorPnPSolver::SolveAll(armors, tf, fx, fy, cx, cy, k1, k2, k3);
                    if (debug_) {
                        if (armor3d.size() > 0) {
                            auto sample =
                                ArmorPnPSolver::Solve(armors[0], fx, fy, cx, cy, k1, k2, k3);
                            debugger.publish_pnp_armor(sample);
                            pnp_result_ = rmcs_description::OdomImu::Position{
                                armor3d[0].position->x(), armor3d[0].position->y(),
                                armor3d[0].position->z()};
                        }
                    }

                    if (auto target = armor_tracker.Update(armor3d, timestamp, tf)) {
                        timestamp_ = timestamp;
                        target_    = target.release();
                        break;
                    }

                } else {
                    if (auto buff = buff_identifier.Identify(img)) {
                        if (auto buff3d =
                                BuffPnPSolver::Solve(*buff, tf, fx, fy, cx, cy, k1, k2, k3)) {
                            if (auto target = buff_tracker.Update(*buff3d, timestamp)) {
                                timestamp_ = timestamp;
                                target_    = target.release();
                                break;
                            }
                        }
                    }
                }
            } while (false);
            auto endTime                                = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> frameDuration = endTime - startTime;
            double fps                                  = 1.0 / frameDuration.count();
            RCLCPP_INFO(get_logger(), "fps: %f", fps);
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
            auto armors = armor_identifier.Identify(img, target_color);

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
                *omni_perception_ = std::make_pair(id, plate_pos);
            }
        }
    }
    Recorder recorder;

    std::queue<std::shared_ptr<cv::Mat>> imageQueue;
    std::mutex mtx;
    std::condition_variable cv;

    rclcpp::Publisher<rmcs_msgs::msg::RobotPose>::SharedPtr pose_pub_;

    OutputInterface<std::pair<rmcs_msgs::ArmorID, Eigen::Vector2d>> omni_perception_;

    InputInterface<rmcs_description::Tf> tf_;
    InputInterface<size_t> update_count_;
    InputInterface<rmcs_msgs::RobotId> robot_msg_;
    InputInterface<rmcs_msgs::Keyboard> keyboard_;
    InputInterface<rmcs_msgs::GameStage> stage_;

    uint8_t debug_color_;
    uint8_t debug_robot_id_;
    bool debug_buff_mode_;

    OutputInterface<Eigen::Vector3d> control_direction_;

    std::chrono::steady_clock::time_point timestamp_;
    std::vector<std::thread> threads_;

    int64_t gimbal_predict_duration_;
    int64_t armor_predict_duration_;
    int64_t buff_predict_duration_;
    int64_t exposure_time_;
    int64_t record_fps_;

    TrajectorySolver trajectory_{};
    TargetInterface* target_;

    rmcs_description::OdomImu::Position pnp_result_;

    std::string armor_model_path_;
    std::string buff_model_path_;

    double omni_exposure_;
    double pitch_error_;
    double yaw_error_;
    bool debug_;
    bool record_;

    double fx, fy, cx, cy, k1, k2, k3;
    double omni_fx, omni_fy, omni_cx, omni_cy, omni_k1, omni_k2, omni_k3;
};
} // namespace rmcs_auto_aim
  // namespace rmcs_auto_aim

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_auto_aim::Controller, rmcs_executor::Component)