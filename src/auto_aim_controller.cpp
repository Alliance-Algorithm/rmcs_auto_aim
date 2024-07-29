
#include <map>
#include <rclcpp/logging.hpp>
#include <utility>

#include "core/identifier/armor/armor_identifier.hpp"
#include "core/identifier/buff/buff_identifier.hpp"
#include "core/pnpsolver/armor/armor_pnp_solver.hpp"
#include "core/pnpsolver/buff/buff_pnp_solver.hpp"
#include "core/tracker/armor/armor_tracker.hpp"
#include "core/tracker/buff/buff_tracker.hpp"

#include "auto_aim_controller.hpp"

using namespace rmcs_auto_aim;

void Controller::gimbal_process() {
    if (*robot_msg_) {
        RCLCPP_INFO(get_logger(), "Robot Info:");
        RCLCPP_INFO(get_logger(), "id: %hu", static_cast<uint16_t>(robot_msg_->id()));
        RCLCPP_INFO(get_logger(), "color: %hu", static_cast<uint16_t>(robot_msg_->color()));
    }

    hikcamera::ImageCapturer::CameraProfile camera_profile;
    camera_profile.exposure_time = std::chrono::milliseconds(exposure_time_);
    camera_profile.gain          = 16.9807;
    if ((debug_mode_ ? debug_robot_id_ == 7 : robot_msg_->id() == rmcs_msgs::ArmorID::Sentry)) {
        camera_profile.invert_image = true;
    } else {
        camera_profile.invert_image = false;
    }

    hikcamera::ImageCapturer img_capture(camera_profile);

    auto package_share_directory = ament_index_cpp::get_package_share_directory("rmcs_auto_aim");

    auto armor_identifier = ArmorIdentifier(package_share_directory + armor_model_path_);
    auto buff_identifier  = BuffIdentifier(package_share_directory + buff_model_path_);

    auto armor_tracker = ArmorTracker(armor_predict_duration_, debug_mode_);
    auto buff_tracker  = BuffTracker(buff_predict_duration_);

    auto buff_enabled = false;

    auto my_color = debug_mode_ ? static_cast<rmcs_msgs::RobotColor>(debug_color_) : robot_msg_->color();

    auto target_color = static_cast<rmcs_msgs::RobotColor>(1 - static_cast<uint8_t>(my_color));

    FPSCounter fps;

    if (record_mode_) {
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
            record_mode_ = false;
        }
    }

    while (rclcpp::ok()) {
        if (!debug_mode_ && *stage_ == rmcs_msgs::GameStage::SETTLING) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        auto img             = img_capture.read();
        auto timestamp       = std::chrono::steady_clock::now();
        thread_local auto tf = *tf_;

        do {
            if (!buff_enabled && (debug_mode_ ? debug_buff_mode_ : keyboard_->g == 1)) {

                buff_tracker.ResetAll(tf);
            }
            buff_enabled = (debug_mode_ ? debug_buff_mode_ : keyboard_->g == 1);

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
                    if (auto buff3d = BuffPnPSolver::Solve(*buff, tf, fx, fy, cx, cy, k1, k2, k3)) {
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

        if (record_mode_ && (debug_mode_ || *stage_ == rmcs_msgs::GameStage::STARTED) && recorder.is_opened()
            && !img.empty()) {

            std::shared_ptr<cv::Mat> imgPtr = std::make_shared<cv::Mat>(img);
            std::unique_lock<std::mutex> lock(img_mtx_);
            image_queue_.push(imgPtr);
            lock.unlock();
            img_cv_.notify_one();
        }
        // cv::imshow("img", img);
        // cv::waitKey(10);

        if (fps.Count()) {
            RCLCPP_INFO(get_logger(), "Fps:%d", fps.GetFPS());
        }
    } // while rclcpp::ok end
}

template <typename Link>
void Controller::omni_perception_process(const std::string& device) {

    auto camera = cv::VideoCapture(device);

    if (!camera.isOpened()) {
        RCLCPP_WARN(get_logger(), "Failed to open camera!");
        return;
    }
    camera.set(cv::CAP_PROP_EXPOSURE, omni_exposure_);

    auto package_share_directory = ament_index_cpp::get_package_share_directory("rmcs_auto_aim");

    auto armor_identifier = ArmorIdentifier(package_share_directory + armor_model_path_);

    auto my_color     = debug_mode_ ? static_cast<rmcs_msgs::RobotColor>(debug_color_) : robot_msg_->color();
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
            auto pnp_result =
                ArmorPnPSolver::Solve(armor, omni_fx, omni_fy, omni_cx, omni_cy, omni_k1, omni_k2, omni_k3);
            typename Link::Position pos{
                pnp_result.pose.position.x, pnp_result.pose.position.y, pnp_result.pose.position.z};
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