
#include <chrono>
#include <cstddef>
#include <exception>
#include <map>
#include <utility>

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <rclcpp/logging.hpp>

#include <robot_color.hpp>
#include <robot_id.hpp>

#include "core/debugger/debugger.hpp"
#include "core/identifier/armor/armor_identifier.hpp"
#include "core/identifier/buff/buff_identifier.hpp"
#include "core/pnpsolver/armor/armor_pnp_solver.hpp"
#include "core/pnpsolver/buff/buff_pnp_solver.hpp"
#include "core/tracker/armor/armor_tracker.hpp"
#include "core/tracker/buff/buff_tracker.hpp"
#include <robot_color.hpp>
#include <robot_id.hpp>

#include "auto_aim_controller.hpp"

using namespace rmcs_auto_aim;

void Controller::gimbal_process() {
    if (robot_msg_->id() != rmcs_msgs::ArmorID::Unknown) {
        RCLCPP_INFO(get_logger(), "Robot Info:");
        RCLCPP_INFO(get_logger(), "id: %hu", static_cast<uint16_t>(robot_msg_->id()));
        RCLCPP_INFO(get_logger(), "color: %hu", static_cast<uint16_t>(robot_msg_->color()));
    } else {
        // RCLCPP_INFO(get_logger(), "Unknown Robot.Use debug mode");
        // debug_mode_ = true;
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

    auto target_color = rmcs_msgs::RobotColor::BLUE;
    if (debug_mode_) {
        target_color = static_cast<rmcs_msgs::RobotColor>(1 + debug_color_);
    } else if (robot_msg_->color() == rmcs_msgs::RobotColor::BLUE) {
        target_color = rmcs_msgs::RobotColor::RED;
    }

    if (target_color == rmcs_msgs::RobotColor::RED) {
        RCLCPP_INFO(get_logger(), "Target Color: RED");
    } else {
        RCLCPP_INFO(get_logger(), "Target Color: BLUE");
    }

    FPSCounter fps;

    if (record_mode_) {
        auto flag = true;
        recorder_.setParam(static_cast<double>(record_fps_), [&img_capture, this, &flag]() {
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

        if (!flag || !recorder_.is_opened()) {
            RCLCPP_WARN(get_logger(), "Failed to open an VideoWriter.");
            record_mode_ = false;
        }
    }

    while (rclcpp::ok()) {
        if (!debug_mode_ && *stage_ == rmcs_msgs::GameStage::SETTLING) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        auto img       = img_capture.read();
        auto timestamp = std::chrono::steady_clock::now();
        auto tf        = *tf_;

        do {
            if (!buff_enabled && (debug_mode_ ? debug_buff_mode_ : keyboard_->g == 1)) {

                buff_tracker.ResetAll(tf);
            }
            buff_enabled = (debug_mode_ ? debug_buff_mode_ : keyboard_->g == 1);

            if (!buff_enabled) {
                auto armors = armor_identifier.Identify(img, target_color, blacklist_.load());
                if (!armors.empty()) {
                    auto center = armors[0].center();
                    *ui_target_ = std::make_pair(center.x, center.y);
                }
                auto armor3d = ArmorPnPSolver::SolveAll(armors, tf, fx_, fy_, cx_, cy_, k1_, k2_, k3_);

                if (!armor3d.empty()) {
                    for (auto& object : armor3d) {
                        auto& id  = object.id;
                        auto& pos = object.position;
                        communicate(id, pos);
                    }
                }

                if (auto target = armor_tracker.Update(armor3d, timestamp, tf)) {
                    timestamp_ = timestamp;
                    target_.swap(target);
                    target_updated_.store(true);
                    break;
                }

            } else {
                if (auto buff = buff_identifier.Identify(img)) {
                    if (auto buff3d = BuffPnPSolver::Solve(*buff, tf, fx_, fy_, cx_, cy_, k1_, k2_, k3_)) {
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

        if (record_mode_ && (debug_mode_ || *stage_ == rmcs_msgs::GameStage::STARTED) && recorder_.is_opened()
            && !img.empty()) {

            std::shared_ptr<cv::Mat> imgPtr = std::make_shared<cv::Mat>(img);
            std::unique_lock<std::mutex> lock(img_mtx_);
            image_queue_.emplace(this->now(), imgPtr);
            lock.unlock();
            img_cv_.notify_one();
        }
        // cv::imshow("img", img);
        // cv::waitKey(10);

        if (fps.Count()) {
            RCLCPP_INFO(
                get_logger(), "Game Stage: %hhu , Fps:%d", static_cast<uint8_t>(*stage_), fps.GetFPS());
        }
    } // while rclcpp::ok end
}

void Controller::communicate(const rmcs_msgs::ArmorID& id, const rmcs_description::OdomImu::Position& pos) {
    Eigen::Vector2d plate_pos{pos->x(), pos->y()};
    Eigen::Vector2d zero{0, 0};
    switch (id) {
    case rmcs_msgs::ArmorID::Hero: {
        *enemies_hero_pose_         = plate_pos;
        *enemies_engineer_pose_     = zero;
        *enemies_infantry_iii_pose_ = zero;
        *enemies_infantry_iv_pose_  = zero;
        *enemies_infantry_v_pose_   = zero;
        *enemies_sentry_pose_       = zero;
        break;
    }
    case rmcs_msgs::ArmorID::Engineer: {
        *enemies_hero_pose_         = zero;
        *enemies_engineer_pose_     = plate_pos;
        *enemies_infantry_iii_pose_ = zero;
        *enemies_infantry_iv_pose_  = zero;
        *enemies_infantry_v_pose_   = zero;
        *enemies_sentry_pose_       = zero;
        break;
    }
    case rmcs_msgs::ArmorID::InfantryIII: {
        *enemies_hero_pose_         = zero;
        *enemies_engineer_pose_     = zero;
        *enemies_infantry_iii_pose_ = plate_pos;
        *enemies_infantry_iv_pose_  = zero;
        *enemies_infantry_v_pose_   = zero;
        *enemies_sentry_pose_       = zero;
        break;
    }
    case rmcs_msgs::ArmorID::InfantryIV: {
        *enemies_hero_pose_         = zero;
        *enemies_engineer_pose_     = zero;
        *enemies_infantry_iii_pose_ = zero;
        *enemies_infantry_iv_pose_  = plate_pos;
        *enemies_infantry_v_pose_   = zero;
        *enemies_sentry_pose_       = zero;
        break;
    }
    case rmcs_msgs::ArmorID::InfantryV: {
        *enemies_hero_pose_         = zero;
        *enemies_engineer_pose_     = zero;
        *enemies_infantry_iii_pose_ = zero;
        *enemies_infantry_iv_pose_  = zero;
        *enemies_infantry_v_pose_   = plate_pos;
        *enemies_sentry_pose_       = zero;
        break;
    }
    case rmcs_msgs::ArmorID::Sentry: {
        *enemies_hero_pose_         = zero;
        *enemies_engineer_pose_     = zero;
        *enemies_infantry_iii_pose_ = zero;
        *enemies_infantry_iv_pose_  = zero;
        *enemies_infantry_v_pose_   = zero;
        *enemies_sentry_pose_       = plate_pos;
        break;
    }
    default: break;
    }
}

template <typename Link>
void Controller::omni_perception_process(const std::string& device) {
    RCLCPP_INFO(get_logger(), "%s Omni-Direction Perception Start.", device.c_str());

    auto camera = cv::VideoCapture(device, cv::CAP_V4L);

    if (!camera.isOpened()) {
        RCLCPP_WARN(get_logger(), "Failed to open camera!");
        return;
    } else {
        RCLCPP_INFO(get_logger(), "Omni-Direction Perception Start.");
    }
    camera.set(cv::CAP_PROP_AUTO_EXPOSURE, 1);
    camera.set(cv::CAP_PROP_EXPOSURE, omni_exposure_);
    RCLCPP_INFO(get_logger(), "exposure time = %f", camera.get(cv::CAP_PROP_EXPOSURE));

    auto package_share_directory = ament_index_cpp::get_package_share_directory("rmcs_auto_aim");

    auto armor_identifier = ArmorIdentifier(package_share_directory + armor_model_path_);

    auto target_color = rmcs_msgs::RobotColor::BLUE;
    if (debug_mode_) {
        target_color = static_cast<rmcs_msgs::RobotColor>(1 + debug_color_);
    } else if (robot_msg_->color() == rmcs_msgs::RobotColor::BLUE) {
        target_color = rmcs_msgs::RobotColor::RED;
    }
    cv::Mat img;

    while (camera.isOpened()) {
        camera >> img;

        if (img.empty()) {
            continue;
        }
        auto armors = armor_identifier.Identify(img, target_color, blacklist_.load());

        std::map<rmcs_msgs::ArmorID, typename Link::Position> targets_map;

        for (auto& armor : armors) {
            auto pnp_result =
                ArmorPnPSolver::Solve(armor, omni_fx, omni_fy, omni_cx, omni_cy, omni_k1, omni_k2, omni_k3);
            typename Link::Position pos{
                pnp_result.pose.position.x, pnp_result.pose.position.y, pnp_result.pose.position.z};
            RCLCPP_INFO(
                get_logger(), "%s,Omni-Direction Perception detected: Armor [%hu]", device.c_str(),
                static_cast<uint16_t>(pnp_result.id));
            targets_map.insert(std::make_pair(pnp_result.id, pos));
        }

        for (auto& [id, target] : targets_map) {
            auto pos = fast_tf::cast<rmcs_description::OdomImu>(target, *tf_);
            communicate(id, pos);
        }
    }
}

void Controller::update() {
    if (*update_count_ == 0) {
        if (!debug_mode_ && !robot_msg_.ready()) {
            RCLCPP_WARN(get_logger(), "Robot ID is unknown. Waiting for robot ID...");
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            return;
        }
        if ((debug_mode_ && debug_robot_id_ == (uint16_t)rmcs_msgs::ArmorID::Sentry)
            || (!debug_mode_ && robot_msg_->id() == rmcs_msgs::ArmorID::Sentry)) {
            RCLCPP_INFO(get_logger(), "Reading omni-direction perception parameters");
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

        /******************************
            Auto Gimbal Aiming System
         ******************************/
        threads_.emplace_back([this]() {
            size_t attempt = 0;
            while (true) {
                if (!debug_mode_ && robot_msg_->id() == rmcs_msgs::ArmorID::Unknown) {
                    RCLCPP_WARN(get_logger(), "Robot ID is unknown. Waiting for robot ID...");
                    // continue;
                } else {
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
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(5000));
            }
        });

        /********************
            Image Recorder
         *******************/
        if (record_mode_) {
            threads_.emplace_back([this]() {
                if (recorder_.is_opened()) {
                    RCLCPP_INFO(get_logger(), "RECORDING %s...", recorder_.get_filename().c_str());
                }
                size_t t = 0;
                while (rclcpp::ok()) {
                    if (!recorder_.is_opened()) {
                        continue;
                    }
                    std::unique_lock<std::mutex> lock(img_mtx_);
                    img_cv_.wait(lock, [this] { return !image_queue_.empty(); });
                    std::shared_ptr<cv::Mat> imgPtr = image_queue_.front().second;
                    auto timestamp                  = image_queue_.front().first;
                    image_queue_.pop();
                    lock.unlock();

                    cv::Mat img = *imgPtr;
                    recorder_.record_frame(img);
                    if (raw_img_pub_mode_ && t++ >= 3) {
                        t = 0;
                        debugger_.publish_raw_image(img, timestamp);
                    }
                }
            });
        }

        /*****************************************
            Omni Direction Perception System
         *****************************************/
        if ((debug_mode_ && debug_robot_id_ == (uint16_t)rmcs_msgs::ArmorID::Sentry)
            || (!debug_mode_ && robot_msg_->id() == rmcs_msgs::ArmorID::Sentry)) {
            threads_.emplace_back([this]() {
                size_t attempt = 0;
                while (true) {
                    try {
                        omni_perception_process<rmcs_description::OmniLinkLeftFront>("/dev/leftfront");
                    } catch (std::exception& e) {
                        attempt++;
                        if (attempt < 10) {
                            RCLCPP_FATAL(get_logger(), "Omni direction perception<Left Front>: %s", e.what());
                        } else if (attempt == 10) {
                            RCLCPP_WARN(get_logger(), "Too many error. Diabled log...");
                        }
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
                }
            });
            threads_.emplace_back([this]() {
                size_t attempt = 0;
                while (true) {
                    try {
                        omni_perception_process<rmcs_description::OmniLinkRightFront>("/dev/rightfront");
                    } catch (std::exception& e) {
                        attempt++;
                        if (attempt < 10) {
                            RCLCPP_FATAL(
                                get_logger(), "Omni direction perception<Right Front>: %s", e.what());
                        } else if (attempt == 10) {
                            RCLCPP_WARN(get_logger(), "Too many error. Diabled log...");
                        }
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
                }
            });
            threads_.emplace_back([this]() {
                size_t attempt = 0;
                while (true) {
                    try {
                        omni_perception_process<rmcs_description::OmniLinkLeft>("/dev/left");
                    } catch (std::exception& e) {
                        attempt++;
                        if (attempt < 10) {
                            RCLCPP_FATAL(get_logger(), "Omni direction perception<Left>: %s", e.what());
                        } else if (attempt == 10) {
                            RCLCPP_WARN(get_logger(), "Too many error. Diabled log...");
                        }
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
                }
            });
            threads_.emplace_back([this]() {
                size_t attempt = 0;
                while (true) {
                    try {
                        omni_perception_process<rmcs_description::OmniLinkRight>("/dev/right");
                    } catch (std::exception& e) {
                        attempt++;
                        if (attempt < 10) {
                            RCLCPP_FATAL(get_logger(), "Omni direction perception<Right>: %s", e.what());
                        } else if (attempt == 10) {
                            RCLCPP_WARN(get_logger(), "Too many error. Diabled log...");
                        }
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
                }
            });
        }
    }

    auto local_target = target_.release();
    if (!local_target) {
        // *control_direction_ = Eigen::Vector3d::Zero();
        return;
    }
    using namespace std::chrono_literals;
    auto diff = std::chrono::steady_clock::now() - timestamp_;
    if (diff > std::chrono::milliseconds(gimbal_predict_duration_)) {
        *control_direction_ = Eigen::Vector3d::Zero();
        return;
    }

    auto offset =
        fast_tf::cast<rmcs_description::OdomImu>(rmcs_description::MuzzleLink::Position{0, 0, 0}, *tf_);

    double fly_time = 0;
    for (int i = 5; i-- > 0;) {
        auto pos =
            local_target->Predict(static_cast<std::chrono::duration<double>>(diff).count() + fly_time + 0.05);
        auto aiming_direction = *trajectory_.GetShotVector(
            {pos->x() - offset->x(), pos->y() - offset->y(), pos->z() - offset->z()}, 27.0, fly_time);

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