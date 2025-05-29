#include <atomic>
#include <chrono>
#include <memory>
#include <thread>
#include <tuple>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <rclcpp/node.hpp>
#include <std_msgs/msg/header.hpp>

#include <hikcamera/image_capturer.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/game_stage.hpp>

#include "core/fire_controller/fire_controller.hpp"
#include "core/identifier/armor/armor_identifier.hpp"
#include "core/pnpsolver/fusion_solver.hpp"
#include "core/tracker/armor/armor_tracker.hpp"
#include "core/trajectory/trajectory_solvor.hpp"
#include "util/image_viewer/image_viewer.hpp"
#include "util/math.hpp"
#include "util/profile/profile.hpp"
#include "util/recorder/recorder.hh"
#include "util/utils.hpp"

namespace rmcs_auto_aim {
class AutoAimController
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    AutoAimController()
        : rclcpp::Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        RCLCPP_INFO(this->get_logger(), "AutoAimController init");

        register_input("/predefined/update_count", update_count_);
        register_input("/auto_aim/target_color", target_color_);
        register_input("/referee/game/stage", game_stage_);
        register_input("/auto_aim/whitelist", whitelist_);
        register_input("/tf", tf_);

        hikcamera::ImageCapturer::CameraProfile profile;
        profile.invert_image  = get_parameter("invert_image").as_bool();
        profile.gain          = 16.9807;
        profile.exposure_time = std::chrono::milliseconds(get_parameter("exposure_time").as_int());

        capturer_ =
            std::make_unique<hikcamera::ImageCapturer>(profile, nullptr, hikcamera::SyncMode::NONE);
        capturer_->set_frame_rate_inner_trigger_mode(100);

        fx_ = get_parameter("fx").as_double();
        fy_ = get_parameter("fy").as_double();
        cx_ = get_parameter("cx").as_double();
        cy_ = get_parameter("cy").as_double();
        k1_ = get_parameter("k1").as_double();
        k2_ = get_parameter("k2").as_double();
        k3_ = get_parameter("k3").as_double();

        util::Profile(fx_, fy_, cx_, cy_, k1_, k2_, k3_);
        std::apply(util::Profile::set_width_height, capturer_->get_width_height());
        util::ImageViewer::ImageViewer::createProduct(
            (int)(get_parameter("image_viewer_type").as_int()), *this, "rmcs_auto_aim/debug");

        register_output(
            "/gimbal/auto_aim/control_direction", control_direction_, Eigen::Vector3d::Zero());
        register_output("/gimbal/auto_aim/fire_control", fire_control_, false);
        register_output("/debug/target_omega", debug_target_omega_, 0);
        register_output("/debug/target_theta_", debug_target_theta_, 0);

        yaw_error_      = get_parameter("yaw_error").as_double();
        pitch_error_    = get_parameter("pitch_error").as_double();
        shoot_velocity_ = get_parameter("shoot_velocity").as_double();
        predict_sec_    = get_parameter("predict_sec").as_double();
        RCLCPP_INFO(get_logger(), "Armor Identifier Node Initialized");

        recorder_ = std::make_unique<Recorder>(*this);
    }

    ~AutoAimController() override {
        for (auto& thread : threads_) {
            if (thread.joinable()) {
                thread.join();
            }
        }
        RCLCPP_INFO(get_logger(), "AutoAimController destroy");
    }

    void update() override {

        tf_buffer_[!tf_index_.load()] = *tf_;
        tf_index_.store(!tf_index_.load());

        if (*update_count_ == 0) {
            if (!target_color_.ready()) {
                RCLCPP_WARN(get_logger(), "target_color_ not ready");
                throw std::runtime_error("target_color_ not ready");
            }

            threads_.emplace_back([this]() {
                auto armor_identifier = std::make_unique<ArmorIdentifier>(
                    ament_index_cpp::get_package_share_directory("rmcs_auto_aim")
                    + "/models/mlp.onnx");
                auto armor_tracker = tracker::armor::ArmorTracker(*this);

                rmcs_auto_aim::util::FPSCounter fps;

                while (rclcpp::ok()) {

                    const auto image = capturer_->read();

                    if (recorder_->ready_save(now()))
                        recorder_->save_image(image);

                    thread_sync_clk_ = std::chrono::steady_clock::now();
                    auto tf          = tf_buffer_[tf_index_.load()];
                    auto timestamp   = std::chrono::steady_clock::now();
                    util::ImageViewer::load_image(image);
                    auto armor_plates =
                        armor_identifier->Identify(image, *target_color_, *whitelist_);

                    auto armor3d = FusionSolver::SolveAll(armor_plates, tf);

                    for (auto& armor2d_ : armor3d) {
                        util::ImageViewer::draw(
                            transform_optimizer::Quadrilateral3d(armor2d_).ToQuadrilateral(
                                tf, true),
                            {0, 255, 0});
                        *debug_target_theta_ =
                            util::math::get_yaw_from_quaternion(*armor2d_.rotation);
                    }
                    if (auto target = armor_tracker.Update(armor3d, timestamp, tf)) {
                        armor_target_buffer_[!armor_target_index_.load()].target_ =
                            std::move(target);
                        armor_target_buffer_[!armor_target_index_.load()].timestamp_ = timestamp;
                        armor_target_index_.store(!armor_target_index_.load());
                    }
                    armor_tracker.draw_armors(tf, {0, 0, 255});
                    util::ImageViewer::show_image();
                    if (fps.Count()) {
                        // RCLCPP_INFO(get_logger(), "FPS: %d", fps.GetFPS());
                    }
                }
            });
        }

        // 摄像头录制器状态切换
        {
            using namespace rmcs_msgs;
            if (*game_stage_ == GameStage::STARTED && last_game_stage_ != GameStage::STARTED)
                recorder_->set_record_status(true);

            if (*game_stage_ == GameStage::SETTLING && last_game_stage_ != GameStage::SETTLING)
                recorder_->set_record_status(false);

            if (*game_stage_ == GameStage::UNKNOWN && last_game_stage_ != GameStage::UNKNOWN)
                recorder_->set_record_status(false);

            last_game_stage_ = *game_stage_;
        }

        auto frame = armor_target_buffer_[armor_target_index_.load()];
        if (!frame.target_) {
            *control_direction_ = Eigen::Vector3d::Zero();
            return;
        }
        *debug_target_omega_ = frame.target_->get_omega();
        auto offset          = fast_tf::cast<rmcs_description::OdomImu>(
            rmcs_description::MuzzleLink::Position{0, 0, 0}, *tf_);

        using namespace std::chrono_literals;
        auto diff = std::chrono::steady_clock::now() - frame.timestamp_;
        if (diff > std::chrono::milliseconds(500)) { // TODO
            *control_direction_ = Eigen::Vector3d::Zero();
            *fire_control_      = false;
            return;
        }

        double fly_time = 0;
        for (int i = 5; i-- > 0;) {
            auto [firecontrol, pos] = frame.target_->UpdateController(
                static_cast<std::chrono::duration<double>>(diff).count() + fly_time + predict_sec_,
                *tf_);
            *fire_control_        = firecontrol;
            auto aiming_direction = *trajectory_.GetShotVector(
                {pos->x() - offset->x(), pos->y() - offset->y(), pos->z() - offset->z()},
                shoot_velocity_, fly_time);

            if (i == 0) {
                auto yaw_axis = fast_tf::cast<rmcs_description::OdomImu>(
                                    rmcs_description::PitchLink::DirectionVector(0, 0, 1), *tf_)
                                    ->normalized();
                auto pitch_axis = fast_tf::cast<rmcs_description::OdomImu>(
                                      rmcs_description::PitchLink::DirectionVector(0, 1, 0), *tf_)
                                      ->normalized();
                auto delta_yaw      = Eigen::AngleAxisd{yaw_error_, yaw_axis};
                auto delta_pitch    = Eigen::AngleAxisd{pitch_error_, pitch_axis};
                aiming_direction    = delta_pitch * (delta_yaw * (aiming_direction));
                *control_direction_ = aiming_direction;
                break;
            }
        }
        bool deadband = (std::chrono::steady_clock::now() - fire_control_deadband_)
                      > std::chrono::milliseconds(50);
        if (fast_tf::cast<rmcs_description::OdomImu>(
                rmcs_description::PitchLink::DirectionVector(), *tf_)
                ->dot(*control_direction_)
            >= 0.996 + fly_time * 0.001)
            *fire_control_ = true && deadband && *fire_control_;
        else
            *fire_control_ = false && deadband && *fire_control_;
        if (fast_tf::cast<rmcs_description::OdomImu>(
                rmcs_description::PitchLink::DirectionVector(), *tf_)
                ->dot(*control_direction_)
            < 0.996 + fly_time * 0.001) {
            fire_control_deadband_ = std::chrono::steady_clock::now();
        }
        // std::cerr << fast_tf::cast<rmcs_description::OdomImu>(
        //                  rmcs_description::PitchLink::DirectionVector(), *tf_)
        //                  ->dot(*control_direction_)
        //           << std::endl;
    }

private:
    struct TargetFrame {
        std::shared_ptr<tracker::IFireController> target_;
        std::chrono::steady_clock::time_point timestamp_;
    };

    TrajectorySolver trajectory_;

    double fx_, fy_, cx_, cy_, k1_, k2_, k3_;
    double pitch_error_;
    double yaw_error_;
    double shoot_velocity_;
    double predict_sec_;

    std::vector<std::thread> threads_;
    std::chrono::time_point<std::chrono::steady_clock> thread_sync_clk_;

    rmcs_description::Tf tf_buffer_[2];
    std::atomic<bool> tf_index_{false};

    std::unique_ptr<hikcamera::ImageCapturer> capturer_;

    // rmcs_auto_aim::Target target_;
    struct TargetFrame armor_target_buffer_[2];
    std::atomic<bool> armor_target_index_{false};

    InputInterface<size_t> update_count_;
    InputInterface<uint8_t> whitelist_;
    InputInterface<rmcs_msgs::RobotColor> target_color_;
    InputInterface<rmcs_description::Tf> tf_;

    OutputInterface<Eigen::Vector3d> control_direction_;
    OutputInterface<bool> fire_control_;

    OutputInterface<double> debug_target_omega_;
    OutputInterface<double> debug_target_theta_;

    std::chrono::time_point<std::chrono::steady_clock> fire_control_deadband_;

    using TimePoint = std::chrono::steady_clock::time_point;
    static inline TimePoint now() { return std::chrono::steady_clock::now(); }

    std::unique_ptr<Recorder> recorder_;
    InputInterface<rmcs_msgs::GameStage> game_stage_;
    rmcs_msgs::GameStage last_game_stage_;
};
} // namespace rmcs_auto_aim

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_auto_aim::AutoAimController, rmcs_executor::Component)
