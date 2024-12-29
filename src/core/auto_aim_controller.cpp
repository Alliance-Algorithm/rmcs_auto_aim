#include <atomic>
#include <memory>
#include <thread>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>

#include <hikcamera/image_capturer.hpp>
#include <rmcs_executor/component.hpp>

#include "core/identifier/armor/armor_identifier.hpp"
#include "core/pnpsolver/armor/armor_pnp_solver.hpp"
#include "core/tracker/armor/armor_tracker.hpp"
#include "core/tracker/armor/target.hpp"
#include "core/trajectory/trajectory_solvor.hpp"
#include "core/transform_optimizer/armor/armor.hpp"
#include "core/transform_optimizer/armor/squad.hpp"
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
        register_input("/auto_aim/whitelist", whitelist_);
        register_input("/tf", tf_);

        register_output(
            "/gimbal/auto_aim/control_direction", control_direction_, Eigen::Vector3d::Zero());

        fx_ = get_parameter("fx").as_double();
        fy_ = get_parameter("fy").as_double();
        cx_ = get_parameter("cx").as_double();
        cy_ = get_parameter("cy").as_double();
        k1_ = get_parameter("k1").as_double();
        k2_ = get_parameter("k2").as_double();
        k3_ = get_parameter("k3").as_double();

        yaw_error_      = get_parameter("yaw_error").as_double();
        pitch_error_    = get_parameter("pitch_error").as_double();
        shoot_velocity_ = get_parameter("shoot_velocity").as_double();
        predict_sec_    = get_parameter("predict_sec").as_double();

        RCLCPP_INFO(get_logger(), "Armor Identifier Node Initialized");
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
                hikcamera::ImageCapturer::CameraProfile profile;
                profile.invert_image = get_parameter("invert_image").as_bool();
                profile.gain         = 16.9807;
                profile.exposure_time =
                    std::chrono::milliseconds(get_parameter("exposure_time").as_int());

                auto capturer = std::make_unique<hikcamera::ImageCapturer>(profile);

                auto armor_identifier = std::make_unique<ArmorIdentifier>(
                    ament_index_cpp::get_package_share_directory("rmcs_auto_aim")
                    + "/models/mlp.onnx");
                auto armor_tracker = rmcs_auto_aim::ArmorTracker(100); // TODO

                rmcs_auto_aim::util::FPSCounter fps;

                while (rclcpp::ok()) {

                    auto image     = capturer->read();
                    auto timestamp = std::chrono::steady_clock::now();
                    auto tf        = tf_buffer_[tf_index_.load()];
                    auto armor_plates =
                        armor_identifier->Identify(image, *target_color_, *whitelist_);
                    auto armor3d = ArmorPnPSolver::SolveAll(
                        armor_plates, tf, fx_, fy_, cx_, cy_, k1_, k2_, k3_);

                    for (auto& armor3dTmp : armor3d) {
                        transform_optimizer::Squad3d::darw_squad(
                            fx_, fy_, cx_, cy_, k1_, k2_, k3_, tf, image,
                            transform_optimizer::Squad3d(armor3dTmp), false, {255, 0, 0}, 1,
                            cv::LineTypes::LINE_4);
                    }
                    // for (auto& armor2dTmp : armor_plates) {
                    //     cv::line(image, armor2dTmp.points[0], armor2dTmp.points[1], {0, 255, 0});
                    //     cv::line(image, armor2dTmp.points[1], armor2dTmp.points[2], {0, 255, 0});
                    //     cv::line(image, armor2dTmp.points[2], armor2dTmp.points[3], {0, 255, 0});
                    //     cv::line(image, armor2dTmp.points[3], armor2dTmp.points[0], {0, 255, 0});
                    // }

                    transform_optimizer::transform_optimize(
                        armor_plates, armor3d, tf, fx_, fy_, cx_, cy_, k1_, k2_, k3_);

                    for (auto& armor3dTmp : armor3d) {
                        transform_optimizer::Squad3d::darw_squad(
                            fx_, fy_, cx_, cy_, k1_, k2_, k3_, tf, image,
                            transform_optimizer::Squad3d(armor3dTmp), false, {0, 0, 255});

                        cv::putText(
                            image,
                            std::to_string(
                                transform_optimizer::get_yaw_from_quaternion(*armor3dTmp.rotation)),
                            {200, 700}, 1, 3, {255, 0, 255});
                    }

                    cv::imshow("squad", image);
                    cv::waitKey(1);
                    if (auto target = armor_tracker.Update(armor3d, timestamp, tf)) {
                        armor_target_buffer_[!armor_target_index_.load()].target_ =
                            std::move(target);
                        armor_target_buffer_[!armor_target_index_.load()].timestamp_ = timestamp;
                        armor_target_index_.store(!armor_target_index_.load());
                    }

                    if (fps.Count()) {
                        RCLCPP_INFO(get_logger(), "FPS: %d", fps.GetFPS());
                    }
                }
            });
        }

        auto frame = armor_target_buffer_[armor_target_index_.load()];
        if (!frame.target_) {
            *control_direction_ = Eigen::Vector3d::Zero();
            return;
        }
        auto offset = fast_tf::cast<rmcs_description::OdomImu>(
            rmcs_description::MuzzleLink::Position{0, 0, 0}, *tf_);

        using namespace std::chrono_literals;
        auto diff = std::chrono::steady_clock::now() - frame.timestamp_;
        if (diff > std::chrono::milliseconds(500)) { // TODO
            *control_direction_ = Eigen::Vector3d::Zero();
            return;
        }

        double fly_time = 0;
        for (int i = 5; i-- > 0;) {
            auto pos = frame.target_->Predict(
                static_cast<std::chrono::duration<double>>(diff).count() + fly_time + predict_sec_);
            auto aiming_direction = *trajectory_.GetShotVector(
                {pos->x() - offset->x(), pos->y() - offset->y(), pos->z() - offset->z()},
                shoot_velocity_, fly_time);

            if (i == 0) {
                auto yaw_axis = fast_tf::cast<rmcs_description::PitchLink>(
                                    rmcs_description::OdomImu::DirectionVector(0, 0, 1), *tf_)
                                    ->normalized();
                auto pitch_axis = fast_tf::cast<rmcs_description::PitchLink>(
                                      rmcs_description::OdomImu::DirectionVector(0, 1, 0), *tf_)
                                      ->normalized();
                auto delta_yaw      = Eigen::AngleAxisd{yaw_error_, yaw_axis};
                auto delta_pitch    = Eigen::AngleAxisd{pitch_error_, pitch_axis};
                aiming_direction    = delta_pitch * (delta_yaw * (aiming_direction));
                *control_direction_ = aiming_direction;
                break;
            }
        }
    }

private:
    struct TargetFrame {
        std::shared_ptr<rmcs_auto_aim::ArmorTarget> target_;
        std::chrono::steady_clock::time_point timestamp_;
    };

    TrajectorySolver trajectory_;

    double fx_, fy_, cx_, cy_, k1_, k2_, k3_;
    double pitch_error_;
    double yaw_error_;
    double shoot_velocity_;
    double predict_sec_;

    std::vector<std::thread> threads_;

    rmcs_description::Tf tf_buffer_[2];
    std::atomic<bool> tf_index_{false};

    // rmcs_auto_aim::Target target_;
    struct TargetFrame armor_target_buffer_[2];
    std::atomic<bool> armor_target_index_{false};

    InputInterface<size_t> update_count_;
    InputInterface<uint8_t> whitelist_;
    InputInterface<rmcs_msgs::RobotColor> target_color_;
    InputInterface<rmcs_description::Tf> tf_;

    OutputInterface<Eigen::Vector3d> control_direction_;
};
} // namespace rmcs_auto_aim

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_auto_aim::AutoAimController, rmcs_executor::Component)