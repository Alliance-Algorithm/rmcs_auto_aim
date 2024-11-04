#include <atomic>
#include <memory>
#include <rmcs_description/tf_description.hpp>
#include <thread>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/node.hpp>

#include "core/pnpsolver/armor/armor_pnp_solver.hpp"
#include "rmcs_executor/component.hpp"
#include <hikcamera/image_capturer.hpp>

#include "identifier/armor/armor_identifier.hpp"

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

        fx_ = get_parameter("fx").as_double();
        fy_ = get_parameter("fy").as_double();
        cx_ = get_parameter("cx").as_double();
        cy_ = get_parameter("cy").as_double();
        k1_ = get_parameter("k1").as_double();
        k2_ = get_parameter("k2").as_double();
        k3_ = get_parameter("k3").as_double();

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

                while (rclcpp::ok()) {
                    auto image = capturer->read();

                    auto armor_plates =
                        armor_identifier->Identify(image, *target_color_, *whitelist_);
                    auto armor3d = ArmorPnPSolver::SolveAll(
                        armor_plates, tf_buffer_[tf_index_], fx_, fy_, cx_, cy_, k1_, k2_, k3_);

                    if (armor3d.size() > 0) {
                        RCLCPP_INFO(
                            get_logger(), "Armor3D: %hu", static_cast<uint16_t>(armor3d[0].id));
                    }
                }
            });
        }
        tf_buffer_[!tf_index_] = *tf_;
        tf_index_              = !tf_index_;
    }

private:
    double fx_, fy_, cx_, cy_, k1_, k2_, k3_;

    std::vector<std::thread> threads_;

    rmcs_description::Tf tf_buffer_[2];
    std::atomic<bool> tf_index_{false};
    // rclcpp::Publisher<rmcs_msgs::msg::ArmorPlateArray>::SharedPtr armor_plates_pub_;

    InputInterface<size_t> update_count_;
    InputInterface<uint8_t> whitelist_;
    InputInterface<rmcs_msgs::RobotColor> target_color_;
    InputInterface<rmcs_description::Tf> tf_;
};
} // namespace rmcs_auto_aim

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rmcs_auto_aim::AutoAimController, rmcs_executor::Component)