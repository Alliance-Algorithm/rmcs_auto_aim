#include <cstddef>
#include <vector>

#include <rclcpp/node.hpp>
#include <rmcs_description/tf_description.hpp>
#include <rmcs_executor/component.hpp>

#include "core/frame.hpp"
#include "core/identifier/armor/armor.hpp"
#include "core/pnpsolver/armor/armor3d.hpp"
#include "core/pnpsolver/armor/armor_pnp_solver.hpp"

namespace rmcs_auto_aim {
class ArmorPnpSolverNode
    : public rmcs_executor::Component
    , public rclcpp::Node {
public:
    explicit ArmorPnpSolverNode()
        : rclcpp::Node(
              get_component_name(),
              rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(true)) {
        register_input("/auto_aim/armor_plates", frame_input_);
        register_input("/tf", tf_);

        register_output("/auto_aim/armor_plates_3d", frame_output_);

        fx_ = get_parameter("fx").as_double();
        fy_ = get_parameter("fy").as_double();
        cx_ = get_parameter("cx").as_double();
        cy_ = get_parameter("cy").as_double();
        k1_ = get_parameter("k1").as_double();
        k2_ = get_parameter("k2").as_double();
        k3_ = get_parameter("k3").as_double();

        RCLCPP_INFO(get_logger(), "Pnp Solver Node Initialized");
    }

    void update() override {
        if (last_frame_id_ != frame_input_->frame_id_) {
            frame_output_->frame_id_ = frame_input_->frame_id_;
            auto result              = ArmorPnPSolver::SolveAll(
                frame_input_->data_, *tf_, fx_, fy_, cx_, cy_, k1_, k2_, k3_);
            if (!result.empty()) {
                frame_output_->data_     = result;
                frame_output_->frame_id_ = frame_input_->frame_id_;
                if (publish_topic_) {
                    // TODO: publish topic
                }
                last_frame_id_ = frame_input_->frame_id_;
            }
        }
    }

private:
    double fx_, fy_, cx_, cy_, k1_, k2_, k3_;
    bool publish_topic_{false};
    size_t last_frame_id_{0};

    InputInterface<rmcs_description::Tf> tf_;
    InputInterface<struct rmcs_auto_aim::Frame<std::vector<ArmorPlate>>> frame_input_;

    OutputInterface<struct rmcs_auto_aim::Frame<std::vector<ArmorPlate3d>>> frame_output_;
};
} // namespace rmcs_auto_aim

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(rmcs_auto_aim::ArmorPnpSolverNode, rmcs_executor::Component)