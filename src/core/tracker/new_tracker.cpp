#include "new_tracker.hpp"
#include "core/transform_optimizer/armor/quadrilateral/quadrilateral.hpp"
#include "tracker_model.hpp"
#include "tracker_state.hpp"
#include "util/image_viewer/image_viewer.hpp"
#include "util/math.hpp"

namespace rmcs_auto_aim::tracker {

class NewTracker::Impl {
public:
    inline std::shared_ptr<IFireController> Update(
        const std::vector<ArmorPlate3d>& armors,
        const std::chrono::steady_clock::time_point& timestamp, const rmcs_description::Tf& tf) {

        const auto dt =
            std::chrono::duration_cast<std::chrono::duration<double>>(timestamp - last_timestamp_)
                .count();
        if (armors.empty()) {
            // if (track_state_ == TrackerState::Track)
            //     predict(dt + 0.2);
            // if (dt > 1.0)
            //     track_state_ = TrackerState::Lost;
            // return nullptr;
        } else {
            if (track_state_ == TrackerState::Lost) {
                last_timestamp_ = timestamp;
                track_state_    = TrackerState::Track;
            } else {
                TrackerModel::ZVec input;
                if (armors.size() == 1) {
                    input << util::math::get_yaw_from_quaternion(*armors[0].rotation),
                        std::atan(armors[0].position->y() / armors[0].position->x()),
                        -std::atan(armors[0].position->z() / armors[0].position->x()),
                        armors[0].position->norm();
                    tracker_model_.Update(input, {}, dt);
                } else if (armors.size() == 2) {
                    const auto armor1_yaw =
                        util::math::get_yaw_from_quaternion(*armors[0].rotation);
                    const auto armor2_yaw =
                        util::math::get_yaw_from_quaternion(*armors[1].rotation);
                    if (std::abs(armor1_yaw) > std::abs(armor2_yaw)) {
                        input << armor1_yaw,
                            std::atan(armors[0].position->y() / armors[0].position->x()),
                            -std::atan(armors[0].position->z() / armors[0].position->x()),
                            armors[0].position->norm();
                        tracker_model_.Update(input, {}, dt);
                        tracker_model_.test();
                        input << armor2_yaw,
                            std::atan(armors[1].position->y() / armors[1].position->x()),
                            -std::atan(armors[1].position->z() / armors[1].position->x()),
                            armors[1].position->norm();
                        tracker_model_.Update(input, {}, 0.);
                    } else {
                        input << armor2_yaw,
                            std::atan(armors[1].position->y() / armors[1].position->x()),
                            -std::atan(armors[1].position->z() / armors[1].position->x()),
                            armors[1].position->norm();
                        tracker_model_.Update(input, {}, dt);
                        tracker_model_.test();
                        input << armor1_yaw,
                            std::atan(armors[0].position->y() / armors[0].position->x()),
                            -std::atan(armors[0].position->z() / armors[0].position->x()),
                            armors[0].position->norm();
                        tracker_model_.Update(input, {}, 0.);
                    }
                }
            };
        }

        // 预测时长，这里主要是把各种延迟加上,包括计算延迟，卡弹延迟以及子弹飞行时间等
        if (track_state_ == TrackerState::Track)
            predict(0);

        last_timestamp_ = timestamp;
        return nullptr;
    };

    inline void draw_armors(const cv::Scalar& color, const rmcs_description::Tf& tf) {
        for (const auto& armor : target_armors_)
            util::ImageViewer::draw(
                transform_optimizer::Quadrilateral3d(armor).ToQuadrilateral(tf, true), color);
    };

    inline std::array<ArmorPlate3d, 4> get_armor() { return target_armors_; }

    TrackerModel::XVec output() { return debug_; }

private:
    void predict(const double& dt) {
        const auto model_output = debug_ = tracker_model_.OutPut();

        ax            = model_output(1) - last_vx;
        ay            = model_output(3) - last_vy;
        last_vx       = model_output(1);
        last_vy       = model_output(3);
        const auto vx = model_output(1);
        const auto vy = model_output(3);

        const double car_x = model_output(0) + vx * dt;
        const double car_y = model_output(2) + vy * dt;

        const double car_z = model_output(4);
        const double z1    = model_output(5);
        const double z2    = model_output(6);
        const double r1    = model_output(7);
        const double r2    = model_output(8);
        double model_yaw   = model_output(9) + model_output(10) * dt;
        // std::cerr << "yaw:" << model_yaw * 180. / std::numbers::pi << std::endl;
        double odom_yaw    = model_yaw;

        // 模型里对yaw的角度进行了一个线性的映射，这里要映射回去以求出装甲板的方位角
        if (odom_yaw > std::numbers::pi)
            odom_yaw -= std::numbers::pi * 2.;

        const double pitch1 = -15. / 180. * std::numbers::pi;
        const double pitch2 = -15. / 180. * std::numbers::pi;

        if (tracker_model_.side_flag()) {
            *target_armors_[0].position << car_x + r1 * std::cos(model_yaw),
                car_y + r1 * std::sin(model_yaw), z1;
            *target_armors_[0].rotation = util::math::euler_to_quaternion(odom_yaw, pitch1, 0.0);

            odom_yaw += std::numbers::pi / 2.;
            model_yaw += std::numbers::pi / 2.;
            *target_armors_[1].position << car_x + r2 * std::cos(model_yaw),
                car_y + r2 * std::sin(model_yaw), z2;
            *target_armors_[1].rotation = util::math::euler_to_quaternion(odom_yaw, pitch2, 0.0);

            odom_yaw += std::numbers::pi / 2.;
            model_yaw += std::numbers::pi / 2.;
            *target_armors_[2].position << car_x + r1 * std::cos(model_yaw),
                car_y + r1 * std::sin(model_yaw), z1;
            *target_armors_[2].rotation = util::math::euler_to_quaternion(odom_yaw, pitch1, 0.0);

            odom_yaw += std::numbers::pi / 2.;
            model_yaw += std::numbers::pi / 2.;
            *target_armors_[3].position << car_x + r2 * std::cos(model_yaw),
                car_y + r2 * std::sin(model_yaw), z2;
            *target_armors_[3].rotation = util::math::euler_to_quaternion(odom_yaw, pitch2, 0.0);
        } else {
            *target_armors_[0].position << car_x + r2 * std::cos(model_yaw),
                car_y + r2 * std::sin(model_yaw), z2;
            *target_armors_[0].rotation = util::math::euler_to_quaternion(odom_yaw, pitch2, 0.0);

            odom_yaw += std::numbers::pi / 2.;
            model_yaw += std::numbers::pi / 2.;
            *target_armors_[1].position << car_x + r1 * std::cos(model_yaw),
                car_y + r1 * std::sin(model_yaw), z1;
            *target_armors_[1].rotation = util::math::euler_to_quaternion(odom_yaw, pitch1, 0.0);

            odom_yaw += std::numbers::pi / 2.;
            model_yaw += std::numbers::pi / 2.;
            *target_armors_[2].position << car_x + r2 * std::cos(model_yaw),
                car_y + r2 * std::sin(model_yaw), z2;
            *target_armors_[2].rotation = util::math::euler_to_quaternion(odom_yaw, pitch2, 0.0);

            odom_yaw += std::numbers::pi / 2.;
            model_yaw += std::numbers::pi / 2.;
            *target_armors_[3].position << car_x + r1 * std::cos(model_yaw),
                car_y + r1 * std::sin(model_yaw), z1;
            *target_armors_[3].rotation = util::math::euler_to_quaternion(odom_yaw, pitch1, 0.0);
        }
    };

    double last_vx = 0., last_vy = 0.;
    double ax = 0., ay = 0.;
    TrackerModel::XVec debug_;
    std::chrono::steady_clock::time_point last_timestamp_{};
    TrackerModel tracker_model_;
    std::array<ArmorPlate3d, 4> target_armors_{};
    TrackerState track_state_{TrackerState::Lost};
};

std::shared_ptr<IFireController> NewTracker::Update(
    const std::vector<ArmorPlate3d>& armors, const std::chrono::steady_clock::time_point& timestamp,
    const rmcs_description::Tf& tf) {
    return pimpl_->Update(armors, timestamp, tf);
};

void NewTracker::draw_armors(const cv::Scalar& color, const rmcs_description::Tf& tf) {
    return pimpl_->draw_armors(color, tf);
};

std::array<ArmorPlate3d, 4> NewTracker::get_armors() { return pimpl_->get_armor(); };
TrackerModel::XVec NewTracker::get_model_output() { return pimpl_->output(); };
NewTracker::NewTracker()
    : pimpl_(std::make_unique<Impl>()) {}
NewTracker::~NewTracker() = default;
} // namespace rmcs_auto_aim::tracker