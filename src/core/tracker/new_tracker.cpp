#include "new_tracker.hpp"
#include "core/transform_optimizer/armor/quadrilateral/quadrilateral.hpp"
#include "tracker_model.hpp"
#include "tracker_state.hpp"
#include "util/image_viewer/image_viewer.hpp"
#include "util/math.hpp"

namespace rmcs_auto_aim::tracker {

class NewTracker::Impl {
public:
    // 火控要做的事情就是挑最近的打
    inline std::shared_ptr<IFireController> Update(
        const std::vector<ArmorPlate3d>& armors,
        const std::chrono::steady_clock::time_point& timestamp, const rmcs_description::Tf& tf) {
        const auto dt =
            std::chrono::duration_cast<std::chrono::duration<double>>(timestamp - last_timestamp_)
                .count();
        if (armors.empty()) {
            if (dt > 1.0)
                track_state_ = TrackerState::Lost;
        } else {
            if (track_state_ == TrackerState::Lost) {
                last_timestamp_ = timestamp;
                track_state_    = TrackerState::Track;
            } else {
                TrackerModel::ZVec input;
                // WARNING: PNP 解算时就要丢掉不合理的值,具体见 TrackerModel
                input << util::math::get_yaw_from_quaternion(*armors[0].rotation),
                    util::math::get_pitch_from_quaternion(*armors[0].rotation),
                    armors[0].position->norm();
                tracker_model_.Update(input, {}, dt);
                if (armors.size() >= 2) {
                    input << util::math::get_yaw_from_quaternion(*armors[1].rotation),
                        util::math::get_pitch_from_quaternion(*armors[1].rotation),
                        armors[1].position->norm();
                    tracker_model_.Update(input, {}, 0);
                }
            };
        }

        // 预测时长，这里主要是把各种延迟加上,包括计算延迟，卡弹延迟以及子弹飞行时间等
        if (track_state_ == TrackerState::Track)
            predict(dt);
        last_timestamp_ = timestamp;
        return nullptr;
    };

    inline void draw_armors(const cv::Scalar& color, const rmcs_description::Tf& tf) {
        for (const auto& armor : target_armors_)
            util::ImageViewer::draw(
                transform_optimizer::Quadrilateral3d(armor).ToQuadrilateral(tf, true), color);
    };

private:
    void predict(const double& dt) {
        // 两种预测模型可选，一种是匀加速，一种是匀速
        const auto model_output = tracker_model_.OutPut();

        // 匀速预测如下
        // const double car_x = model_output(0) + model_output(1) * dt;
        // const double car_y = model_output(3) + model_output(4) * dt;

        // 匀加速预测模型如下
        const double car_x =
            model_output(0) + ((2. * model_output(1) + dt * model_output(2)) / 2. * dt);
        const double car_y =
            model_output(3) + ((2. * model_output(4) + dt * model_output(5)) / 2. * dt);

        const double car_z = model_output(6);
        const double z1    = model_output(7);
        const double z2    = model_output(8);
        const double r1    = model_output(9);
        const double r2    = model_output(10);
        double model_yaw   = model_output(11) + model_output(12) * dt;
        double odom_yaw    = model_yaw;

        // 模型里对yaw的角度进行了一个线性的映射，这里要映射回去以求出装甲板的方位角
        while (odom_yaw <= -std::numbers::pi / 2.)
            odom_yaw += std::numbers::pi * 2.;
        while (odom_yaw >= std::numbers::pi * 3. / 2.)
            odom_yaw -= std::numbers::pi * 2.;

        if (odom_yaw >= -std::numbers::pi / 2. && odom_yaw <= std::numbers::pi / 2.)
            odom_yaw += std::numbers::pi / 2.;
        else
            odom_yaw -= 3. * std::numbers::pi / 2.;

        const double pitch1 = 15. / 180. * std::numbers::pi - std::atan((car_z - z1) / r1);
        const double pitch2 = 15. / 180. * std::numbers::pi - std::atan((car_z - z2) / r2);

        if (tracker_model_.get_side_flag_()) {
            *target_armors_[0].position << car_x - r1 * std::sin(model_yaw),
                car_y - r1 * std::cos(model_yaw), z1;
            *target_armors_[0].rotation = util::math::euler_to_quaternion(odom_yaw, pitch1, 0.0);

            odom_yaw += std::numbers::pi / 2.;
            model_yaw += std::numbers::pi / 2.;
            *target_armors_[1].position << car_x - r2 * std::sin(model_yaw),
                car_y - r2 * std::cos(model_yaw), z2;
            *target_armors_[1].rotation = util::math::euler_to_quaternion(odom_yaw, pitch2, 0.0);

            odom_yaw += std::numbers::pi / 2.;
            model_yaw += std::numbers::pi / 2.;
            *target_armors_[2].position << car_x - r1 * std::sin(model_yaw),
                car_y - r1 * std::cos(model_yaw), z1;
            *target_armors_[2].rotation = util::math::euler_to_quaternion(odom_yaw, pitch1, 0.0);

            odom_yaw += std::numbers::pi / 2.;
            model_yaw += std::numbers::pi / 2.;
            *target_armors_[3].position << car_x - r2 * std::sin(model_yaw),
                car_y - r2 * std::cos(model_yaw), z2;
            *target_armors_[3].rotation = util::math::euler_to_quaternion(odom_yaw, pitch2, 0.0);
        } else {
            *target_armors_[0].position << car_x - r2 * std::sin(model_yaw),
                car_y - r2 * std::cos(model_yaw), z2;
            *target_armors_[0].rotation = util::math::euler_to_quaternion(odom_yaw, pitch2, 0.0);

            odom_yaw += std::numbers::pi / 2.;
            model_yaw += std::numbers::pi / 2.;
            *target_armors_[1].position << car_x - r1 * std::sin(model_yaw),
                car_y - r1 * std::cos(model_yaw), z1;
            *target_armors_[1].rotation = util::math::euler_to_quaternion(odom_yaw, pitch1, 0.0);

            odom_yaw += std::numbers::pi / 2.;
            model_yaw += std::numbers::pi / 2.;
            *target_armors_[2].position << car_x - r2 * std::sin(model_yaw),
                car_y - r2 * std::cos(model_yaw), z2;
            *target_armors_[2].rotation = util::math::euler_to_quaternion(odom_yaw, pitch2, 0.0);

            odom_yaw += std::numbers::pi / 2.;
            model_yaw += std::numbers::pi / 2.;
            *target_armors_[3].position << car_x - r1 * std::sin(model_yaw),
                car_y - r1 * std::cos(model_yaw), z1;
            *target_armors_[3].rotation = util::math::euler_to_quaternion(odom_yaw, pitch1, 0.0);
        }
    };

    std::chrono::steady_clock::time_point last_timestamp_{};
    TrackerModel tracker_model_;
    std::array<ArmorPlate3d, 4> target_armors_;
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
} // namespace rmcs_auto_aim::tracker