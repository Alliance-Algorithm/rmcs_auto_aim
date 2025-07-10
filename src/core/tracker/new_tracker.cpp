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
            if (track_state_ == TrackerState::Track)
                predict(dt + 0.2);
            if (dt > 1.0)
                track_state_ = TrackerState::Lost;
            return nullptr;
        } else {
            if (track_state_ == TrackerState::Lost) {
                last_timestamp_ = timestamp;
                track_state_    = TrackerState::Track;
            } else {
                if (armors.size() >= 2) {
                    int index1 = 0, index2 = 1;
                    double index1_yaw, index2_yaw;
                    double min_diff{std::numeric_limits<double>::max()};
                    // 找yaw差值最接近std::numbers::pi / 2.的装甲板来更新车架参数
                    for (int i = 0; i < static_cast<int>(armors.size()); i++) {
                        for (int j = i + 1; j < static_cast<int>(armors.size()); j++) {
                            double armor1_yaw =
                                util::math::get_yaw_from_quaternion(*armors[i].rotation);
                            double armor2_yaw =
                                util::math::get_yaw_from_quaternion(*armors[j].rotation);
                            if (armor1_yaw <= 0)
                                armor1_yaw += std::numbers::pi * 2.;
                            if (armor2_yaw <= 0)
                                armor2_yaw += std::numbers::pi * 2.;
                            const auto yaw_diff =
                                std::abs(std::abs(armor2_yaw - armor1_yaw) - std::numbers::pi / 2.);
                            if (yaw_diff <= min_diff) {
                                index1_yaw = armor1_yaw, index2_yaw = armor2_yaw;
                                index1 = i, index2 = j;
                                min_diff = yaw_diff;
                            }
                        }
                    }

                    int nearst_index, another_index;

                    if (std::abs(index1_yaw) > std::abs(index2_yaw))
                        nearst_index = index1, another_index = index2;
                    else
                        nearst_index = index2, another_index = index1;
                    const auto r_n = calculate_r(armors[nearst_index], armors[another_index]);
                    r[0]           = r[0] + (0.5) * (std::get<0>(r_n) - r[0]),
                    r[1]           = r[1] + (0.5) * (std::get<1>(r_n) - r[1]);

                    TrackerModel::ZVec input;
                    const auto processed_theta{process_theta(
                        util::math::get_yaw_from_quaternion(*armors[nearst_index].rotation))};
                    input << processed_theta,
                        std::atan(
                            armors[nearst_index].position->y()
                            / armors[nearst_index].position->x()),
                        -std::atan(
                            armors[nearst_index].position->z()
                            / armors[nearst_index].position->x()),
                        armors[nearst_index].position->norm();
                    switch_logic(processed_theta, tf);
                    if (side_flag_) {
                        z[0] = fast_tf::cast<rmcs_description::OdomImu>(
                                   armors[nearst_index].position, tf)
                                   ->z();
                        z[1] = fast_tf::cast<rmcs_description::OdomImu>(
                                   armors[another_index].position, tf)
                                   ->z();
                    } else {
                        z[0] = fast_tf::cast<rmcs_description::OdomImu>(
                                   armors[another_index].position, tf)
                                   ->z();
                        z[1] = fast_tf::cast<rmcs_description::OdomImu>(
                                   armors[nearst_index].position, tf)
                                   ->z();
                    }
                    tracker_model_.set_z_r(z[0], z[1], r[0], r[1]);
                    tracker_model_.Update(input, {}, dt, tf);
                    last_theta_ = processed_theta;
                } else {
                    TrackerModel::ZVec input;
                    const auto processed_theta{
                        process_theta(util::math::get_yaw_from_quaternion(*armors[0].rotation))};
                    input << processed_theta,
                        std::atan(armors[0].position->y() / armors[0].position->x()),
                        -std::atan(armors[0].position->z() / armors[0].position->x()),
                        armors[0].position->norm();
                    switch_logic(processed_theta, tf);
                    if (side_flag_)
                        z[0] = armors[0].position->z();
                    else
                        z[1] = armors[0].position->z();
                    tracker_model_.set_z_r(z[0], z[1], r[0], r[1]);
                    tracker_model_.Update(input, {}, dt, tf);
                    last_theta_ = processed_theta;
                }
            };
        }

        // 预测时长，这里主要是把各种延迟加上,包括计算延迟，卡弹延迟以及子弹飞行时间等
        if (track_state_ == TrackerState::Track)
            predict(0.);

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

        const double car_x = model_output(0) + model_output(1) * dt;
        const double car_y = model_output(2) + model_output(3) * dt;

        const double car_z = model_output(4);
        const double z1    = model_output(5);
        const double z2    = model_output(6);
        const double r1    = model_output(7);
        const double r2    = model_output(8);
        double model_yaw   = model_output(9) + model_output(10) * dt;
        // std::cerr << "yaw:" << model_yaw * 180. / std::numbers::pi << std::endl;
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

        const double pitch1 = -15. / 180. * std::numbers::pi;
        const double pitch2 = -15. / 180. * std::numbers::pi;

        if (side_flag_) {
            *target_armors_[0].position << car_x - r1 * std::sin(model_yaw),
                car_y + r1 * std::cos(model_yaw), z1;
            *target_armors_[0].rotation = util::math::euler_to_quaternion(odom_yaw, pitch1, 0.0);

            odom_yaw += std::numbers::pi / 2.;
            model_yaw += std::numbers::pi / 2.;
            *target_armors_[1].position << car_x - r2 * std::sin(model_yaw),
                car_y + r2 * std::cos(model_yaw), z2;
            *target_armors_[1].rotation = util::math::euler_to_quaternion(odom_yaw, pitch2, 0.0);

            odom_yaw += std::numbers::pi / 2.;
            model_yaw += std::numbers::pi / 2.;
            *target_armors_[2].position << car_x - r1 * std::sin(model_yaw),
                car_y + r1 * std::cos(model_yaw), z1;
            *target_armors_[2].rotation = util::math::euler_to_quaternion(odom_yaw, pitch1, 0.0);

            odom_yaw += std::numbers::pi / 2.;
            model_yaw += std::numbers::pi / 2.;
            *target_armors_[3].position << car_x - r2 * std::sin(model_yaw),
                car_y + r2 * std::cos(model_yaw), z2;
            *target_armors_[3].rotation = util::math::euler_to_quaternion(odom_yaw, pitch2, 0.0);
        } else {
            *target_armors_[0].position << car_x - r2 * std::sin(model_yaw),
                car_y + r2 * std::cos(model_yaw), z2;
            *target_armors_[0].rotation = util::math::euler_to_quaternion(odom_yaw, pitch2, 0.0);

            odom_yaw += std::numbers::pi / 2.;
            model_yaw += std::numbers::pi / 2.;
            *target_armors_[1].position << car_x - r1 * std::sin(model_yaw),
                car_y + r1 * std::cos(model_yaw), z1;
            *target_armors_[1].rotation = util::math::euler_to_quaternion(odom_yaw, pitch1, 0.0);

            odom_yaw += std::numbers::pi / 2.;
            model_yaw += std::numbers::pi / 2.;
            *target_armors_[2].position << car_x - r2 * std::sin(model_yaw),
                car_y + r2 * std::cos(model_yaw), z2;
            *target_armors_[2].rotation = util::math::euler_to_quaternion(odom_yaw, pitch2, 0.0);

            odom_yaw += std::numbers::pi / 2.;
            model_yaw += std::numbers::pi / 2.;
            *target_armors_[3].position << car_x - r1 * std::sin(model_yaw),
                car_y + r1 * std::cos(model_yaw), z1;
            *target_armors_[3].rotation = util::math::euler_to_quaternion(odom_yaw, pitch1, 0.0);
        }
    };

    static inline std::tuple<double, double>
        calculate_r(const ArmorPlate3d& plate1, const ArmorPlate3d& plate2) {

        const auto plate1_yaw{util::math::get_yaw_from_quaternion(*plate1.rotation)};
        const auto plate2_yaw{util::math::get_yaw_from_quaternion(*plate2.rotation)};
        if (plate1_yaw != std::numbers::pi / 2. && plate1_yaw != -std::numbers::pi / 2.
            && plate2_yaw != std::numbers::pi / 2. && plate2_yaw != -std::numbers::pi / 2.)
            [[__likely__]] {
            const auto x1 = plate1.position->x();
            const auto y1 = plate1.position->y();
            const auto k1 = tan(plate1_yaw);
            const auto x2 = plate2.position->x();
            const auto y2 = plate2.position->y();
            const auto k2 = tan(plate2_yaw);

            const auto x = (y2 - y1 + k1 * x1 - k2 * x2) / (k1 - k2);
            const auto y = k1 * (x - x1) + y1;

            return {
                std::sqrt((x - x1) * (x - x1) + (y - y1) * (y - y1)),
                std::sqrt((x - x2) * (x - x2) + (y - y2) * (y - y2))};
        } else if (plate1_yaw == std::numbers::pi / 2. || plate1_yaw == -std::numbers::pi / 2.) {
            const auto x1 = plate1.position->x();
            const auto y1 = plate1.position->y();
            const auto x2 = plate2.position->x();
            const auto y2 = plate2.position->y();
            const auto k2 = tan(plate2_yaw);

            const auto y = k2 * (x1 - x2) + y2;

            return {std::abs(y1), std::sqrt((x1 - x2) * (x1 - x2) + (y - y2) * (y - y2))};
        } else if (plate2_yaw == std::numbers::pi / 2. || plate2_yaw == -std::numbers::pi / 2.) {
            const auto x1 = plate1.position->x();
            const auto y1 = plate1.position->y();
            const auto k1 = tan(plate1_yaw);
            const auto x2 = plate2.position->x();
            const auto y2 = plate2.position->y();

            const auto y = k1 * (x2 - x1) + y1;

            return {std::sqrt((x2 - x1) * (x2 - x1) + (y - y1) * (y - y1)), std::abs(y2)};
        }
    }

    static inline double process_theta(const double& theta) {
        if (theta >= 0.)
            return theta - std::numbers::pi / 2.;
        else
            return theta + 3 * std::numbers::pi / 2.;
    }

    inline void switch_logic(const double& theta, const rmcs_description::Tf& tf) {

        if (std::abs(last_theta_ - theta) >= switch_angle_difference_) {
            side_flag_ = !side_flag_;
            if (theta > last_theta_)
                tracker_model_.set_theta_offset(-std::numbers::pi / 2.);
            else
                tracker_model_.set_theta_offset(std::numbers::pi / 2.);
        } else
            tracker_model_.set_theta_offset(0.);
        tracker_model_.set_side_flag_(side_flag_);
        tracker_model_.process_yaw_offset(*fast_tf::cast<rmcs_description::OdomImu>(
            rmcs_description::CameraLink::Rotation{}, tf));
    }

    static constexpr double switch_angle_difference_ = std::numbers::pi / 4.;
    TrackerModel::XVec debug_;
    double last_theta_{std::numbers::pi / 2.};
    bool side_flag_{true};
    std::chrono::steady_clock::time_point last_timestamp_{};
    TrackerModel tracker_model_;
    double r[2] = {0.2, 0.2}, z[2] = {0.15, 0.15};
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