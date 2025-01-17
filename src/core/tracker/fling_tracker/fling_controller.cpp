#include "fling_controller.hpp"
#include "core/tracker/fling_tracker/armor_tracker/armor_ekf.hpp"
#include "core/tracker/fling_tracker/armor_tracker/armor_processor.hpp"
#include "core/tracker/fling_tracker/armor_tracker/armor_target.hpp"
#include "core/tracker/fling_tracker/car_tracker/car_tracker.hpp"
#include "core/transform_optimizer/armor/quadrilateral/quadrilateral.hpp"
#include <map>
#include <robot_id.hpp>

using namespace rmcs_auto_aim::tracker::fling_tracker;
class rmcs_auto_aim::tracker::FLiNGTracker::Impl {
public:
    Impl() {
        car_trackers[rmcs_msgs::ArmorID::Hero]        = std::make_shared<CarTracker>();
        car_trackers[rmcs_msgs::ArmorID::Engineer]    = std::make_shared<CarTracker>();
        car_trackers[rmcs_msgs::ArmorID::InfantryIII] = std::make_shared<CarTracker>();
        car_trackers[rmcs_msgs::ArmorID::InfantryIV]  = std::make_shared<CarTracker>();
        car_trackers[rmcs_msgs::ArmorID::InfantryV]   = std::make_shared<CarTracker>();
        car_trackers[rmcs_msgs::ArmorID::Sentry]      = std::make_shared<CarTracker>();
        car_trackers[rmcs_msgs::ArmorID::Outpost]     = std::make_shared<CarTracker>();

        armor_trackers[rmcs_msgs::ArmorID::Hero]        = std::vector<ArmorEKF>{};
        armor_trackers[rmcs_msgs::ArmorID::Engineer]    = std::vector<ArmorEKF>{};
        armor_trackers[rmcs_msgs::ArmorID::InfantryIII] = std::vector<ArmorEKF>{};
        armor_trackers[rmcs_msgs::ArmorID::InfantryIV]  = std::vector<ArmorEKF>{};
        armor_trackers[rmcs_msgs::ArmorID::InfantryV]   = std::vector<ArmorEKF>{};
        armor_trackers[rmcs_msgs::ArmorID::Sentry]      = std::vector<ArmorEKF>{};
        armor_trackers[rmcs_msgs::ArmorID::Outpost]     = std::vector<ArmorEKF>{};
        for (int i = 0; i < 4; i++)
            for (auto& [_, armors] : armor_trackers)
                armors.emplace_back();

        grouped_armor[rmcs_msgs::ArmorID::Hero]        = {};
        grouped_armor[rmcs_msgs::ArmorID::Engineer]    = {};
        grouped_armor[rmcs_msgs::ArmorID::InfantryIII] = {};
        grouped_armor[rmcs_msgs::ArmorID::InfantryIV]  = {};
        grouped_armor[rmcs_msgs::ArmorID::InfantryV]   = {};
        grouped_armor[rmcs_msgs::ArmorID::Sentry]      = {};
        grouped_armor[rmcs_msgs::ArmorID::Outpost]     = {};

        index1_[rmcs_msgs::ArmorID::Hero]        = 0;
        index1_[rmcs_msgs::ArmorID::Engineer]    = 0;
        index1_[rmcs_msgs::ArmorID::InfantryIII] = 0;
        index1_[rmcs_msgs::ArmorID::InfantryIV]  = 0;
        index1_[rmcs_msgs::ArmorID::InfantryV]   = 0;
        index1_[rmcs_msgs::ArmorID::Sentry]      = 0;
        index1_[rmcs_msgs::ArmorID::Outpost]     = 0;

        index2_[rmcs_msgs::ArmorID::Hero]        = 0;
        index2_[rmcs_msgs::ArmorID::Engineer]    = 0;
        index2_[rmcs_msgs::ArmorID::InfantryIII] = 0;
        index2_[rmcs_msgs::ArmorID::InfantryIV]  = 0;
        index2_[rmcs_msgs::ArmorID::InfantryV]   = 0;
        index2_[rmcs_msgs::ArmorID::Sentry]      = 0;
        index2_[rmcs_msgs::ArmorID::Outpost]     = 0;
    };

    std::shared_ptr<ITarget> Update(
        const std::vector<ArmorPlate3d>& armors,
        const std::chrono::steady_clock::time_point& timestamp, const rmcs_description::Tf& tf,
        const cv::Mat&) {

        std::shared_ptr<ITarget> target = nullptr;

        double dt    = std::chrono::duration<double>(timestamp - last_update_).count();
        last_update_ = timestamp;
        dt           = std::clamp(dt, 0., .5);

        armor::prcessor::get_grouped_armor(grouped_armor, armors);

        for (const auto& [armorID, car] : car_trackers) {
            auto len = grouped_armor[armorID].size();
            ArmorEKF::ZVec z_armor{};
            z_armor.setIdentity();
            if (len == 1) {
                z_armor = single_armor_update(
                    grouped_armor[armorID], car, armorID, tf, dt, car->get_omega() >= 0.0);
            } else if (len == 2) {
                z_armor = double_armor_update(grouped_armor[armorID], car, armorID, tf, dt);
            } else {
                car->update_self(dt);
                continue;
            }

            last_car_id_ = armorID;
            Eigen::Vector<double, 3> z_car{};
            z_car << z_armor(0), z_armor(1), z_armor(3);

            auto car_z = car->get_z();

            car->update_car(z_car, dt);
            for (int i = 0; i < 4; i++)
                if (i == index1_[armorID] || i == index2_[armorID])
                    car_z(i) = armor::prcessor::armor_get_odom_z(armor_trackers[armorID][i], tf);
            car->update_z(car_z);
            Eigen::Vector2d vec{};

            target = std::make_shared<fling_tracker::armor::ArmorTarget>(
                CarTracker{*car}, index1_[armorID]);

            last_armors1 = car_trackers[last_car_id_]->get_armor(dt);
            last_armors2 = car_trackers[last_car_id_]->get_armor();
        }
        return target;
    }

    void draw_armors(const rmcs_description::Tf& tf, const cv::Scalar& color) {
        if (last_armors1.empty())
            return;
        auto color_ = color;
        for (const auto& armor3d : last_armors1) {
            util::ImageViewer::draw(
                transform_optimizer::Quadrilateral3d(armor3d).ToQuadrilateral(tf, false), color_);
            color_ = color_ / 1.2;
        }
        for (const auto& armor3d : last_armors2) {
            util::ImageViewer::draw(
                transform_optimizer::Quadrilateral3d(armor3d).ToQuadrilateral(tf, false),
                {0, 0, 255});
        }
    }

private:
    ArmorEKF::ZVec single_armor_update(
        const std::vector<ArmorPlate3d>& armors, const std::shared_ptr<CarTracker>& car,
        const rmcs_msgs::ArmorID& id, const rmcs_description::Tf& tf, const double& dt,
        const bool& clockwise) {
        auto car_armors = car->get_armor(dt);
        int index       = index1_[id];

        if (car->check_armor_tracked() && index1_ == index2_)
            index = index1_[id];
        else {
            for (auto& armor : armors) {
                double min = 1e7;
                for (int i = 0; i < 4; i++) {
                    if (armor::prcessor::distance_in_four(i, index1_[id] + (clockwise ? 1 : -1)) > 0
                        && armor::prcessor::distance_in_four(i, index1_[id]) > 0)
                        continue;
                    auto distance = armor::prcessor::calculate_similarity(
                        *car_armors[i].position, Eigen::Quaterniond{*car_armors[i].rotation},
                        *armor.position, Eigen::Quaterniond{*armor.rotation});
                    if (distance < min) {
                        index = i;
                        min   = distance;
                    }
                }
            }
        }

        index1_[id] = index2_[id] = index;

        auto& tracker = armor_trackers[id][index];

        ArmorEKF::ZVec z{};
        z.setZero();
        auto a0_position = fast_tf::cast<rmcs_description::CameraLink>(armors[0].position, tf);

        auto [l1, l2] = car->get_frame();
        auto l        = l1;
        if (index % 2 != 0)
            l = l2;
        z << a0_position->x(), a0_position->y(), a0_position->z(),
            util::math::get_yaw_from_quaternion(*armors[0].rotation);

        tracker.Update(z, {}, dt);
        Eigen::Vector<double, 4> z1            = tracker.h(tracker.OutPut(), {});
        rmcs_description::OdomImu::Position p1 = armor::prcessor::armor_to_car(z, l);
        return {p1->x(), p1->y(), p1->z(), z1(3) + -index * std::numbers::pi / 2};
    }

    ArmorEKF::ZVec double_armor_update(
        const std::vector<ArmorPlate3d>& armors, const std::shared_ptr<CarTracker>& car,
        const rmcs_msgs::ArmorID& id, const rmcs_description::Tf& tf, const double& dt) {
        auto& armor1 = armors[0];
        auto& armor2 = armors[1];

        Eigen::Vector2d p1 = {armor1.position->x(), armor1.position->y()};
        Eigen::Vector2d p2 = {armor2.position->x(), armor2.position->y()};
        Eigen::Vector3d f1{};
        f1 << armor1.rotation->toRotationMatrix() * Eigen::Vector3d::UnitX();
        Eigen::Vector3d f2{};
        f2 << armor2.rotation->toRotationMatrix() * Eigen::Vector3d::UnitX();
        Eigen::Vector2d diffp{};
        diffp << p1 - p2;
        auto l1 = abs(diffp.dot(Eigen::Vector2d{f1.x(), f1.y()}.normalized()));
        auto l2 = abs(diffp.dot(Eigen::Vector2d{f2.x(), f2.y()}.normalized()));

        auto car_armors = car->get_armor(dt);
        int index1 = index1_[id], index2 = index2_[id];
        double min1 = 1e7;
        double min2 = 1e7;
        if (car->check_armor_tracked() && index1_ != index2_) {
        } else {
            do {
                if (index1 != index2)
                    break;
                for (int i = 0; i < 4; i++) {
                    if (armor::prcessor::distance_in_four(i, index1) > 1)
                        continue;
                    auto distance = armor::prcessor::calculate_similarity(
                        *car_armors[i].position, Eigen::Quaterniond{*car_armors[i].rotation},
                        *armor1.position, Eigen::Quaterniond{*armor1.rotation});
                    if (distance < min1) {
                        index1 = i;
                        min1   = distance;
                    }
                }
                for (int i = 0; i < 4; i++) {
                    if (armor::prcessor::distance_in_four(i, index2) > 1)
                        continue;
                    auto distance = armor::prcessor::calculate_similarity(
                        *car_armors[i].position, Eigen::Quaterniond{*car_armors[i].rotation},
                        *armor2.position, Eigen::Quaterniond{*armor2.rotation});
                    if (distance < min2) {
                        index2 = i;
                        min2   = distance;
                    }
                }

            } while (false);
        }

        index1_[id] = index1;
        index2_[id] = index2;
        if (index1 % 2 != 0)
            std::swap(l1, l2);
        car->update_frame(l1, l2);
        auto [l1_, l2_] = car->get_frame();
        l1              = l1_;
        l2              = l2_;

        if (index1 % 2 != 0)
            std::swap(l1, l2);

        auto& tracker = armor_trackers[id][index1];
        ArmorEKF::ZVec z{};
        auto a1_position = fast_tf::cast<rmcs_description::CameraLink>(armor1.position, tf);
        z << a1_position->x(), a1_position->y(), a1_position->z(),
            util::math::get_yaw_from_quaternion(*armors[0].rotation);

        tracker.Update(z, {}, dt);
        Eigen::Vector<double, 4> z1              = tracker.h(tracker.OutPut(), {});
        rmcs_description::OdomImu::Position pos1 = armor::prcessor::armor_to_car(z, l1);
        return {pos1->x(), pos1->y(), pos1->z(), z1(3) + -index1 * std::numbers::pi / 2};
    }

    std::map<rmcs_msgs::ArmorID, int> index1_;
    std::map<rmcs_msgs::ArmorID, int> index2_;
    std::map<rmcs_msgs::ArmorID, std::shared_ptr<CarTracker>> car_trackers;
    std::map<rmcs_msgs::ArmorID, std::vector<ArmorEKF>> armor_trackers;
    std::chrono::steady_clock::time_point last_update_;
    std::map<rmcs_msgs::ArmorID, std::vector<ArmorPlate3d>> grouped_armor;
    rmcs_msgs::ArmorID last_car_id_ = rmcs_msgs::ArmorID::Hero;

    std::vector<ArmorPlate3d> last_armors1;
    std::vector<ArmorPlate3d> last_armors2;
};

rmcs_auto_aim::tracker::FLiNGTracker::FLiNGTracker() { pimpl_ = std::make_unique<Impl>(); }
rmcs_auto_aim::tracker::FLiNGTracker ::~FLiNGTracker() = default;

std::shared_ptr<rmcs_auto_aim::tracker::ITarget> rmcs_auto_aim::tracker::FLiNGTracker::Update(
    const std::vector<ArmorPlate3d>& armors, const std::chrono::steady_clock::time_point& timestamp,
    const rmcs_description::Tf& tf, const cv::Mat& image) {
    return pimpl_->Update(armors, timestamp, tf, image);
};
void rmcs_auto_aim::tracker::FLiNGTracker::draw_armors(
    const rmcs_description::Tf& tf, const cv::Scalar& color) const {
    pimpl_->draw_armors(tf, color);
}