
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <initializer_list>
#include <limits>
#include <memory>
#include <vector>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <rmcs_description/tf_description.hpp>

#include "core/pnpsolver/armor/armor3d.hpp"
#include "core/tracker/armor/armor_tracker.hpp"
#include "core/tracker/armor/target.hpp"
#include "core/tracker/armor/track_unit.hpp"

using namespace rmcs_auto_aim;

class ArmorTracker::Impl {
public:
    explicit Impl(const int64_t& predict_duration)
        : predict_duration_(predict_duration) {
        tracker_map_[rmcs_msgs::ArmorID::Hero]        = std::vector<TrackerUnit>{};
        tracker_map_[rmcs_msgs::ArmorID::Engineer]    = std::vector<TrackerUnit>{};
        tracker_map_[rmcs_msgs::ArmorID::InfantryIII] = std::vector<TrackerUnit>{};
        tracker_map_[rmcs_msgs::ArmorID::InfantryIV]  = std::vector<TrackerUnit>{};
        tracker_map_[rmcs_msgs::ArmorID::InfantryV]   = std::vector<TrackerUnit>{};
        tracker_map_[rmcs_msgs::ArmorID::Sentry]      = std::vector<TrackerUnit>{};
        tracker_map_[rmcs_msgs::ArmorID::Outpost]     = std::vector<TrackerUnit>{};
    }

    ~Impl() {}

    std::shared_ptr<ArmorTarget> Update(
        const std::vector<ArmorPlate3d>& armors,
        const std::chrono::steady_clock::time_point& timestamp, const rmcs_description::Tf& tf) {
        //
        // dt: interval between adjacent updates by seconds.
        double dt    = std::chrono::duration<double>(timestamp - last_update_).count();
        last_update_ = timestamp;

        for (auto& [armor_id, tracker_array] : tracker_map_) {
            for (auto iter = tracker_array.begin(); iter != tracker_array.end();) {
                auto& tracker = *iter;
                if (timestamp - tracker.last_update
                    > std::chrono::milliseconds(predict_duration_)) {
                    iter = tracker_array.erase(iter);
                } else {
                    tracker.Predict(dt);
                    ++iter;
                }
            }
        }

        for (const auto& armor : armors) {
            auto iter = tracker_map_.find(armor.id);
            if (iter != tracker_map_.end()) {
                auto& tracker_array = iter->second;

                if (tracker_array.empty()) {
                    tracker_array.emplace_back(armor, timestamp);
                } else {
                    UpdateTracker(tracker_array, armor, timestamp);
                }
            }
        }

        TrackerUnit* selected_tracker = nullptr;
        int selected_level            = 0;
        double minimum_angle          = std::numeric_limits<double>::infinity();
        for (auto& [armor_id, tracker_array] : tracker_map_) {
            for (auto iter = tracker_array.begin(); iter != tracker_array.end();) {
                auto& tracker = *iter;
                int level     = 0;
                if (tracker.tracking_density > 100) {
                    level = 2;
                } else if (tracker.tracking_density > 40) {
                    level = 1;
                }

                auto center = *fast_tf::cast<rmcs_description::MuzzleLink>(
                    rmcs_description::OdomImu::Position{
                        tracker.imm.getState()(0), tracker.imm.getState()(2),
                        tracker.imm.getState()(4)},
                    tf);

                if (center.norm() <= 0.8) {
                    tracker.collision = true;
                } else {
                    tracker.collision = false;
                }

                double angle = std::acos(center.dot(Eigen::Vector3d{1, 0, 0}) / center.norm());

                if (angle < minimum_angle) {
                    minimum_angle    = angle;
                    selected_tracker = &tracker;
                    selected_level   = level;
                } else if (
                    !std::isinf(angle) && fabs(angle - minimum_angle) < 1e-3
                    && selected_level < level) {
                    selected_level = level;
                    minimum_angle  = angle;

                    selected_tracker = &tracker;
                }
                ++iter;
            }
        }
        if (selected_tracker) {
            return std::make_shared<rmcs_auto_aim::ArmorTarget>(*selected_tracker);
        } else {
            return nullptr;
        }
    }

private:
    // untested code start
    static inline void UpdateTracker(
        std::vector<TrackerUnit>& tracker_array, const ArmorPlate3d& armor,
        const std::chrono::steady_clock::time_point& timestamp) {
        double min_distance   = std::numeric_limits<double>::infinity();
        size_t selected_index = 0;
        for (int index = 0; auto& tracker : tracker_array) {
            const auto vector =
                tracker.measurement_pos
                - Eigen::Vector3d{armor.position->x(), armor.position->y(), armor.position->z()};
            const auto distance = vector.norm();
            if (distance < min_distance) {
                min_distance   = distance;
                selected_index = index;
            }
            index++;
        }
        tracker_array[selected_index].Update(armor, timestamp);
    }
    // end

    const int64_t predict_duration_;

    std::chrono::steady_clock::time_point last_update_;

    std::map<rmcs_msgs::ArmorID, std::vector<TrackerUnit>> tracker_map_;
};

ArmorTracker::ArmorTracker(const int64_t& predict_duration)
    : pImpl_(new Impl{predict_duration}) {}

std::shared_ptr<ArmorTarget> ArmorTracker::Update(
    const std::vector<ArmorPlate3d>& armors, const std::chrono::steady_clock::time_point& timestamp,
    const rmcs_description::Tf& tf) {
    return pImpl_->Update(armors, timestamp, tf);
}

ArmorTracker::~ArmorTracker() = default;