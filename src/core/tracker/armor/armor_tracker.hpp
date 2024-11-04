/**
 * @file armor_tracker.hpp
 * @author Lorenzo Feng (lorenzo.feng@njust.edu.cn)
 * @brief
 * @version 0.1
 * @date 2024-06-06
 *
 * (C)Copyright: NJUST.Alliance - All rights reserved
 *
 */

#pragma once

#include <chrono>
#include <cstdint>
#include <memory>

#include "core/pnpsolver/armor/armor3d.hpp"
#include "core/tracker/armor/target.hpp"

namespace rmcs_auto_aim {

class ArmorTracker {
public:
    explicit ArmorTracker(const int64_t& predict_duration);
    ~ArmorTracker();

    std::shared_ptr<ArmorTarget> Update(
        const std::vector<ArmorPlate3d>& armors,
        const std::chrono::steady_clock::time_point& timestamp, const rmcs_description::Tf& tf);

private:
    class Impl;
    std::unique_ptr<Impl> pImpl_;
};

} // namespace rmcs_auto_aim