/**
 * @file buff_tracker.hpp
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

#include "core/pnpsolver/buff/buff3d.hpp"
#include "core/tracker/tracker.hpp"

namespace rmcs_auto_aim {
class BuffTracker : public TrackerInterface {
public:
    explicit BuffTracker(int64_t predict_duration);
    ~BuffTracker();

    std::shared_ptr<TargetInterface>
        Update(const BuffPlate3d& buff, std::chrono::steady_clock::time_point timestamp);

    void ResetAll(const rmcs_description::Tf& tf);

private:
    class Impl;
    std::unique_ptr<Impl> pImpl_;
};
} // namespace rmcs_auto_aim
