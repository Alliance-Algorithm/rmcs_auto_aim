/**
 * @file debugger.hpp
 * @author Lorenzo Feng (lorenzo.feng@njust.edu.cn)
 * @brief
 * @version 0.1
 * @date 2024-07-04
 *
 * (C)Copyright: NJUST.Alliance - All rights reserved
 *
 */

#pragma once

#include "core/identifier/armor/armor.hpp"
#include "core/pnpsolver/armor/armor3d.hpp"
#include <memory>
#include <opencv2/core/mat.hpp>
#include <vector>

class Debugger {
public:
    void publish_raw_image(const cv::Mat& img);
    void publish_armors(const std::vector<auto_aim::ArmorPlate>& armors, const cv::Mat& img);
    void publish_3d_armors(const std::vector<auto_aim::ArmorPlate3d>& armors);
    void publish_raw_roi(const cv::Mat& img);

private:
    class Impl;
    std::unique_ptr<Impl> pImpl_;
};