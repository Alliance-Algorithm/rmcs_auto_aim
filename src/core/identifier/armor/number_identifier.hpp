/**
 * @file number_identifier.hpp
 * @author Lorenzo Feng (lorenzo.feng@njust.edu.cn)
 * @brief
 * @version 0.1
 * @date 2024-06-07
 *
 * (C)Copyright: NJUST.Alliance - All rights reserved
 *
 */
#pragma once

#include <string>

#include <opencv2/opencv.hpp>

#include "core/identifier/armor/armor.hpp"

namespace rmcs_auto_aim {
class NumberIdentifier {
private:
    cv::dnn::Net _net;

public:
    explicit NumberIdentifier(const std::string& model_path);
    NumberIdentifier(const NumberIdentifier&) = delete;
    NumberIdentifier(NumberIdentifier&&)      = delete;

    bool Identify(const cv::Mat& imgGray, ArmorPlate& armor, const uint8_t& whitelist);
};
} // namespace rmcs_auto_aim