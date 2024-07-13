/**
 * @file recorder.hpp
 * @author Lorenzo Feng (lorenzo.feng@njust.edu.cn)
 * @brief
 * @version 0.1
 * @date 2024-07-12
 *
 * (C)Copyright: NJUST.Alliance - All rights reserved
 *
 */

#pragma once

#include <memory>

#include <opencv2/opencv.hpp>

namespace rmcs_auto_aim {
class Recorder {
public:
    explicit Recorder(const double& fps, const cv::Size& size);
    ~Recorder();
    bool record_frame(const cv::Mat& frame);
    bool is_opened() const;

private:
    class Impl;
    std::unique_ptr<Impl> pImpl_;
};
} // namespace rmcs_auto_aim