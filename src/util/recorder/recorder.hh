#pragma once

#include <opencv2/core/mat.hpp>
#include <rclcpp/node.hpp>

namespace rmcs_auto_aim {

class Recorder {
public:
    using TimePoint = std::chrono::steady_clock::time_point;

    explicit Recorder(rclcpp::Node& node);
    ~Recorder();

    bool ready_save(const TimePoint& now);

    void save_image(const cv::Mat& image);

    void set_record_status(bool is_start);

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl;
};

} // namespace rmcs_auto_aim
