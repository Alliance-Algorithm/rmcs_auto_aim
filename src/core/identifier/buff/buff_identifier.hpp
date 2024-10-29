#pragma once
#include <memory>
#include <optional>

#include <rmcs_msgs/robot_color.hpp>

#include "core/identifier/buff/buff.hpp"

namespace rmcs_auto_aim {

class BuffIdentifier {
public:
    explicit BuffIdentifier(const std::string& model_path);
    ~BuffIdentifier();

    std::optional<BuffPlate> Identify(const cv::Mat& img);

private:
    class Impl;
    std::unique_ptr<Impl> pImpl_;
};
}