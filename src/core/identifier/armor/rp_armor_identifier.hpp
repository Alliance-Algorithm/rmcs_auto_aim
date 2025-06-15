#pragma once
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core/mat.hpp>

#include <rmcs_msgs/robot_color.hpp>

#include "core/identifier/armor/armor.hpp"

namespace rmcs_auto_aim {

class RPArmorIdentifier {
public:
    explicit RPArmorIdentifier(const std::string& model_path, const std::string& device);
    ~RPArmorIdentifier();
    RPArmorIdentifier(const RPArmorIdentifier&)            = delete;
    RPArmorIdentifier& operator=(const RPArmorIdentifier&) = delete;

    std::vector<ArmorPlate>
        Identify(const cv::Mat& img, const rmcs_msgs::RobotColor& target_color, uint8_t blacklist);

    void draw_armors(const cv::Scalar& color);

private:
    class Impl;
    std::unique_ptr<Impl> pImpl_;
};

} // namespace rmcs_auto_aim
