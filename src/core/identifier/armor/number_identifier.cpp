#include <cstdint>
#include <std_msgs/msg/detail/int8__struct.hpp>
#include <stdexcept>
#include <string>

#include <opencv2/opencv.hpp>

#include "number_identifier.hpp"

using namespace rmcs_auto_aim;

NumberIdentifier::NumberIdentifier(const std::string& model_path) {
    _net = cv::dnn::readNetFromONNX(model_path);
    if (_net.empty()) {
        throw std::runtime_error("Could not read the model file.");
    }
}

bool NumberIdentifier::Identify(
    const cv::Mat& imgGray, ArmorPlate& armor, const int8_t& blacklist) {
    static const std::vector<cv::Point2f> dst = {
        {-4.5, 10.125},
        {-4.5, 25.875},
        {40.5, 25.875},
        {40.5, 10.125}
    };
    cv::Mat imgWarped, imgNumber, imgInput, M = getPerspectiveTransform(armor.points, dst);

    warpPerspective(imgGray, imgWarped, M, cv::Size(36, 36));

    double imgWrapedMaxVal;
    minMaxLoc(imgWarped, nullptr, &imgWrapedMaxVal, nullptr, nullptr);
    if (imgWrapedMaxVal > 120)
        return false;

    threshold(imgWarped, imgNumber, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

    cv::Mat blobImage = cv::dnn::blobFromImage(imgNumber, 1.0, cv::Size(36, 36), false, false);

    _net.setInput(blobImage);
    cv::Mat pred = _net.forward();

    double maxVal;
    cv::Point maxLoc;
    minMaxLoc(pred, nullptr, &maxVal, nullptr, &maxLoc);

    switch (maxLoc.x) {
    case 0: return false;
    case 1:
        if (!(blacklist & 0x1)) {
            return false;
        }
        armor.id             = rmcs_msgs::ArmorID::Hero;
        armor.is_large_armor = true;
        break;
    case 2:
        if (!(blacklist & 0x2)) {
            return false;
        }
        armor.id             = rmcs_msgs::ArmorID::Engineer;
        armor.is_large_armor = false;
        break;
    case 3:
        if (!(blacklist & 0x4)) {
            return false;
        }
        armor.id             = rmcs_msgs::ArmorID::InfantryIII;
        armor.is_large_armor = false;
        break;
    case 4:
        if (!(blacklist & 0x8)) {
            return false;
        }
        armor.id             = rmcs_msgs::ArmorID::InfantryIV;
        armor.is_large_armor = false;
        break;
    case 5:
        if (!(blacklist & 0x10)) {
            return false;
        }
        armor.id             = rmcs_msgs::ArmorID::InfantryV;
        armor.is_large_armor = false;
        break;
    case 6:
        if (!(blacklist & 0x20)) {
            return false;
        }
        armor.id             = rmcs_msgs::ArmorID::Sentry;
        armor.is_large_armor = false;
        break;
    case 7:
        if (!(blacklist & 0x4)) {
            return false;
        }
        armor.id             = rmcs_msgs::ArmorID::InfantryIII;
        armor.is_large_armor = true;
        break;
    case 8:
        if (!(blacklist & 0x8)) {
            return false;
        }
        armor.id             = rmcs_msgs::ArmorID::InfantryIV;
        armor.is_large_armor = true;
        break;
    case 9:
        if (!(blacklist & 0x10)) {
            return false;
        }
        armor.id             = rmcs_msgs::ArmorID::InfantryV;
        armor.is_large_armor = true;
        break;
    case 10:
        armor.id             = rmcs_msgs::ArmorID::Outpost;
        armor.is_large_armor = false;
        break;
    case 11:
        armor.id             = rmcs_msgs::ArmorID::Base;
        armor.is_large_armor = false;
        break;
    default: return false;
    }

    return true;
}
