#include <cstdint>
#include <opencv2/highgui.hpp>
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

bool NumberIdentifier::Identify(const cv::Mat& imgGray, ArmorPlate& armor, const int8_t& blacklist) {

    static const int light_length      = 12;
    static const int warp_height       = 28;
    static const int small_armor_width = 32;
    static const int large_armor_width = 54;
    static const int top_light_y       = (warp_height - light_length) / 2 - 1;
    static const int bottom_light_y    = top_light_y + light_length;
    static const int warp_width        = armor.is_large_armor ? large_armor_width : small_armor_width;
    static const cv::Size roi_size(20, 28);

    static const std::vector<cv::Point2f> dst = {
        cv::Point(0, top_light_y),
        cv::Point(0, bottom_light_y),
        cv::Point(warp_width - 1, bottom_light_y),
        cv::Point(warp_width - 1, top_light_y),
    };
    cv::Mat imgWarped, imgNumber, imgInput, M = getPerspectiveTransform(armor.points, dst);

    warpPerspective(imgGray, imgWarped, M, cv::Size(warp_width, warp_height));

    // Get ROI
    imgWarped = imgWarped(cv::Rect(cv::Point((warp_width - roi_size.width) / 2, 0), roi_size));

    // double imgWrapedMaxVal;
    // minMaxLoc(imgWarped, nullptr, &imgWrapedMaxVal, nullptr, nullptr);
    // if (imgWrapedMaxVal > 120)
    //     return false;

    cv::cvtColor(imgWarped, imgWarped, cv::COLOR_BGR2GRAY);
    threshold(imgWarped, imgNumber, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

    imgNumber = imgNumber / 255.0;

    // cv::imshow("imgNumber", imgNumber);
    // cv::waitKey(1);

    cv::Mat blobImage;
    cv::dnn::blobFromImage(imgNumber, blobImage);

    _net.setInput(blobImage);
    cv::Mat pred = _net.forward();

    float max_prob = *std::max_element(pred.begin<float>(), pred.end<float>());
    cv::Mat softmax_prob;
    cv::exp(pred - max_prob, softmax_prob);
    float sum = static_cast<float>(cv::sum(softmax_prob)[0]);
    softmax_prob /= sum;

    double confidence;
    cv::Point class_id_point;
    minMaxLoc(softmax_prob.reshape(1, 1), nullptr, &confidence, nullptr, &class_id_point);
    int label_id = class_id_point.x;

    switch (label_id) {
    case 0:
        if (!(blacklist & 0x1)) {
            return false;
        }
        armor.id = rmcs_msgs::ArmorID::Hero;
        break;
    case 1:
        if (!(blacklist & 0x2)) {
            return false;
        }
        armor.id = rmcs_msgs::ArmorID::Engineer;
        break;
    case 2:
        if (!(blacklist & 0x4)) {
            return false;
        }
        armor.id = rmcs_msgs::ArmorID::InfantryIII;
        break;
    case 3:
        if (!(blacklist & 0x8)) {
            return false;
        }
        armor.id = rmcs_msgs::ArmorID::InfantryIV;
        break;
    case 4:
        if (!(blacklist & 0x10)) {
            return false;
        }
        armor.id = rmcs_msgs::ArmorID::InfantryV;
        break;
    case 5: armor.id = rmcs_msgs::ArmorID::Outpost; break;
    case 6:
        if (!(blacklist & 0x20)) {
            return false;
        }
        armor.id = rmcs_msgs::ArmorID::Sentry;
        break;
    case 7: armor.id = rmcs_msgs::ArmorID::Base; break;
    default: return false;
    }

    return true;
}
