#include <opencv2/imgcodecs.hpp>
#include <optional>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include <rmcs_msgs/robot_color.hpp>

#include "core/identifier/armor/armor.hpp"
#include "core/identifier/armor/number_identifier.hpp"

#include "armor_identifier_for_outpost.hpp"

using namespace rmcs_auto_aim;

class ArmorIdentifierForOutpost::Impl {
public:
    template <class... Args>
    explicit Impl(Args&&... args)
        : _numberIdentifier(std::forward<Args>(args)...) {}

    std::vector<ArmorPlate> Identify(
        const cv::Mat& img, const rmcs_msgs::RobotColor& target_color, const uint8_t& whitelist) {
        cv::Mat imgThre, imgGray;
        cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);
        cv::threshold(imgGray, imgThre, 150, 255, cv::THRESH_BINARY);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(imgThre, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

        contour_rect_.clear();
        for (const auto& contour : contours)
            contour_rect_.emplace_back(cv::boundingRect(contour));

        const auto guide_lamp_point = solveToGuideLamp(img, contours);

        if (guide_lamp_point == cv::Point{-1, -1}) {
            return {};
        }

        const auto lightBars = solveToLightbar(img, contours, guide_lamp_point, target_color);

        std::vector<ArmorPlate> result;
        size_t&& lightBarsSize = lightBars.size();
        for (size_t i = 0; i < lightBarsSize; ++i) {
            float Isize         = P2PDis(lightBars[i].top, lightBars[i].bottom);
            cv::Point2f Icenter = (lightBars[i].top + lightBars[i].bottom) / 2;
            for (size_t j = i + 1; j < lightBarsSize; ++j) {
                float Jsize = P2PDis(lightBars[j].top, lightBars[j].bottom);
                if (fmax(Isize, Jsize) / fmin(Isize, Jsize) > maxArmorLightRatio) {
                    //    std::cout << "one" << std::endl;
                    continue;
                }
                if (fabs(lightBars[i].angle - lightBars[j].angle) > maxdAngle) {
                    //    std::cout << "two" << std::endl;
                    continue;
                }
                if (malposition(lightBars[i], lightBars[j]) > maxMalposition) {
                    //    std::cout << "three" << std::endl;
                    continue;
                }
                cv::Point2f Jcenter = (lightBars[j].top + lightBars[j].bottom) / 2;
                if (fabs(Icenter.y - Jcenter.y) * 2 / (Isize + Jsize) > maxLightDy) {
                    // std::cout << "for" << std::endl;
                    continue;
                }
                float lightBarDis = P2PDis(Icenter, Jcenter) * 2 / (Isize + Jsize);

                if ((lightBarDis < minsmallArmorDis || lightBarDis > maxsmallArmorDis)
                    && (lightBarDis < minbigArmorDis || lightBarDis > maxbigArmorDis)) {
                    // std::cout << "five" << std::endl;
                    continue;
                }

                ArmorPlate armor(lightBars[i], lightBars[j], rmcs_msgs::ArmorID::Outpost);

                if (_numberIdentifier.Identify(img, armor, whitelist)) {
                    result.push_back(armor);
                }

                cv::rectangle(
                    img, cv::Rect{armor.points[0], armor.points[2]}, cv::Scalar(0, 255, 0), 2);
                cv::putText(
                    img, std::to_string((int)armor.id), armor.center(), 2, 2, cv::Scalar(0, 255, 0),
                    2);
                //}
            }
        }

        return result;
    }

private:
    NumberIdentifier _numberIdentifier;

    inline static constexpr const double maxArmorLightRatio = 2;
    inline static constexpr const double maxdAngle          = 9.5;
    inline static constexpr const double maxMalposition     = 0.7;
    inline static constexpr const double maxLightDy         = 0.9;
    inline static constexpr const double maxbigArmorDis     = 5.5;
    inline static constexpr const double minbigArmorDis     = 3.2;
    inline static constexpr const double maxsmallArmorDis   = 3.2;
    inline static constexpr const double minsmallArmorDis   = 0.8;
    inline static constexpr const double BlueLightBarHue    = 228.0f;
    inline static constexpr const double RedLightBarHue     = 11.0f;

    static inline double malposition(const LightBar& LBl, const LightBar& LBr) {
        cv::Point2f axis = (LBl.top - LBl.bottom + LBr.top - LBr.bottom) / 2;
        cv::Point2f dis  = (LBl.top + LBl.bottom - LBr.top - LBr.bottom) / 2;
        return fabs(axis.dot(dis) / axis.cross(dis));
    }

    static inline float P2PDis(const cv::Point2f& a, const cv::Point2f& b) {
        cv::Point2f tmp = b - a;
        return sqrt(tmp.x * tmp.x + tmp.y * tmp.y);
    }

    static inline float P2PDis(const cv::Point3f& a, const cv::Point3f& b) {
        cv::Point3f tmp = b - a;
        return sqrt(tmp.x * tmp.x + tmp.y * tmp.y + tmp.z * tmp.z);
    }

    template <typename T>
    inline void LimitRange(T& value, const T min, const T max) {
        value = value < min ? min : value;
        value = value > max ? max : value;
    }

    inline cv::Point
        solveToGuideLamp(const cv::Mat& img, const std::vector<std::vector<cv::Point>>& contours) {
        for (const auto& b_rect : contour_rect_) {
            contour_rect_.emplace_back(b_rect);

            const auto filled_ratio = cv::contourArea(contours) / (b_rect.width * b_rect.height);
            const auto filled       = filled_ratio > 0.6 && filled_ratio < 0.9 ? true : false;
            const auto ratio =
                std::abs(static_cast<double>(b_rect.width) / b_rect.height - 1.) < 0.3 ? true
                                                                                       : false;

            if (filled && ratio) {
                const auto channels = cv::mean(img(b_rect));
                if (channels[1] >= channels[0] + channels[2] && channels[1] >= 150) {
                    std::cout << "green" << std::endl;
                    return {b_rect.x + b_rect.width / 2, b_rect.y + b_rect.height};
                }
            }

            return {-1, -1};
        }
    }

    inline std::vector<LightBar> solveToLightbar(
        const cv::Mat& img, const std::vector<std::vector<cv::Point>>& contours,
        const cv::Point guide_lamp_pos_, const rmcs_msgs::RobotColor& target_color) {

        std::vector<LightBar> light_bars;
        const auto contour_size = contours.size();
        for (size_t i = 0; i < contour_size; i++) {
            if (contour_rect_[i].y > guide_lamp_pos_.y
                || std::abs(contour_rect_[i].x - guide_lamp_pos_.x) >= 200) {
                continue;
            }
            const auto& contour = contours[i];
            auto&& contourSize  = contour.size();
            if (contourSize >= 5) {
                auto b_rect  = cv::boundingRect(contour);
                auto r_rect  = cv::minAreaRect(contour);
                cv::Mat mask = cv::Mat::zeros(b_rect.size(), CV_8UC1);
                std::vector<cv::Point> mask_contour;
                mask_contour.reserve(contour.size());
                for (const auto& p : contour) {
                    mask_contour.emplace_back(p - cv::Point(b_rect.x, b_rect.y));
                }
                cv::fillPoly(mask, {mask_contour}, 255);
                std::vector<cv::Point> points;
                cv::findNonZero(mask, points);
                bool filled = (float)points.size() / (r_rect.size.width * r_rect.size.height) > 0.8;
                cv::Vec4f return_param;
                cv::fitLine(points, return_param, cv::DIST_L2, 0, 0.01, 0.01);
                cv::Point2f top, bottom;
                float angle_k;
                if (int(return_param[0] * 100) == 100 || int(return_param[1] * 100) == 0) {
                    top = cv::Point2f((float)b_rect.x + (float)b_rect.width / 2, (float)b_rect.y);
                    bottom = cv::Point2f(
                        (float)b_rect.x + (float)b_rect.width / 2,
                        (float)b_rect.y + (float)b_rect.height);
                    angle_k = 0;
                } else {
                    auto k = return_param[1] / return_param[0];
                    auto b = (return_param[3] + (float)b_rect.y)
                           - k * (return_param[2] + (float)b_rect.x);
                    top    = cv::Point2f(((float)b_rect.y - b) / k, (float)b_rect.y);
                    bottom = cv::Point2f(
                        ((float)b_rect.y + (float)b_rect.height - b) / k,
                        (float)b_rect.y + (float)b_rect.height);
                    angle_k = (float)(std::atan(k) / CV_PI * 180 - 90);
                    if (angle_k > 90) {
                        angle_k = 180 - angle_k;
                    }
                }
                if (angle_k > 70.0)
                    continue;

                auto length = cv::norm(bottom - top);
                auto width  = (double)points.size() / length;

                auto ratio = width / length;
                if (!(ratio > 0.1 && ratio < 0.4 && filled))
                    continue;
                angle_k  = (float)(angle_k / 180 * CV_PI);
                auto tmp = LightBar{top, bottom, angle_k};
                if (0 <= b_rect.x && 0 <= b_rect.width && b_rect.x + b_rect.width <= img.cols
                    && 0 <= b_rect.y && 0 <= b_rect.height
                    && b_rect.y + b_rect.height <= img.rows) {
                    cv::Mat output;
                    const auto channels = cv::mean(img(b_rect));
                    if (channels[1] > 200)
                        break;
                    const auto sum = channels[0] - channels[2];
                    if ((sum > 0 && target_color == rmcs_msgs::RobotColor::BLUE)
                        || (sum < 0 && target_color == rmcs_msgs::RobotColor::RED))
                        light_bars.emplace_back(tmp);
                }
            }
        }

        if (light_bars.size() > 2) {
            std::sort(
                light_bars.begin(), light_bars.end(),
                [&guide_lamp_pos_](const LightBar& a, const LightBar& b) {
                    return cv::norm(
                               static_cast<cv::Point2f>(guide_lamp_pos_) - (a.top + a.bottom) / 2)
                         > cv::norm(
                               static_cast<cv::Point2f>(guide_lamp_pos_) - (a.top + a.bottom) / 2);
                });
            light_bars.erase(light_bars.begin() + 2, light_bars.end());
        }

        return {};
    }

    std::vector<cv::Rect> contour_rect_;
};

ArmorIdentifierForOutpost::ArmorIdentifierForOutpost(const std::string& model_path)
    : pImpl_(new Impl{model_path}) {}

std::vector<ArmorPlate> ArmorIdentifierForOutpost::Identify(
    const cv::Mat& img, const rmcs_msgs::RobotColor& target_color, uint8_t whitelist) {
    return pImpl_->Identify(img, target_color, whitelist);
}

ArmorIdentifierForOutpost::~ArmorIdentifierForOutpost() = default;
