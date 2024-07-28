#include <iostream>
#include <opencv2/core/types.hpp>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include <rmcs_msgs/robot_color.hpp>

#include "core/identifier/armor/armor.hpp"
#include "core/identifier/armor/color_identifier.hpp"
#include "core/identifier/armor/number_identifier.hpp"

#include "armor_identifier.hpp"

using namespace rmcs_auto_aim;

class ArmorIdentifier::Impl {
public:
    template <class... Args>
    explicit Impl(Args&&... args)
        : _blueIdentifier(BlueLightBarHue)
        , _redIdentifier(RedLightBarHue)
        , _numberIdentifier(std::forward<Args>(args)...) {}

    std::vector<ArmorPlate>
        Identify(const cv::Mat& img, const rmcs_msgs::RobotColor& target_color, const int8_t& blacklist) {
        cv::Mat imgThre, imgGray;
        cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);
        cv::threshold(imgGray, imgThre, 150, 255, cv::THRESH_BINARY);

        std::vector<std::vector<cv::Point>> contours;
        std::vector<LightBar> lightBars;
        std::vector<ArmorPlate> result;
        cv::findContours(imgThre, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

        for (const auto& contour : contours) {
            if (auto&& lightBarOpt = _solveToLightbar(img, contour, target_color)) {
                lightBars.push_back(*lightBarOpt);

                std::sort(lightBars.begin(), lightBars.end(), [](LightBar& a, LightBar& b) {
                    return a.top.x < b.top.x;
                });
            }
        }

        size_t&& lightBarsSize = lightBars.size();
        for (size_t i = 0; i < lightBarsSize; ++i) {
            float Isize         = P2PDis(lightBars[i].top, lightBars[i].bottom);
            cv::Point2f Icenter = (lightBars[i].top + lightBars[i].bottom) / 2;
            for (size_t j = i + 1; j < lightBarsSize; ++j) { // 一些筛选条件

                float Jsize = P2PDis(lightBars[j].top, lightBars[j].bottom);
                if (fmax(Isize, Jsize) / fmin(Isize, Jsize) > maxArmorLightRatio)
                    continue;
                if (fabs(lightBars[i].angle - lightBars[j].angle) > maxdAngle)
                    continue;
                if (malposition(lightBars[i], lightBars[j]) > maxMalposition)
                    continue;
                cv::Point2f Jcenter = (lightBars[j].top + lightBars[j].bottom) / 2;
                if (fabs(Icenter.y - Jcenter.y) * 2 / (Isize + Jsize) > maxLightDy)
                    continue;
                float lightBarDis = P2PDis(Icenter, Jcenter) * 2 / (Isize + Jsize);

                if ((lightBarDis < minsmallArmorDis || lightBarDis > maxsmallArmorDis)
                    && (lightBarDis < minbigArmorDis || lightBarDis > maxbigArmorDis)) {

                    continue;
                }
                ArmorPlate armor(
                    lightBars[i], lightBars[j], rmcs_msgs::ArmorID::Unknown, lightBarDis > minbigArmorDis);

                if (_numberIdentifier.Identify(img, armor, blacklist)) {
                    result.push_back(armor);

                    cv::rectangle(img, cv::Rect{armor.points[0], armor.points[2]}, cv::Scalar(0, 255, 0), 2);
                    cv::putText(
                        img, std::to_string((int)armor.id), armor.center(), 2, 2, cv::Scalar(0, 255, 0), 2);
                }
            }
        }
        return result;
    }

private:
    ColorIdentifier _blueIdentifier, _redIdentifier;
    NumberIdentifier _numberIdentifier;

    inline static constexpr const double maxArmorLightRatio = 1.5;
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

    std::optional<LightBar> _solveToLightbar(
        const cv::Mat& img, const std::vector<cv::Point>& contour,
        const rmcs_msgs::RobotColor& target_color) {
        auto&& contourSize = contour.size();
        if (contourSize >= 5) {
            float scoreMap[256];
            float confidence                                            = 0.0f;
            scoreMap[static_cast<size_t>(ColorConfidence::NotCredible)] = 0.0f;
            scoreMap[static_cast<size_t>(ColorConfidence::CredibleOneChannelOverexposure)] =
                1.0f / (float)contourSize;
            scoreMap[static_cast<size_t>(ColorConfidence::CredibleTwoChannelOverexposure)] =
                0.5f / (float)contourSize;
            scoreMap[static_cast<size_t>(ColorConfidence::CredibleThreeChannelOverexposure)] =
                0.2f / (float)contourSize;

            auto& colorIdentifier =
                target_color == rmcs_msgs::RobotColor::BLUE ? _blueIdentifier : _redIdentifier;
            int maxPointY = 0;
            for (const auto& point : contour) {
                maxPointY = std::max(maxPointY, point.y);
                auto c    = reinterpret_cast<const uchar*>(&img.at<cv::Vec3b>(point));
                confidence += scoreMap[static_cast<size_t>(colorIdentifier.Identify(c))];
            }
            if (img.rows == maxPointY + 1)
                confidence = 0;

            if (confidence > 0.45f) {
                auto b_rect  = cv::boundingRect(contour);
                cv::Mat mask = cv::Mat::zeros(b_rect.size(), CV_8UC1);
                std::vector<cv::Point> mask_contour;
                mask_contour.reserve(contour.size());
                for (const auto& p : contour) {
                    mask_contour.emplace_back(p - cv::Point(b_rect.x, b_rect.y));
                }
                cv::fillPoly(mask, {mask_contour}, 255);
                std::vector<cv::Point> points;
                cv::findNonZero(mask, points);
                cv::Vec4f return_param;
                cv::fitLine(points, return_param, cv::DIST_L2, 0, 0.01, 0.01);
                cv::Point2f top, bottom;
                float angle_k;
                if (int(return_param[0] * 100) == 100 || int(return_param[1] * 100) == 0) {
                    top    = cv::Point2f((float)b_rect.x + (float)b_rect.width / 2, (float)b_rect.y);
                    bottom = cv::Point2f(
                        (float)b_rect.x + (float)b_rect.width / 2, (float)b_rect.y + (float)b_rect.height);
                    angle_k = 0;
                } else {
                    auto k = return_param[1] / return_param[0];
                    auto b = (return_param[3] + (float)b_rect.y) - k * (return_param[2] + (float)b_rect.x);
                    top    = cv::Point2f(((float)b_rect.y - b) / k, (float)b_rect.y);
                    bottom = cv::Point2f(
                        ((float)b_rect.y + (float)b_rect.height - b) / k,
                        (float)b_rect.y + (float)b_rect.height);
                    angle_k = (float)(std::atan(k) / CV_PI * 180 - 90);
                    if (angle_k > 90) {
                        angle_k = 180 - angle_k;
                    }
                }
                angle_k = (float)(angle_k / 180 * CV_PI);
                return LightBar{top, bottom, angle_k};
            }
        }
        return std::nullopt;
    }
};

ArmorIdentifier::ArmorIdentifier(const std::string& model_path)
    : pImpl_(new Impl{model_path}) {}

std::vector<ArmorPlate> ArmorIdentifier::Identify(
    const cv::Mat& img, const rmcs_msgs::RobotColor& target_color, int8_t blacklist) {
    return pImpl_->Identify(img, target_color, blacklist);
}

ArmorIdentifier::~ArmorIdentifier() = default;