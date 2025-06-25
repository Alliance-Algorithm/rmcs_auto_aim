#include <opencv2/imgcodecs.hpp>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include "armor.hpp"
#include <rmcs_msgs/robot_color.hpp>

#include "core/identifier/armor/armor.hpp"
#include "core/identifier/armor/armor_info.hpp"
#include "openvino/openvino.hpp"

#include "rp_armor_identifier.hpp"

using namespace rmcs_auto_aim;

class RPArmorIdentifier::Impl {
public:
    explicit Impl(const std::string& model_path, const std::string& device) {
        ov::Core core_;
        auto model_ = core_.read_model(model_path);

        std::unique_ptr<ov::preprocess::PrePostProcessor> pre_post_processor_ =
            std::make_unique<ov::preprocess::PrePostProcessor>(model_);
        ov::Shape input_shape_{1, image_height_, image_width_, 3};
        pre_post_processor_->input()
            .tensor()
            .set_element_type(ov::element::u8)
            .set_layout("NHWC")
            .set_color_format(ov::preprocess::ColorFormat::BGR);

        pre_post_processor_->input()
            .preprocess()
            .convert_element_type(ov::element::f32)
            .convert_color(ov::preprocess::ColorFormat::RGB)
            .scale({255., 255., 255.});

        pre_post_processor_->input().model().set_layout("NCHW");
        pre_post_processor_->output().tensor().set_element_type(ov::element::f32);
        model_ = pre_post_processor_->build();

        compiled_model_ = core_.compile_model(model_, device);
    }

    std::vector<ArmorPlate> Identify(
        const cv::Mat& img, const rmcs_msgs::RobotColor& target_color, const uint8_t& whitelist) {
        armor_plates_.clear();
        const auto invalid_armor_plates = model_infer(img, target_color, whitelist);
        matchPlate(img, invalid_armor_plates);
        return armor_plates_;
    }

    void draw_armors(const cv::Mat& img, const cv::Scalar& color) {
        for (const auto& armor_plate : armor_plates_) {
            cv::line(img, armor_plate.points[0], armor_plate.points[1], color);
            cv::line(img, armor_plate.points[1], armor_plate.points[2], color);
            cv::line(img, armor_plate.points[2], armor_plate.points[3], color);
            cv::line(img, armor_plate.points[3], armor_plate.points[0], color);
        }
    }

private:
    std::vector<ArmorInfo> model_infer(
        const cv::Mat& img, const rmcs_msgs::RobotColor& target_color, const uint8_t& whitelist) {
        cv::Mat resized_img;
        cv::resize(img, resized_img, cv::Size(image_width_, image_height_));
        const auto input_tensor = ov::Tensor{
            compiled_model_.input().get_element_type(), compiled_model_.input().get_shape(),
            resized_img.data};
        ov::InferRequest infer_request = compiled_model_.create_infer_request();
        infer_request.set_input_tensor(input_tensor);
        infer_request.infer();

        const auto output        = infer_request.get_output_tensor(0);
        const auto& output_shape = output.get_shape();

        cv::Mat output_buffer(
            static_cast<int>(output_shape[1]), static_cast<int>(output_shape[2]), CV_32F,
            output.data());

        std::vector<cv::Rect> boxes;
        std::vector<int> class_ids;
        std::vector<float> class_scores;
        std::vector<float> confidences;
        std::vector<ArmorInfo> tmp_objects_;
        for (int i = 0; i < output_buffer.rows; i++) {
            float confidence = output_buffer.at<float>(i, 8);
            confidence       = static_cast<float>(sigmoid(confidence));
            if (confidence < conf_threshold_)
                continue;

            const auto color_scores   = output_buffer.row(i).colRange(9, 13);  // color
            const auto classes_scores = output_buffer.row(i).colRange(13, 22); // num
            cv::Point class_id, color_id;
            double score_color, score_num;
            cv::minMaxLoc(classes_scores, nullptr, &score_num, nullptr, &class_id);
            cv::minMaxLoc(color_scores, nullptr, &score_color, nullptr, &color_id);

            if (color_id.x >= 2 || (color_id.x == 1 && target_color == rmcs_msgs::RobotColor::RED)
                || (color_id.x == 0 && target_color == rmcs_msgs::RobotColor::BLUE))
                continue;

            ArmorInfo obj;
            if (class_id.x == 3) {
                if (whitelist & rmcs_auto_aim::whitelist_code::InfantryIII)
                    obj.robot_id_ = rmcs_msgs::ArmorID::InfantryIII;
                else
                    continue;
            } else if (class_id.x == 4) {
                if (whitelist & rmcs_auto_aim::whitelist_code::InfantryIV)
                    obj.robot_id_ = rmcs_msgs::ArmorID::InfantryIV;
                else
                    continue;
            } else if (class_id.x == 6) {
                if (whitelist & rmcs_auto_aim::whitelist_code::Outpost)
                    obj.robot_id_ = rmcs_msgs::ArmorID::Outpost;
                else
                    continue;
            } else if (class_id.x == 1) {
                if (whitelist & rmcs_auto_aim::whitelist_code::Hero)
                    obj.robot_id_ = rmcs_msgs::ArmorID::Hero;
                else
                    continue;
            } else if (class_id.x == 2) {
                if (whitelist & rmcs_auto_aim::whitelist_code::Engineer)
                    obj.robot_id_ = rmcs_msgs::ArmorID::Engineer;
                else
                    continue;
            } else if (class_id.x == 0) {
                if (whitelist & rmcs_auto_aim::whitelist_code::Sentry)
                    obj.robot_id_ = rmcs_msgs::ArmorID::Sentry;
                else
                    continue;
            } else if (class_id.x == 7) {
                if (whitelist & rmcs_auto_aim::whitelist_code::Base)
                    obj.robot_id_ = rmcs_msgs::ArmorID::Base;
                else
                    continue;
            } else if (class_id.x == 5)
                obj.robot_id_ = rmcs_msgs::ArmorID::Aerial;
            else
                continue;

            obj.color_ = color_id.x == 1 ? rmcs_msgs::RobotColor::BLUE : rmcs_msgs::RobotColor::RED;
            tmp_objects_.emplace_back(obj);

            std::array<cv::Point2f, 4> points{
                cv::Point2f{output_buffer.at<float>(i, 0), output_buffer.at<float>(i, 1)},
                cv::Point2f{output_buffer.at<float>(i, 6), output_buffer.at<float>(i, 7)},
                cv::Point2f{output_buffer.at<float>(i, 4), output_buffer.at<float>(i, 5)},
                cv::Point2f{output_buffer.at<float>(i, 2), output_buffer.at<float>(i, 3)}
            };
            float min_x = points[0].x;
            float max_x = points[0].x;
            float min_y = points[0].y;
            float max_y = points[0].y;
            for (std::size_t i = 1; i < points.size(); i++) {
                if (points[i].x < min_x)
                    min_x = points[i].x;
                if (points[i].x > max_x)
                    max_x = points[i].x;
                if (points[i].y < min_y)
                    min_y = points[i].y;
                if (points[i].y > max_y)
                    max_y = points[i].y;
            }
            boxes.emplace_back(
                min_x * width_ratio_, min_y * height_ratio_, (max_x - min_x) * width_ratio_,
                (max_y - min_y) * height_ratio_);
            confidences.emplace_back(score_num);
        }

        std::vector<int> indices;
        cv::dnn::NMSBoxes(boxes, confidences, conf_threshold_, nms_threshold_, indices);
        std::vector<ArmorInfo> objects_;
        for (const std::size_t valid_index : indices)
            if (valid_index <= boxes.size()) {
                auto object  = tmp_objects_[valid_index];
                object.rect_ = boxes[valid_index];
                objects_.emplace_back(object);
            }

        return objects_;
    }

    static inline double sigmoid(double x) {
        if (x > 0)
            return 1.0 / (1.0 + std::exp(-x));
        else
            return std::exp(x) / (1.0 + std::exp(x));
    }

    void matchPlate(const cv::Mat& img, const std::vector<ArmorInfo>& armor_plates) {
        cv::Mat gray_img;
        cv::cvtColor(img, gray_img, cv::COLOR_BGR2GRAY);
        cv::threshold(gray_img, gray_img, 150, 255, cv::THRESH_BINARY);

        for (const auto& armor : armor_plates) {
            const auto roi = gray_img(
                cv::Rect{
                    cv::Point{
                              static_cast<int>(
                              armor.rect_.x
                              - armor.rect_.width / 2. * (match_magnification_ratio_ - 1.)),
                              static_cast<int>(
                              armor.rect_.y
                              - armor.rect_.height / 2. * (match_magnification_ratio_ - 1.))   },
                    cv::Size{
                              static_cast<int>(match_magnification_ratio_ * armor.rect_.width),
                              static_cast<int>(match_magnification_ratio_ * armor.rect_.height)}
            });
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(roi, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
            std::sort(
                contours.begin(), contours.end(),
                [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
                    return cv::contourArea(a, false) > cv::contourArea(b, false);
                });
            std::vector<LightBar> lightbars_;
            for (const auto& contour : contours) {
                auto r_rect = cv::minAreaRect(contour);
                auto b_rect = cv::boundingRect(contour);
                const auto roi =
                    img(cv::Rect{
                        cv::Point{armor.rect_.x + b_rect.x, armor.rect_.y + b_rect.y},
                        b_rect.size()
                });
                const auto channels       = cv::mean(roi);
                const auto b_r_difference = channels[0] - channels[2];
                if ((armor.color_ == rmcs_msgs::RobotColor::RED && b_r_difference > 0)
                    || (armor.color_ == rmcs_msgs::RobotColor::BLUE && b_r_difference < 0))
                    continue;

                // 可根据实际情况加一些形态学约束
                //  if(){
                //      continue; // 过滤掉不符合要求的矩形
                //  }

                cv::Point2f corners[4];
                r_rect.points(corners);

                std::vector<cv::Point2f> points(corners, corners + 4);

                std::sort(
                    points.begin(), points.end(),
                    [](const cv::Point2f& a, const cv::Point2f& b) { return a.y < b.y; });

                cv::Point2f tl, tr;
                if (points[0].x < points[1].x) {
                    tl = points[0];
                    tr = points[1];
                } else {
                    tl = points[1];
                    tr = points[0];
                }

                cv::Point2f bl, br;
                if (points[2].x < points[3].x) {
                    bl = points[2];
                    br = points[3];
                } else {
                    bl = points[3];
                    br = points[2];
                }

                lightbars_.emplace_back((tl + tr) / 2., (bl + br) / 2., r_rect.angle);
            }

            const auto lightbar_size_ = lightbars_.size();
            if (lightbar_size_ >= 2) {
                bool plate_matched{false};
                for (std::size_t i = 0; i < lightbar_size_ - 1 && !plate_matched; ++i) {
                    for (std::size_t j = i + 1; j < lightbar_size_ && !plate_matched; ++j) {
                        const auto& first  = lightbars_[i];
                        const auto& second = lightbars_[j];
                        if ((std::max(first.top.x, first.bottom.x)
                             < std::min(second.bottom.x, second.top.x))) {
                            armor_plates_.emplace_back(first, second, armor.robot_id_);
                            plate_matched = true;
                        } else if ((std::max(second.top.x, second.bottom.x)
                                    < std::min(first.bottom.x, first.top.x))) {
                            armor_plates_.emplace_back(second, first, armor.robot_id_);
                            plate_matched = true;
                        }
                    }
                }
            }
        }
    }

    static constexpr int image_height_      = 640;
    static constexpr int image_width_       = 640;
    static constexpr double width_ratio_    = 1440. / image_width_;
    static constexpr double height_ratio_   = 1080. / image_height_;
    static constexpr double conf_threshold_ = 0.65;
    static constexpr double nms_threshold_  = 0.45;

    static constexpr double match_magnification_ratio_ = 1.3;

    std::vector<ArmorPlate> armor_plates_;
    ov::CompiledModel compiled_model_;
};

RPArmorIdentifier::RPArmorIdentifier(const std::string& model_path, const std::string& device)
    : pImpl_(std::make_unique<Impl>(model_path, device)) {}

std::vector<ArmorPlate> RPArmorIdentifier::Identify(
    const cv::Mat& img, const rmcs_msgs::RobotColor& target_color, uint8_t whitelist) {
    return pImpl_->Identify(img, target_color, whitelist);
}

void RPArmorIdentifier::draw_armors(const cv::Mat& img, const cv::Scalar& color) {
    return pImpl_->draw_armors(img, color);
}

RPArmorIdentifier::~RPArmorIdentifier() = default;
