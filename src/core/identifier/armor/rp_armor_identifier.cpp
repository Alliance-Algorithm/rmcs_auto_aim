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
        const cv::Mat& img, const rmcs_msgs::RobotColor& target_color, const uint8_t& whitelist) {}

    void draw_armors(const cv::Scalar& color) {

        // for (const auto& box : valid_boxs_)
        //     cv::rectangle(img, box, color);
    }

private:
    void model_infer(const cv::Mat& img, const rmcs_msgs::RobotColor& target_color) {
        // valid_boxs_.clear();
        objects_.clear();

        cv::Mat resized_img;
        cv::resize(img, resized_img, cv::Size(image_width_, image_height_));
        const auto input_tensor = ov::Tensor{
            compiled_model_.input().get_element_type(), compiled_model_.input().get_shape(),
            resized_img.data};
        ov::InferRequest infer_request = compiled_model_.create_infer_request();
        infer_request.set_input_tensor(input_tensor);
        // const auto ta = cv::getTickCount();
        infer_request.infer();
        // const auto tb = cv::getTickCount();

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
            if (confidence < conf_threshold) {
                continue;
            }
            const auto color_scores   = output_buffer.row(i).colRange(9, 13);  // color
            const auto classes_scores = output_buffer.row(i).colRange(13, 22); // num
            cv::Point class_id, color_id;
            double score_color, score_num;
            cv::minMaxLoc(classes_scores, nullptr, &score_num, nullptr, &class_id);
            cv::minMaxLoc(color_scores, nullptr, &score_color, nullptr, &color_id);

            if (color_id.x >= 2)
                continue;
            else if (color_id.x == 1 && target_color == rmcs_msgs::RobotColor::RED)
                continue;
            else if (color_id.x == 0 && target_color == rmcs_msgs::RobotColor::BLUE)
                continue;

            ArmorInfo obj;
            obj.color_        = color_id.x;
            obj.label_        = class_id.x;
            obj.landmarks_[0] = output_buffer.at<float>(i, 0);
            obj.landmarks_[1] = output_buffer.at<float>(i, 1);
            obj.landmarks_[2] = output_buffer.at<float>(i, 2);
            obj.landmarks_[3] = output_buffer.at<float>(i, 3);
            obj.landmarks_[4] = output_buffer.at<float>(i, 4);
            obj.landmarks_[5] = output_buffer.at<float>(i, 5);
            obj.landmarks_[6] = output_buffer.at<float>(i, 6);
            obj.landmarks_[7] = output_buffer.at<float>(i, 7);

            std::array<cv::Point2f, 4> points{
                cv::Point2f{obj.landmarks_[0], obj.landmarks_[1]},
                cv::Point2f{obj.landmarks_[6], obj.landmarks_[7]},
                cv::Point2f{obj.landmarks_[4], obj.landmarks_[5]},
                cv::Point2f{obj.landmarks_[2], obj.landmarks_[3]}
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

            tmp_objects_.emplace_back(obj);
            boxes.emplace_back(
                min_x * width_ratio_, min_y * height_ratio_, (max_x - min_x) * width_ratio_,
                (max_y - min_y) * height_ratio_);
            confidences.emplace_back(score_num);
        }

        std::vector<int> indices;
        cv::dnn::NMSBoxes(boxes, confidences, conf_threshold, nms_threshold, indices);
        for (const std::size_t valid_index : indices) {
            if (valid_index <= boxes.size()) {
                rmcs_msgs::ArmorID robot_id{rmcs_msgs::ArmorID::Unknown};
                // 13到22是数字 G（哨兵） 1（一号） 2（二号） 3（三号） 4（四号） 5（五号）
                // O（前哨站） Bs（基地） Bb（基地大装甲）
                // const auto id = tmp_objects_[valid_index].label_;
                // switch (id) {
                // case 13: robot_id = rmcs_msgs::ArmorID::Sentry;
                // case 14: robot_id = rmcs_msgs::ArmorID::Hero;
                // case 15: robot_id = rmcs_msgs::ArmorID::Engineer;
                // case 16: robot_id = rmcs_msgs::ArmorID::InfantryIII;
                // case 17: robot_id = rmcs_msgs::ArmorID::InfantryIV;
                // case 18: robot_id = rmcs_msgs::ArmorID::Aerial;
                // case 19: robot_id = rmcs_msgs::ArmorID::Outpost;
                // case 20: robot_id = rmcs_msgs::ArmorID::Base;
                // case 21: robot_id = rmcs_msgs::ArmorID::LargeBaseArmor;
                // default: robot_id = rmcs_msgs::ArmorID::Unknown;
                // }
                // objects_.emplace_back(boxes[valid_index], robot_id);

                valid_armor_.emplace_back();
            }
        }
    }

    static inline double sigmoid(double x) {
        if (x > 0)
            return 1.0 / (1.0 + std::exp(-x));
        else
            return std::exp(x) / (1.0 + std::exp(x));
    }

    void Optimize(const cv::Mat& img) {
        cv::Mat gray;
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
        cv::threshold(gray, gray, 150, 255, cv::THRESH_BINARY);

        for (const auto& armor : valid_armor_) {
            const auto roi = gray(
                cv::Rect{
                    cv::Point{
                              armor.rect_.x - armor.rect_.width / 2,
                              armor.rect_.y - armor.rect_.height / 2                       },
                    cv::Size{                2 * armor.rect_.width, 2 * armor.rect_.height}
            });
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(roi, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
            std::vector<LightBar> lightbars_;
            for (const auto& contour : contours) {
                auto r_rect = cv::minAreaRect(contour);
                // r_rect.size()
                // 获取内四点
                // const auto angle_k=std::tan(r_rect.angle);
                // cv::Point2f top=r_rect.center+r_rect.size.
                // lightbars_.emplace_back(cv::Point2f{},cv::Point2f{},r_rect.angle);
            }
        }
    }

    static constexpr int image_height_     = 640;
    static constexpr int image_width_      = 640;
    static constexpr double width_ratio_   = 1440. / image_width_;
    static constexpr double height_ratio_  = 1080. / image_height_;
    static constexpr double conf_threshold = 0.65;
    static constexpr double nms_threshold  = 0.45;

    std::vector<ArmorInfo> valid_armor_;
    std::vector<ArmorPlate> objects_;
    ov::CompiledModel compiled_model_;
};

RPArmorIdentifier::RPArmorIdentifier(const std::string& model_path, const std::string& device)
    : pImpl_(std::make_unique<Impl>(model_path, device)) {}

std::vector<ArmorPlate> RPArmorIdentifier::Identify(
    const cv::Mat& img, const rmcs_msgs::RobotColor& target_color, uint8_t whitelist) {
    return pImpl_->Identify(img, target_color, whitelist);
}

void RPArmorIdentifier::draw_armors(const cv::Scalar& color) { return pImpl_->draw_armors(color); }

RPArmorIdentifier::~RPArmorIdentifier() = default;
