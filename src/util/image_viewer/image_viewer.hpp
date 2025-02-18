#pragma once

#include <memory>
#include <string>

#include <opencv2/core/mat.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
namespace rmcs_auto_aim::util {

class IAutoAimDrawable {
public:
    virtual void draw(cv::InputOutputArray image, const cv::Scalar& color) const = 0;

    virtual ~IAutoAimDrawable() = default;
};

class ImageViewer {

public:
    class ImageViewer_ {
    public:
        virtual void draw(const IAutoAimDrawable&, const cv::Scalar&) = 0;

        virtual void load_image(const cv::Mat& image) = 0;

        virtual void show_image() = 0;

        virtual ~ImageViewer_() = default;
    };
    static inline void load_image(const cv::Mat& image) {
        if (viewer_ == nullptr) [[unlikely]]
            return;

        viewer_->load_image(image);
    }

    static inline void draw(const IAutoAimDrawable& drawable, const cv::Scalar& color) {
        if (viewer_ == nullptr) [[unlikely]]
            return;

        viewer_->draw(drawable, color);
    }
    static inline void show_image() {
        if (viewer_ == nullptr) [[unlikely]]
            return;

        viewer_->show_image();
    }

    static void createProduct(int type, rclcpp::Node& node, const std::string& name);

private:
    static std::unique_ptr<ImageViewer_> viewer_;
};
} // namespace rmcs_auto_aim::util