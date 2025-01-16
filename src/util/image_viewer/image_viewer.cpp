
#include <memory>
#include <opencv2/highgui.hpp>

#include <cv_bridge/cv_bridge.h>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <std_msgs/msg/header.hpp>

#include <string>

#include "util/image_viewer/image_viewer.hpp"
#include <sensor_msgs/msg/detail/image__struct.hpp>

using namespace rmcs_auto_aim::util;

class CVBridgeViewer final : public ImageViewer::ImageViewer_ {
public:
    explicit CVBridgeViewer(rclcpp::Node& node, const std::string& name)
        : name_(name) {

        publisher_ = node.create_publisher<sensor_msgs::msg::Image>(name_, 10);
    }

    void draw(const IAutoAimDrawable& drawable, const cv::Scalar& color) final {
        drawable.draw(image, color);
    };

    void load_image(const cv::Mat&) final {};

    void show_image() final {
        auto output_msg =
            cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::BGR8, image)
                .toImageMsg();
    };

private:
    const std::string& name_;
    cv::Mat image;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

class ImShowViewer final : public ImageViewer::ImageViewer_ {
public:
    explicit ImShowViewer(const std::string& name)
        : name_(name) {}

    void draw(const IAutoAimDrawable& drawable, const cv::Scalar& color) final {
        drawable.draw(image, color);
    };

    void load_image(const cv::Mat& image) final { this->image = image.clone(); };

    void show_image() final { cv::imshow(name_, image); };

private:
    const std::string& name_;
    cv::Mat image;
};

class NullViewer final : public ImageViewer::ImageViewer_ {
public:
    void draw(const IAutoAimDrawable&, const cv::Scalar&) final {};

    void load_image(const cv::Mat&) final {};

    void show_image() final {};
};

std::unique_ptr<ImageViewer::ImageViewer_>
    ImageViewer::createProduct(int type, rclcpp::Node& node, const std::string& name) {
    switch (type) {
    case 1: return std::make_unique<ImShowViewer>(name);
    case 2: return std::make_unique<CVBridgeViewer>(node, name);
    default: return std::make_unique<NullViewer>();
    }
}
