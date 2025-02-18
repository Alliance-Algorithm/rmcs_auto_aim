
#include <iostream>
#include <memory>
#include <opencv2/highgui.hpp>
#include <ostream>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <libavutil/pixfmt.h>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <std_msgs/msg/header.hpp>

#include "util/image_viewer/image_viewer.hpp"
#include "util/profile/profile.hpp"
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
    ~CVBridgeViewer() { image.release(); }

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

    void show_image() final {
        cv::imshow(name_, image);
        cv::waitKey(1);
    };

    ~ImShowViewer() { image.release(); }

private:
    const std::string& name_;
    cv::Mat image;
};

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libswscale/swscale.h>
}
class RtspViewer final : public ImageViewer::ImageViewer_ {
public:
    explicit RtspViewer(const std::string& name)
        : name_("rtsp://localhost:8554/live/" + name) {
        frameCount_ = 0;

        avformat_network_init();
        auto& [width, height] = Profile::get_width_height();
        createOutputContext(name_.c_str(), width, height, 60);

        frame_         = av_frame_alloc();
        frame_->format = codecCtx_->pix_fmt;
        frame_->width  = codecCtx_->width;
        frame_->height = codecCtx_->height;
        if (av_frame_get_buffer(frame_, 32) < 0) {
            std::cerr << "Could not allocate frame buffer" << std::endl;
        }

        swsCtx_ = sws_getContext(
            width, height, AV_PIX_FMT_BGR24, width, height, codecCtx_->pix_fmt, SWS_BILINEAR,
            nullptr, nullptr, nullptr);
    }

    void draw(const IAutoAimDrawable& drawable, const cv::Scalar& color) final {
        drawable.draw(image_, color);
    };

    void load_image(const cv::Mat& image) final { this->image_ = image.clone(); };

    void show_image() final {
        const uint8_t* srcSlice[1] = {image_.data};
        int srcStride[1]           = {static_cast<int>(image_.step[0])};
        int ret                    = 0;
        ret                        = sws_scale(
            swsCtx_, srcSlice, srcStride, 0, frame_->height, frame_->data, frame_->linesize);

        if (ret < 0) {
            std::cerr << "Error: Could not scale image" << ret << std::endl;
            return;
        }

        frame_->pts = frameCount_++;

        if ((ret = avcodec_send_frame(codecCtx_, frame_)) < 0) {
            std::cerr << "Error: Could not send frame" << std::endl;
            return;
        }

        AVPacket pkt{};
        if (avcodec_receive_packet(codecCtx_, &pkt) == 0) {
            pkt.stream_index = oc_->streams[0]->index;
            av_packet_rescale_ts(&pkt, codecCtx_->time_base, oc_->streams[0]->time_base);
            ret = av_interleaved_write_frame(oc_, &pkt);
            if (ret < 0) {
                std::cerr << "Error: Could not write frame" << std::endl;
            }
            av_packet_unref(&pkt);
        }

        while (avcodec_receive_packet(codecCtx_, &pkt) == 0)
            av_packet_unref(&pkt);
    };

    RtspViewer() = delete;
    ~RtspViewer() {
        image_.release();

        av_write_trailer(oc_);
        avcodec_close(codecCtx_);
        avio_close(oc_->pb);
        avformat_free_context(oc_);
        av_frame_free(&frame_);
        sws_freeContext(swsCtx_);
    }

private:
    void createOutputContext(const char* url, int width, int height, int fps) {
        AVFormatContext* oc = nullptr;
        avformat_alloc_output_context2(&oc, nullptr, "rtsp", url);
        if (!oc) {
            RCLCPP_ERROR(rclcpp::get_logger("RTSP Viewer"), "Could not create output context");
            return;
        }

        AVStream* stream = avformat_new_stream(oc, nullptr);
        if (!stream) {
            RCLCPP_ERROR(rclcpp::get_logger("RTSP Viewer"), "Could not create stream");
            return;
        }

        codecCtx_ = avcodec_alloc_context3(avcodec_find_encoder(AV_CODEC_ID_H264));
        if (!codecCtx_) {
            RCLCPP_ERROR(rclcpp::get_logger("RTSP Viewer"), "Could not allocate codec context");
            return;
        }

        codecCtx_->codec_id  = AV_CODEC_ID_H264;
        codecCtx_->width     = width;
        codecCtx_->height    = height;
        codecCtx_->time_base = {1, fps};
        codecCtx_->framerate = {fps, 1};
        codecCtx_->pix_fmt   = AV_PIX_FMT_YUV420P;
        codecCtx_->bit_rate  = 400000;

        if (oc->oformat->flags & AVFMT_GLOBALHEADER) {
            codecCtx_->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;
        }

        if (avcodec_open2(codecCtx_, avcodec_find_encoder(codecCtx_->codec_id), nullptr) < 0) {
            RCLCPP_ERROR(rclcpp::get_logger("RTSP Viewer"), "Could not open codec");
            return;
        }

        auto ret = avcodec_parameters_from_context(stream->codecpar, codecCtx_);

        if (ret < 0) {
            RCLCPP_ERROR(rclcpp::get_logger("RTSP Viewer"), "Could not copy codec parameters");
            return;
        }

        if (!(oc->oformat->flags & AVFMT_NOFILE)) {
            if (avio_open(&oc->pb, url, AVIO_FLAG_WRITE) < 0) {
                RCLCPP_ERROR(rclcpp::get_logger("RTSP Viewer"), "Could not open output URL");
                return;
            }
        }

        ret = avformat_write_header(oc, nullptr);

        if (ret < 0) {
            RCLCPP_INFO(
                rclcpp::get_logger("RTSP Viewer"),
                "Error return code from avformat_write_header: %x", ret);
            return;
        }

        if (!oc) {
            RCLCPP_INFO(
                rclcpp::get_logger("RTSP Viewer"), "Error: Could not create output context");
        } else
            oc_ = oc;
    }
    const std::string name_;
    cv::Mat image_;

    struct SwsContext* swsCtx_;
    AVFormatContext* oc_;
    AVCodecContext* codecCtx_;
    AVFrame* frame_;

    int frameCount_;
};

class NullViewer final : public ImageViewer::ImageViewer_ {
public:
    void draw(const IAutoAimDrawable&, const cv::Scalar&) final {};

    void load_image(const cv::Mat&) final {};

    void show_image() final {};
};

void ImageViewer::createProduct(int type, rclcpp::Node& node, const std::string& name) {
    switch (type) {
    case 1: viewer_ = std::make_unique<ImShowViewer>(name); break;
    case 2: viewer_ = std::make_unique<CVBridgeViewer>(node, name); break;
    case 3: viewer_ = std::make_unique<RtspViewer>(name); break;
    default: viewer_ = std::make_unique<NullViewer>(); break;
    }
}

std::unique_ptr<rmcs_auto_aim::util::ImageViewer::ImageViewer_>
    rmcs_auto_aim::util::ImageViewer::viewer_;