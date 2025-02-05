
#include <memory>
#include <opencv2/highgui.hpp>

#include <cv_bridge/cv_bridge.h>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <std_msgs/msg/header.hpp>

#include <string>

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

    void show_image() final { cv::imshow(name_, image); };

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
        createOutputContext(name_.c_str(), width, height, 120);

        AVCodecContext* codecCtx = avcodec_alloc_context3(avcodec_find_encoder(AV_CODEC_ID_H264));
        avcodec_parameters_to_context(codecCtx, oc_->streams[0]->codecpar);

        AVFrame* frame = av_frame_alloc();
        frame->format  = codecCtx->pix_fmt;
        frame->width   = codecCtx->width;
        frame->height  = codecCtx->height;
        if (av_frame_get_buffer(frame, 32) < 0) {
            std::cerr << "Could not allocate frame buffer" << std::endl;
        }

        swsCtx_ = sws_getContext(
            width, height, AV_PIX_FMT_BGR24, width, height, codecCtx->pix_fmt, SWS_BILINEAR,
            nullptr, nullptr, nullptr);
    }

    void draw(const IAutoAimDrawable& drawable, const cv::Scalar& color) final {
        drawable.draw(image_, color);
    };

    void load_image(const cv::Mat& image) final { this->image_ = image.clone(); };

    void show_image() final {
        const uint8_t* srcSlice[1] = {image_.data};
        int srcStride[1]           = {static_cast<int>(image_.step[0])};
        sws_scale(swsCtx_, srcSlice, srcStride, 0, frame_->height, frame_->data, frame_->linesize);

        frame_->pts = frameCount_++;

        if (avcodec_send_frame(codecCtx_, frame_) < 0) {
            std::cerr << "Error: Could not send frame to encoder" << std::endl;
        }

        AVPacket pkt{};
        while (avcodec_receive_packet(codecCtx_, &pkt) == 0) {
            pkt.stream_index = oc_->streams[0]->index;
            av_packet_rescale_ts(&pkt, codecCtx_->time_base, oc_->streams[0]->time_base);
            av_interleaved_write_frame(oc_, &pkt);
            av_packet_unref(&pkt);
        }
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

        AVCodecContext* codecCtx = avcodec_alloc_context3(avcodec_find_encoder(AV_CODEC_ID_H264));
        if (!codecCtx) {
            RCLCPP_ERROR(rclcpp::get_logger("RTSP Viewer"), "Could not allocate codec context");
            return;
        }

        codecCtx->codec_id  = AV_CODEC_ID_H264;
        codecCtx->width     = width;
        codecCtx->height    = height;
        codecCtx->time_base = {1, fps};
        codecCtx->framerate = {fps, 1};
        codecCtx->pix_fmt   = AV_PIX_FMT_BGR24;
        codecCtx->bit_rate  = 400000;

        if (oc->oformat->flags & AVFMT_GLOBALHEADER) {
            codecCtx->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;
        }

        if (avcodec_open2(codecCtx, avcodec_find_encoder(codecCtx->codec_id), nullptr) < 0) {
            RCLCPP_ERROR(rclcpp::get_logger("RTSP Viewer"), "Could not open codec");
            return;
        }

        avcodec_parameters_from_context(stream->codecpar, codecCtx);

        if (!(oc->oformat->flags & AVFMT_NOFILE)) {
            if (avio_open(&oc->pb, url, AVIO_FLAG_WRITE) < 0) {
                RCLCPP_ERROR(rclcpp::get_logger("RTSP Viewer"), "Could not open output URL");
                return;
            }
        }

        auto _ = avformat_write_header(oc, nullptr);
        avcodec_free_context(&codecCtx);

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

std::unique_ptr<ImageViewer::ImageViewer_>
    ImageViewer::createProduct(int type, rclcpp::Node& node, const std::string& name) {
    switch (type) {
    case 1: return std::make_unique<ImShowViewer>(name);
    case 2: return std::make_unique<CVBridgeViewer>(node, name);
    case 3: return std::make_unique<RtspViewer>(name);
    default: return std::make_unique<NullViewer>();
    }
}
