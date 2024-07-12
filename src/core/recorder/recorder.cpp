#include <string>

#include "recorder.hpp"

using namespace rmcs_auto_aim;

class Recorder::Impl {
public:
    explicit Impl(const double& fps, const cv::Size& size) {

        auto now       = std::chrono::system_clock::now();
        auto in_time_t = std::chrono::system_clock::to_time_t(now);

        std::stringstream ss;
        ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d_%H-%M-%S");
        filename_ = "output_" + ss.str() + ".avi";

        video_ = cv::VideoWriter(filename_, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), fps, size);
    }

    ~Impl() { video_.release(); }

    constexpr bool is_opened() const { return video_.isOpened(); }

    constexpr bool record_frame(const cv::Mat& frame) {
        if (frame.empty()) {
            return false;
        }
        video_.write(frame);
        return true;
    }

private:
    std::string filename_;
    cv::VideoWriter video_;
};

Recorder::Recorder(const double& fps, const cv::Size& size)
    : pImpl_(new Impl{fps, size}) {}

bool Recorder::record_frame(const cv::Mat& frame) { return pImpl_->record_frame(frame); }

bool Recorder::is_opened() const { return pImpl_->is_opened(); }