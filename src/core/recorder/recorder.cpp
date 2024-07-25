#include <string>

#include "recorder.hpp"

using namespace rmcs_auto_aim;

class Recorder::Impl {
public:
    explicit Impl() {

        auto now       = std::chrono::system_clock::now();
        auto in_time_t = std::chrono::system_clock::to_time_t(now);

        std::stringstream ss;
        ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d_%H-%M-%S");
        filename_ = "output_" + ss.str() + ".avi";
    }

    void setParam(const double& fps, const cv::Size& size) {
        fps_ = fps;
        video_ =
            cv::VideoWriter(filename_, cv::VideoWriter::fourcc('M', 'P', '4', '2'), fps_, size);
    }

    ~Impl() { video_.release(); }

    bool is_opened() const { return video_.isOpened(); }

    bool record_frame(const cv::Mat& frame) {
        if (frame.empty()) {
            return false;
        }
        video_.write(frame);
        return true;
    }

private:
    std::string filename_;
    cv::VideoWriter video_;
    double fps_;
};

Recorder::Recorder()
    : pImpl_(new Impl{}) {}

Recorder::~Recorder() {}

void Recorder::setParam(const double& fps, const cv::Size& size) {
    return pImpl_->setParam(fps, size);
}

bool Recorder::record_frame(const cv::Mat& frame) { return pImpl_->record_frame(frame); }

bool Recorder::is_opened() const { return pImpl_->is_opened(); }