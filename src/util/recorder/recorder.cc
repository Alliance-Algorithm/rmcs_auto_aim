#include "recorder.hh"

#include <filesystem>
#include <opencv2/imgcodecs.hpp>

using namespace rmcs_auto_aim;

struct Recorder::Impl {
    std::atomic<bool> start_record = false;
    bool enable_record             = false;
    bool enable_forced             = false;
    std::string record_folder_path;
    std::string record_item_path;
    std::chrono::milliseconds record_interval;

    TimePoint last_time_point;

    rclcpp::Logger logger;

    explicit Impl(rclcpp::Node& node)
        : logger(node.get_logger()) {

        last_time_point = std::chrono::steady_clock::now();

        const auto p = [&node](const std::string& name, const auto& type) {
            using T    = std::remove_cvref_t<decltype(type)>;
            auto param = T{};
            node.get_parameter(name, param);
            return param;
        };

        enable_record      = p("enable_record", bool{});
        enable_forced      = p("enable_forced", bool{});
        record_folder_path = p("record_path", std::string{});
        record_interval    = std::chrono::milliseconds{p("record_interval", int{})};

        RCLCPP_INFO(logger, "AutoAim Enable Record: %d", enable_record);
        RCLCPP_INFO(logger, "AutoAim Enable Forced: %d", enable_forced);
        RCLCPP_INFO(logger, "AutoAim Record Interval: %ldms", record_interval.count());
        RCLCPP_INFO(logger, "AutoAim Path: %s", record_folder_path.c_str());

        if (enable_forced)
            set_record_status(true);
    }

    void set_record_status(bool is_start) {
        if (is_start == true) {
            record_item_path = record_folder_path + format_steady_time() + "/";
            start_record.store(true, std::memory_order::relaxed);
            if (!std::filesystem::exists(record_item_path))
                std::filesystem::create_directories(record_item_path);
            RCLCPP_INFO(logger, "AutoAim Recorder Is Running Now: %s", record_item_path.c_str());
        } else if (!enable_forced) {
            start_record.store(false, std::memory_order::relaxed);
            RCLCPP_INFO(logger, "AutoAim Recorder Is Stopping Now");
        }
    }

    void save_image(const cv::Mat& image) const {
        const auto file = record_item_path + format_steady_time() + ".png";
        RCLCPP_INFO(logger, "Save Image Now: %s", file.c_str());
        cv::imwrite(file, image);
    }

    static std::string format_steady_time() {
        auto now     = std::chrono::steady_clock::now();
        auto sys_now = std::chrono::system_clock::now() + (now - std::chrono::steady_clock::now());
        auto in_time_t = std::chrono::system_clock::to_time_t(sys_now);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d_%H:%M:%S");
        return ss.str();
    }
};

Recorder::Recorder(rclcpp::Node& node)
    : pimpl(std::make_unique<Impl>(node)) {}

Recorder::~Recorder() = default;

bool Recorder::ready_save(const TimePoint& now) {
    if (!pimpl->enable_forced) {
        if (!pimpl->enable_record)
            return false;
        if (!pimpl->start_record)
            return false;
    }

    if (now - pimpl->last_time_point < pimpl->record_interval)
        return false;

    pimpl->last_time_point = now;
    return true;
}

void Recorder::save_image(const cv::Mat& image) { pimpl->save_image(image); }

void Recorder::set_record_status(bool is_start) { pimpl->set_record_status(is_start); }
