
#include <cstddef>
#include <opencv2/core/mat.hpp>

namespace rmcs_auto_aim {
struct Frame {
    size_t frame_id;
    cv::Mat img;
};
} // namespace rmcs_auto_aim
