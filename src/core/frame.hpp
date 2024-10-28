
#include <cstddef>
#include <type_traits>
#include <vector>

#include <opencv2/core/mat.hpp>

#include "core/identifier/armor/armor.hpp"
#include "core/pnpsolver/armor/armor3d.hpp"

namespace rmcs_auto_aim {

template <typename T>
requires std::is_same_v<T, cv::Mat> || std::is_same_v<T, std::vector<ArmorPlate>>
      || std::is_same_v<T, std::vector<ArmorPlate3d>> struct Frame {
    size_t frame_id_;
    T data_;
};
} // namespace rmcs_auto_aim
