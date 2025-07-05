#pragma once

#include <cstdint>
namespace rmcs_auto_aim::tracker {
enum class TrackerState : uint8_t { Lost, NearlyTrack, Track, NearlyLost };
}