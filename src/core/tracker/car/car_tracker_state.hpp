
#include <cstdint>
namespace rmcs_auto_aim::tracker {
enum class CarTrackerState : uint8_t { Lost, NearlyTrack, Track, NearlyLost };
}