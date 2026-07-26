#ifndef AR4_ANGLE_CONVERSION_CONTRACT_H
#define AR4_ANGLE_CONVERSION_CONTRACT_H

#include <cmath>
#include <limits>

namespace ar4_protocol {

constexpr double kRadiansPerDegree =
    0.017453292519943295769236907684886;

inline bool degrees_to_radians(float degrees, float& radians) {
    if (!std::isfinite(degrees)) return false;
    const double converted =
        static_cast<double>(degrees) * kRadiansPerDegree;
    if (
        !std::isfinite(converted)
        || std::fabs(converted) > std::numeric_limits<float>::max()
    ) {
        return false;
    }
    const float staged = static_cast<float>(converted);
    if (degrees != 0.0f && staged == 0.0f) return false;
    const double round_trip_degrees =
        static_cast<double>(staged) / kRadiansPerDegree;
    if (
        !std::isfinite(round_trip_degrees)
        || std::fabs(round_trip_degrees) > std::numeric_limits<float>::max()
    ) {
        return false;
    }
    radians = staged;
    return true;
}

}  // namespace ar4_protocol

#endif
