#ifndef AR4_TOOL_JOG_CONTRACT_H
#define AR4_TOOL_JOG_CONTRACT_H

#include <cmath>

#include "angle_conversion_contract.h"

namespace ar4_protocol {

constexpr float kLiveToolJogIncrement = 0.25f;

inline int tool_frame_axis_index(char axis) {
    if (axis == 'X') return 0;
    if (axis == 'Y') return 1;
    if (axis == 'Z') return 2;
    if (axis == 'W') return 3;
    if (axis == 'P') return 4;
    if (axis == 'R') return 5;
    return -1;
}

inline bool decode_discrete_tool_offset(
    char axis,
    char direction,
    float distance,
    int& frame_index,
    float& frame_offset
) {
    const int parsed_index = tool_frame_axis_index(axis);
    if (
        parsed_index < 0
        || (direction != '0' && direction != '1')
        || !std::isfinite(distance)
        || distance < 0.0f
    ) {
        return false;
    }
    const float signed_distance = direction == '1' ? distance : -distance;
    float native_offset = signed_distance;
    if (
        parsed_index >= 3
        && !degrees_to_radians(signed_distance, native_offset)
    ) {
        return false;
    }
    frame_index = parsed_index;
    frame_offset = native_offset;
    return true;
}

inline bool decode_live_tool_offset(
    float vector,
    float distance,
    int& frame_index,
    float& frame_offset
) {
    if (
        !std::isfinite(vector)
        || !std::isfinite(distance)
        || distance < 0.0f
        || vector < 10.0f
        || vector > 61.0f
    ) {
        return false;
    }

    const int encoded = static_cast<int>(vector);
    if (static_cast<float>(encoded) != vector) return false;
    const int direction = encoded % 10;
    if (direction != 0 && direction != 1) return false;

    const int axis_group = encoded / 10;
    int parsed_index = -1;
    if (axis_group == 1) parsed_index = 0;
    if (axis_group == 2) parsed_index = 1;
    if (axis_group == 3) parsed_index = 2;
    if (axis_group == 4) parsed_index = 5;
    if (axis_group == 5) parsed_index = 4;
    if (axis_group == 6) parsed_index = 3;
    if (parsed_index < 0) return false;

    const float signed_distance = direction == 0 ? distance : -distance;
    float native_offset = signed_distance;
    if (
        parsed_index >= 3
        && !degrees_to_radians(signed_distance, native_offset)
    ) {
        return false;
    }
    frame_index = parsed_index;
    frame_offset = native_offset;
    return true;
}

}  // namespace ar4_protocol

#endif
