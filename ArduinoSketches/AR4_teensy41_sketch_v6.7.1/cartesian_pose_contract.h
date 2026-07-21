#ifndef AR4_CARTESIAN_POSE_CONTRACT_H
#define AR4_CARTESIAN_POSE_CONTRACT_H

#include <cmath>

#include "angle_conversion_contract.h"

namespace ar4_protocol {

constexpr int kCartesianPoseSize = 6;

inline bool reorder_cartesian_rotation_axes(
    const float (&source)[kCartesianPoseSize],
    float (&destination)[kCartesianPoseSize]
) {
    for (int index = 0; index < kCartesianPoseSize; ++index) {
        if (!std::isfinite(source[index])) return false;
    }

    const float reordered[kCartesianPoseSize] = {
        source[0],
        source[1],
        source[2],
        source[5],
        source[4],
        source[3],
    };
    for (int index = 0; index < kCartesianPoseSize; ++index) {
        destination[index] = reordered[index];
    }
    return true;
}

inline bool external_cartesian_pose_to_native(
    const float (&external)[kCartesianPoseSize],
    float (&native)[kCartesianPoseSize]
) {
    return reorder_cartesian_rotation_axes(external, native);
}

inline bool external_cartesian_pose_to_native_radians(
    const float (&external)[kCartesianPoseSize],
    float (&native)[kCartesianPoseSize]
) {
    float staged[kCartesianPoseSize] = {};
    if (!external_cartesian_pose_to_native(external, staged)) return false;
    for (int index = 3; index < kCartesianPoseSize; ++index) {
        float radians = 0.0f;
        if (!degrees_to_radians(staged[index], radians)) return false;
        staged[index] = radians;
    }
    for (int index = 0; index < kCartesianPoseSize; ++index) {
        native[index] = staged[index];
    }
    return true;
}

inline bool native_cartesian_pose_to_external(
    const float (&native)[kCartesianPoseSize],
    float (&external)[kCartesianPoseSize]
) {
    return reorder_cartesian_rotation_axes(native, external);
}

}  // namespace ar4_protocol

#endif
