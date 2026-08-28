#ifndef AR4_JOINT_SPEED_TARGET_CONTRACT_H
#define AR4_JOINT_SPEED_TARGET_CONTRACT_H

#include <cmath>
#include <limits>

#include "cartesian_pose_contract.h"

namespace ar4_protocol {

using JointSpeedForwardKinematics = void (*)(const float *, float *);

inline bool native_joint_speed_target_to_external_degrees(
    const float (&native)[kCartesianPoseSize],
    float (&external_degrees)[kCartesianPoseSize]
) {
    float staged[kCartesianPoseSize] = {};
    if (!native_cartesian_pose_to_external(native, staged)) return false;

    for (int index = 3; index < kCartesianPoseSize; ++index) {
        const double converted =
            static_cast<double>(staged[index]) / kRadiansPerDegree;
        if (
            !std::isfinite(converted)
            || std::fabs(converted) > std::numeric_limits<float>::max()
        ) {
            return false;
        }
        const float converted_float = static_cast<float>(converted);
        if (staged[index] != 0.0f && converted_float == 0.0f) return false;
        staged[index] = converted_float;
    }

    for (int index = 0; index < kCartesianPoseSize; ++index) {
        external_degrees[index] = staged[index];
    }
    return true;
}

inline bool joint_speed_target_from_joints(
    const float *robot_joints_degrees,
    JointSpeedForwardKinematics forward_kinematics,
    float (&external_degrees)[kCartesianPoseSize]
) {
    if (robot_joints_degrees == nullptr || forward_kinematics == nullptr) {
        return false;
    }
    float native[kCartesianPoseSize] = {};
    forward_kinematics(robot_joints_degrees, native);
    return native_joint_speed_target_to_external_degrees(
        native,
        external_degrees
    );
}

}  // namespace ar4_protocol

#endif
