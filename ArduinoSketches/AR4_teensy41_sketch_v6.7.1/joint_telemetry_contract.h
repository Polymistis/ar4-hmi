#ifndef AR4_JOINT_TELEMETRY_CONTRACT_H
#define AR4_JOINT_TELEMETRY_CONTRACT_H

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

namespace ar4_protocol {

constexpr size_t kJointTelemetryAxisCount = 6;
constexpr size_t kJointTelemetryFrameCapacity = 96;
constexpr uint32_t kJointTelemetryPeriodMicroseconds = 100000;
constexpr int kJointTelemetryTerminalReserveBytes = 2048;
constexpr int kJointTelemetryMinimumWriteCapacity =
    static_cast<int>(kJointTelemetryFrameCapacity)
    + kJointTelemetryTerminalReserveBytes;

struct JointTelemetryResponseOwnership {
    volatile bool active;
    volatile bool estop_response_pending;
    volatile bool estop_admission_blocked;
};

enum class JointTelemetryTerminalKind : uint8_t {
    kNotOwned,
    kAlreadySent,
    kPosition,
    kError,
};

struct JointTelemetryTerminalDecision {
    JointTelemetryTerminalKind kind;
    bool emergency_stop;
};

inline void begin_joint_telemetry_response_ownership(
    bool telemetry_requested,
    volatile JointTelemetryResponseOwnership& ownership
) {
    if (!telemetry_requested) return;
    ownership.estop_response_pending = false;
    ownership.active = true;
}

inline bool defer_joint_telemetry_estop_response(
    volatile JointTelemetryResponseOwnership& ownership
) {
    if (!ownership.active) return false;
    ownership.estop_response_pending = true;
    return true;
}

inline JointTelemetryTerminalDecision decide_joint_telemetry_terminal(
    bool telemetry_requested,
    bool drive_succeeded,
    bool estop_active,
    const volatile JointTelemetryResponseOwnership& ownership
) {
    if (!telemetry_requested || !ownership.active) {
        return {
            JointTelemetryTerminalKind::kNotOwned,
            false,
        };
    }
    if (estop_active && !ownership.estop_response_pending) {
        return {
            JointTelemetryTerminalKind::kAlreadySent,
            true,
        };
    }
    if (estop_active || drive_succeeded) {
        return {
            JointTelemetryTerminalKind::kPosition,
            estop_active,
        };
    }
    return {
        JointTelemetryTerminalKind::kError,
        false,
    };
}

inline bool commit_joint_telemetry_terminal(
    const JointTelemetryTerminalDecision& decision,
    volatile JointTelemetryResponseOwnership& ownership
) {
    if (
        !ownership.active
        || decision.kind == JointTelemetryTerminalKind::kNotOwned
    ) {
        return false;
    }
    const bool late_estop =
        ownership.estop_response_pending && !decision.emergency_stop;
    if (late_estop) {
        ownership.estop_admission_blocked = true;
    }
    ownership.active = false;
    ownership.estop_response_pending = false;
    return late_estop;
}

inline bool joint_telemetry_estop_admission_blocked(
    const volatile JointTelemetryResponseOwnership& ownership
) {
    return ownership.estop_admission_blocked;
}

inline void clear_joint_telemetry_estop_admission_block(
    volatile JointTelemetryResponseOwnership& ownership
) {
    ownership.estop_admission_blocked = false;
}

inline bool joint_telemetry_due(
    uint32_t now_microseconds,
    uint32_t last_attempt_microseconds
) {
    return static_cast<uint32_t>(
        now_microseconds - last_attempt_microseconds
    ) >= kJointTelemetryPeriodMicroseconds;
}

inline bool encoder_count_to_joint_millidegrees(
    int32_t encoder_count,
    float encoder_multiplier,
    int32_t zero_step,
    float steps_per_degree,
    int32_t& output
) {
    if (
        !isfinite(encoder_multiplier)
        || encoder_multiplier <= 0.0f
        || !isfinite(steps_per_degree)
        || steps_per_degree <= 0.0f
    ) {
        return false;
    }
    const double encoder_step =
        static_cast<double>(encoder_count) / encoder_multiplier;
    const double degrees =
        (encoder_step - static_cast<double>(zero_step))
        / steps_per_degree;
    const double scaled = degrees * 1000.0;
    if (
        !isfinite(scaled)
        || scaled < static_cast<double>(INT32_MIN)
        || scaled > static_cast<double>(INT32_MAX)
    ) {
        return false;
    }
    const double rounded = scaled >= 0.0
        ? floor(scaled + 0.5)
        : ceil(scaled - 0.5);
    if (
        rounded < static_cast<double>(INT32_MIN)
        || rounded > static_cast<double>(INT32_MAX)
    ) {
        return false;
    }
    output = static_cast<int32_t>(rounded);
    return true;
}

inline bool encoder_counts_to_joint_millidegrees(
    const int32_t (&encoder_counts)[kJointTelemetryAxisCount],
    const float (&encoder_multipliers)[kJointTelemetryAxisCount],
    const int32_t (&zero_steps)[kJointTelemetryAxisCount],
    const float (&steps_per_degree)[kJointTelemetryAxisCount],
    int32_t (&output)[kJointTelemetryAxisCount]
) {
    int32_t staged[kJointTelemetryAxisCount] = {};
    for (size_t axis = 0; axis < kJointTelemetryAxisCount; ++axis) {
        if (!encoder_count_to_joint_millidegrees(
            encoder_counts[axis],
            encoder_multipliers[axis],
            zero_steps[axis],
            steps_per_degree[axis],
            staged[axis]
        )) {
            return false;
        }
    }
    for (size_t axis = 0; axis < kJointTelemetryAxisCount; ++axis) {
        output[axis] = staged[axis];
    }
    return true;
}

inline bool build_joint_telemetry_frame(
    const int32_t (&millidegrees)[kJointTelemetryAxisCount],
    char* output,
    size_t output_capacity,
    size_t& output_length
) {
    if (output == nullptr || output_capacity == 0) return false;

    char staged[kJointTelemetryFrameCapacity] = {};
    const int written = snprintf(
        staged,
        sizeof(staged),
        "TMA%ldB%ldC%ldD%ldE%ldF%ld\n",
        static_cast<long>(millidegrees[0]),
        static_cast<long>(millidegrees[1]),
        static_cast<long>(millidegrees[2]),
        static_cast<long>(millidegrees[3]),
        static_cast<long>(millidegrees[4]),
        static_cast<long>(millidegrees[5])
    );
    if (
        written <= 0
        || static_cast<size_t>(written) >= sizeof(staged)
        || static_cast<size_t>(written) + 1 > output_capacity
    ) {
        return false;
    }
    memcpy(output, staged, static_cast<size_t>(written) + 1);
    output_length = static_cast<size_t>(written);
    return true;
}

}

#endif
