#ifndef AR4_JSON_JOINT_MOTION_CONTRACT_H
#define AR4_JSON_JOINT_MOTION_CONTRACT_H

#if \
  defined(__GNUC__) \
  && !defined(__clang__) \
  && !defined(AR4_TEST_UNSUPPRESSED_ARDUINOJSON)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#endif
#include <ArduinoJson.h>
#if \
  defined(__GNUC__) \
  && !defined(__clang__) \
  && !defined(AR4_TEST_UNSUPPRESSED_ARDUINOJSON)
#pragma GCC diagnostic pop
#endif

#include <float.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "json_response_contract.h"

namespace ar4_protocol {

constexpr size_t kJsonJointMotionPrimaryAxisCount = 6;
constexpr size_t kJsonJointMotionExternalAxisCount = 3;
constexpr size_t kJsonJointMotionAxisCount =
  kJsonJointMotionPrimaryAxisCount + kJsonJointMotionExternalAxisCount;
constexpr size_t kJsonJointMotionControllerDebugMaximumLength = 31;
constexpr size_t kJsonJointMotionControllerDebugCapacity =
  kJsonJointMotionControllerDebugMaximumLength + 1;
constexpr size_t kJsonJointPositionTelemetryFrameCapacity = 256;
constexpr size_t kJsonJointMotionTerminalPayloadReservationBytes = 1024;

enum class JsonJointMotionSpeedMode : uint8_t {
  kPercent,
  kSeconds,
  kMillimetersPerSecond,
};

enum class JsonJointMotionWristConfiguration : uint8_t {
  kNear,
  kFar,
  kAutomatic,
};

struct JsonMainMoveJointsParameters {
  float robot_joints_degrees[kJsonJointMotionPrimaryAxisCount];
  float external_axes_units[kJsonJointMotionExternalAxisCount];
  JsonJointMotionSpeedMode speed_mode;
  float speed_value;
  float acceleration_percent;
  float deceleration_percent;
  float ramp_percent;
  JsonJointMotionWristConfiguration wrist_configuration;
  bool loop_modes[kJsonJointMotionPrimaryAxisCount];
  bool telemetry_enabled;
};

enum class JsonMainMoveJointsOutcome : uint8_t {
  kInvalid,
  kCompleted,
  kJointLimitViolation,
  kPositionNotRepresentable,
  kEmergencyStop,
  kPositionUnavailable,
  kMotionExecutionFailed,
  kEncoderCollision,
  kEncoderStateUnavailable,
};

struct JsonMainMoveJointsExecutionResult {
  JsonMainMoveJointsOutcome outcome;
  JsonMainPositionSnapshot position;
  bool axes[kJsonJointMotionAxisCount];
  bool speed_limited;
  char controller_debug[kJsonJointMotionControllerDebugCapacity];
};

namespace json_joint_motion_detail {

inline bool string_equals(
  ArduinoJson::JsonString value,
  const char *expected
) {
  if (!value || expected == nullptr) return false;
  const size_t expected_length = strlen(expected);
  return value.size() == expected_length
    && memcmp(value.c_str(), expected, expected_length) == 0;
}

inline bool extract_controller_float(
  ArduinoJson::JsonVariantConst value,
  float &output
) {
  if (value.is<bool>() || !value.is<double>()) return false;
  const double parsed = value.as<double>();
  if (!isfinite(parsed) || fabs(parsed) > FLT_MAX) return false;
  const float converted = static_cast<float>(parsed);
  if (
    !isfinite(converted)
    || (parsed != 0.0 && converted == 0.0f)
  ) {
    return false;
  }
  output = converted;
  return true;
}

inline bool extract_controller_float_array(
  ArduinoJson::JsonVariantConst value,
  float *output,
  size_t expected_count
) {
  if (
    output == nullptr
    || !value.is<ArduinoJson::JsonArrayConst>()
  ) {
    return false;
  }
  const ArduinoJson::JsonArrayConst values =
    value.as<ArduinoJson::JsonArrayConst>();
  if (values.size() != expected_count) return false;
  size_t index = 0;
  for (ArduinoJson::JsonVariantConst item : values) {
    if (!extract_controller_float(item, output[index++])) return false;
  }
  return index == expected_count;
}

inline bool extract_bool_array(
  ArduinoJson::JsonVariantConst value,
  bool *output,
  size_t expected_count
) {
  if (
    output == nullptr
    || !value.is<ArduinoJson::JsonArrayConst>()
  ) {
    return false;
  }
  const ArduinoJson::JsonArrayConst values =
    value.as<ArduinoJson::JsonArrayConst>();
  if (values.size() != expected_count) return false;
  size_t index = 0;
  for (ArduinoJson::JsonVariantConst item : values) {
    if (!item.is<bool>()) return false;
    output[index++] = item.as<bool>();
  }
  return index == expected_count;
}

inline bool controller_value_representable(double value) {
  if (!isfinite(value) || fabs(value) > FLT_MAX) return false;
  const float converted = static_cast<float>(value);
  return isfinite(converted)
    && (value == 0.0 || converted != 0.0f);
}

inline bool move_joints_parameters_valid(
  const JsonMainMoveJointsParameters &params
) {
  const bool speed_mode_valid =
    params.speed_mode == JsonJointMotionSpeedMode::kPercent
    || params.speed_mode == JsonJointMotionSpeedMode::kSeconds
    || params.speed_mode
      == JsonJointMotionSpeedMode::kMillimetersPerSecond;
  const bool wrist_configuration_valid =
    params.wrist_configuration
      == JsonJointMotionWristConfiguration::kNear
    || params.wrist_configuration
      == JsonJointMotionWristConfiguration::kFar
    || params.wrist_configuration
      == JsonJointMotionWristConfiguration::kAutomatic;
  const bool speed_valid = speed_mode_valid
    && params.speed_value > 0.0f
    && (
      params.speed_mode != JsonJointMotionSpeedMode::kPercent
      || params.speed_value <= 100.0f
    )
    && (
      params.speed_mode != JsonJointMotionSpeedMode::kSeconds
      || controller_value_representable(
        static_cast<double>(params.speed_value) * 1000000.0
      )
    );
  const double combined_profile =
    static_cast<double>(params.acceleration_percent)
      + static_cast<double>(params.deceleration_percent);
  const bool profile_valid = params.acceleration_percent > 0.0f
    && params.acceleration_percent <= 100.0f
    && params.deceleration_percent > 0.0f
    && params.deceleration_percent < 100.0f
    && controller_value_representable(combined_profile)
    && static_cast<float>(combined_profile) <= 100.0f
    && params.ramp_percent > 0.0f
    && params.ramp_percent <= 100.0f;
  return speed_valid && profile_valid && wrist_configuration_valid;
}

inline bool position_snapshot_empty(
  const JsonMainPositionSnapshot &snapshot
) {
  for (size_t axis = 0; axis < kJsonJointMotionPrimaryAxisCount; ++axis) {
    if (snapshot.robot_joints_millidegrees[axis] != 0) return false;
  }
  for (size_t axis = 0; axis < kJsonJointMotionExternalAxisCount; ++axis) {
    if (
      snapshot.external_axes_milliunits[axis] != 0
      || snapshot.cartesian_micrometers[axis] != 0
      || snapshot.orientation_millidegrees[axis] != 0
    ) {
      return false;
    }
  }
  return true;
}

inline bool axes_empty(
  const bool *axes,
  size_t count
) {
  if (axes == nullptr) return false;
  for (size_t axis = 0; axis < count; ++axis) {
    if (axes[axis]) return false;
  }
  return true;
}

inline bool any_axis_set(
  const bool *axes,
  size_t count
) {
  if (axes == nullptr) return false;
  for (size_t axis = 0; axis < count; ++axis) {
    if (axes[axis]) return true;
  }
  return false;
}

inline bool execution_result_valid(
  const JsonMainMoveJointsExecutionResult &result
) {
  const bool debug_valid = json_protocol_text_valid(
    result.controller_debug,
    kJsonJointMotionControllerDebugMaximumLength,
    true
  );
  const bool all_axes_empty = axes_empty(
    result.axes,
    kJsonJointMotionAxisCount
  );
  const bool inactive_fields_empty = !result.speed_limited
    && result.controller_debug[0] == '\0';
  switch (result.outcome) {
    case JsonMainMoveJointsOutcome::kCompleted:
      return debug_valid && all_axes_empty;
    case JsonMainMoveJointsOutcome::kJointLimitViolation:
    case JsonMainMoveJointsOutcome::kPositionNotRepresentable:
      return inactive_fields_empty
        && position_snapshot_empty(result.position)
        && any_axis_set(result.axes, kJsonJointMotionAxisCount);
    case JsonMainMoveJointsOutcome::kEmergencyStop:
    case JsonMainMoveJointsOutcome::kMotionExecutionFailed:
      return inactive_fields_empty && all_axes_empty;
    case JsonMainMoveJointsOutcome::kPositionUnavailable:
      return inactive_fields_empty
        && all_axes_empty
        && position_snapshot_empty(result.position);
    case JsonMainMoveJointsOutcome::kEncoderCollision:
    case JsonMainMoveJointsOutcome::kEncoderStateUnavailable:
      return inactive_fields_empty
        && any_axis_set(
          result.axes,
          kJsonJointMotionPrimaryAxisCount
        )
        && axes_empty(
          result.axes + kJsonJointMotionPrimaryAxisCount,
          kJsonJointMotionExternalAxisCount
        );
    case JsonMainMoveJointsOutcome::kInvalid:
      return false;
  }
  return false;
}

inline bool append_position_snapshot(
  const JsonMainPositionSnapshot &snapshot,
  char *output,
  size_t output_capacity,
  size_t &index
) {
  return append_main_json_position_snapshot(
    snapshot,
    output,
    output_capacity,
    index
  );
}

enum class ErrorDetailKind : uint8_t {
  kNone,
  kAxes,
  kPosition,
  kAxesAndPosition,
};

struct ErrorDescriptor {
  JsonErrorResponseStatus status;
  const char *code;
  const char *message;
  ErrorDetailKind details;
  size_t axis_count;
};

inline bool error_descriptor(
  JsonMainMoveJointsOutcome outcome,
  ErrorDescriptor &descriptor
) {
  switch (outcome) {
    case JsonMainMoveJointsOutcome::kJointLimitViolation:
      descriptor = {
        JsonErrorResponseStatus::kRejected,
        "joint_limit_violation",
        "requested joint target exceeds configured limits",
        ErrorDetailKind::kAxes,
        kJsonJointMotionAxisCount,
      };
      return true;
    case JsonMainMoveJointsOutcome::kPositionNotRepresentable:
      descriptor = {
        JsonErrorResponseStatus::kRejected,
        "position_not_representable",
        "requested joint position cannot be represented",
        ErrorDetailKind::kAxes,
        kJsonJointMotionAxisCount,
      };
      return true;
    case JsonMainMoveJointsOutcome::kEmergencyStop:
      descriptor = {
        JsonErrorResponseStatus::kCancelled,
        "emergency_stop",
        "emergency stop interrupted joint motion",
        ErrorDetailKind::kPosition,
        0,
      };
      return true;
    case JsonMainMoveJointsOutcome::kPositionUnavailable:
      descriptor = {
        JsonErrorResponseStatus::kFailed,
        "position_unavailable",
        "controller position is unavailable",
        ErrorDetailKind::kNone,
        0,
      };
      return true;
    case JsonMainMoveJointsOutcome::kMotionExecutionFailed:
      descriptor = {
        JsonErrorResponseStatus::kFailed,
        "motion_execution_failed",
        "joint motion could not complete",
        ErrorDetailKind::kPosition,
        0,
      };
      return true;
    case JsonMainMoveJointsOutcome::kEncoderCollision:
      descriptor = {
        JsonErrorResponseStatus::kFailed,
        "encoder_collision",
        "encoder collision detected during joint motion",
        ErrorDetailKind::kAxesAndPosition,
        kJsonJointMotionPrimaryAxisCount,
      };
      return true;
    case JsonMainMoveJointsOutcome::kEncoderStateUnavailable:
      descriptor = {
        JsonErrorResponseStatus::kFailed,
        "encoder_state_unavailable",
        "encoder state is unavailable",
        ErrorDetailKind::kAxesAndPosition,
        kJsonJointMotionPrimaryAxisCount,
      };
      return true;
    case JsonMainMoveJointsOutcome::kInvalid:
    case JsonMainMoveJointsOutcome::kCompleted:
      return false;
  }
  return false;
}

inline bool append_error_details(
  const JsonMainMoveJointsExecutionResult &result,
  const ErrorDescriptor &descriptor,
  char *output,
  size_t output_capacity,
  size_t &index
) {
  if (!append_json_text("{", output, output_capacity, index)) return false;
  if (
    descriptor.details == ErrorDetailKind::kAxes
    || descriptor.details == ErrorDetailKind::kAxesAndPosition
  ) {
    if (
      !append_json_text("\"axes\":", output, output_capacity, index)
      || !append_json_bool_array(
        result.axes,
        descriptor.axis_count,
        output,
        output_capacity,
        index
      )
    ) {
      return false;
    }
  }
  if (
    descriptor.details == ErrorDetailKind::kPosition
    || descriptor.details == ErrorDetailKind::kAxesAndPosition
  ) {
    if (
      descriptor.details == ErrorDetailKind::kAxesAndPosition
      && !append_json_text(",", output, output_capacity, index)
    ) {
      return false;
    }
    if (
      !append_json_text("\"position\":", output, output_capacity, index)
      || !append_position_snapshot(
        result.position,
        output,
        output_capacity,
        index
      )
    ) {
      return false;
    }
  }
  return append_json_text("}", output, output_capacity, index);
}

}  // namespace json_joint_motion_detail

inline bool json_main_move_joints_execution_result_valid(
  const JsonMainMoveJointsExecutionResult &result
) {
  return json_joint_motion_detail::execution_result_valid(result);
}

inline bool extract_main_move_joints_parameters(
  ArduinoJson::JsonObjectConst params,
  JsonMainMoveJointsParameters &output
) {
  if (params.size() != 10) return false;
  JsonMainMoveJointsParameters staged = {};
  bool robot_joints_present = false;
  bool external_axes_present = false;
  bool speed_mode_present = false;
  bool speed_value_present = false;
  bool acceleration_present = false;
  bool deceleration_present = false;
  bool ramp_present = false;
  bool wrist_present = false;
  bool loop_modes_present = false;
  bool telemetry_present = false;
  for (ArduinoJson::JsonPairConst pair : params) {
    const ArduinoJson::JsonString key = pair.key();
    if (json_joint_motion_detail::string_equals(
        key,
        "robot_joints_degrees"
    )) {
      if (
        robot_joints_present
        || !json_joint_motion_detail::extract_controller_float_array(
          pair.value(),
          staged.robot_joints_degrees,
          kJsonJointMotionPrimaryAxisCount
        )
      ) return false;
      robot_joints_present = true;
    } else if (json_joint_motion_detail::string_equals(
        key,
        "external_axes_units"
    )) {
      if (
        external_axes_present
        || !json_joint_motion_detail::extract_controller_float_array(
          pair.value(),
          staged.external_axes_units,
          kJsonJointMotionExternalAxisCount
        )
      ) return false;
      external_axes_present = true;
    } else if (json_joint_motion_detail::string_equals(key, "speed_mode")) {
      if (speed_mode_present) return false;
      const ArduinoJson::JsonString value =
        pair.value().as<ArduinoJson::JsonString>();
      if (json_joint_motion_detail::string_equals(value, "percent")) {
        staged.speed_mode = JsonJointMotionSpeedMode::kPercent;
      } else if (json_joint_motion_detail::string_equals(value, "seconds")) {
        staged.speed_mode = JsonJointMotionSpeedMode::kSeconds;
      } else if (json_joint_motion_detail::string_equals(
          value,
          "millimeters_per_second"
      )) {
        staged.speed_mode =
          JsonJointMotionSpeedMode::kMillimetersPerSecond;
      } else {
        return false;
      }
      speed_mode_present = true;
    } else if (json_joint_motion_detail::string_equals(key, "speed_value")) {
      if (
        speed_value_present
        || !json_joint_motion_detail::extract_controller_float(
          pair.value(),
          staged.speed_value
        )
      ) return false;
      speed_value_present = true;
    } else if (json_joint_motion_detail::string_equals(
        key,
        "acceleration_percent"
    )) {
      if (
        acceleration_present
        || !json_joint_motion_detail::extract_controller_float(
          pair.value(),
          staged.acceleration_percent
        )
      ) return false;
      acceleration_present = true;
    } else if (json_joint_motion_detail::string_equals(
        key,
        "deceleration_percent"
    )) {
      if (
        deceleration_present
        || !json_joint_motion_detail::extract_controller_float(
          pair.value(),
          staged.deceleration_percent
        )
      ) return false;
      deceleration_present = true;
    } else if (json_joint_motion_detail::string_equals(key, "ramp_percent")) {
      if (
        ramp_present
        || !json_joint_motion_detail::extract_controller_float(
          pair.value(),
          staged.ramp_percent
        )
      ) return false;
      ramp_present = true;
    } else if (json_joint_motion_detail::string_equals(
        key,
        "wrist_configuration"
    )) {
      if (wrist_present) return false;
      const ArduinoJson::JsonString value =
        pair.value().as<ArduinoJson::JsonString>();
      if (json_joint_motion_detail::string_equals(value, "near")) {
        staged.wrist_configuration =
          JsonJointMotionWristConfiguration::kNear;
      } else if (json_joint_motion_detail::string_equals(value, "far")) {
        staged.wrist_configuration =
          JsonJointMotionWristConfiguration::kFar;
      } else if (json_joint_motion_detail::string_equals(
          value,
          "automatic"
      )) {
        staged.wrist_configuration =
          JsonJointMotionWristConfiguration::kAutomatic;
      } else {
        return false;
      }
      wrist_present = true;
    } else if (json_joint_motion_detail::string_equals(key, "loop_modes")) {
      if (
        loop_modes_present
        || !json_joint_motion_detail::extract_bool_array(
          pair.value(),
          staged.loop_modes,
          kJsonJointMotionPrimaryAxisCount
        )
      ) return false;
      loop_modes_present = true;
    } else if (json_joint_motion_detail::string_equals(
        key,
        "telemetry_enabled"
    )) {
      if (telemetry_present || !pair.value().is<bool>()) return false;
      staged.telemetry_enabled = pair.value().as<bool>();
      telemetry_present = true;
    } else {
      return false;
    }
  }
  if (
    !robot_joints_present
    || !external_axes_present
    || !speed_mode_present
    || !speed_value_present
    || !acceleration_present
    || !deceleration_present
    || !ramp_present
    || !wrist_present
    || !loop_modes_present
    || !telemetry_present
    || !json_joint_motion_detail::move_joints_parameters_valid(staged)
  ) {
    return false;
  }
  output = staged;
  return true;
}

inline bool build_main_json_move_joints_response(
  uint32_t request_id,
  const JsonMainMoveJointsExecutionResult &result,
  size_t maximum_payload_bytes,
  char *output,
  size_t output_capacity
) {
  if (output == nullptr || output_capacity == 0) return false;
  output[0] = '\0';
  if (
    request_id == 0
    || maximum_payload_bytes == 0
    || maximum_payload_bytes > kJsonProtocolMaximumPayloadBytes
    || !json_joint_motion_detail::execution_result_valid(result)
  ) {
    return false;
  }
  const size_t bounded_capacity = output_capacity < maximum_payload_bytes + 1
    ? output_capacity
    : maximum_payload_bytes + 1;
  size_t index = 0;
  bool built = false;
  if (result.outcome == JsonMainMoveJointsOutcome::kCompleted) {
    built = append_json_text(
        "{\"cmd\":\"move_joints\",\"id\":",
        output,
        bounded_capacity,
        index
      )
      && append_json_uint32(request_id, output, bounded_capacity, index)
      && append_json_text(
        ",\"result\":{\"controller_debug\":\"",
        output,
        bounded_capacity,
        index
      )
      && append_json_escaped_text(
        result.controller_debug,
        kJsonJointMotionControllerDebugMaximumLength,
        true,
        output,
        bounded_capacity,
        index
      )
      && append_json_text(
        "\",\"position\":",
        output,
        bounded_capacity,
        index
      )
      && json_joint_motion_detail::append_position_snapshot(
        result.position,
        output,
        bounded_capacity,
        index
      )
      && append_json_text(
        ",\"speed_limited\":",
        output,
        bounded_capacity,
        index
      )
      && append_json_text(
        result.speed_limited ? "true" : "false",
        output,
        bounded_capacity,
        index
      )
      && append_json_text(
        "},\"status\":\"completed\",\"type\":\"response\",\"v\":1}",
        output,
        bounded_capacity,
        index
      );
  } else {
    json_joint_motion_detail::ErrorDescriptor descriptor = {};
    if (!json_joint_motion_detail::error_descriptor(
        result.outcome,
        descriptor
    )) {
      return false;
    }
    const char *status_name = json_error_response_status_name(
      descriptor.status
    );
    built = status_name != nullptr
      && append_json_text(
        "{\"cmd\":\"move_joints\",\"error\":{\"code\":\"",
        output,
        bounded_capacity,
        index
      )
      && append_json_text(
        descriptor.code,
        output,
        bounded_capacity,
        index
      )
      && append_json_text(
        "\",\"details\":",
        output,
        bounded_capacity,
        index
      )
      && json_joint_motion_detail::append_error_details(
        result,
        descriptor,
        output,
        bounded_capacity,
        index
      )
      && append_json_text(
        ",\"message\":\"",
        output,
        bounded_capacity,
        index
      )
      && append_json_escaped_text(
        descriptor.message,
        kJsonProtocolMaximumErrorMessageLength,
        false,
        output,
        bounded_capacity,
        index
      )
      && append_json_text("\"},\"id\":", output, bounded_capacity, index)
      && append_json_uint32(request_id, output, bounded_capacity, index)
      && append_json_text(
        ",\"status\":\"",
        output,
        bounded_capacity,
        index
      )
      && append_json_text(status_name, output, bounded_capacity, index)
      && append_json_text(
        "\",\"type\":\"response\",\"v\":1}",
        output,
        bounded_capacity,
        index
      );
  }
  if (!built) output[0] = '\0';
  return built;
}

inline bool build_json_joint_position_telemetry_frame(
  uint32_t sequence,
  const int32_t (&robot_joints_millidegrees)[
    kJsonJointMotionPrimaryAxisCount
  ],
  char *output,
  size_t output_capacity,
  size_t &output_length
) {
  output_length = 0;
  if (output == nullptr || output_capacity == 0) return false;
  output[0] = '\0';
  size_t index = 0;
  const bool built = append_json_text(
      "{\"data\":{\"robot_joints_millidegrees\":",
      output,
      output_capacity,
      index
    )
    && append_json_int32_array(
      robot_joints_millidegrees,
      kJsonJointMotionPrimaryAxisCount,
      output,
      output_capacity,
      index
    )
    && append_json_text(
      "},\"seq\":",
      output,
      output_capacity,
      index
    )
    && append_json_uint32(sequence, output, output_capacity, index)
    && append_json_text(
      ",\"stream\":\"joint_position\",\"type\":\"telemetry\",\"v\":1}",
      output,
      output_capacity,
      index
    );
  if (
    !built
    || index == 0
    || index > kJsonProtocolMaximumPayloadBytes
    || index + 2 > output_capacity
  ) {
    output[0] = '\0';
    return false;
  }
  output[index] = '\n';
  output[index + 1] = '\0';
  output_length = index + 1;
  return true;
}

}  // namespace ar4_protocol

#endif
