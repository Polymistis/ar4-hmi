#ifndef AR4_JSON_CARTESIAN_MOTION_CONTRACT_H
#define AR4_JSON_CARTESIAN_MOTION_CONTRACT_H

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "json_joint_motion_contract.h"

namespace ar4_protocol {

constexpr size_t kJsonCartesianMotionTranslationCount = 3;
constexpr size_t kJsonCartesianMotionOrientationCount = 3;
constexpr size_t kJsonCartesianMotionExternalAxisCount = 3;
constexpr size_t kJsonCartesianMotionPrimaryAxisCount = 6;
constexpr size_t kJsonCartesianMotionAxisCount =
  kJsonCartesianMotionPrimaryAxisCount
  + kJsonCartesianMotionExternalAxisCount;
constexpr size_t kJsonCartesianMotionControllerDebugMaximumLength = 31;
constexpr size_t kJsonCartesianMotionControllerDebugCapacity =
  kJsonCartesianMotionControllerDebugMaximumLength + 1;
constexpr size_t kJsonCartesianMotionTerminalPayloadReservationBytes = 1024;

enum class JsonCartesianMotionSpeedMode : uint8_t {
  kPercent,
  kSeconds,
  kMillimetersPerSecond,
};

enum class JsonCartesianMotionWristConfiguration : uint8_t {
  kAutomatic,
  kNear,
  kFar,
};

struct JsonMainMoveCartesianParameters {
  float translation_millimeters[kJsonCartesianMotionTranslationCount];
  // The JSON boundary uses user-facing Rx, Ry, Rz order.
  float orientation_degrees[kJsonCartesianMotionOrientationCount];
  float external_axes_units[kJsonCartesianMotionExternalAxisCount];
  JsonCartesianMotionSpeedMode speed_mode;
  float speed_value;
  float acceleration_percent;
  float deceleration_percent;
  float ramp_percent;
  JsonCartesianMotionWristConfiguration wrist_configuration;
  bool loop_modes[kJsonCartesianMotionPrimaryAxisCount];
  bool telemetry_enabled;
};

enum class JsonMainMoveCartesianOutcome : uint8_t {
  kInvalid,
  kCompleted,
  kKinematicsUnreachable,
  kJointLimitViolation,
  kPositionNotRepresentable,
  kEmergencyStop,
  kPositionUnavailable,
  kMotionExecutionFailed,
  kEncoderCollision,
  kEncoderStateUnavailable,
};

struct JsonMainMoveCartesianExecutionResult {
  JsonMainMoveCartesianOutcome outcome;
  JsonMainPositionSnapshot position;
  bool axes[kJsonCartesianMotionAxisCount];
  bool speed_limited;
  char controller_debug[kJsonCartesianMotionControllerDebugCapacity];
};

namespace json_cartesian_motion_detail {

inline bool parameters_valid(
  const JsonMainMoveCartesianParameters &params
) {
  const bool speed_mode_valid =
    params.speed_mode == JsonCartesianMotionSpeedMode::kPercent
    || params.speed_mode == JsonCartesianMotionSpeedMode::kSeconds
    || params.speed_mode
      == JsonCartesianMotionSpeedMode::kMillimetersPerSecond;
  const bool wrist_configuration_valid =
    params.wrist_configuration
      == JsonCartesianMotionWristConfiguration::kAutomatic
    || params.wrist_configuration
      == JsonCartesianMotionWristConfiguration::kNear
    || params.wrist_configuration
      == JsonCartesianMotionWristConfiguration::kFar;
  const bool speed_valid = speed_mode_valid
    && params.speed_value > 0.0f
    && (
      params.speed_mode != JsonCartesianMotionSpeedMode::kPercent
      || params.speed_value <= 100.0f
    )
    && (
      params.speed_mode != JsonCartesianMotionSpeedMode::kSeconds
      || json_joint_motion_detail::controller_value_representable(
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
    && json_joint_motion_detail::controller_value_representable(
      combined_profile
    )
    && static_cast<float>(combined_profile) <= 100.0f
    && params.ramp_percent > 0.0f
    && params.ramp_percent <= 100.0f;
  if (
    !speed_valid
    || !profile_valid
    || !wrist_configuration_valid
  ) return false;
  constexpr double kRadiansPerDegree =
    0.017453292519943295769236907684886;
  for (
    size_t axis = 0;
    axis < kJsonCartesianMotionOrientationCount;
    ++axis
  ) {
    if (!json_joint_motion_detail::controller_value_representable(
        static_cast<double>(params.orientation_degrees[axis])
          * kRadiansPerDegree
    )) {
      return false;
    }
  }
  return true;
}

inline bool execution_result_valid(
  const JsonMainMoveCartesianExecutionResult &result
) {
  const bool debug_valid = json_protocol_text_valid(
    result.controller_debug,
    kJsonCartesianMotionControllerDebugMaximumLength,
    true
  );
  const bool all_axes_empty = json_joint_motion_detail::axes_empty(
    result.axes,
    kJsonCartesianMotionAxisCount
  );
  const bool inactive_fields_empty = !result.speed_limited
    && result.controller_debug[0] == '\0';
  switch (result.outcome) {
    case JsonMainMoveCartesianOutcome::kCompleted:
      return debug_valid && all_axes_empty;
    case JsonMainMoveCartesianOutcome::kKinematicsUnreachable:
      return inactive_fields_empty
        && all_axes_empty
        && json_joint_motion_detail::position_snapshot_empty(
          result.position
        );
    case JsonMainMoveCartesianOutcome::kJointLimitViolation:
    case JsonMainMoveCartesianOutcome::kPositionNotRepresentable:
      return inactive_fields_empty
        && json_joint_motion_detail::position_snapshot_empty(
          result.position
        )
        && json_joint_motion_detail::any_axis_set(
          result.axes,
          kJsonCartesianMotionAxisCount
        );
    case JsonMainMoveCartesianOutcome::kEmergencyStop:
    case JsonMainMoveCartesianOutcome::kMotionExecutionFailed:
      return inactive_fields_empty && all_axes_empty;
    case JsonMainMoveCartesianOutcome::kPositionUnavailable:
      return inactive_fields_empty
        && all_axes_empty
        && json_joint_motion_detail::position_snapshot_empty(
          result.position
        );
    case JsonMainMoveCartesianOutcome::kEncoderCollision:
    case JsonMainMoveCartesianOutcome::kEncoderStateUnavailable:
      return inactive_fields_empty
        && json_joint_motion_detail::any_axis_set(
          result.axes,
          kJsonCartesianMotionPrimaryAxisCount
        )
        && json_joint_motion_detail::axes_empty(
          result.axes + kJsonCartesianMotionPrimaryAxisCount,
          kJsonCartesianMotionExternalAxisCount
        );
    case JsonMainMoveCartesianOutcome::kInvalid:
      return false;
  }
  return false;
}

inline bool execution_result_valid_for_command(
  const char *command,
  const JsonMainMoveCartesianExecutionResult &result
) {
  if (execution_result_valid(result)) return true;
  return command != nullptr
    && strcmp(command, "jog_tool") == 0
    && result.outcome
      == JsonMainMoveCartesianOutcome::kPositionNotRepresentable
    && !result.speed_limited
    && result.controller_debug[0] == '\0'
    && json_joint_motion_detail::axes_empty(
      result.axes,
      kJsonCartesianMotionAxisCount
    )
    && json_joint_motion_detail::position_snapshot_empty(result.position);
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
  JsonMainMoveCartesianOutcome outcome,
  ErrorDescriptor &descriptor
) {
  switch (outcome) {
    case JsonMainMoveCartesianOutcome::kKinematicsUnreachable:
      descriptor = {
        JsonErrorResponseStatus::kRejected,
        "kinematics_unreachable",
        "Cartesian target has no selected inverse solution",
        ErrorDetailKind::kNone,
        0,
      };
      return true;
    case JsonMainMoveCartesianOutcome::kJointLimitViolation:
      descriptor = {
        JsonErrorResponseStatus::kRejected,
        "joint_limit_violation",
        "Cartesian solution exceeds configured joint limits",
        ErrorDetailKind::kAxes,
        kJsonCartesianMotionAxisCount,
      };
      return true;
    case JsonMainMoveCartesianOutcome::kPositionNotRepresentable:
      descriptor = {
        JsonErrorResponseStatus::kRejected,
        "position_not_representable",
        "Cartesian solution cannot be represented by calibration",
        ErrorDetailKind::kAxes,
        kJsonCartesianMotionAxisCount,
      };
      return true;
    case JsonMainMoveCartesianOutcome::kEmergencyStop:
      descriptor = {
        JsonErrorResponseStatus::kCancelled,
        "emergency_stop",
        "emergency stop interrupted Cartesian motion",
        ErrorDetailKind::kPosition,
        0,
      };
      return true;
    case JsonMainMoveCartesianOutcome::kPositionUnavailable:
      descriptor = {
        JsonErrorResponseStatus::kFailed,
        "position_unavailable",
        "controller position is unavailable",
        ErrorDetailKind::kNone,
        0,
      };
      return true;
    case JsonMainMoveCartesianOutcome::kMotionExecutionFailed:
      descriptor = {
        JsonErrorResponseStatus::kFailed,
        "motion_execution_failed",
        "Cartesian motion could not complete",
        ErrorDetailKind::kPosition,
        0,
      };
      return true;
    case JsonMainMoveCartesianOutcome::kEncoderCollision:
      descriptor = {
        JsonErrorResponseStatus::kFailed,
        "encoder_collision",
        "encoder collision detected during Cartesian motion",
        ErrorDetailKind::kAxesAndPosition,
        kJsonCartesianMotionPrimaryAxisCount,
      };
      return true;
    case JsonMainMoveCartesianOutcome::kEncoderStateUnavailable:
      descriptor = {
        JsonErrorResponseStatus::kFailed,
        "encoder_state_unavailable",
        "encoder state is unavailable",
        ErrorDetailKind::kAxesAndPosition,
        kJsonCartesianMotionPrimaryAxisCount,
      };
      return true;
    case JsonMainMoveCartesianOutcome::kInvalid:
    case JsonMainMoveCartesianOutcome::kCompleted:
      return false;
  }
  return false;
}

inline const char *error_message_for_command(
  const char *command,
  const JsonMainMoveCartesianExecutionResult &result,
  const char *cartesian_message
) {
  if (
    command == nullptr
    || cartesian_message == nullptr
    || strcmp(command, "jog_tool") != 0
  ) {
    return cartesian_message;
  }
  switch (result.outcome) {
    case JsonMainMoveCartesianOutcome::kKinematicsUnreachable:
      return "tool-frame target has no selected inverse solution";
    case JsonMainMoveCartesianOutcome::kJointLimitViolation:
      return "tool-frame solution exceeds configured joint limits";
    case JsonMainMoveCartesianOutcome::kPositionNotRepresentable:
      if (json_joint_motion_detail::axes_empty(
          result.axes,
          kJsonCartesianMotionAxisCount
      )) {
        return "tool-frame offset cannot be represented by controller arithmetic";
      }
      return "tool-frame solution cannot be represented by calibration";
    case JsonMainMoveCartesianOutcome::kEmergencyStop:
      return "emergency stop interrupted tool-frame jog";
    case JsonMainMoveCartesianOutcome::kMotionExecutionFailed:
      return "tool-frame jog could not complete";
    case JsonMainMoveCartesianOutcome::kEncoderCollision:
      return "encoder collision detected during tool-frame jog";
    case JsonMainMoveCartesianOutcome::kPositionUnavailable:
    case JsonMainMoveCartesianOutcome::kEncoderStateUnavailable:
      return cartesian_message;
    case JsonMainMoveCartesianOutcome::kInvalid:
    case JsonMainMoveCartesianOutcome::kCompleted:
      return nullptr;
  }
  return nullptr;
}

inline bool append_error_details(
  const JsonMainMoveCartesianExecutionResult &result,
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
      || !json_joint_motion_detail::append_position_snapshot(
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

}  // namespace json_cartesian_motion_detail

inline bool json_main_move_cartesian_execution_result_valid(
  const JsonMainMoveCartesianExecutionResult &result
) {
  return json_cartesian_motion_detail::execution_result_valid(result);
}

inline bool extract_main_move_cartesian_parameters(
  ArduinoJson::JsonObjectConst params,
  JsonMainMoveCartesianParameters &output
) {
  if (params.size() != 11) return false;
  JsonMainMoveCartesianParameters staged = {};
  bool translation_present = false;
  bool orientation_present = false;
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
        "translation_millimeters"
    )) {
      if (
        translation_present
        || !json_joint_motion_detail::extract_controller_float_array(
          pair.value(),
          staged.translation_millimeters,
          kJsonCartesianMotionTranslationCount
        )
      ) return false;
      translation_present = true;
    } else if (json_joint_motion_detail::string_equals(
        key,
        "orientation_degrees"
    )) {
      if (
        orientation_present
        || !json_joint_motion_detail::extract_controller_float_array(
          pair.value(),
          staged.orientation_degrees,
          kJsonCartesianMotionOrientationCount
        )
      ) return false;
      orientation_present = true;
    } else if (json_joint_motion_detail::string_equals(
        key,
        "external_axes_units"
    )) {
      if (
        external_axes_present
        || !json_joint_motion_detail::extract_controller_float_array(
          pair.value(),
          staged.external_axes_units,
          kJsonCartesianMotionExternalAxisCount
        )
      ) return false;
      external_axes_present = true;
    } else if (json_joint_motion_detail::string_equals(key, "speed_mode")) {
      if (speed_mode_present) return false;
      const ArduinoJson::JsonString value =
        pair.value().as<ArduinoJson::JsonString>();
      if (json_joint_motion_detail::string_equals(value, "percent")) {
        staged.speed_mode = JsonCartesianMotionSpeedMode::kPercent;
      } else if (json_joint_motion_detail::string_equals(value, "seconds")) {
        staged.speed_mode = JsonCartesianMotionSpeedMode::kSeconds;
      } else if (json_joint_motion_detail::string_equals(
          value,
          "millimeters_per_second"
      )) {
        staged.speed_mode =
          JsonCartesianMotionSpeedMode::kMillimetersPerSecond;
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
      if (json_joint_motion_detail::string_equals(value, "automatic")) {
        staged.wrist_configuration =
          JsonCartesianMotionWristConfiguration::kAutomatic;
      } else if (json_joint_motion_detail::string_equals(value, "near")) {
        staged.wrist_configuration =
          JsonCartesianMotionWristConfiguration::kNear;
      } else if (json_joint_motion_detail::string_equals(value, "far")) {
        staged.wrist_configuration =
          JsonCartesianMotionWristConfiguration::kFar;
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
          kJsonCartesianMotionPrimaryAxisCount
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
    !translation_present
    || !orientation_present
    || !external_axes_present
    || !speed_mode_present
    || !speed_value_present
    || !acceleration_present
    || !deceleration_present
    || !ramp_present
    || !wrist_present
    || !loop_modes_present
    || !telemetry_present
    || !json_cartesian_motion_detail::parameters_valid(staged)
  ) {
    return false;
  }
  output = staged;
  return true;
}

inline bool build_main_json_cartesian_motion_response(
  const char *command,
  uint32_t request_id,
  const JsonMainMoveCartesianExecutionResult &result,
  size_t maximum_payload_bytes,
  char *output,
  size_t output_capacity
) {
  if (output == nullptr || output_capacity == 0) return false;
  output[0] = '\0';
  if (
    command == nullptr
    || (
      strcmp(command, "move_cartesian") != 0
      && strcmp(command, "jog_tool") != 0
      && strcmp(command, "move_linear") != 0
      && strcmp(command, "move_arc") != 0
      && strcmp(command, "move_circle") != 0
      && strcmp(command, "move_spline") != 0
      && strcmp(command, "move_vision") != 0
      && strcmp(command, "write_gcode_move") != 0
      && strcmp(command, "play_gcode_file") != 0
    )
    || request_id == 0
    || maximum_payload_bytes == 0
    || maximum_payload_bytes > kJsonProtocolMaximumPayloadBytes
    || !json_cartesian_motion_detail::execution_result_valid_for_command(
      command,
      result
    )
  ) {
    return false;
  }
  const size_t bounded_capacity = output_capacity < maximum_payload_bytes + 1
    ? output_capacity
    : maximum_payload_bytes + 1;
  size_t index = 0;
  bool built = false;
  if (result.outcome == JsonMainMoveCartesianOutcome::kCompleted) {
    built = append_json_text(
        "{\"cmd\":\"",
        output,
        bounded_capacity,
        index
      )
      && append_json_text(command, output, bounded_capacity, index)
      && append_json_text("\",\"id\":", output, bounded_capacity, index)
      && append_json_uint32(request_id, output, bounded_capacity, index)
      && append_json_text(
        ",\"result\":{\"controller_debug\":\"",
        output,
        bounded_capacity,
        index
      )
      && append_json_escaped_text(
        result.controller_debug,
        kJsonCartesianMotionControllerDebugMaximumLength,
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
    json_cartesian_motion_detail::ErrorDescriptor descriptor = {};
    if (!json_cartesian_motion_detail::error_descriptor(
        result.outcome,
        descriptor
    )) {
      return false;
    }
    const char *status_name = json_error_response_status_name(
      descriptor.status
    );
    const char *message =
      json_cartesian_motion_detail::error_message_for_command(
        command,
        result,
        descriptor.message
      );
    built = status_name != nullptr
      && message != nullptr
      && append_json_text(
        "{\"cmd\":\"",
        output,
        bounded_capacity,
        index
      )
      && append_json_text(command, output, bounded_capacity, index)
      && append_json_text(
        "\",\"error\":{\"code\":\"",
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
      && json_cartesian_motion_detail::append_error_details(
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
        message,
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

inline bool build_main_json_move_cartesian_response(
  uint32_t request_id,
  const JsonMainMoveCartesianExecutionResult &result,
  size_t maximum_payload_bytes,
  char *output,
  size_t output_capacity
) {
  return build_main_json_cartesian_motion_response(
    "move_cartesian",
    request_id,
    result,
    maximum_payload_bytes,
    output,
    output_capacity
  );
}

}  // namespace ar4_protocol

#endif
