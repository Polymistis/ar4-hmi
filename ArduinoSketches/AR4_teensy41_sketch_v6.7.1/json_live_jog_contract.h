#ifndef AR4_JSON_LIVE_JOG_CONTRACT_H
#define AR4_JSON_LIVE_JOG_CONTRACT_H

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "json_live_motion_runtime_contract.h"
#include "json_tool_jog_contract.h"

namespace ar4_protocol {

constexpr size_t kJsonLiveJogPrimaryAxisCount = 6;
constexpr size_t kJsonLiveJogControllerAxisCount = 9;
constexpr size_t kJsonLiveJogTerminalPayloadReservationBytes =
  kJsonCartesianMotionTerminalPayloadReservationBytes;

enum class JsonLiveJogKind : uint8_t {
  kInvalid,
  kJoint,
  kCartesian,
  kTool,
};

struct JsonMainLiveJogParameters {
  JsonLiveJogKind kind;
  uint8_t axis_index;
  JsonToolJogDirection direction;
  JsonJointMotionSpeedMode speed_mode;
  float speed_value;
  float acceleration_percent;
  float deceleration_percent;
  float ramp_percent;
  JsonJointMotionWristConfiguration wrist_configuration;
  bool loop_modes[kJsonLiveJogPrimaryAxisCount];
  bool telemetry_enabled;
  uint32_t lease_milliseconds;
};

struct JsonMainLiveMotionControlParameters {
  uint32_t motion_id;
};

using JsonMainStopParameters = JsonMainLiveMotionControlParameters;
using JsonMainRenewLiveMotionParameters = JsonMainLiveMotionControlParameters;

enum class JsonMainLiveJogOutcome : uint8_t {
  kInvalid,
  kCompleted,
  kJointLimitReached,
  kPositionNotRepresentable,
  kKinematicsUnreachable,
  kEmergencyStop,
  kPositionUnavailable,
  kMotionExecutionFailed,
  kEncoderCollision,
  kEncoderStateUnavailable,
  kControlLeaseExpired,
};

struct JsonMainLiveJogExecutionResult {
  JsonMainLiveJogOutcome outcome;
  JsonMainPositionSnapshot position;
  bool axes[kJsonLiveJogControllerAxisCount];
  bool speed_limited;
  char controller_debug[kJsonJointMotionControllerDebugCapacity];
};

namespace json_live_jog_detail {

inline bool kind_valid(JsonLiveJogKind kind) {
  return kind == JsonLiveJogKind::kJoint
    || kind == JsonLiveJogKind::kCartesian
    || kind == JsonLiveJogKind::kTool;
}

inline const char *command_name(JsonLiveJogKind kind) {
  switch (kind) {
    case JsonLiveJogKind::kJoint:
      return "live_joint_jog";
    case JsonLiveJogKind::kCartesian:
      return "live_cart_jog";
    case JsonLiveJogKind::kTool:
      return "live_tool_jog";
    case JsonLiveJogKind::kInvalid:
      return nullptr;
  }
  return nullptr;
}

inline JsonLiveJogKind kind_from_command(const char *command) {
  if (command == nullptr) return JsonLiveJogKind::kInvalid;
  if (strcmp(command, "live_joint_jog") == 0) {
    return JsonLiveJogKind::kJoint;
  }
  if (strcmp(command, "live_cart_jog") == 0) {
    return JsonLiveJogKind::kCartesian;
  }
  if (strcmp(command, "live_tool_jog") == 0) {
    return JsonLiveJogKind::kTool;
  }
  return JsonLiveJogKind::kInvalid;
}

inline bool parameters_valid(const JsonMainLiveJogParameters &params) {
  if (
    !kind_valid(params.kind)
    || params.axis_index >= (
      params.kind == JsonLiveJogKind::kJoint
        ? kJsonLiveJogControllerAxisCount
        : kJsonLiveJogPrimaryAxisCount
    )
    || !json_tool_jog_detail::direction_valid(params.direction)
    || params.speed_mode != JsonJointMotionSpeedMode::kPercent
    || !json_live_motion_lease_valid(params.lease_milliseconds)
  ) {
    return false;
  }
  JsonMainMoveJointsParameters profile = {};
  profile.speed_mode = params.speed_mode;
  profile.speed_value = params.speed_value;
  profile.acceleration_percent = params.acceleration_percent;
  profile.deceleration_percent = params.deceleration_percent;
  profile.ramp_percent = params.ramp_percent;
  profile.wrist_configuration = params.wrist_configuration;
  return json_joint_motion_detail::move_joints_parameters_valid(profile);
}

inline bool parameters_empty(const JsonMainLiveJogParameters &params) {
  if (
    params.kind != JsonLiveJogKind::kInvalid
    || params.axis_index != 0
    || params.direction != JsonToolJogDirection::kNegative
    || params.speed_mode != JsonJointMotionSpeedMode::kPercent
    || params.speed_value != 0.0f
    || params.acceleration_percent != 0.0f
    || params.deceleration_percent != 0.0f
    || params.ramp_percent != 0.0f
    || params.wrist_configuration
      != JsonJointMotionWristConfiguration::kNear
    || params.telemetry_enabled
    || params.lease_milliseconds != 0
  ) {
    return false;
  }
  for (size_t axis = 0; axis < kJsonLiveJogPrimaryAxisCount; ++axis) {
    if (params.loop_modes[axis]) return false;
  }
  return true;
}

inline bool extract_axis(
  ArduinoJson::JsonVariantConst value,
  JsonLiveJogKind kind,
  uint8_t &axis_index
) {
  if (kind == JsonLiveJogKind::kJoint) {
    if (value.is<bool>() || !value.is<uint32_t>()) return false;
    const uint32_t axis = value.as<uint32_t>();
    if (axis < 1 || axis > kJsonLiveJogControllerAxisCount) return false;
    axis_index = static_cast<uint8_t>(axis - 1);
    return true;
  }
  if (
    kind != JsonLiveJogKind::kCartesian
    && kind != JsonLiveJogKind::kTool
  ) {
    return false;
  }
  const ArduinoJson::JsonString axis = value.as<ArduinoJson::JsonString>();
  const char *const names[kJsonLiveJogPrimaryAxisCount] = {
    "x", "y", "z", "rx", "ry", "rz",
  };
  for (uint8_t index = 0; index < kJsonLiveJogPrimaryAxisCount; ++index) {
    if (json_joint_motion_detail::string_equals(axis, names[index])) {
      axis_index = index;
      return true;
    }
  }
  return false;
}

inline bool extract_parameters(
  ArduinoJson::JsonObjectConst params,
  JsonLiveJogKind kind,
  JsonMainLiveJogParameters &output
) {
  if (!kind_valid(kind) || params.size() != 11) return false;
  JsonMainLiveJogParameters staged = {};
  staged.kind = kind;
  bool axis_present = false;
  bool direction_present = false;
  bool speed_mode_present = false;
  bool speed_value_present = false;
  bool acceleration_present = false;
  bool deceleration_present = false;
  bool ramp_present = false;
  bool wrist_present = false;
  bool loop_modes_present = false;
  bool telemetry_present = false;
  bool lease_present = false;
  for (ArduinoJson::JsonPairConst pair : params) {
    const ArduinoJson::JsonString key = pair.key();
    if (json_joint_motion_detail::string_equals(key, "axis")) {
      if (
        axis_present
        || !extract_axis(pair.value(), kind, staged.axis_index)
      ) return false;
      axis_present = true;
    } else if (json_joint_motion_detail::string_equals(key, "direction")) {
      if (direction_present) return false;
      const ArduinoJson::JsonString value =
        pair.value().as<ArduinoJson::JsonString>();
      if (json_joint_motion_detail::string_equals(value, "negative")) {
        staged.direction = JsonToolJogDirection::kNegative;
      } else if (
        json_joint_motion_detail::string_equals(value, "positive")
      ) {
        staged.direction = JsonToolJogDirection::kPositive;
      } else {
        return false;
      }
      direction_present = true;
    } else if (json_joint_motion_detail::string_equals(key, "speed_mode")) {
      if (speed_mode_present) return false;
      const ArduinoJson::JsonString value =
        pair.value().as<ArduinoJson::JsonString>();
      if (!json_joint_motion_detail::string_equals(value, "percent")) {
        return false;
      }
      staged.speed_mode = JsonJointMotionSpeedMode::kPercent;
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
      } else if (
        json_joint_motion_detail::string_equals(value, "automatic")
      ) {
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
          kJsonLiveJogPrimaryAxisCount
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
    } else if (json_joint_motion_detail::string_equals(
        key,
        "lease_milliseconds"
    )) {
      if (
        lease_present
        || pair.value().is<bool>()
        || !pair.value().is<uint32_t>()
      ) return false;
      staged.lease_milliseconds = pair.value().as<uint32_t>();
      if (!json_live_motion_lease_valid(staged.lease_milliseconds)) {
        return false;
      }
      lease_present = true;
    } else {
      return false;
    }
  }
  if (
    !axis_present
    || !direction_present
    || !speed_mode_present
    || !speed_value_present
    || !acceleration_present
    || !deceleration_present
    || !ramp_present
    || !wrist_present
    || !loop_modes_present
    || !telemetry_present
    || !lease_present
    || !parameters_valid(staged)
  ) {
    return false;
  }
  output = staged;
  return true;
}

inline bool extract_live_motion_control_parameters(
  ArduinoJson::JsonObjectConst params,
  JsonMainLiveMotionControlParameters &output
) {
  if (params.size() != 1) return false;
  JsonMainLiveMotionControlParameters staged = {};
  bool motion_id_present = false;
  for (ArduinoJson::JsonPairConst pair : params) {
    if (
      motion_id_present
      || !json_joint_motion_detail::string_equals(pair.key(), "motion_id")
      || pair.value().is<bool>()
      || !pair.value().is<uint32_t>()
    ) {
      return false;
    }
    staged.motion_id = pair.value().as<uint32_t>();
    if (staged.motion_id == 0) return false;
    motion_id_present = true;
  }
  if (!motion_id_present) return false;
  output = staged;
  return true;
}

inline bool append_response_prefix(
  const char *command,
  uint32_t request_id,
  char *output,
  size_t output_capacity,
  size_t &index
) {
  return append_json_text("{\"cmd\":\"", output, output_capacity, index)
    && append_json_text(command, output, output_capacity, index)
    && append_json_text("\",\"id\":", output, output_capacity, index)
    && append_json_uint32(request_id, output, output_capacity, index);
}

inline bool response_builder_input_valid(
  uint32_t request_id,
  const char *command,
  size_t maximum_payload_bytes,
  char *output,
  size_t output_capacity
) {
  return request_id != 0
    && json_protocol_name_valid(command)
    && maximum_payload_bytes > 0
    && maximum_payload_bytes <= kJsonProtocolMaximumPayloadBytes
    && output != nullptr
    && output_capacity > 0;
}

enum class TerminalDetailKind : uint8_t {
  kNone,
  kPosition,
  kAxesAndPosition,
};

struct TerminalDescriptor {
  JsonErrorResponseStatus status;
  const char *code;
  const char *message;
  TerminalDetailKind details;
  size_t axis_count;
};

inline bool terminal_descriptor(
  JsonLiveJogKind kind,
  JsonMainLiveJogOutcome outcome,
  TerminalDescriptor &descriptor
) {
  if (!kind_valid(kind)) return false;
  switch (outcome) {
    case JsonMainLiveJogOutcome::kJointLimitReached:
      descriptor = {
        JsonErrorResponseStatus::kFailed,
        "joint_limit_reached",
        "live jog reached a configured joint limit",
        TerminalDetailKind::kAxesAndPosition,
        kJsonLiveJogControllerAxisCount,
      };
      return true;
    case JsonMainLiveJogOutcome::kPositionNotRepresentable:
      descriptor = {
        JsonErrorResponseStatus::kFailed,
        "position_not_representable",
        "live jog target cannot be represented by calibration",
        TerminalDetailKind::kAxesAndPosition,
        kJsonLiveJogControllerAxisCount,
      };
      return true;
    case JsonMainLiveJogOutcome::kKinematicsUnreachable:
      descriptor = {
        JsonErrorResponseStatus::kFailed,
        "kinematics_unreachable",
        "live jog target has no selected inverse solution",
        TerminalDetailKind::kPosition,
        0,
      };
      return true;
    case JsonMainLiveJogOutcome::kEmergencyStop:
      descriptor = {
        JsonErrorResponseStatus::kCancelled,
        "emergency_stop",
        "emergency stop interrupted live jog",
        TerminalDetailKind::kPosition,
        0,
      };
      return true;
    case JsonMainLiveJogOutcome::kPositionUnavailable:
      descriptor = {
        JsonErrorResponseStatus::kFailed,
        "position_unavailable",
        "controller position is unavailable",
        TerminalDetailKind::kNone,
        0,
      };
      return true;
    case JsonMainLiveJogOutcome::kMotionExecutionFailed:
      descriptor = {
        JsonErrorResponseStatus::kFailed,
        "motion_execution_failed",
        "live jog could not complete",
        TerminalDetailKind::kPosition,
        0,
      };
      return true;
    case JsonMainLiveJogOutcome::kEncoderCollision:
      descriptor = {
        JsonErrorResponseStatus::kFailed,
        "encoder_collision",
        "encoder collision detected during live jog",
        TerminalDetailKind::kAxesAndPosition,
        kJsonLiveJogPrimaryAxisCount,
      };
      return true;
    case JsonMainLiveJogOutcome::kEncoderStateUnavailable:
      descriptor = {
        JsonErrorResponseStatus::kFailed,
        "encoder_state_unavailable",
        "encoder state is unavailable",
        TerminalDetailKind::kAxesAndPosition,
        kJsonLiveJogPrimaryAxisCount,
      };
      return true;
    case JsonMainLiveJogOutcome::kControlLeaseExpired:
      descriptor = {
        JsonErrorResponseStatus::kFailed,
        "control_lease_expired",
        "live-motion control lease expired",
        TerminalDetailKind::kPosition,
        0,
      };
      return true;
    case JsonMainLiveJogOutcome::kInvalid:
    case JsonMainLiveJogOutcome::kCompleted:
      return false;
  }
  return false;
}

inline bool execution_result_valid(
  JsonLiveJogKind kind,
  const JsonMainLiveJogExecutionResult &result
) {
  if (!kind_valid(kind)) return false;
  const bool all_axes_empty = json_joint_motion_detail::axes_empty(
    result.axes,
    kJsonLiveJogControllerAxisCount
  );
  const bool primary_axis_set = json_joint_motion_detail::any_axis_set(
    result.axes,
    kJsonLiveJogPrimaryAxisCount
  );
  const bool external_axes_empty = json_joint_motion_detail::axes_empty(
    result.axes + kJsonLiveJogPrimaryAxisCount,
    kJsonLiveJogControllerAxisCount - kJsonLiveJogPrimaryAxisCount
  );
  const bool inactive_fields_empty = !result.speed_limited
    && result.controller_debug[0] == '\0';
  switch (result.outcome) {
    case JsonMainLiveJogOutcome::kCompleted:
      return all_axes_empty && json_protocol_text_valid(
        result.controller_debug,
        kJsonJointMotionControllerDebugMaximumLength,
        true
      );
    case JsonMainLiveJogOutcome::kJointLimitReached:
      return inactive_fields_empty
        && json_joint_motion_detail::any_axis_set(
          result.axes,
          kJsonLiveJogControllerAxisCount
        );
    case JsonMainLiveJogOutcome::kPositionNotRepresentable:
      return inactive_fields_empty
        && (
          kind == JsonLiveJogKind::kTool
          || json_joint_motion_detail::any_axis_set(
            result.axes,
            kJsonLiveJogControllerAxisCount
          )
        );
    case JsonMainLiveJogOutcome::kKinematicsUnreachable:
      return kind != JsonLiveJogKind::kJoint
        && inactive_fields_empty
        && all_axes_empty;
    case JsonMainLiveJogOutcome::kEmergencyStop:
    case JsonMainLiveJogOutcome::kMotionExecutionFailed:
    case JsonMainLiveJogOutcome::kControlLeaseExpired:
      return inactive_fields_empty && all_axes_empty;
    case JsonMainLiveJogOutcome::kPositionUnavailable:
      return inactive_fields_empty
        && all_axes_empty
        && json_joint_motion_detail::position_snapshot_empty(
          result.position
        );
    case JsonMainLiveJogOutcome::kEncoderCollision:
    case JsonMainLiveJogOutcome::kEncoderStateUnavailable:
      return inactive_fields_empty
        && primary_axis_set
        && external_axes_empty;
    case JsonMainLiveJogOutcome::kInvalid:
      return false;
  }
  return false;
}

inline bool append_axes(
  const bool *axes,
  size_t axis_count,
  char *output,
  size_t output_capacity,
  size_t &index
) {
  if (
    axes == nullptr
    || axis_count == 0
    || !append_json_text("[", output, output_capacity, index)
  ) return false;
  for (size_t axis = 0; axis < axis_count; ++axis) {
    if (
      (axis != 0
        && !append_json_text(",", output, output_capacity, index))
      || !append_json_text(
        axes[axis] ? "true" : "false",
        output,
        output_capacity,
        index
      )
    ) return false;
  }
  return append_json_text("]", output, output_capacity, index);
}

inline bool append_terminal_details(
  const JsonMainLiveJogExecutionResult &result,
  const TerminalDescriptor &descriptor,
  char *output,
  size_t output_capacity,
  size_t &index
) {
  if (!append_json_text("{", output, output_capacity, index)) return false;
  bool field_present = false;
  if (descriptor.details == TerminalDetailKind::kAxesAndPosition) {
    if (
      !append_json_text("\"axes\":", output, output_capacity, index)
      || !append_axes(
        result.axes,
        descriptor.axis_count,
        output,
        output_capacity,
        index
      )
    ) return false;
    field_present = true;
  }
  if (
    descriptor.details == TerminalDetailKind::kPosition
    || descriptor.details == TerminalDetailKind::kAxesAndPosition
  ) {
    if (
      (field_present
        && !append_json_text(",", output, output_capacity, index))
      || !append_json_text(
        "\"position\":",
        output,
        output_capacity,
        index
      )
      || !json_joint_motion_detail::append_position_snapshot(
        result.position,
        output,
        output_capacity,
        index
      )
    ) return false;
  }
  return append_json_text("}", output, output_capacity, index);
}

}  // namespace json_live_jog_detail

inline bool extract_main_live_jog_parameters(
  ArduinoJson::JsonObjectConst params,
  JsonLiveJogKind kind,
  JsonMainLiveJogParameters &output
) {
  return json_live_jog_detail::extract_parameters(params, kind, output);
}

inline bool extract_main_stop_parameters(
  ArduinoJson::JsonObjectConst params,
  JsonMainStopParameters &output
) {
  return json_live_jog_detail::extract_live_motion_control_parameters(
    params,
    output
  );
}

inline bool extract_main_renew_live_motion_parameters(
  ArduinoJson::JsonObjectConst params,
  JsonMainRenewLiveMotionParameters &output
) {
  return json_live_jog_detail::extract_live_motion_control_parameters(
    params,
    output
  );
}

inline bool build_main_json_accepted_response(
  uint32_t request_id,
  const char *command,
  size_t maximum_payload_bytes,
  char *output,
  size_t output_capacity
) {
  if (output != nullptr && output_capacity > 0) output[0] = '\0';
  if (!json_live_jog_detail::response_builder_input_valid(
      request_id,
      command,
      maximum_payload_bytes,
      output,
      output_capacity
  )) return false;
  const size_t bounded_capacity = output_capacity < maximum_payload_bytes + 1
    ? output_capacity
    : maximum_payload_bytes + 1;
  size_t index = 0;
  const bool built = json_live_jog_detail::append_response_prefix(
      command,
      request_id,
      output,
      bounded_capacity,
      index
    )
    && append_json_text(
      ",\"result\":{},\"status\":\"accepted\",\"type\":\"response\",\"v\":1}",
      output,
      bounded_capacity,
      index
    );
  if (!built) output[0] = '\0';
  return built;
}

inline bool build_main_json_stop_completed_response(
  uint32_t request_id,
  uint32_t motion_id,
  size_t maximum_payload_bytes,
  char *output,
  size_t output_capacity
) {
  if (output != nullptr && output_capacity > 0) output[0] = '\0';
  if (
    motion_id == 0
    || !json_live_jog_detail::response_builder_input_valid(
      request_id,
      "stop",
      maximum_payload_bytes,
      output,
      output_capacity
    )
  ) return false;
  const size_t bounded_capacity = output_capacity < maximum_payload_bytes + 1
    ? output_capacity
    : maximum_payload_bytes + 1;
  size_t index = 0;
  const bool built = json_live_jog_detail::append_response_prefix(
      "stop",
      request_id,
      output,
      bounded_capacity,
      index
    )
    && append_json_text(",\"result\":{\"motion_id\":", output,
      bounded_capacity, index)
    && append_json_uint32(motion_id, output, bounded_capacity, index)
    && append_json_text(
      "},\"status\":\"completed\",\"type\":\"response\",\"v\":1}",
      output,
      bounded_capacity,
      index
    );
  if (!built) output[0] = '\0';
  return built;
}

inline bool build_main_json_renew_live_motion_completed_response(
  uint32_t request_id,
  uint32_t motion_id,
  size_t maximum_payload_bytes,
  char *output,
  size_t output_capacity
) {
  if (output != nullptr && output_capacity > 0) output[0] = '\0';
  if (
    motion_id == 0
    || !json_live_jog_detail::response_builder_input_valid(
      request_id,
      "renew_live_motion",
      maximum_payload_bytes,
      output,
      output_capacity
    )
  ) return false;
  const size_t bounded_capacity = output_capacity < maximum_payload_bytes + 1
    ? output_capacity
    : maximum_payload_bytes + 1;
  size_t index = 0;
  const bool built = json_live_jog_detail::append_response_prefix(
      "renew_live_motion",
      request_id,
      output,
      bounded_capacity,
      index
    )
    && append_json_text(",\"result\":{\"motion_id\":", output,
      bounded_capacity, index)
    && append_json_uint32(motion_id, output, bounded_capacity, index)
    && append_json_text(
      "},\"status\":\"completed\",\"type\":\"response\",\"v\":1}",
      output,
      bounded_capacity,
      index
    );
  if (!built) output[0] = '\0';
  return built;
}

inline bool build_main_json_live_motion_control_mismatch_response(
  const char *command,
  uint32_t request_id,
  uint32_t active_motion_id,
  size_t maximum_payload_bytes,
  char *output,
  size_t output_capacity
) {
  if (output != nullptr && output_capacity > 0) output[0] = '\0';
  if (
    active_motion_id == 0
    || (
      strcmp(command, "stop") != 0
      && strcmp(command, "renew_live_motion") != 0
    )
    || !json_live_jog_detail::response_builder_input_valid(
      request_id,
      command,
      maximum_payload_bytes,
      output,
      output_capacity
    )
  ) return false;
  const size_t bounded_capacity = output_capacity < maximum_payload_bytes + 1
    ? output_capacity
    : maximum_payload_bytes + 1;
  size_t index = 0;
  const bool built = append_json_text("{\"cmd\":\"", output,
      bounded_capacity, index)
    && append_json_text(command, output, bounded_capacity, index)
    && append_json_text(
      "\",\"error\":{\"code\":\"live_motion_mismatch\","
      "\"details\":{\"active_motion_id\":",
      output,
      bounded_capacity,
      index
    )
    && append_json_uint32(
      active_motion_id,
      output,
      bounded_capacity,
      index
    )
    && append_json_text(
      "},\"message\":\"control target does not own the active live motion\"},"
      "\"id\":",
      output,
      bounded_capacity,
      index
    )
    && append_json_uint32(request_id, output, bounded_capacity, index)
    && append_json_text(
      ",\"status\":\"rejected\",\"type\":\"response\",\"v\":1}",
      output,
      bounded_capacity,
      index
    );
  if (!built) output[0] = '\0';
  return built;
}

inline bool build_main_json_stop_mismatch_response(
  uint32_t request_id,
  uint32_t active_motion_id,
  size_t maximum_payload_bytes,
  char *output,
  size_t output_capacity
) {
  return build_main_json_live_motion_control_mismatch_response(
    "stop",
    request_id,
    active_motion_id,
    maximum_payload_bytes,
    output,
    output_capacity
  );
}

inline bool build_main_json_live_jog_response(
  JsonLiveJogKind kind,
  uint32_t request_id,
  const JsonMainLiveJogExecutionResult &result,
  size_t maximum_payload_bytes,
  char *output,
  size_t output_capacity
) {
  const char *command = json_live_jog_detail::command_name(kind);
  if (output != nullptr && output_capacity > 0) output[0] = '\0';
  if (
    command == nullptr
    || !json_live_jog_detail::response_builder_input_valid(
      request_id,
      command,
      maximum_payload_bytes,
      output,
      output_capacity
    )
    || !json_live_jog_detail::execution_result_valid(kind, result)
  ) return false;
  const size_t bounded_capacity = output_capacity < maximum_payload_bytes + 1
    ? output_capacity
    : maximum_payload_bytes + 1;
  size_t index = 0;
  bool built = false;
  if (result.outcome == JsonMainLiveJogOutcome::kCompleted) {
    built = json_live_jog_detail::append_response_prefix(
        command,
        request_id,
        output,
        bounded_capacity,
        index
      )
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
    json_live_jog_detail::TerminalDescriptor descriptor = {};
    const char *status_name = nullptr;
    built = json_live_jog_detail::terminal_descriptor(
        kind,
        result.outcome,
        descriptor
      )
      && (status_name = json_error_response_status_name(
        descriptor.status
      )) != nullptr
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
      && json_live_jog_detail::append_terminal_details(
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
      && append_json_text(
        "\"},\"id\":",
        output,
        bounded_capacity,
        index
      )
      && append_json_uint32(
        request_id,
        output,
        bounded_capacity,
        index
      )
      && append_json_text(
        ",\"status\":\"",
        output,
        bounded_capacity,
        index
      )
      && append_json_text(
        status_name,
        output,
        bounded_capacity,
        index
      )
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

}  // namespace ar4_protocol

#endif
