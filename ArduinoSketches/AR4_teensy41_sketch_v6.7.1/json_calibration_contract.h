#ifndef AR4_JSON_CALIBRATION_CONTRACT_H
#define AR4_JSON_CALIBRATION_CONTRACT_H

#include <stddef.h>
#include <stdint.h>

#include "json_joint_motion_contract.h"

namespace ar4_protocol {

constexpr size_t kJsonCalibrationAxisCount = kJsonJointMotionAxisCount;
constexpr size_t kJsonCalibrationTerminalPayloadReservationBytes = 1024;

struct JsonMainCalibrationParameters {
  bool axes[kJsonCalibrationAxisCount];
  float offsets[kJsonCalibrationAxisCount];
};

enum class JsonMainCalibrationStage : uint8_t {
  kNone,
  kFastLimitSearch,
  kSwitchRelease,
  kSlowLimitSearch,
  kCenterMove,
};

enum class JsonMainCalibrationOutcome : uint8_t {
  kInvalid,
  kCompleted,
  kNotRepresentable,
  kEmergencyStop,
  kCalibrationFailed,
  kPositionUnavailable,
};

struct JsonMainCalibrationExecutionResult {
  JsonMainCalibrationOutcome outcome;
  JsonMainPositionSnapshot position;
  bool axes[kJsonCalibrationAxisCount];
  JsonMainCalibrationStage stage;
  bool speed_limited;
  char controller_debug[kJsonJointMotionControllerDebugCapacity];
};

namespace json_calibration_detail {

inline const char *stage_name(JsonMainCalibrationStage stage) {
  switch (stage) {
    case JsonMainCalibrationStage::kFastLimitSearch:
      return "fast_limit_search";
    case JsonMainCalibrationStage::kSwitchRelease:
      return "switch_release";
    case JsonMainCalibrationStage::kSlowLimitSearch:
      return "slow_limit_search";
    case JsonMainCalibrationStage::kCenterMove:
      return "center_move";
    case JsonMainCalibrationStage::kNone:
      return nullptr;
  }
  return nullptr;
}

inline bool parameters_valid(const JsonMainCalibrationParameters &parameters) {
  return json_joint_motion_detail::any_axis_set(
    parameters.axes,
    kJsonCalibrationAxisCount
  );
}

inline bool execution_result_valid(
  const JsonMainCalibrationExecutionResult &result
) {
  const bool axes_empty = json_joint_motion_detail::axes_empty(
    result.axes,
    kJsonCalibrationAxisCount
  );
  const bool inactive_fields_empty = !result.speed_limited
    && result.controller_debug[0] == '\0';
  switch (result.outcome) {
    case JsonMainCalibrationOutcome::kCompleted:
      return axes_empty
        && result.stage == JsonMainCalibrationStage::kNone
        && json_protocol_text_valid(
          result.controller_debug,
          kJsonJointMotionControllerDebugMaximumLength,
          true
        );
    case JsonMainCalibrationOutcome::kNotRepresentable:
      return json_joint_motion_detail::position_snapshot_empty(result.position)
        && json_joint_motion_detail::any_axis_set(
          result.axes,
          kJsonCalibrationAxisCount
        )
        && result.stage == JsonMainCalibrationStage::kNone
        && inactive_fields_empty;
    case JsonMainCalibrationOutcome::kEmergencyStop:
      return axes_empty
        && result.stage == JsonMainCalibrationStage::kNone
        && inactive_fields_empty;
    case JsonMainCalibrationOutcome::kCalibrationFailed:
      return axes_empty
        && stage_name(result.stage) != nullptr
        && inactive_fields_empty;
    case JsonMainCalibrationOutcome::kPositionUnavailable:
      return axes_empty
        && result.stage == JsonMainCalibrationStage::kNone
        && json_joint_motion_detail::position_snapshot_empty(result.position)
        && inactive_fields_empty;
    case JsonMainCalibrationOutcome::kInvalid:
      return false;
  }
  return false;
}

enum class ErrorDetailKind : uint8_t {
  kAxes,
  kPosition,
  kPositionAndStage,
  kNone,
};

struct ErrorDescriptor {
  JsonErrorResponseStatus status;
  const char *code;
  const char *message;
  ErrorDetailKind details;
};

inline bool error_descriptor(
  JsonMainCalibrationOutcome outcome,
  ErrorDescriptor &descriptor
) {
  switch (outcome) {
    case JsonMainCalibrationOutcome::kNotRepresentable:
      descriptor = {
        JsonErrorResponseStatus::kRejected,
        "calibration_not_representable",
        "calibration reference cannot be represented",
        ErrorDetailKind::kAxes,
      };
      return true;
    case JsonMainCalibrationOutcome::kEmergencyStop:
      descriptor = {
        JsonErrorResponseStatus::kCancelled,
        "emergency_stop",
        "emergency stop interrupted calibration",
        ErrorDetailKind::kPosition,
      };
      return true;
    case JsonMainCalibrationOutcome::kCalibrationFailed:
      descriptor = {
        JsonErrorResponseStatus::kFailed,
        "calibration_failed",
        "calibration could not complete",
        ErrorDetailKind::kPositionAndStage,
      };
      return true;
    case JsonMainCalibrationOutcome::kPositionUnavailable:
      descriptor = {
        JsonErrorResponseStatus::kFailed,
        "position_unavailable",
        "controller position is unavailable",
        ErrorDetailKind::kNone,
      };
      return true;
    case JsonMainCalibrationOutcome::kCompleted:
    case JsonMainCalibrationOutcome::kInvalid:
      return false;
  }
  return false;
}

inline bool append_error_details(
  const JsonMainCalibrationExecutionResult &result,
  ErrorDetailKind kind,
  char *output,
  size_t output_capacity,
  size_t &index
) {
  if (!append_json_text("{", output, output_capacity, index)) return false;
  if (kind == ErrorDetailKind::kAxes) {
    if (!append_json_text("\"axes\":", output, output_capacity, index)
        || !append_json_bool_array(
          result.axes,
          kJsonCalibrationAxisCount,
          output,
          output_capacity,
          index
        )) {
      return false;
    }
  } else if (
    kind == ErrorDetailKind::kPosition
    || kind == ErrorDetailKind::kPositionAndStage
  ) {
    if (!append_json_text("\"position\":", output, output_capacity, index)
        || !json_joint_motion_detail::append_position_snapshot(
          result.position,
          output,
          output_capacity,
          index
        )) {
      return false;
    }
    if (kind == ErrorDetailKind::kPositionAndStage) {
      const char *stage = stage_name(result.stage);
      if (stage == nullptr
          || !append_json_text(",\"stage\":\"", output, output_capacity, index)
          || !append_json_text(stage, output, output_capacity, index)
          || !append_json_text("\"", output, output_capacity, index)) {
        return false;
      }
    }
  }
  return append_json_text("}", output, output_capacity, index);
}

}  // namespace json_calibration_detail

inline bool extract_main_calibration_parameters(
  ArduinoJson::JsonObjectConst params,
  JsonMainCalibrationParameters &output
) {
  if (params.size() != 2) return false;
  JsonMainCalibrationParameters staged = {};
  bool axes_present = false;
  bool offsets_present = false;
  for (ArduinoJson::JsonPairConst pair : params) {
    const ArduinoJson::JsonString key = pair.key();
    if (json_joint_motion_detail::string_equals(key, "axes")) {
      if (axes_present
          || !json_joint_motion_detail::extract_bool_array(
            pair.value(),
            staged.axes,
            kJsonCalibrationAxisCount
          )) {
        return false;
      }
      axes_present = true;
    } else if (json_joint_motion_detail::string_equals(key, "offsets")) {
      if (offsets_present
          || !json_joint_motion_detail::extract_controller_float_array(
            pair.value(),
            staged.offsets,
            kJsonCalibrationAxisCount
          )) {
        return false;
      }
      offsets_present = true;
    } else {
      return false;
    }
  }
  if (
    !axes_present
    || !offsets_present
    || !json_calibration_detail::parameters_valid(staged)
  ) {
    return false;
  }
  output = staged;
  return true;
}

inline bool json_main_calibration_execution_result_valid(
  const JsonMainCalibrationExecutionResult &result
) {
  return json_calibration_detail::execution_result_valid(result);
}

inline bool build_main_json_calibration_response(
  uint32_t request_id,
  const JsonMainCalibrationExecutionResult &result,
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
    || !json_calibration_detail::execution_result_valid(result)
  ) {
    return false;
  }
  const size_t bounded_capacity = output_capacity < maximum_payload_bytes + 1
    ? output_capacity
    : maximum_payload_bytes + 1;
  size_t index = 0;
  bool built = false;
  if (result.outcome == JsonMainCalibrationOutcome::kCompleted) {
    built = append_json_text(
        "{\"cmd\":\"calibrate\",\"id\":",
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
    json_calibration_detail::ErrorDescriptor descriptor = {};
    if (!json_calibration_detail::error_descriptor(
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
        "{\"cmd\":\"calibrate\",\"error\":{\"code\":\"",
        output,
        bounded_capacity,
        index
      )
      && append_json_text(descriptor.code, output, bounded_capacity, index)
      && append_json_text(
        "\",\"details\":",
        output,
        bounded_capacity,
        index
      )
      && json_calibration_detail::append_error_details(
        result,
        descriptor.details,
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

}  // namespace ar4_protocol

#endif
