#ifndef AR4_JSON_DIRECT_COMMAND_CONTRACT_H
#define AR4_JSON_DIRECT_COMMAND_CONTRACT_H

#include <ArduinoJson.h>

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "controller_domain_contract.h"
#include "json_cartesian_motion_contract.h"

namespace ar4_protocol {

constexpr size_t kJsonMainSplineMaximumSegmentCount = 6;

enum class JsonMainDirectParameterKind : uint8_t {
  kEmpty,
  kLinearMotion,
  kArcMotion,
  kCircleMotion,
  kSplineMotion,
  kVisionMotion,
  kModbusWait,
  kStorageFile,
  kStorageMotion,
};

struct JsonMainSplineSegment {
  JsonMainMoveCartesianParameters motion;
  float rounding_millimeters;
};

struct JsonMainDirectParameters {
  JsonMainDirectParameterKind kind;
  JsonMainMoveCartesianParameters motion;
  float midpoint_translation_millimeters[3];
  float center_translation_millimeters[3];
  float plane_translation_millimeters[3];
  JsonMainSplineSegment spline_segments[
    kJsonMainSplineMaximumSegmentCount
  ];
  size_t spline_segment_count;
  float motion_modifier;
  bool disable_wrist_rotation;
  int slave_id;
  int address;
  int expected;
  uint32_t timeout_milliseconds;
  char media_id[kControllerMediaIdCapacity];
  char filename[kControllerFilenameMaxLength + 1];
};

enum class JsonMainDirectResponseStatus : uint8_t {
  kCompleted,
  kRejected,
  kCancelled,
  kFailed,
  kInvalid,
};

using JsonMainDirectExecute = JsonMainDirectResponseStatus (*)(
  const char *command,
  const JsonMainDirectParameters &parameters,
  uint32_t request_id,
  size_t maximum_payload_bytes,
  char *output,
  size_t output_capacity,
  void *context
);

struct JsonMainDirectCommandSource {
  JsonMainDirectExecute execute;
  void *context;
};

namespace json_direct_command_detail {

inline bool copy_media_id(
  ArduinoJson::JsonVariantConst value,
  char (&output)[kControllerMediaIdCapacity]
) {
  const ArduinoJson::JsonString text = value.as<ArduinoJson::JsonString>();
  if (text.isNull() || text.size() != kControllerMediaIdLength) return false;
  for (size_t index = 0; index < text.size(); ++index) {
    const unsigned char character =
      static_cast<unsigned char>(text.c_str()[index]);
    if (!uppercase_hex_character(character)) return false;
    output[index] = static_cast<char>(character);
  }
  output[text.size()] = '\0';
  return true;
}

inline bool copy_filename(
  ArduinoJson::JsonVariantConst value,
  char (&output)[kControllerFilenameMaxLength + 1]
) {
  const ArduinoJson::JsonString text = value.as<ArduinoJson::JsonString>();
  if (
    text.isNull()
    || text.size() == 0
    || text.size() > kControllerFilenameMaxLength
    || (text.size() == 1 && text.c_str()[0] == '.')
    || (text.size() == 2
      && text.c_str()[0] == '.' && text.c_str()[1] == '.')
  ) return false;
  for (size_t index = 0; index < text.size(); ++index) {
    const unsigned char character =
      static_cast<unsigned char>(text.c_str()[index]);
    if (
      character < 32
      || character > 126
      || fat_reserved_filename_character(character)
      || character == static_cast<unsigned char>(kControllerDirectorySeparator)
      || (character == ' ' && (index == 0 || index + 1 == text.size()))
    ) return false;
    output[index] = static_cast<char>(character);
  }
  output[text.size()] = '\0';
  return true;
}

inline bool extract_motion(
  ArduinoJson::JsonVariantConst value,
  JsonMainMoveCartesianParameters &output
) {
  return value.is<ArduinoJson::JsonObjectConst>()
    && extract_main_move_cartesian_parameters(
      value.as<ArduinoJson::JsonObjectConst>(),
      output
    );
}

inline bool extract_arc_motion(
  ArduinoJson::JsonObjectConst params,
  JsonMainDirectParameters &output
) {
  if (params.size() != 2) return false;
  JsonMainDirectParameters staged = {};
  if (
    !extract_motion(params["motion"], staged.motion)
    || staged.motion.telemetry_enabled
    || !json_joint_motion_detail::extract_controller_float_array(
      params["midpoint_translation_millimeters"],
      staged.midpoint_translation_millimeters,
      3
    )
  ) return false;
  staged.kind = JsonMainDirectParameterKind::kArcMotion;
  output = staged;
  return true;
}

inline bool extract_circle_motion(
  ArduinoJson::JsonObjectConst params,
  JsonMainDirectParameters &output
) {
  if (params.size() != 3) return false;
  JsonMainDirectParameters staged = {};
  if (
    !extract_motion(params["motion"], staged.motion)
    || staged.motion.telemetry_enabled
    || !json_joint_motion_detail::extract_controller_float_array(
      params["center_translation_millimeters"],
      staged.center_translation_millimeters,
      3
    )
    || !json_joint_motion_detail::extract_controller_float_array(
      params["plane_translation_millimeters"],
      staged.plane_translation_millimeters,
      3
    )
  ) return false;
  staged.kind = JsonMainDirectParameterKind::kCircleMotion;
  output = staged;
  return true;
}

inline bool extract_spline_motion(
  ArduinoJson::JsonObjectConst params,
  JsonMainDirectParameters &output
) {
  if (
    params.size() != 1
    || !params["segments"].is<ArduinoJson::JsonArrayConst>()
  ) return false;
  const ArduinoJson::JsonArrayConst segments =
    params["segments"].as<ArduinoJson::JsonArrayConst>();
  if (
    segments.size() == 0
    || segments.size() > kJsonMainSplineMaximumSegmentCount
  ) return false;
  JsonMainDirectParameters staged = {};
  staged.kind = JsonMainDirectParameterKind::kSplineMotion;
  staged.spline_segment_count = segments.size();
  for (size_t index = 0; index < segments.size(); ++index) {
    if (!segments[index].is<ArduinoJson::JsonObjectConst>()) return false;
    const ArduinoJson::JsonObjectConst segment =
      segments[index].as<ArduinoJson::JsonObjectConst>();
    JsonMainSplineSegment &destination = staged.spline_segments[index];
    if (
      segment.size() != 2
      || !extract_motion(segment["motion"], destination.motion)
      || destination.motion.telemetry_enabled
      || !json_joint_motion_detail::extract_controller_float(
        segment["rounding_millimeters"],
        destination.rounding_millimeters
      )
      || destination.rounding_millimeters < 0.0f
    ) return false;
  }
  if (
    staged.spline_segments[segments.size() - 1].rounding_millimeters
      != 0.0f
  ) return false;
  output = staged;
  return true;
}

inline bool extract_extended_motion(
  ArduinoJson::JsonObjectConst params,
  const char *modifier_name,
  bool linear,
  JsonMainDirectParameters &output
) {
  const size_t expected_fields = linear ? 3 : 2;
  if (params.size() != expected_fields || modifier_name == nullptr) {
    return false;
  }
  JsonMainDirectParameters staged = {};
  const ArduinoJson::JsonVariantConst modifier = params[modifier_name];
  if (
    !extract_motion(params["motion"], staged.motion)
    || !json_joint_motion_detail::extract_controller_float(
      modifier,
      staged.motion_modifier
    )
    || (linear && staged.motion_modifier != 0.0f)
  ) return false;
  if (linear) {
    const ArduinoJson::JsonVariantConst disabled =
      params["disable_wrist_rotation"];
    if (!disabled.is<bool>() || disabled.as<bool>()) return false;
    staged.kind = JsonMainDirectParameterKind::kLinearMotion;
    staged.disable_wrist_rotation = false;
  } else {
    staged.kind = JsonMainDirectParameterKind::kVisionMotion;
  }
  output = staged;
  return true;
}

inline bool extract_modbus_wait(
  ArduinoJson::JsonObjectConst params,
  ModbusOperation operation,
  JsonMainDirectParameters &output
) {
  if (params.size() != 4) return false;
  const ArduinoJson::JsonVariantConst slave = params["slave_id"];
  const ArduinoJson::JsonVariantConst address = params["address"];
  const ArduinoJson::JsonVariantConst expected = params["expected"];
  const ArduinoJson::JsonVariantConst timeout = params["timeout_seconds"];
  if (
    slave.is<bool>() || !slave.is<int>()
    || address.is<bool>() || !address.is<int>()
    || expected.is<bool>() || !expected.is<int>()
    || timeout.is<bool>() || !timeout.is<int>()
  ) return false;
  JsonMainDirectParameters staged = {};
  staged.kind = JsonMainDirectParameterKind::kModbusWait;
  staged.slave_id = slave.as<int>();
  staged.address = address.as<int>();
  staged.expected = expected.as<int>();
  if (!validate_modbus_wait(
      operation,
      staged.slave_id,
      staged.address,
      staged.expected,
      timeout.as<int>(),
      staged.timeout_milliseconds
  )) return false;
  output = staged;
  return true;
}

inline bool extract_storage_file(
  ArduinoJson::JsonObjectConst params,
  bool with_motion,
  JsonMainDirectParameters &output
) {
  if (params.size() != (with_motion ? 3U : 2U)) return false;
  JsonMainDirectParameters staged = {};
  if (
    !copy_media_id(params["media_id"], staged.media_id)
    || !copy_filename(params["filename"], staged.filename)
  ) return false;
  if (with_motion) {
    if (
      !extract_motion(params["motion"], staged.motion)
      || staged.motion.telemetry_enabled
    ) return false;
    staged.kind = JsonMainDirectParameterKind::kStorageMotion;
  } else {
    staged.kind = JsonMainDirectParameterKind::kStorageFile;
  }
  output = staged;
  return true;
}

}  // namespace json_direct_command_detail

inline bool extract_main_direct_parameters(
  const char *command,
  ArduinoJson::JsonObjectConst params,
  JsonMainDirectParameters &output
) {
  if (command == nullptr) return false;
  if (strcmp(command, "move_linear") == 0) {
    return json_direct_command_detail::extract_extended_motion(
      params,
      "rounding_millimeters",
      true,
      output
    );
  }
  if (strcmp(command, "move_arc") == 0) {
    return json_direct_command_detail::extract_arc_motion(params, output);
  }
  if (strcmp(command, "move_circle") == 0) {
    return json_direct_command_detail::extract_circle_motion(params, output);
  }
  if (strcmp(command, "move_spline") == 0) {
    return json_direct_command_detail::extract_spline_motion(params, output);
  }
  if (strcmp(command, "move_vision") == 0) {
    return json_direct_command_detail::extract_extended_motion(
      params,
      "vision_rotation_degrees",
      false,
      output
    );
  }
  if (
    strcmp(command, "wait_modbus_coil") == 0
    || strcmp(command, "wait_modbus_discrete_input") == 0
    || strcmp(command, "wait_modbus_holding_register") == 0
  ) {
    return json_direct_command_detail::extract_modbus_wait(
      params,
      strcmp(command, "wait_modbus_coil") == 0
        ? ModbusOperation::kReadCoil
        : strcmp(command, "wait_modbus_discrete_input") == 0
          ? ModbusOperation::kReadDiscreteInput
          : ModbusOperation::kReadHoldingRegisters,
      output
    );
  }
  if (
    strcmp(command, "delete_sd_program") == 0
    || strcmp(command, "play_gcode_file") == 0
  ) {
    return json_direct_command_detail::extract_storage_file(
      params,
      false,
      output
    );
  }
  if (strcmp(command, "write_gcode_move") == 0) {
    return json_direct_command_detail::extract_storage_file(
      params,
      true,
      output
    );
  }
  if (
    strcmp(command, "list_sd_programs") == 0
  ) {
    if (params.size() != 0) return false;
    output = {};
    output.kind = JsonMainDirectParameterKind::kEmpty;
    return true;
  }
  return false;
}

}  // namespace ar4_protocol

#endif
