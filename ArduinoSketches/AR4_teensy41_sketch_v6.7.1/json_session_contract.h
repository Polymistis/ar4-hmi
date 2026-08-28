#ifndef AR4_JSON_SESSION_CONTRACT_H
#define AR4_JSON_SESSION_CONTRACT_H

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "identity_contract.h"
#include "home_reference_contract.h"
#include "json_serial_frame_contract.h"

namespace ar4_protocol {

constexpr uint32_t kJsonProtocolVersion = 1;
constexpr size_t kJsonProtocolMaximumNameLength = 64;
constexpr size_t kJsonProtocolMaximumStringLength = 1024;
constexpr size_t kJsonProtocolMaximumErrorMessageLength = 512;
constexpr size_t kJsonSessionIdentifierLength = 32;
constexpr size_t kJsonHelloResponseCapacity = 4096;
constexpr size_t kJsonPositionResponseCapacity = 1024;
constexpr size_t kJsonPositionControllerDebugMaximumLength = 31;
constexpr size_t kJsonPositionMotionFaultMaximumLength = 8;
constexpr size_t kJsonPositionControllerAlarmMaximumLength = 11;
constexpr size_t kJsonHomeReferenceResponseCapacity = 512;
constexpr char kJsonProtocolName[] = "ar4_json";
constexpr char kJsonMainControllerDevice[] = "main_controller";
constexpr char kJsonPositionAxisSource[] = "controller_step_state";
constexpr char kJsonProtocolCapability[] = "JSON_PROTOCOL_V1";
constexpr char kJsonRequestCorrelationCapability[] =
  "REQUEST_CORRELATION_V1";
constexpr char kJsonEventStreamCapability[] = "EVENT_STREAM_V1";

struct JsonMainPositionSnapshot {
  int32_t robot_joints_millidegrees[6];
  int32_t external_axes_milliunits[3];
  int32_t cartesian_micrometers[3];
  int32_t orientation_millidegrees[3];
};

inline bool json_position_controller_debug_valid(const char *value) {
  if (value == nullptr) return false;
  size_t length = 0;
  while (value[length] != '\0') {
    if (length >= kJsonPositionControllerDebugMaximumLength) return false;
    ++length;
  }
  if (length == 0) return true;

  size_t index = value[0] == '-' ? 1 : 0;
  if (index == length) return false;
  bool digit_present = false;
  while (index < length && value[index] >= '0' && value[index] <= '9') {
    digit_present = true;
    ++index;
  }
  if (index == length) return digit_present;
  if (value[index++] != '.') return false;
  while (index < length && value[index] >= '0' && value[index] <= '9') {
    digit_present = true;
    ++index;
  }
  return digit_present && index == length;
}

inline bool json_position_text_length(
  const char *value,
  size_t maximum_length,
  size_t &length
) {
  if (value == nullptr) return false;
  length = 0;
  while (length <= maximum_length && value[length] != '\0') ++length;
  return length <= maximum_length;
}

inline bool json_position_motion_fault_valid(const char *value) {
  size_t length = 0;
  if (!json_position_text_length(
      value,
      kJsonPositionMotionFaultMaximumLength,
      length
  )) return false;
  if (length == 0) return true;
  if (length == 2) return strcmp(value, "EA") == 0 || strcmp(value, "EB") == 0;
  if (length != kJsonPositionMotionFaultMaximumLength
      || value[0] != 'E' || value[1] != 'C') {
    return false;
  }
  for (size_t index = 2; index < length; ++index) {
    if (value[index] != '0' && value[index] != '1') return false;
  }
  return true;
}

inline bool json_position_controller_alarm_valid(const char *value) {
  size_t length = 0;
  if (!json_position_text_length(
      value,
      kJsonPositionControllerAlarmMaximumLength,
      length
  )) return false;
  if (length == 2) return strcmp(value, "ER") == 0;
  if ((length != 8 && length != kJsonPositionControllerAlarmMaximumLength)
      || value[0] != 'E' || value[1] != 'L') {
    return false;
  }
  for (size_t index = 2; index < length; ++index) {
    if (value[index] != '0' && value[index] != '1') return false;
  }
  return true;
}

inline bool json_session_identifier_valid(const char *value) {
  if (value == nullptr) return false;
  for (size_t index = 0; index < kJsonSessionIdentifierLength; ++index) {
    if (!uppercase_hex_character(value[index])) return false;
  }
  return value[kJsonSessionIdentifierLength] == '\0';
}

inline bool json_hello_capabilities_valid(
  const char *const *capabilities,
  size_t capability_count
) {
  if (!protocol_capabilities_valid(capabilities, capability_count)) {
    return false;
  }
  bool protocol_present = false;
  bool correlation_present = false;
  bool event_stream_present = false;
  for (size_t index = 0; index < capability_count; ++index) {
    protocol_present = protocol_present || protocol_capability_equal(
      capabilities[index],
      kJsonProtocolCapability
    );
    correlation_present = correlation_present || protocol_capability_equal(
      capabilities[index],
      kJsonRequestCorrelationCapability
    );
    event_stream_present = event_stream_present || protocol_capability_equal(
      capabilities[index],
      kJsonEventStreamCapability
    );
  }
  return protocol_present && correlation_present && event_stream_present;
}

inline bool json_command_manifest_valid(
  const char *const *commands, size_t command_count
) {
  if (commands == nullptr || command_count == 0 || command_count > 64) return false;
  for (size_t index = 0; index < command_count; ++index) {
    const char *name = commands[index];
    if (name == nullptr || name[0] < 'a' || name[0] > 'z') return false;
    size_t length = 1;
    while (name[length] != '\0') {
      const char value = name[length++];
      if (length > kJsonProtocolMaximumNameLength
          || !((value >= 'a' && value <= 'z')
            || (value >= '0' && value <= '9') || value == '_')) return false;
    }
    for (size_t prior = 0; prior < index; ++prior) {
      if (strcmp(name, commands[prior]) == 0) return false;
    }
  }
  return true;
}

inline bool append_json_command_manifest(
  const char *const *commands, size_t command_count,
  char *output, size_t output_capacity, size_t &index
) {
  if (!json_command_manifest_valid(commands, command_count)
      || !append_json_text("[", output, output_capacity, index)) return false;
  for (size_t command = 0; command < command_count; ++command) {
    if ((command != 0 && !append_json_text(",", output, output_capacity, index))
        || !append_json_text("\"", output, output_capacity, index)
        || !append_json_text(commands[command], output, output_capacity, index)
        || !append_json_text("\"", output, output_capacity, index)) return false;
  }
  return append_json_text("]", output, output_capacity, index);
}

inline bool append_json_uint32(
  uint32_t value,
  char *output,
  size_t output_capacity,
  size_t &output_index
) {
  if (
    output == nullptr
    || output_capacity == 0
    || output_index >= output_capacity
  ) {
    return false;
  }
  char reversed[10] = {0};
  size_t digit_count = 0;
  do {
    reversed[digit_count++] = static_cast<char>('0' + value % 10);
    value /= 10;
  } while (value != 0);
  if (digit_count >= output_capacity - output_index) return false;
  while (digit_count > 0) {
    output[output_index++] = reversed[--digit_count];
  }
  output[output_index] = '\0';
  return true;
}

inline bool append_json_int32(
  int32_t value,
  char *output,
  size_t output_capacity,
  size_t &output_index
) {
  if (
    output == nullptr
    || output_capacity == 0
    || output_index >= output_capacity
  ) {
    return false;
  }
  if (value < 0) {
    if (!append_json_text("-", output, output_capacity, output_index)) {
      return false;
    }
    const uint32_t magnitude = static_cast<uint32_t>(
      -static_cast<int64_t>(value)
    );
    return append_json_uint32(
      magnitude,
      output,
      output_capacity,
      output_index
    );
  }
  return append_json_uint32(
    static_cast<uint32_t>(value),
    output,
    output_capacity,
    output_index
  );
}

inline bool append_json_int32_array(
  const int32_t *values,
  size_t value_count,
  char *output,
  size_t output_capacity,
  size_t &output_index
) {
  if (
    values == nullptr
    || output == nullptr
    || output_capacity == 0
    || output_index >= output_capacity
    || !append_json_text("[", output, output_capacity, output_index)
  ) {
    return false;
  }
  for (size_t index = 0; index < value_count; ++index) {
    if (
      (index > 0
        && !append_json_text(",", output, output_capacity, output_index))
      || !append_json_int32(
        values[index],
        output,
        output_capacity,
        output_index
      )
    ) {
      return false;
    }
  }
  return append_json_text("]", output, output_capacity, output_index);
}

inline bool append_json_bool_array(
  const bool *values,
  size_t value_count,
  char *output,
  size_t output_capacity,
  size_t &output_index
) {
  if (
    values == nullptr
    || output == nullptr
    || output_capacity == 0
    || output_index >= output_capacity
    || !append_json_text("[", output, output_capacity, output_index)
  ) {
    return false;
  }
  for (size_t index = 0; index < value_count; ++index) {
    if (
      (index > 0
        && !append_json_text(",", output, output_capacity, output_index))
      || !append_json_text(
        values[index] ? "true" : "false",
        output,
        output_capacity,
        output_index
      )
    ) {
      return false;
    }
  }
  return append_json_text("]", output, output_capacity, output_index);
}

inline bool build_main_json_hello_response(
  uint32_t request_id,
  const char *session_id,
  const char *firmware_name,
  const char *firmware_version,
  const char *firmware_build,
  const char *controller_hardware_id,
  const char *driver_model,
  const char *robot_model,
  const char *robot_version,
  const char *serial_number,
  const char *asset_tag,
  const char *const *capabilities,
  size_t capability_count,
  const char *const *commands,
  size_t command_count,
  size_t maximum_payload_bytes,
  char *output,
  size_t output_capacity
) {
  if (output == nullptr || output_capacity == 0) return false;
  output[0] = '\0';
  if (
    request_id == 0
    || !json_session_identifier_valid(session_id)
    || !identity_field_valid(firmware_name)
    || !identity_field_valid(firmware_version)
    || !identity_field_valid(firmware_build)
    || !controller_hardware_id_valid(controller_hardware_id)
    || !identity_field_valid(driver_model)
    || !identity_field_valid(robot_model)
    || !identity_field_valid(robot_version)
    || !identity_field_valid(serial_number)
    || !identity_field_valid(asset_tag)
    || !json_hello_capabilities_valid(capabilities, capability_count)
    || !json_command_manifest_valid(commands, command_count)
    || maximum_payload_bytes == 0
    || maximum_payload_bytes > kJsonProtocolMaximumPayloadBytes
  ) {
    return false;
  }

  const size_t bounded_capacity =
    output_capacity < maximum_payload_bytes + 1
    ? output_capacity
    : maximum_payload_bytes + 1;
  size_t index = 0;
  const bool built = append_json_text(
      "{\"cmd\":\"hello\",\"id\":",
      output,
      bounded_capacity,
      index
    )
    && append_json_uint32(request_id, output, bounded_capacity, index)
    && append_json_text(
      ",\"result\":{\"capabilities\":",
      output,
      bounded_capacity,
      index
    )
    && append_protocol_capabilities_json(
      capabilities,
      capability_count,
      output,
      bounded_capacity,
      index
    )
    && append_json_text(",\"commands\":", output, bounded_capacity, index)
    && append_json_command_manifest(
      commands, command_count, output, bounded_capacity, index
    )
    && append_json_text(
      ",\"device\":\"",
      output,
      bounded_capacity,
      index
    )
    && append_json_text(
      kJsonMainControllerDevice,
      output,
      bounded_capacity,
      index
    )
    && append_json_text(
      "\",\"firmware\":{\"build\":\"",
      output,
      bounded_capacity,
      index
    )
    && append_identity_json_value(
      firmware_build,
      output,
      bounded_capacity,
      index
    )
    && append_json_text(
      "\",\"name\":\"",
      output,
      bounded_capacity,
      index
    )
    && append_identity_json_value(
      firmware_name,
      output,
      bounded_capacity,
      index
    )
    && append_json_text(
      "\",\"version\":\"",
      output,
      bounded_capacity,
      index
    )
    && append_identity_json_value(
      firmware_version,
      output,
      bounded_capacity,
      index
    )
    && append_json_text(
      "\"},\"identity\":{\"asset_tag\":\"",
      output,
      bounded_capacity,
      index
    )
    && append_identity_json_value(
      asset_tag,
      output,
      bounded_capacity,
      index
    )
    && append_json_text(
      "\",\"controller_hardware_id\":\"",
      output,
      bounded_capacity,
      index
    )
    && append_json_text(
      controller_hardware_id,
      output,
      bounded_capacity,
      index
    )
    && append_json_text(
      "\",\"driver_model\":\"",
      output,
      bounded_capacity,
      index
    )
    && append_identity_json_value(
      driver_model,
      output,
      bounded_capacity,
      index
    )
    && append_json_text(
      "\",\"robot_model\":\"",
      output,
      bounded_capacity,
      index
    )
    && append_identity_json_value(
      robot_model,
      output,
      bounded_capacity,
      index
    )
    && append_json_text(
      "\",\"robot_version\":\"",
      output,
      bounded_capacity,
      index
    )
    && append_identity_json_value(
      robot_version,
      output,
      bounded_capacity,
      index
    )
    && append_json_text(
      "\",\"serial_number\":\"",
      output,
      bounded_capacity,
      index
    )
    && append_identity_json_value(
      serial_number,
      output,
      bounded_capacity,
      index
    )
    && append_json_text(
      "\"},\"protocol\":{\"max_payload_bytes\":",
      output,
      bounded_capacity,
      index
    )
    && append_json_uint32(
      static_cast<uint32_t>(maximum_payload_bytes),
      output,
      bounded_capacity,
      index
    )
    && append_json_text(
      ",\"name\":\"",
      output,
      bounded_capacity,
      index
    )
    && append_json_text(
      kJsonProtocolName,
      output,
      bounded_capacity,
      index
    )
    && append_json_text(
      "\",\"version\":",
      output,
      bounded_capacity,
      index
    )
    && append_json_uint32(
      kJsonProtocolVersion,
      output,
      bounded_capacity,
      index
    )
    && append_json_text(
      "},\"session_id\":\"",
      output,
      bounded_capacity,
      index
    )
    && append_json_text(session_id, output, bounded_capacity, index)
    && append_json_text(
      "\"},\"status\":\"completed\",\"type\":\"response\",\"v\":",
      output,
      bounded_capacity,
      index
    )
    && append_json_uint32(
      kJsonProtocolVersion,
      output,
      bounded_capacity,
      index
    )
    && append_json_text(
      "}",
      output,
      bounded_capacity,
      index
    );
  if (!built) {
    output[0] = '\0';
    return false;
  }
  return true;
}

namespace json_session_detail {

inline bool build_main_json_position_response(
  uint32_t request_id,
  const char *command,
  const JsonMainPositionSnapshot &snapshot,
  bool include_disposition,
  bool speed_limited,
  const char *controller_debug,
  const char *motion_fault,
  size_t maximum_payload_bytes,
  char *output,
  size_t output_capacity
) {
  if (output == nullptr || output_capacity == 0) return false;
  output[0] = '\0';
  if (
    request_id == 0
    || command == nullptr
    || maximum_payload_bytes == 0
    || maximum_payload_bytes > kJsonProtocolMaximumPayloadBytes
    || (
      include_disposition
      && (
        !json_position_controller_debug_valid(controller_debug)
        || !json_position_motion_fault_valid(motion_fault)
      )
    )
  ) {
    return false;
  }
  const bool command_valid = include_disposition
    && (
      strcmp(command, "get_position_disposition") == 0
      || strcmp(command, "correct_position") == 0
      || strcmp(command, "zero_j7") == 0
      || strcmp(command, "zero_j8") == 0
      || strcmp(command, "zero_j9") == 0
    );
  if (!command_valid) return false;

  const size_t bounded_capacity =
    output_capacity < maximum_payload_bytes + 1
    ? output_capacity
    : maximum_payload_bytes + 1;
  size_t index = 0;
  const bool built = append_json_text(
      "{\"cmd\":\"",
      output,
      bounded_capacity,
      index
    )
    && append_json_text(command, output, bounded_capacity, index)
    && append_json_text("\",\"id\":", output, bounded_capacity, index)
    && append_json_uint32(request_id, output, bounded_capacity, index)
    && append_json_text(
      ",\"result\":{\"axis_source\":\"",
      output,
      bounded_capacity,
      index
    )
    && append_json_text(
      kJsonPositionAxisSource,
      output,
      bounded_capacity,
      index
    )
    && append_json_text(
      "\",\"cartesian_micrometers\":",
      output,
      bounded_capacity,
      index
    )
    && append_json_int32_array(
      snapshot.cartesian_micrometers,
      3,
      output,
      bounded_capacity,
      index
    )
    && (
      !include_disposition
      || (
        append_json_text(
          ",\"controller_debug\":\"",
          output,
          bounded_capacity,
          index
        )
        && append_json_text(
          controller_debug,
          output,
          bounded_capacity,
          index
        )
        && append_json_text(
          "\"",
          output,
          bounded_capacity,
          index
        )
      )
    )
    && append_json_text(
      ",\"external_axes_milliunits\":",
      output,
      bounded_capacity,
      index
    )
    && append_json_int32_array(
      snapshot.external_axes_milliunits,
      3,
      output,
      bounded_capacity,
      index
    )
    && (
      !include_disposition
      || (
        append_json_text(
          ",\"motion_fault\":\"",
          output,
          bounded_capacity,
          index
        )
        && append_json_text(
          motion_fault,
          output,
          bounded_capacity,
          index
        )
        && append_json_text(
          "\"",
          output,
          bounded_capacity,
          index
        )
      )
    )
    && append_json_text(
      ",\"orientation_millidegrees\":",
      output,
      bounded_capacity,
      index
    )
    && append_json_int32_array(
      snapshot.orientation_millidegrees,
      3,
      output,
      bounded_capacity,
      index
    )
    && append_json_text(
      ",\"robot_joints_millidegrees\":",
      output,
      bounded_capacity,
      index
    )
    && append_json_int32_array(
      snapshot.robot_joints_millidegrees,
      6,
      output,
      bounded_capacity,
      index
    )
    && (
      !include_disposition
      || (
        append_json_text(
          ",\"speed_limited\":",
          output,
          bounded_capacity,
          index
        )
        && append_json_text(
          speed_limited ? "true" : "false",
          output,
          bounded_capacity,
          index
        )
      )
    )
    && append_json_text(
      "},\"status\":\"completed\",\"type\":\"response\",\"v\":",
      output,
      bounded_capacity,
      index
    )
    && append_json_uint32(
      kJsonProtocolVersion,
      output,
      bounded_capacity,
      index
    )
    && append_json_text("}", output, bounded_capacity, index);
  if (!built) {
    output[0] = '\0';
    return false;
  }
  return true;
}

}  // namespace json_session_detail

inline bool append_main_json_position_snapshot(
  const JsonMainPositionSnapshot &snapshot,
  char *output,
  size_t output_capacity,
  size_t &index
) {
  return append_json_text(
      "{\"axis_source\":\"controller_step_state\","
      "\"cartesian_micrometers\":",
      output,
      output_capacity,
      index
    )
    && append_json_int32_array(
      snapshot.cartesian_micrometers,
      3,
      output,
      output_capacity,
      index
    )
    && append_json_text(
      ",\"external_axes_milliunits\":",
      output,
      output_capacity,
      index
    )
    && append_json_int32_array(
      snapshot.external_axes_milliunits,
      3,
      output,
      output_capacity,
      index
    )
    && append_json_text(
      ",\"orientation_millidegrees\":",
      output,
      output_capacity,
      index
    )
    && append_json_int32_array(
      snapshot.orientation_millidegrees,
      3,
      output,
      output_capacity,
      index
    )
    && append_json_text(
      ",\"robot_joints_millidegrees\":",
      output,
      output_capacity,
      index
    )
    && append_json_int32_array(
      snapshot.robot_joints_millidegrees,
      6,
      output,
      output_capacity,
      index
    )
    && append_json_text("}", output, output_capacity, index);
}

inline bool build_main_json_position_disposition_response(
  uint32_t request_id,
  const JsonMainPositionSnapshot &snapshot,
  bool speed_limited,
  const char *controller_debug,
  const char *motion_fault,
  size_t maximum_payload_bytes,
  char *output,
  size_t output_capacity
) {
  return json_session_detail::build_main_json_position_response(
    request_id,
    "get_position_disposition",
    snapshot,
    true,
    speed_limited,
    controller_debug,
    motion_fault,
    maximum_payload_bytes,
    output,
    output_capacity
  );
}

inline bool build_main_json_correct_position_response(
  uint32_t request_id,
  const JsonMainPositionSnapshot &snapshot,
  bool speed_limited,
  const char *controller_debug,
  const char *motion_fault,
  size_t maximum_payload_bytes,
  char *output,
  size_t output_capacity
) {
  return json_session_detail::build_main_json_position_response(
    request_id,
    "correct_position",
    snapshot,
    true,
    speed_limited,
    controller_debug,
    motion_fault,
    maximum_payload_bytes,
    output,
    output_capacity
  );
}

inline bool build_main_json_external_axis_zero_response(
  uint32_t request_id,
  const char *command,
  const JsonMainPositionSnapshot &snapshot,
  bool speed_limited,
  const char *controller_debug,
  const char *motion_fault,
  size_t maximum_payload_bytes,
  char *output,
  size_t output_capacity
) {
  if (
    command == nullptr
    || (
      strcmp(command, "zero_j7") != 0
      && strcmp(command, "zero_j8") != 0
      && strcmp(command, "zero_j9") != 0
    )
  ) {
    if (output != nullptr && output_capacity != 0) output[0] = '\0';
    return false;
  }
  return json_session_detail::build_main_json_position_response(
    request_id,
    command,
    snapshot,
    true,
    speed_limited,
    controller_debug,
    motion_fault,
    maximum_payload_bytes,
    output,
    output_capacity
  );
}

inline bool build_main_json_home_reference_response(
  uint32_t request_id,
  const PrimaryHomeReferenceState &state,
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
  ) {
    return false;
  }
  for (size_t axis = 0; axis < kPrimaryHomeReferenceAxisCount; ++axis) {
    if (!state.valid[axis] && state.millidegrees[axis] != 0) return false;
  }

  const size_t bounded_capacity =
    output_capacity < maximum_payload_bytes + 1
    ? output_capacity
    : maximum_payload_bytes + 1;
  size_t index = 0;
  const bool built = append_json_text(
      "{\"cmd\":\"get_home_reference\",\"id\":",
      output,
      bounded_capacity,
      index
    )
    && append_json_uint32(request_id, output, bounded_capacity, index)
    && append_json_text(
      ",\"result\":{\"positions_millidegrees\":",
      output,
      bounded_capacity,
      index
    )
    && append_json_int32_array(
      state.millidegrees,
      kPrimaryHomeReferenceAxisCount,
      output,
      bounded_capacity,
      index
    )
    && append_json_text(",\"valid\":", output, bounded_capacity, index)
    && append_json_bool_array(
      state.valid,
      kPrimaryHomeReferenceAxisCount,
      output,
      bounded_capacity,
      index
    )
    && append_json_text(
      "},\"status\":\"completed\",\"type\":\"response\",\"v\":",
      output,
      bounded_capacity,
      index
    )
    && append_json_uint32(
      kJsonProtocolVersion,
      output,
      bounded_capacity,
      index
    )
    && append_json_text("}", output, bounded_capacity, index);
  if (!built) {
    output[0] = '\0';
    return false;
  }
  return true;
}

}  // namespace ar4_protocol

#endif
