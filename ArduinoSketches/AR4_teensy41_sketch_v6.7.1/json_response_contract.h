#ifndef AR4_JSON_RESPONSE_CONTRACT_H
#define AR4_JSON_RESPONSE_CONTRACT_H

#include "json_session_contract.h"

namespace ar4_protocol {

// Active firmware must retain the maximum error buffer outside the motion stack.
constexpr size_t kJsonErrorResponseCapacity =
  kJsonProtocolMaximumPayloadBytes + 1;

enum class JsonErrorResponseStatus {
  kRejected,
  kCancelled,
  kFailed,
};

struct JsonStringErrorDetail {
  const char *field_name;
  const char *value;
};

inline bool json_protocol_text_valid(
  const char *value,
  size_t maximum_length,
  bool allow_empty
) {
  if (value == nullptr) return false;
  size_t length = 0;
  while (value[length] != '\0') {
    if (length >= maximum_length) return false;
    const unsigned char character =
      static_cast<unsigned char>(value[length]);
    if (character < 0x20 || character > 0x7E) return false;
    ++length;
  }
  return allow_empty || length > 0;
}

inline bool json_protocol_name_valid(const char *value) {
  if (
    !json_protocol_text_valid(
      value,
      kJsonProtocolMaximumNameLength,
      false
    )
    || value[0] < 'a'
    || value[0] > 'z'
  ) {
    return false;
  }
  for (size_t index = 1; value[index] != '\0'; ++index) {
    const char character = value[index];
    if (
      !(
        (character >= 'a' && character <= 'z')
        || (character >= '0' && character <= '9')
        || character == '_'
      )
    ) {
      return false;
    }
  }
  return true;
}

inline bool json_protocol_field_name_valid(const char *value) {
  if (!json_protocol_text_valid(
      value,
      kJsonProtocolMaximumNameLength,
      false
  )) {
    return false;
  }
  const char first = value[0];
  if (
    !(
      (first >= 'A' && first <= 'Z')
      || (first >= 'a' && first <= 'z')
    )
  ) {
    return false;
  }
  for (size_t index = 1; value[index] != '\0'; ++index) {
    const char character = value[index];
    if (
      !(
        (character >= 'A' && character <= 'Z')
        || (character >= 'a' && character <= 'z')
        || (character >= '0' && character <= '9')
        || character == '_'
      )
    ) {
      return false;
    }
  }
  return true;
}

inline bool append_json_escaped_text(
  const char *value,
  size_t maximum_length,
  bool allow_empty,
  char *output,
  size_t output_capacity,
  size_t &output_index
) {
  if (
    output == nullptr
    || output_capacity == 0
    || output_index >= output_capacity
    || !json_protocol_text_valid(value, maximum_length, allow_empty)
  ) {
    return false;
  }
  for (size_t index = 0; value[index] != '\0'; ++index) {
    const char character = value[index];
    const size_t required =
      character == '"' || character == '\\' ? 2 : 1;
    if (required >= output_capacity - output_index) return false;
    if (required == 2) output[output_index++] = '\\';
    output[output_index++] = character;
  }
  output[output_index] = '\0';
  return true;
}

inline const char *json_error_response_status_name(
  JsonErrorResponseStatus status
) {
  switch (status) {
    case JsonErrorResponseStatus::kRejected:
      return "rejected";
    case JsonErrorResponseStatus::kCancelled:
      return "cancelled";
    case JsonErrorResponseStatus::kFailed:
      return "failed";
  }
  return nullptr;
}

inline bool json_string_error_detail_valid(
  const JsonStringErrorDetail *detail
) {
  return detail == nullptr
    || (
      json_protocol_field_name_valid(detail->field_name)
      && json_protocol_text_valid(
        detail->value,
        kJsonProtocolMaximumStringLength,
        true
      )
    );
}

inline bool append_json_error_object(
  const char *error_code,
  const char *message,
  const JsonStringErrorDetail *detail,
  char *output,
  size_t output_capacity,
  size_t &output_index
) {
  if (
    output == nullptr
    || output_capacity == 0
    || output_index >= output_capacity
    || !json_protocol_name_valid(error_code)
    || !json_protocol_text_valid(
      message,
      kJsonProtocolMaximumErrorMessageLength,
      false
    )
    || !json_string_error_detail_valid(detail)
    || !append_json_text(
      "{\"code\":\"",
      output,
      output_capacity,
      output_index
    )
    || !append_json_text(
      error_code,
      output,
      output_capacity,
      output_index
    )
    || !append_json_text(
      "\",\"details\":{",
      output,
      output_capacity,
      output_index
    )
  ) {
    return false;
  }
  if (
    detail != nullptr
    && (
      !append_json_text("\"", output, output_capacity, output_index)
      || !append_json_text(
        detail->field_name,
        output,
        output_capacity,
        output_index
      )
      || !append_json_text("\":\"", output, output_capacity, output_index)
      || !append_json_escaped_text(
        detail->value,
        kJsonProtocolMaximumStringLength,
        true,
        output,
        output_capacity,
        output_index
      )
      || !append_json_text("\"", output, output_capacity, output_index)
    )
  ) {
    return false;
  }
  return append_json_text(
      "},\"message\":\"",
      output,
      output_capacity,
      output_index
    )
    && append_json_escaped_text(
      message,
      kJsonProtocolMaximumErrorMessageLength,
      false,
      output,
      output_capacity,
      output_index
    )
    && append_json_text("\"}", output, output_capacity, output_index);
}

// Top-level builder inputs must not overlap output storage.
inline bool build_main_json_error_response(
  uint32_t request_id,
  const char *command,
  JsonErrorResponseStatus status,
  const char *error_code,
  const char *message,
  const JsonStringErrorDetail *detail,
  size_t maximum_payload_bytes,
  char *output,
  size_t output_capacity
) {
  if (output == nullptr || output_capacity == 0) return false;
  const char *status_name = json_error_response_status_name(status);
  const bool input_valid =
    request_id != 0
    && json_protocol_name_valid(command)
    && status_name != nullptr
    && json_protocol_name_valid(error_code)
    && json_protocol_text_valid(
      message,
      kJsonProtocolMaximumErrorMessageLength,
      false
    )
    && json_string_error_detail_valid(detail)
    && maximum_payload_bytes > 0
    && maximum_payload_bytes <= kJsonProtocolMaximumPayloadBytes;
  output[0] = '\0';
  if (!input_valid) return false;

  const size_t bounded_capacity = output_capacity < maximum_payload_bytes + 1
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
    && append_json_text("\",\"error\":", output, bounded_capacity, index)
    && append_json_error_object(
      error_code,
      message,
      detail,
      output,
      bounded_capacity,
      index
    )
    && append_json_text(",\"id\":", output, bounded_capacity, index)
    && append_json_uint32(request_id, output, bounded_capacity, index)
    && append_json_text(",\"status\":\"", output, bounded_capacity, index)
    && append_json_text(status_name, output, bounded_capacity, index)
    && append_json_text(
      "\",\"type\":\"response\",\"v\":",
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

inline bool build_main_json_completed_response(
  uint32_t request_id,
  const char *command,
  size_t maximum_payload_bytes,
  char *output,
  size_t output_capacity
) {
  if (output == nullptr || output_capacity == 0) return false;
  const bool input_valid = request_id != 0
    && json_protocol_name_valid(command)
    && maximum_payload_bytes > 0
    && maximum_payload_bytes <= kJsonProtocolMaximumPayloadBytes;
  output[0] = '\0';
  if (!input_valid) return false;

  const size_t bounded_capacity = output_capacity < maximum_payload_bytes + 1
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
    && append_json_text(
      "\",\"id\":",
      output,
      bounded_capacity,
      index
    )
    && append_json_uint32(request_id, output, bounded_capacity, index)
    && append_json_text(
      ",\"result\":{},\"status\":\"completed\",\"type\":\"response\",\"v\":",
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

inline bool build_json_protocol_error_response(
  const char *error_code,
  const char *message,
  const JsonStringErrorDetail *detail,
  size_t maximum_payload_bytes,
  char *output,
  size_t output_capacity
) {
  if (output == nullptr || output_capacity == 0) return false;
  const bool input_valid =
    json_protocol_name_valid(error_code)
    && json_protocol_text_valid(
      message,
      kJsonProtocolMaximumErrorMessageLength,
      false
    )
    && json_string_error_detail_valid(detail)
    && maximum_payload_bytes > 0
    && maximum_payload_bytes <= kJsonProtocolMaximumPayloadBytes;
  output[0] = '\0';
  if (!input_valid) return false;

  const size_t bounded_capacity = output_capacity < maximum_payload_bytes + 1
    ? output_capacity
    : maximum_payload_bytes + 1;
  size_t index = 0;
  const bool built = append_json_text(
      "{\"error\":",
      output,
      bounded_capacity,
      index
    )
    && append_json_error_object(
      error_code,
      message,
      detail,
      output,
      bounded_capacity,
      index
    )
    && append_json_text(
      ",\"type\":\"protocol_error\",\"v\":",
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
