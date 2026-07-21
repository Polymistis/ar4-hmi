#ifndef AR4_NUMERIC_PARSE_CONTRACT_H
#define AR4_NUMERIC_PARSE_CONTRACT_H

#include <errno.h>
#include <limits.h>
#include <math.h>
#include <stddef.h>
#include <stdlib.h>

namespace ar4_protocol {

template <size_t Count>
inline bool marker_positions_are_ordered(
  size_t command_length,
  const int (&positions)[Count]
) {
  static_assert(Count > 0, "marker list cannot be empty");
  if (positions[0] < 0) return false;
  for (size_t index = 1; index < Count; ++index) {
    if (positions[index] <= positions[index - 1]) return false;
  }
  return static_cast<size_t>(positions[Count - 1]) < command_length;
}

template <size_t Count>
inline bool marker_positions_are_ordered_from(
  size_t command_length,
  const int (&positions)[Count],
  int expected_first_position
) {
  return positions[0] == expected_first_position
    && marker_positions_are_ordered(command_length, positions);
}

template <size_t Count>
inline bool field_boundaries_cover_command(
  size_t command_length,
  const int (&positions)[Count],
  int expected_first_position = 0
) {
  static_assert(Count > 1, "field boundaries require a final sentinel");
  if (
    positions[0] != expected_first_position
    || positions[Count - 1] != static_cast<int>(command_length)
  ) {
    return false;
  }
  for (size_t index = 1; index < Count; ++index) {
    if (positions[index] <= positions[index - 1]) return false;
  }
  return true;
}

inline bool decimal_float_syntax(
  const char *text,
  bool &nonzero_mantissa
) {
  if (text == nullptr || *text == '\0') return false;

  const char *cursor = text;
  if (*cursor == '-') ++cursor;

  bool has_mantissa_digit = false;
  nonzero_mantissa = false;
  while (*cursor >= '0' && *cursor <= '9') {
    has_mantissa_digit = true;
    if (*cursor != '0') nonzero_mantissa = true;
    ++cursor;
  }

  if (*cursor == '.') {
    ++cursor;
    while (*cursor >= '0' && *cursor <= '9') {
      has_mantissa_digit = true;
      if (*cursor != '0') nonzero_mantissa = true;
      ++cursor;
    }
  }
  if (!has_mantissa_digit) return false;

  return *cursor == '\0';
}

inline bool parse_finite_decimal_float(const char *text, float &output) {
  bool nonzero_mantissa = false;
  if (!decimal_float_syntax(text, nonzero_mantissa)) return false;

  errno = 0;
  char *end = nullptr;
  const float parsed = strtof(text, &end);
  if (end == text || *end != '\0' || !isfinite(parsed)) return false;
  if (parsed == 0.0f && nonzero_mantissa) return false;

  output = parsed;
  return true;
}

inline bool parse_decimal_int(const char *text, int &output) {
  if (text == nullptr || *text == '\0') return false;

  const char *cursor = text;
  if (*cursor == '-') ++cursor;
  const char *digits_start = cursor;
  while (*cursor >= '0' && *cursor <= '9') ++cursor;
  if (cursor == digits_start || *cursor != '\0') return false;

  errno = 0;
  char *end = nullptr;
  const long parsed = strtol(text, &end, 10);
  if (
    end == text
    || *end != '\0'
    || errno == ERANGE
    || parsed < INT_MIN
    || parsed > INT_MAX
  ) {
    return false;
  }

  output = static_cast<int>(parsed);
  return true;
}

template <typename Text>
inline bool parse_float_span(
  const Text &command,
  int begin,
  int end,
  float &output
) {
  if (
    begin < 0
    || end <= begin
    || end > static_cast<int>(command.length())
  ) {
    return false;
  }
  const Text field = command.substring(begin, end);
  return parse_finite_decimal_float(field.c_str(), output);
}

template <typename Text>
inline bool parse_int_span(
  const Text &command,
  int begin,
  int end,
  int &output
) {
  if (
    begin < 0
    || end <= begin
    || end > static_cast<int>(command.length())
  ) {
    return false;
  }
  const Text field = command.substring(begin, end);
  return parse_decimal_int(field.c_str(), output);
}

template <typename Text, size_t Count>
inline bool parse_float_marker_fields(
  const Text &command,
  const int *positions,
  float (&outputs)[Count]
) {
  static_assert(Count > 0, "field list cannot be empty");
  if (positions == nullptr) return false;
  float staged[Count];
  for (size_t index = 0; index < Count; ++index) {
    if (
      positions[index] < 0
      || positions[index + 1] <= positions[index]
      || !parse_float_span(
        command,
        positions[index] + 1,
        positions[index + 1],
        staged[index]
      )
    ) {
      return false;
    }
  }
  for (size_t index = 0; index < Count; ++index) outputs[index] = staged[index];
  return true;
}

template <typename Text, size_t Count>
inline bool parse_int_marker_fields(
  const Text &command,
  const int *positions,
  int (&outputs)[Count]
) {
  static_assert(Count > 0, "field list cannot be empty");
  if (positions == nullptr) return false;
  int staged[Count];
  for (size_t index = 0; index < Count; ++index) {
    if (
      positions[index] < 0
      || positions[index + 1] <= positions[index]
      || !parse_int_span(
        command,
        positions[index] + 1,
        positions[index + 1],
        staged[index]
      )
    ) {
      return false;
    }
  }
  for (size_t index = 0; index < Count; ++index) outputs[index] = staged[index];
  return true;
}

template <typename Text, size_t Count>
inline bool parse_float_spans(
  const Text &command,
  const int (&begins)[Count],
  const int (&ends)[Count],
  float (&outputs)[Count]
) {
  float staged[Count];
  for (size_t index = 0; index < Count; ++index) {
    if (!parse_float_span(command, begins[index], ends[index], staged[index])) {
      return false;
    }
  }
  for (size_t index = 0; index < Count; ++index) outputs[index] = staged[index];
  return true;
}

template <typename Text, size_t Count>
inline bool parse_int_spans(
  const Text &command,
  const int (&begins)[Count],
  const int (&ends)[Count],
  int (&outputs)[Count]
) {
  int staged[Count];
  for (size_t index = 0; index < Count; ++index) {
    if (!parse_int_span(command, begins[index], ends[index], staged[index])) {
      return false;
    }
  }
  for (size_t index = 0; index < Count; ++index) outputs[index] = staged[index];
  return true;
}

template <typename Text, size_t Count>
inline bool parse_binary_digit_span(
  const Text &command,
  int begin,
  int end,
  int (&outputs)[Count]
) {
  if (
    begin < 0
    || end - begin != static_cast<int>(Count)
    || end > static_cast<int>(command.length())
  ) {
    return false;
  }

  int staged[Count];
  for (size_t index = 0; index < Count; ++index) {
    const char value = command.charAt(begin + static_cast<int>(index));
    if (value != '0' && value != '1') return false;
    staged[index] = value - '0';
  }
  for (size_t index = 0; index < Count; ++index) outputs[index] = staged[index];
  return true;
}

inline bool values_are_binary(const int *values, size_t count) {
  if (values == nullptr) return false;
  for (size_t index = 0; index < count; ++index) {
    if (values[index] != 0 && values[index] != 1) return false;
  }
  return true;
}

template <size_t Count>
inline bool values_are_binary(const int (&values)[Count]) {
  return values_are_binary(values, Count);
}

}  // namespace ar4_protocol

#endif
