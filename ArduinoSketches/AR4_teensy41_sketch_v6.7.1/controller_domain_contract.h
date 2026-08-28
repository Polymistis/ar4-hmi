#ifndef AR4_CONTROLLER_DOMAIN_CONTRACT_H
#define AR4_CONTROLLER_DOMAIN_CONTRACT_H

#include <float.h>
#include <limits.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>

namespace ar4_protocol {

constexpr size_t kControllerAxisCount = 9;
constexpr size_t kControllerPrimaryAxisCount = 6;
constexpr size_t kControllerFilenameMaxLength = 255;
constexpr size_t kControllerDirectoryPayloadMaxLength = 4096;
constexpr size_t kControllerMediaIdByteLength = 16;
constexpr size_t kControllerMediaIdLength =
  kControllerMediaIdByteLength * 2;
constexpr size_t kControllerMediaIdCapacity = kControllerMediaIdLength + 1;
constexpr char kControllerDirectoryIdentityPrefix[] = "MID:";
constexpr char kControllerDirectoryIdentitySeparator = '|';
constexpr size_t kControllerDirectoryIdentityPrefixLength =
  sizeof(kControllerDirectoryIdentityPrefix) - 1
  + kControllerMediaIdLength
  + 1;
constexpr int kMainFirmwareWaitMaxSeconds = 2147483;
constexpr int kModbusMinimumSlaveId = 1;
constexpr int kModbusMaximumSlaveId = 247;
constexpr int kModbusMaximumAddress = 65535;
constexpr int kModbusMaximumRegisterValue = 65535;
constexpr int kModbusMaximumRegisterReadQuantity = 1;
constexpr double kMaximumPulseDelayMicroseconds = 4294967295.0;

enum class ModbusOperation {
  kReadCoil,
  kReadDiscreteInput,
  kReadHoldingRegisters,
  kReadInputRegisters,
  kWriteCoil,
  kWriteRegister,
};

enum class SDFileLookupStatus {
  kPresent,
  kAbsent,
  kError,
};

struct ExternalAxisCalibration {
  float positive_limit;
  float steps_per_unit;
  int step_limit;
  int zero_step;
};

inline bool checked_double_to_int(double value, int &output) {
  if (
    !isfinite(value)
    || value < static_cast<double>(INT_MIN)
    || value > static_cast<double>(INT_MAX)
  ) {
    return false;
  }
  output = static_cast<int>(value);
  return true;
}

inline bool checked_nonnegative_double_to_int(double value, int &output) {
  return value >= 0.0 && checked_double_to_int(value, output);
}

inline bool validate_encoder_calibration(
  int step_limit,
  float encoder_counts_per_step
) {
  if (
    step_limit < 0
    || !isfinite(encoder_counts_per_step)
    || encoder_counts_per_step <= 0.0f
  ) {
    return false;
  }
  const double scale = static_cast<double>(encoder_counts_per_step);
  const double maximum_written_count =
    static_cast<double>(step_limit) * scale;
  return (
    isfinite(maximum_written_count)
    && maximum_written_count >= 0.0
    && maximum_written_count <= static_cast<double>(INT32_MAX)
  );
}

inline bool configured_encoder_count_to_step(
  int32_t encoder_count,
  float encoder_counts_per_step,
  int &output
) {
  if (
    !isfinite(encoder_counts_per_step)
    || encoder_counts_per_step <= 0.0f
  ) {
    return false;
  }
  return checked_double_to_int(
    static_cast<double>(encoder_count)
      / static_cast<double>(encoder_counts_per_step),
    output
  );
}

inline bool configured_step_to_encoder_count(
  int step,
  float encoder_counts_per_step,
  int32_t &output
) {
  if (
    !isfinite(encoder_counts_per_step)
    || encoder_counts_per_step <= 0.0f
  ) {
    return false;
  }
  const double count =
    static_cast<double>(step)
      * static_cast<double>(encoder_counts_per_step);
  if (
    !isfinite(count)
    || count < static_cast<double>(INT32_MIN)
    || count > static_cast<double>(INT32_MAX)
  ) {
    return false;
  }
  output = static_cast<int32_t>(count);
  return true;
}

inline bool encoder_step_difference_reaches_threshold(
  int encoder_step,
  int commanded_step,
  int threshold
) {
  if (threshold < 0) return true;
  int64_t difference = static_cast<int64_t>(encoder_step)
    - static_cast<int64_t>(commanded_step);
  if (difference < 0) difference = -difference;
  return difference >= static_cast<int64_t>(threshold);
}

inline bool calibration_release_step_limit(
  float steps_per_unit,
  float maximum_travel,
  int configured_step_limit,
  int &step_limit
) {
  if (
    !isfinite(steps_per_unit)
    || !isfinite(maximum_travel)
    || steps_per_unit <= 0.0f
    || maximum_travel <= 0.0f
    || configured_step_limit <= 0
  ) {
    return false;
  }
  const double required_steps = ceil(
    static_cast<double>(steps_per_unit)
    * static_cast<double>(maximum_travel)
  );
  int staged_step_limit = 0;
  if (
    required_steps < 1.0
    || !checked_nonnegative_double_to_int(
      required_steps,
      staged_step_limit
    )
  ) {
    return false;
  }
  if (staged_step_limit > configured_step_limit) {
    staged_step_limit = configured_step_limit;
  }
  step_limit = staged_step_limit;
  return true;
}

inline bool validate_axis_calibration(
  float negative_limit,
  float positive_limit,
  float steps_per_unit,
  int &step_limit,
  int &zero_step
) {
  if (
    !isfinite(negative_limit)
    || !isfinite(positive_limit)
    || !isfinite(steps_per_unit)
    || negative_limit < 0.0f
    || positive_limit < 0.0f
    || steps_per_unit <= 0.0f
  ) {
    return false;
  }

  const float travel = negative_limit + positive_limit;
  const float step_limit_value = travel * steps_per_unit;
  const float zero_step_value = negative_limit * steps_per_unit;
  int staged_step_limit = 0;
  int staged_zero_step = 0;
  if (
    !isfinite(travel)
    || !isfinite(step_limit_value)
    || !isfinite(zero_step_value)
    || !checked_nonnegative_double_to_int(step_limit_value, staged_step_limit)
    || !checked_nonnegative_double_to_int(zero_step_value, staged_zero_step)
    || staged_zero_step > staged_step_limit
  ) {
    return false;
  }

  step_limit = staged_step_limit;
  zero_step = staged_zero_step;
  return true;
}

inline bool validate_primary_axis_calibration(
  float negative_limit,
  float positive_limit,
  float steps_per_unit,
  int &step_limit,
  int &zero_step
) {
  const float travel = negative_limit + positive_limit;
  return isfinite(travel)
    && travel > 0.0f
    && validate_axis_calibration(
      negative_limit,
      positive_limit,
      steps_per_unit,
      step_limit,
      zero_step
    );
}

struct ControllerPositionRebase {
  int step_monitors[kControllerAxisCount];
  int32_t encoder_counts[kControllerPrimaryAxisCount];
};

inline bool build_controller_position_rebase(
  const int (&step_monitors)[kControllerAxisCount],
  const int (&step_limits)[kControllerAxisCount],
  const float (&encoder_counts_per_step)[kControllerPrimaryAxisCount],
  ControllerPositionRebase &output
) {
  ControllerPositionRebase staged = {};
  for (size_t axis = 0; axis < kControllerAxisCount; ++axis) {
    if (
      step_limits[axis] < 0
      || step_monitors[axis] < 0
      || step_monitors[axis] > step_limits[axis]
    ) {
      return false;
    }
    staged.step_monitors[axis] = step_monitors[axis];
  }
  for (size_t axis = 0; axis < kControllerPrimaryAxisCount; ++axis) {
    if (!validate_encoder_calibration(
        step_limits[axis],
        encoder_counts_per_step[axis]
    )) {
      return false;
    }
    if (!configured_step_to_encoder_count(
        step_monitors[axis],
        encoder_counts_per_step[axis],
        staged.encoder_counts[axis]
    )) {
      return false;
    }
  }
  output = staged;
  return true;
}

inline bool calibrated_position_to_step(
  float position,
  float negative_limit,
  float positive_limit,
  float steps_per_unit,
  int configured_step_limit,
  int &future_step
) {
  int derived_step_limit = 0;
  int zero_step = 0;
  if (
    !isfinite(position)
    || !validate_axis_calibration(
      negative_limit,
      positive_limit,
      steps_per_unit,
      derived_step_limit,
      zero_step
    )
    || configured_step_limit < 0
    || configured_step_limit != derived_step_limit
    || position < -negative_limit
    || position > positive_limit
  ) {
    return false;
  }

  int staged_future_step = 0;
  const float shifted_position = position + negative_limit;
  const float future_step_value = shifted_position * steps_per_unit;
  if (
    !isfinite(shifted_position)
    || !isfinite(future_step_value)
    || !checked_nonnegative_double_to_int(future_step_value, staged_future_step)
    || staged_future_step > configured_step_limit
  ) {
    return false;
  }
  future_step = staged_future_step;
  return true;
}

template <size_t Count>
inline bool calibrated_positions_to_steps(
  const float (&positions)[Count],
  const float (&negative_limits)[Count],
  const float (&positive_limits)[Count],
  const float (&steps_per_unit)[Count],
  const int (&step_limits)[Count],
  int (&future_steps)[Count]
) {
  int staged[Count];
  for (size_t index = 0; index < Count; ++index) {
    if (!calibrated_position_to_step(
        positions[index],
        negative_limits[index],
        positive_limits[index],
        steps_per_unit[index],
        step_limits[index],
        staged[index]
    )) {
      return false;
    }
  }
  for (size_t index = 0; index < Count; ++index) {
    future_steps[index] = staged[index];
  }
  return true;
}

inline bool validate_external_axis_calibration(
  float length,
  float rotation,
  float steps,
  ExternalAxisCalibration &output
) {
  if (
    !isfinite(length)
    || !isfinite(rotation)
    || !isfinite(steps)
    || length < 0.0f
    || rotation <= 0.0f
    || steps <= 0.0f
  ) {
    return false;
  }
  const float steps_per_unit = steps / rotation;
  int step_limit = 0;
  int zero_step = 0;
  if (
    !isfinite(steps_per_unit)
    || steps_per_unit <= 0.0f
    || !validate_axis_calibration(
      0.0f,
      length,
      steps_per_unit,
      step_limit,
      zero_step
    )
  ) {
    return false;
  }

  ExternalAxisCalibration staged = {};
  staged.positive_limit = length;
  staged.steps_per_unit = steps_per_unit;
  staged.step_limit = step_limit;
  staged.zero_step = zero_step;
  output = staged;
  return true;
}

inline bool calibration_reference_steps(
  int requested,
  int calibration_direction,
  float positive_limit,
  float negative_limit,
  float steps_per_unit,
  int step_limit,
  float base_offset,
  float command_offset,
  bool requires_joint_five_offset,
  int &master_step,
  int &center_step,
  int &joint_five_step
) {
  if (requested == 0) {
    master_step = 0;
    center_step = 0;
    joint_five_step = 0;
    return true;
  }
  int checked_step_limit = 0;
  int zero_step = 0;
  if (
    requested != 1
    || (calibration_direction != 0 && calibration_direction != 1)
    || !isfinite(base_offset)
    || !isfinite(command_offset)
    || !validate_axis_calibration(
      negative_limit,
      positive_limit,
      steps_per_unit,
      checked_step_limit,
      zero_step
    )
    || checked_step_limit <= 0
    || checked_step_limit != step_limit
  ) {
    return false;
  }

  const double offset = static_cast<double>(base_offset)
    + static_cast<double>(command_offset);
  const double master_units = calibration_direction == 1
    ? static_cast<double>(positive_limit)
      + static_cast<double>(negative_limit)
      + offset
    : offset;
  const double center_units = calibration_direction == 1
    ? static_cast<double>(positive_limit) + offset
    : static_cast<double>(negative_limit) - offset;
  const double joint_five_units = calibration_direction == 1
    ? static_cast<double>(negative_limit) + offset - 45.0
    : static_cast<double>(negative_limit) - offset + 45.0;

  int staged_master_step = 0;
  int staged_center_step = 0;
  int staged_joint_five_step = 0;
  if (
    !checked_double_to_int(
      master_units * static_cast<double>(steps_per_unit),
      staged_master_step
    )
    || !checked_nonnegative_double_to_int(
      center_units * static_cast<double>(steps_per_unit),
      staged_center_step
    )
    || staged_center_step > step_limit
    || (
      requires_joint_five_offset
      && (
        !checked_nonnegative_double_to_int(
          joint_five_units * static_cast<double>(steps_per_unit),
          staged_joint_five_step
        )
        || staged_joint_five_step > step_limit
      )
    )
  ) {
    return false;
  }

  master_step = staged_master_step;
  center_step = staged_center_step;
  joint_five_step = staged_joint_five_step;
  return true;
}

inline bool wait_seconds_to_milliseconds(
  float seconds,
  uint32_t &milliseconds
) {
  if (
    !isfinite(seconds)
    || seconds < 0.0f
    || seconds > static_cast<float>(kMainFirmwareWaitMaxSeconds)
  ) {
    return false;
  }
  const double converted = static_cast<double>(seconds) * 1000.0;
  if (converted < 0.0 || converted > 2147483000.0) return false;
  milliseconds = static_cast<uint32_t>(converted);
  return true;
}

inline bool wait_seconds_to_milliseconds(
  int seconds,
  uint32_t &milliseconds
) {
  if (seconds < 0 || seconds > kMainFirmwareWaitMaxSeconds) return false;
  milliseconds = static_cast<uint32_t>(seconds) * 1000U;
  return true;
}

inline bool validate_modbus_request(
  ModbusOperation operation,
  int slave_id,
  int address,
  int value
) {
  if (
    slave_id < kModbusMinimumSlaveId
    || slave_id > kModbusMaximumSlaveId
    || address < 0
    || address > kModbusMaximumAddress
  ) {
    return false;
  }

  switch (operation) {
    case ModbusOperation::kReadCoil:
    case ModbusOperation::kReadDiscreteInput:
      return value == 1;
    case ModbusOperation::kReadHoldingRegisters:
    case ModbusOperation::kReadInputRegisters:
      return value == kModbusMaximumRegisterReadQuantity;
    case ModbusOperation::kWriteCoil:
      return value == 0 || value == 1;
    case ModbusOperation::kWriteRegister:
      return value >= 0 && value <= kModbusMaximumRegisterValue;
  }
  return false;
}

inline bool validate_modbus_wait(
  ModbusOperation operation,
  int slave_id,
  int address,
  int expected_value,
  int timeout_seconds,
  uint32_t &timeout_milliseconds
) {
  const bool operation_valid =
    operation == ModbusOperation::kReadCoil
      || operation == ModbusOperation::kReadDiscreteInput
      || operation == ModbusOperation::kReadHoldingRegisters;
  const bool expected_valid =
    operation == ModbusOperation::kReadHoldingRegisters
      ? expected_value >= 0
        && expected_value <= kModbusMaximumRegisterValue
      : (expected_value == 0 || expected_value == 1);
  return operation_valid
    && expected_valid
    && timeout_seconds > 0
    && validate_modbus_request(operation, slave_id, address, 1)
    && wait_seconds_to_milliseconds(timeout_seconds, timeout_milliseconds);
}

inline bool fat_reserved_filename_character(unsigned char value) {
  switch (value) {
    case '"':
    case '*':
    case '/':
    case ':':
    case '<':
    case '>':
    case '?':
    case '\\':
    case '|':
      return true;
    default:
      return false;
  }
}

constexpr char kControllerDirectorySeparator = ',';

inline bool uppercase_hex_character(unsigned char value) {
  return (
    (value >= static_cast<unsigned char>('0')
      && value <= static_cast<unsigned char>('9'))
    || (value >= static_cast<unsigned char>('A')
      && value <= static_cast<unsigned char>('F'))
  );
}

template <typename Text>
inline bool valid_controller_media_id(
  const Text &text,
  int begin,
  int end
) {
  if (
    begin < 0
    || end - begin != static_cast<int>(kControllerMediaIdLength)
    || end > static_cast<int>(text.length())
  ) {
    return false;
  }
  for (int index = begin; index < end; ++index) {
    if (!uppercase_hex_character(
        static_cast<unsigned char>(text.charAt(index))
    )) {
      return false;
    }
  }
  return true;
}

inline bool format_controller_media_id(
  const uint8_t *cid_bytes,
  size_t cid_length,
  char *output,
  size_t output_capacity
) {
  if (
    cid_bytes == nullptr
    || cid_length != kControllerMediaIdByteLength
    || output == nullptr
    || output_capacity < kControllerMediaIdCapacity
  ) {
    return false;
  }
  static const char kHexDigits[] = "0123456789ABCDEF";
  for (size_t index = 0; index < cid_length; ++index) {
    output[index * 2] = kHexDigits[(cid_bytes[index] >> 4) & 0x0F];
    output[index * 2 + 1] = kHexDigits[cid_bytes[index] & 0x0F];
  }
  output[kControllerMediaIdLength] = '\0';
  return true;
}

template <typename Text>
inline bool valid_controller_filename(
  const Text &text,
  int begin,
  int end
) {
  if (
    begin < 0
    || end <= begin
    || end > static_cast<int>(text.length())
  ) {
    return false;
  }
  const size_t length = static_cast<size_t>(end - begin);
  if (length > kControllerFilenameMaxLength) return false;
  if (
    (length == 1 && text.charAt(begin) == '.')
    || (
      length == 2
      && text.charAt(begin) == '.'
      && text.charAt(begin + 1) == '.'
    )
  ) {
    return false;
  }
  for (int index = begin; index < end; ++index) {
    const unsigned char value = static_cast<unsigned char>(text.charAt(index));
    if (
      value < 32
      || value > 126
      || fat_reserved_filename_character(value)
      || value == static_cast<unsigned char>(kControllerDirectorySeparator)
      || (value == ' ' && (index == begin || index == end - 1))
    ) {
      return false;
    }
  }
  return true;
}

template <typename Text>
inline bool parse_gcode_media_filename_suffix(
  const Text &text,
  int &filename_begin
) {
  constexpr int kMediaBegin = 2;
  constexpr int kMediaEnd =
    kMediaBegin + static_cast<int>(kControllerMediaIdLength);
  constexpr int kFilenameMarkerEnd = kMediaEnd + 2;
  if (
    static_cast<int>(text.length()) <= kFilenameMarkerEnd
    || text.charAt(0) != 'M'
    || text.charAt(1) != 'i'
    || !valid_controller_media_id(text, kMediaBegin, kMediaEnd)
    || text.charAt(kMediaEnd) != 'F'
    || text.charAt(kMediaEnd + 1) != 'n'
    || !valid_controller_filename(
      text,
      kFilenameMarkerEnd,
      static_cast<int>(text.length())
    )
  ) {
    return false;
  }
  filename_begin = kFilenameMarkerEnd;
  return true;
}

inline bool ascii_case_matches(char value, char lowercase) {
  return value == lowercase || value == static_cast<char>(lowercase - 'a' + 'A');
}

inline unsigned char ascii_fold_lowercase(unsigned char value) {
  return value >= static_cast<unsigned char>('A')
      && value <= static_cast<unsigned char>('Z')
    ? static_cast<unsigned char>(value - 'A' + 'a')
    : value;
}

inline bool controller_filenames_equal_ignore_case(
  const char *left,
  const char *right
) {
  if (left == nullptr || right == nullptr) return false;
  size_t index = 0;
  while (left[index] != '\0' && right[index] != '\0') {
    if (
      ascii_fold_lowercase(static_cast<unsigned char>(left[index]))
      != ascii_fold_lowercase(static_cast<unsigned char>(right[index]))
    ) {
      return false;
    }
    ++index;
  }
  return left[index] == '\0' && right[index] == '\0';
}

template <typename DirectoryEntry>
inline bool read_controller_directory_entry_name(
  DirectoryEntry &entry,
  char *name,
  size_t capacity,
  size_t &name_length
) {
  if (name == nullptr || capacity < 2) return false;
  for (size_t index = 0; index < capacity; ++index) {
    name[index] = static_cast<char>(0x7F);
  }
  // Supported SdFat releases expose either a success flag or copied length.
  // Zero/nonzero is shared; the terminated output defines the wire length.
  if (!entry.getName(name, capacity)) {
    name[0] = '\0';
    return false;
  }

  size_t staged_length = 0;
  while (staged_length < capacity && name[staged_length] != '\0') {
    ++staged_length;
  }
  if (staged_length == 0 || staged_length >= capacity) {
    name[0] = '\0';
    return false;
  }
  name_length = staged_length;
  return true;
}

template <typename Text>
inline bool controller_filename_has_txt_suffix(
  const Text &text,
  int begin,
  int end
) {
  return (
    begin >= 0
    && end - begin >= 4
    && end <= static_cast<int>(text.length())
    && text.charAt(end - 4) == '.'
    && ascii_case_matches(text.charAt(end - 3), 't')
    && ascii_case_matches(text.charAt(end - 2), 'x')
    && ascii_case_matches(text.charAt(end - 1), 't')
  );
}

template <typename Text>
inline bool valid_controller_directory_entry_filename(
  const Text &text,
  int begin,
  int end
) {
  if (!valid_controller_filename(text, begin, end)) return false;
  if (!controller_filename_has_txt_suffix(text, begin, end)) return true;

  const int stem_end = end - 4;
  const int stem_length = stem_end - begin;
  return (
    stem_length > 0
    && !(
      stem_length == 1
      && text.charAt(begin) == '.'
    )
    && !(
      stem_length == 2
      && text.charAt(begin) == '.'
      && text.charAt(begin + 1) == '.'
    )
  );
}

inline bool controller_directory_entry_fits_payload(
  size_t current_length,
  size_t filename_length
) {
  return (
    filename_length > 0
    && current_length < kControllerDirectoryPayloadMaxLength
    && filename_length
      < kControllerDirectoryPayloadMaxLength - current_length
  );
}

inline bool stored_step_target(
  int current_step,
  int step_count,
  int direction,
  int step_limit,
  int &future_step
) {
  if (
    current_step < 0
    || current_step > step_limit
    || step_count < 0
    || step_count > step_limit
    || (direction != 0 && direction != 1)
    || step_limit < 0
  ) {
    return false;
  }
  const int64_t staged = static_cast<int64_t>(current_step)
    + (direction == 1
      ? static_cast<int64_t>(step_count)
      : -static_cast<int64_t>(step_count));
  if (staged < 0 || staged > step_limit) return false;
  future_step = static_cast<int>(staged);
  return true;
}

inline bool valid_delay_envelope(
  double cruise_delay,
  double acceleration_delay,
  double deceleration_delay,
  bool use_rounding_delay,
  double rounding_delay
) {
  const double delays[] = {
    cruise_delay,
    acceleration_delay,
    deceleration_delay,
  };
  for (size_t index = 0; index < 3; ++index) {
    if (
      !isfinite(delays[index])
      || delays[index] <= 0.0
      || delays[index] > kMaximumPulseDelayMicroseconds
    ) {
      return false;
    }
  }
  return !use_rounding_delay
    || (
      isfinite(rounding_delay)
      && rounding_delay > 0.0
      && rounding_delay <= kMaximumPulseDelayMicroseconds
    );
}

inline bool consume_motion_rounding_continuation(
  bool continuation_enabled,
  bool &continuation_pending
) {
  const bool selected = continuation_enabled && continuation_pending;
  continuation_pending = false;
  return selected;
}

inline bool pulse_delay_microseconds(
  double requested_delay,
  double distribution_delay,
  double minimum_delay,
  uint32_t &delay_microseconds
) {
  if (
    !isfinite(requested_delay)
    || !isfinite(distribution_delay)
    || !isfinite(minimum_delay)
    || requested_delay <= 0.0
    || distribution_delay < 0.0
    || minimum_delay <= 0.0
  ) {
    return false;
  }
  const double staged = fmax(
    minimum_delay,
    requested_delay - distribution_delay
  );
  if (
    !isfinite(staged)
    || staged > kMaximumPulseDelayMicroseconds
  ) {
    return false;
  }
  delay_microseconds = static_cast<uint32_t>(ceil(staged));
  return delay_microseconds > 0;
}

inline bool waypoint_count_for_path(
  float path_length,
  float waypoint_spacing,
  int &waypoint_count
) {
  if (
    !isfinite(path_length)
    || !isfinite(waypoint_spacing)
    || path_length <= 0.0f
    || waypoint_spacing <= 0.0f
  ) {
    return false;
  }
  const double count = ceil(
    static_cast<double>(path_length) / waypoint_spacing
  );
  if (count < 1.0 || count > static_cast<double>(INT_MAX)) return false;
  waypoint_count = static_cast<int>(count);
  return true;
}

inline bool interpolated_step_target(
  int start_step,
  int target_step,
  int waypoint_index,
  int waypoint_count,
  int step_limit,
  int &future_step
) {
  if (
    step_limit < 0
    || start_step < 0
    || start_step > step_limit
    || target_step < 0
    || target_step > step_limit
    || waypoint_count <= 0
    || waypoint_index < 0
    || waypoint_index > waypoint_count
  ) {
    return false;
  }
  const int64_t delta = static_cast<int64_t>(target_step) - start_step;
  const int64_t staged = static_cast<int64_t>(start_step)
    + delta * waypoint_index / waypoint_count;
  if (staged < 0 || staged > step_limit) return false;
  future_step = static_cast<int>(staged);
  return true;
}

inline bool supported_trajectory_rotation(float value) {
  return isfinite(value) && value == 0.0f;
}

inline bool valid_positive_spline_rounding(
  float rounding,
  float incoming_length,
  float outgoing_length
) {
  return isfinite(rounding) && rounding > 0.0f
    && isfinite(incoming_length) && incoming_length > 0.0f
    && isfinite(outgoing_length) && outgoing_length > 0.0f
    && rounding <= incoming_length * 0.45f
    && rounding <= outgoing_length * 0.45f;
}

inline bool valid_circle_geometry(
  const float *center,
  const float *start,
  const float *plane,
  float minimum_path_length
) {
  if (
    center == nullptr
    || start == nullptr
    || plane == nullptr
    || !isfinite(minimum_path_length)
    || minimum_path_length <= 0.0f
  ) {
    return false;
  }
  for (size_t index = 0; index < 3; ++index) {
    if (!isfinite(center[index]) || !isfinite(start[index]) || !isfinite(plane[index])) {
      return false;
    }
  }
  const double start_vector[3] = {
    static_cast<double>(start[0]) - center[0],
    static_cast<double>(start[1]) - center[1],
    static_cast<double>(start[2]) - center[2],
  };
  const double plane_vector[3] = {
    static_cast<double>(plane[0]) - center[0],
    static_cast<double>(plane[1]) - center[1],
    static_cast<double>(plane[2]) - center[2],
  };
  const double start_radius = sqrt(
    start_vector[0] * start_vector[0]
      + start_vector[1] * start_vector[1]
      + start_vector[2] * start_vector[2]
  );
  const double plane_radius = sqrt(
    plane_vector[0] * plane_vector[0]
      + plane_vector[1] * plane_vector[1]
      + plane_vector[2] * plane_vector[2]
  );
  const double cross[3] = {
    start_vector[1] * plane_vector[2] - start_vector[2] * plane_vector[1],
    start_vector[2] * plane_vector[0] - start_vector[0] * plane_vector[2],
    start_vector[0] * plane_vector[1] - start_vector[1] * plane_vector[0],
  };
  const double cross_magnitude = sqrt(
    cross[0] * cross[0] + cross[1] * cross[1] + cross[2] * cross[2]
  );
  const double circumference = 2.0 * 3.14159265358979323846 * start_radius;
  return isfinite(start_radius)
    && isfinite(plane_radius)
    && isfinite(cross_magnitude)
    && isfinite(circumference)
    && start_radius > 0.0
    && plane_radius > 0.0
    && cross_magnitude > start_radius * plane_radius * 0.000001
    && circumference <= static_cast<double>(FLT_MAX)
    && circumference >= static_cast<double>(minimum_path_length);
}

struct OrderedArcGeometry {
  double center[3];
  double axis[3];
  double radius;
  double radians;
};

inline bool calculate_ordered_arc_geometry(
  const float *start,
  const float *middle,
  const float *end,
  OrderedArcGeometry& geometry
) {
  if (start == nullptr || middle == nullptr || end == nullptr) return false;
  for (size_t index = 0; index < 3; ++index) {
    if (!isfinite(start[index]) || !isfinite(middle[index]) || !isfinite(end[index])) {
      return false;
    }
  }

  const double start_to_middle[3] = {
    static_cast<double>(middle[0]) - start[0],
    static_cast<double>(middle[1]) - start[1],
    static_cast<double>(middle[2]) - start[2],
  };
  const double start_to_end[3] = {
    static_cast<double>(end[0]) - start[0],
    static_cast<double>(end[1]) - start[1],
    static_cast<double>(end[2]) - start[2],
  };
  const double normal[3] = {
    start_to_middle[1] * start_to_end[2]
      - start_to_middle[2] * start_to_end[1],
    start_to_middle[2] * start_to_end[0]
      - start_to_middle[0] * start_to_end[2],
    start_to_middle[0] * start_to_end[1]
      - start_to_middle[1] * start_to_end[0],
  };
  const double middle_distance_squared =
    start_to_middle[0] * start_to_middle[0]
    + start_to_middle[1] * start_to_middle[1]
    + start_to_middle[2] * start_to_middle[2];
  const double end_distance_squared =
    start_to_end[0] * start_to_end[0]
    + start_to_end[1] * start_to_end[1]
    + start_to_end[2] * start_to_end[2];
  const double normal_squared = normal[0] * normal[0]
    + normal[1] * normal[1]
    + normal[2] * normal[2];
  if (
    !isfinite(middle_distance_squared)
    || !isfinite(end_distance_squared)
    || !isfinite(normal_squared)
    || middle_distance_squared <= 0.0
    || end_distance_squared <= 0.0
    || normal_squared <= 0.0
  ) {
    return false;
  }

  const double end_cross_normal[3] = {
    start_to_end[1] * normal[2] - start_to_end[2] * normal[1],
    start_to_end[2] * normal[0] - start_to_end[0] * normal[2],
    start_to_end[0] * normal[1] - start_to_end[1] * normal[0],
  };
  const double normal_cross_middle[3] = {
    normal[1] * start_to_middle[2] - normal[2] * start_to_middle[1],
    normal[2] * start_to_middle[0] - normal[0] * start_to_middle[2],
    normal[0] * start_to_middle[1] - normal[1] * start_to_middle[0],
  };
  OrderedArcGeometry staged = {};
  for (size_t index = 0; index < 3; ++index) {
    const double offset = (
      middle_distance_squared * end_cross_normal[index]
      + end_distance_squared * normal_cross_middle[index]
    ) / (2.0 * normal_squared);
    staged.center[index] = static_cast<double>(start[index]) + offset;
  }

  const double start_vector[3] = {
    static_cast<double>(start[0]) - staged.center[0],
    static_cast<double>(start[1]) - staged.center[1],
    static_cast<double>(start[2]) - staged.center[2],
  };
  const double middle_vector[3] = {
    static_cast<double>(middle[0]) - staged.center[0],
    static_cast<double>(middle[1]) - staged.center[1],
    static_cast<double>(middle[2]) - staged.center[2],
  };
  const double end_vector[3] = {
    static_cast<double>(end[0]) - staged.center[0],
    static_cast<double>(end[1]) - staged.center[1],
    static_cast<double>(end[2]) - staged.center[2],
  };
  staged.radius = sqrt(
    start_vector[0] * start_vector[0]
    + start_vector[1] * start_vector[1]
    + start_vector[2] * start_vector[2]
  );
  const double normal_magnitude = sqrt(normal_squared);
  for (size_t index = 0; index < 3; ++index) {
    staged.axis[index] = normal[index] / normal_magnitude;
  }

  const double start_middle_cross[3] = {
    start_vector[1] * middle_vector[2]
      - start_vector[2] * middle_vector[1],
    start_vector[2] * middle_vector[0]
      - start_vector[0] * middle_vector[2],
    start_vector[0] * middle_vector[1]
      - start_vector[1] * middle_vector[0],
  };
  const double middle_end_cross[3] = {
    middle_vector[1] * end_vector[2]
      - middle_vector[2] * end_vector[1],
    middle_vector[2] * end_vector[0]
      - middle_vector[0] * end_vector[2],
    middle_vector[0] * end_vector[1]
      - middle_vector[1] * end_vector[0],
  };
  const double start_middle_sine =
    staged.axis[0] * start_middle_cross[0]
    + staged.axis[1] * start_middle_cross[1]
    + staged.axis[2] * start_middle_cross[2];
  const double middle_end_sine = staged.axis[0] * middle_end_cross[0]
    + staged.axis[1] * middle_end_cross[1]
    + staged.axis[2] * middle_end_cross[2];
  const double start_middle_cosine =
    start_vector[0] * middle_vector[0]
    + start_vector[1] * middle_vector[1]
    + start_vector[2] * middle_vector[2];
  const double middle_end_cosine = middle_vector[0] * end_vector[0]
    + middle_vector[1] * end_vector[1]
    + middle_vector[2] * end_vector[2];
  constexpr double full_turn = 6.28318530717958647692;
  double start_middle_radians = atan2(
    start_middle_sine,
    start_middle_cosine
  );
  double middle_end_radians = atan2(middle_end_sine, middle_end_cosine);
  if (start_middle_radians <= 0.0) start_middle_radians += full_turn;
  if (middle_end_radians <= 0.0) middle_end_radians += full_turn;
  staged.radians = start_middle_radians + middle_end_radians;

  const double maximum_float = static_cast<double>(FLT_MAX);
  if (
    !isfinite(staged.radius)
    || !isfinite(staged.radians)
    || staged.radius <= 0.0
    || staged.radius > maximum_float
    || staged.radians <= 0.0
    || staged.radians > full_turn + 0.000000000001
  ) {
    return false;
  }
  for (size_t index = 0; index < 3; ++index) {
    if (
      !isfinite(staged.center[index])
      || !isfinite(staged.axis[index])
      || fabs(staged.center[index]) > maximum_float
    ) {
      return false;
    }
  }
  geometry = staged;
  return true;
}

inline bool valid_arc_geometry(
  const float *start,
  const float *middle,
  const float *end,
  float minimum_path_length,
  OrderedArcGeometry *geometry = nullptr
) {
  if (
    !isfinite(minimum_path_length)
    || minimum_path_length <= 0.0f
  ) {
    return false;
  }
  OrderedArcGeometry staged = {};
  if (!calculate_ordered_arc_geometry(start, middle, end, staged)) {
    return false;
  }
  const double path_length = staged.radius * staged.radians;
  if (
    !isfinite(path_length)
    || path_length > static_cast<double>(FLT_MAX)
    || path_length < static_cast<double>(minimum_path_length)
  ) {
    return false;
  }
  if (geometry != nullptr) *geometry = staged;
  return true;
}

}  // namespace ar4_protocol

#endif
