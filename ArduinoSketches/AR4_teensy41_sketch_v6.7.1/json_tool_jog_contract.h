#ifndef AR4_JSON_TOOL_JOG_CONTRACT_H
#define AR4_JSON_TOOL_JOG_CONTRACT_H

#include <stddef.h>
#include <stdint.h>

#include "json_cartesian_motion_contract.h"
#include "tool_jog_contract.h"

namespace ar4_protocol {

constexpr size_t kJsonToolJogPrimaryAxisCount = 6;
constexpr size_t kJsonToolJogTerminalPayloadReservationBytes =
  kJsonCartesianMotionTerminalPayloadReservationBytes;

enum class JsonToolJogAxis : uint8_t {
  kX,
  kY,
  kZ,
  kRx,
  kRy,
  kRz,
};

enum class JsonToolJogDirection : uint8_t {
  kNegative,
  kPositive,
};

struct JsonMainToolJogParameters {
  JsonToolJogAxis axis;
  JsonToolJogDirection direction;
  float distance;
  JsonCartesianMotionSpeedMode speed_mode;
  float speed_value;
  float acceleration_percent;
  float deceleration_percent;
  float ramp_percent;
  JsonCartesianMotionWristConfiguration wrist_configuration;
  bool loop_modes[kJsonToolJogPrimaryAxisCount];
};

using JsonMainToolJogOutcome = JsonMainMoveCartesianOutcome;
using JsonMainToolJogExecutionResult = JsonMainMoveCartesianExecutionResult;

inline bool json_main_tool_jog_execution_result_valid(
  const JsonMainToolJogExecutionResult &result
) {
  return json_cartesian_motion_detail::execution_result_valid_for_command(
    "jog_tool",
    result
  );
}

namespace json_tool_jog_detail {

inline bool axis_valid(JsonToolJogAxis axis) {
  return axis == JsonToolJogAxis::kX
    || axis == JsonToolJogAxis::kY
    || axis == JsonToolJogAxis::kZ
    || axis == JsonToolJogAxis::kRx
    || axis == JsonToolJogAxis::kRy
    || axis == JsonToolJogAxis::kRz;
}

inline bool direction_valid(JsonToolJogDirection direction) {
  return direction == JsonToolJogDirection::kNegative
    || direction == JsonToolJogDirection::kPositive;
}

inline bool rotational_axis(JsonToolJogAxis axis) {
  return axis == JsonToolJogAxis::kRx
    || axis == JsonToolJogAxis::kRy
    || axis == JsonToolJogAxis::kRz;
}

inline char legacy_axis(JsonToolJogAxis axis) {
  switch (axis) {
    case JsonToolJogAxis::kX: return 'X';
    case JsonToolJogAxis::kY: return 'Y';
    case JsonToolJogAxis::kZ: return 'Z';
    case JsonToolJogAxis::kRx: return 'W';
    case JsonToolJogAxis::kRy: return 'P';
    case JsonToolJogAxis::kRz: return 'R';
  }
  return '\0';
}

inline bool parameters_valid(const JsonMainToolJogParameters &params) {
  if (
    !axis_valid(params.axis)
    || !direction_valid(params.direction)
    || !json_joint_motion_detail::controller_value_representable(
      params.distance
    )
    || params.distance < 0.0f
    || params.speed_mode
      == JsonCartesianMotionSpeedMode::kMillimetersPerSecond
  ) {
    return false;
  }
  if (rotational_axis(params.axis)) {
    float radians = 0.0f;
    if (!degrees_to_radians(params.distance, radians)) return false;
  }
  JsonMainMoveCartesianParameters profile = {};
  profile.speed_mode = params.speed_mode;
  profile.speed_value = params.speed_value;
  profile.acceleration_percent = params.acceleration_percent;
  profile.deceleration_percent = params.deceleration_percent;
  profile.ramp_percent = params.ramp_percent;
  profile.wrist_configuration = params.wrist_configuration;
  for (size_t axis = 0; axis < kJsonToolJogPrimaryAxisCount; ++axis) {
    profile.loop_modes[axis] = params.loop_modes[axis];
  }
  return json_cartesian_motion_detail::parameters_valid(profile);
}

}  // namespace json_tool_jog_detail

inline bool decode_json_tool_jog_offset(
  const JsonMainToolJogParameters &params,
  int &frame_index,
  float &frame_offset
) {
  if (!json_tool_jog_detail::parameters_valid(params)) return false;
  return decode_discrete_tool_offset(
    json_tool_jog_detail::legacy_axis(params.axis),
    params.direction == JsonToolJogDirection::kNegative ? '1' : '0',
    params.distance,
    frame_index,
    frame_offset
  );
}

inline bool stage_json_tool_frame_offset(
  float current_value,
  float offset,
  float &staged_value
) {
  if (
    !json_joint_motion_detail::controller_value_representable(current_value)
    || !json_joint_motion_detail::controller_value_representable(offset)
  ) {
    return false;
  }
  const double combined = static_cast<double>(current_value)
    + static_cast<double>(offset);
  if (!json_joint_motion_detail::controller_value_representable(combined)) {
    return false;
  }
  const float converted = static_cast<float>(combined);
  if (offset != 0.0f && converted == current_value) return false;
  staged_value = converted;
  return true;
}

inline bool extract_main_tool_jog_parameters(
  ArduinoJson::JsonObjectConst params,
  JsonMainToolJogParameters &output
) {
  if (params.size() != 10) return false;
  JsonMainToolJogParameters staged = {};
  bool axis_present = false;
  bool direction_present = false;
  bool distance_present = false;
  bool speed_mode_present = false;
  bool speed_value_present = false;
  bool acceleration_present = false;
  bool deceleration_present = false;
  bool ramp_present = false;
  bool wrist_present = false;
  bool loop_modes_present = false;
  for (ArduinoJson::JsonPairConst pair : params) {
    const ArduinoJson::JsonString key = pair.key();
    if (json_joint_motion_detail::string_equals(key, "axis")) {
      if (axis_present) return false;
      const ArduinoJson::JsonString value =
        pair.value().as<ArduinoJson::JsonString>();
      if (json_joint_motion_detail::string_equals(value, "x")) {
        staged.axis = JsonToolJogAxis::kX;
      } else if (json_joint_motion_detail::string_equals(value, "y")) {
        staged.axis = JsonToolJogAxis::kY;
      } else if (json_joint_motion_detail::string_equals(value, "z")) {
        staged.axis = JsonToolJogAxis::kZ;
      } else if (json_joint_motion_detail::string_equals(value, "rx")) {
        staged.axis = JsonToolJogAxis::kRx;
      } else if (json_joint_motion_detail::string_equals(value, "ry")) {
        staged.axis = JsonToolJogAxis::kRy;
      } else if (json_joint_motion_detail::string_equals(value, "rz")) {
        staged.axis = JsonToolJogAxis::kRz;
      } else {
        return false;
      }
      axis_present = true;
    } else if (json_joint_motion_detail::string_equals(key, "direction")) {
      if (direction_present) return false;
      const ArduinoJson::JsonString value =
        pair.value().as<ArduinoJson::JsonString>();
      if (json_joint_motion_detail::string_equals(value, "negative")) {
        staged.direction = JsonToolJogDirection::kNegative;
      } else if (json_joint_motion_detail::string_equals(value, "positive")) {
        staged.direction = JsonToolJogDirection::kPositive;
      } else {
        return false;
      }
      direction_present = true;
    } else if (json_joint_motion_detail::string_equals(key, "distance")) {
      if (
        distance_present
        || !json_joint_motion_detail::extract_controller_float(
          pair.value(),
          staged.distance
        )
      ) return false;
      distance_present = true;
    } else if (json_joint_motion_detail::string_equals(key, "speed_mode")) {
      if (speed_mode_present) return false;
      const ArduinoJson::JsonString value =
        pair.value().as<ArduinoJson::JsonString>();
      if (json_joint_motion_detail::string_equals(value, "percent")) {
        staged.speed_mode = JsonCartesianMotionSpeedMode::kPercent;
      } else if (json_joint_motion_detail::string_equals(value, "seconds")) {
        staged.speed_mode = JsonCartesianMotionSpeedMode::kSeconds;
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
          kJsonToolJogPrimaryAxisCount
        )
      ) return false;
      loop_modes_present = true;
    } else {
      return false;
    }
  }
  if (
    !axis_present
    || !direction_present
    || !distance_present
    || !speed_mode_present
    || !speed_value_present
    || !acceleration_present
    || !deceleration_present
    || !ramp_present
    || !wrist_present
    || !loop_modes_present
    || !json_tool_jog_detail::parameters_valid(staged)
  ) {
    return false;
  }
  output = staged;
  return true;
}

inline bool build_main_json_tool_jog_response(
  uint32_t request_id,
  const JsonMainToolJogExecutionResult &result,
  size_t maximum_payload_bytes,
  char *output,
  size_t output_capacity
) {
  return build_main_json_cartesian_motion_response(
    "jog_tool",
    request_id,
    result,
    maximum_payload_bytes,
    output,
    output_capacity
  );
}

}  // namespace ar4_protocol

#endif
