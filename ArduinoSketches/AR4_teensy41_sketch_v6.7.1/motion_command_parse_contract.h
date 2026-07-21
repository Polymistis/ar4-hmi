#ifndef AR4_MOTION_COMMAND_PARSE_CONTRACT_H
#define AR4_MOTION_COMMAND_PARSE_CONTRACT_H

#include "numeric_parse_contract.h"

namespace ar4_protocol {

constexpr size_t kMotionJointCount = 6;
constexpr int kMotionAxisCount = 9;

enum class CartesianMotionCommandKind {
  kStandard,
  kLinear,
  kVision,
};

enum class LiveJogCommandKind {
  kCartesian,
  kJoint,
  kTool,
};

struct CartesianMoveCommandFields {
  float pose[kMotionJointCount];
  float auxiliary[3];
  char speed_mode;
  float speed;
  float acceleration;
  float deceleration;
  float ramp;
  float rounding;
  char wrist_config;
  int loop_modes[kMotionJointCount];
  float vision_rotation_degrees;
};

struct ToolJogCommandFields {
  char axis;
  int direction;
  float distance;
  char speed_mode;
  float speed;
  float acceleration;
  float deceleration;
  float ramp;
  char wrist_config;
  int loop_modes[kMotionJointCount];
};

struct JointMoveCommandFields {
  float positions[9];
  char speed_mode;
  float speed;
  float acceleration;
  float deceleration;
  float ramp;
  char wrist_config;
  int loop_modes[kMotionJointCount];
};

struct LiveJogCommandFields {
  int vector;
  char speed_mode;
  float speed;
  float acceleration;
  float deceleration;
  float ramp;
  char wrist_config;
  int loop_modes[kMotionJointCount];
};

inline bool valid_speed_mode(char mode) {
  return mode == 'p' || mode == 's' || mode == 'm';
}

inline bool valid_wrist_config(char config) {
  return config == 'A' || config == 'F' || config == 'N';
}

inline bool valid_tool_jog_axis(char axis) {
  return axis == 'X'
    || axis == 'Y'
    || axis == 'Z'
    || axis == 'R'
    || axis == 'P'
    || axis == 'W';
}

inline bool valid_motion_profile(
  char speed_mode,
  float speed,
  float acceleration,
  float deceleration,
  float ramp
) {
  return valid_speed_mode(speed_mode)
    && isfinite(speed)
    && isfinite(acceleration)
    && isfinite(deceleration)
    && isfinite(ramp)
    && speed > 0.0f
    && (speed_mode != 'p' || speed <= 100.0f)
    && acceleration > 0.0f
    && acceleration <= 100.0f
    && deceleration > 0.0f
    && deceleration < 100.0f
    && acceleration + deceleration <= 100.0f
    && ramp > 0.0f
    && ramp <= 100.0f;
}

inline bool valid_live_jog_vector(int vector, int maximum_axis) {
  if (maximum_axis < 1 || maximum_axis > 9) return false;
  const int axis = vector / 10;
  const int direction = vector % 10;
  return axis >= 1
    && axis <= maximum_axis
    && (direction == 0 || direction == 1)
    && vector == axis * 10 + direction;
}

template <typename Text>
inline bool parse_cartesian_move_command_impl(
  const Text &command,
  CartesianMotionCommandKind kind,
  CartesianMoveCommandFields &output
) {
  if (
    kind != CartesianMotionCommandKind::kStandard
    && kind != CartesianMotionCommandKind::kLinear
    && kind != CartesianMotionCommandKind::kVision
  ) {
    return false;
  }
  const bool requires_rounding =
    kind == CartesianMotionCommandKind::kLinear;
  const bool requires_vision_rotation =
    kind == CartesianMotionCommandKind::kVision;
  const int x_start = command.indexOf('X');
  const int y_start = command.indexOf('Y');
  const int z_start = command.indexOf('Z');
  const int rz_start = command.indexOf("Rz");
  const int ry_start = command.indexOf("Ry");
  const int rx_start = command.indexOf("Rx");
  const int j7_start = command.indexOf("J7");
  const int j8_start = command.indexOf("J8");
  const int j9_start = command.indexOf("J9");
  const int speed_start = command.indexOf('S');
  const int acceleration_start = command.indexOf("Ac");
  const int deceleration_start = command.indexOf("Dc");
  const int ramp_start = command.indexOf("Rm");
  const int rounding_start = command.indexOf("Rnd");
  const int wrist_start = command.indexOf('W');
  const int vision_start = command.indexOf("Vr");
  const int loop_mode_start = command.indexOf("Lm");

  const int fixed_markers[] = {
    x_start,
    y_start,
    z_start,
    rz_start,
    ry_start,
    rx_start,
    j7_start,
    j8_start,
    j9_start,
    speed_start,
    acceleration_start,
    deceleration_start,
    ramp_start,
  };
  if (
    x_start != 0
    || !marker_positions_are_ordered(command.length(), fixed_markers)
    || (requires_rounding ? rounding_start < 0 : rounding_start >= 0)
    || wrist_start <= ramp_start
    || loop_mode_start < 0
    || loop_mode_start + 2 + static_cast<int>(kMotionJointCount)
      != static_cast<int>(command.length())
  ) {
    return false;
  }

  int ramp_end = wrist_start;
  float rounding = 0.0f;
  if (rounding_start >= 0) {
    if (rounding_start <= ramp_start || wrist_start <= rounding_start) {
      return false;
    }
    ramp_end = rounding_start;
    if (!parse_float_span(command, rounding_start + 3, wrist_start, rounding)) {
      return false;
    }
  }

  int wrist_end = loop_mode_start;
  float vision_rotation = 0.0f;
  if (requires_vision_rotation) {
    if (
      vision_start != wrist_start + 2
      || loop_mode_start <= vision_start + 2
    ) {
      return false;
    }
    wrist_end = vision_start;
    if (!parse_float_span(
        command,
        vision_start + 2,
        loop_mode_start,
        vision_rotation
    )) {
      return false;
    }
  } else if (vision_start >= 0 || loop_mode_start != wrist_start + 2) {
    return false;
  }

  if (wrist_end != wrist_start + 2) return false;
  const char speed_mode = command.charAt(speed_start + 1);
  const char wrist_config = command.charAt(wrist_start + 1);
  if (!valid_speed_mode(speed_mode) || !valid_wrist_config(wrist_config)) {
    return false;
  }

  const int begins[] = {
    x_start + 1,
    y_start + 1,
    z_start + 1,
    rz_start + 2,
    ry_start + 2,
    rx_start + 2,
    j7_start + 2,
    j8_start + 2,
    j9_start + 2,
    speed_start + 2,
    acceleration_start + 2,
    deceleration_start + 2,
    ramp_start + 2,
  };
  const int ends[] = {
    y_start,
    z_start,
    rz_start,
    ry_start,
    rx_start,
    j7_start,
    j8_start,
    j9_start,
    speed_start,
    acceleration_start,
    deceleration_start,
    ramp_start,
    ramp_end,
  };
  float parsed[13];
  int loop_modes[kMotionJointCount];
  if (
    !parse_float_spans(command, begins, ends, parsed)
    || !parse_binary_digit_span(
      command,
      loop_mode_start + 2,
      static_cast<int>(command.length()),
      loop_modes
    )
  ) {
    return false;
  }
  if (
    !valid_motion_profile(
      speed_mode,
      parsed[9],
      parsed[10],
      parsed[11],
      parsed[12]
    )
    || rounding < 0.0f
  ) {
    return false;
  }

  CartesianMoveCommandFields staged = {};
  for (size_t index = 0; index < kMotionJointCount; ++index) {
    staged.pose[index] = parsed[index];
    staged.loop_modes[index] = loop_modes[index];
  }
  for (size_t index = 0; index < 3; ++index) {
    staged.auxiliary[index] = parsed[index + kMotionJointCount];
  }
  staged.speed_mode = speed_mode;
  staged.speed = parsed[9];
  staged.acceleration = parsed[10];
  staged.deceleration = parsed[11];
  staged.ramp = parsed[12];
  staged.rounding = rounding;
  staged.wrist_config = wrist_config;
  staged.vision_rotation_degrees = vision_rotation;
  output = staged;
  return true;
}

template <typename Text>
inline bool parse_cartesian_move_command(
  const Text &command,
  CartesianMoveCommandFields &output
) {
  return parse_cartesian_move_command_impl(
    command,
    CartesianMotionCommandKind::kStandard,
    output
  );
}

template <typename Text>
inline bool parse_vision_move_command(
  const Text &command,
  CartesianMoveCommandFields &output
) {
  return parse_cartesian_move_command_impl(
    command,
    CartesianMotionCommandKind::kVision,
    output
  );
}

template <typename Text>
inline bool parse_linear_move_command(
  const Text &command,
  CartesianMoveCommandFields &output
) {
  const int disable_wrist_start = command.indexOf('Q');
  if (
    disable_wrist_start <= 0
    || disable_wrist_start + 2 != static_cast<int>(command.length())
    || command.charAt(disable_wrist_start + 1) != '0'
  ) {
    return false;
  }
  const Text motion = command.substring(0, disable_wrist_start);
  if (!parse_cartesian_move_command_impl(
      motion,
      CartesianMotionCommandKind::kLinear,
      output
  )) {
    return false;
  }
  return true;
}

template <typename Text>
inline bool parse_tool_jog_command(
  const Text &command,
  ToolJogCommandFields &output
) {
  const int speed_start = command.indexOf('S');
  const int acceleration_start = command.indexOf('G');
  const int deceleration_start = command.indexOf('H');
  const int ramp_start = command.indexOf('I');
  const int loop_mode_start = command.indexOf("Lm");
  const int wrist_start = loop_mode_start > 0
    ? command.lastIndexOf('W', loop_mode_start - 1)
    : -1;
  const int markers[] = {
    speed_start,
    acceleration_start,
    deceleration_start,
    ramp_start,
    wrist_start,
    loop_mode_start,
  };
  if (
    command.length() < 3
    || !marker_positions_are_ordered(command.length(), markers)
    || wrist_start + 2 != loop_mode_start
    || loop_mode_start + 2 + static_cast<int>(kMotionJointCount)
      != static_cast<int>(command.length())
  ) {
    return false;
  }

  const char axis = command.charAt(0);
  const char direction_text = command.charAt(1);
  const char speed_mode = command.charAt(speed_start + 1);
  const char wrist_config = command.charAt(wrist_start + 1);
  if (
    !valid_tool_jog_axis(axis)
    || (direction_text != '0' && direction_text != '1')
    || !valid_speed_mode(speed_mode)
    || !valid_wrist_config(wrist_config)
  ) {
    return false;
  }

  const int begins[] = {
    2,
    speed_start + 2,
    acceleration_start + 1,
    deceleration_start + 1,
    ramp_start + 1,
  };
  const int ends[] = {
    speed_start,
    acceleration_start,
    deceleration_start,
    ramp_start,
    wrist_start,
  };
  float parsed[5];
  int loop_modes[kMotionJointCount];
  if (
    !parse_float_spans(command, begins, ends, parsed)
    || !parse_binary_digit_span(
      command,
      loop_mode_start + 2,
      static_cast<int>(command.length()),
      loop_modes
    )
  ) {
    return false;
  }
  if (
    parsed[0] < 0.0f
    || !valid_motion_profile(
      speed_mode,
      parsed[1],
      parsed[2],
      parsed[3],
      parsed[4]
    )
  ) {
    return false;
  }

  ToolJogCommandFields staged = {};
  staged.axis = axis;
  staged.direction = direction_text - '0';
  staged.distance = parsed[0];
  staged.speed_mode = speed_mode;
  staged.speed = parsed[1];
  staged.acceleration = parsed[2];
  staged.deceleration = parsed[3];
  staged.ramp = parsed[4];
  staged.wrist_config = wrist_config;
  for (size_t index = 0; index < kMotionJointCount; ++index) {
    staged.loop_modes[index] = loop_modes[index];
  }
  output = staged;
  return true;
}

template <typename Text>
inline bool parse_joint_move_command(
  const Text &command,
  JointMoveCommandFields &output
) {
  const int markers[] = {
    command.indexOf('A'),
    command.indexOf('B'),
    command.indexOf('C'),
    command.indexOf('D'),
    command.indexOf('E'),
    command.indexOf('F'),
    command.indexOf("J7"),
    command.indexOf("J8"),
    command.indexOf("J9"),
    command.indexOf('S'),
    command.indexOf("Ac"),
    command.indexOf("Dc"),
    command.indexOf("Rm"),
    command.indexOf('W'),
    command.indexOf("Lm"),
  };
  if (
    !marker_positions_are_ordered_from(command.length(), markers, 0)
    || markers[13] + 2 != markers[14]
    || markers[14] + 2 + static_cast<int>(kMotionJointCount)
      != static_cast<int>(command.length())
  ) {
    return false;
  }

  const char speed_mode = command.charAt(markers[9] + 1);
  const char wrist_config = command.charAt(markers[13] + 1);
  if (!valid_speed_mode(speed_mode) || !valid_wrist_config(wrist_config)) {
    return false;
  }
  const int begins[] = {
    markers[0] + 1,
    markers[1] + 1,
    markers[2] + 1,
    markers[3] + 1,
    markers[4] + 1,
    markers[5] + 1,
    markers[6] + 2,
    markers[7] + 2,
    markers[8] + 2,
    markers[9] + 2,
    markers[10] + 2,
    markers[11] + 2,
    markers[12] + 2,
  };
  const int ends[] = {
    markers[1],
    markers[2],
    markers[3],
    markers[4],
    markers[5],
    markers[6],
    markers[7],
    markers[8],
    markers[9],
    markers[10],
    markers[11],
    markers[12],
    markers[13],
  };
  float parsed[13];
  int loop_modes[kMotionJointCount];
  if (
    !parse_float_spans(command, begins, ends, parsed)
    || !parse_binary_digit_span(
      command,
      markers[14] + 2,
      static_cast<int>(command.length()),
      loop_modes
    )
    || !valid_motion_profile(
      speed_mode,
      parsed[9],
      parsed[10],
      parsed[11],
      parsed[12]
    )
  ) {
    return false;
  }

  JointMoveCommandFields staged = {};
  for (size_t index = 0; index < 9; ++index) staged.positions[index] = parsed[index];
  staged.speed_mode = speed_mode;
  staged.speed = parsed[9];
  staged.acceleration = parsed[10];
  staged.deceleration = parsed[11];
  staged.ramp = parsed[12];
  staged.wrist_config = wrist_config;
  for (size_t index = 0; index < kMotionJointCount; ++index) {
    staged.loop_modes[index] = loop_modes[index];
  }
  output = staged;
  return true;
}

template <typename Text>
inline bool parse_live_jog_command(
  const Text &command,
  LiveJogCommandKind kind,
  LiveJogCommandFields &output
) {
  int maximum_axis = static_cast<int>(kMotionJointCount);
  if (kind == LiveJogCommandKind::kJoint) {
    maximum_axis = kMotionAxisCount;
  } else if (
    kind != LiveJogCommandKind::kCartesian
    && kind != LiveJogCommandKind::kTool
  ) {
    return false;
  }
  const int markers[] = {
    command.indexOf('V'),
    command.indexOf('S'),
    command.indexOf("Ac"),
    command.indexOf("Dc"),
    command.indexOf("Rm"),
    command.indexOf('W'),
    command.indexOf("Lm"),
  };
  if (
    !marker_positions_are_ordered_from(command.length(), markers, 0)
    || markers[5] + 2 != markers[6]
    || markers[6] + 2 + static_cast<int>(kMotionJointCount)
      != static_cast<int>(command.length())
  ) {
    return false;
  }

  const char speed_mode = command.charAt(markers[1] + 1);
  const char wrist_config = command.charAt(markers[5] + 1);
  if (
    speed_mode != 'p'
    || !valid_wrist_config(wrist_config)
    || (kind == LiveJogCommandKind::kJoint && wrist_config != 'A')
  ) {
    return false;
  }
  const int begins[] = {
    markers[1] + 2,
    markers[2] + 2,
    markers[3] + 2,
    markers[4] + 2,
  };
  const int ends[] = {
    markers[2],
    markers[3],
    markers[4],
    markers[5],
  };
  int vector = 0;
  float parsed[4];
  int loop_modes[kMotionJointCount];
  if (
    !parse_int_span(command, markers[0] + 1, markers[1], vector)
    || !parse_float_spans(command, begins, ends, parsed)
    || !parse_binary_digit_span(
      command,
      markers[6] + 2,
      static_cast<int>(command.length()),
      loop_modes
    )
  ) {
    return false;
  }
  if (
    !valid_live_jog_vector(vector, maximum_axis)
    || !valid_motion_profile(
      speed_mode,
      parsed[0],
      parsed[1],
      parsed[2],
      parsed[3]
    )
  ) {
    return false;
  }

  LiveJogCommandFields staged = {};
  staged.vector = vector;
  staged.speed_mode = speed_mode;
  staged.speed = parsed[0];
  staged.acceleration = parsed[1];
  staged.deceleration = parsed[2];
  staged.ramp = parsed[3];
  staged.wrist_config = wrist_config;
  for (size_t index = 0; index < kMotionJointCount; ++index) {
    staged.loop_modes[index] = loop_modes[index];
  }
  output = staged;
  return true;
}

}  // namespace ar4_protocol

#endif
