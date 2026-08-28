#ifndef AR4_JSON_DIRECT_COMMAND_RUNTIME_H
#define AR4_JSON_DIRECT_COMMAND_RUNTIME_H

namespace ar4_json_direct_runtime {

using ar4_protocol::JsonMainDirectParameters;
using ar4_protocol::JsonMainDirectResponseStatus;
using ar4_protocol::JsonMainMoveCartesianExecutionResult;
using ar4_protocol::JsonMainMoveCartesianOutcome;

inline JsonMainDirectResponseStatus response_status(
  JsonMainMoveCartesianOutcome outcome
) {
  switch (outcome) {
    case JsonMainMoveCartesianOutcome::kCompleted:
      return JsonMainDirectResponseStatus::kCompleted;
    case JsonMainMoveCartesianOutcome::kKinematicsUnreachable:
    case JsonMainMoveCartesianOutcome::kJointLimitViolation:
    case JsonMainMoveCartesianOutcome::kPositionNotRepresentable:
      return JsonMainDirectResponseStatus::kRejected;
    case JsonMainMoveCartesianOutcome::kEmergencyStop:
      return JsonMainDirectResponseStatus::kCancelled;
    case JsonMainMoveCartesianOutcome::kPositionUnavailable:
    case JsonMainMoveCartesianOutcome::kMotionExecutionFailed:
    case JsonMainMoveCartesianOutcome::kEncoderCollision:
    case JsonMainMoveCartesianOutcome::kEncoderStateUnavailable:
      return JsonMainDirectResponseStatus::kFailed;
    case JsonMainMoveCartesianOutcome::kInvalid:
      return JsonMainDirectResponseStatus::kInvalid;
  }
  return JsonMainDirectResponseStatus::kInvalid;
}

inline bool build_position_response(
  const char *command,
  uint32_t request_id,
  const ar4_protocol::JsonMainPositionSnapshot &position,
  size_t maximum_payload_bytes,
  char *output,
  size_t output_capacity
) {
  if (
    command == nullptr || request_id == 0 || output == nullptr
    || output_capacity == 0 || maximum_payload_bytes == 0
    || maximum_payload_bytes > ar4_protocol::kJsonProtocolMaximumPayloadBytes
  ) return false;
  const size_t capacity = output_capacity < maximum_payload_bytes + 1
    ? output_capacity : maximum_payload_bytes + 1;
  size_t index = 0;
  output[0] = '\0';
  const bool built = ar4_protocol::append_json_text(
      "{\"cmd\":\"", output, capacity, index
    )
    && ar4_protocol::append_json_text(command, output, capacity, index)
    && ar4_protocol::append_json_text(
      "\",\"id\":", output, capacity, index
    )
    && ar4_protocol::append_json_uint32(
      request_id, output, capacity, index
    )
    && ar4_protocol::append_json_text(
      ",\"result\":", output, capacity, index
    )
    && ar4_protocol::append_main_json_position_snapshot(
      position, output, capacity, index
    )
    && ar4_protocol::append_json_text(
      ",\"status\":\"completed\",\"type\":\"response\",\"v\":1}",
      output, capacity, index
    );
  if (!built) output[0] = '\0';
  return built;
}

inline bool build_integer_response(
  const char *command,
  uint32_t request_id,
  const char *field,
  int32_t value,
  size_t maximum_payload_bytes,
  char *output,
  size_t output_capacity
) {
  if (
    command == nullptr || field == nullptr || request_id == 0
    || output == nullptr || output_capacity == 0
    || maximum_payload_bytes == 0
    || maximum_payload_bytes > ar4_protocol::kJsonProtocolMaximumPayloadBytes
  ) return false;
  const size_t capacity = output_capacity < maximum_payload_bytes + 1
    ? output_capacity : maximum_payload_bytes + 1;
  size_t index = 0;
  output[0] = '\0';
  const bool built = ar4_protocol::append_json_text(
      "{\"cmd\":\"", output, capacity, index
    )
    && ar4_protocol::append_json_text(command, output, capacity, index)
    && ar4_protocol::append_json_text(
      "\",\"id\":", output, capacity, index
    )
    && ar4_protocol::append_json_uint32(
      request_id, output, capacity, index
    )
    && ar4_protocol::append_json_text(
      ",\"result\":{\"", output, capacity, index
    )
    && ar4_protocol::append_json_text(field, output, capacity, index)
    && ar4_protocol::append_json_text("\":", output, capacity, index)
    && ar4_protocol::append_json_int32(value, output, capacity, index)
    && ar4_protocol::append_json_text(
      "},\"status\":\"completed\",\"type\":\"response\",\"v\":1}",
      output, capacity, index
    );
  if (!built) output[0] = '\0';
  return built;
}

inline bool build_bool_response(
  const char *command, uint32_t request_id, const char *field, bool value,
  size_t maximum_payload_bytes, char *output, size_t output_capacity
) {
  if (command == nullptr || field == nullptr || output == nullptr) return false;
  const size_t capacity = output_capacity < maximum_payload_bytes + 1
    ? output_capacity : maximum_payload_bytes + 1;
  size_t index = 0;
  output[0] = '\0';
  const bool built = request_id != 0 && ar4_protocol::append_json_text(
      "{\"cmd\":\"", output, capacity, index)
    && ar4_protocol::append_json_text(command, output, capacity, index)
    && ar4_protocol::append_json_text("\",\"id\":", output, capacity, index)
    && ar4_protocol::append_json_uint32(request_id, output, capacity, index)
    && ar4_protocol::append_json_text(",\"result\":{\"", output, capacity, index)
    && ar4_protocol::append_json_text(field, output, capacity, index)
    && ar4_protocol::append_json_text(
      value ? "\":true}" : "\":false}", output, capacity, index)
    && ar4_protocol::append_json_text(
      ",\"status\":\"completed\",\"type\":\"response\",\"v\":1}",
      output, capacity, index);
  if (!built) output[0] = '\0';
  return built;
}

inline bool build_error_response(
  const char *command,
  uint32_t request_id,
  ar4_protocol::JsonErrorResponseStatus status,
  const char *code,
  const char *message,
  size_t maximum_payload_bytes,
  char *output,
  size_t output_capacity
) {
  return ar4_protocol::build_main_json_error_response(
    request_id,
    command,
    status,
    code,
    message,
    nullptr,
    maximum_payload_bytes,
    output,
    output_capacity
  );
}

inline JsonMainDirectResponseStatus finish_motion_response(
  const char *command,
  uint32_t request_id,
  const JsonMainMoveCartesianExecutionResult &result,
  bool flat_completed_position,
  size_t maximum_payload_bytes,
  char *output,
  size_t output_capacity
) {
  const JsonMainDirectResponseStatus status = response_status(result.outcome);
  const bool built = flat_completed_position
      && result.outcome == JsonMainMoveCartesianOutcome::kCompleted
    ? build_position_response(
        command, request_id, result.position, maximum_payload_bytes,
        output, output_capacity
      )
    : ar4_protocol::build_main_json_cartesian_motion_response(
        command, request_id, result, maximum_payload_bytes,
        output, output_capacity
      );
  return built ? status : JsonMainDirectResponseStatus::kInvalid;
}

inline char wrist_code(
  ar4_protocol::JsonCartesianMotionWristConfiguration configuration
) {
  switch (configuration) {
    case ar4_protocol::JsonCartesianMotionWristConfiguration::kAutomatic:
      return 'A';
    case ar4_protocol::JsonCartesianMotionWristConfiguration::kNear:
      return 'N';
    case ar4_protocol::JsonCartesianMotionWristConfiguration::kFar:
      return 'F';
  }
  return '\0';
}

inline String speed_code(
  ar4_protocol::JsonCartesianMotionSpeedMode mode
) {
  switch (mode) {
    case ar4_protocol::JsonCartesianMotionSpeedMode::kPercent:
      return "p";
    case ar4_protocol::JsonCartesianMotionSpeedMode::kSeconds:
      return "s";
    case ar4_protocol::JsonCartesianMotionSpeedMode::kMillimetersPerSecond:
      return "m";
  }
  return "";
}

inline void load_pose(
  const ar4_protocol::JsonMainMoveCartesianParameters &params,
  float (&pose)[ROBOT_nDOFs]
) {
  pose[0] = params.translation_millimeters[0];
  pose[1] = params.translation_millimeters[1];
  pose[2] = params.translation_millimeters[2];
  pose[3] = params.orientation_degrees[2];
  pose[4] = params.orientation_degrees[1];
  pose[5] = params.orientation_degrees[0];
}

inline bool step_move_from_targets(
  const int (&current)[numJoints],
  const int (&targets)[numJoints],
  const int (&limits)[numJoints],
  int (&steps)[numJoints],
  int (&directions)[numJoints],
  bool (&axes)[numJoints]
) {
  bool valid = true;
  for (int axis = 0; axis < numJoints; ++axis) {
    if (
      future_step_is_outside_limit(current[axis], limits[axis])
      || future_step_is_outside_limit(targets[axis], limits[axis])
    ) {
      axes[axis] = true;
      valid = false;
      continue;
    }
    const int64_t difference = static_cast<int64_t>(current[axis])
      - static_cast<int64_t>(targets[axis]);
    const uint64_t magnitude = difference < 0
      ? static_cast<uint64_t>(-difference)
      : static_cast<uint64_t>(difference);
    if (magnitude > static_cast<uint64_t>(
        std::numeric_limits<int>::max()
    )) {
      axes[axis] = true;
      valid = false;
      continue;
    }
    steps[axis] = static_cast<int>(magnitude);
    directions[axis] = difference <= 0 ? 1 : 0;
  }
  return valid;
}

inline JsonMainDirectResponseStatus execute_vision(
  const char *command,
  const JsonMainDirectParameters &parameters,
  uint32_t request_id,
  size_t maximum_payload_bytes,
  char *output,
  size_t output_capacity
) {
  float radians = 0.0f;
  JsonMainMoveCartesianExecutionResult result = {};
  {
    MotionToolFrameTransaction tool_frame;
    if (
      !ar4_protocol::degrees_to_radians(
        parameters.motion_modifier,
        radians
      )
      || !tool_frame.apply_offset(3, -radians)
    ) {
      result.outcome =
        JsonMainMoveCartesianOutcome::kPositionNotRepresentable;
      result.axes[3] = true;
    } else if (!execute_json_move_cartesian_with_timing(
        parameters.motion,
        result,
        nullptr,
        false,
        false
    )) {
      return JsonMainDirectResponseStatus::kInvalid;
    }
  }
  if (
    result.outcome == JsonMainMoveCartesianOutcome::kCompleted
    || result.outcome == JsonMainMoveCartesianOutcome::kEmergencyStop
    || result.outcome == JsonMainMoveCartesianOutcome::kMotionExecutionFailed
    || result.outcome == JsonMainMoveCartesianOutcome::kEncoderCollision
    || result.outcome
      == JsonMainMoveCartesianOutcome::kEncoderStateUnavailable
  ) {
    if (!capture_json_motion_position(result)) {
      result = {};
      result.outcome = JsonMainMoveCartesianOutcome::kPositionUnavailable;
    }
  }
  return finish_motion_response(
    command,
    request_id,
    result,
    false,
    maximum_payload_bytes,
    output,
    output_capacity
  );
}

inline bool prepare_linear_waypoint(
  const float (&start)[ROBOT_nDOFs],
  const float (&vector)[ROBOT_nDOFs],
  int waypoint,
  int waypoint_count,
  char wrist,
  const int (&external_start)[3],
  const int (&external_target)[3],
  const int (&limits)[numJoints],
  int (&target_steps)[numJoints],
  bool (&axes)[numJoints],
  JsonMainMoveCartesianOutcome &failure
) {
  const float fraction = static_cast<float>(waypoint)
    / static_cast<float>(waypoint_count);
  for (int axis = 0; axis < ROBOT_nDOFs; ++axis) {
    xyzuvw_In[axis] = start[axis] + vector[axis] * fraction;
  }
  SolveInverseKinematics(wrist);
  if (KinematicError != 0) {
    failure = JsonMainMoveCartesianOutcome::kKinematicsUnreachable;
    return false;
  }
  int primary[ROBOT_nDOFs] = {};
  if (!primary_inverse_solution_to_future_steps(primary)) {
    for (int axis = 0; axis < ROBOT_nDOFs; ++axis) axes[axis] = true;
    failure = JsonMainMoveCartesianOutcome::kPositionNotRepresentable;
    return false;
  }
  for (int axis = 0; axis < ROBOT_nDOFs; ++axis) {
    target_steps[axis] = primary[axis];
  }
  for (int axis = 0; axis < 3; ++axis) {
    if (!ar4_protocol::interpolated_step_target(
        external_start[axis], external_target[axis],
        waypoint, waypoint_count, limits[axis + ROBOT_nDOFs],
        target_steps[axis + ROBOT_nDOFs]
    )) {
      axes[axis + ROBOT_nDOFs] = true;
      failure = JsonMainMoveCartesianOutcome::kPositionNotRepresentable;
      return false;
    }
  }
  for (int axis = 0; axis < numJoints; ++axis) {
    if (future_step_is_outside_limit(target_steps[axis], limits[axis])) {
      axes[axis] = true;
      failure = JsonMainMoveCartesianOutcome::kJointLimitViolation;
    }
  }
  return failure == JsonMainMoveCartesianOutcome::kInvalid;
}

inline bool linear_delay_profile(
  const ar4_protocol::JsonMainMoveCartesianParameters &params,
  float line_distance,
  int high_step,
  int waypoint_count,
  float &minimum_delay,
  float &start_delay,
  float &end_delay,
  float &acceleration_increment,
  float &deceleration_increment
) {
  if (high_step < 1 || waypoint_count < 1 || minSpeedDelay <= 0.0f) {
    return false;
  }
  const float acceleration_steps =
    high_step * params.acceleration_percent / 100.0f;
  const float normal_steps = high_step
    * (100.0f - params.acceleration_percent
       - params.deceleration_percent) / 100.0f;
  const float deceleration_steps =
    high_step * params.deceleration_percent / 100.0f;
  if (params.speed_mode
      == ar4_protocol::JsonCartesianMotionSpeedMode::kPercent) {
    minimum_delay = minSpeedDelay / (params.speed_value / 100.0f);
  } else {
    const float duration = params.speed_mode
        == ar4_protocol::JsonCartesianMotionSpeedMode::kSeconds
      ? params.speed_value * 1000000.0f * 1.2f
      : (line_distance / params.speed_value) * 1000000.0f * 1.2f;
    const float zero_gap = duration / static_cast<float>(high_step);
    const float acceleration_step_increment =
      (zero_gap * (100.0f / params.ramp_percent)) / acceleration_steps;
    const float deceleration_step_increment =
      (zero_gap * (100.0f / params.ramp_percent)) / deceleration_steps;
    const float zero_time =
      acceleration_steps * zero_gap
      + (acceleration_steps - 9.0f)
        * acceleration_steps * acceleration_step_increment / 2.0f
      + normal_steps * zero_gap
      + deceleration_steps * zero_gap
      + (deceleration_steps - 9.0f)
        * deceleration_steps * deceleration_step_increment / 2.0f;
    if (!isfinite(zero_time) || zero_time <= 0.0f) {
      minimum_delay = minSpeedDelay;
      speedViolation = "1";
    } else {
      minimum_delay = zero_gap * (duration / zero_time);
    }
    if (minimum_delay <= minSpeedDelay) {
      minimum_delay = minSpeedDelay;
      speedViolation = "1";
    }
  }
  const float acceleration_step_increment =
    (minimum_delay * (100.0f / params.ramp_percent))
      / acceleration_steps;
  const float deceleration_step_increment =
    (minimum_delay * (100.0f / params.ramp_percent))
      / deceleration_steps;
  start_delay = acceleration_step_increment * acceleration_steps * 2.0f;
  end_delay = deceleration_step_increment * deceleration_steps * 2.0f;
  const float acceleration_waypoints =
    waypoint_count * params.acceleration_percent / 100.0f;
  const float deceleration_waypoints =
    waypoint_count * params.deceleration_percent / 100.0f;
  acceleration_increment =
    (start_delay - minimum_delay) / acceleration_waypoints;
  deceleration_increment =
    (end_delay - minimum_delay) / deceleration_waypoints;
  return isfinite(acceleration_increment)
    && isfinite(deceleration_increment)
    && ar4_protocol::valid_delay_envelope(
      minimum_delay, start_delay, end_delay, false, 0.0f
    );
}

inline JsonMainDirectResponseStatus execute_linear(
  const char *command,
  const JsonMainDirectParameters &parameters,
  uint32_t request_id,
  size_t maximum_payload_bytes,
  char *output,
  size_t output_capacity
) {
  JsonMainMoveCartesianExecutionResult result = {};
  MotionKinematicsTransaction kinematics;
  if (!refresh_motion_source_position()) {
    result.outcome = JsonMainMoveCartesianOutcome::kPositionUnavailable;
    return finish_motion_response(
      command, request_id, result, false, maximum_payload_bytes,
      output, output_capacity
    );
  }
  float start[ROBOT_nDOFs] = {};
  float target[ROBOT_nDOFs] = {};
  float vector[ROBOT_nDOFs] = {};
  load_pose(parameters.motion, target);
  float line_distance_squared = 0.0f;
  for (int axis = 0; axis < ROBOT_nDOFs; ++axis) {
    start[axis] = xyzuvw_Out[axis];
    vector[axis] = target[axis] - start[axis];
    line_distance_squared += vector[axis] * vector[axis];
  }
  const float line_distance = sqrtf(line_distance_squared);
  if (line_distance == 0.0f) {
    if (!execute_json_move_cartesian(
        parameters.motion, result, nullptr
    )) return JsonMainDirectResponseStatus::kInvalid;
    return finish_motion_response(
      command, request_id, result, false, maximum_payload_bytes,
      output, output_capacity
    );
  }
  int waypoint_count = 0;
  if (!ar4_protocol::waypoint_count_for_path(
      line_distance, linWayDistSP, waypoint_count
  )) return JsonMainDirectResponseStatus::kInvalid;
  const char wrist = wrist_code(parameters.motion.wrist_configuration);
  const int current[numJoints] = {
    J1StepM, J2StepM, J3StepM, J4StepM, J5StepM,
    J6StepM, J7StepM, J8StepM, J9StepM,
  };
  const int limits[numJoints] = {
    J1StepLim, J2StepLim, J3StepLim, J4StepLim, J5StepLim,
    J6StepLim, J7StepLim, J8StepLim, J9StepLim,
  };
  const int external_start[3] = {J7StepM, J8StepM, J9StepM};
  int external_target[3] = {};
  if (!external_positions_to_future_steps(
      parameters.motion.external_axes_units[0],
      parameters.motion.external_axes_units[1],
      parameters.motion.external_axes_units[2],
      external_target
  )) {
    result.outcome = JsonMainMoveCartesianOutcome::kPositionNotRepresentable;
    for (int axis = ROBOT_nDOFs; axis < numJoints; ++axis) {
      result.axes[axis] = true;
    }
    return finish_motion_response(
      command, request_id, result, false, maximum_payload_bytes,
      output, output_capacity
    );
  }
  int final_steps[numJoints] = {};
  for (int waypoint = 1; waypoint <= waypoint_count; ++waypoint) {
    JsonMainMoveCartesianOutcome failure =
      JsonMainMoveCartesianOutcome::kInvalid;
    int staged[numJoints] = {};
    if (!prepare_linear_waypoint(
        start, vector, waypoint, waypoint_count, wrist,
        external_start, external_target, limits, staged,
        result.axes, failure
    )) {
      result.outcome = failure;
      return finish_motion_response(
        command, request_id, result, false, maximum_payload_bytes,
        output, output_capacity
      );
    }
    if (waypoint == waypoint_count) {
      for (int axis = 0; axis < numJoints; ++axis) {
        final_steps[axis] = staged[axis];
      }
    }
  }
  int full_steps[numJoints] = {};
  int full_directions[numJoints] = {};
  bool full_axes[numJoints] = {};
  if (!step_move_from_targets(
      current, final_steps, limits,
      full_steps, full_directions, full_axes
  )) {
    result.outcome = JsonMainMoveCartesianOutcome::kJointLimitViolation;
    for (int axis = 0; axis < numJoints; ++axis) {
      result.axes[axis] = full_axes[axis];
    }
    return finish_motion_response(
      command, request_id, result, false, maximum_payload_bytes,
      output, output_capacity
    );
  }
  int high_step = 1;
  for (int axis = 0; axis < numJoints; ++axis) {
    if (full_steps[axis] > high_step) high_step = full_steps[axis];
  }
  float minimum_delay = 0.0f;
  float start_delay = 0.0f;
  float end_delay = 0.0f;
  float acceleration_increment = 0.0f;
  float deceleration_increment = 0.0f;
  speedViolation = "0";
  if (!linear_delay_profile(
      parameters.motion, line_distance, high_step, waypoint_count,
      minimum_delay, start_delay, end_delay,
      acceleration_increment, deceleration_increment
  )) return JsonMainDirectResponseStatus::kInvalid;
  if (controller_mutation_estop_blocked()) {
    result.outcome = JsonMainMoveCartesianOutcome::kEmergencyStop;
  } else if (!resetEncoders()) {
    result.outcome = JsonMainMoveCartesianOutcome::kEncoderStateUnavailable;
  } else {
    int loop_modes[ROBOT_nDOFs] = {};
    for (int axis = 0; axis < ROBOT_nDOFs; ++axis) {
      loop_modes[axis] = parameters.motion.loop_modes[axis] ? 1 : 0;
    }
    FirmwareMotionModeTransaction motion_modes(
      WristCon, JointLoopModes, String(wrist), loop_modes
    );
    float delay_value = start_delay;
    const float acceleration_waypoints =
      waypoint_count * parameters.motion.acceleration_percent / 100.0f;
    const float deceleration_waypoints =
      waypoint_count * parameters.motion.deceleration_percent / 100.0f;
    for (int waypoint = 1; waypoint <= waypoint_count; ++waypoint) {
      if (waypoint <= acceleration_waypoints) {
        delay_value = fmaxf(
          minimum_delay, delay_value - acceleration_increment
        );
      } else if (waypoint >= waypoint_count - deceleration_waypoints) {
        delay_value = fminf(
          end_delay, delay_value + deceleration_increment
        );
      } else {
        delay_value = minimum_delay;
      }
      JsonMainMoveCartesianOutcome failure =
        JsonMainMoveCartesianOutcome::kInvalid;
      int staged[numJoints] = {};
      bool axes[numJoints] = {};
      if (!prepare_linear_waypoint(
          start, vector, waypoint, waypoint_count, wrist,
          external_start, external_target, limits, staged, axes, failure
      )) {
        result.outcome = JsonMainMoveCartesianOutcome::kMotionExecutionFailed;
        break;
      }
      const int live_current[numJoints] = {
        J1StepM, J2StepM, J3StepM, J4StepM, J5StepM,
        J6StepM, J7StepM, J8StepM, J9StepM,
      };
      int steps[numJoints] = {};
      int directions[numJoints] = {};
      if (!step_move_from_targets(
          live_current, staged, limits, steps, directions, axes
      ) || !driveMotorsL(
          steps[0], steps[1], steps[2], steps[3], steps[4],
          steps[5], steps[6], steps[7], steps[8],
          directions[0], directions[1], directions[2],
          directions[3], directions[4], directions[5],
          directions[6], directions[7], directions[8],
          delay_value, &motion_modes
      )) {
        result.outcome = controller_mutation_estop_blocked()
          ? JsonMainMoveCartesianOutcome::kEmergencyStop
          : JsonMainMoveCartesianOutcome::kMotionExecutionFailed;
        break;
      }
      updatePos();
    }
    if (result.outcome == JsonMainMoveCartesianOutcome::kInvalid) {
      if (!checkEncoders()) {
        result.outcome = JsonMainMoveCartesianOutcome::kEncoderStateUnavailable;
      } else if (TotalCollision > 0) {
        result.outcome = JsonMainMoveCartesianOutcome::kEncoderCollision;
      } else {
        result.outcome = JsonMainMoveCartesianOutcome::kCompleted;
        result.speed_limited = speedViolation == "1";
      }
    }
  }
  if (
    result.outcome == JsonMainMoveCartesianOutcome::kEncoderCollision
    || result.outcome
      == JsonMainMoveCartesianOutcome::kEncoderStateUnavailable
  ) {
    const int faults[ROBOT_nDOFs] = {
      J1collisionTrue, J2collisionTrue, J3collisionTrue,
      J4collisionTrue, J5collisionTrue, J6collisionTrue,
    };
    for (int axis = 0; axis < ROBOT_nDOFs; ++axis) {
      result.axes[axis] = faults[axis] != 0;
    }
  }
  if (!capture_json_motion_position(result)) {
    result = {};
    result.outcome = JsonMainMoveCartesianOutcome::kPositionUnavailable;
  }
  speedViolation = "0";
  flag = "";
  return finish_motion_response(
    command, request_id, result, false, maximum_payload_bytes,
    output, output_capacity
  );
}

constexpr size_t kAtomicMotionMaximumPrimitiveCount =
  ar4_protocol::kJsonMainSplineMaximumSegmentCount * 2 - 1;

struct AtomicMotionPrimitive {
  bool arc;
  size_t profile_index;
  ar4_protocol::JsonMainMoveCartesianParameters motion;
  float start[ROBOT_nDOFs];
  float end[ROBOT_nDOFs];
  int external_start[3];
  int external_target[3];
  ar4_protocol::OrderedArcGeometry geometry;
  float path_length;
  int waypoint_count;
  int profile_waypoint_offset;
};

struct AtomicMotionProfile {
  bool used;
  ar4_protocol::JsonMainMoveCartesianParameters motion;
  float path_length;
  int waypoint_count;
  int high_step;
  float minimum_delay;
  float start_delay;
  float end_delay;
  float acceleration_increment;
  float deceleration_increment;
};

inline void mark_translation_not_representable(
  JsonMainMoveCartesianExecutionResult &result
) {
  result.outcome =
    JsonMainMoveCartesianOutcome::kPositionNotRepresentable;
  for (int axis = 0; axis < 3; ++axis) result.axes[axis] = true;
}

inline float translation_distance(
  const float (&start)[ROBOT_nDOFs],
  const float (&end)[ROBOT_nDOFs]
) {
  double squared = 0.0;
  for (int axis = 0; axis < 3; ++axis) {
    const double delta = static_cast<double>(end[axis]) - start[axis];
    squared += delta * delta;
  }
  const double distance = sqrt(squared);
  return isfinite(distance) && distance <= static_cast<double>(FLT_MAX)
    ? static_cast<float>(distance) : -1.0f;
}

inline bool interpolate_external_target(
  const int (&start)[3],
  const int (&target)[3],
  float fraction,
  const int (&limits)[numJoints],
  int (&output)[3]
) {
  if (!isfinite(fraction) || fraction < 0.0f || fraction > 1.0f) {
    return false;
  }
  for (int axis = 0; axis < 3; ++axis) {
    const double value = static_cast<double>(start[axis])
      + (static_cast<double>(target[axis]) - start[axis]) * fraction;
    if (
      !isfinite(value) || value < 0.0
      || value > static_cast<double>(limits[axis + ROBOT_nDOFs])
      || value > static_cast<double>(INT_MAX)
    ) return false;
    output[axis] = static_cast<int>(value);
  }
  return true;
}

inline void interpolate_pose(
  const float (&start)[ROBOT_nDOFs],
  const float (&end)[ROBOT_nDOFs],
  float fraction,
  float (&output)[ROBOT_nDOFs]
) {
  for (int axis = 0; axis < ROBOT_nDOFs; ++axis) {
    output[axis] = start[axis] + (end[axis] - start[axis]) * fraction;
  }
}

inline bool append_atomic_primitive(
  bool arc,
  size_t profile_index,
  const ar4_protocol::JsonMainMoveCartesianParameters &motion,
  const float (&start)[ROBOT_nDOFs],
  const float (&end)[ROBOT_nDOFs],
  const int (&external_start)[3],
  const int (&external_target)[3],
  const ar4_protocol::OrderedArcGeometry *geometry,
  float arc_path_length,
  AtomicMotionPrimitive (&primitives)[kAtomicMotionMaximumPrimitiveCount],
  size_t &primitive_count
) {
  float path_length = translation_distance(start, end);
  if (path_length < 0.0f) return false;
  if (arc) {
    if (
      geometry == nullptr || !isfinite(arc_path_length)
      || arc_path_length <= 0.0f
    ) return false;
    path_length = arc_path_length;
  }
  bool orientation_motion = false;
  for (int axis = 3; axis < ROBOT_nDOFs; ++axis) {
    orientation_motion = orientation_motion || start[axis] != end[axis];
  }
  bool external_motion = false;
  for (int axis = 0; axis < 3; ++axis) {
    external_motion = external_motion
      || external_start[axis] != external_target[axis];
  }
  if (path_length == 0.0f && !orientation_motion && !external_motion) {
    return true;
  }
  if (primitive_count >= kAtomicMotionMaximumPrimitiveCount) return false;

  AtomicMotionPrimitive staged = {};
  staged.arc = arc;
  staged.profile_index = profile_index;
  staged.motion = motion;
  staged.path_length = path_length > 0.0f ? path_length : linWayDistSP;
  for (int axis = 0; axis < ROBOT_nDOFs; ++axis) {
    staged.start[axis] = start[axis];
    staged.end[axis] = end[axis];
  }
  for (int axis = 0; axis < 3; ++axis) {
    staged.external_start[axis] = external_start[axis];
    staged.external_target[axis] = external_target[axis];
  }
  if (geometry != nullptr) staged.geometry = *geometry;
  if (path_length == 0.0f) {
    staged.waypoint_count = 1;
  } else if (!ar4_protocol::waypoint_count_for_path(
      path_length, linWayDistSP, staged.waypoint_count
  )) return false;
  if (arc && staged.waypoint_count < 2) staged.waypoint_count = 2;
  primitives[primitive_count++] = staged;
  return true;
}

inline bool prepare_atomic_waypoint(
  const AtomicMotionPrimitive &primitive,
  int waypoint,
  const int (&limits)[numJoints],
  int (&target_steps)[numJoints],
  bool (&axes)[numJoints],
  JsonMainMoveCartesianOutcome &failure
) {
  const float fraction = static_cast<float>(waypoint)
    / static_cast<float>(primitive.waypoint_count);
  if (primitive.arc) {
    const double radians = primitive.geometry.radians * fraction;
    const double cosine = cos(radians);
    const double sine = sin(radians);
    const double vector[3] = {
      static_cast<double>(primitive.start[0]) - primitive.geometry.center[0],
      static_cast<double>(primitive.start[1]) - primitive.geometry.center[1],
      static_cast<double>(primitive.start[2]) - primitive.geometry.center[2],
    };
    const double cross[3] = {
      primitive.geometry.axis[1] * vector[2]
        - primitive.geometry.axis[2] * vector[1],
      primitive.geometry.axis[2] * vector[0]
        - primitive.geometry.axis[0] * vector[2],
      primitive.geometry.axis[0] * vector[1]
        - primitive.geometry.axis[1] * vector[0],
    };
    const double projection = primitive.geometry.axis[0] * vector[0]
      + primitive.geometry.axis[1] * vector[1]
      + primitive.geometry.axis[2] * vector[2];
    for (int axis = 0; axis < 3; ++axis) {
      const double value = primitive.geometry.center[axis]
        + vector[axis] * cosine + cross[axis] * sine
        + primitive.geometry.axis[axis] * projection * (1.0 - cosine);
      if (!isfinite(value) || fabs(value) > FLT_MAX) {
        axes[axis] = true;
        failure = JsonMainMoveCartesianOutcome::kPositionNotRepresentable;
        return false;
      }
      xyzuvw_In[axis] = waypoint == primitive.waypoint_count
        ? primitive.end[axis] : static_cast<float>(value);
    }
  } else {
    for (int axis = 0; axis < 3; ++axis) {
      xyzuvw_In[axis] = primitive.start[axis]
        + (primitive.end[axis] - primitive.start[axis]) * fraction;
    }
  }
  for (int axis = 3; axis < ROBOT_nDOFs; ++axis) {
    xyzuvw_In[axis] = primitive.start[axis]
      + (primitive.end[axis] - primitive.start[axis]) * fraction;
  }
  SolveInverseKinematics(wrist_code(primitive.motion.wrist_configuration));
  if (KinematicError != 0) {
    failure = JsonMainMoveCartesianOutcome::kKinematicsUnreachable;
    return false;
  }
  int primary[ROBOT_nDOFs] = {};
  if (!primary_inverse_solution_to_future_steps(primary)) {
    for (int axis = 0; axis < ROBOT_nDOFs; ++axis) axes[axis] = true;
    failure = JsonMainMoveCartesianOutcome::kPositionNotRepresentable;
    return false;
  }
  for (int axis = 0; axis < ROBOT_nDOFs; ++axis) {
    target_steps[axis] = primary[axis];
  }
  for (int axis = 0; axis < 3; ++axis) {
    if (!ar4_protocol::interpolated_step_target(
        primitive.external_start[axis], primitive.external_target[axis],
        waypoint, primitive.waypoint_count,
        limits[axis + ROBOT_nDOFs],
        target_steps[axis + ROBOT_nDOFs]
    )) {
      axes[axis + ROBOT_nDOFs] = true;
      failure = JsonMainMoveCartesianOutcome::kPositionNotRepresentable;
      return false;
    }
  }
  for (int axis = 0; axis < numJoints; ++axis) {
    if (future_step_is_outside_limit(target_steps[axis], limits[axis])) {
      axes[axis] = true;
      failure = JsonMainMoveCartesianOutcome::kJointLimitViolation;
    }
  }
  return failure == JsonMainMoveCartesianOutcome::kInvalid;
}

inline bool preflight_atomic_motion(
  AtomicMotionPrimitive *primitives,
  size_t primitive_count,
  const int (&current)[numJoints],
  const int (&limits)[numJoints],
  AtomicMotionProfile (&profiles)[
    ar4_protocol::kJsonMainSplineMaximumSegmentCount
  ],
  JsonMainMoveCartesianExecutionResult &result
) {
  int previous[numJoints] = {};
  for (int axis = 0; axis < numJoints; ++axis) previous[axis] = current[axis];
  for (size_t index = 0; index < primitive_count; ++index) {
    AtomicMotionPrimitive &primitive = primitives[index];
    if (
      primitive.profile_index
        >= ar4_protocol::kJsonMainSplineMaximumSegmentCount
    ) return false;
    AtomicMotionProfile &profile = profiles[primitive.profile_index];
    if (!profile.used) {
      profile.used = true;
      profile.motion = primitive.motion;
    }
    primitive.profile_waypoint_offset = profile.waypoint_count;
    if (
      primitive.waypoint_count > INT_MAX - profile.waypoint_count
      || !isfinite(profile.path_length + primitive.path_length)
    ) return false;
    profile.waypoint_count += primitive.waypoint_count;
    profile.path_length += primitive.path_length;
    for (int waypoint = 1; waypoint <= primitive.waypoint_count; ++waypoint) {
      JsonMainMoveCartesianOutcome failure =
        JsonMainMoveCartesianOutcome::kInvalid;
      int staged[numJoints] = {};
      if (!prepare_atomic_waypoint(
          primitive, waypoint, limits, staged, result.axes, failure
      )) {
        result.outcome = failure;
        return true;
      }
      int steps[numJoints] = {};
      int directions[numJoints] = {};
      if (!step_move_from_targets(
          previous, staged, limits, steps, directions, result.axes
      )) {
        result.outcome = JsonMainMoveCartesianOutcome::kJointLimitViolation;
        return true;
      }
      int waypoint_high_step = 0;
      for (int axis = 0; axis < numJoints; ++axis) {
        if (steps[axis] > waypoint_high_step) {
          waypoint_high_step = steps[axis];
        }
        previous[axis] = staged[axis];
      }
      if (waypoint_high_step > INT_MAX - profile.high_step) return false;
      profile.high_step += waypoint_high_step;
    }
  }
  speedViolation = "0";
  for (
    size_t index = 0;
    index < ar4_protocol::kJsonMainSplineMaximumSegmentCount;
    ++index
  ) {
    AtomicMotionProfile &profile = profiles[index];
    if (!profile.used) continue;
    if (!linear_delay_profile(
        profile.motion,
        profile.path_length,
        profile.high_step > 0 ? profile.high_step : 1,
        profile.waypoint_count,
        profile.minimum_delay,
        profile.start_delay,
        profile.end_delay,
        profile.acceleration_increment,
        profile.deceleration_increment
    )) return false;
  }
  return true;
}

inline float atomic_waypoint_delay(
  const AtomicMotionProfile &profile,
  int waypoint
) {
  const float acceleration_waypoints = profile.waypoint_count
    * profile.motion.acceleration_percent / 100.0f;
  const float deceleration_waypoints = profile.waypoint_count
    * profile.motion.deceleration_percent / 100.0f;
  if (waypoint <= acceleration_waypoints) {
    return fmaxf(
      profile.minimum_delay,
      profile.start_delay - profile.acceleration_increment * waypoint
    );
  }
  if (waypoint >= profile.waypoint_count - deceleration_waypoints) {
    const float elapsed = waypoint
      - (profile.waypoint_count - deceleration_waypoints) + 1.0f;
    return fminf(
      profile.end_delay,
      profile.minimum_delay + profile.deceleration_increment * elapsed
    );
  }
  return profile.minimum_delay;
}

inline bool execute_atomic_primitives(
  AtomicMotionPrimitive *primitives,
  size_t primitive_count,
  const int (&limits)[numJoints],
  const AtomicMotionProfile (&profiles)[
    ar4_protocol::kJsonMainSplineMaximumSegmentCount
  ],
  JsonMainMoveCartesianExecutionResult &result
) {
  for (size_t index = 0; index < primitive_count; ++index) {
    const AtomicMotionPrimitive &primitive = primitives[index];
    const AtomicMotionProfile &profile = profiles[primitive.profile_index];
    int loop_modes[ROBOT_nDOFs] = {};
    for (int axis = 0; axis < ROBOT_nDOFs; ++axis) {
      loop_modes[axis] = primitive.motion.loop_modes[axis] ? 1 : 0;
    }
    FirmwareMotionModeTransaction motion_modes(
      WristCon,
      JointLoopModes,
      String(wrist_code(primitive.motion.wrist_configuration)),
      loop_modes
    );
    for (int waypoint = 1; waypoint <= primitive.waypoint_count; ++waypoint) {
      JsonMainMoveCartesianOutcome failure =
        JsonMainMoveCartesianOutcome::kInvalid;
      int staged[numJoints] = {};
      bool axes[numJoints] = {};
      if (!prepare_atomic_waypoint(
          primitive, waypoint, limits, staged, axes, failure
      )) {
        result.outcome = JsonMainMoveCartesianOutcome::kMotionExecutionFailed;
        return true;
      }
      const int live_current[numJoints] = {
        J1StepM, J2StepM, J3StepM, J4StepM, J5StepM,
        J6StepM, J7StepM, J8StepM, J9StepM,
      };
      int steps[numJoints] = {};
      int directions[numJoints] = {};
      const int profile_waypoint =
        primitive.profile_waypoint_offset + waypoint;
      if (!step_move_from_targets(
          live_current, staged, limits, steps, directions, axes
      ) || !driveMotorsL(
          steps[0], steps[1], steps[2], steps[3], steps[4],
          steps[5], steps[6], steps[7], steps[8],
          directions[0], directions[1], directions[2],
          directions[3], directions[4], directions[5],
          directions[6], directions[7], directions[8],
          atomic_waypoint_delay(profile, profile_waypoint),
          &motion_modes
      )) {
        result.outcome = controller_mutation_estop_blocked()
          ? JsonMainMoveCartesianOutcome::kEmergencyStop
          : JsonMainMoveCartesianOutcome::kMotionExecutionFailed;
        return true;
      }
      updatePos();
    }
  }
  return true;
}

inline bool run_atomic_motion(
  AtomicMotionPrimitive *primitives,
  size_t primitive_count,
  JsonMainMoveCartesianExecutionResult &result
) {
  const int current[numJoints] = {
    J1StepM, J2StepM, J3StepM, J4StepM, J5StepM,
    J6StepM, J7StepM, J8StepM, J9StepM,
  };
  const int limits[numJoints] = {
    J1StepLim, J2StepLim, J3StepLim, J4StepLim, J5StepLim,
    J6StepLim, J7StepLim, J8StepLim, J9StepLim,
  };
  AtomicMotionProfile profiles[
    ar4_protocol::kJsonMainSplineMaximumSegmentCount
  ] = {};
  bool preflight_valid = false;
  {
    MotionKinematicsTransaction preflight_kinematics;
    preflight_valid = preflight_atomic_motion(
      primitives, primitive_count, current, limits, profiles, result
    );
  }
  if (!preflight_valid) {
    speedViolation = "0";
    return false;
  }
  if (result.outcome != JsonMainMoveCartesianOutcome::kInvalid) {
    speedViolation = "0";
    return true;
  }
  if (primitive_count == 0) {
    result.outcome = JsonMainMoveCartesianOutcome::kCompleted;
  } else if (controller_mutation_estop_blocked()) {
    result.outcome = JsonMainMoveCartesianOutcome::kEmergencyStop;
  } else if (!resetEncoders()) {
    result.outcome = JsonMainMoveCartesianOutcome::kEncoderStateUnavailable;
  } else if (!execute_atomic_primitives(
      primitives, primitive_count, limits, profiles, result
  )) {
    speedViolation = "0";
    return false;
  } else if (result.outcome == JsonMainMoveCartesianOutcome::kInvalid) {
    if (!checkEncoders()) {
      result.outcome = JsonMainMoveCartesianOutcome::kEncoderStateUnavailable;
    } else if (TotalCollision > 0) {
      result.outcome = JsonMainMoveCartesianOutcome::kEncoderCollision;
    } else {
      result.outcome = JsonMainMoveCartesianOutcome::kCompleted;
      result.speed_limited = speedViolation == "1";
    }
  }
  if (
    result.outcome == JsonMainMoveCartesianOutcome::kEncoderCollision
    || result.outcome
      == JsonMainMoveCartesianOutcome::kEncoderStateUnavailable
  ) {
    const int faults[ROBOT_nDOFs] = {
      J1collisionTrue, J2collisionTrue, J3collisionTrue,
      J4collisionTrue, J5collisionTrue, J6collisionTrue,
    };
    for (int axis = 0; axis < ROBOT_nDOFs; ++axis) {
      result.axes[axis] = faults[axis] != 0;
    }
  }
  if (!capture_json_motion_position(result)) {
    result = {};
    result.outcome = JsonMainMoveCartesianOutcome::kPositionUnavailable;
  }
  speedViolation = "0";
  flag = "";
  return true;
}

enum class AtomicFilletStatus : uint8_t {
  kValid,
  kStraight,
  kInvalid,
};

inline AtomicFilletStatus calculate_atomic_fillet(
  const float (&before)[ROBOT_nDOFs],
  const float (&corner)[ROBOT_nDOFs],
  const float (&after)[ROBOT_nDOFs],
  ar4_protocol::OrderedArcGeometry &geometry
) {
  double before_vector[3] = {};
  double after_vector[3] = {};
  double before_length_squared = 0.0;
  double after_length_squared = 0.0;
  for (int axis = 0; axis < 3; ++axis) {
    before_vector[axis] = static_cast<double>(before[axis]) - corner[axis];
    after_vector[axis] = static_cast<double>(after[axis]) - corner[axis];
    before_length_squared += before_vector[axis] * before_vector[axis];
    after_length_squared += after_vector[axis] * after_vector[axis];
  }
  const double before_length = sqrt(before_length_squared);
  const double after_length = sqrt(after_length_squared);
  if (
    !isfinite(before_length) || !isfinite(after_length)
    || before_length <= 0.0 || after_length <= 0.0
  ) return AtomicFilletStatus::kInvalid;
  double dot = 0.0;
  for (int axis = 0; axis < 3; ++axis) {
    before_vector[axis] /= before_length;
    after_vector[axis] /= after_length;
    dot += before_vector[axis] * after_vector[axis];
  }
  if (dot <= -0.999999) return AtomicFilletStatus::kStraight;
  if (!isfinite(dot) || dot >= 0.999999) {
    return AtomicFilletStatus::kInvalid;
  }
  const double cosine_half = sqrt(fmax(0.0, (1.0 + dot) * 0.5));
  double bisector[3] = {};
  double bisector_squared = 0.0;
  for (int axis = 0; axis < 3; ++axis) {
    bisector[axis] = before_vector[axis] + after_vector[axis];
    bisector_squared += bisector[axis] * bisector[axis];
  }
  const double bisector_length = sqrt(bisector_squared);
  if (cosine_half <= 0.0 || bisector_length <= 0.0) {
    return AtomicFilletStatus::kInvalid;
  }
  const double cut_distance = (before_length + after_length) * 0.5;
  const double center_distance = cut_distance / cosine_half;
  ar4_protocol::OrderedArcGeometry staged = {};
  for (int axis = 0; axis < 3; ++axis) {
    staged.center[axis] = corner[axis]
      + bisector[axis] / bisector_length * center_distance;
  }
  const double start_radius[3] = {
    static_cast<double>(before[0]) - staged.center[0],
    static_cast<double>(before[1]) - staged.center[1],
    static_cast<double>(before[2]) - staged.center[2],
  };
  const double end_radius[3] = {
    static_cast<double>(after[0]) - staged.center[0],
    static_cast<double>(after[1]) - staged.center[1],
    static_cast<double>(after[2]) - staged.center[2],
  };
  const double cross[3] = {
    start_radius[1] * end_radius[2] - start_radius[2] * end_radius[1],
    start_radius[2] * end_radius[0] - start_radius[0] * end_radius[2],
    start_radius[0] * end_radius[1] - start_radius[1] * end_radius[0],
  };
  const double cross_length = sqrt(
    cross[0] * cross[0] + cross[1] * cross[1] + cross[2] * cross[2]
  );
  double radius_squared = 0.0;
  double radius_dot = 0.0;
  for (int axis = 0; axis < 3; ++axis) {
    radius_squared += start_radius[axis] * start_radius[axis];
    radius_dot += start_radius[axis] * end_radius[axis];
  }
  staged.radius = sqrt(radius_squared);
  staged.radians = atan2(cross_length, radius_dot);
  if (
    !isfinite(cross_length) || cross_length <= 0.0
    || !isfinite(staged.radius) || staged.radius <= 0.0
    || !isfinite(staged.radians) || staged.radians <= 0.0
  ) return AtomicFilletStatus::kInvalid;
  for (int axis = 0; axis < 3; ++axis) {
    staged.axis[axis] = cross[axis] / cross_length;
  }
  geometry = staged;
  return AtomicFilletStatus::kValid;
}

inline bool calculate_atomic_circle(
  const float *center,
  const float *start,
  const float *plane,
  ar4_protocol::OrderedArcGeometry &geometry
) {
  if (!ar4_protocol::valid_circle_geometry(
      center, start, plane, linWayDistSP
  )) return false;
  const double start_radius[3] = {
    static_cast<double>(start[0]) - center[0],
    static_cast<double>(start[1]) - center[1],
    static_cast<double>(start[2]) - center[2],
  };
  const double plane_radius[3] = {
    static_cast<double>(plane[0]) - center[0],
    static_cast<double>(plane[1]) - center[1],
    static_cast<double>(plane[2]) - center[2],
  };
  const double cross[3] = {
    start_radius[1] * plane_radius[2] - start_radius[2] * plane_radius[1],
    start_radius[2] * plane_radius[0] - start_radius[0] * plane_radius[2],
    start_radius[0] * plane_radius[1] - start_radius[1] * plane_radius[0],
  };
  const double radius = sqrt(
    start_radius[0] * start_radius[0]
      + start_radius[1] * start_radius[1]
      + start_radius[2] * start_radius[2]
  );
  const double cross_length = sqrt(
    cross[0] * cross[0] + cross[1] * cross[1] + cross[2] * cross[2]
  );
  if (
    !isfinite(radius) || radius <= 0.0
    || !isfinite(cross_length) || cross_length <= 0.0
  ) return false;
  ar4_protocol::OrderedArcGeometry staged = {};
  staged.radius = radius;
  staged.radians = 2.0 * 3.14159265358979323846;
  for (int axis = 0; axis < 3; ++axis) {
    staged.center[axis] = center[axis];
    staged.axis[axis] = cross[axis] / cross_length;
  }
  geometry = staged;
  return true;
}

inline JsonMainDirectResponseStatus execute_arc_or_circle(
  const char *command,
  const JsonMainDirectParameters &parameters,
  bool circle,
  uint32_t request_id,
  size_t maximum_payload_bytes,
  char *output,
  size_t output_capacity
) {
  JsonMainMoveCartesianExecutionResult result = {};
  MotionKinematicsTransaction kinematics;
  if (!refresh_motion_source_position()) {
    result.outcome = JsonMainMoveCartesianOutcome::kPositionUnavailable;
    return finish_motion_response(
      command, request_id, result, false, maximum_payload_bytes,
      output, output_capacity
    );
  }
  float start[ROBOT_nDOFs] = {};
  float end[ROBOT_nDOFs] = {};
  for (int axis = 0; axis < ROBOT_nDOFs; ++axis) start[axis] = xyzuvw_Out[axis];
  load_pose(parameters.motion, end);
  const int current[numJoints] = {
    J1StepM, J2StepM, J3StepM, J4StepM, J5StepM,
    J6StepM, J7StepM, J8StepM, J9StepM,
  };
  const int limits[numJoints] = {
    J1StepLim, J2StepLim, J3StepLim, J4StepLim, J5StepLim,
    J6StepLim, J7StepLim, J8StepLim, J9StepLim,
  };
  const int external_start[3] = {J7StepM, J8StepM, J9StepM};
  int external_target[3] = {};
  if (!external_positions_to_future_steps(
      parameters.motion.external_axes_units[0],
      parameters.motion.external_axes_units[1],
      parameters.motion.external_axes_units[2],
      external_target
  )) {
    result.outcome = JsonMainMoveCartesianOutcome::kPositionNotRepresentable;
    for (int axis = ROBOT_nDOFs; axis < numJoints; ++axis) {
      result.axes[axis] = true;
    }
    return finish_motion_response(
      command, request_id, result, false, maximum_payload_bytes,
      output, output_capacity
    );
  }
  ar4_protocol::OrderedArcGeometry geometry = {};
  if (circle) {
    float zero[ROBOT_nDOFs] = {};
    int declared_steps[numJoints] = {};
    JsonMainMoveCartesianOutcome failure =
      JsonMainMoveCartesianOutcome::kInvalid;
    if (!prepare_linear_waypoint(
        end, zero, 1, 1, wrist_code(parameters.motion.wrist_configuration),
        external_target, external_target, limits, declared_steps,
        result.axes, failure
    )) {
      result.outcome = failure;
      return finish_motion_response(
        command, request_id, result, false, maximum_payload_bytes,
        output, output_capacity
      );
    }
    bool start_matches = true;
    for (int axis = 0; axis < numJoints; ++axis) {
      if (declared_steps[axis] != current[axis]) {
        result.axes[axis] = true;
        start_matches = false;
      }
    }
    if (!start_matches) {
      result.outcome =
        JsonMainMoveCartesianOutcome::kPositionNotRepresentable;
      return finish_motion_response(
        command, request_id, result, false, maximum_payload_bytes,
        output, output_capacity
      );
    }
    if (!calculate_atomic_circle(
        parameters.center_translation_millimeters,
        end,
        parameters.plane_translation_millimeters,
        geometry
    )) {
      mark_translation_not_representable(result);
      return finish_motion_response(
        command, request_id, result, false, maximum_payload_bytes,
        output, output_capacity
      );
    }
    for (int axis = 0; axis < ROBOT_nDOFs; ++axis) start[axis] = end[axis];
  } else if (!ar4_protocol::valid_arc_geometry(
      start,
      parameters.midpoint_translation_millimeters,
      end,
      linWayDistSP,
      &geometry
  )) {
    mark_translation_not_representable(result);
    return finish_motion_response(
      command, request_id, result, false, maximum_payload_bytes,
      output, output_capacity
    );
  }
  const double path = geometry.radius * geometry.radians;
  AtomicMotionPrimitive primitives[kAtomicMotionMaximumPrimitiveCount] = {};
  size_t primitive_count = 0;
  if (
    !isfinite(path) || path <= 0.0 || path > FLT_MAX
    || !append_atomic_primitive(
      true, 0, parameters.motion, start, end,
      external_start, external_target, &geometry,
      static_cast<float>(path), primitives, primitive_count
    )
    || !run_atomic_motion(primitives, primitive_count, result)
  ) return JsonMainDirectResponseStatus::kInvalid;
  return finish_motion_response(
    command, request_id, result, false, maximum_payload_bytes,
    output, output_capacity
  );
}

inline JsonMainDirectResponseStatus execute_spline(
  const char *command,
  const JsonMainDirectParameters &parameters,
  uint32_t request_id,
  size_t maximum_payload_bytes,
  char *output,
  size_t output_capacity
) {
  JsonMainMoveCartesianExecutionResult result = {};
  MotionKinematicsTransaction kinematics;
  if (!refresh_motion_source_position()) {
    result.outcome = JsonMainMoveCartesianOutcome::kPositionUnavailable;
    return finish_motion_response(
      command, request_id, result, false, maximum_payload_bytes,
      output, output_capacity
    );
  }
  const int limits[numJoints] = {
    J1StepLim, J2StepLim, J3StepLim, J4StepLim, J5StepLim,
    J6StepLim, J7StepLim, J8StepLim, J9StepLim,
  };
  float targets[ar4_protocol::kJsonMainSplineMaximumSegmentCount][
    ROBOT_nDOFs
  ] = {};
  int external_targets[
    ar4_protocol::kJsonMainSplineMaximumSegmentCount
  ][3] = {};
  for (size_t index = 0; index < parameters.spline_segment_count; ++index) {
    load_pose(parameters.spline_segments[index].motion, targets[index]);
    if (!external_positions_to_future_steps(
        parameters.spline_segments[index].motion.external_axes_units[0],
        parameters.spline_segments[index].motion.external_axes_units[1],
        parameters.spline_segments[index].motion.external_axes_units[2],
        external_targets[index]
    )) {
      result.outcome =
        JsonMainMoveCartesianOutcome::kPositionNotRepresentable;
      for (int axis = ROBOT_nDOFs; axis < numJoints; ++axis) {
        result.axes[axis] = true;
      }
      return finish_motion_response(
        command, request_id, result, false, maximum_payload_bytes,
        output, output_capacity
      );
    }
  }
  float previous[ROBOT_nDOFs] = {};
  float cursor[ROBOT_nDOFs] = {};
  for (int axis = 0; axis < ROBOT_nDOFs; ++axis) {
    previous[axis] = xyzuvw_Out[axis];
    cursor[axis] = xyzuvw_Out[axis];
  }
  int previous_external[3] = {J7StepM, J8StepM, J9StepM};
  int cursor_external[3] = {J7StepM, J8StepM, J9StepM};
  AtomicMotionPrimitive primitives[kAtomicMotionMaximumPrimitiveCount] = {};
  size_t primitive_count = 0;
  for (size_t index = 0; index < parameters.spline_segment_count; ++index) {
    const ar4_protocol::JsonMainSplineSegment &segment =
      parameters.spline_segments[index];
    const bool rounded = segment.rounding_millimeters > 0.0f;
    if (!rounded) {
      if (!append_atomic_primitive(
          false, index, segment.motion, cursor, targets[index],
          cursor_external, external_targets[index], nullptr, 0.0f,
          primitives, primitive_count
      )) return JsonMainDirectResponseStatus::kInvalid;
      for (int axis = 0; axis < ROBOT_nDOFs; ++axis) {
        cursor[axis] = targets[index][axis];
      }
      for (int axis = 0; axis < 3; ++axis) {
        cursor_external[axis] = external_targets[index][axis];
      }
    } else {
      if (index + 1 >= parameters.spline_segment_count) {
        mark_translation_not_representable(result);
        return finish_motion_response(
          command, request_id, result, false, maximum_payload_bytes,
          output, output_capacity
        );
      }
      const float incoming = translation_distance(previous, targets[index]);
      const float outgoing = translation_distance(
        targets[index], targets[index + 1]
      );
      if (!ar4_protocol::valid_positive_spline_rounding(
          segment.rounding_millimeters, incoming, outgoing
      )) {
        mark_translation_not_representable(result);
        return finish_motion_response(
          command, request_id, result, false, maximum_payload_bytes,
          output, output_capacity
        );
      }
      const float blend = segment.rounding_millimeters;
      const float before_fraction = 1.0f - blend / incoming;
      const float after_fraction = blend / outgoing;
      float before[ROBOT_nDOFs] = {};
      float after[ROBOT_nDOFs] = {};
      int before_external[3] = {};
      int after_external[3] = {};
      interpolate_pose(previous, targets[index], before_fraction, before);
      interpolate_pose(targets[index], targets[index + 1], after_fraction, after);
      if (
        !interpolate_external_target(
          previous_external, external_targets[index], before_fraction,
          limits, before_external
        )
        || !interpolate_external_target(
          external_targets[index], external_targets[index + 1],
          after_fraction, limits, after_external
        )
      ) {
        result.outcome =
          JsonMainMoveCartesianOutcome::kPositionNotRepresentable;
        for (int axis = ROBOT_nDOFs; axis < numJoints; ++axis) {
          result.axes[axis] = true;
        }
        return finish_motion_response(
          command, request_id, result, false, maximum_payload_bytes,
          output, output_capacity
        );
      }
      ar4_protocol::OrderedArcGeometry geometry = {};
      const AtomicFilletStatus fillet = calculate_atomic_fillet(
        before, targets[index], after, geometry
      );
      if (fillet == AtomicFilletStatus::kInvalid) {
        mark_translation_not_representable(result);
        return finish_motion_response(
          command, request_id, result, false, maximum_payload_bytes,
          output, output_capacity
        );
      }
      if (fillet == AtomicFilletStatus::kStraight) {
        if (!append_atomic_primitive(
            false, index, segment.motion, cursor, targets[index],
            cursor_external, external_targets[index], nullptr, 0.0f,
            primitives, primitive_count
        )) return JsonMainDirectResponseStatus::kInvalid;
        for (int axis = 0; axis < ROBOT_nDOFs; ++axis) {
          cursor[axis] = targets[index][axis];
        }
        for (int axis = 0; axis < 3; ++axis) {
          cursor_external[axis] = external_targets[index][axis];
        }
      } else {
        const double arc_path = geometry.radius * geometry.radians;
        if (
          !isfinite(arc_path) || arc_path <= 0.0 || arc_path > FLT_MAX
          || !append_atomic_primitive(
            false, index, segment.motion, cursor, before,
            cursor_external, before_external, nullptr, 0.0f,
            primitives, primitive_count
          )
          || !append_atomic_primitive(
            true, index, segment.motion, before, after,
            before_external, after_external, &geometry,
            static_cast<float>(arc_path), primitives, primitive_count
          )
        ) return JsonMainDirectResponseStatus::kInvalid;
        for (int axis = 0; axis < ROBOT_nDOFs; ++axis) {
          cursor[axis] = after[axis];
        }
        for (int axis = 0; axis < 3; ++axis) {
          cursor_external[axis] = after_external[axis];
        }
      }
    }
    for (int axis = 0; axis < ROBOT_nDOFs; ++axis) {
      previous[axis] = targets[index][axis];
    }
    for (int axis = 0; axis < 3; ++axis) {
      previous_external[axis] = external_targets[index][axis];
    }
  }
  if (!run_atomic_motion(primitives, primitive_count, result)) {
    return JsonMainDirectResponseStatus::kInvalid;
  }
  return finish_motion_response(
    command, request_id, result, false, maximum_payload_bytes,
    output, output_capacity
  );
}

inline JsonMainDirectResponseStatus execute_modbus_wait(
  const char *command,
  const JsonMainDirectParameters &parameters,
  ar4_protocol::ModbusOperation operation,
  uint32_t request_id,
  size_t maximum_payload_bytes,
  char *output,
  size_t output_capacity
) {
  const uint32_t started = static_cast<uint32_t>(millis());
  int32_t value = -1;
  while (
    static_cast<uint32_t>(millis() - started)
      < parameters.timeout_milliseconds
  ) {
    if (controller_mutation_estop_blocked()) {
      return build_error_response(
          command, request_id,
          ar4_protocol::JsonErrorResponseStatus::kCancelled,
          "emergency_stop",
          "emergency stop interrupted Modbus wait",
          maximum_payload_bytes, output, output_capacity
        )
        ? JsonMainDirectResponseStatus::kCancelled
        : JsonMainDirectResponseStatus::kInvalid;
    }
    value = execute_modbus_operation(
      operation,
      parameters.slave_id,
      parameters.address,
      1
    );
    if (value < 0) {
      return build_error_response(
          command, request_id,
          ar4_protocol::JsonErrorResponseStatus::kFailed,
          "modbus_error",
          "Modbus read failed",
          maximum_payload_bytes, output, output_capacity
        )
        ? JsonMainDirectResponseStatus::kFailed
        : JsonMainDirectResponseStatus::kInvalid;
    }
    if (value == parameters.expected) {
      return build_integer_response(
          command, request_id, "value", value,
          maximum_payload_bytes, output, output_capacity
        )
        ? JsonMainDirectResponseStatus::kCompleted
        : JsonMainDirectResponseStatus::kInvalid;
    }
    if (!wait_for_controller_duration(100, true)) break;
  }
  if (controller_mutation_estop_blocked()) {
    return build_error_response(
        command, request_id,
        ar4_protocol::JsonErrorResponseStatus::kCancelled,
        "emergency_stop",
        "emergency stop interrupted Modbus wait",
        maximum_payload_bytes, output, output_capacity
      )
      ? JsonMainDirectResponseStatus::kCancelled
      : JsonMainDirectResponseStatus::kInvalid;
  }
  return build_error_response(
      command, request_id,
      ar4_protocol::JsonErrorResponseStatus::kFailed,
      "timeout",
      "Modbus wait expired before the expected value",
      maximum_payload_bytes, output, output_capacity
    )
    ? JsonMainDirectResponseStatus::kFailed
    : JsonMainDirectResponseStatus::kInvalid;
}

inline int compare_filenames(const char *left, const char *right) {
  if (left == nullptr || right == nullptr) return 0;
  for (size_t index = 0;; ++index) {
    unsigned char a = static_cast<unsigned char>(left[index]);
    unsigned char b = static_cast<unsigned char>(right[index]);
    if (a >= 'A' && a <= 'Z') a += 'a' - 'A';
    if (b >= 'A' && b <= 'Z') b += 'a' - 'A';
    if (a != b) return a < b ? -1 : 1;
    if (a == 0) return 0;
  }
}

inline JsonMainDirectResponseStatus execute_delete(
  const char *command, const JsonMainDirectParameters &parameters,
  uint32_t request_id, size_t maximum_payload_bytes,
  char *output, size_t output_capacity
) {
  const String media(parameters.media_id);
  const String filename(parameters.filename);
  if (!initSD()) {
    return build_error_response(command, request_id,
      ar4_protocol::JsonErrorResponseStatus::kFailed, "storage_error",
      "SD card is unavailable", maximum_payload_bytes, output, output_capacity)
      ? JsonMainDirectResponseStatus::kFailed : JsonMainDirectResponseStatus::kInvalid;
  }
  if (!verifyExpectedSDMediaId(media)) {
    return build_error_response(command, request_id,
      ar4_protocol::JsonErrorResponseStatus::kFailed, "media_changed",
      "SD media identity changed", maximum_payload_bytes, output, output_capacity)
      ? JsonMainDirectResponseStatus::kFailed : JsonMainDirectResponseStatus::kInvalid;
  }
  const ar4_protocol::SDFileLookupStatus found = findSDFile(filename);
  if (found == ar4_protocol::SDFileLookupStatus::kError) {
    return build_error_response(command, request_id,
      ar4_protocol::JsonErrorResponseStatus::kFailed, "storage_error",
      "SD directory lookup failed", maximum_payload_bytes, output, output_capacity)
      ? JsonMainDirectResponseStatus::kFailed : JsonMainDirectResponseStatus::kInvalid;
  }
  const bool deleted = found == ar4_protocol::SDFileLookupStatus::kPresent;
  if (deleted && !deleteSD(filename, media)) {
    const char *code = verifyExpectedSDMediaId(media)
      ? "storage_error" : "media_changed";
    return build_error_response(command, request_id,
      ar4_protocol::JsonErrorResponseStatus::kFailed, code,
      "SD deletion failed", maximum_payload_bytes, output, output_capacity)
      ? JsonMainDirectResponseStatus::kFailed : JsonMainDirectResponseStatus::kInvalid;
  }
  return build_bool_response(command, request_id, "deleted", deleted,
      maximum_payload_bytes, output, output_capacity)
    ? JsonMainDirectResponseStatus::kCompleted : JsonMainDirectResponseStatus::kInvalid;
}

inline JsonMainDirectResponseStatus execute_list(
  const char *command, uint32_t request_id, size_t maximum_payload_bytes,
  char *output, size_t output_capacity
) {
  if (!initSD()) {
    return build_error_response(command, request_id,
      ar4_protocol::JsonErrorResponseStatus::kFailed, "storage_error",
      "SD card is unavailable", maximum_payload_bytes, output, output_capacity)
      ? JsonMainDirectResponseStatus::kFailed : JsonMainDirectResponseStatus::kInvalid;
  }
  char media[ar4_protocol::kControllerMediaIdCapacity] = {};
  if (!copyMountedSDMediaId(media)) {
    return build_error_response(command, request_id,
      ar4_protocol::JsonErrorResponseStatus::kFailed, "storage_error",
      "SD media identity is unavailable", maximum_payload_bytes,
      output, output_capacity)
      ? JsonMainDirectResponseStatus::kFailed : JsonMainDirectResponseStatus::kInvalid;
  }
  const size_t capacity = output_capacity < maximum_payload_bytes + 1
    ? output_capacity : maximum_payload_bytes + 1;
  size_t index = 0;
  output[0] = '\0';
  bool built = ar4_protocol::append_json_text("{\"cmd\":\"", output, capacity, index)
    && ar4_protocol::append_json_text(command, output, capacity, index)
    && ar4_protocol::append_json_text("\",\"id\":", output, capacity, index)
    && ar4_protocol::append_json_uint32(request_id, output, capacity, index)
    && ar4_protocol::append_json_text(",\"result\":{\"files\":[", output, capacity, index);
  String last;
  bool first = true;
  while (built) {
    FsFile root = SD.sdfs.open("/");
    if (!root) { built = false; break; }
    String next;
    while (true) {
      FsFile entry = root.openNextFile();
      if (!entry) { if (root.getError() != 0) built = false; break; }
      char name[ar4_protocol::kControllerFilenameMaxLength + 1] = {};
      size_t length = 0;
      const bool name_read = ar4_protocol::read_controller_directory_entry_name(
        entry, name, sizeof(name), length);
      const bool directory = entry.isDirectory();
      entry.close();
      if (!name_read) { built = false; break; }
      if (directory || strcmp(name, "System Volume Information") == 0) continue;
      if (!ar4_protocol::valid_controller_directory_entry_filename(
          String(name), 0, static_cast<int>(length))) {
        built = false;
        break;
      }
      if (compare_filenames(name, last.c_str()) > 0
          && (next.length() == 0 || compare_filenames(name, next.c_str()) < 0)) next = name;
    }
    root.close();
    if (!built || next.length() == 0) break;
    built = (first || ar4_protocol::append_json_text(",", output, capacity, index))
      && ar4_protocol::append_json_text("\"", output, capacity, index)
      && ar4_protocol::append_json_text(next.c_str(), output, capacity, index)
      && ar4_protocol::append_json_text("\"", output, capacity, index);
    first = false;
    last = next;
  }
  built = built && verifyExpectedSDMediaId(String(media))
    && ar4_protocol::append_json_text("],\"media_id\":\"", output, capacity, index)
    && ar4_protocol::append_json_text(media, output, capacity, index)
    && ar4_protocol::append_json_text(
      "\"},\"status\":\"completed\",\"type\":\"response\",\"v\":1}",
      output, capacity, index);
  if (!built) {
    return build_error_response(command, request_id,
      ar4_protocol::JsonErrorResponseStatus::kFailed, "storage_error",
      "SD directory listing failed", maximum_payload_bytes, output, output_capacity)
      ? JsonMainDirectResponseStatus::kFailed : JsonMainDirectResponseStatus::kInvalid;
  }
  return JsonMainDirectResponseStatus::kCompleted;
}

inline JsonMainDirectResponseStatus execute_write_move(
  const char *command, const JsonMainDirectParameters &parameters,
  uint32_t request_id, size_t maximum_payload_bytes,
  char *output, size_t output_capacity
) {
  JsonMainMoveCartesianExecutionResult result = {};
  MotionKinematicsTransaction kinematics;
  float pose[ROBOT_nDOFs] = {};
  load_pose(parameters.motion, pose);
  for (int axis = 0; axis < ROBOT_nDOFs; ++axis) xyzuvw_In[axis] = pose[axis];
  SolveInverseKinematics(wrist_code(parameters.motion.wrist_configuration));
  int targets[numJoints] = {};
  if (KinematicError != 0) {
    result.outcome = JsonMainMoveCartesianOutcome::kKinematicsUnreachable;
  } else if (!inverse_solution_to_future_steps(
      parameters.motion.external_axes_units[0],
      parameters.motion.external_axes_units[1],
      parameters.motion.external_axes_units[2], targets
  )) {
    result.outcome = JsonMainMoveCartesianOutcome::kPositionNotRepresentable;
    for (int axis = 0; axis < numJoints; ++axis) result.axes[axis] = true;
  }
  const int current[numJoints] = {
    J1StepM, J2StepM, J3StepM, J4StepM, J5StepM,
    J6StepM, J7StepM, J8StepM, J9StepM};
  const int limits[numJoints] = {
    J1StepLim, J2StepLim, J3StepLim, J4StepLim, J5StepLim,
    J6StepLim, J7StepLim, J8StepLim, J9StepLim};
  int steps[numJoints] = {}, directions[numJoints] = {};
  if (result.outcome == JsonMainMoveCartesianOutcome::kInvalid
      && !step_move_from_targets(current, targets, limits, steps,
        directions, result.axes)) {
    result.outcome = JsonMainMoveCartesianOutcome::kJointLimitViolation;
  }
  if (result.outcome != JsonMainMoveCartesianOutcome::kInvalid) {
    return finish_motion_response(command, request_id, result, true,
      maximum_payload_bytes, output, output_capacity);
  }
  String row;
  if (!row.reserve(384)) return JsonMainDirectResponseStatus::kInvalid;
  row += 'X'; row += String(parameters.motion.translation_millimeters[0], 6);
  row += 'Y'; row += String(parameters.motion.translation_millimeters[1], 6);
  row += 'Z'; row += String(parameters.motion.translation_millimeters[2], 6);
  row += "Rz"; row += String(parameters.motion.orientation_degrees[2], 6);
  row += "Ry"; row += String(parameters.motion.orientation_degrees[1], 6);
  row += "Rx"; row += String(parameters.motion.orientation_degrees[0], 6);
  row += "J7"; row += String(parameters.motion.external_axes_units[0], 6);
  row += "J8"; row += String(parameters.motion.external_axes_units[1], 6);
  row += "J9"; row += String(parameters.motion.external_axes_units[2], 6);
  row += 'S'; row += speed_code(parameters.motion.speed_mode);
  row += String(parameters.motion.speed_value, 6);
  row += "Ac"; row += String(parameters.motion.acceleration_percent, 6);
  row += "Dc"; row += String(parameters.motion.deceleration_percent, 6);
  row += "Rm"; row += String(parameters.motion.ramp_percent, 6);
  row += 'W'; row += wrist_code(parameters.motion.wrist_configuration);
  row += "Lm";
  for (int axis = 0; axis < ROBOT_nDOFs; ++axis) {
    row += parameters.motion.loop_modes[axis] ? '1' : '0';
  }
  const String media(parameters.media_id), filename(parameters.filename);
  if (!initSD() || !verifyExpectedSDMediaId(media)
      || !writeSD(filename, row, media)) {
    const bool media_changed = sd_ok && !verifyExpectedSDMediaId(media);
    return build_error_response(command, request_id,
      ar4_protocol::JsonErrorResponseStatus::kFailed,
      media_changed ? "media_changed" : "storage_error",
      media_changed ? "SD media identity changed" : "SD write failed",
      maximum_payload_bytes, output, output_capacity)
      ? JsonMainDirectResponseStatus::kFailed : JsonMainDirectResponseStatus::kInvalid;
  }
  result.outcome = JsonMainMoveCartesianOutcome::kCompleted;
  if (!capture_json_motion_position(result)) {
    result = {}; result.outcome = JsonMainMoveCartesianOutcome::kPositionUnavailable;
  }
  return finish_motion_response(command, request_id, result, true,
    maximum_payload_bytes, output, output_capacity);
}

inline bool stored_cartesian_parameters(
  const String &row, ar4_protocol::JsonMainMoveCartesianParameters &params
) {
  ar4_protocol::CartesianMoveCommandFields fields = {};
  if (!ar4_protocol::parse_cartesian_move_command(row, fields)) return false;
  for (int axis = 0; axis < 3; ++axis) {
    params.translation_millimeters[axis] = fields.pose[axis];
    params.orientation_degrees[axis] = fields.pose[5 - axis];
    params.external_axes_units[axis] = fields.auxiliary[axis];
  }
  params.speed_mode = fields.speed_mode == 'p'
    ? ar4_protocol::JsonCartesianMotionSpeedMode::kPercent
    : fields.speed_mode == 's'
      ? ar4_protocol::JsonCartesianMotionSpeedMode::kSeconds
      : ar4_protocol::JsonCartesianMotionSpeedMode::kMillimetersPerSecond;
  params.speed_value = fields.speed;
  params.acceleration_percent = fields.acceleration;
  params.deceleration_percent = fields.deceleration;
  params.ramp_percent = fields.ramp;
  params.wrist_configuration = fields.wrist_config == 'A'
    ? ar4_protocol::JsonCartesianMotionWristConfiguration::kAutomatic
    : fields.wrist_config == 'N'
      ? ar4_protocol::JsonCartesianMotionWristConfiguration::kNear
      : ar4_protocol::JsonCartesianMotionWristConfiguration::kFar;
  for (int axis = 0; axis < ROBOT_nDOFs; ++axis) {
    params.loop_modes[axis] = fields.loop_modes[axis] != 0;
  }
  params.telemetry_enabled = false;
  return true;
}

inline JsonMainDirectResponseStatus execute_playback(
  const char *command, const JsonMainDirectParameters &parameters,
  uint32_t request_id, size_t maximum_payload_bytes,
  char *output, size_t output_capacity
) {
  JsonMainMoveCartesianExecutionResult result = {};
  if (controller_mutation_estop_blocked()) {
    result.outcome = JsonMainMoveCartesianOutcome::kEmergencyStop;
    if (!capture_json_motion_position(result)) {
      result = {};
      result.outcome = JsonMainMoveCartesianOutcome::kPositionUnavailable;
    }
    return finish_motion_response(command, request_id, result, false,
      maximum_payload_bytes, output, output_capacity);
  }
  const String media(parameters.media_id);
  if (!initSD() || !verifyExpectedSDMediaId(media)) {
    return build_error_response(command, request_id,
      ar4_protocol::JsonErrorResponseStatus::kFailed,
      sd_ok ? "media_changed" : "storage_error", "SD playback is unavailable",
      maximum_payload_bytes, output, output_capacity)
      ? JsonMainDirectResponseStatus::kFailed : JsonMainDirectResponseStatus::kInvalid;
  }
  File file = SD.open(parameters.filename);
  if (!file) {
    return build_error_response(command, request_id,
      ar4_protocol::JsonErrorResponseStatus::kFailed, "storage_error",
      "SD program could not be opened", maximum_payload_bytes, output, output_capacity)
      ? JsonMainDirectResponseStatus::kFailed : JsonMainDirectResponseStatus::kInvalid;
  }
  String stored_row, row;
  bool playback_speed_limited = false;
  while (result.outcome == JsonMainMoveCartesianOutcome::kInvalid
      && file.available()) {
    if (read_stored_command_row(file, stored_row)
          != ar4_protocol::StoredRowReadStatus::kComplete
        || !ar4_protocol::extract_stored_command_payload(stored_row, row)) {
      result.outcome = JsonMainMoveCartesianOutcome::kMotionExecutionFailed;
      break;
    }
    ar4_protocol::JsonMainMoveCartesianParameters motion = {};
    if (!stored_cartesian_parameters(row, motion)
        || !execute_json_move_cartesian_with_timing(
          motion, result, nullptr, true, true)) {
      result.outcome = JsonMainMoveCartesianOutcome::kMotionExecutionFailed;
    }
    if (result.outcome == JsonMainMoveCartesianOutcome::kCompleted) {
      playback_speed_limited = playback_speed_limited || result.speed_limited;
      result = {};
    }
  }
  file.close();
  if (
    result.outcome == JsonMainMoveCartesianOutcome::kKinematicsUnreachable
    || result.outcome == JsonMainMoveCartesianOutcome::kJointLimitViolation
    || result.outcome
      == JsonMainMoveCartesianOutcome::kPositionNotRepresentable
  ) {
    result = {};
    result.outcome = JsonMainMoveCartesianOutcome::kMotionExecutionFailed;
  }
  if (result.outcome == JsonMainMoveCartesianOutcome::kInvalid)
    result.outcome = JsonMainMoveCartesianOutcome::kCompleted;
  if (!capture_json_motion_position(result)) {
    result = {}; result.outcome = JsonMainMoveCartesianOutcome::kPositionUnavailable;
  }
  if (result.outcome == JsonMainMoveCartesianOutcome::kCompleted)
    result.speed_limited = playback_speed_limited;
  if (!verifyExpectedSDMediaId(media)) {
    return build_error_response(command, request_id,
      ar4_protocol::JsonErrorResponseStatus::kFailed, "media_changed",
      "SD media identity changed during playback", maximum_payload_bytes,
      output, output_capacity)
      ? JsonMainDirectResponseStatus::kFailed : JsonMainDirectResponseStatus::kInvalid;
  }
  return finish_motion_response(command, request_id, result, false,
    maximum_payload_bytes, output, output_capacity);
}

inline JsonMainDirectResponseStatus execute(
  const char *command, const JsonMainDirectParameters &parameters,
  uint32_t request_id, size_t maximum_payload_bytes,
  char *output, size_t output_capacity, void *context
) {
  if (context != nullptr || command == nullptr) return JsonMainDirectResponseStatus::kInvalid;
  if (strcmp(command, "move_linear") == 0) return execute_linear(command,
    parameters, request_id, maximum_payload_bytes, output, output_capacity);
  if (strcmp(command, "move_arc") == 0) return execute_arc_or_circle(command,
    parameters, false, request_id, maximum_payload_bytes, output,
    output_capacity);
  if (strcmp(command, "move_circle") == 0) return execute_arc_or_circle(command,
    parameters, true, request_id, maximum_payload_bytes, output,
    output_capacity);
  if (strcmp(command, "move_spline") == 0) return execute_spline(command,
    parameters, request_id, maximum_payload_bytes, output, output_capacity);
  if (strcmp(command, "move_vision") == 0) return execute_vision(command,
    parameters, request_id, maximum_payload_bytes, output, output_capacity);
  if (strcmp(command, "wait_modbus_coil") == 0
      || strcmp(command, "wait_modbus_discrete_input") == 0
      || strcmp(command, "wait_modbus_holding_register") == 0)
    return execute_modbus_wait(command, parameters,
      strcmp(command, "wait_modbus_coil") == 0
        ? ar4_protocol::ModbusOperation::kReadCoil
        : strcmp(command, "wait_modbus_discrete_input") == 0
          ? ar4_protocol::ModbusOperation::kReadDiscreteInput
          : ar4_protocol::ModbusOperation::kReadHoldingRegisters,
      request_id, maximum_payload_bytes, output, output_capacity);
  if (strcmp(command, "delete_sd_program") == 0) return execute_delete(command,
    parameters, request_id, maximum_payload_bytes, output, output_capacity);
  if (strcmp(command, "list_sd_programs") == 0) return execute_list(command,
    request_id, maximum_payload_bytes, output, output_capacity);
  if (strcmp(command, "write_gcode_move") == 0) return execute_write_move(command,
    parameters, request_id, maximum_payload_bytes, output, output_capacity);
  if (strcmp(command, "play_gcode_file") == 0)
    return execute_playback(command, parameters, request_id,
      maximum_payload_bytes, output, output_capacity);
  return JsonMainDirectResponseStatus::kInvalid;
}

}  // namespace ar4_json_direct_runtime

#endif
