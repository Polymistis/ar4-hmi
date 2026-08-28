#ifndef AR4_JSON_CONTROLLER_CONFIGURATION_CONTRACT_H
#define AR4_JSON_CONTROLLER_CONFIGURATION_CONTRACT_H

#include <math.h>
#include <stddef.h>
#include <stdint.h>

#include "angle_conversion_contract.h"
#include "controller_domain_contract.h"

namespace ar4_protocol {

constexpr size_t kJsonPrimaryJointCount = 6;
constexpr size_t kJsonControllerAxisCount = 9;
constexpr size_t kJsonExternalAxisCount = 3;

struct JsonMainUpdateParameters {
  float tool_translation_millimeters[3];
  float tool_rotation_degrees[3];
  int motor_directions[kJsonControllerAxisCount];
  int calibration_directions[kJsonControllerAxisCount];
  bool calibration_switch_active_high[kJsonControllerAxisCount];
  float positive_joint_limits_degrees[kJsonPrimaryJointCount];
  float negative_joint_limits_degrees[kJsonPrimaryJointCount];
  float steps_per_degree[kJsonPrimaryJointCount];
  float encoder_counts_per_step[kJsonPrimaryJointCount];
  float dh_theta_degrees[kJsonPrimaryJointCount];
  float dh_alpha_degrees[kJsonPrimaryJointCount];
  float dh_d_millimeters[kJsonPrimaryJointCount];
  float dh_a_millimeters[kJsonPrimaryJointCount];
};

struct JsonMainUpdateParametersValidation {
  float tool_rotation_radians[3];
  float dh_theta_radians[kJsonPrimaryJointCount];
  float dh_alpha_radians[kJsonPrimaryJointCount];
  int step_limits[kJsonPrimaryJointCount];
  int zero_steps[kJsonPrimaryJointCount];
};

inline bool validate_json_main_update_parameters(
  const JsonMainUpdateParameters &params,
  JsonMainUpdateParametersValidation &output
) {
  JsonMainUpdateParametersValidation staged = {};
  for (size_t axis = 0; axis < 3; ++axis) {
    if (
      !isfinite(params.tool_translation_millimeters[axis])
      || !degrees_to_radians(
        params.tool_rotation_degrees[axis],
        staged.tool_rotation_radians[axis]
      )
    ) {
      return false;
    }
  }
  for (size_t axis = 0; axis < kJsonControllerAxisCount; ++axis) {
    if (
      (params.motor_directions[axis] != 0
        && params.motor_directions[axis] != 1)
      || (params.calibration_directions[axis] != 0
        && params.calibration_directions[axis] != 1)
    ) {
      return false;
    }
  }
  for (size_t axis = 0; axis < kJsonPrimaryJointCount; ++axis) {
    if (
      !validate_primary_axis_calibration(
        params.negative_joint_limits_degrees[axis],
        params.positive_joint_limits_degrees[axis],
        params.steps_per_degree[axis],
        staged.step_limits[axis],
        staged.zero_steps[axis]
      )
      || !validate_encoder_calibration(
        staged.step_limits[axis],
        params.encoder_counts_per_step[axis]
      )
      || !degrees_to_radians(
        params.dh_theta_degrees[axis],
        staged.dh_theta_radians[axis]
      )
      || !degrees_to_radians(
        params.dh_alpha_degrees[axis],
        staged.dh_alpha_radians[axis]
      )
      || !isfinite(params.dh_d_millimeters[axis])
      || !isfinite(params.dh_a_millimeters[axis])
    ) {
      return false;
    }
  }
  output = staged;
  return true;
}

struct JsonMainUpdateConfiguration {
  float tool[6];
  int motor_directions[kJsonControllerAxisCount];
  int calibration_directions[kJsonControllerAxisCount];
  bool calibration_switch_active_high[kJsonControllerAxisCount];
  float positive_joint_limits_degrees[kJsonPrimaryJointCount];
  float negative_joint_limits_degrees[kJsonPrimaryJointCount];
  float joint_travel_degrees[kJsonPrimaryJointCount];
  float steps_per_degree[kJsonPrimaryJointCount];
  float encoder_counts_per_step[kJsonPrimaryJointCount];
  float dh_degrees[kJsonPrimaryJointCount][4];
  float dh_theta_radians[kJsonPrimaryJointCount];
  float dh_alpha_radians[kJsonPrimaryJointCount];
  int step_limits[kJsonPrimaryJointCount];
  int zero_steps[kJsonPrimaryJointCount];
};

inline bool build_json_main_update_configuration(
  const JsonMainUpdateParameters &params,
  JsonMainUpdateConfiguration &output
) {
  JsonMainUpdateParametersValidation validation = {};
  if (!validate_json_main_update_parameters(params, validation)) {
    return false;
  }
  JsonMainUpdateConfiguration staged = {};
  for (size_t axis = 0; axis < 3; ++axis) {
    staged.tool[axis] = params.tool_translation_millimeters[axis];
    staged.tool[axis + 3] = validation.tool_rotation_radians[axis];
  }
  for (size_t axis = 0; axis < kJsonControllerAxisCount; ++axis) {
    staged.motor_directions[axis] = params.motor_directions[axis];
    staged.calibration_directions[axis] =
      params.calibration_directions[axis];
    staged.calibration_switch_active_high[axis] =
      params.calibration_switch_active_high[axis];
  }
  for (size_t axis = 0; axis < kJsonPrimaryJointCount; ++axis) {
    staged.positive_joint_limits_degrees[axis] =
      params.positive_joint_limits_degrees[axis];
    staged.negative_joint_limits_degrees[axis] =
      params.negative_joint_limits_degrees[axis];
    staged.joint_travel_degrees[axis] =
      params.positive_joint_limits_degrees[axis]
      + params.negative_joint_limits_degrees[axis];
    staged.steps_per_degree[axis] = params.steps_per_degree[axis];
    staged.encoder_counts_per_step[axis] =
      params.encoder_counts_per_step[axis];
    staged.dh_degrees[axis][0] = params.dh_theta_degrees[axis];
    staged.dh_degrees[axis][1] = params.dh_alpha_degrees[axis];
    staged.dh_degrees[axis][2] = params.dh_d_millimeters[axis];
    staged.dh_degrees[axis][3] = params.dh_a_millimeters[axis];
    staged.dh_theta_radians[axis] = validation.dh_theta_radians[axis];
    staged.dh_alpha_radians[axis] = validation.dh_alpha_radians[axis];
    staged.step_limits[axis] = validation.step_limits[axis];
    staged.zero_steps[axis] = validation.zero_steps[axis];
  }
  output = staged;
  return true;
}

struct JsonMainExternalAxisParameters {
  float travel_units[kJsonExternalAxisCount];
  float drive_rotations[kJsonExternalAxisCount];
  float motor_steps[kJsonExternalAxisCount];
};

struct JsonMainExternalAxisValidation {
  ExternalAxisCalibration axes[kJsonExternalAxisCount];
};

inline bool validate_json_main_external_axis_parameters(
  const JsonMainExternalAxisParameters &params,
  JsonMainExternalAxisValidation &output
) {
  JsonMainExternalAxisValidation staged = {};
  for (size_t axis = 0; axis < kJsonExternalAxisCount; ++axis) {
    if (!validate_external_axis_calibration(
        params.travel_units[axis],
        params.drive_rotations[axis],
        params.motor_steps[axis],
        staged.axes[axis]
    )) {
      return false;
    }
  }
  output = staged;
  return true;
}

struct JsonMainExternalAxisConfiguration {
  float travel_units[kJsonExternalAxisCount];
  float drive_rotations[kJsonExternalAxisCount];
  float motor_steps[kJsonExternalAxisCount];
  ExternalAxisCalibration axes[kJsonExternalAxisCount];
};

inline bool build_json_main_external_axis_configuration(
  const JsonMainExternalAxisParameters &params,
  JsonMainExternalAxisConfiguration &output
) {
  JsonMainExternalAxisValidation validation = {};
  if (!validate_json_main_external_axis_parameters(params, validation)) {
    return false;
  }
  JsonMainExternalAxisConfiguration staged = {};
  for (size_t axis = 0; axis < kJsonExternalAxisCount; ++axis) {
    staged.travel_units[axis] = params.travel_units[axis];
    staged.drive_rotations[axis] = params.drive_rotations[axis];
    staged.motor_steps[axis] = params.motor_steps[axis];
    staged.axes[axis] = validation.axes[axis];
  }
  output = staged;
  return true;
}

}  // namespace ar4_protocol

#endif
