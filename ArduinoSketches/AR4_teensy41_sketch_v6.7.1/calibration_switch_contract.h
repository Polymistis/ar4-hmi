#ifndef AR4_CALIBRATION_SWITCH_CONTRACT_H
#define AR4_CALIBRATION_SWITCH_CONTRACT_H

#include <stddef.h>
#include <stdint.h>

namespace ar4_protocol {

constexpr size_t kCalibrationSwitchCount = 9;
constexpr int kCalibrationSwitchMaskMaximum =
  (1 << kCalibrationSwitchCount) - 1;
constexpr uint32_t kCalibrationSwitchStableMicroseconds = 3000U;
constexpr uint32_t kCalibrationLimitSearchLoopDelayMicroseconds = 100U;
constexpr uint32_t kCalibrationStepPulseMicroseconds = 5U;
constexpr float kCalibrationFastSearchSpeedPercent = 25.0f;
constexpr float kCalibrationSlowSearchSpeedPercent = 2.0f;
constexpr float kCalibrationReleaseSpeedPercent = 5.0f;
constexpr float kCalibrationReleaseMaximumTravelUnits = 10.0f;
constexpr float kCalibrationCenterSpeedPercent = 100.0f;
constexpr float kCalibrationCenterAccelerationPercent = 10.0f;
constexpr float kCalibrationCenterDecelerationPercent = 10.0f;
constexpr float kCalibrationCenterRampPercent = 50.0f;
constexpr float kCalibrationCenterMaximumDelayMultiplier = 5.0f;
static_assert(
  kCalibrationCenterMaximumDelayMultiplier
    == kCalibrationCenterRampPercent / 10.0f,
  "calibration center delay bound must match the ramp profile"
);

inline bool calibration_stage_maximum_iterations(
  int64_t maximum_progress_steps,
  uint32_t minimum_loop_delay_microseconds,
  uint64_t &maximum_iterations
) {
  if (
    maximum_progress_steps < 0
    || minimum_loop_delay_microseconds == 0
  ) {
    return false;
  }
  // A candidate interval either settles or returns to a progress-producing
  // sample. Budget every possible pre-progress interval plus the final
  // completion rescan so switch chatter cannot retain motor-loop ownership.
  const uint64_t candidate_iterations_per_progress =
    (
      static_cast<uint64_t>(kCalibrationSwitchStableMicroseconds)
      + minimum_loop_delay_microseconds - 1U
    ) / minimum_loop_delay_microseconds;
  const uint64_t progress_intervals =
    static_cast<uint64_t>(maximum_progress_steps) + 1U;
  const uint64_t iterations_per_progress =
    candidate_iterations_per_progress + 1U;
  if (
    progress_intervals
      > (UINT64_MAX - 1U) / iterations_per_progress
  ) {
    return false;
  }
  maximum_iterations =
    progress_intervals * iterations_per_progress + 1U;
  return true;
}

inline bool calibration_switch_mask_valid(int mask) {
  return mask >= 0 && mask <= kCalibrationSwitchMaskMaximum;
}

inline bool decode_calibration_switch_mask(
  int mask,
  uint8_t *active_states,
  size_t count
) {
  if (
    !calibration_switch_mask_valid(mask)
    || active_states == nullptr
    || count != kCalibrationSwitchCount
  ) {
    return false;
  }
  for (size_t axis = 0; axis < count; ++axis) {
    active_states[axis] = static_cast<uint8_t>((mask >> axis) & 1);
  }
  return true;
}

template <size_t Count>
inline bool decode_calibration_switch_mask(
  int mask,
  uint8_t (&active_states)[Count]
) {
  return decode_calibration_switch_mask(mask, active_states, Count);
}

inline bool calibration_switch_is_active(
  int sampled_state,
  uint8_t active_state
) {
  return (
    (sampled_state == 0 || sampled_state == 1)
    && (active_state == 0 || active_state == 1)
    && sampled_state == active_state
  );
}

inline bool calibration_switch_is_released(
  int sampled_state,
  uint8_t active_state
) {
  return (
    (sampled_state == 0 || sampled_state == 1)
    && (active_state == 0 || active_state == 1)
    && sampled_state != active_state
  );
}

}  // namespace ar4_protocol

#endif
