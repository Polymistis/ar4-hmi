#ifndef AR4_CALIBRATION_SWITCH_CONTRACT_H
#define AR4_CALIBRATION_SWITCH_CONTRACT_H

#include <stddef.h>
#include <stdint.h>

namespace ar4_protocol {

constexpr size_t kCalibrationSwitchCount = 9;
constexpr int kCalibrationSwitchMaskMaximum =
  (1 << kCalibrationSwitchCount) - 1;

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
