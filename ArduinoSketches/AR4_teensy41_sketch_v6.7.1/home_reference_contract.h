#ifndef AR4_HOME_REFERENCE_CONTRACT_H
#define AR4_HOME_REFERENCE_CONTRACT_H

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

namespace ar4_protocol {

constexpr size_t kPrimaryHomeReferenceAxisCount = 3;
constexpr size_t kPrimaryHomeReferenceV1ResponseCapacity = 64;
constexpr size_t kPrimaryHomeReferenceV2ResponseCapacity = 64;

struct PrimaryHomeReferenceState {
  bool valid[kPrimaryHomeReferenceAxisCount];
  int32_t millidegrees[kPrimaryHomeReferenceAxisCount];
};

inline bool invalidate_primary_home_reference_axis(
  PrimaryHomeReferenceState &state,
  size_t axis
) {
  if (axis >= kPrimaryHomeReferenceAxisCount) return false;
  state.valid[axis] = false;
  state.millidegrees[axis] = 0;
  return true;
}

inline void invalidate_primary_home_reference(
  PrimaryHomeReferenceState &state
) {
  for (size_t axis = 0; axis < kPrimaryHomeReferenceAxisCount; ++axis) {
    invalidate_primary_home_reference_axis(state, axis);
  }
}

inline bool primary_home_reference_millidegrees(
  float degrees,
  int32_t &millidegrees
) {
  if (!isfinite(degrees)) return false;
  const double scaled = static_cast<double>(degrees) * 1000.0;
  if (
    scaled < static_cast<double>(INT32_MIN)
    || scaled > static_cast<double>(INT32_MAX)
  ) {
    return false;
  }
  const long rounded = lround(scaled);
  if (
    rounded < static_cast<long>(INT32_MIN)
    || rounded > static_cast<long>(INT32_MAX)
  ) {
    return false;
  }
  millidegrees = static_cast<int32_t>(rounded);
  return true;
}

inline bool set_primary_home_reference(
  PrimaryHomeReferenceState &state,
  size_t axis,
  int32_t millidegrees
) {
  if (axis >= kPrimaryHomeReferenceAxisCount) return false;
  state.valid[axis] = true;
  state.millidegrees[axis] = millidegrees;
  return true;
}

inline bool build_primary_home_reference_v1_response(
  const PrimaryHomeReferenceState &state,
  char *output,
  size_t output_capacity
) {
  if (output == nullptr || output_capacity == 0) return false;
  const long first_position = state.valid[0]
    ? static_cast<long>(state.millidegrees[0])
    : 0L;
  const long second_position = state.valid[1]
    ? static_cast<long>(state.millidegrees[1])
    : 0L;
  const int written = snprintf(
    output,
    output_capacity,
    "A%dB%ldC%dD%ld",
    state.valid[0] ? 1 : 0,
    first_position,
    state.valid[1] ? 1 : 0,
    second_position
  );
  return (
    written > 0
    && static_cast<size_t>(written) < output_capacity
  );
}

inline bool build_primary_home_reference_v2_response(
  const PrimaryHomeReferenceState &state,
  char *output,
  size_t output_capacity
) {
  if (output == nullptr || output_capacity == 0) return false;
  const long first_position = state.valid[0]
    ? static_cast<long>(state.millidegrees[0])
    : 0L;
  const long second_position = state.valid[1]
    ? static_cast<long>(state.millidegrees[1])
    : 0L;
  const long third_position = state.valid[2]
    ? static_cast<long>(state.millidegrees[2])
    : 0L;
  const int written = snprintf(
    output,
    output_capacity,
    "A%dB%ldC%dD%ldE%dF%ld",
    state.valid[0] ? 1 : 0,
    first_position,
    state.valid[1] ? 1 : 0,
    second_position,
    state.valid[2] ? 1 : 0,
    third_position
  );
  return (
    written > 0
    && static_cast<size_t>(written) < output_capacity
  );
}

}

#endif
