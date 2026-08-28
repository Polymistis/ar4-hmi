#ifndef AR4_JSON_RUNTIME_CONTRACT_H
#define AR4_JSON_RUNTIME_CONTRACT_H

#include <math.h>
#include <stddef.h>
#include <stdint.h>

#include "json_session_contract.h"

namespace ar4_protocol {

enum class JsonOutputProgressStatus : uint8_t {
  kBlocked,
  kProgress,
  kCompleted,
  kInvalid,
};

enum class JsonOutputRoute : uint8_t {
  kIdle,
  kFaultSignal,
  kFaulted,
  kResponse,
  kEvent,
  kInvalid,
};

inline JsonOutputProgressStatus plan_json_output_chunk(
  size_t frame_length,
  size_t offset,
  int available_bytes,
  size_t &admitted_bytes
) {
  admitted_bytes = 0;
  if (frame_length == 0 || offset > frame_length) {
    return JsonOutputProgressStatus::kInvalid;
  }
  if (offset == frame_length) {
    return JsonOutputProgressStatus::kCompleted;
  }
  if (available_bytes <= 0) {
    return JsonOutputProgressStatus::kBlocked;
  }
  const size_t remaining = frame_length - offset;
  const size_t available = static_cast<size_t>(available_bytes);
  admitted_bytes = available < remaining ? available : remaining;
  return JsonOutputProgressStatus::kProgress;
}

inline JsonOutputProgressStatus record_json_output_chunk(
  size_t frame_length,
  size_t admitted_bytes,
  size_t written_bytes,
  size_t &offset
) {
  if (
    frame_length == 0
    || offset >= frame_length
    || admitted_bytes == 0
    || admitted_bytes > frame_length - offset
    || written_bytes > admitted_bytes
  ) {
    return JsonOutputProgressStatus::kInvalid;
  }
  if (written_bytes == 0) {
    return JsonOutputProgressStatus::kBlocked;
  }
  offset += written_bytes;
  return offset == frame_length
    ? JsonOutputProgressStatus::kCompleted
    : JsonOutputProgressStatus::kProgress;
}

inline JsonOutputRoute select_json_output_route(
  bool fault_signal_owned,
  bool runtime_fault,
  bool response_write_started,
  bool event_write_started,
  bool response_owned,
  bool event_owned
) {
  if (fault_signal_owned) return JsonOutputRoute::kFaultSignal;
  if (runtime_fault) return JsonOutputRoute::kFaulted;
  if (response_write_started && event_write_started) {
    return JsonOutputRoute::kInvalid;
  }
  if (response_write_started) {
    return response_owned
      ? JsonOutputRoute::kResponse
      : JsonOutputRoute::kInvalid;
  }
  if (event_write_started) {
    return event_owned
      ? JsonOutputRoute::kEvent
      : JsonOutputRoute::kInvalid;
  }
  if (response_owned) return JsonOutputRoute::kResponse;
  if (event_owned) return JsonOutputRoute::kEvent;
  return JsonOutputRoute::kIdle;
}

inline bool scale_json_position_value(
  float value,
  double scale,
  int32_t &output
) {
  if (!isfinite(value) || !isfinite(scale) || scale <= 0.0) {
    return false;
  }
  const double scaled = static_cast<double>(value) * scale;
  if (
    !isfinite(scaled)
    || scaled < static_cast<double>(INT32_MIN)
    || scaled > static_cast<double>(INT32_MAX)
  ) {
    return false;
  }
  const double rounded = scaled >= 0.0
    ? floor(scaled + 0.5)
    : ceil(scaled - 0.5);
  if (
    rounded < static_cast<double>(INT32_MIN)
    || rounded > static_cast<double>(INT32_MAX)
  ) {
    return false;
  }
  output = static_cast<int32_t>(rounded);
  return true;
}

inline bool build_json_main_position_snapshot(
  const float (&robot_joints)[6],
  const float (&external_axes)[3],
  const float (&cartesian)[3],
  const float (&orientation)[3],
  JsonMainPositionSnapshot &snapshot
) {
  JsonMainPositionSnapshot staged = {};
  for (size_t index = 0; index < 6; ++index) {
    if (!scale_json_position_value(
        robot_joints[index],
        1000.0,
        staged.robot_joints_millidegrees[index]
    )) {
      return false;
    }
  }
  for (size_t index = 0; index < 3; ++index) {
    if (
      !scale_json_position_value(
        external_axes[index],
        1000.0,
        staged.external_axes_milliunits[index]
      )
      || !scale_json_position_value(
        cartesian[index],
        1000.0,
        staged.cartesian_micrometers[index]
      )
      || !scale_json_position_value(
        orientation[index],
        1000.0,
        staged.orientation_millidegrees[index]
      )
    ) {
      return false;
    }
  }
  snapshot = staged;
  return true;
}

}  // namespace ar4_protocol

#endif
