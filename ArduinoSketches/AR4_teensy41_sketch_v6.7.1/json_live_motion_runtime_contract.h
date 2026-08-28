#ifndef AR4_JSON_LIVE_MOTION_RUNTIME_CONTRACT_H
#define AR4_JSON_LIVE_MOTION_RUNTIME_CONTRACT_H

#include <stddef.h>
#include <stdint.h>

namespace ar4_protocol {

constexpr uint32_t kJsonLiveMotionLeaseMinimumMilliseconds = 1000;
constexpr uint32_t kJsonLiveMotionLeaseMaximumMilliseconds = 5000;
constexpr uint32_t kJsonLiveMotionControlServiceIntervalMicroseconds = 1000;
constexpr size_t kJsonLiveMotionControlReadBudgetBytes = 128;

enum class JsonLiveMotionRuntimeState : uint8_t {
  kIdle,
  kAwaitingAcceptance,
  kActive,
  kStopRequested,
  kLeaseExpired,
  kTerminalReady,
  kTerminalStaged,
};

struct JsonLiveMotionRuntimeOwner {
  JsonLiveMotionRuntimeState state;
  uint32_t motion_id;
  uint32_t lease_milliseconds;
  uint32_t lease_started_at_milliseconds;
};

using JsonLiveMotionContinuationCheck = bool (*)(void *);
using JsonLiveMotionControlService = bool (*)(void *);
using JsonLiveMotionMicrosecondsSource = uint32_t (*)(void *);
using JsonLiveMotionDelaySource = void (*)(uint32_t, void *);
using JsonLiveMotionControlInputPending = bool (*)(void *);
using JsonLiveMotionControlInputService = bool (*)(void *);

struct JsonLiveMotionContinuationSource {
  JsonLiveMotionContinuationCheck check;
  JsonLiveMotionControlService service_control;
  void *context;
};

struct JsonLiveMotionPulseClockSource {
  JsonLiveMotionMicrosecondsSource microseconds;
  JsonLiveMotionDelaySource delay_microseconds;
  void *context;
};

struct JsonLiveMotionControlInputSource {
  JsonLiveMotionControlInputPending pending;
  JsonLiveMotionControlInputService service_one;
  void *context;
};

inline bool json_live_motion_continuation_source_valid(
  const JsonLiveMotionContinuationSource *source
) {
  return source == nullptr
    || (source->check != nullptr && source->service_control != nullptr);
}

inline bool service_json_live_motion_continuation(
  const JsonLiveMotionContinuationSource *source
) {
  if (source == nullptr) return true;
  return json_live_motion_continuation_source_valid(source)
    && source->service_control(source->context)
    && source->check(source->context);
}

inline uint32_t json_live_motion_control_wait_slice_microseconds(
  uint32_t remaining_microseconds
) {
  return remaining_microseconds
      < kJsonLiveMotionControlServiceIntervalMicroseconds
    ? remaining_microseconds
    : kJsonLiveMotionControlServiceIntervalMicroseconds;
}

inline bool wait_json_live_motion_pulse(
  uint32_t wait_started_microseconds,
  uint32_t wait_microseconds,
  const JsonLiveMotionContinuationSource &continuation,
  const JsonLiveMotionPulseClockSource &clock
) {
  if (
    !json_live_motion_continuation_source_valid(&continuation)
    || clock.microseconds == nullptr
    || clock.delay_microseconds == nullptr
  ) {
    return false;
  }
  uint32_t elapsed = static_cast<uint32_t>(
    clock.microseconds(clock.context) - wait_started_microseconds
  );
  while (elapsed < wait_microseconds) {
    const uint32_t slice = json_live_motion_control_wait_slice_microseconds(
      wait_microseconds - elapsed
    );
    clock.delay_microseconds(slice, clock.context);
    if (!service_json_live_motion_continuation(&continuation)) return false;
    elapsed = static_cast<uint32_t>(
      clock.microseconds(clock.context) - wait_started_microseconds
    );
  }
  return true;
}

inline bool service_bounded_json_live_motion_control_input(
  const JsonLiveMotionControlInputSource &source,
  size_t maximum_bytes = kJsonLiveMotionControlReadBudgetBytes
) {
  if (
    source.pending == nullptr
    || source.service_one == nullptr
    || maximum_bytes == 0
    || maximum_bytes > kJsonLiveMotionControlReadBudgetBytes
  ) {
    return false;
  }
  size_t serviced = 0;
  while (serviced < maximum_bytes && source.pending(source.context)) {
    if (!source.service_one(source.context)) return false;
    ++serviced;
  }
  return true;
}

inline bool json_live_motion_interrupted_by_control(
  JsonLiveMotionRuntimeState state
) {
  return state == JsonLiveMotionRuntimeState::kStopRequested
    || state == JsonLiveMotionRuntimeState::kLeaseExpired;
}

enum class JsonLiveMotionServiceAction : uint8_t {
  kIdle,
  kWait,
  kAdvanceSegment,
  kSettleStop,
  kSettleLeaseExpiry,
  kSettleEmergencyStop,
  kStageTerminal,
  kClear,
  kFault,
};

inline bool json_live_motion_lease_valid(uint32_t lease_milliseconds) {
  return lease_milliseconds >= kJsonLiveMotionLeaseMinimumMilliseconds
    && lease_milliseconds <= kJsonLiveMotionLeaseMaximumMilliseconds;
}

inline bool json_live_motion_owner_empty(
  const JsonLiveMotionRuntimeOwner &owner
) {
  return owner.state == JsonLiveMotionRuntimeState::kIdle
    && owner.motion_id == 0
    && owner.lease_milliseconds == 0
    && owner.lease_started_at_milliseconds == 0;
}

inline void clear_json_live_motion_owner(
  JsonLiveMotionRuntimeOwner &owner
) {
  owner = {};
}

inline bool begin_json_live_motion(
  uint32_t motion_id,
  uint32_t lease_milliseconds,
  JsonLiveMotionRuntimeOwner &owner
) {
  if (
    motion_id == 0
    || !json_live_motion_lease_valid(lease_milliseconds)
    || !json_live_motion_owner_empty(owner)
  ) {
    return false;
  }
  owner.state = JsonLiveMotionRuntimeState::kAwaitingAcceptance;
  owner.motion_id = motion_id;
  owner.lease_milliseconds = lease_milliseconds;
  return true;
}

inline bool activate_json_live_motion(
  uint32_t motion_id,
  uint32_t now_milliseconds,
  JsonLiveMotionRuntimeOwner &owner
) {
  if (
    owner.state != JsonLiveMotionRuntimeState::kAwaitingAcceptance
    || owner.motion_id != motion_id
    || !json_live_motion_lease_valid(owner.lease_milliseconds)
  ) {
    return false;
  }
  owner.state = JsonLiveMotionRuntimeState::kActive;
  owner.lease_started_at_milliseconds = now_milliseconds;
  return true;
}

inline bool json_live_motion_lease_expired(
  uint32_t now_milliseconds,
  const JsonLiveMotionRuntimeOwner &owner
) {
  return owner.state == JsonLiveMotionRuntimeState::kActive
    && json_live_motion_lease_valid(owner.lease_milliseconds)
    && static_cast<uint32_t>(
      now_milliseconds - owner.lease_started_at_milliseconds
    ) >= owner.lease_milliseconds;
}

inline bool renew_json_live_motion(
  uint32_t motion_id,
  uint32_t now_milliseconds,
  JsonLiveMotionRuntimeOwner &owner
) {
  if (
    owner.state != JsonLiveMotionRuntimeState::kActive
    || owner.motion_id != motion_id
  ) {
    return false;
  }
  if (json_live_motion_lease_expired(now_milliseconds, owner)) {
    owner.state = JsonLiveMotionRuntimeState::kLeaseExpired;
    return false;
  }
  owner.lease_started_at_milliseconds = now_milliseconds;
  return true;
}

inline bool request_json_live_motion_stop(
  uint32_t motion_id,
  JsonLiveMotionRuntimeOwner &owner
) {
  if (
    owner.motion_id != motion_id
    || (
      owner.state != JsonLiveMotionRuntimeState::kActive
      && owner.state != JsonLiveMotionRuntimeState::kLeaseExpired
      && owner.state != JsonLiveMotionRuntimeState::kStopRequested
    )
  ) {
    return false;
  }
  owner.state = JsonLiveMotionRuntimeState::kStopRequested;
  return true;
}

inline bool json_live_motion_may_continue(
  uint32_t motion_id,
  uint32_t now_milliseconds,
  JsonLiveMotionRuntimeOwner &owner
) {
  if (
    owner.state != JsonLiveMotionRuntimeState::kActive
    || owner.motion_id != motion_id
  ) {
    return false;
  }
  if (json_live_motion_lease_expired(now_milliseconds, owner)) {
    owner.state = JsonLiveMotionRuntimeState::kLeaseExpired;
    return false;
  }
  return true;
}

inline bool mark_json_live_motion_terminal_ready(
  JsonLiveMotionRuntimeOwner &owner
) {
  switch (owner.state) {
    case JsonLiveMotionRuntimeState::kActive:
    case JsonLiveMotionRuntimeState::kStopRequested:
    case JsonLiveMotionRuntimeState::kLeaseExpired:
      owner.state = JsonLiveMotionRuntimeState::kTerminalReady;
      return true;
    case JsonLiveMotionRuntimeState::kIdle:
    case JsonLiveMotionRuntimeState::kAwaitingAcceptance:
    case JsonLiveMotionRuntimeState::kTerminalReady:
    case JsonLiveMotionRuntimeState::kTerminalStaged:
      return false;
  }
  return false;
}

inline bool mark_json_live_motion_terminal_staged(
  JsonLiveMotionRuntimeOwner &owner
) {
  if (owner.state != JsonLiveMotionRuntimeState::kTerminalReady) {
    return false;
  }
  owner.state = JsonLiveMotionRuntimeState::kTerminalStaged;
  return true;
}

inline JsonLiveMotionServiceAction plan_json_live_motion_service(
  uint32_t now_milliseconds,
  uint32_t active_motion_id,
  bool active_kind_matches,
  bool controller_owner_idle,
  bool runtime_fault,
  bool emergency_stop_active,
  bool control_frame_pending,
  bool advance_segment,
  JsonLiveMotionRuntimeOwner &owner
) {
  if (owner.state == JsonLiveMotionRuntimeState::kIdle) {
    return json_live_motion_owner_empty(owner)
      ? JsonLiveMotionServiceAction::kIdle
      : JsonLiveMotionServiceAction::kFault;
  }
  if (runtime_fault) return JsonLiveMotionServiceAction::kClear;
  if (owner.state == JsonLiveMotionRuntimeState::kAwaitingAcceptance) {
    if (active_motion_id == owner.motion_id && active_kind_matches) {
      return activate_json_live_motion(
        owner.motion_id,
        now_milliseconds,
        owner
      )
        ? JsonLiveMotionServiceAction::kWait
        : JsonLiveMotionServiceAction::kFault;
    }
    return active_motion_id == 0 && !controller_owner_idle
      ? JsonLiveMotionServiceAction::kWait
      : JsonLiveMotionServiceAction::kFault;
  }
  if (owner.state == JsonLiveMotionRuntimeState::kTerminalStaged) {
    if (active_motion_id == 0 && controller_owner_idle) {
      return JsonLiveMotionServiceAction::kClear;
    }
    return active_motion_id == owner.motion_id && active_kind_matches
      ? JsonLiveMotionServiceAction::kWait
      : JsonLiveMotionServiceAction::kFault;
  }
  if (
    active_motion_id != owner.motion_id
    || !active_kind_matches
  ) {
    return JsonLiveMotionServiceAction::kFault;
  }
  if (owner.state == JsonLiveMotionRuntimeState::kTerminalReady) {
    return controller_owner_idle && !control_frame_pending
      ? JsonLiveMotionServiceAction::kStageTerminal
      : JsonLiveMotionServiceAction::kWait;
  }
  if (json_live_motion_lease_expired(now_milliseconds, owner)) {
    owner.state = JsonLiveMotionRuntimeState::kLeaseExpired;
  }
  if (control_frame_pending) {
    return JsonLiveMotionServiceAction::kWait;
  }
  if (!controller_owner_idle) return JsonLiveMotionServiceAction::kWait;
  if (emergency_stop_active) {
    return JsonLiveMotionServiceAction::kSettleEmergencyStop;
  }
  if (
    owner.state == JsonLiveMotionRuntimeState::kLeaseExpired
  ) {
    return JsonLiveMotionServiceAction::kSettleLeaseExpiry;
  }
  if (owner.state == JsonLiveMotionRuntimeState::kStopRequested) {
    return JsonLiveMotionServiceAction::kSettleStop;
  }
  if (owner.state != JsonLiveMotionRuntimeState::kActive) {
    return JsonLiveMotionServiceAction::kFault;
  }
  return advance_segment
    ? JsonLiveMotionServiceAction::kAdvanceSegment
    : JsonLiveMotionServiceAction::kWait;
}

}  // namespace ar4_protocol

#endif
