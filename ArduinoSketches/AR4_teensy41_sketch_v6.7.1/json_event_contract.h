#ifndef AR4_JSON_EVENT_CONTRACT_H
#define AR4_JSON_EVENT_CONTRACT_H

#include <stddef.h>
#include <stdint.h>

#include "json_session_contract.h"

namespace ar4_protocol {

constexpr size_t kJsonEmergencyStopEventCapacity = 128;

enum class JsonEventOutputState : uint8_t {
  kIdle,
  kResponseReady,
  kWriteInProgress,
  kFaulted,
};

enum class JsonEventQueueStatus : uint8_t {
  kResponseReady,
  kBusy,
  kControllerFault,
  kSessionFaulted,
};

enum class JsonEventOutputBeginStatus : uint8_t {
  kReady,
  kNoResponse,
  kBusy,
  kSessionFaulted,
};

enum class JsonEventOutputCompletionStatus : uint8_t {
  kCompleted,
  kControllerFault,
  kSessionFaulted,
};

struct JsonEventFrameView {
  const char *data;
  size_t length;
};

inline bool build_json_emergency_stop_event(
  uint32_t sequence,
  bool asserted,
  size_t maximum_payload_bytes,
  char *output,
  size_t output_capacity
) {
  if (output != nullptr && output_capacity > 0) output[0] = '\0';
  if (
    output == nullptr
    || output_capacity == 0
    || maximum_payload_bytes == 0
    || maximum_payload_bytes > kJsonProtocolMaximumPayloadBytes
  ) {
    return false;
  }
  const size_t bounded_capacity =
    output_capacity < maximum_payload_bytes + 1
    ? output_capacity
    : maximum_payload_bytes + 1;
  size_t index = 0;
  const bool built = append_json_text(
      "{\"data\":{\"asserted\":",
      output,
      bounded_capacity,
      index
    )
    && append_json_text(
      asserted ? "true" : "false",
      output,
      bounded_capacity,
      index
    )
    && append_json_text(
      "},\"event\":\"emergency_stop\",\"seq\":",
      output,
      bounded_capacity,
      index
    )
    && append_json_uint32(
      sequence,
      output,
      bounded_capacity,
      index
    )
    && append_json_text(
      ",\"type\":\"event\",\"v\":1}",
      output,
      bounded_capacity,
      index
    );
  if (!built) output[0] = '\0';
  return built;
}

class JsonControllerEventOutputOwner {
 public:
  explicit JsonControllerEventOutputOwner(uint32_t initial_sequence = 0)
    : state_(JsonEventOutputState::kIdle),
      next_sequence_(initial_sequence),
      frame_length_(0) {
    output_[0] = '\0';
  }

  JsonControllerEventOutputOwner(
    const JsonControllerEventOutputOwner &
  ) = delete;
  JsonControllerEventOutputOwner &operator=(
    const JsonControllerEventOutputOwner &
  ) = delete;
  JsonControllerEventOutputOwner(
    JsonControllerEventOutputOwner &&
  ) = delete;
  JsonControllerEventOutputOwner &operator=(
    JsonControllerEventOutputOwner &&
  ) = delete;

  JsonEventQueueStatus queue_emergency_stop(
    bool asserted,
    size_t maximum_payload_bytes
  ) {
    switch (state_) {
      case JsonEventOutputState::kIdle:
        break;
      case JsonEventOutputState::kResponseReady:
      case JsonEventOutputState::kWriteInProgress:
        return JsonEventQueueStatus::kBusy;
      case JsonEventOutputState::kFaulted:
        return JsonEventQueueStatus::kSessionFaulted;
    }
    if (!build_json_emergency_stop_event(
        next_sequence_,
        asserted,
        maximum_payload_bytes,
        output_,
        sizeof(output_)
    )) {
      fault();
      return JsonEventQueueStatus::kControllerFault;
    }
    size_t payload_length = 0;
    while (
      payload_length < sizeof(output_)
      && output_[payload_length] != '\0'
    ) {
      ++payload_length;
    }
    if (
      payload_length == 0
      || payload_length > maximum_payload_bytes
      || payload_length + 2 > sizeof(output_)
    ) {
      fault();
      return JsonEventQueueStatus::kControllerFault;
    }
    output_[payload_length] = '\n';
    output_[payload_length + 1] = '\0';
    frame_length_ = payload_length + 1;
    state_ = JsonEventOutputState::kResponseReady;
    return JsonEventQueueStatus::kResponseReady;
  }

  JsonEventOutputBeginStatus begin_write(JsonEventFrameView &view) {
    view.data = nullptr;
    view.length = 0;
    switch (state_) {
      case JsonEventOutputState::kIdle:
        return JsonEventOutputBeginStatus::kNoResponse;
      case JsonEventOutputState::kResponseReady:
        view.data = output_;
        view.length = frame_length_;
        state_ = JsonEventOutputState::kWriteInProgress;
        return JsonEventOutputBeginStatus::kReady;
      case JsonEventOutputState::kWriteInProgress:
        return JsonEventOutputBeginStatus::kBusy;
      case JsonEventOutputState::kFaulted:
        return JsonEventOutputBeginStatus::kSessionFaulted;
    }
    fault();
    return JsonEventOutputBeginStatus::kSessionFaulted;
  }

  JsonEventOutputCompletionStatus complete_write(size_t written_bytes) {
    if (state_ == JsonEventOutputState::kFaulted) {
      return JsonEventOutputCompletionStatus::kSessionFaulted;
    }
    if (
      state_ != JsonEventOutputState::kWriteInProgress
      || written_bytes != frame_length_
    ) {
      fault();
      return JsonEventOutputCompletionStatus::kControllerFault;
    }
    output_[0] = '\0';
    frame_length_ = 0;
    next_sequence_ = next_sequence_ == UINT32_MAX
      ? 0
      : next_sequence_ + 1;
    state_ = JsonEventOutputState::kIdle;
    return JsonEventOutputCompletionStatus::kCompleted;
  }

  JsonEventOutputState state() const {
    return state_;
  }

  uint32_t next_sequence() const {
    return next_sequence_;
  }

 private:
  void fault() {
    output_[0] = '\0';
    frame_length_ = 0;
    state_ = JsonEventOutputState::kFaulted;
  }

  JsonEventOutputState state_;
  uint32_t next_sequence_;
  size_t frame_length_;
  char output_[kJsonEmergencyStopEventCapacity];
};

}  // namespace ar4_protocol

#endif
