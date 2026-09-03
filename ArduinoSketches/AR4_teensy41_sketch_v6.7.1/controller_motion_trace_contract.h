#ifndef AR4_CONTROLLER_MOTION_TRACE_CONTRACT_H
#define AR4_CONTROLLER_MOTION_TRACE_CONTRACT_H

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "json_joint_motion_contract.h"
#include "json_session_contract.h"

namespace ar4_protocol {

constexpr size_t kControllerMotionTraceBufferBytes = 65536;
constexpr size_t kControllerMotionTraceRecordCapacity = 1024;
constexpr size_t kControllerMotionTracePageRecords = 8;
constexpr uint8_t kControllerMotionTraceFlagClockWrapped = 0x01;
constexpr uint8_t kControllerMotionTraceFlagTimingOverrun = 0x02;

struct alignas(4) ControllerMotionTraceRecord {
  uint32_t controller_microseconds;
  uint32_t master_index;
  uint32_t scheduled_delay_microseconds;
  int32_t commanded_steps[6];
  int32_t encoder_counts[6];
  uint8_t phase;
  uint8_t flags;
};

static_assert(
  sizeof(ControllerMotionTraceRecord) == 64,
  "controller motion-trace records must remain 64 bytes"
);
static_assert(
  kControllerMotionTraceRecordCapacity
      * sizeof(ControllerMotionTraceRecord)
    == kControllerMotionTraceBufferBytes,
  "controller motion-trace storage must remain 64 KiB"
);

struct JsonMainMotionTraceParameters {
  uint32_t motion_request_id;
  uint32_t page_index;
};

enum class ControllerMotionTraceOutcome : uint8_t {
  kInvalid,
  kCompleted,
  kFailed,
  kCancelled,
};

inline const char *controller_motion_trace_outcome_name(
  ControllerMotionTraceOutcome outcome
) {
  switch (outcome) {
    case ControllerMotionTraceOutcome::kCompleted:
      return "completed";
    case ControllerMotionTraceOutcome::kFailed:
      return "failed";
    case ControllerMotionTraceOutcome::kCancelled:
      return "cancelled";
    case ControllerMotionTraceOutcome::kInvalid:
      return nullptr;
  }
  return nullptr;
}

constexpr uint32_t controller_motion_trace_sample_index(
  uint32_t slot,
  uint32_t master_ticks,
  uint32_t record_count
) {
  return record_count <= 1
    ? 0
    : static_cast<uint32_t>(
        static_cast<uint64_t>(slot) * master_ticks / (record_count - 1)
      );
}

static_assert(
  controller_motion_trace_sample_index(0, 2000, 1024) == 0,
  "capacity-limited capture must retain the first state"
);
static_assert(
  controller_motion_trace_sample_index(1023, 2000, 1024) == 2000,
  "capacity-limited capture must retain the final state"
);

class ControllerMotionTraceCapture {
 public:
  ControllerMotionTraceCapture(void *storage, size_t storage_bytes)
    : records_(
        storage_bytes == kControllerMotionTraceBufferBytes
          && reinterpret_cast<uintptr_t>(storage)
              % alignof(ControllerMotionTraceRecord) == 0
        ? static_cast<ControllerMotionTraceRecord *>(storage)
        : nullptr
      ) {
    reset();
  }

  void reset() {
    capturing_ = false;
    available_ = false;
    complete_ = false;
    capacity_limited_ = false;
    generation_ = 0;
    source_motion_request_id_ = 0;
    master_ticks_ = 0;
    target_record_count_ = 0;
    record_count_ = 0;
    start_clock_ = 0;
    last_clock_ = 0;
    sticky_flags_ = 0;
    outcome_ = ControllerMotionTraceOutcome::kInvalid;
    source_session_id_[0] = '\0';
    configuration_fingerprint_[0] = '\0';
  }

  bool begin(
    uint32_t source_motion_request_id,
    const char *source_session_id,
    const char *configuration_fingerprint,
    uint32_t master_ticks,
    uint32_t controller_clock
  ) {
    if (
      records_ == nullptr
      || source_motion_request_id == 0
      || !json_session_identifier_valid(source_session_id)
      || !json_joint_motion_detail::configuration_fingerprint_valid(
        configuration_fingerprint
      )
    ) return false;
    generation_ = generation_ == UINT32_MAX ? 1 : generation_ + 1;
    source_motion_request_id_ = source_motion_request_id;
    master_ticks_ = master_ticks;
    const uint64_t observable_states =
      static_cast<uint64_t>(master_ticks) + 1U;
    target_record_count_ = observable_states
        > kControllerMotionTraceRecordCapacity
      ? kControllerMotionTraceRecordCapacity
      : static_cast<uint32_t>(observable_states);
    record_count_ = 0;
    start_clock_ = controller_clock;
    last_clock_ = controller_clock;
    sticky_flags_ = 0;
    capacity_limited_ =
      observable_states > kControllerMotionTraceRecordCapacity;
    complete_ = false;
    available_ = false;
    capturing_ = true;
    outcome_ = ControllerMotionTraceOutcome::kInvalid;
    memcpy(
      source_session_id_,
      source_session_id,
      kJsonSessionIdentifierLength + 1
    );
    memcpy(
      configuration_fingerprint_,
      configuration_fingerprint,
      kJsonConfigurationFingerprintCapacity
    );
    return true;
  }

  void record_state(
    uint32_t master_index,
    uint32_t scheduled_delay_microseconds,
    const int32_t (&commanded_steps)[6],
    const int32_t (&encoder_counts)[6],
    uint8_t phase,
    uint32_t controller_clock
  ) {
    if (!needs_state(master_index)) return;
    if (controller_clock < last_clock_) {
      sticky_flags_ |= kControllerMotionTraceFlagClockWrapped;
    }
    last_clock_ = controller_clock;
    ControllerMotionTraceRecord &record = records_[record_count_++];
    record.controller_microseconds = controller_clock - start_clock_;
    record.master_index = master_index;
    record.scheduled_delay_microseconds = scheduled_delay_microseconds;
    for (size_t axis = 0; axis < 6; ++axis) {
      record.commanded_steps[axis] = commanded_steps[axis];
      record.encoder_counts[axis] = encoder_counts[axis];
    }
    record.phase = phase;
    record.flags = sticky_flags_;
  }

  bool needs_state(uint32_t master_index) const {
    return capturing_
      && record_count_ < target_record_count_
      && controller_motion_trace_sample_index(
          record_count_, master_ticks_, target_record_count_
        ) == master_index;
  }

  void note_timing_overrun() {
    if (!capturing_) return;
    sticky_flags_ |= kControllerMotionTraceFlagTimingOverrun;
    if (record_count_ > 0) records_[record_count_ - 1].flags = sticky_flags_;
  }

  void finalize(ControllerMotionTraceOutcome outcome) {
    if (!capturing_) return;
    capturing_ = false;
    outcome_ = outcome;
    complete_ = record_count_ == target_record_count_
      && outcome != ControllerMotionTraceOutcome::kInvalid;
    available_ = record_count_ > 0
      && outcome != ControllerMotionTraceOutcome::kInvalid;
  }

  bool available_for(uint32_t motion_request_id) const {
    return available_ && source_motion_request_id_ == motion_request_id;
  }

  bool page_valid(uint32_t page_index) const {
    return available_
      && page_index < page_count();
  }

  uint32_t page_count() const {
    return record_count_ == 0
      ? 0
      : (record_count_ + kControllerMotionTracePageRecords - 1)
          / kControllerMotionTracePageRecords;
  }

  const ControllerMotionTraceRecord *records() const { return records_; }
  uint32_t record_count() const { return record_count_; }
  uint32_t generation() const { return generation_; }
  uint32_t source_motion_request_id() const {
    return source_motion_request_id_;
  }
  const char *source_session_id() const { return source_session_id_; }
  const char *configuration_fingerprint() const {
    return configuration_fingerprint_;
  }
  bool complete() const { return complete_; }
  bool capacity_limited() const { return capacity_limited_; }
  bool clock_wrapped() const {
    return (sticky_flags_ & kControllerMotionTraceFlagClockWrapped) != 0;
  }
  bool timing_overrun() const {
    return (sticky_flags_ & kControllerMotionTraceFlagTimingOverrun) != 0;
  }
  ControllerMotionTraceOutcome outcome() const { return outcome_; }

 private:
  ControllerMotionTraceRecord *records_;
  bool capturing_;
  bool available_;
  bool complete_;
  bool capacity_limited_;
  uint32_t generation_;
  uint32_t source_motion_request_id_;
  uint32_t master_ticks_;
  uint32_t target_record_count_;
  uint32_t record_count_;
  uint32_t start_clock_;
  uint32_t last_clock_;
  uint8_t sticky_flags_;
  ControllerMotionTraceOutcome outcome_;
  char source_session_id_[kJsonSessionIdentifierLength + 1];
  char configuration_fingerprint_[kJsonConfigurationFingerprintCapacity];
};

struct ControllerMotionTraceRequest {
  ControllerMotionTraceCapture *capture;
  uint32_t source_motion_request_id;
  const char *source_session_id;
  const char *configuration_fingerprint;
};

inline bool extract_main_motion_trace_parameters(
  ArduinoJson::JsonObjectConst params,
  JsonMainMotionTraceParameters &output
) {
  if (params.size() != 2) return false;
  bool motion_id_present = false;
  bool page_index_present = false;
  JsonMainMotionTraceParameters staged = {};
  for (ArduinoJson::JsonPairConst pair : params) {
    const ArduinoJson::JsonString key = pair.key();
    if (json_joint_motion_detail::string_equals(key, "motion_request_id")) {
      if (
        motion_id_present
        || !pair.value().is<uint32_t>()
        || pair.value().as<uint32_t>() == 0
      ) return false;
      staged.motion_request_id = pair.value().as<uint32_t>();
      motion_id_present = true;
    } else if (json_joint_motion_detail::string_equals(key, "page_index")) {
      if (page_index_present || !pair.value().is<uint32_t>()) return false;
      staged.page_index = pair.value().as<uint32_t>();
      page_index_present = true;
    } else {
      return false;
    }
  }
  if (!motion_id_present || !page_index_present) return false;
  output = staged;
  return true;
}

inline bool append_controller_motion_trace_record(
  const ControllerMotionTraceRecord &record,
  char *output,
  size_t capacity,
  size_t &index
) {
  return append_json_text("{\"commanded_steps\":", output, capacity, index)
    && append_json_int32_array(record.commanded_steps, 6, output, capacity, index)
    && append_json_text(",\"controller_microseconds\":", output, capacity, index)
    && append_json_uint32(record.controller_microseconds, output, capacity, index)
    && append_json_text(",\"encoder_counts\":", output, capacity, index)
    && append_json_int32_array(record.encoder_counts, 6, output, capacity, index)
    && append_json_text(",\"flags\":", output, capacity, index)
    && append_json_uint32(record.flags, output, capacity, index)
    && append_json_text(",\"master_index\":", output, capacity, index)
    && append_json_uint32(record.master_index, output, capacity, index)
    && append_json_text(",\"phase\":", output, capacity, index)
    && append_json_uint32(record.phase, output, capacity, index)
    && append_json_text(",\"scheduled_delay_microseconds\":", output, capacity, index)
    && append_json_uint32(record.scheduled_delay_microseconds, output, capacity, index)
    && append_json_text("}", output, capacity, index);
}

inline bool build_main_json_motion_trace_response(
  uint32_t request_id,
  const JsonMainMotionTraceParameters &params,
  const ControllerMotionTraceCapture &capture,
  const char *current_session_id,
  const char *firmware_name,
  const char *firmware_version,
  const char *firmware_build,
  size_t maximum_payload_bytes,
  char *output,
  size_t output_capacity
) {
  if (
    request_id == 0
    || params.motion_request_id == 0
    || output == nullptr
    || output_capacity == 0
    || maximum_payload_bytes == 0
    || maximum_payload_bytes > kJsonProtocolMaximumPayloadBytes
    || !json_session_identifier_valid(current_session_id)
  ) return false;
  const size_t capacity = output_capacity < maximum_payload_bytes + 1
    ? output_capacity : maximum_payload_bytes + 1;
  size_t index = 0;
  if (
    !capture.available_for(params.motion_request_id)
    || strcmp(capture.source_session_id(), current_session_id) != 0
    || !capture.page_valid(params.page_index)
  ) {
    return append_json_text("{\"cmd\":\"get_motion_trace\",\"id\":", output, capacity, index)
      && append_json_uint32(request_id, output, capacity, index)
      && append_json_text(",\"result\":{\"capture_state\":\"no_capture\",\"source_motion_request_id\":", output, capacity, index)
      && append_json_uint32(params.motion_request_id, output, capacity, index)
      && append_json_text("},\"status\":\"completed\",\"type\":\"response\",\"v\":1}", output, capacity, index);
  }
  if (
    !json_protocol_text_valid(firmware_name, 31, false)
    || !json_protocol_text_valid(firmware_version, 31, false)
    || !json_protocol_text_valid(firmware_build, 31, false)
  ) return false;
  const uint32_t record_start =
    params.page_index * kControllerMotionTracePageRecords;
  const uint32_t remaining = capture.record_count() - record_start;
  const uint32_t page_records = remaining < kControllerMotionTracePageRecords
    ? remaining : kControllerMotionTracePageRecords;
  const char *outcome = controller_motion_trace_outcome_name(capture.outcome());
  bool built = outcome != nullptr
    && append_json_text("{\"cmd\":\"get_motion_trace\",\"id\":", output, capacity, index)
    && append_json_uint32(request_id, output, capacity, index)
    && append_json_text(",\"result\":{\"capture_generation\":", output, capacity, index)
    && append_json_uint32(capture.generation(), output, capacity, index)
    && append_json_text(",\"capture_state\":\"available\",\"configuration_fingerprint\":\"", output, capacity, index)
    && append_json_text(capture.configuration_fingerprint(), output, capacity, index)
    && append_json_text("\",\"disposition\":{\"capacity_limited\":", output, capacity, index)
    && append_json_text(capture.capacity_limited() ? "true" : "false", output, capacity, index)
    && append_json_text(",\"clock_wrapped\":", output, capacity, index)
    && append_json_text(capture.clock_wrapped() ? "true" : "false", output, capacity, index)
    && append_json_text(",\"complete\":", output, capacity, index)
    && append_json_text(capture.complete() ? "true" : "false", output, capacity, index)
    && append_json_text(",\"motion_outcome\":\"", output, capacity, index)
    && append_json_text(outcome, output, capacity, index)
    && append_json_text("\",\"timing_overrun\":", output, capacity, index)
    && append_json_text(capture.timing_overrun() ? "true" : "false", output, capacity, index)
    && append_json_text("},\"firmware\":{\"build\":\"", output, capacity, index)
    && append_json_escaped_text(firmware_build, 31, false, output, capacity, index)
    && append_json_text("\",\"name\":\"", output, capacity, index)
    && append_json_escaped_text(firmware_name, 31, false, output, capacity, index)
    && append_json_text("\",\"version\":\"", output, capacity, index)
    && append_json_escaped_text(firmware_version, 31, false, output, capacity, index)
    && append_json_text("\"},\"page_count\":", output, capacity, index)
    && append_json_uint32(capture.page_count(), output, capacity, index)
    && append_json_text(",\"page_index\":", output, capacity, index)
    && append_json_uint32(params.page_index, output, capacity, index)
    && append_json_text(",\"record_start\":", output, capacity, index)
    && append_json_uint32(record_start, output, capacity, index)
    && append_json_text(",\"records\":[", output, capacity, index);
  for (uint32_t offset = 0; built && offset < page_records; ++offset) {
    built = (offset == 0 || append_json_text(",", output, capacity, index))
      && append_controller_motion_trace_record(
        capture.records()[record_start + offset], output, capacity, index
      );
  }
  return built
    && append_json_text("],\"source_motion_request_id\":", output, capacity, index)
    && append_json_uint32(capture.source_motion_request_id(), output, capacity, index)
    && append_json_text(",\"source_session_id\":\"", output, capacity, index)
    && append_json_text(capture.source_session_id(), output, capacity, index)
    && append_json_text("\",\"total_records\":", output, capacity, index)
    && append_json_uint32(capture.record_count(), output, capacity, index)
    && append_json_text("},\"status\":\"completed\",\"type\":\"response\",\"v\":1}", output, capacity, index);
}

}  // namespace ar4_protocol

#endif
