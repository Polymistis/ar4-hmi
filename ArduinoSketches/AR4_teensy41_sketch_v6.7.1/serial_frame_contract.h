#ifndef AR4_SERIAL_FRAME_CONTRACT_H
#define AR4_SERIAL_FRAME_CONTRACT_H

#include <stddef.h>
#include <stdint.h>

namespace ar4_protocol {

// Match the host transmission boundary so every accepted host command can be
// accumulated while malformed unterminated input remains memory-bounded.
constexpr size_t kSerialCommandFrameMaximumLength = 4096;
constexpr size_t kStoredCommandRowMaximumLength =
  kSerialCommandFrameMaximumLength - 1;
constexpr uint32_t kSerialFrameReceiveTimeoutMilliseconds = 5000;

enum class SerialFrameReadStatus {
  kPending,
  kComplete,
  kOverflow,
};

enum class StoredRowReadStatus {
  kPending,
  kComplete,
  kOverflow,
  kRejected,
};

struct SerialFrameReceiveDeadline {
  bool active;
  bool discarding_timed_out_frame;
  uint32_t started_at_milliseconds;
};

template <typename Text>
inline SerialFrameReadStatus append_serial_frame_byte(
  Text &frame,
  bool &discarding,
  int received
);

inline void abandon_serial_frame_receive_deadline(
  SerialFrameReceiveDeadline &deadline
) {
  deadline = {};
}

inline void update_serial_frame_receive_deadline(
  uint32_t now_milliseconds,
  SerialFrameReadStatus status,
  size_t partial_frame_length,
  bool discarding,
  SerialFrameReceiveDeadline &deadline
) {
  if (deadline.discarding_timed_out_frame) {
    if (!discarding) abandon_serial_frame_receive_deadline(deadline);
    return;
  }
  const bool pending = partial_frame_length != 0 || discarding;
  if (status == SerialFrameReadStatus::kComplete || !pending) {
    abandon_serial_frame_receive_deadline(deadline);
    return;
  }
  if (!deadline.active) {
    deadline.active = true;
    deadline.started_at_milliseconds = now_milliseconds;
  }
}

template <typename Text>
inline void expire_serial_frame_receive(
  Text &frame,
  bool &discarding,
  SerialFrameReceiveDeadline &deadline
) {
  frame = "";
  discarding = true;
  deadline.active = false;
  deadline.discarding_timed_out_frame = true;
  deadline.started_at_milliseconds = 0;
}

inline bool serial_frame_receive_timed_out(
  uint32_t now_milliseconds,
  const SerialFrameReceiveDeadline &deadline
) {
  return deadline.active
    && static_cast<uint32_t>(
      now_milliseconds - deadline.started_at_milliseconds
    ) >= kSerialFrameReceiveTimeoutMilliseconds;
}

template <typename ReadByte, typename Text>
inline SerialFrameReadStatus service_serial_frame_input(
  uint32_t now_milliseconds,
  ReadByte &read_byte,
  Text &frame,
  bool &discarding,
  SerialFrameReceiveDeadline &deadline,
  size_t maximum_bytes = kSerialCommandFrameMaximumLength
) {
  if (maximum_bytes == 0 || maximum_bytes > kSerialCommandFrameMaximumLength) {
    return SerialFrameReadStatus::kOverflow;
  }
  SerialFrameReadStatus status = SerialFrameReadStatus::kPending;
  size_t consumed = 0;
  int received = -1;
  while (consumed < maximum_bytes && (received = read_byte()) >= 0) {
    status = append_serial_frame_byte(frame, discarding, received);
    update_serial_frame_receive_deadline(
      now_milliseconds,
      status,
      frame.length(),
      discarding,
      deadline
    );
    ++consumed;
    if (status != SerialFrameReadStatus::kPending) return status;
  }
  return status;
}

inline bool retain_serial_frame_discarding(
  bool discarding,
  size_t partial_frame_length
) {
  return discarding || partial_frame_length != 0;
}

template <typename Text>
inline SerialFrameReadStatus append_serial_frame_byte(
  Text &frame,
  bool &discarding,
  int received
) {
  if (received < 0) return SerialFrameReadStatus::kPending;

  if (discarding) {
    if (received == '\n') discarding = false;
    return SerialFrameReadStatus::kPending;
  }

  if (
    received > 255
    || frame.length() >= kSerialCommandFrameMaximumLength
  ) {
    frame = "";
    discarding = received != '\n';
    return SerialFrameReadStatus::kOverflow;
  }

  frame += static_cast<char>(received);
  return received == '\n'
    ? SerialFrameReadStatus::kComplete
    : SerialFrameReadStatus::kPending;
}

template <typename Text>
inline StoredRowReadStatus append_stored_row_byte(
  Text &row,
  int received
) {
  if (received < 0 || received > 255) {
    return StoredRowReadStatus::kRejected;
  }
  if (received == '\n') return StoredRowReadStatus::kComplete;
  if (row.length() >= kStoredCommandRowMaximumLength) {
    row = "";
    return StoredRowReadStatus::kOverflow;
  }
  row += static_cast<char>(received);
  return StoredRowReadStatus::kPending;
}

template <typename Text>
inline StoredRowReadStatus finish_stored_row(const Text &row) {
  return row.length() == 0
    ? StoredRowReadStatus::kRejected
    : StoredRowReadStatus::kComplete;
}

}  // namespace ar4_protocol

#endif
