#ifndef AR4_SERIAL_FRAME_CONTRACT_H
#define AR4_SERIAL_FRAME_CONTRACT_H

#include <stddef.h>

namespace ar4_protocol {

// Match the host transmission boundary so every accepted host command can be
// accumulated while malformed unterminated input remains memory-bounded.
constexpr size_t kSerialCommandFrameMaximumLength = 4096;
constexpr size_t kStoredCommandRowMaximumLength =
  kSerialCommandFrameMaximumLength - 1;

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

enum class LiveControlFrameStatus {
  kPending,
  kStop,
  kRejected,
};

enum class LiveTerminalResponseKind {
  kPosition,
  kError,
  kAxisLimit,
};

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

inline LiveControlFrameStatus classify_live_control_frame(
  SerialFrameReadStatus status,
  const char *frame,
  size_t frame_length
) {
  if (status == SerialFrameReadStatus::kPending) {
    return LiveControlFrameStatus::kPending;
  }
  if (status == SerialFrameReadStatus::kOverflow || frame == nullptr) {
    return LiveControlFrameStatus::kRejected;
  }
  const bool line_feed_stop =
    frame_length == 2 && frame[0] == 'S' && frame[1] == '\n';
  const bool carriage_return_stop =
    frame_length == 3
    && frame[0] == 'S'
    && frame[1] == '\r'
    && frame[2] == '\n';
  return line_feed_stop || carriage_return_stop
    ? LiveControlFrameStatus::kStop
    : LiveControlFrameStatus::kRejected;
}

inline LiveTerminalResponseKind select_live_terminal_response(
  LiveControlFrameStatus control_status,
  int kinematic_error,
  int axis_fault
) {
  if (
    control_status == LiveControlFrameStatus::kRejected
    || kinematic_error != 0
  ) {
    return LiveTerminalResponseKind::kError;
  }
  return axis_fault == 0
    ? LiveTerminalResponseKind::kPosition
    : LiveTerminalResponseKind::kAxisLimit;
}

}  // namespace ar4_protocol

#endif
