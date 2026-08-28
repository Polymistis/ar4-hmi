#ifndef AR4_JSON_SERIAL_FRAME_CONTRACT_H
#define AR4_JSON_SERIAL_FRAME_CONTRACT_H

#include <stddef.h>

namespace ar4_protocol {

constexpr size_t kJsonProtocolMaximumFrameBytes = 4096;
constexpr size_t kJsonProtocolMaximumPayloadBytes =
  kJsonProtocolMaximumFrameBytes - 2;

enum class JsonSerialFrameStatus {
  kComplete,
  kInvalidArgument,
  kInvalidFrameLength,
  kInvalidDelimiter,
  kInvalidPayloadLength,
  kNonPrintablePayload,
};

struct JsonSerialFrameView {
  // Storage remains caller-owned and is not NUL-terminated; use payload_length.
  const char *payload;
  size_t payload_length;
};

inline JsonSerialFrameStatus validate_json_serial_frame(
  const char *frame,
  size_t frame_length,
  JsonSerialFrameView &view
) {
  view.payload = nullptr;
  view.payload_length = 0;
  if (frame == nullptr) {
    return JsonSerialFrameStatus::kInvalidArgument;
  }
  if (
    frame_length == 0
    || frame_length > kJsonProtocolMaximumFrameBytes
  ) {
    return JsonSerialFrameStatus::kInvalidFrameLength;
  }
  if (frame[frame_length - 1] != '\n') {
    return JsonSerialFrameStatus::kInvalidDelimiter;
  }

  size_t payload_length = frame_length - 1;
  if (payload_length > 0 && frame[payload_length - 1] == '\r') {
    --payload_length;
  }
  if (
    payload_length == 0
    || payload_length > kJsonProtocolMaximumPayloadBytes
  ) {
    return JsonSerialFrameStatus::kInvalidPayloadLength;
  }
  for (size_t index = 0; index < payload_length; ++index) {
    const unsigned char value = static_cast<unsigned char>(frame[index]);
    if (value < 0x20 || value > 0x7E) {
      return JsonSerialFrameStatus::kNonPrintablePayload;
    }
  }

  view.payload = frame;
  view.payload_length = payload_length;
  return JsonSerialFrameStatus::kComplete;
}

}  // namespace ar4_protocol

#endif
