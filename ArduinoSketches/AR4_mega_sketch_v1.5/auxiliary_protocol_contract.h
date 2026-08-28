#ifndef AR4_AUXILIARY_PROTOCOL_CONTRACT_H
#define AR4_AUXILIARY_PROTOCOL_CONTRACT_H

#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace ar4_auxiliary {

static const size_t kMaximumPayloadLength = 384;
static const uint32_t kMaximumWaitSeconds = 32767UL;

enum BoardProfile : uint8_t {
  kNanoBoard,
  kMegaBoard,
};

enum CommandKind : uint8_t {
  kHelloCommand,
  kServoCommand,
  kInputReadCommand,
  kSetOutputCommand,
  kWaitInputCommand,
  kGripperCurrentCommand,
  kStopCommand,
  kGripperDetachCommand,
  kUnknownCommand,
};

struct ParsedCommand {
  CommandKind kind;
  uint32_t requestId;
  uint8_t channel;
  uint8_t pin;
  uint8_t state;
  uint16_t position;
  uint16_t timeoutSeconds;
};

enum RequestParseStatus : uint8_t {
  kRequestReady,
  kRequestInvalidEnvelope,
  kRequestInvalidParameters,
};

enum WaitResult : uint8_t {
  kWaitInactive,
  kWaitPending,
  kWaitMatched,
  kWaitTimedOut,
};

struct WaitState {
  bool active;
  uint8_t pin;
  uint8_t expectedState;
  uint32_t requestId;
  uint32_t startedAtMilliseconds;
  uint32_t timeoutMilliseconds;
};

enum FrameStatus : uint8_t {
  kFramePending,
  kFrameReady,
  kFrameRejected,
};

struct Frame {
  const char* data;
  size_t length;
};

class FrameBuffer {
 public:
  FrameBuffer() : length_(0), discarding_(false) {
    data_[0] = '\0';
  }

  FrameStatus push(char byte, Frame* frame) {
    const uint8_t value = static_cast<uint8_t>(byte);
    if (value == static_cast<uint8_t>('\n')) {
      size_t payloadLength = length_;
      if (
        payloadLength > 0
        && data_[payloadLength - 1] == '\r'
      ) {
        --payloadLength;
      }
      const bool rejected = (
        discarding_
        || payloadLength == 0
        || frame == NULL
      );
      if (!rejected) {
        data_[payloadLength] = '\0';
        frame->data = data_;
        frame->length = payloadLength;
      }
      length_ = 0;
      discarding_ = false;
      return rejected ? kFrameRejected : kFrameReady;
    }

    if (discarding_) {
      return kFramePending;
    }
    if (
      (value != static_cast<uint8_t>('\r')
        && (value < static_cast<uint8_t>(' ')
          || value > static_cast<uint8_t>('~')))
      || length_ >= kMaximumPayloadLength
    ) {
      // Retain framing ownership through LF after one malformed payload.
      discarding_ = true;
      return kFramePending;
    }

    data_[length_++] = byte;
    return kFramePending;
  }

 private:
  char data_[kMaximumPayloadLength + 1];
  size_t length_;
  bool discarding_;
};

inline bool inputPinValid(BoardProfile board, uint32_t pin) {
  if (board == kNanoBoard) {
    return pin >= 2UL && pin <= 7UL;
  }
  return board == kMegaBoard && pin >= 2UL && pin <= 27UL;
}

inline bool outputPinValid(BoardProfile board, uint32_t pin) {
  if (board == kNanoBoard) {
    return pin >= 8UL && pin <= 13UL;
  }
  return board == kMegaBoard && pin >= 28UL && pin <= 53UL;
}

inline bool servoChannelValid(BoardProfile board, uint32_t channel) {
  if (board == kNanoBoard) {
    return channel <= 5UL;
  }
  return board == kMegaBoard && channel <= 6UL;
}

// Canonical host key order permits fixed-memory parsing on the 2 KiB Nano.
class RequestCursor {
 public:
  RequestCursor(const char* text, size_t length)
    : text_(text), length_(length), index_(0) {}

  bool take(const char* literal) {
    if (literal == NULL) return false;
    const size_t literalLength = strlen(literal);
    if (
      text_ == NULL
      || literalLength > length_ - index_
      || memcmp(text_ + index_, literal, literalLength) != 0
    ) {
      return false;
    }
    index_ += literalLength;
    return true;
  }

  bool takeUnsigned(uint32_t maximum, uint32_t* result) {
    if (text_ == NULL || result == NULL || index_ >= length_) return false;
    if (text_[index_] < '0' || text_[index_] > '9') return false;

    uint32_t value = 0;
    if (text_[index_] == '0') {
      ++index_;
      if (
        index_ < length_
        && text_[index_] >= '0'
        && text_[index_] <= '9'
      ) {
        return false;
      }
    } else {
      while (
        index_ < length_
        && text_[index_] >= '0'
        && text_[index_] <= '9'
      ) {
        const uint32_t digit = static_cast<uint32_t>(text_[index_] - '0');
        if (value > (maximum - digit) / 10UL) return false;
        value = value * 10UL + digit;
        ++index_;
      }
    }
    if (value > maximum) return false;
    *result = value;
    return true;
  }

  bool takeBoolean(uint8_t* result) {
    if (result == NULL) return false;
    if (take("true")) {
      *result = 1;
      return true;
    }
    if (take("false")) {
      *result = 0;
      return true;
    }
    return false;
  }

  bool complete() const {
    return index_ == length_;
  }

 private:
  const char* text_;
  size_t length_;
  size_t index_;
};

inline CommandKind takeCommandName(RequestCursor* cursor) {
  if (cursor == NULL) return kUnknownCommand;
  if (cursor->take("hello\"")) return kHelloCommand;
  if (cursor->take("servo\"")) return kServoCommand;
  if (cursor->take("input_read\"")) return kInputReadCommand;
  if (cursor->take("set_output\"")) return kSetOutputCommand;
  if (cursor->take("wait_input\"")) return kWaitInputCommand;
  if (cursor->take("test_gripper_amps\"")) {
    return kGripperCurrentCommand;
  }
  if (cursor->take("stop\"")) return kStopCommand;
  if (cursor->take("gripper_detach\"")) return kGripperDetachCommand;
  return kUnknownCommand;
}

inline bool takeParameters(
  RequestCursor* cursor,
  BoardProfile board,
  ParsedCommand* command
) {
  if (cursor == NULL || command == NULL) return false;
  uint32_t first = 0;
  uint32_t second = 0;
  switch (command->kind) {
    case kHelloCommand:
    case kGripperCurrentCommand:
    case kStopCommand:
    case kGripperDetachCommand:
      return cursor->take("{}");
    case kServoCommand:
      if (
        !cursor->take("{\"channel\":")
        || !cursor->takeUnsigned(6UL, &first)
        || !cursor->take(",\"position\":")
        || !cursor->takeUnsigned(180UL, &second)
        || !cursor->take("}")
        || !servoChannelValid(board, first)
      ) {
        return false;
      }
      command->channel = static_cast<uint8_t>(first);
      command->position = static_cast<uint16_t>(second);
      return true;
    case kInputReadCommand:
      if (
        !cursor->take("{\"pin\":")
        || !cursor->takeUnsigned(53UL, &first)
        || !cursor->take("}")
        || !inputPinValid(board, first)
      ) {
        return false;
      }
      command->pin = static_cast<uint8_t>(first);
      return true;
    case kSetOutputCommand:
      if (
        !cursor->take("{\"pin\":")
        || !cursor->takeUnsigned(53UL, &first)
        || !cursor->take(",\"state\":")
        || !cursor->takeBoolean(&command->state)
        || !cursor->take("}")
        || !outputPinValid(board, first)
      ) {
        return false;
      }
      command->pin = static_cast<uint8_t>(first);
      return true;
    case kWaitInputCommand:
      if (
        !cursor->take("{\"pin\":")
        || !cursor->takeUnsigned(53UL, &first)
        || !cursor->take(",\"state\":")
        || !cursor->takeBoolean(&command->state)
        || !cursor->take(",\"timeout_seconds\":")
        || !cursor->takeUnsigned(kMaximumWaitSeconds, &second)
        || second == 0UL
        || !cursor->take("}")
        || !inputPinValid(board, first)
      ) {
        return false;
      }
      command->pin = static_cast<uint8_t>(first);
      command->timeoutSeconds = static_cast<uint16_t>(second);
      return true;
    case kUnknownCommand:
      return false;
  }
  return false;
}

inline RequestParseStatus parseRequest(
  const char* text,
  size_t length,
  BoardProfile board,
  ParsedCommand* result
) {
  if (
    text == NULL
    || result == NULL
    || length == 0
    || length > kMaximumPayloadLength
    || (board != kNanoBoard && board != kMegaBoard)
  ) {
    return kRequestInvalidEnvelope;
  }

  ParsedCommand parsed = {
    kUnknownCommand,
    0,
    0,
    0,
    0,
    0,
    0,
  };
  RequestCursor cursor(text, length);
  if (!cursor.take("{\"cmd\":\"")) return kRequestInvalidEnvelope;
  parsed.kind = takeCommandName(&cursor);
  if (
    parsed.kind == kUnknownCommand
    || !cursor.take(",\"id\":")
    || !cursor.takeUnsigned(UINT32_MAX, &parsed.requestId)
    || parsed.requestId == 0
    || !cursor.take(",\"params\":")
  ) {
    return kRequestInvalidEnvelope;
  }

  *result = parsed;
  if (!takeParameters(&cursor, board, &parsed)) {
    return kRequestInvalidParameters;
  }
  if (
    !cursor.take(",\"type\":\"request\",\"v\":1}")
    || !cursor.complete()
  ) {
    return kRequestInvalidEnvelope;
  }
  *result = parsed;
  return kRequestReady;
}

inline bool waitExpired(
  uint32_t startedAtMilliseconds,
  uint32_t timeoutMilliseconds,
  uint32_t nowMilliseconds
) {
  return static_cast<uint32_t>(
    nowMilliseconds - startedAtMilliseconds
  ) >= timeoutMilliseconds;
}

inline bool startWait(
  WaitState* state,
  const ParsedCommand& command,
  uint32_t nowMilliseconds
) {
  if (
    state == NULL
    || state->active
    || command.kind != kWaitInputCommand
    || command.requestId == 0
    || command.timeoutSeconds == 0
    || command.timeoutSeconds > kMaximumWaitSeconds
    || command.state > 1
  ) {
    return false;
  }
  WaitState started = {
    true,
    command.pin,
    command.state,
    command.requestId,
    nowMilliseconds,
    static_cast<uint32_t>(command.timeoutSeconds) * 1000UL,
  };
  *state = started;
  return true;
}

inline WaitResult updateWait(
  WaitState* state,
  uint8_t observedState,
  uint32_t nowMilliseconds
) {
  if (state == NULL || !state->active || observedState > 1) {
    return kWaitInactive;
  }
  if (observedState == state->expectedState) {
    state->active = false;
    return kWaitMatched;
  }
  if (waitExpired(
      state->startedAtMilliseconds,
      state->timeoutMilliseconds,
      nowMilliseconds
  )) {
    state->active = false;
    return kWaitTimedOut;
  }
  return kWaitPending;
}

inline bool cancelWait(WaitState* state) {
  if (state == NULL || !state->active) return false;
  state->active = false;
  return true;
}

}  // namespace ar4_auxiliary

#endif  // AR4_AUXILIARY_PROTOCOL_CONTRACT_H
