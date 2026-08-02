#ifndef AR4_AUXILIARY_PROTOCOL_CONTRACT_H
#define AR4_AUXILIARY_PROTOCOL_CONTRACT_H

#include <stddef.h>
#include <stdint.h>

namespace ar4_auxiliary {

static const size_t kMaximumCommandLength = 96;
static const uint32_t kMaximumWaitSeconds = 32767UL;

enum BoardProfile : uint8_t {
  kNanoBoard,
  kMegaBoard,
};

enum CommandKind : uint8_t {
  kServoCommand,
  kInputReadCommand,
  kOutputOnCommand,
  kOutputOffCommand,
  kWaitInputCommand,
  kGripperCurrentCommand,
  kStopCommand,
  kEchoCommand,
};

struct ParsedCommand {
  CommandKind kind;
  uint8_t channel;
  uint8_t pin;
  uint8_t state;
  uint16_t position;
  uint16_t timeoutSeconds;
  const char* payload;
  size_t payloadLength;
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
  uint32_t startedAtMilliseconds;
  uint32_t timeoutMilliseconds;
};

enum CommandDisposition : uint8_t {
  kExecuteCommand,
  kRejectDuringWait,
  kStopActiveWait,
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
      const bool rejected = discarding_ || length_ == 0 || frame == NULL;
      if (!rejected) {
        data_[length_] = '\0';
        frame->data = data_;
        frame->length = length_;
      }
      length_ = 0;
      discarding_ = false;
      return rejected ? kFrameRejected : kFrameReady;
    }

    if (discarding_) {
      return kFramePending;
    }
    if (
      value < static_cast<uint8_t>(' ')
      || value > static_cast<uint8_t>('~')
      || length_ >= kMaximumCommandLength
    ) {
      // Discard through LF so one malformed frame produces one response.
      discarding_ = true;
      return kFramePending;
    }

    data_[length_] = byte;
    ++length_;
    return kFramePending;
  }

  size_t length() const {
    return length_;
  }

  bool discarding() const {
    return discarding_;
  }

 private:
  char data_[kMaximumCommandLength + 1];
  size_t length_;
  bool discarding_;
};

inline bool boardProfileValid(BoardProfile board) {
  return board == kNanoBoard || board == kMegaBoard;
}

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

inline bool literalMatches(
  const char* text,
  size_t length,
  const char* literal,
  size_t literalLength
) {
  if (text == NULL || literal == NULL || length != literalLength) {
    return false;
  }
  for (size_t index = 0; index < length; ++index) {
    if (text[index] != literal[index]) {
      return false;
    }
  }
  return true;
}

inline bool prefixMatches(
  const char* text,
  size_t length,
  const char* prefix,
  size_t prefixLength
) {
  if (
    text == NULL
    || prefix == NULL
    || length < prefixLength
  ) {
    return false;
  }
  for (size_t index = 0; index < prefixLength; ++index) {
    if (text[index] != prefix[index]) {
      return false;
    }
  }
  return true;
}

inline bool parseUnsigned(
  const char* text,
  size_t begin,
  size_t end,
  uint32_t maximum,
  uint32_t* result
) {
  if (
    text == NULL
    || result == NULL
    || begin >= end
  ) {
    return false;
  }
  uint32_t value = 0;
  for (size_t index = begin; index < end; ++index) {
    const char digitCharacter = text[index];
    if (digitCharacter < '0' || digitCharacter > '9') {
      return false;
    }
    const uint32_t digit = static_cast<uint32_t>(
      digitCharacter - '0'
    );
    if (
      digit > maximum
      || value > (maximum - digit) / 10UL
    ) {
      return false;
    }
    value = value * 10UL + digit;
  }
  *result = value;
  return true;
}

inline size_t findMarker(
  const char* text,
  size_t begin,
  size_t length,
  char marker
) {
  if (text == NULL || begin >= length) {
    return length;
  }
  for (size_t index = begin; index < length; ++index) {
    if (text[index] == marker) {
      return index;
    }
  }
  return length;
}

inline bool parseCommand(
  const char* text,
  size_t length,
  BoardProfile board,
  ParsedCommand* result
) {
  if (
    text == NULL
    || result == NULL
    || length == 0
    || length > kMaximumCommandLength
    || !boardProfileValid(board)
  ) {
    return false;
  }

  ParsedCommand parsed;
  parsed.channel = 0;
  parsed.pin = 0;
  parsed.state = 0;
  parsed.position = 0;
  parsed.timeoutSeconds = 0;
  parsed.payload = NULL;
  parsed.payloadLength = 0;

  if (literalMatches(text, length, "TG", 2)) {
    parsed.kind = kGripperCurrentCommand;
  } else if (
    literalMatches(text, length, "STOP", 4)
    || literalMatches(text, length, "STOPWI", 6)
  ) {
    parsed.kind = kStopCommand;
  } else if (prefixMatches(text, length, "TM", 2)) {
    parsed.kind = kEchoCommand;
    parsed.payload = text + 2;
    parsed.payloadLength = length - 2;
  } else if (prefixMatches(text, length, "SV", 2)) {
    const size_t positionMarker = findMarker(text, 2, length, 'P');
    uint32_t channel = 0;
    uint32_t position = 0;
    if (
      positionMarker == length
      || !parseUnsigned(text, 2, positionMarker, 6UL, &channel)
      || !parseUnsigned(
        text,
        positionMarker + 1,
        length,
        180UL,
        &position
      )
      || !servoChannelValid(board, channel)
    ) {
      return false;
    }
    parsed.kind = kServoCommand;
    parsed.channel = static_cast<uint8_t>(channel);
    parsed.position = static_cast<uint16_t>(position);
  } else if (prefixMatches(text, length, "JFX", 3)) {
    uint32_t pin = 0;
    if (
      !parseUnsigned(text, 3, length, 53UL, &pin)
      || !inputPinValid(board, pin)
    ) {
      return false;
    }
    parsed.kind = kInputReadCommand;
    parsed.pin = static_cast<uint8_t>(pin);
  } else if (
    prefixMatches(text, length, "ONX", 3)
    || prefixMatches(text, length, "OFX", 3)
  ) {
    uint32_t pin = 0;
    if (
      !parseUnsigned(text, 3, length, 53UL, &pin)
      || !outputPinValid(board, pin)
    ) {
      return false;
    }
    parsed.kind = text[1] == 'N' ? kOutputOnCommand : kOutputOffCommand;
    parsed.pin = static_cast<uint8_t>(pin);
  } else if (prefixMatches(text, length, "WIA", 3)) {
    const size_t stateMarker = findMarker(text, 3, length, 'B');
    const size_t timeoutMarker = (
      stateMarker == length
      ? length
      : findMarker(text, stateMarker + 1, length, 'C')
    );
    uint32_t pin = 0;
    uint32_t state = 0;
    uint32_t timeout = 0;
    if (
      stateMarker == length
      || timeoutMarker == length
      || !parseUnsigned(text, 3, stateMarker, 53UL, &pin)
      || !parseUnsigned(
        text,
        stateMarker + 1,
        timeoutMarker,
        1UL,
        &state
      )
      || !parseUnsigned(
        text,
        timeoutMarker + 1,
        length,
        kMaximumWaitSeconds,
        &timeout
      )
      || timeout == 0UL
      || !inputPinValid(board, pin)
    ) {
      return false;
    }
    parsed.kind = kWaitInputCommand;
    parsed.pin = static_cast<uint8_t>(pin);
    parsed.state = static_cast<uint8_t>(state);
    parsed.timeoutSeconds = static_cast<uint16_t>(timeout);
  } else {
    return false;
  }

  *result = parsed;
  return true;
}

inline bool waitExpired(
  uint32_t startedAtMilliseconds,
  uint32_t timeoutMilliseconds,
  uint32_t nowMilliseconds
) {
  // Unsigned subtraction preserves elapsed time across millis() rollover.
  return (
    static_cast<uint32_t>(
      nowMilliseconds - startedAtMilliseconds
    )
    >= timeoutMilliseconds
  );
}

inline bool startWait(
  WaitState* state,
  uint8_t pin,
  uint8_t expectedState,
  uint16_t timeoutSeconds,
  uint32_t nowMilliseconds
) {
  if (
    state == NULL
    || state->active
    || expectedState > 1
    || timeoutSeconds == 0
    || timeoutSeconds > kMaximumWaitSeconds
  ) {
    return false;
  }
  WaitState started = {
    true,
    pin,
    expectedState,
    nowMilliseconds,
    static_cast<uint32_t>(
      static_cast<uint32_t>(timeoutSeconds) * 1000UL
    ),
  };
  *state = started;
  return true;
}

inline WaitResult updateWait(
  WaitState* state,
  uint8_t observedState,
  uint32_t nowMilliseconds
) {
  if (
    state == NULL
    || !state->active
    || observedState > 1
  ) {
    return kWaitInactive;
  }
  if (observedState == state->expectedState) {
    state->active = false;
    return kWaitMatched;
  }
  if (
    waitExpired(
      state->startedAtMilliseconds,
      state->timeoutMilliseconds,
      nowMilliseconds
    )
  ) {
    state->active = false;
    return kWaitTimedOut;
  }
  return kWaitPending;
}

inline bool cancelWait(WaitState* state) {
  if (state == NULL || !state->active) {
    return false;
  }
  state->active = false;
  return true;
}

inline CommandDisposition commandDisposition(
  bool waitActive,
  CommandKind kind
) {
  if (!waitActive) {
    return kExecuteCommand;
  }
  if (kind == kStopCommand) {
    return kStopActiveWait;
  }
  return kRejectDuringWait;
}

}  // namespace ar4_auxiliary

#endif  // AR4_AUXILIARY_PROTOCOL_CONTRACT_H
