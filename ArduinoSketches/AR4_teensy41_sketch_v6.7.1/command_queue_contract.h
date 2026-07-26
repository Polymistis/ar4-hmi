#ifndef AR4_COMMAND_QUEUE_CONTRACT_H
#define AR4_COMMAND_QUEUE_CONTRACT_H

namespace ar4_protocol {

enum class MotionCommandStatus {
    kCompleted,
    kRejected,
    kTerminalFaultReported,
};

inline bool should_emit_generic_motion_error(MotionCommandStatus status) {
    return status == MotionCommandStatus::kRejected;
}

inline bool should_continue_stored_playback(MotionCommandStatus status) {
    return status == MotionCommandStatus::kCompleted;
}

template <typename Text>
inline bool extract_serial_command_payload(
    const Text& frame,
    Text& payload
) {
    int end = static_cast<int>(frame.length());
    if (end == 0 || frame.charAt(end - 1) != '\n') return false;
    --end;
    if (end > 0 && frame.charAt(end - 1) == '\r') --end;
    if (end < 2) return false;
    for (int index = 0; index < end; ++index) {
        if (frame.charAt(index) == '\r' || frame.charAt(index) == '\n') {
            return false;
        }
    }
    payload = frame.substring(0, end);
    return true;
}

template <typename Text>
inline bool extract_stored_command_payload(
    const Text& row,
    Text& payload
) {
    int end = static_cast<int>(row.length());
    if (end > 0 && row.charAt(end - 1) == '\r') --end;
    if (end == 0) return false;
    for (int index = 0; index < end; ++index) {
        if (row.charAt(index) == '\r' || row.charAt(index) == '\n') {
            return false;
        }
    }
    payload = row.substring(0, end);
    return true;
}

template <typename Text>
inline void consume_command_queue(
    Text& current_input,
    Text& first,
    Text& second,
    Text& third
) {
    current_input = Text();
    first = second;
    second = third;
    third = Text();
    if (first.length() == 0) {
        first = second;
        second = Text();
    }
}

}

#endif
