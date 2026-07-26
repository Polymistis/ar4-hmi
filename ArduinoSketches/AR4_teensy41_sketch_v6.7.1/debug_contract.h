#ifndef AR4_DEBUG_CONTRACT_H
#define AR4_DEBUG_CONTRACT_H

#include <cstddef>
#include <cstring>

namespace ar4_protocol {

enum class DebugCommandStatus {
    kValid,
    kMissingDebugField,
    kInvalidDebugValue,
    kInvalidPersistenceValue,
    kInvalidFormat,
};

struct DebugCommand {
    bool debug_value;
    bool persistence_requested;
    bool persistence_value;
};

inline DebugCommandStatus parse_debug_command(
    const char* payload,
    DebugCommand& command
) {
    const std::size_t length = payload == nullptr ? 0 : std::strlen(payload);
    if (length < 3 || std::strncmp(payload, "[D]", 3) != 0) {
        return DebugCommandStatus::kMissingDebugField;
    }
    if (length < 4 || (payload[3] != '0' && payload[3] != '1')) {
        return DebugCommandStatus::kInvalidDebugValue;
    }

    DebugCommand parsed = {payload[3] == '1', false, false};
    if (length == 4) {
        command = parsed;
        return DebugCommandStatus::kValid;
    }
    if (length < 7 || std::strncmp(payload + 4, "[P]", 3) != 0) {
        return DebugCommandStatus::kInvalidFormat;
    }
    if (length != 8 || (payload[7] != '0' && payload[7] != '1')) {
        return DebugCommandStatus::kInvalidPersistenceValue;
    }

    parsed.persistence_requested = true;
    parsed.persistence_value = payload[7] == '1';
    command = parsed;
    return DebugCommandStatus::kValid;
}

template <typename PersistenceWriter>
bool apply_debug_command(
    const DebugCommand& command,
    bool& live_debug,
    PersistenceWriter persistence_writer
) {
    if (
        command.persistence_requested
        && !persistence_writer(command.persistence_value)
    ) {
        return false;
    }
    live_debug = command.debug_value;
    return true;
}

}  // namespace ar4_protocol

#endif
