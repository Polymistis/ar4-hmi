#ifndef AR4_IDENTITY_CONTRACT_H
#define AR4_IDENTITY_CONTRACT_H

#include <stddef.h>
#include <stdint.h>

namespace ar4_protocol {

constexpr size_t kIdentityFieldMaximumLength = 31;
constexpr size_t kIdentityFieldCapacity = kIdentityFieldMaximumLength + 1;
constexpr size_t kIdentitySetMarkerLength = 3;
constexpr size_t kIdentitySetMarkerCount = 5;
constexpr size_t kIdentitySetCommandMaximumLength =
    kIdentitySetMarkerCount
    * (kIdentitySetMarkerLength + kIdentityFieldMaximumLength);
constexpr size_t kEscapedIdentityCapacity =
    2 * kIdentityFieldMaximumLength + 1;
constexpr size_t kProtocolCapabilityMaximumLength = 31;
constexpr size_t kProtocolCapabilityMaximumCount = 8;
constexpr size_t kIdentityJsonCapacity = 1024;
constexpr size_t kControllerHardwareIdLength = 6;
constexpr size_t kControllerHardwareIdCapacity =
    kControllerHardwareIdLength + 1;

struct IdentitySetCommandFields {
    char robot_model[kIdentityFieldCapacity];
    char robot_version[kIdentityFieldCapacity];
    char driver_board[kIdentityFieldCapacity];
    char serial_number[kIdentityFieldCapacity];
    char asset_tag[kIdentityFieldCapacity];
};

inline bool identity_field_valid(const char* value) {
    if (value == nullptr) return false;

    size_t length = 0;
    while (value[length] != '\0') {
        if (length >= kIdentityFieldMaximumLength) return false;
        const unsigned char character =
            static_cast<unsigned char>(value[length]);
        if (character < 32 || character > 126) return false;
        ++length;
    }
    return length > 0;
}

inline bool uppercase_hex_character(char value) {
    return (value >= '0' && value <= '9')
        || (value >= 'A' && value <= 'F');
}

inline bool controller_hardware_id_valid(const char* value) {
    if (value == nullptr) return false;
    for (size_t index = 0; index < kControllerHardwareIdLength; ++index) {
        if (!uppercase_hex_character(value[index])) return false;
    }
    return value[kControllerHardwareIdLength] == '\0';
}

inline bool format_controller_hardware_id(
    uint32_t value,
    char* output,
    size_t output_capacity
) {
    if (
        output == nullptr
        || output_capacity < kControllerHardwareIdCapacity
        || value > 0xFFFFFFu
    ) {
        return false;
    }
    static const char kHexDigits[] = "0123456789ABCDEF";
    for (size_t index = 0; index < kControllerHardwareIdLength; ++index) {
        const size_t shift =
            (kControllerHardwareIdLength - index - 1) * 4;
        output[index] = kHexDigits[(value >> shift) & 0x0Fu];
    }
    output[kControllerHardwareIdLength] = '\0';
    return true;
}

inline bool protocol_capability_valid(const char* value) {
    if (value == nullptr || value[0] < 'A' || value[0] > 'Z') {
        return false;
    }
    size_t length = 1;
    while (value[length] != '\0') {
        if (length >= kProtocolCapabilityMaximumLength) return false;
        const char character = value[length];
        if (
            !(
                (character >= 'A' && character <= 'Z')
                || (character >= '0' && character <= '9')
                || character == '_'
            )
        ) {
            return false;
        }
        ++length;
    }
    return true;
}

inline bool protocol_capability_equal(
    const char* left,
    const char* right
) {
    if (left == nullptr || right == nullptr) return false;
    size_t index = 0;
    while (left[index] != '\0' && right[index] != '\0') {
        if (left[index] != right[index]) return false;
        ++index;
    }
    return left[index] == right[index];
}

inline bool protocol_capabilities_valid(
    const char* const* capabilities,
    size_t capability_count
) {
    if (
        capabilities == nullptr
        || capability_count == 0
        || capability_count > kProtocolCapabilityMaximumCount
    ) {
        return false;
    }
    for (size_t index = 0; index < capability_count; ++index) {
        if (!protocol_capability_valid(capabilities[index])) return false;
        for (size_t prior = 0; prior < index; ++prior) {
            if (
                protocol_capability_equal(
                    capabilities[index],
                    capabilities[prior]
                )
            ) {
                return false;
            }
        }
    }
    return true;
}

inline size_t identity_set_command_length(const char* command) {
    if (command == nullptr) return kIdentitySetCommandMaximumLength + 1;
    size_t length = 0;
    while (command[length] != '\0') {
        if (length >= kIdentitySetCommandMaximumLength) {
            return kIdentitySetCommandMaximumLength + 1;
        }
        ++length;
    }
    return length;
}

inline bool identity_marker_at(
    const char* command,
    size_t command_length,
    size_t position,
    const char* marker
) {
    return command != nullptr
        && marker != nullptr
        && position + kIdentitySetMarkerLength <= command_length
        && command[position] == marker[0]
        && command[position + 1] == marker[1]
        && command[position + 2] == marker[2]
        && marker[kIdentitySetMarkerLength] == '\0';
}

inline int find_identity_marker(
    const char* command,
    size_t command_length,
    const char* marker
) {
    for (
        size_t position = 0;
        position + kIdentitySetMarkerLength <= command_length;
        ++position
    ) {
        if (identity_marker_at(command, command_length, position, marker)) {
            return static_cast<int>(position);
        }
    }
    return -1;
}

inline size_t count_identity_marker(
    const char* command,
    size_t command_length,
    const char* marker
) {
    size_t count = 0;
    for (
        size_t position = 0;
        position + kIdentitySetMarkerLength <= command_length;
        ++position
    ) {
        if (identity_marker_at(command, command_length, position, marker)) {
            ++count;
        }
    }
    return count;
}

inline bool copy_identity_command_field(
    const char* command,
    size_t begin,
    size_t end,
    char* output
) {
    if (
        command == nullptr
        || output == nullptr
        || end <= begin
        || end - begin > kIdentityFieldMaximumLength
    ) {
        return false;
    }
    const size_t length = end - begin;
    for (size_t index = 0; index < length; ++index) {
        output[index] = command[begin + index];
    }
    output[length] = '\0';
    return identity_field_valid(output);
}

inline bool parse_identity_set_command(
    const char* command,
    IdentitySetCommandFields& output
) {
    const size_t command_length = identity_set_command_length(command);
    if (command_length > kIdentitySetCommandMaximumLength) return false;

    const char* markers[kIdentitySetMarkerCount] = {
        "[M]",
        "[V]",
        "[B]",
        "[S]",
        "[A]",
    };
    int positions[kIdentitySetMarkerCount] = {};
    for (size_t index = 0; index < kIdentitySetMarkerCount; ++index) {
        positions[index] = find_identity_marker(
            command,
            command_length,
            markers[index]
        );
        if (
            positions[index] < 0
            || count_identity_marker(command, command_length, markers[index])
                != 1
        ) {
            return false;
        }
    }
    if (positions[0] != 0) return false;
    for (size_t index = 1; index < kIdentitySetMarkerCount; ++index) {
        if (
            positions[index]
            < positions[index - 1]
                + static_cast<int>(kIdentitySetMarkerLength)
        ) {
            return false;
        }
    }

    const size_t begins[kIdentitySetMarkerCount] = {
        static_cast<size_t>(positions[0]) + kIdentitySetMarkerLength,
        static_cast<size_t>(positions[1]) + kIdentitySetMarkerLength,
        static_cast<size_t>(positions[2]) + kIdentitySetMarkerLength,
        static_cast<size_t>(positions[3]) + kIdentitySetMarkerLength,
        static_cast<size_t>(positions[4]) + kIdentitySetMarkerLength,
    };
    const size_t ends[kIdentitySetMarkerCount] = {
        static_cast<size_t>(positions[1]),
        static_cast<size_t>(positions[2]),
        static_cast<size_t>(positions[3]),
        static_cast<size_t>(positions[4]),
        command_length,
    };
    IdentitySetCommandFields staged = {};
    char* fields[kIdentitySetMarkerCount] = {
        staged.robot_model,
        staged.robot_version,
        staged.driver_board,
        staged.serial_number,
        staged.asset_tag,
    };
    for (size_t index = 0; index < kIdentitySetMarkerCount; ++index) {
        if (!copy_identity_command_field(
            command,
            begins[index],
            ends[index],
            fields[index]
        )) {
            return false;
        }
    }
    output = staged;
    return true;
}

inline bool escape_identity_json(
    const char* value,
    char* output,
    size_t output_capacity
) {
    if (output == nullptr || output_capacity == 0) return false;
    output[0] = '\0';
    if (!identity_field_valid(value)) return false;

    size_t output_index = 0;
    for (size_t input_index = 0; value[input_index] != '\0'; ++input_index) {
        const char character = value[input_index];
        const size_t required = character == '"' || character == '\\' ? 2 : 1;
        if (output_index + required >= output_capacity) return false;
        if (required == 2) output[output_index++] = '\\';
        output[output_index++] = character;
    }
    output[output_index] = '\0';
    return true;
}

inline bool append_json_text(
    const char* value,
    char* output,
    size_t output_capacity,
    size_t& output_index
) {
    if (value == nullptr) return false;
    for (size_t index = 0; value[index] != '\0'; ++index) {
        if (output_index + 1 >= output_capacity) return false;
        output[output_index++] = value[index];
    }
    output[output_index] = '\0';
    return true;
}

inline bool append_identity_json_value(
    const char* value,
    char* output,
    size_t output_capacity,
    size_t& output_index
) {
    char escaped[kEscapedIdentityCapacity] = {0};
    return escape_identity_json(value, escaped, sizeof(escaped))
        && append_json_text(escaped, output, output_capacity, output_index);
}

inline bool append_protocol_capabilities_json(
    const char* const* capabilities,
    size_t capability_count,
    char* output,
    size_t output_capacity,
    size_t& output_index
) {
    if (
        !protocol_capabilities_valid(capabilities, capability_count)
        || !append_json_text("[", output, output_capacity, output_index)
    ) {
        return false;
    }
    for (size_t index = 0; index < capability_count; ++index) {
        if (
            (index > 0
                && !append_json_text(",", output, output_capacity, output_index))
            || !append_json_text("\"", output, output_capacity, output_index)
            || !append_json_text(
                capabilities[index],
                output,
                output_capacity,
                output_index
            )
            || !append_json_text("\"", output, output_capacity, output_index)
        ) {
            return false;
        }
    }
    return append_json_text("]", output, output_capacity, output_index);
}

inline bool build_identity_json(
    const char* controller_hardware_id,
    const char* driver_model,
    const char* firmware_version,
    const char* robot_model,
    const char* robot_version,
    const char* serial_number,
    const char* asset_tag,
    const char* const* protocol_capabilities,
    size_t protocol_capability_count,
    char* output,
    size_t output_capacity
) {
    if (output == nullptr || output_capacity == 0) return false;
    output[0] = '\0';
    if (!controller_hardware_id_valid(controller_hardware_id)) return false;
    size_t index = 0;
    return append_json_text(
            "{\"ControllerHardwareId\":\"",
            output,
            output_capacity,
            index
        )
        && append_json_text(
            controller_hardware_id,
            output,
            output_capacity,
            index
        )
        && append_json_text("\",\"DriverModel\":\"", output, output_capacity, index)
        && append_identity_json_value(driver_model, output, output_capacity, index)
        && append_json_text("\",\"FirmwareVersion\":\"", output, output_capacity, index)
        && append_identity_json_value(firmware_version, output, output_capacity, index)
        && append_json_text("\",\"RobotModel\":\"", output, output_capacity, index)
        && append_identity_json_value(robot_model, output, output_capacity, index)
        && append_json_text("\",\"RobotVersion\":\"", output, output_capacity, index)
        && append_identity_json_value(robot_version, output, output_capacity, index)
        && append_json_text("\",\"SerialNumber\":\"", output, output_capacity, index)
        && append_identity_json_value(serial_number, output, output_capacity, index)
        && append_json_text("\",\"AssetTag\":\"", output, output_capacity, index)
        && append_identity_json_value(asset_tag, output, output_capacity, index)
        && append_json_text(
            "\",\"ProtocolCapabilities\":",
            output,
            output_capacity,
            index
        )
        && append_protocol_capabilities_json(
            protocol_capabilities,
            protocol_capability_count,
            output,
            output_capacity,
            index
        )
        && append_json_text("}", output, output_capacity, index);
}

}

#endif
