#ifndef AR4_PERSISTENCE_CONTRACT_H
#define AR4_PERSISTENCE_CONTRACT_H

#include <stdint.h>
#include <string.h>

#include "identity_contract.h"

namespace ar4_protocol {

constexpr int kIdentityMagicAddress = 0;
constexpr int kDebugValueAddress = 4;
constexpr int kRobotModelAddress = 5;
constexpr int kRobotVersionAddress = 37;
constexpr int kDriverBoardAddress = 69;
constexpr int kSerialNumberAddress = 101;
constexpr int kAssetTagAddress = 133;
constexpr int kIdentityFieldStorageSize = 32;
static_assert(
    kIdentityFieldStorageSize == kIdentityFieldMaximumLength + 1,
    "Identity storage must include the maximum field and terminator"
);
constexpr int kDebugMagicAddress =
    kAssetTagAddress + kIdentityFieldStorageSize;

constexpr uint32_t kIdentityLegacyMagicNumber = 0x41523401UL;
constexpr uint32_t kIdentityMagicNumber = 0x41523402UL;
constexpr uint32_t kIdentityTransactionMarker = 0x41523400UL;
// Teensy 4.1 EEPROM emulation reads unwritten bytes as 0xFF. Any other
// non-commit marker represents an interrupted or incompatible record.
constexpr uint32_t kIdentityErasedMarker = 0xFFFFFFFFUL;
constexpr uint8_t kLegacyErasedDebugValue = 0xFF;
constexpr bool kLegacyDefaultDebugValue = false;
constexpr uint32_t kDebugMagicNumber = 0x41524401UL;
static_assert(
    kIdentityTransactionMarker != kIdentityMagicNumber
        && kIdentityTransactionMarker != kIdentityLegacyMagicNumber
        && kIdentityMagicNumber != kIdentityLegacyMagicNumber,
    "Identity schema and transaction markers must remain distinct"
);
static_assert(
    kIdentityErasedMarker != kIdentityMagicNumber
        && kIdentityErasedMarker != kIdentityLegacyMagicNumber
        && kIdentityErasedMarker != kIdentityTransactionMarker,
    "Erased identity storage must remain distinct from transaction states"
);

enum class IdentityRecordStatus : uint8_t {
    kUninitialized,
    kMigrationRequired,
    kValid,
    kCorrupt,
};

enum class PersistenceMigrationStatus : uint8_t {
    kNotRequired,
    kMigrated,
    kFailed,
};

inline bool decode_legacy_debug_value(uint8_t stored, bool& value) {
    if (stored == kLegacyErasedDebugValue) {
        value = kLegacyDefaultDebugValue;
        return true;
    }
    if (stored > 1) return false;
    value = stored == 1;
    return true;
}

template <typename Eeprom, typename Value>
inline bool write_eeprom_verified(
    Eeprom& eeprom,
    int address,
    const Value& value
) {
    eeprom.put(address, value);
    Value verified = {};
    eeprom.get(address, verified);
    return verified == value;
}

template <typename Eeprom>
inline bool eeprom_marker_valid(
    Eeprom& eeprom,
    int address,
    uint32_t expected
) {
    uint32_t marker = 0;
    eeprom.get(address, marker);
    return marker == expected;
}

template <typename Eeprom>
inline bool load_debug_record(Eeprom& eeprom, bool& value) {
    if (!eeprom_marker_valid(eeprom, kDebugMagicAddress, kDebugMagicNumber)) {
        return false;
    }

    uint8_t stored = 0;
    eeprom.get(kDebugValueAddress, stored);
    if (stored > 1) return false;
    value = stored == 1;
    return true;
}

template <typename Eeprom>
inline bool load_identity_field(
    Eeprom& eeprom,
    int address,
    char (&value)[kIdentityFieldStorageSize]
) {
    eeprom.get(address, value);
    if (value[kIdentityFieldStorageSize - 1] != '\0') return false;
    return identity_field_valid(value);
}

template <typename Eeprom>
inline bool write_identity_field(
    Eeprom& eeprom,
    int address,
    const char* value
) {
    if (!identity_field_valid(value)) return false;

    char stored[kIdentityFieldStorageSize] = {0};
    const size_t length = strlen(value);
    memcpy(stored, value, length);
    eeprom.put(address, stored);

    char verified[kIdentityFieldStorageSize] = {0};
    eeprom.get(address, verified);
    return memcmp(stored, verified, sizeof(stored)) == 0;
}

template <typename Eeprom>
inline IdentityRecordStatus load_identity_record(
    Eeprom& eeprom,
    char (&robot_model)[kIdentityFieldStorageSize],
    char (&robot_version)[kIdentityFieldStorageSize],
    char (&driver_board)[kIdentityFieldStorageSize],
    char (&serial_number)[kIdentityFieldStorageSize],
    char (&asset_tag)[kIdentityFieldStorageSize]
) {
    uint32_t marker = 0;
    eeprom.get(kIdentityMagicAddress, marker);
    if (marker == kIdentityErasedMarker) {
        return IdentityRecordStatus::kUninitialized;
    }
    if (marker == kIdentityLegacyMagicNumber) {
        return IdentityRecordStatus::kMigrationRequired;
    }
    if (marker != kIdentityMagicNumber) {
        return IdentityRecordStatus::kCorrupt;
    }

    if (
        !load_identity_field(eeprom, kRobotModelAddress, robot_model)
        || !load_identity_field(eeprom, kRobotVersionAddress, robot_version)
        || !load_identity_field(eeprom, kDriverBoardAddress, driver_board)
        || !load_identity_field(eeprom, kSerialNumberAddress, serial_number)
        || !load_identity_field(eeprom, kAssetTagAddress, asset_tag)
    ) {
        return IdentityRecordStatus::kCorrupt;
    }
    return IdentityRecordStatus::kValid;
}

template <typename Eeprom>
inline bool save_debug_record(Eeprom& eeprom, bool value) {
    const uint32_t invalid_marker = 0;
    if (
        !write_eeprom_verified(
            eeprom,
            kDebugMagicAddress,
            invalid_marker
        )
    ) {
        return false;
    }

    const uint8_t stored = value ? 1 : 0;
    if (!write_eeprom_verified(eeprom, kDebugValueAddress, stored)) {
        return false;
    }
    return write_eeprom_verified(
        eeprom,
        kDebugMagicAddress,
        kDebugMagicNumber
    );
}

template <typename Eeprom, typename FieldWriter>
inline bool save_identity_record(Eeprom& eeprom, FieldWriter field_writer) {
    if (
        !write_eeprom_verified(
            eeprom,
            kIdentityMagicAddress,
            kIdentityTransactionMarker
        )
    ) {
        return false;
    }
    if (!field_writer()) return false;
    return write_eeprom_verified(
        eeprom,
        kIdentityMagicAddress,
        kIdentityMagicNumber
    );
}

template <typename Eeprom>
inline bool load_legacy_identity_field(
    Eeprom& eeprom,
    int address,
    char (&value)[kIdentityFieldStorageSize]
) {
    eeprom.get(address, value);
    bool erased = true;
    for (size_t index = 0; index < sizeof(value); ++index) {
        if (static_cast<unsigned char>(value[index]) != 0xFF) {
            erased = false;
            break;
        }
    }
    if (erased) {
        const char unset[] = "Unset";
        memset(value, 0, sizeof(value));
        memcpy(value, unset, sizeof(unset));
        return true;
    }

    bool terminated = false;
    for (size_t index = 0; index < sizeof(value); ++index) {
        if (value[index] == '\0') {
            terminated = true;
            break;
        }
    }
    return terminated && identity_field_valid(value);
}

template <typename Eeprom>
inline PersistenceMigrationStatus migrate_legacy_persistence(Eeprom& eeprom) {
    uint32_t marker = 0;
    eeprom.get(kIdentityMagicAddress, marker);
    if (marker != kIdentityLegacyMagicNumber) {
        return PersistenceMigrationStatus::kNotRequired;
    }

    char robot_model[kIdentityFieldStorageSize] = {0};
    char robot_version[kIdentityFieldStorageSize] = {0};
    char driver_board[kIdentityFieldStorageSize] = {0};
    char serial_number[kIdentityFieldStorageSize] = {0};
    char asset_tag[kIdentityFieldStorageSize] = {0};
    if (
        !load_legacy_identity_field(
            eeprom,
            kRobotModelAddress,
            robot_model
        )
        || !load_legacy_identity_field(
            eeprom,
            kRobotVersionAddress,
            robot_version
        )
        || !load_legacy_identity_field(
            eeprom,
            kDriverBoardAddress,
            driver_board
        )
        || !load_legacy_identity_field(
            eeprom,
            kSerialNumberAddress,
            serial_number
        )
        || !load_legacy_identity_field(
            eeprom,
            kAssetTagAddress,
            asset_tag
        )
    ) {
        return PersistenceMigrationStatus::kFailed;
    }

    uint8_t stored_debug = 0;
    eeprom.get(kDebugValueAddress, stored_debug);
    bool debug_value = false;
    if (!decode_legacy_debug_value(stored_debug, debug_value)) {
        return PersistenceMigrationStatus::kFailed;
    }
    if (!save_debug_record(eeprom, debug_value)) {
        return PersistenceMigrationStatus::kFailed;
    }
    if (!save_identity_record(eeprom, [&]() {
        return write_identity_field(eeprom, kRobotModelAddress, robot_model)
            && write_identity_field(
                eeprom,
                kRobotVersionAddress,
                robot_version
            )
            && write_identity_field(
                eeprom,
                kDriverBoardAddress,
                driver_board
            )
            && write_identity_field(
                eeprom,
                kSerialNumberAddress,
                serial_number
            )
            && write_identity_field(eeprom, kAssetTagAddress, asset_tag);
    })) {
        return PersistenceMigrationStatus::kFailed;
    }

    bool verified_debug = false;
    char verified_robot_model[kIdentityFieldStorageSize] = {0};
    char verified_robot_version[kIdentityFieldStorageSize] = {0};
    char verified_driver_board[kIdentityFieldStorageSize] = {0};
    char verified_serial_number[kIdentityFieldStorageSize] = {0};
    char verified_asset_tag[kIdentityFieldStorageSize] = {0};
    if (
        !load_debug_record(eeprom, verified_debug)
        || verified_debug != debug_value
        || load_identity_record(
            eeprom,
            verified_robot_model,
            verified_robot_version,
            verified_driver_board,
            verified_serial_number,
            verified_asset_tag
        ) != IdentityRecordStatus::kValid
    ) {
        return PersistenceMigrationStatus::kFailed;
    }
    return PersistenceMigrationStatus::kMigrated;
}

}  // namespace ar4_protocol

#endif
