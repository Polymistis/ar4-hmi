#ifndef AR4_JSON_SESSION_IDENTITY_CONTRACT_H
#define AR4_JSON_SESSION_IDENTITY_CONTRACT_H

#include <stddef.h>
#include <stdint.h>

#include "json_session_contract.h"
#include "persistence_contract.h"

namespace ar4_protocol {

constexpr uint32_t kJsonSessionCounterMagic = 0x41524A31UL;
constexpr uint32_t kJsonSessionCounterTransaction = 0x41524A30UL;
constexpr int kJsonSessionCounterStorageAddress =
  (kDebugMagicAddress + static_cast<int>(sizeof(uint32_t)) + 3) & ~3;
constexpr int kJsonSessionCounterSlotSize =
  static_cast<int>(sizeof(uint32_t) + 2 * sizeof(uint64_t));
constexpr int kJsonSessionCounterSlotAddresses[] = {
  kJsonSessionCounterStorageAddress,
  kJsonSessionCounterStorageAddress + kJsonSessionCounterSlotSize,
};
constexpr size_t kJsonSessionCounterStorageEnd = static_cast<size_t>(
  kJsonSessionCounterSlotAddresses[1] + kJsonSessionCounterSlotSize
);

static_assert(
  kJsonSessionCounterTransaction != kJsonSessionCounterMagic,
  "JSON session transaction and commit markers must remain distinct"
);
static_assert(
  kJsonSessionCounterStorageAddress
    >= kDebugMagicAddress + static_cast<int>(sizeof(uint32_t)),
  "JSON session storage must not overlap identity or debug persistence"
);

inline bool json_session_storage_fits(size_t eeprom_length) {
  return eeprom_length >= kJsonSessionCounterStorageEnd;
}

enum class JsonSessionCounterSlotStatus : uint8_t {
  kErased,
  kTransaction,
  kValid,
  kCorrupt,
};

struct JsonSessionCounterSlot {
  JsonSessionCounterSlotStatus status;
  uint64_t counter;
};

enum class JsonSessionIdentityStatus : uint8_t {
  kAvailable,
  kInvalidHardwareIdentity,
  kInvalidOutputBuffer,
  kPersistenceUnavailable,
  kCounterExhausted,
};

template <typename Eeprom>
inline JsonSessionCounterSlot read_json_session_counter_slot(
  Eeprom &eeprom,
  int address
) {
  uint32_t marker = 0;
  eeprom.get(address, marker);
  if (marker == kIdentityErasedMarker) {
    return {JsonSessionCounterSlotStatus::kErased, 0};
  }
  if (marker == kJsonSessionCounterTransaction) {
    return {JsonSessionCounterSlotStatus::kTransaction, 0};
  }
  if (marker != kJsonSessionCounterMagic) {
    return {JsonSessionCounterSlotStatus::kCorrupt, 0};
  }

  uint64_t counter = 0;
  uint64_t counter_inverse = 0;
  eeprom.get(address + static_cast<int>(sizeof(marker)), counter);
  eeprom.get(
    address + static_cast<int>(sizeof(marker) + sizeof(counter)),
    counter_inverse
  );
  if (counter == 0 || counter_inverse != ~counter) {
    return {JsonSessionCounterSlotStatus::kCorrupt, 0};
  }
  return {JsonSessionCounterSlotStatus::kValid, counter};
}

template <typename Eeprom>
inline bool write_json_session_counter_slot(
  Eeprom &eeprom,
  int address,
  uint64_t counter
) {
  if (counter == 0) return false;
  if (!write_eeprom_verified(
      eeprom,
      address,
      kJsonSessionCounterTransaction
  )) {
    return false;
  }
  if (!write_eeprom_verified(
      eeprom,
      address + static_cast<int>(sizeof(uint32_t)),
      counter
  )) {
    return false;
  }
  const uint64_t counter_inverse = ~counter;
  if (!write_eeprom_verified(
      eeprom,
      address + static_cast<int>(sizeof(uint32_t) + sizeof(uint64_t)),
      counter_inverse
  )) {
    return false;
  }
  if (!write_eeprom_verified(
      eeprom,
      address,
      kJsonSessionCounterMagic
  )) {
    return false;
  }
  const JsonSessionCounterSlot verified =
    read_json_session_counter_slot(eeprom, address);
  return verified.status == JsonSessionCounterSlotStatus::kValid
    && verified.counter == counter;
}

inline bool format_json_session_identifier(
  const char *controller_hardware_id,
  uint64_t counter,
  char *output,
  size_t output_capacity
) {
  if (output != nullptr && output_capacity > 0) output[0] = '\0';
  if (
    !controller_hardware_id_valid(controller_hardware_id)
    || counter == 0
    || output == nullptr
    || output_capacity < kJsonSessionIdentifierLength + 1
  ) {
    return false;
  }
  static const char kHexDigits[] = "0123456789ABCDEF";
  size_t index = 0;
  for (; index < kControllerHardwareIdLength; ++index) {
    output[index] = controller_hardware_id[index];
  }
  for (; index < kJsonSessionIdentifierLength - 16; ++index) {
    output[index] = '0';
  }
  for (size_t digit = 0; digit < 16; ++digit) {
    const size_t shift = (15 - digit) * 4;
    output[index + digit] = kHexDigits[(counter >> shift) & 0x0FULL];
  }
  output[kJsonSessionIdentifierLength] = '\0';
  return json_session_identifier_valid(output);
}

template <typename Eeprom>
inline JsonSessionIdentityStatus advance_json_session_identity(
  Eeprom &eeprom,
  const char *controller_hardware_id,
  char *output,
  size_t output_capacity
) {
  if (output != nullptr && output_capacity > 0) output[0] = '\0';
  if (!controller_hardware_id_valid(controller_hardware_id)) {
    return JsonSessionIdentityStatus::kInvalidHardwareIdentity;
  }
  if (
    output == nullptr
    || output_capacity < kJsonSessionIdentifierLength + 1
  ) {
    return JsonSessionIdentityStatus::kInvalidOutputBuffer;
  }
  if (!json_session_storage_fits(
      static_cast<size_t>(eeprom.length())
  )) {
    return JsonSessionIdentityStatus::kPersistenceUnavailable;
  }

  const JsonSessionCounterSlot slots[] = {
    read_json_session_counter_slot(
      eeprom,
      kJsonSessionCounterSlotAddresses[0]
    ),
    read_json_session_counter_slot(
      eeprom,
      kJsonSessionCounterSlotAddresses[1]
    ),
  };
  for (const JsonSessionCounterSlot &slot : slots) {
    if (slot.status == JsonSessionCounterSlotStatus::kCorrupt) {
      return JsonSessionIdentityStatus::kPersistenceUnavailable;
    }
  }

  int current_index = -1;
  if (slots[0].status == JsonSessionCounterSlotStatus::kValid) {
    current_index = 0;
  }
  if (
    slots[1].status == JsonSessionCounterSlotStatus::kValid
    && (current_index < 0 || slots[1].counter > slots[0].counter)
  ) {
    current_index = 1;
  }
  if (
    slots[0].status == JsonSessionCounterSlotStatus::kValid
    && slots[1].status == JsonSessionCounterSlotStatus::kValid
  ) {
    const uint64_t lower = slots[0].counter < slots[1].counter
      ? slots[0].counter
      : slots[1].counter;
    const uint64_t upper = slots[0].counter < slots[1].counter
      ? slots[1].counter
      : slots[0].counter;
    if (upper - lower != 1) {
      return JsonSessionIdentityStatus::kPersistenceUnavailable;
    }
  }

  const uint64_t current_counter = current_index < 0
    ? 0
    : slots[current_index].counter;
  if (current_counter == UINT64_MAX) {
    return JsonSessionIdentityStatus::kCounterExhausted;
  }
  const uint64_t next_counter = current_counter + 1;
  const int target_index = current_index == 0 ? 1 : 0;
  if (!write_json_session_counter_slot(
      eeprom,
      kJsonSessionCounterSlotAddresses[target_index],
      next_counter
  )) {
    return JsonSessionIdentityStatus::kPersistenceUnavailable;
  }
  if (!format_json_session_identifier(
      controller_hardware_id,
      next_counter,
      output,
      output_capacity
  )) {
    return JsonSessionIdentityStatus::kInvalidOutputBuffer;
  }
  return JsonSessionIdentityStatus::kAvailable;
}

}  // namespace ar4_protocol

#endif
