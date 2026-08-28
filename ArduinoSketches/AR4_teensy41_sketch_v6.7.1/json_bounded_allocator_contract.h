#ifndef AR4_JSON_BOUNDED_ALLOCATOR_CONTRACT_H
#define AR4_JSON_BOUNDED_ALLOCATOR_CONTRACT_H

#include <ArduinoJson/Memory/Allocator.hpp>

#include <assert.h>
#include <new>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#if \
  ARDUINOJSON_VERSION_MAJOR != 7 \
  || ARDUINOJSON_VERSION_MINOR != 4 \
  || ARDUINOJSON_VERSION_REVISION != 3
#error "AR4 JSON allocation requires ArduinoJson 7.4.3"
#endif

namespace ar4_protocol {

template <size_t Capacity>
class JsonBoundedAllocator : public ArduinoJson::Allocator {
 public:
  enum class RecoveryStatus {
    kReady,
    kClearedFault,
    kDiscardedAllocations,
    kInvalidLimit,
  };

  JsonBoundedAllocator() {
    reset();
  }

  JsonBoundedAllocator(const JsonBoundedAllocator &) = delete;
  JsonBoundedAllocator &operator=(const JsonBoundedAllocator &) = delete;

  void *allocate(size_t requested_size) override {
    size_t aligned_size = 0;
    if (!normalize_size(requested_size, aligned_size)) {
      failed_ = true;
      return nullptr;
    }
    for (Block *block = head_; block != nullptr; block = block->next) {
      if (!block->free || block->size < aligned_size) continue;
      split(block, aligned_size);
      block->free = false;
      current_bytes_ += block->size;
      if (current_bytes_ > peak_bytes_) peak_bytes_ = current_bytes_;
      return payload(block);
    }
    failed_ = true;
    return nullptr;
  }

  void deallocate(void *pointer) override {
    if (pointer == nullptr) return;
    Block *block = find_allocated_block(pointer);
    if (block == nullptr) {
      invalid_operation_ = true;
      failed_ = true;
      return;
    }
    current_bytes_ -= block->size;
    block->free = true;
    coalesce(block);
  }

  void *reallocate(void *pointer, size_t requested_size) override {
    if (pointer == nullptr) return allocate(requested_size);
    if (requested_size == 0) {
      deallocate(pointer);
      return nullptr;
    }

    Block *block = find_allocated_block(pointer);
    if (block == nullptr) {
      invalid_operation_ = true;
      failed_ = true;
      return nullptr;
    }
    size_t aligned_size = 0;
    if (!normalize_size(requested_size, aligned_size)) {
      failed_ = true;
      return nullptr;
    }

    const size_t previous_size = block->size;
    if (aligned_size <= previous_size) {
      split(block, aligned_size);
      current_bytes_ -= previous_size - block->size;
      return pointer;
    }

    if (
      block->next != nullptr
      && block->next->free
      && aligned_size - previous_size
        <= sizeof(Block) + block->next->size
    ) {
      merge_next(block);
      split(block, aligned_size);
      current_bytes_ += block->size - previous_size;
      if (current_bytes_ > peak_bytes_) peak_bytes_ = current_bytes_;
      return pointer;
    }

    void *replacement = allocate(requested_size);
    if (replacement == nullptr) return nullptr;
    memcpy(replacement, pointer, previous_size);
    deallocate(pointer);
    return replacement;
  }

  bool reset(size_t requested_limit = Capacity) {
    if (head_ != nullptr && current_bytes_ != 0) {
      failed_ = true;
      invalid_operation_ = true;
      return false;
    }
    return initialize(requested_limit);
  }

  // Requires destruction of every allocator client before pointer invalidation.
  RecoveryStatus recover_after_quiescence(
    size_t requested_limit = Capacity
  ) {
    // Rejecting a recovery limit must preserve the arena being recovered.
    if (!valid_limit(requested_limit)) {
      failed_ = true;
      invalid_operation_ = true;
      return RecoveryStatus::kInvalidLimit;
    }
    const bool discarded_allocations = current_bytes_ != 0;
    const bool cleared_fault = failed_ || invalid_operation_;
    initialize_valid(requested_limit);
    if (discarded_allocations) {
      return RecoveryStatus::kDiscardedAllocations;
    }
    return cleared_fault
      ? RecoveryStatus::kClearedFault
      : RecoveryStatus::kReady;
  }

  bool failed() const {
    return failed_;
  }

  bool invalid_operation() const {
    return invalid_operation_;
  }

  size_t current_bytes() const {
    return current_bytes_;
  }

  size_t peak_bytes() const {
    return peak_bytes_;
  }

  size_t limit_bytes() const {
    return limit_bytes_;
  }

  static constexpr size_t storage_capacity() {
    return Capacity;
  }

  static constexpr size_t minimum_limit() {
    return sizeof(Block) + kAlignment;
  }

  static constexpr bool valid_limit(size_t requested_limit) {
    return requested_limit <= Capacity
      && align_down(requested_limit) >= minimum_limit();
  }

 private:
  struct alignas(max_align_t) Block {
    size_t size;
    Block *previous;
    Block *next;
    bool free;
  };

  static constexpr size_t kAlignment = alignof(max_align_t);

  static_assert(
    Capacity >= sizeof(Block) + kAlignment,
    "JSON allocator storage cannot hold an aligned allocation"
  );

  bool initialize(size_t requested_limit) {
    if (!valid_limit(requested_limit)) {
      failed_ = true;
      invalid_operation_ = true;
      current_bytes_ = 0;
      peak_bytes_ = 0;
      head_ = nullptr;
      limit_bytes_ = 0;
      return false;
    }
    initialize_valid(requested_limit);
    return true;
  }

  void initialize_valid(size_t requested_limit) {
    assert(valid_limit(requested_limit));
    failed_ = false;
    invalid_operation_ = false;
    current_bytes_ = 0;
    peak_bytes_ = 0;
    limit_bytes_ = align_down(requested_limit);
    head_ = new (storage_) Block{
      limit_bytes_ - sizeof(Block),
      nullptr,
      nullptr,
      true,
    };
  }

  static constexpr size_t align_down(size_t value) {
    return value - value % kAlignment;
  }

  static bool normalize_size(size_t requested_size, size_t &aligned_size) {
    if (requested_size == 0) requested_size = 1;
    if (requested_size > SIZE_MAX - (kAlignment - 1)) return false;
    aligned_size =
      (requested_size + (kAlignment - 1)) / kAlignment * kAlignment;
    return true;
  }

  static unsigned char *payload(Block *block) {
    return reinterpret_cast<unsigned char *>(block) + sizeof(Block);
  }

  bool block_address_valid(const Block *block) const {
    const uintptr_t storage_start = reinterpret_cast<uintptr_t>(storage_);
    const uintptr_t storage_end = storage_start + limit_bytes_;
    const uintptr_t block_start = reinterpret_cast<uintptr_t>(block);
    return block_start >= storage_start
      && block_start <= storage_end - sizeof(Block)
      && (block_start - storage_start) % kAlignment == 0;
  }

  bool block_layout_valid(
    const Block *block,
    const Block *expected_previous
  ) const {
    if (!block_address_valid(block)) return false;
    const uintptr_t storage_start = reinterpret_cast<uintptr_t>(storage_);
    const uintptr_t storage_end = storage_start + limit_bytes_;
    const uintptr_t block_start = reinterpret_cast<uintptr_t>(block);
    const uintptr_t payload_start = block_start + sizeof(Block);
    if (
      block->previous != expected_previous
      || block->size % kAlignment != 0
      || block->size > storage_end - payload_start
    ) {
      return false;
    }
    const uintptr_t expected_next = payload_start + block->size;
    if (block->next == nullptr) return expected_next == storage_end;
    return reinterpret_cast<uintptr_t>(block->next) == expected_next
      && block_address_valid(block->next);
  }

  Block *find_allocated_block(void *pointer) {
    Block *previous = nullptr;
    Block *block = head_;
    size_t remaining_blocks = limit_bytes_ / sizeof(Block) + 1;
    while (block != nullptr && remaining_blocks-- > 0) {
      if (!block_layout_valid(block, previous)) return nullptr;
      if (payload(block) == pointer) return block->free ? nullptr : block;
      previous = block;
      block = block->next;
    }
    return nullptr;
  }

  static void merge_next(Block *block) {
    Block *next = block->next;
    if (next == nullptr || !next->free) return;
    block->size += sizeof(Block) + next->size;
    block->next = next->next;
    if (block->next != nullptr) block->next->previous = block;
  }

  void coalesce(Block *block) {
    merge_next(block);
    if (block->previous != nullptr && block->previous->free) {
      block = block->previous;
      merge_next(block);
    }
  }

  static void split(Block *block, size_t requested_size) {
    if (
      block->size < requested_size
      || block->size - requested_size < sizeof(Block) + kAlignment
    ) {
      return;
    }
    unsigned char *next_address = payload(block) + requested_size;
    Block *next = new (next_address) Block{
      block->size - requested_size - sizeof(Block),
      block,
      block->next,
      true,
    };
    if (next->next != nullptr) next->next->previous = next;
    block->next = next;
    block->size = requested_size;
    merge_next(next);
  }

  alignas(max_align_t) unsigned char storage_[Capacity];
  Block *head_ = nullptr;
  size_t limit_bytes_ = 0;
  size_t current_bytes_ = 0;
  size_t peak_bytes_ = 0;
  bool failed_ = false;
  bool invalid_operation_ = false;
};

}  // namespace ar4_protocol

#endif
