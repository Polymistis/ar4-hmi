#ifndef AR4_MOTION_MODE_TRANSACTION_H
#define AR4_MOTION_MODE_TRANSACTION_H

#include <stddef.h>

namespace ar4_protocol {

template <typename WristState, size_t JointCount>
class MotionModeTransaction {
 public:
  MotionModeTransaction(
    WristState &active_wrist,
    int (&active_loop_modes)[JointCount],
    const WristState &requested_wrist,
    const int (&requested_loop_modes)[JointCount]
  )
    : active_wrist_(active_wrist),
      active_loop_modes_(active_loop_modes),
      previous_wrist_(active_wrist),
      requested_wrist_(requested_wrist),
      committed_(false) {
    for (size_t index = 0; index < JointCount; ++index) {
      previous_loop_modes_[index] = active_loop_modes[index];
      requested_loop_modes_[index] = requested_loop_modes[index];
    }
  }

  ~MotionModeTransaction() {
    if (!committed_) restore();
  }

  void commit() {
    if (committed_) return;
    active_wrist_ = requested_wrist_;
    for (size_t index = 0; index < JointCount; ++index) {
      active_loop_modes_[index] = requested_loop_modes_[index];
    }
    committed_ = true;
  }

  bool committed() const {
    return committed_;
  }

 private:
  void restore() {
    active_wrist_ = previous_wrist_;
    for (size_t index = 0; index < JointCount; ++index) {
      active_loop_modes_[index] = previous_loop_modes_[index];
    }
  }

  WristState &active_wrist_;
  int (&active_loop_modes_)[JointCount];
  WristState previous_wrist_;
  WristState requested_wrist_;
  int previous_loop_modes_[JointCount];
  int requested_loop_modes_[JointCount];
  bool committed_;

  MotionModeTransaction(const MotionModeTransaction &) = delete;
  MotionModeTransaction &operator=(const MotionModeTransaction &) = delete;
};

}  // namespace ar4_protocol

#endif
