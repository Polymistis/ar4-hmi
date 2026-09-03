#ifndef AR4_JSON_MAIN_CONTROLLER_DISPATCH_CONTRACT_H
#define AR4_JSON_MAIN_CONTROLLER_DISPATCH_CONTRACT_H

#include <stddef.h>
#include <stdint.h>

#include "json_request_disposition_contract.h"

namespace ar4_protocol {

constexpr size_t kJsonMainControllerOutputCapacity =
  kJsonProtocolMaximumFrameBytes;

enum class JsonMainHelloSourceStatus {
  kAvailable,
  kIdentityUnavailable,
  kSessionUnavailable,
};

struct JsonMainHelloResponseSource {
  JsonMainHelloSourceStatus status;
  const char *session_id;
  const char *firmware_name;
  const char *firmware_version;
  const char *firmware_build;
  const char *controller_hardware_id;
  const char *driver_model;
  const char *robot_model;
  const char *robot_version;
  const char *serial_number;
  const char *asset_tag;
  const char *const *capabilities;
  size_t capability_count;
  const char *const *commands;
  size_t command_count;
};

enum class JsonMainHomeReferenceSourceStatus {
  kAvailable,
  kHomeReferenceUnavailable,
};

struct JsonMainHomeReferenceResponseSource {
  JsonMainHomeReferenceSourceStatus status;
  PrimaryHomeReferenceState state;
};

enum class JsonMainPositionSourceStatus {
  kAvailable,
  kPositionUnavailable,
  kDispositionUnavailable,
  kControllerAlarm,
};

struct JsonMainPositionResponseSource {
  JsonMainPositionSourceStatus status;
  JsonMainPositionSnapshot snapshot;
  bool speed_limited;
  char controller_debug[kJsonPositionControllerDebugMaximumLength + 1];
  char motion_fault[kJsonPositionMotionFaultMaximumLength + 1];
  char controller_alarm[kJsonPositionControllerAlarmMaximumLength + 1];
};

enum class JsonMainCorrectPositionOutcome {
  kCompleted,
  kEncoderStateUnavailable,
  kPositionUnavailable,
};

struct JsonMainCorrectPositionResult {
  JsonMainCorrectPositionOutcome outcome;
  bool axes[6];
  JsonMainPositionResponseSource position;
};

enum class JsonMainCorrectPositionApplyStatus {
  kApplied,
  kEmergencyStopActive,
  kInvalid,
};

using JsonMainCorrectPositionPrepare = bool (*)(
  JsonMainCorrectPositionResult &,
  void *
);

using JsonMainCorrectPositionApply = JsonMainCorrectPositionApplyStatus (*)(
  void *
);

struct JsonMainCorrectPositionCommandSource {
  JsonMainCorrectPositionPrepare prepare;
  JsonMainCorrectPositionApply apply;
  void *context;
};

enum class JsonMainExternalAxisZeroApplyStatus {
  kApplied,
  kEmergencyStopActive,
  kInvalid,
};

using JsonMainExternalAxisZeroStagePostZeroPosition = bool (*)(
  uint8_t axis,
  JsonMainPositionResponseSource &post_zero_position,
  void *context
);
using JsonMainExternalAxisZeroApply = JsonMainExternalAxisZeroApplyStatus (*)(
  uint8_t,
  void *
);

struct JsonMainExternalAxisZeroCommandSource {
  JsonMainExternalAxisZeroStagePostZeroPosition stage_post_zero_position;
  JsonMainExternalAxisZeroApply apply;
  void *context;
};

enum class JsonMainSetPositionApplyStatus {
  kApplied,
  kPositionNotRepresentable,
  kEmergencyStopActive,
};

using JsonMainSetPositionApply = JsonMainSetPositionApplyStatus (*)(
  const JsonMainSetPositionParameters &,
  void *
);

struct JsonMainSetPositionCommandSource {
  JsonMainSetPositionApply apply;
  void *context;
};

enum class JsonMainConfigurationApplyStatus {
  kApplied,
  kConfigurationNotRepresentable,
  kEmergencyStopActive,
};

using JsonMainUpdateParametersApply = JsonMainConfigurationApplyStatus (*)(
  const JsonMainUpdateParameters &,
  void *
);

struct JsonMainUpdateParametersCommandSource {
  JsonMainUpdateParametersApply apply;
  void *context;
};

using JsonMainExternalAxisApply = JsonMainConfigurationApplyStatus (*)(
  const JsonMainExternalAxisParameters &,
  void *
);

struct JsonMainExternalAxisCommandSource {
  JsonMainExternalAxisApply apply;
  void *context;
};

using JsonMainCalibrationExecute = bool (*)(
  const JsonMainCalibrationParameters &,
  JsonMainCalibrationExecutionResult &,
  void *
);

struct JsonMainCalibrationCommandSource {
  JsonMainCalibrationExecute execute;
  void *context;
};

using JsonMainMoveJointsExecute = bool (*)(
  uint32_t,
  const JsonMainMoveJointsParameters &,
  JsonMainMoveJointsExecutionResult &,
  void *
);

struct JsonMainMoveJointsCommandSource {
  JsonMainMoveJointsExecute execute;
  void *context;
};

using JsonMainMoveCartesianExecute = bool (*)(
  const JsonMainMoveCartesianParameters &,
  JsonMainMoveCartesianExecutionResult &,
  void *
);

struct JsonMainMoveCartesianCommandSource {
  JsonMainMoveCartesianExecute execute;
  void *context;
};

using JsonMainToolJogExecute = bool (*)(
  const JsonMainToolJogParameters &,
  JsonMainToolJogExecutionResult &,
  void *
);

struct JsonMainToolJogCommandSource {
  JsonMainToolJogExecute execute;
  void *context;
};

using JsonMainLiveJogBegin = bool (*)(
  uint32_t,
  const JsonMainLiveJogParameters &,
  void *
);

using JsonMainLiveJogStop = bool (*)(uint32_t, void *);

enum class JsonMainLiveJogRenewStatus : uint8_t {
  kRenewed,
  kLeaseExpired,
  kMotionSettled,
  kFault,
};

using JsonMainLiveJogRenew = JsonMainLiveJogRenewStatus (*)(
  uint32_t,
  void *
);

struct JsonMainLiveJogCommandSource {
  uint32_t active_motion_id;
  JsonMainLiveJogBegin begin;
  JsonMainLiveJogStop stop;
  JsonMainLiveJogRenew renew;
  void *context;
};

enum class JsonMainDiagnosticOutcome {
  kCompleted,
  kEmergencyStopActive,
  kUnavailable,
};

using JsonMainDiagnosticExecute = JsonMainDiagnosticOutcome (*)(
  JsonMainRequestCommand,
  bool *,
  int32_t *,
  void *
);

struct JsonMainDiagnosticCommandSource {
  JsonMainDiagnosticExecute execute;
  void *context;
};

struct JsonMainMotionTraceResponseSource {
  const ControllerMotionTraceCapture *capture;
  const char *session_id;
  const char *firmware_name;
  const char *firmware_version;
  const char *firmware_build;
};

enum class JsonMainControllerWaitOutcome : uint8_t {
  kInvalid,
  kCompleted,
  kEmergencyStop,
};

using JsonMainControllerWaitExecute = JsonMainControllerWaitOutcome (*)(
  uint32_t duration_milliseconds, void *context
);

struct JsonMainControllerWaitCommandSource {
  JsonMainControllerWaitExecute execute;
  void *context;
};

enum class JsonMainModbusReadOutcome : uint8_t {
  kInvalid,
  kCompleted,
  kModbusError,
  kEmergencyStop,
};

using JsonMainModbusReadExecute = JsonMainModbusReadOutcome (*)(
  ModbusOperation, const JsonMainModbusReadParameters &, int32_t &, void *
);

struct JsonMainModbusReadCommandSource {
  JsonMainModbusReadExecute execute;
  void *context;
};

enum class JsonMainModbusWriteOutcome : uint8_t {
  kInvalid,
  kCompleted,
  kModbusError,
  kEmergencyStop,
};

using JsonMainModbusWriteExecute = JsonMainModbusWriteOutcome (*)(
  ModbusOperation, const JsonMainModbusWriteParameters &, void *
);

struct JsonMainModbusWriteCommandSource {
  JsonMainModbusWriteExecute execute;
  void *context;
};

enum class JsonMainControllerAdmissionStatus {
  kAvailable,
  kEmergencyStopActive,
};

struct JsonMainControllerDispatchSources {
  const JsonMainHelloResponseSource *hello;
  const JsonMainHomeReferenceResponseSource *home_reference;
  const JsonMainPositionResponseSource *position;
  JsonMainControllerAdmissionStatus admission;
  const JsonMainSetPositionCommandSource *set_position;
  const JsonMainUpdateParametersCommandSource *update_params;
  const JsonMainExternalAxisCommandSource *config_ext_axis;
  const JsonMainMoveJointsCommandSource *move_joints;
  const JsonMainMoveCartesianCommandSource *move_cartesian;
  const JsonMainToolJogCommandSource *jog_tool;
  const JsonMainLiveJogCommandSource *live_jog;
  const JsonMainDiagnosticCommandSource *diagnostics;
  const JsonMainMotionTraceResponseSource *motion_trace;
  const JsonMainCalibrationCommandSource *calibration;
  const JsonMainCorrectPositionCommandSource *correct_position;
  const JsonMainExternalAxisZeroCommandSource *zero_external_axis;
  const JsonMainControllerWaitCommandSource *controller_wait;
  const JsonMainModbusReadCommandSource *modbus_read;
  const JsonMainModbusWriteCommandSource *modbus_write;
  const JsonMainDirectCommandSource *direct;
};

enum class JsonMainControllerResponseKind {
  kNone,
  kHelloCompleted,
  kHelloFailed,
  kHomeReferenceCompleted,
  kHomeReferenceFailed,
  kPositionDispositionCompleted,
  kPositionFailed,
  kPositionAlarmFailed,
  kCorrectPositionCompleted,
  kCorrectPositionEncoderFailed,
  kCorrectPositionFailed,
  kExternalAxisZeroCompleted,
  kExternalAxisZeroFailed,
  kDiagnosticCompleted,
  kDiagnosticRejected,
  kDiagnosticFailed,
  kMotionTraceCompleted,
  kMotionTraceRejected,
  kSetPositionCompleted,
  kSetPositionRejected,
  kUpdateParamsCompleted,
  kUpdateParamsRejected,
  kConfigExtAxisCompleted,
  kConfigExtAxisRejected,
  kControllerWaitCompleted,
  kControllerWaitRejected,
  kControllerWaitCancelled,
  kModbusReadCompleted,
  kModbusReadRejected,
  kModbusReadCancelled,
  kModbusReadFailed,
  kModbusWriteCompleted,
  kModbusWriteRejected,
  kModbusWriteCancelled,
  kModbusWriteFailed,
  kCalibrationCompleted,
  kCalibrationRejected,
  kCalibrationCancelled,
  kCalibrationFailed,
  kJogToolCompleted,
  kJogToolRejected,
  kJogToolCancelled,
  kJogToolFailed,
  kLiveJogAccepted,
  kLiveJogCompleted,
  kLiveJogCancelled,
  kLiveJogFailed,
  kStopCompleted,
  kStopRejected,
  kRenewLiveMotionCompleted,
  kRenewLiveMotionRejected,
  kMoveCartesianCompleted,
  kMoveCartesianRejected,
  kMoveCartesianCancelled,
  kMoveCartesianFailed,
  kMoveJointsCompleted,
  kMoveJointsRejected,
  kMoveJointsCancelled,
  kMoveJointsFailed,
  kDirectAccepted,
  kDirectCompleted,
  kDirectRejected,
  kDirectCancelled,
  kDirectFailed,
  kAdmissionRejected,
  kCorrelatedRejection,
  kProtocolError,
};

enum class JsonMainControllerOwnerState {
  kIdle,
  kResponseReady,
  kWriteInProgress,
  kFaulted,
};

enum class JsonMainControllerFault {
  kNone,
  kInvalidRequestArgument,
  kInternalStorageAlias,
  kInvalidCommandSource,
  kParserInvariant,
  kResponseSerializationFailure,
  kInvalidOutputLifecycle,
  kAmbiguousOutputWrite,
  kInvalidRecoveryLimit,
};

enum class JsonMainControllerProcessStatus {
  kResponseReady,
  kBusy,
  kControllerFault,
  kSessionFaulted,
};

enum class JsonMainControllerOutputBeginStatus {
  kReady,
  kNoResponse,
  kBusy,
  kControllerFault,
  kSessionFaulted,
};

enum class JsonMainControllerOutputCompletionStatus {
  kCompleted,
  kControllerFault,
  kSessionFaulted,
};

enum class JsonMainControllerRecoveryStatus {
  kReady,
  kClearedFault,
  kDiscardedParserAllocations,
  kBusy,
  kInvalidLimit,
};

struct JsonMainControllerFrameView {
  const char *data;
  size_t length;
  JsonMainControllerResponseKind kind;
};

class JsonMainControllerRequestOwner {
 public:
  JsonMainControllerRequestOwner()
    : parser_workspace_(),
      semantic_allocation_limit_(kJsonRequestSemanticStorageBytes),
      state_(JsonMainControllerOwnerState::kIdle),
      fault_(JsonMainControllerFault::kNone),
      response_kind_(JsonMainControllerResponseKind::kNone),
      session_bound_(false),
      configuration_sync_required_(false),
      pending_live_motion_id_(0),
      pending_live_kind_(JsonLiveJogKind::kInvalid),
      active_live_motion_id_(0),
      active_live_kind_(JsonLiveJogKind::kInvalid),
      pending_playback_request_id_(0),
      active_playback_request_id_(0),
      playback_execution_active_(false),
      playback_parameters_(),
      frame_length_(0) {
    output_[0] = '\0';
  }

  JsonMainControllerRequestOwner(
    const JsonMainControllerRequestOwner &
  ) = delete;
  JsonMainControllerRequestOwner &operator=(
    const JsonMainControllerRequestOwner &
  ) = delete;
  JsonMainControllerRequestOwner(
    JsonMainControllerRequestOwner &&
  ) = delete;
  JsonMainControllerRequestOwner &operator=(
    JsonMainControllerRequestOwner &&
  ) = delete;

  JsonMainControllerProcessStatus process_payload(
    const char *payload,
    size_t payload_length,
    const JsonMainControllerDispatchSources &sources,
    size_t maximum_payload_bytes
  ) {
    switch (state_) {
      case JsonMainControllerOwnerState::kIdle:
        break;
      case JsonMainControllerOwnerState::kResponseReady:
      case JsonMainControllerOwnerState::kWriteInProgress:
        return JsonMainControllerProcessStatus::kBusy;
      case JsonMainControllerOwnerState::kFaulted:
        return JsonMainControllerProcessStatus::kSessionFaulted;
    }
    if (state_ != JsonMainControllerOwnerState::kIdle) {
      return fault_process(
        JsonMainControllerFault::kInvalidOutputLifecycle
      );
    }
    if (active_playback_request_id_ != 0) {
      return JsonMainControllerProcessStatus::kBusy;
    }
    if (
      pending_live_motion_id_ != 0
      || pending_live_kind_ != JsonLiveJogKind::kInvalid
      || pending_playback_request_id_ != 0
      || playback_execution_active_
    ) {
      return fault_process(
        JsonMainControllerFault::kInvalidOutputLifecycle
      );
    }
    if (
      payload == nullptr
      || maximum_payload_bytes == 0
      || maximum_payload_bytes > kJsonProtocolMaximumPayloadBytes
    ) {
      return fault_process(
        JsonMainControllerFault::kInvalidRequestArgument
      );
    }
    if (
      range_aliases_internal_storage(&sources, sizeof(sources))
      || range_aliases_internal_storage(payload, payload_length)
      || command_sources_alias_internal_storage(sources)
    ) {
      return fault_process(JsonMainControllerFault::kInternalStorageAlias);
    }
    clear_frame();

    JsonMainRequestParseResult result = {};
    const JsonMainRequestParseOptions parse_options = {
      semantic_allocation_limit_,
    };
    const JsonMainRequestParseStatus parse_status = parse_main_json_request(
      payload,
      payload_length,
      parser_workspace_,
      result,
      parse_options
    );
    if (parse_status != result.status) {
      return fault_process(JsonMainControllerFault::kParserInvariant);
    }

    const JsonMainRequestParseResponseStatus response_status =
      build_main_json_parse_response(
        result,
        maximum_payload_bytes,
        output_,
        sizeof(output_)
      );
    if (
      active_live_motion_id_ != 0
      && (
        sources.live_jog == nullptr
        || sources.live_jog->active_motion_id != active_live_motion_id_
      )
    ) {
      return fault_process(JsonMainControllerFault::kInvalidCommandSource);
    }
    if (
      active_live_motion_id_ != 0
      && response_status
        != JsonMainRequestParseResponseStatus::kDispatchStop
      && response_status
        != JsonMainRequestParseResponseStatus::kDispatchRenewLiveMotion
      && response_status_dispatches(response_status)
    ) {
      return process_live_motion_exclusion(
        result,
        sources.admission,
        maximum_payload_bytes
      );
    }
    switch (response_status) {
      case JsonMainRequestParseResponseStatus::kDispatchHello:
        return process_hello(
          result,
          sources.hello,
          sources.admission,
          maximum_payload_bytes
        );
      case JsonMainRequestParseResponseStatus::kDispatchGetHomeReference:
        return process_home_reference(
          result,
          sources.home_reference,
          sources.admission,
          maximum_payload_bytes
        );
      case JsonMainRequestParseResponseStatus::kDispatchGetPositionDisposition:
        return process_position(
          result,
          sources.position,
          sources.admission,
          maximum_payload_bytes
        );
      case JsonMainRequestParseResponseStatus::kDispatchCorrectPosition:
        return process_correct_position(
          result,
          sources.correct_position,
          sources.admission,
          maximum_payload_bytes
        );
      case JsonMainRequestParseResponseStatus::kDispatchZeroExternalAxis:
        return process_external_axis_zero(
          result,
          sources.zero_external_axis,
          sources.admission,
          maximum_payload_bytes
        );
      case JsonMainRequestParseResponseStatus::kDispatchTestLimitSwitches:
      case JsonMainRequestParseResponseStatus::kDispatchSetEncoders:
      case JsonMainRequestParseResponseStatus::kDispatchReadEncoders:
        return process_diagnostic(
          result, sources.diagnostics, sources.admission,
          maximum_payload_bytes
        );
      case JsonMainRequestParseResponseStatus::kDispatchGetMotionTrace:
        return process_motion_trace(
          result,
          sources.motion_trace,
          sources.admission,
          maximum_payload_bytes
        );
      case JsonMainRequestParseResponseStatus::kDispatchSetPosition:
        return process_set_position(
          result,
          sources.set_position,
          sources.admission,
          maximum_payload_bytes
        );
      case JsonMainRequestParseResponseStatus::kDispatchUpdateParams:
        return process_update_params(
          result,
          sources.update_params,
          sources.admission,
          maximum_payload_bytes
        );
      case JsonMainRequestParseResponseStatus::kDispatchConfigExtAxis:
        return process_config_ext_axis(
          result,
          sources.config_ext_axis,
          sources.admission,
          maximum_payload_bytes
        );
      case JsonMainRequestParseResponseStatus::kDispatchControllerWait:
        return process_controller_wait(
          result, sources.controller_wait, sources.admission,
          maximum_payload_bytes
        );
      case JsonMainRequestParseResponseStatus::kDispatchModbusRead:
        return process_modbus_read(
          result, sources.modbus_read,
          sources.admission, maximum_payload_bytes
        );
      case JsonMainRequestParseResponseStatus::kDispatchModbusWrite:
        return process_modbus_write(
          result, sources.modbus_write,
          sources.admission, maximum_payload_bytes
        );
      case JsonMainRequestParseResponseStatus::kDispatchCalibrate:
        return process_calibration(
          result,
          sources.calibration,
          sources.admission,
          maximum_payload_bytes
        );
      case JsonMainRequestParseResponseStatus::kDispatchJogTool:
        return process_jog_tool(
          result,
          sources.jog_tool,
          sources.admission,
          maximum_payload_bytes
        );
      case JsonMainRequestParseResponseStatus::kDispatchLiveJointJog:
      case JsonMainRequestParseResponseStatus::kDispatchLiveCartJog:
      case JsonMainRequestParseResponseStatus::kDispatchLiveToolJog:
        return process_live_jog(
          result,
          sources.live_jog,
          sources.admission,
          maximum_payload_bytes
        );
      case JsonMainRequestParseResponseStatus::kDispatchMoveCartesian:
        return process_move_cartesian(
          result,
          sources.move_cartesian,
          sources.admission,
          maximum_payload_bytes
        );
      case JsonMainRequestParseResponseStatus::kDispatchMoveJoints:
        return process_move_joints(
          result,
          sources.move_joints,
          sources.admission,
          maximum_payload_bytes
        );
      case JsonMainRequestParseResponseStatus::kDispatchDirect:
        return process_direct(
          result,
          sources.direct,
          sources.admission,
          maximum_payload_bytes
        );
      case JsonMainRequestParseResponseStatus::kDispatchStop:
        return process_stop(
          result,
          sources.live_jog,
          sources.admission,
          maximum_payload_bytes
        );
      case JsonMainRequestParseResponseStatus::kDispatchRenewLiveMotion:
        return process_renew_live_motion(
          result,
          sources.live_jog,
          sources.admission,
          maximum_payload_bytes
        );
      case JsonMainRequestParseResponseStatus::kCorrelatedRejectionBuilt:
        return finish_response(
          JsonMainControllerResponseKind::kCorrelatedRejection,
          maximum_payload_bytes
        );
      case JsonMainRequestParseResponseStatus::kProtocolErrorBuilt:
        return finish_response(
          JsonMainControllerResponseKind::kProtocolError,
          maximum_payload_bytes
        );
      case JsonMainRequestParseResponseStatus::kControllerFault:
        return fault_process(JsonMainControllerFault::kParserInvariant);
      case JsonMainRequestParseResponseStatus::kSerializationFailure:
        return fault_process(
          JsonMainControllerFault::kResponseSerializationFailure
        );
    }
    return fault_process(JsonMainControllerFault::kParserInvariant);
  }

  JsonMainControllerProcessStatus stage_live_terminal(
    uint32_t motion_id,
    JsonLiveJogKind kind,
    const JsonMainLiveJogExecutionResult &result,
    size_t maximum_payload_bytes
  ) {
    switch (state_) {
      case JsonMainControllerOwnerState::kIdle:
        break;
      case JsonMainControllerOwnerState::kResponseReady:
      case JsonMainControllerOwnerState::kWriteInProgress:
        return JsonMainControllerProcessStatus::kBusy;
      case JsonMainControllerOwnerState::kFaulted:
        return JsonMainControllerProcessStatus::kSessionFaulted;
    }
    if (
      motion_id == 0
      || motion_id != active_live_motion_id_
      || kind != active_live_kind_
      || !session_bound_
      || configuration_sync_required_
      || maximum_payload_bytes == 0
      || maximum_payload_bytes > kJsonProtocolMaximumPayloadBytes
    ) {
      return fault_process(JsonMainControllerFault::kInvalidRequestArgument);
    }
    if (range_aliases_internal_storage(&result, sizeof(result))) {
      return fault_process(JsonMainControllerFault::kInternalStorageAlias);
    }
    clear_frame();
    if (!build_main_json_live_jog_response(
        kind,
        motion_id,
        result,
        maximum_payload_bytes,
        output_,
        sizeof(output_)
    )) {
      return fault_process(
        JsonMainControllerFault::kResponseSerializationFailure
      );
    }
    JsonMainControllerResponseKind response_kind =
      JsonMainControllerResponseKind::kNone;
    switch (result.outcome) {
      case JsonMainLiveJogOutcome::kCompleted:
        response_kind = JsonMainControllerResponseKind::kLiveJogCompleted;
        break;
      case JsonMainLiveJogOutcome::kEmergencyStop:
        response_kind = JsonMainControllerResponseKind::kLiveJogCancelled;
        break;
      case JsonMainLiveJogOutcome::kJointLimitReached:
      case JsonMainLiveJogOutcome::kPositionNotRepresentable:
      case JsonMainLiveJogOutcome::kKinematicsUnreachable:
      case JsonMainLiveJogOutcome::kPositionUnavailable:
      case JsonMainLiveJogOutcome::kMotionExecutionFailed:
      case JsonMainLiveJogOutcome::kEncoderCollision:
      case JsonMainLiveJogOutcome::kEncoderStateUnavailable:
      case JsonMainLiveJogOutcome::kControlLeaseExpired:
        response_kind = JsonMainControllerResponseKind::kLiveJogFailed;
        break;
      case JsonMainLiveJogOutcome::kInvalid:
        return fault_process(JsonMainControllerFault::kInvalidCommandSource);
    }
    return finish_response(response_kind, maximum_payload_bytes);
  }

  JsonMainControllerProcessStatus stage_playback_terminal(
    const JsonMainDirectCommandSource *source,
    size_t maximum_payload_bytes
  ) {
    switch (state_) {
      case JsonMainControllerOwnerState::kIdle:
        break;
      case JsonMainControllerOwnerState::kResponseReady:
      case JsonMainControllerOwnerState::kWriteInProgress:
        return JsonMainControllerProcessStatus::kBusy;
      case JsonMainControllerOwnerState::kFaulted:
        return JsonMainControllerProcessStatus::kSessionFaulted;
    }
    if (
      source == nullptr
      || source->execute == nullptr
      || range_aliases_internal_storage(source, sizeof(*source))
      || range_aliases_internal_storage(source->context, 1)
      || active_playback_request_id_ == 0
      || pending_playback_request_id_ != 0
      || playback_execution_active_
      || playback_parameters_.kind != JsonMainDirectParameterKind::kStorageFile
      || !session_bound_
      || configuration_sync_required_
      || maximum_payload_bytes
        < kJsonCartesianMotionTerminalPayloadReservationBytes
      || maximum_payload_bytes > kJsonProtocolMaximumPayloadBytes
    ) {
      return fault_process(JsonMainControllerFault::kInvalidCommandSource);
    }
    clear_frame();
    playback_execution_active_ = true;
    const JsonMainDirectResponseStatus response_status = source->execute(
      "play_gcode_file",
      playback_parameters_,
      active_playback_request_id_,
      maximum_payload_bytes,
      output_,
      sizeof(output_),
      source->context
    );
    playback_execution_active_ = false;
    JsonMainControllerResponseKind response_kind =
      JsonMainControllerResponseKind::kNone;
    switch (response_status) {
      case JsonMainDirectResponseStatus::kCompleted:
        response_kind = JsonMainControllerResponseKind::kDirectCompleted;
        break;
      case JsonMainDirectResponseStatus::kCancelled:
        response_kind = JsonMainControllerResponseKind::kDirectCancelled;
        break;
      case JsonMainDirectResponseStatus::kFailed:
        response_kind = JsonMainControllerResponseKind::kDirectFailed;
        break;
      case JsonMainDirectResponseStatus::kRejected:
      case JsonMainDirectResponseStatus::kInvalid:
        return fault_process(JsonMainControllerFault::kInvalidCommandSource);
    }
    if (output_[0] == '\0') {
      return fault_process(
        JsonMainControllerFault::kResponseSerializationFailure
      );
    }
    return finish_response(response_kind, maximum_payload_bytes);
  }

  JsonMainControllerProcessStatus process_protocol_error(
    const char *error_code,
    const char *message,
    size_t maximum_payload_bytes
  ) {
    switch (state_) {
      case JsonMainControllerOwnerState::kIdle:
        break;
      case JsonMainControllerOwnerState::kResponseReady:
      case JsonMainControllerOwnerState::kWriteInProgress:
        return JsonMainControllerProcessStatus::kBusy;
      case JsonMainControllerOwnerState::kFaulted:
        return JsonMainControllerProcessStatus::kSessionFaulted;
    }
    if (
      error_code == nullptr
      || message == nullptr
      || maximum_payload_bytes == 0
      || maximum_payload_bytes > kJsonProtocolMaximumPayloadBytes
    ) {
      return fault_process(
        JsonMainControllerFault::kInvalidRequestArgument
      );
    }
    if (
      range_aliases_internal_storage(error_code, 1)
      || range_aliases_internal_storage(message, 1)
    ) {
      return fault_process(JsonMainControllerFault::kInternalStorageAlias);
    }
    clear_frame();
    if (!build_json_protocol_error_response(
        error_code,
        message,
        nullptr,
        maximum_payload_bytes,
        output_,
        sizeof(output_)
    )) {
      return fault_process(
        JsonMainControllerFault::kResponseSerializationFailure
      );
    }
    return finish_response(
      JsonMainControllerResponseKind::kProtocolError,
      maximum_payload_bytes
    );
  }

  JsonMainControllerOutputBeginStatus begin_response_write(
    JsonMainControllerFrameView &view
  ) {
    view.data = nullptr;
    view.length = 0;
    view.kind = JsonMainControllerResponseKind::kNone;
    switch (state_) {
      case JsonMainControllerOwnerState::kIdle:
        return JsonMainControllerOutputBeginStatus::kNoResponse;
      case JsonMainControllerOwnerState::kResponseReady:
        view.data = output_;
        view.length = frame_length_;
        view.kind = response_kind_;
        state_ = JsonMainControllerOwnerState::kWriteInProgress;
        return JsonMainControllerOutputBeginStatus::kReady;
      case JsonMainControllerOwnerState::kWriteInProgress:
        return JsonMainControllerOutputBeginStatus::kBusy;
      case JsonMainControllerOwnerState::kFaulted:
        return JsonMainControllerOutputBeginStatus::kSessionFaulted;
    }
    fault_owner(JsonMainControllerFault::kInvalidOutputLifecycle);
    return JsonMainControllerOutputBeginStatus::kControllerFault;
  }

  JsonMainControllerOutputCompletionStatus complete_response_write(
    size_t written_bytes
  ) {
    if (state_ == JsonMainControllerOwnerState::kFaulted) {
      return JsonMainControllerOutputCompletionStatus::kSessionFaulted;
    }
    if (state_ != JsonMainControllerOwnerState::kWriteInProgress) {
      fault_owner(JsonMainControllerFault::kInvalidOutputLifecycle);
      return JsonMainControllerOutputCompletionStatus::kControllerFault;
    }
    if (written_bytes != frame_length_) {
      fault_owner(JsonMainControllerFault::kAmbiguousOutputWrite);
      return JsonMainControllerOutputCompletionStatus::kControllerFault;
    }
    const JsonMainControllerResponseKind completed_response_kind =
      response_kind_;
    if (
      completed_response_kind
        == JsonMainControllerResponseKind::kLiveJogAccepted
    ) {
      if (
        pending_live_motion_id_ == 0
        || pending_live_kind_ == JsonLiveJogKind::kInvalid
        || active_live_motion_id_ != 0
        || active_live_kind_ != JsonLiveJogKind::kInvalid
      ) {
        fault_owner(JsonMainControllerFault::kInvalidOutputLifecycle);
        return JsonMainControllerOutputCompletionStatus::kControllerFault;
      }
      active_live_motion_id_ = pending_live_motion_id_;
      active_live_kind_ = pending_live_kind_;
      pending_live_motion_id_ = 0;
      pending_live_kind_ = JsonLiveJogKind::kInvalid;
    } else if (
      completed_response_kind
        == JsonMainControllerResponseKind::kLiveJogCompleted
      || completed_response_kind
        == JsonMainControllerResponseKind::kLiveJogCancelled
      || completed_response_kind
        == JsonMainControllerResponseKind::kLiveJogFailed
    ) {
      if (
        active_live_motion_id_ == 0
        || active_live_kind_ == JsonLiveJogKind::kInvalid
        || pending_live_motion_id_ != 0
        || pending_live_kind_ != JsonLiveJogKind::kInvalid
      ) {
        fault_owner(JsonMainControllerFault::kInvalidOutputLifecycle);
        return JsonMainControllerOutputCompletionStatus::kControllerFault;
      }
      active_live_motion_id_ = 0;
      active_live_kind_ = JsonLiveJogKind::kInvalid;
    } else if (
      completed_response_kind
        == JsonMainControllerResponseKind::kDirectAccepted
    ) {
      if (
        pending_playback_request_id_ == 0
        || active_playback_request_id_ != 0
        || active_live_motion_id_ != 0
        || active_live_kind_ != JsonLiveJogKind::kInvalid
      ) {
        fault_owner(JsonMainControllerFault::kInvalidOutputLifecycle);
        return JsonMainControllerOutputCompletionStatus::kControllerFault;
      }
      active_playback_request_id_ = pending_playback_request_id_;
      pending_playback_request_id_ = 0;
    } else if (
      active_playback_request_id_ != 0
      && (
        completed_response_kind
          == JsonMainControllerResponseKind::kDirectCompleted
        || completed_response_kind
          == JsonMainControllerResponseKind::kDirectCancelled
        || completed_response_kind
          == JsonMainControllerResponseKind::kDirectFailed
      )
    ) {
      if (pending_playback_request_id_ != 0 || playback_execution_active_) {
        fault_owner(JsonMainControllerFault::kInvalidOutputLifecycle);
        return JsonMainControllerOutputCompletionStatus::kControllerFault;
      }
      active_playback_request_id_ = 0;
      playback_parameters_ = {};
    }
    clear_frame();
    state_ = JsonMainControllerOwnerState::kIdle;
    fault_ = JsonMainControllerFault::kNone;
    if (
      completed_response_kind
        == JsonMainControllerResponseKind::kHelloCompleted
    ) {
      session_bound_ = true;
    }
    return JsonMainControllerOutputCompletionStatus::kCompleted;
  }

  JsonMainControllerRecoveryStatus recover_after_quiescence(
    size_t semantic_allocation_limit = kJsonRequestSemanticStorageBytes
  ) {
    if (
      state_ == JsonMainControllerOwnerState::kResponseReady
      || state_ == JsonMainControllerOwnerState::kWriteInProgress
    ) {
      return JsonMainControllerRecoveryStatus::kBusy;
    }
    const bool cleared_owner_fault =
      state_ == JsonMainControllerOwnerState::kFaulted
      || fault_ != JsonMainControllerFault::kNone;
    const JsonMainRequestSemanticAllocator::RecoveryStatus parser_status =
      parser_workspace_.semantic_allocator.recover_after_quiescence(
        semantic_allocation_limit
      );
    if (
      parser_status
        == JsonMainRequestSemanticAllocator::RecoveryStatus::kInvalidLimit
    ) {
      fault_owner(JsonMainControllerFault::kInvalidRecoveryLimit);
      return JsonMainControllerRecoveryStatus::kInvalidLimit;
    }

    semantic_allocation_limit_ = semantic_allocation_limit;
    clear_frame();
    state_ = JsonMainControllerOwnerState::kIdle;
    fault_ = JsonMainControllerFault::kNone;
    // Recovery begins a new logical session; framing release again requires
    // an exactly written hello response from the recovered owner.
    session_bound_ = false;
    pending_live_motion_id_ = 0;
    pending_live_kind_ = JsonLiveJogKind::kInvalid;
    active_live_motion_id_ = 0;
    active_live_kind_ = JsonLiveJogKind::kInvalid;
    pending_playback_request_id_ = 0;
    active_playback_request_id_ = 0;
    playback_execution_active_ = false;
    playback_parameters_ = {};
    if (
      parser_status
        == JsonMainRequestSemanticAllocator::RecoveryStatus::
          kDiscardedAllocations
    ) {
      return JsonMainControllerRecoveryStatus::
        kDiscardedParserAllocations;
    }
    if (
      cleared_owner_fault
      || parser_status
        == JsonMainRequestSemanticAllocator::RecoveryStatus::kClearedFault
    ) {
      return JsonMainControllerRecoveryStatus::kClearedFault;
    }
    return JsonMainControllerRecoveryStatus::kReady;
  }

  JsonMainControllerOwnerState state() const {
    return state_;
  }

  JsonMainControllerFault fault() const {
    return fault_;
  }

  JsonMainControllerResponseKind response_kind() const {
    return response_kind_;
  }

  bool session_bound() const {
    return session_bound_;
  }

  bool configuration_sync_required() const {
    return configuration_sync_required_;
  }

  uint32_t active_live_motion_id() const {
    return active_live_motion_id_;
  }

  JsonLiveJogKind active_live_kind() const {
    return active_live_kind_;
  }

  uint32_t active_playback_request_id() const {
    return active_playback_request_id_;
  }

  bool playback_execution_active() const {
    return playback_execution_active_;
  }

  void mark_configuration_sync_required() {
    configuration_sync_required_ = true;
  }

  void confirm_configuration_sync() {
    configuration_sync_required_ = false;
  }

 private:
  static bool response_status_dispatches(
    JsonMainRequestParseResponseStatus status
  ) {
    switch (status) {
      case JsonMainRequestParseResponseStatus::kDispatchHello:
      case JsonMainRequestParseResponseStatus::kDispatchGetHomeReference:
      case JsonMainRequestParseResponseStatus::kDispatchGetPositionDisposition:
      case JsonMainRequestParseResponseStatus::kDispatchCorrectPosition:
      case JsonMainRequestParseResponseStatus::kDispatchZeroExternalAxis:
      case JsonMainRequestParseResponseStatus::kDispatchTestLimitSwitches:
      case JsonMainRequestParseResponseStatus::kDispatchSetEncoders:
      case JsonMainRequestParseResponseStatus::kDispatchReadEncoders:
      case JsonMainRequestParseResponseStatus::kDispatchGetMotionTrace:
      case JsonMainRequestParseResponseStatus::kDispatchSetPosition:
      case JsonMainRequestParseResponseStatus::kDispatchUpdateParams:
      case JsonMainRequestParseResponseStatus::kDispatchConfigExtAxis:
      case JsonMainRequestParseResponseStatus::kDispatchControllerWait:
      case JsonMainRequestParseResponseStatus::kDispatchModbusRead:
      case JsonMainRequestParseResponseStatus::kDispatchModbusWrite:
      case JsonMainRequestParseResponseStatus::kDispatchCalibrate:
      case JsonMainRequestParseResponseStatus::kDispatchJogTool:
      case JsonMainRequestParseResponseStatus::kDispatchLiveJointJog:
      case JsonMainRequestParseResponseStatus::kDispatchLiveCartJog:
      case JsonMainRequestParseResponseStatus::kDispatchLiveToolJog:
      case JsonMainRequestParseResponseStatus::kDispatchMoveCartesian:
      case JsonMainRequestParseResponseStatus::kDispatchMoveJoints:
      case JsonMainRequestParseResponseStatus::kDispatchStop:
      case JsonMainRequestParseResponseStatus::kDispatchRenewLiveMotion:
      case JsonMainRequestParseResponseStatus::kDispatchDirect:
        return true;
      case JsonMainRequestParseResponseStatus::kCorrelatedRejectionBuilt:
      case JsonMainRequestParseResponseStatus::kProtocolErrorBuilt:
      case JsonMainRequestParseResponseStatus::kControllerFault:
      case JsonMainRequestParseResponseStatus::kSerializationFailure:
        return false;
    }
    return false;
  }

  bool range_aliases_internal_storage(
    const void *address,
    size_t length
  ) const {
    if (address == nullptr || length == 0) return false;
    const uintptr_t maximum_address = static_cast<uintptr_t>(-1);
    const uintptr_t range_begin = reinterpret_cast<uintptr_t>(address);
    const uintptr_t owner_begin = reinterpret_cast<uintptr_t>(this);
    if (
      length > maximum_address - range_begin
      || sizeof(*this) > maximum_address - owner_begin
    ) {
      return true;
    }
    const uintptr_t range_end = range_begin + length;
    const uintptr_t owner_end = owner_begin + sizeof(*this);
    return range_begin < owner_end && owner_begin < range_end;
  }

  bool command_sources_alias_internal_storage(
    const JsonMainControllerDispatchSources &sources
  ) const {
    if (range_aliases_internal_storage(
        sources.hello,
        sizeof(JsonMainHelloResponseSource)
      )
      || range_aliases_internal_storage(
        sources.home_reference,
        sizeof(JsonMainHomeReferenceResponseSource)
      )
      || range_aliases_internal_storage(
        sources.position,
        sizeof(JsonMainPositionResponseSource)
      )
      || range_aliases_internal_storage(
        sources.set_position,
        sizeof(JsonMainSetPositionCommandSource)
      )
      || range_aliases_internal_storage(
        sources.update_params,
        sizeof(JsonMainUpdateParametersCommandSource)
      )
      || range_aliases_internal_storage(
        sources.config_ext_axis,
        sizeof(JsonMainExternalAxisCommandSource)
      )
      || range_aliases_internal_storage(
        sources.move_cartesian,
        sizeof(JsonMainMoveCartesianCommandSource)
      )
      || range_aliases_internal_storage(
        sources.move_joints,
        sizeof(JsonMainMoveJointsCommandSource)
      )
      || range_aliases_internal_storage(
        sources.jog_tool,
        sizeof(JsonMainToolJogCommandSource)
      )
      || range_aliases_internal_storage(
        sources.live_jog,
        sizeof(JsonMainLiveJogCommandSource)
      )
      || range_aliases_internal_storage(
        sources.diagnostics,
        sizeof(JsonMainDiagnosticCommandSource)
      )
      || range_aliases_internal_storage(
        sources.motion_trace,
        sizeof(JsonMainMotionTraceResponseSource)
      )
      || range_aliases_internal_storage(
        sources.calibration,
        sizeof(JsonMainCalibrationCommandSource)
      )
      || range_aliases_internal_storage(
        sources.correct_position,
        sizeof(JsonMainCorrectPositionCommandSource)
      )
      || range_aliases_internal_storage(
        sources.zero_external_axis,
        sizeof(JsonMainExternalAxisZeroCommandSource)
      )
      || range_aliases_internal_storage(
        sources.controller_wait,
        sizeof(JsonMainControllerWaitCommandSource)
      )
      || range_aliases_internal_storage(
        sources.modbus_read, sizeof(JsonMainModbusReadCommandSource))
      || range_aliases_internal_storage(
        sources.modbus_write, sizeof(JsonMainModbusWriteCommandSource))
      || range_aliases_internal_storage(
        sources.direct, sizeof(JsonMainDirectCommandSource))
      || (
        sources.set_position != nullptr
        && range_aliases_internal_storage(
          sources.set_position->context,
          1
        )
      )
      || (
        sources.update_params != nullptr
        && range_aliases_internal_storage(
          sources.update_params->context,
          1
        )
      )
      || (
        sources.config_ext_axis != nullptr
        && range_aliases_internal_storage(
          sources.config_ext_axis->context,
          1
        )
      )
      || (
        sources.move_cartesian != nullptr
        && range_aliases_internal_storage(
          sources.move_cartesian->context,
          1
        )
      )
      || (
        sources.move_joints != nullptr
        && range_aliases_internal_storage(
          sources.move_joints->context,
          1
        )
      )
      || (
        sources.jog_tool != nullptr
        && range_aliases_internal_storage(
          sources.jog_tool->context,
          1
        )
      )
      || (
        sources.live_jog != nullptr
        && range_aliases_internal_storage(
          sources.live_jog->context,
          1
        )
      )
      || (
        sources.diagnostics != nullptr
        && range_aliases_internal_storage(
          sources.diagnostics->context,
          1
        )
      )
      || (
        sources.motion_trace != nullptr
        && range_aliases_internal_storage(
          sources.motion_trace->capture,
          1
        )
      )
      || (
        sources.calibration != nullptr
        && range_aliases_internal_storage(
          sources.calibration->context,
          1
        )
      )
      || (
        sources.correct_position != nullptr
        && range_aliases_internal_storage(
          sources.correct_position->context,
          1
        )
      )
      || (
        sources.zero_external_axis != nullptr
        && range_aliases_internal_storage(
          sources.zero_external_axis->context,
          1
        )
      )
      || (
        sources.controller_wait != nullptr
        && range_aliases_internal_storage(
          sources.controller_wait->context,
          1
        )
      )
      || (
        sources.modbus_read != nullptr
        && range_aliases_internal_storage(sources.modbus_read->context, 1))
      || (
        sources.modbus_write != nullptr
        && range_aliases_internal_storage(sources.modbus_write->context, 1))
      || (
        sources.direct != nullptr
        && range_aliases_internal_storage(sources.direct->context, 1))
    ) {
      return true;
    }
    return sources.hello != nullptr
      && sources.hello->status == JsonMainHelloSourceStatus::kAvailable
      && hello_source_aliases_internal_storage(*sources.hello);
  }

  bool hello_source_aliases_internal_storage(
    const JsonMainHelloResponseSource &source
  ) const {
    const struct {
      const void *address;
      size_t length;
    } fields[] = {
      {source.session_id, kJsonSessionIdentifierLength + 1},
      {source.firmware_name, kIdentityFieldCapacity},
      {source.firmware_version, kIdentityFieldCapacity},
      {source.firmware_build, kIdentityFieldCapacity},
      {source.controller_hardware_id, kControllerHardwareIdCapacity},
      {source.driver_model, kIdentityFieldCapacity},
      {source.robot_model, kIdentityFieldCapacity},
      {source.robot_version, kIdentityFieldCapacity},
      {source.serial_number, kIdentityFieldCapacity},
      {source.asset_tag, kIdentityFieldCapacity},
    };
    for (const auto &field : fields) {
      if (range_aliases_internal_storage(field.address, field.length)) {
        return true;
      }
    }
    if (source.capabilities != nullptr) {
      if (
        source.capability_count
          > static_cast<size_t>(-1) / sizeof(*source.capabilities)
        || range_aliases_internal_storage(
          source.capabilities,
          source.capability_count * sizeof(*source.capabilities)
        )
      ) {
        return true;
      }
    }
    if (source.commands != nullptr) {
      if (
        source.command_count > 64
        || range_aliases_internal_storage(
          source.commands,
          source.command_count * sizeof(*source.commands)
        )
      ) return true;
      for (size_t index = 0; index < source.command_count; ++index) {
        if (range_aliases_internal_storage(
            source.commands[index], kJsonProtocolMaximumNameLength + 1
        )) return true;
      }
    }
    if (
      source.capabilities == nullptr
      || source.capability_count > kProtocolCapabilityMaximumCount
    ) {
      return false;
    }
    for (size_t index = 0; index < source.capability_count; ++index) {
      if (range_aliases_internal_storage(
          source.capabilities[index],
          kProtocolCapabilityMaximumLength + 1
      )) {
        return true;
      }
    }
    return false;
  }

  static bool hello_source_valid(
    const JsonMainHelloResponseSource &source
  ) {
    return json_session_identifier_valid(source.session_id)
      && identity_field_valid(source.firmware_name)
      && identity_field_valid(source.firmware_version)
      && identity_field_valid(source.firmware_build)
      && controller_hardware_id_valid(source.controller_hardware_id)
      && identity_field_valid(source.driver_model)
      && identity_field_valid(source.robot_model)
      && identity_field_valid(source.robot_version)
      && identity_field_valid(source.serial_number)
      && identity_field_valid(source.asset_tag)
      && json_hello_capabilities_valid(
        source.capabilities,
        source.capability_count
      ) && json_command_manifest_valid(source.commands, source.command_count);
  }

  bool build_diagnostic_response(
    const JsonMainRequestParseResult &result,
    const bool *active,
    const int32_t *counts,
    size_t maximum_payload_bytes
  ) {
    const size_t capacity = sizeof(output_) < maximum_payload_bytes + 1
      ? sizeof(output_) : maximum_payload_bytes + 1;
    size_t index = 0;
    output_[0] = '\0';
    const JsonMainRequestCommand command_kind =
      result.payload.command_kind();
    bool built = append_json_text("{\"cmd\":\"", output_, capacity, index)
      && append_json_text(result.command, output_, capacity, index)
      && append_json_text("\",\"id\":", output_, capacity, index)
      && append_json_uint32(result.request_id, output_, capacity, index)
      && append_json_text(",\"result\":{", output_, capacity, index);
    if (command_kind == JsonMainRequestCommand::kTestLimitSwitches) {
      built = built
        && append_json_text("\"active\":", output_, capacity, index)
        && append_json_bool_array(active, 6, output_, capacity, index);
    } else if (command_kind == JsonMainRequestCommand::kReadEncoders) {
      built = built
        && append_json_text("\"counts\":", output_, capacity, index)
        && append_json_int32_array(counts, 6, output_, capacity, index);
    } else if (command_kind != JsonMainRequestCommand::kSetEncoders) {
      built = false;
    }
    built = built && append_json_text(
      "},\"status\":\"completed\",\"type\":\"response\",\"v\":1}",
      output_, capacity, index
    );
    if (!built) output_[0] = '\0';
    return built;
  }

  bool build_modbus_read_response(
    const JsonMainRequestParseResult &result, int32_t value,
    size_t maximum_payload_bytes) {
    const size_t capacity = sizeof(output_) < maximum_payload_bytes + 1
      ? sizeof(output_) : maximum_payload_bytes + 1;
    size_t index = 0;
    output_[0] = '\0';
    const bool built = append_json_text("{\"cmd\":\"", output_, capacity, index)
      && append_json_text(result.command, output_, capacity, index)
      && append_json_text("\",\"id\":", output_, capacity, index)
      && append_json_uint32(result.request_id, output_, capacity, index)
      && append_json_text(",\"result\":{\"value\":", output_, capacity, index)
      && append_json_int32(value, output_, capacity, index)
      && append_json_text(
        "},\"status\":\"completed\",\"type\":\"response\",\"v\":1}",
        output_, capacity, index);
    if (!built) output_[0] = '\0';
    return built;
  }

  static bool modbus_read_value_valid(ModbusOperation operation, int32_t value) {
    if (
      operation == ModbusOperation::kReadCoil
      || operation == ModbusOperation::kReadDiscreteInput
    ) return value == 0 || value == 1;
    const bool register_read =
      operation == ModbusOperation::kReadHoldingRegisters
      || operation == ModbusOperation::kReadInputRegisters;
    return register_read && value >= 0 && value <= kModbusMaximumRegisterValue;
  }

  bool build_failed_response(
    uint32_t request_id,
    const char *command,
    const char *error_code,
    const char *message,
    size_t maximum_payload_bytes
  ) {
    return build_main_json_error_response(
      request_id,
      command,
      JsonErrorResponseStatus::kFailed,
      error_code,
      message,
      nullptr,
      maximum_payload_bytes,
      output_,
      sizeof(output_)
    );
  }

  bool build_rejected_response(
    uint32_t request_id,
    const char *command,
    const char *error_code,
    const char *message,
    size_t maximum_payload_bytes
  ) {
    return build_main_json_error_response(
      request_id,
      command,
      JsonErrorResponseStatus::kRejected,
      error_code,
      message,
      nullptr,
      maximum_payload_bytes,
      output_,
      sizeof(output_)
    );
  }

  bool process_admission(
    const JsonMainRequestParseResult &result,
    JsonMainControllerAdmissionStatus admission,
    size_t maximum_payload_bytes,
    JsonMainControllerProcessStatus &status
  ) {
    switch (admission) {
      case JsonMainControllerAdmissionStatus::kAvailable:
        status = JsonMainControllerProcessStatus::kResponseReady;
        return true;
      case JsonMainControllerAdmissionStatus::kEmergencyStopActive:
        if (!build_rejected_response(
            result.request_id,
            result.command,
            "emergency_stop_active",
            "physical emergency stop blocks request admission",
            maximum_payload_bytes
        )) {
          status = fault_process(
            JsonMainControllerFault::kResponseSerializationFailure
          );
          return false;
        }
        status = finish_response(
          JsonMainControllerResponseKind::kAdmissionRejected,
          maximum_payload_bytes
        );
        return false;
    }
    status = fault_process(JsonMainControllerFault::kInvalidCommandSource);
    return false;
  }

  JsonMainControllerProcessStatus process_live_motion_exclusion(
    const JsonMainRequestParseResult &result,
    JsonMainControllerAdmissionStatus admission,
    size_t maximum_payload_bytes
  ) {
    JsonMainControllerProcessStatus admission_status =
      JsonMainControllerProcessStatus::kControllerFault;
    if (!process_admission(
        result,
        admission,
        maximum_payload_bytes,
        admission_status
    )) {
      return admission_status;
    }
    if (!build_rejected_response(
        result.request_id,
        result.command,
        "live_motion_active",
        "active live motion excludes unrelated requests",
        maximum_payload_bytes
    )) {
      return fault_process(
        JsonMainControllerFault::kResponseSerializationFailure
      );
    }
    return finish_response(
      JsonMainControllerResponseKind::kCorrelatedRejection,
      maximum_payload_bytes
    );
  }

  JsonMainControllerProcessStatus process_late_estop_admission(
    const JsonMainRequestParseResult &result,
    size_t maximum_payload_bytes
  ) {
    clear_frame();
    JsonMainControllerProcessStatus status =
      JsonMainControllerProcessStatus::kControllerFault;
    if (process_admission(
        result,
        JsonMainControllerAdmissionStatus::kEmergencyStopActive,
        maximum_payload_bytes,
        status
    )) {
      return fault_process(JsonMainControllerFault::kInvalidCommandSource);
    }
    return status;
  }

  JsonMainControllerProcessStatus process_hello(
    const JsonMainRequestParseResult &result,
    const JsonMainHelloResponseSource *source,
    JsonMainControllerAdmissionStatus admission,
    size_t maximum_payload_bytes
  ) {
    JsonMainControllerProcessStatus admission_status =
      JsonMainControllerProcessStatus::kControllerFault;
    if (!process_admission(
        result,
        admission,
        maximum_payload_bytes,
        admission_status
    )) {
      return admission_status;
    }
    if (source == nullptr) {
      return fault_process(JsonMainControllerFault::kInvalidCommandSource);
    }
    if (range_aliases_internal_storage(source, sizeof(*source))) {
      return fault_process(JsonMainControllerFault::kInternalStorageAlias);
    }
    switch (source->status) {
      case JsonMainHelloSourceStatus::kIdentityUnavailable:
        if (!build_failed_response(
            result.request_id,
            result.command,
            "identity_unavailable",
            "controller identity is unavailable",
            maximum_payload_bytes
        )) {
          return fault_process(
            JsonMainControllerFault::kResponseSerializationFailure
          );
        }
        return finish_response(
          JsonMainControllerResponseKind::kHelloFailed,
          maximum_payload_bytes
        );
      case JsonMainHelloSourceStatus::kSessionUnavailable:
        if (!build_failed_response(
            result.request_id,
            result.command,
            "session_unavailable",
            "controller session identity is unavailable",
            maximum_payload_bytes
        )) {
          return fault_process(
            JsonMainControllerFault::kResponseSerializationFailure
          );
        }
        return finish_response(
          JsonMainControllerResponseKind::kHelloFailed,
          maximum_payload_bytes
        );
      case JsonMainHelloSourceStatus::kAvailable:
        break;
    }
    if (source->status != JsonMainHelloSourceStatus::kAvailable) {
      return fault_process(JsonMainControllerFault::kInvalidCommandSource);
    }
    if (hello_source_aliases_internal_storage(*source)) {
      return fault_process(JsonMainControllerFault::kInternalStorageAlias);
    }
    if (!hello_source_valid(*source)) {
      return fault_process(JsonMainControllerFault::kInvalidCommandSource);
    }
    if (!build_main_json_hello_response(
        result.request_id,
        source->session_id,
        source->firmware_name,
        source->firmware_version,
        source->firmware_build,
        source->controller_hardware_id,
        source->driver_model,
        source->robot_model,
        source->robot_version,
        source->serial_number,
        source->asset_tag,
        source->capabilities,
        source->capability_count,
        source->commands,
        source->command_count,
        maximum_payload_bytes,
        output_,
        sizeof(output_)
    )) {
      return fault_process(
        JsonMainControllerFault::kResponseSerializationFailure
      );
    }
    return finish_response(
      JsonMainControllerResponseKind::kHelloCompleted,
      maximum_payload_bytes
    );
  }

  JsonMainControllerProcessStatus process_position(
    const JsonMainRequestParseResult &result,
    const JsonMainPositionResponseSource *source,
    JsonMainControllerAdmissionStatus admission,
    size_t maximum_payload_bytes
  ) {
    JsonMainControllerProcessStatus admission_status =
      JsonMainControllerProcessStatus::kControllerFault;
    if (!process_admission(
        result,
        admission,
        maximum_payload_bytes,
        admission_status
    )) {
      return admission_status;
    }
    if (!session_bound_) {
      if (!build_rejected_response(
          result.request_id,
          result.command,
          "session_not_established",
          "hello must complete before position disposition",
          maximum_payload_bytes
      )) {
        return fault_process(
          JsonMainControllerFault::kResponseSerializationFailure
        );
      }
      return finish_response(
        JsonMainControllerResponseKind::kCorrelatedRejection,
        maximum_payload_bytes
      );
    }
    if (configuration_sync_required_) {
      if (!build_failed_response(
          result.request_id,
          result.command,
          "position_unavailable",
          "controller position requires configuration synchronization",
          maximum_payload_bytes
      )) {
        return fault_process(
          JsonMainControllerFault::kResponseSerializationFailure
        );
      }
      return finish_response(
        JsonMainControllerResponseKind::kPositionFailed,
        maximum_payload_bytes
      );
    }
    if (source == nullptr) {
      return fault_process(JsonMainControllerFault::kInvalidCommandSource);
    }
    if (range_aliases_internal_storage(source, sizeof(*source))) {
      return fault_process(JsonMainControllerFault::kInternalStorageAlias);
    }
    switch (source->status) {
      case JsonMainPositionSourceStatus::kPositionUnavailable:
        if (!build_failed_response(
            result.request_id,
            result.command,
            "position_unavailable",
            "controller position is unavailable",
            maximum_payload_bytes
        )) {
          return fault_process(
            JsonMainControllerFault::kResponseSerializationFailure
          );
        }
        return finish_response(
          JsonMainControllerResponseKind::kPositionFailed,
          maximum_payload_bytes
        );
      case JsonMainPositionSourceStatus::kControllerAlarm: {
        if (!json_position_controller_alarm_valid(
            source->controller_alarm
        )) {
          return fault_process(JsonMainControllerFault::kInvalidCommandSource);
        }
        const JsonStringErrorDetail detail = {
          "controller_alarm",
          source->controller_alarm,
        };
        if (!build_main_json_error_response(
            result.request_id,
            result.command,
            JsonErrorResponseStatus::kFailed,
            "controller_alarm",
            "controller position is blocked by a motion alarm",
            &detail,
            maximum_payload_bytes,
            output_,
            sizeof(output_)
        )) {
          return fault_process(
            JsonMainControllerFault::kResponseSerializationFailure
          );
        }
        return finish_response(
          JsonMainControllerResponseKind::kPositionAlarmFailed,
          maximum_payload_bytes
        );
      }
      case JsonMainPositionSourceStatus::kDispositionUnavailable:
        if (!build_failed_response(
            result.request_id,
            result.command,
            "position_unavailable",
            "controller position disposition is unavailable",
            maximum_payload_bytes
        )) {
          return fault_process(
            JsonMainControllerFault::kResponseSerializationFailure
          );
        }
        return finish_response(
          JsonMainControllerResponseKind::kPositionFailed,
          maximum_payload_bytes
        );
      case JsonMainPositionSourceStatus::kAvailable:
        break;
    }
    if (source->status != JsonMainPositionSourceStatus::kAvailable) {
      return fault_process(JsonMainControllerFault::kInvalidCommandSource);
    }
    return finish_position_response(
      result,
      source,
      maximum_payload_bytes
    );
  }

  JsonMainControllerProcessStatus finish_position_response(
    const JsonMainRequestParseResult &result,
    const JsonMainPositionResponseSource *source,
    size_t maximum_payload_bytes
  ) {
    const bool built = build_main_json_position_disposition_response(
      result.request_id,
      source->snapshot,
      source->speed_limited,
      source->controller_debug,
      source->motion_fault,
      maximum_payload_bytes,
      output_,
      sizeof(output_)
    );
    if (!built) {
      return fault_process(
        JsonMainControllerFault::kResponseSerializationFailure
      );
    }
    return finish_response(
      JsonMainControllerResponseKind::kPositionDispositionCompleted,
      maximum_payload_bytes
    );
  }

  bool build_correct_position_encoder_failure_response(
    const JsonMainRequestParseResult &request,
    const JsonMainCorrectPositionResult &result,
    size_t maximum_payload_bytes
  ) {
    if (
      maximum_payload_bytes == 0
      || maximum_payload_bytes > kJsonProtocolMaximumPayloadBytes
    ) return false;
    const size_t bounded_capacity =
      sizeof(output_) < maximum_payload_bytes + 1
      ? sizeof(output_)
      : maximum_payload_bytes + 1;
    size_t index = 0;
    const bool built = append_json_text(
        "{\"cmd\":\"correct_position\",\"error\":{\"code\":"
        "\"encoder_state_unavailable\",\"details\":{\"axes\":",
        output_,
        bounded_capacity,
        index
      )
      && append_json_bool_array(
        result.axes,
        6,
        output_,
        bounded_capacity,
        index
      )
      && append_json_text(
        ",\"position\":",
        output_,
        bounded_capacity,
        index
      )
      && append_main_json_position_snapshot(
        result.position.snapshot,
        output_,
        bounded_capacity,
        index
      )
      && append_json_text(
        "},\"message\":\"encoder state is unavailable\"},\"id\":",
        output_,
        bounded_capacity,
        index
      )
      && append_json_uint32(
        request.request_id,
        output_,
        bounded_capacity,
        index
      )
      && append_json_text(
        ",\"status\":\"failed\",\"type\":\"response\",\"v\":",
        output_,
        bounded_capacity,
        index
      )
      && append_json_uint32(
        kJsonProtocolVersion,
        output_,
        bounded_capacity,
        index
      )
      && append_json_text("}", output_, bounded_capacity, index);
    if (!built) output_[0] = '\0';
    return built;
  }

  JsonMainControllerProcessStatus process_correct_position(
    const JsonMainRequestParseResult &request,
    const JsonMainCorrectPositionCommandSource *source,
    JsonMainControllerAdmissionStatus admission,
    size_t maximum_payload_bytes
  ) {
    JsonMainControllerProcessStatus admission_status =
      JsonMainControllerProcessStatus::kControllerFault;
    if (!process_admission(
        request,
        admission,
        maximum_payload_bytes,
        admission_status
    )) {
      return admission_status;
    }
    if (!session_bound_) {
      if (!build_rejected_response(
          request.request_id,
          request.command,
          "session_not_established",
          "hello must complete before position correction",
          maximum_payload_bytes
      )) {
        return fault_process(
          JsonMainControllerFault::kResponseSerializationFailure
        );
      }
      return finish_response(
        JsonMainControllerResponseKind::kCorrectPositionFailed,
        maximum_payload_bytes
      );
    }
    if (configuration_sync_required_) {
      if (!build_rejected_response(
          request.request_id,
          request.command,
          "configuration_sync_required",
          "set_position must complete before position correction",
          maximum_payload_bytes
      )) {
        return fault_process(
          JsonMainControllerFault::kResponseSerializationFailure
        );
      }
      return finish_response(
        JsonMainControllerResponseKind::kCorrectPositionFailed,
        maximum_payload_bytes
      );
    }
    if (
      source == nullptr
      || source->prepare == nullptr
      || source->apply == nullptr
      || range_aliases_internal_storage(source, sizeof(*source))
      || range_aliases_internal_storage(source->context, 1)
    ) {
      return fault_process(JsonMainControllerFault::kInvalidCommandSource);
    }

    JsonMainCorrectPositionResult result = {};
    if (!source->prepare(result, source->context)) {
      return fault_process(JsonMainControllerFault::kInvalidCommandSource);
    }
    bool affected_axis_present = false;
    for (bool affected : result.axes) {
      affected_axis_present = affected_axis_present || affected;
    }
    switch (result.outcome) {
      case JsonMainCorrectPositionOutcome::kPositionUnavailable:
        if (
          affected_axis_present
          || result.position.status
            != JsonMainPositionSourceStatus::kPositionUnavailable
        ) {
          return fault_process(JsonMainControllerFault::kInvalidCommandSource);
        }
        if (!build_failed_response(
            request.request_id,
            request.command,
            "position_unavailable",
            "controller position correction is unavailable",
            maximum_payload_bytes
        )) {
          return fault_process(
            JsonMainControllerFault::kResponseSerializationFailure
          );
        }
        return finish_response(
          JsonMainControllerResponseKind::kCorrectPositionFailed,
          maximum_payload_bytes
        );
      case JsonMainCorrectPositionOutcome::kEncoderStateUnavailable:
        if (
          !affected_axis_present
          || result.position.status
            != JsonMainPositionSourceStatus::kAvailable
        ) {
          return fault_process(JsonMainControllerFault::kInvalidCommandSource);
        }
        if (!build_correct_position_encoder_failure_response(
            request,
            result,
            maximum_payload_bytes
        )) {
          return fault_process(
            JsonMainControllerFault::kResponseSerializationFailure
          );
        }
        return finish_response(
          JsonMainControllerResponseKind::kCorrectPositionEncoderFailed,
          maximum_payload_bytes
        );
      case JsonMainCorrectPositionOutcome::kCompleted:
        if (
          affected_axis_present
          || result.position.status
            != JsonMainPositionSourceStatus::kAvailable
          || !json_position_controller_debug_valid(
            result.position.controller_debug
          )
          || !json_position_motion_fault_valid(
            result.position.motion_fault
          )
        ) {
          return fault_process(JsonMainControllerFault::kInvalidCommandSource);
        }
        if (!build_main_json_correct_position_response(
            request.request_id,
            result.position.snapshot,
            result.position.speed_limited,
            result.position.controller_debug,
            result.position.motion_fault,
            maximum_payload_bytes,
            output_,
            sizeof(output_)
        )) {
          return fault_process(
            JsonMainControllerFault::kResponseSerializationFailure
          );
        }
        break;
      default:
        return fault_process(JsonMainControllerFault::kInvalidCommandSource);
    }

    const JsonMainCorrectPositionApplyStatus apply_status =
      source->apply(source->context);
    if (apply_status == JsonMainCorrectPositionApplyStatus::kApplied) {
      return finish_response(
        JsonMainControllerResponseKind::kCorrectPositionCompleted,
        maximum_payload_bytes
      );
    }
    if (
      apply_status
        == JsonMainCorrectPositionApplyStatus::kEmergencyStopActive
    ) {
      return process_late_estop_admission(request, maximum_payload_bytes);
    }
    return fault_process(JsonMainControllerFault::kInvalidCommandSource);
  }

  JsonMainControllerProcessStatus process_external_axis_zero(
    const JsonMainRequestParseResult &request,
    const JsonMainExternalAxisZeroCommandSource *source,
    JsonMainControllerAdmissionStatus admission,
    size_t maximum_payload_bytes
  ) {
    JsonMainControllerProcessStatus admission_status =
      JsonMainControllerProcessStatus::kControllerFault;
    if (!process_admission(
        request, admission, maximum_payload_bytes, admission_status
    )) return admission_status;
    if (!session_bound_ || configuration_sync_required_) {
      const bool session_missing = !session_bound_;
      if (!build_rejected_response(
          request.request_id,
          request.command,
          session_missing ? "session_not_established" : "configuration_sync_required",
          session_missing
            ? "hello must complete before external-axis zero"
            : "set_position must complete before external-axis zero",
          maximum_payload_bytes
      )) return fault_process(
        JsonMainControllerFault::kResponseSerializationFailure
      );
      return finish_response(
        JsonMainControllerResponseKind::kExternalAxisZeroFailed,
        maximum_payload_bytes
      );
    }
    if (source == nullptr) {
      if (!build_rejected_response(
          request.request_id,
          request.command,
          "unsupported_command",
          "request command is unsupported",
          maximum_payload_bytes
      )) return fault_process(
        JsonMainControllerFault::kResponseSerializationFailure
      );
      return finish_response(
        JsonMainControllerResponseKind::kCorrelatedRejection,
        maximum_payload_bytes
      );
    }
    if (source->stage_post_zero_position == nullptr
        || source->apply == nullptr) {
      return fault_process(JsonMainControllerFault::kInvalidCommandSource);
    }
    const JsonMainRequestCommand command = request.payload.command_kind();
    const uint8_t axis = command == JsonMainRequestCommand::kZeroJ7 ? 7
      : command == JsonMainRequestCommand::kZeroJ8 ? 8
      : command == JsonMainRequestCommand::kZeroJ9 ? 9 : 0;
    if (axis == 0) {
      return fault_process(JsonMainControllerFault::kInvalidCommandSource);
    }
    JsonMainPositionResponseSource post_zero_position = {};
    if (!source->stage_post_zero_position(
          axis, post_zero_position, source->context
        )
        || post_zero_position.status
          != JsonMainPositionSourceStatus::kAvailable) {
      if (!build_failed_response(
          request.request_id,
          request.command,
          "position_unavailable",
          "authoritative position is unavailable for external-axis zero",
          maximum_payload_bytes
      )) return fault_process(
        JsonMainControllerFault::kResponseSerializationFailure
      );
      return finish_response(
        JsonMainControllerResponseKind::kExternalAxisZeroFailed,
        maximum_payload_bytes
      );
    }
    if (!json_position_controller_debug_valid(
          post_zero_position.controller_debug
        )
        || !json_position_motion_fault_valid(
          post_zero_position.motion_fault
        )) {
      return fault_process(JsonMainControllerFault::kInvalidCommandSource);
    }
    if (!build_main_json_external_axis_zero_response(
        request.request_id,
        request.command,
        post_zero_position.snapshot,
        post_zero_position.speed_limited,
        post_zero_position.controller_debug,
        post_zero_position.motion_fault,
        maximum_payload_bytes,
        output_,
        sizeof(output_)
    )) return fault_process(
      JsonMainControllerFault::kResponseSerializationFailure
    );
    const JsonMainExternalAxisZeroApplyStatus apply_status =
      source->apply(axis, source->context);
    if (apply_status == JsonMainExternalAxisZeroApplyStatus::kApplied) {
      return finish_response(
        JsonMainControllerResponseKind::kExternalAxisZeroCompleted,
        maximum_payload_bytes
      );
    }
    if (apply_status
        == JsonMainExternalAxisZeroApplyStatus::kEmergencyStopActive) {
      return process_late_estop_admission(request, maximum_payload_bytes);
    }
    return fault_process(JsonMainControllerFault::kInvalidCommandSource);
  }

  JsonMainControllerProcessStatus process_home_reference(
    const JsonMainRequestParseResult &result,
    const JsonMainHomeReferenceResponseSource *source,
    JsonMainControllerAdmissionStatus admission,
    size_t maximum_payload_bytes
  ) {
    JsonMainControllerProcessStatus admission_status =
      JsonMainControllerProcessStatus::kControllerFault;
    if (!process_admission(
        result,
        admission,
        maximum_payload_bytes,
        admission_status
    )) {
      return admission_status;
    }
    if (configuration_sync_required_) {
      if (!build_failed_response(
          result.request_id,
          result.command,
          "home_reference_unavailable",
          "home reference requires configuration synchronization",
          maximum_payload_bytes
      )) {
        return fault_process(
          JsonMainControllerFault::kResponseSerializationFailure
        );
      }
      return finish_response(
        JsonMainControllerResponseKind::kHomeReferenceFailed,
        maximum_payload_bytes
      );
    }
    if (source == nullptr) {
      return fault_process(JsonMainControllerFault::kInvalidCommandSource);
    }
    if (range_aliases_internal_storage(source, sizeof(*source))) {
      return fault_process(JsonMainControllerFault::kInternalStorageAlias);
    }
    switch (source->status) {
      case JsonMainHomeReferenceSourceStatus::kHomeReferenceUnavailable:
        if (!build_failed_response(
            result.request_id,
            result.command,
            "home_reference_unavailable",
            "controller home reference is unavailable",
            maximum_payload_bytes
        )) {
          return fault_process(
            JsonMainControllerFault::kResponseSerializationFailure
          );
        }
        return finish_response(
          JsonMainControllerResponseKind::kHomeReferenceFailed,
          maximum_payload_bytes
        );
      case JsonMainHomeReferenceSourceStatus::kAvailable:
        break;
    }
    if (source->status != JsonMainHomeReferenceSourceStatus::kAvailable) {
      return fault_process(JsonMainControllerFault::kInvalidCommandSource);
    }
    if (!build_main_json_home_reference_response(
        result.request_id,
        source->state,
        maximum_payload_bytes,
        output_,
        sizeof(output_)
    )) {
      return fault_process(
        JsonMainControllerFault::kResponseSerializationFailure
      );
    }
    return finish_response(
      JsonMainControllerResponseKind::kHomeReferenceCompleted,
      maximum_payload_bytes
    );
  }

  JsonMainControllerProcessStatus process_motion_trace(
    const JsonMainRequestParseResult &result,
    const JsonMainMotionTraceResponseSource *source,
    JsonMainControllerAdmissionStatus admission,
    size_t maximum_payload_bytes
  ) {
    JsonMainControllerProcessStatus status =
      JsonMainControllerProcessStatus::kControllerFault;
    if (!process_admission(
        result,
        admission,
        maximum_payload_bytes,
        status
    )) return status;
    if (!session_bound_) {
      if (!build_rejected_response(
          result.request_id,
          result.command,
          "session_not_established",
          "hello must complete before motion-trace retrieval",
          maximum_payload_bytes
      )) {
        return fault_process(
          JsonMainControllerFault::kResponseSerializationFailure
        );
      }
      return finish_response(
        JsonMainControllerResponseKind::kMotionTraceRejected,
        maximum_payload_bytes
      );
    }
    const JsonMainMotionTraceParameters *parameters =
      result.payload.motion_trace();
    if (
      source == nullptr
      || source->capture == nullptr
      || parameters == nullptr
      || range_aliases_internal_storage(source->capture, 1)
    ) {
      return fault_process(JsonMainControllerFault::kInvalidCommandSource);
    }
    if (!build_main_json_motion_trace_response(
        result.request_id,
        *parameters,
        *source->capture,
        source->session_id,
        source->firmware_name,
        source->firmware_version,
        source->firmware_build,
        maximum_payload_bytes,
        output_,
        sizeof(output_)
    )) {
      return fault_process(
        JsonMainControllerFault::kResponseSerializationFailure
      );
    }
    return finish_response(
      JsonMainControllerResponseKind::kMotionTraceCompleted,
      maximum_payload_bytes
    );
  }

  JsonMainControllerProcessStatus process_diagnostic(
    const JsonMainRequestParseResult &result,
    const JsonMainDiagnosticCommandSource *source,
    JsonMainControllerAdmissionStatus admission,
    size_t maximum_payload_bytes
  ) {
    JsonMainControllerProcessStatus status =
      JsonMainControllerProcessStatus::kControllerFault;
    if (!process_admission(
        result, admission, maximum_payload_bytes, status
    )) return status;
    const char *code = !session_bound_
      ? "session_not_established"
      : configuration_sync_required_
        ? "configuration_sync_required" : nullptr;
    if (code != nullptr) {
      if (!build_rejected_response(
          result.request_id, result.command, code,
          !session_bound_
            ? "hello must complete before controller diagnostics"
            : "controller diagnostics require configuration synchronization",
          maximum_payload_bytes
      )) return fault_process(
        JsonMainControllerFault::kResponseSerializationFailure
      );
      return finish_response(
        JsonMainControllerResponseKind::kDiagnosticRejected,
        maximum_payload_bytes
      );
    }
    if (source == nullptr || source->execute == nullptr) {
      if (!build_failed_response(
          result.request_id, result.command, "diagnostic_unavailable",
          "controller diagnostic source is unavailable",
          maximum_payload_bytes
      )) return fault_process(
        JsonMainControllerFault::kResponseSerializationFailure
      );
      return finish_response(
        JsonMainControllerResponseKind::kDiagnosticFailed,
        maximum_payload_bytes
      );
    }
    bool active[6] = {};
    int32_t counts[6] = {};
    const JsonMainRequestCommand command_kind =
      result.payload.command_kind();
    if (
      command_kind == JsonMainRequestCommand::kSetEncoders
      && !build_diagnostic_response(
        result, active, counts, maximum_payload_bytes
      )
    ) return fault_process(
      JsonMainControllerFault::kResponseSerializationFailure
    );
    const JsonMainDiagnosticOutcome outcome = source->execute(
      command_kind, active, counts, source->context
    );
    if (outcome == JsonMainDiagnosticOutcome::kEmergencyStopActive) {
      return process_late_estop_admission(result, maximum_payload_bytes);
    }
    if (outcome == JsonMainDiagnosticOutcome::kUnavailable) {
      clear_frame();
      if (!build_failed_response(
          result.request_id, result.command, "diagnostic_unavailable",
          "controller diagnostic operation is unavailable",
          maximum_payload_bytes
      )) return fault_process(
        JsonMainControllerFault::kResponseSerializationFailure
      );
      return finish_response(
        JsonMainControllerResponseKind::kDiagnosticFailed,
        maximum_payload_bytes
      );
    }
    if (outcome != JsonMainDiagnosticOutcome::kCompleted) {
      return fault_process(JsonMainControllerFault::kInvalidCommandSource);
    }
    if (
      command_kind != JsonMainRequestCommand::kSetEncoders
      && !build_diagnostic_response(
        result, active, counts, maximum_payload_bytes
      )
    ) return fault_process(
      JsonMainControllerFault::kResponseSerializationFailure
    );
    return finish_response(
      JsonMainControllerResponseKind::kDiagnosticCompleted,
      maximum_payload_bytes
    );
  }

  JsonMainControllerProcessStatus process_set_position(
    const JsonMainRequestParseResult &result,
    const JsonMainSetPositionCommandSource *source,
    JsonMainControllerAdmissionStatus admission,
    size_t maximum_payload_bytes
  ) {
    JsonMainControllerProcessStatus admission_status =
      JsonMainControllerProcessStatus::kControllerFault;
    if (!process_admission(
        result,
        admission,
        maximum_payload_bytes,
        admission_status
    )) {
      return admission_status;
    }
    if (!session_bound_) {
      if (!build_rejected_response(
          result.request_id,
          result.command,
          "session_not_established",
          "hello must complete before set-position mutation",
          maximum_payload_bytes
      )) {
        return fault_process(
          JsonMainControllerFault::kResponseSerializationFailure
        );
      }
      return finish_response(
        JsonMainControllerResponseKind::kSetPositionRejected,
        maximum_payload_bytes
      );
    }
    if (
      source == nullptr
      || source->apply == nullptr
      || range_aliases_internal_storage(source, sizeof(*source))
      || range_aliases_internal_storage(source->context, 1)
    ) {
      return fault_process(JsonMainControllerFault::kInvalidCommandSource);
    }
    const JsonMainSetPositionParameters *parameters =
      result.payload.set_position();
    if (parameters == nullptr) {
      return fault_process(JsonMainControllerFault::kInvalidCommandSource);
    }
    // Build the acknowledgement before mutation so serialization failure
    // cannot leave applied state without a response owner.
    if (!build_main_json_completed_response(
        result.request_id,
        result.command,
        maximum_payload_bytes,
        output_,
        sizeof(output_)
    )) {
      return fault_process(
        JsonMainControllerFault::kResponseSerializationFailure
      );
    }
    const JsonMainSetPositionApplyStatus apply_status = source->apply(
      *parameters,
      source->context
    );
    if (apply_status == JsonMainSetPositionApplyStatus::kApplied) {
      configuration_sync_required_ = false;
      return finish_response(
        JsonMainControllerResponseKind::kSetPositionCompleted,
        maximum_payload_bytes
      );
    }
    if (
      apply_status
        == JsonMainSetPositionApplyStatus::kPositionNotRepresentable
    ) {
      clear_frame();
      if (!build_rejected_response(
          result.request_id,
          result.command,
          "position_not_representable",
          "requested position cannot be represented by current calibration",
          maximum_payload_bytes
      )) {
        return fault_process(
          JsonMainControllerFault::kResponseSerializationFailure
        );
      }
      return finish_response(
        JsonMainControllerResponseKind::kSetPositionRejected,
        maximum_payload_bytes
      );
    }
    if (
      apply_status == JsonMainSetPositionApplyStatus::kEmergencyStopActive
    ) {
      return process_late_estop_admission(result, maximum_payload_bytes);
    }
    return fault_process(JsonMainControllerFault::kInvalidCommandSource);
  }

  JsonMainControllerProcessStatus process_update_params(
    const JsonMainRequestParseResult &result,
    const JsonMainUpdateParametersCommandSource *source,
    JsonMainControllerAdmissionStatus admission,
    size_t maximum_payload_bytes
  ) {
    JsonMainControllerProcessStatus admission_status =
      JsonMainControllerProcessStatus::kControllerFault;
    if (!process_admission(
        result,
        admission,
        maximum_payload_bytes,
        admission_status
    )) {
      return admission_status;
    }
    if (!session_bound_) {
      if (!build_rejected_response(
          result.request_id,
          result.command,
          "session_not_established",
          "hello must complete before controller configuration",
          maximum_payload_bytes
      )) {
        return fault_process(
          JsonMainControllerFault::kResponseSerializationFailure
        );
      }
      return finish_response(
        JsonMainControllerResponseKind::kUpdateParamsRejected,
        maximum_payload_bytes
      );
    }
    if (
      source == nullptr
      || source->apply == nullptr
      || range_aliases_internal_storage(source, sizeof(*source))
      || range_aliases_internal_storage(source->context, 1)
    ) {
      return fault_process(JsonMainControllerFault::kInvalidCommandSource);
    }
    const JsonMainUpdateParameters *parameters =
      result.payload.update_params();
    if (parameters == nullptr) {
      return fault_process(JsonMainControllerFault::kInvalidCommandSource);
    }
    if (!build_main_json_completed_response(
        result.request_id,
        result.command,
        maximum_payload_bytes,
        output_,
        sizeof(output_)
    )) {
      return fault_process(
        JsonMainControllerFault::kResponseSerializationFailure
      );
    }
    const JsonMainConfigurationApplyStatus apply_status = source->apply(
      *parameters,
      source->context
    );
    if (apply_status == JsonMainConfigurationApplyStatus::kApplied) {
      configuration_sync_required_ = true;
      return finish_response(
        JsonMainControllerResponseKind::kUpdateParamsCompleted,
        maximum_payload_bytes
      );
    }
    if (
      apply_status
        == JsonMainConfigurationApplyStatus::kConfigurationNotRepresentable
    ) {
      clear_frame();
      if (!build_rejected_response(
          result.request_id,
          result.command,
          "configuration_not_representable",
          "requested configuration cannot be represented by controller",
          maximum_payload_bytes
      )) {
        return fault_process(
          JsonMainControllerFault::kResponseSerializationFailure
        );
      }
      return finish_response(
        JsonMainControllerResponseKind::kUpdateParamsRejected,
        maximum_payload_bytes
      );
    }
    if (
      apply_status
        == JsonMainConfigurationApplyStatus::kEmergencyStopActive
    ) {
      return process_late_estop_admission(result, maximum_payload_bytes);
    }
    return fault_process(JsonMainControllerFault::kInvalidCommandSource);
  }

  JsonMainControllerProcessStatus process_config_ext_axis(
    const JsonMainRequestParseResult &result,
    const JsonMainExternalAxisCommandSource *source,
    JsonMainControllerAdmissionStatus admission,
    size_t maximum_payload_bytes
  ) {
    JsonMainControllerProcessStatus admission_status =
      JsonMainControllerProcessStatus::kControllerFault;
    if (!process_admission(
        result,
        admission,
        maximum_payload_bytes,
        admission_status
    )) {
      return admission_status;
    }
    if (!session_bound_) {
      if (!build_rejected_response(
          result.request_id,
          result.command,
          "session_not_established",
          "hello must complete before external-axis configuration",
          maximum_payload_bytes
      )) {
        return fault_process(
          JsonMainControllerFault::kResponseSerializationFailure
        );
      }
      return finish_response(
        JsonMainControllerResponseKind::kConfigExtAxisRejected,
        maximum_payload_bytes
      );
    }
    if (
      source == nullptr
      || source->apply == nullptr
      || range_aliases_internal_storage(source, sizeof(*source))
      || range_aliases_internal_storage(source->context, 1)
    ) {
      return fault_process(JsonMainControllerFault::kInvalidCommandSource);
    }
    const JsonMainExternalAxisParameters *parameters =
      result.payload.config_ext_axis();
    if (parameters == nullptr) {
      return fault_process(JsonMainControllerFault::kInvalidCommandSource);
    }
    if (!build_main_json_completed_response(
        result.request_id,
        result.command,
        maximum_payload_bytes,
        output_,
        sizeof(output_)
    )) {
      return fault_process(
        JsonMainControllerFault::kResponseSerializationFailure
      );
    }
    const JsonMainConfigurationApplyStatus apply_status = source->apply(
      *parameters,
      source->context
    );
    if (apply_status == JsonMainConfigurationApplyStatus::kApplied) {
      configuration_sync_required_ = true;
      return finish_response(
        JsonMainControllerResponseKind::kConfigExtAxisCompleted,
        maximum_payload_bytes
      );
    }
    if (
      apply_status
        == JsonMainConfigurationApplyStatus::kConfigurationNotRepresentable
    ) {
      clear_frame();
      if (!build_rejected_response(
          result.request_id,
          result.command,
          "configuration_not_representable",
          "requested configuration cannot be represented by controller",
          maximum_payload_bytes
      )) {
        return fault_process(
          JsonMainControllerFault::kResponseSerializationFailure
        );
      }
      return finish_response(
        JsonMainControllerResponseKind::kConfigExtAxisRejected,
        maximum_payload_bytes
      );
    }
    if (
      apply_status
        == JsonMainConfigurationApplyStatus::kEmergencyStopActive
    ) {
      return process_late_estop_admission(result, maximum_payload_bytes);
    }
    return fault_process(JsonMainControllerFault::kInvalidCommandSource);
  }

  JsonMainControllerProcessStatus process_controller_wait(
    const JsonMainRequestParseResult &result,
    const JsonMainControllerWaitCommandSource *source,
    JsonMainControllerAdmissionStatus admission,
    size_t maximum_payload_bytes
  ) {
    JsonMainControllerProcessStatus admission_status =
      JsonMainControllerProcessStatus::kControllerFault;
    if (!process_admission(
        result, admission, maximum_payload_bytes, admission_status
    )) return admission_status;
    if (!session_bound_ || configuration_sync_required_) {
      const bool session_missing = !session_bound_;
      if (!build_rejected_response(
          result.request_id,
          result.command,
          session_missing
            ? "session_not_established"
            : "configuration_sync_required",
          session_missing
            ? "hello must complete before controller wait"
            : "set_position must complete before controller wait",
          maximum_payload_bytes
      )) return fault_process(JsonMainControllerFault::kResponseSerializationFailure);
      return finish_response(
        JsonMainControllerResponseKind::kControllerWaitRejected,
        maximum_payload_bytes
      );
    }
    if (source == nullptr) {
      if (!build_rejected_response(
          result.request_id,
          result.command,
          "unsupported_command",
          "request command is unsupported",
          maximum_payload_bytes
      )) return fault_process(
        JsonMainControllerFault::kResponseSerializationFailure
      );
      return finish_response(
        JsonMainControllerResponseKind::kCorrelatedRejection,
        maximum_payload_bytes
      );
    }
    if (source->execute == nullptr) {
      return fault_process(JsonMainControllerFault::kInvalidCommandSource);
    }
    const JsonMainControllerWaitParameters *parameters =
      result.payload.controller_wait();
    if (parameters == nullptr) return fault_process(
      JsonMainControllerFault::kInvalidCommandSource
    );

    // Prove both terminal frames fit before a callback can consume wait time.
    if (!build_main_json_error_response(
        result.request_id, result.command, JsonErrorResponseStatus::kCancelled,
        "emergency_stop",
        "emergency stop interrupted controller wait",
        nullptr, maximum_payload_bytes, output_, sizeof(output_)
    ) || !build_main_json_completed_response(
        result.request_id, result.command, maximum_payload_bytes,
        output_, sizeof(output_)
    )) {
      return fault_process(
        JsonMainControllerFault::kResponseSerializationFailure
      );
    }

    const JsonMainControllerWaitOutcome outcome = source->execute(
      parameters->duration_milliseconds, source->context
    );
    if (outcome == JsonMainControllerWaitOutcome::kCompleted) {
      return finish_response(
        JsonMainControllerResponseKind::kControllerWaitCompleted,
        maximum_payload_bytes
      );
    }
    if (outcome == JsonMainControllerWaitOutcome::kEmergencyStop) {
      clear_frame();
      if (!build_main_json_error_response(
          result.request_id, result.command,
          JsonErrorResponseStatus::kCancelled, "emergency_stop",
          "emergency stop interrupted controller wait",
          nullptr, maximum_payload_bytes, output_, sizeof(output_)
      )) {
        return fault_process(
          JsonMainControllerFault::kResponseSerializationFailure
        );
      }
      return finish_response(
        JsonMainControllerResponseKind::kControllerWaitCancelled,
        maximum_payload_bytes
      );
    }
    return fault_process(JsonMainControllerFault::kInvalidCommandSource);
  }

  JsonMainControllerProcessStatus process_modbus_read(
    const JsonMainRequestParseResult &result,
    const JsonMainModbusReadCommandSource *source,
    JsonMainControllerAdmissionStatus admission, size_t maximum_payload_bytes) {
    JsonMainControllerProcessStatus status = JsonMainControllerProcessStatus::kControllerFault;
    if (!process_admission(
        result, admission, maximum_payload_bytes, status)) return status;
    const bool session_missing = !session_bound_;
    if (session_missing || configuration_sync_required_) {
      if (!build_rejected_response(
          result.request_id, result.command,
          session_missing ? "session_not_established"
            : "configuration_sync_required",
          session_missing ? "hello must complete before Modbus read"
            : "set_position must complete before Modbus read",
          maximum_payload_bytes
      )) return fault_process(JsonMainControllerFault::kResponseSerializationFailure);
      return finish_response(JsonMainControllerResponseKind::kModbusReadRejected, maximum_payload_bytes);
    }
    if (source == nullptr) {
      if (!build_rejected_response(
          result.request_id, result.command, "unsupported_command",
          "request command is unsupported", maximum_payload_bytes
      )) return fault_process(JsonMainControllerFault::kResponseSerializationFailure);
      return finish_response(JsonMainControllerResponseKind::kCorrelatedRejection, maximum_payload_bytes);
    }
    const JsonMainModbusReadParameters *parameters = result.payload.modbus_read();
    ModbusOperation operation = ModbusOperation::kReadCoil;
    const JsonMainRequestCommand command = result.payload.command_kind();
    if (
      source->execute == nullptr || parameters == nullptr
      || !json_main_modbus_read_parameters_valid(
        command, *parameters
      ) || !json_main_modbus_read_operation(
        command, operation
      )
    ) return fault_process(JsonMainControllerFault::kInvalidCommandSource);

    // This longest terminal proves every response fits before bus access.
    if (!build_main_json_error_response(
        result.request_id, result.command, JsonErrorResponseStatus::kCancelled,
        "emergency_stop", "emergency stop interrupted Modbus read", nullptr,
        maximum_payload_bytes, output_, sizeof(output_)
      )) return fault_process(JsonMainControllerFault::kResponseSerializationFailure);
    int32_t value = -1;
    const JsonMainModbusReadOutcome outcome = source->execute(
      operation, *parameters, value, source->context);
    if (outcome == JsonMainModbusReadOutcome::kCompleted) {
      if (!modbus_read_value_valid(operation, value)) return fault_process(
        JsonMainControllerFault::kInvalidCommandSource);
      if (!build_modbus_read_response(result, value, maximum_payload_bytes)) {
        return fault_process(
          JsonMainControllerFault::kResponseSerializationFailure);
      }
      return finish_response(JsonMainControllerResponseKind::kModbusReadCompleted, maximum_payload_bytes);
    }
    const bool stopped = outcome == JsonMainModbusReadOutcome::kEmergencyStop;
    if (stopped || outcome == JsonMainModbusReadOutcome::kModbusError) {
      if (!build_main_json_error_response(
          result.request_id, result.command,
          stopped ? JsonErrorResponseStatus::kCancelled : JsonErrorResponseStatus::kFailed,
          stopped ? "emergency_stop" : "modbus_error",
          stopped ? "emergency stop interrupted Modbus read" : "Modbus read failed",
          nullptr, maximum_payload_bytes, output_, sizeof(output_)
      )) return fault_process(JsonMainControllerFault::kResponseSerializationFailure);
      const JsonMainControllerResponseKind kind = stopped
        ? JsonMainControllerResponseKind::kModbusReadCancelled
        : JsonMainControllerResponseKind::kModbusReadFailed;
      return finish_response(kind, maximum_payload_bytes);
    }
    return fault_process(JsonMainControllerFault::kInvalidCommandSource);
  }

  JsonMainControllerProcessStatus process_modbus_write(
    const JsonMainRequestParseResult &result,
    const JsonMainModbusWriteCommandSource *source,
    JsonMainControllerAdmissionStatus admission, size_t maximum_payload_bytes) {
    JsonMainControllerProcessStatus status = JsonMainControllerProcessStatus::kControllerFault;
    if (!process_admission(
        result, admission, maximum_payload_bytes, status)) return status;
    const bool session_missing = !session_bound_;
    if (session_missing || configuration_sync_required_) {
      if (!build_rejected_response(
          result.request_id, result.command,
          session_missing ? "session_not_established"
            : "configuration_sync_required",
          session_missing ? "hello must complete before Modbus write"
            : "set_position must complete before Modbus write",
          maximum_payload_bytes
      )) return fault_process(JsonMainControllerFault::kResponseSerializationFailure);
      return finish_response(JsonMainControllerResponseKind::kModbusWriteRejected, maximum_payload_bytes);
    }
    if (source == nullptr) {
      if (!build_rejected_response(
          result.request_id, result.command, "unsupported_command",
          "request command is unsupported", maximum_payload_bytes
      )) return fault_process(JsonMainControllerFault::kResponseSerializationFailure);
      return finish_response(JsonMainControllerResponseKind::kCorrelatedRejection, maximum_payload_bytes);
    }
    const JsonMainModbusWriteParameters *parameters = result.payload.modbus_write();
    ModbusOperation operation = ModbusOperation::kWriteCoil;
    const JsonMainRequestCommand command = result.payload.command_kind();
    if (
      source->execute == nullptr || parameters == nullptr
      || !json_main_modbus_write_parameters_valid(command, *parameters)
      || !json_main_modbus_write_operation(command, operation)
    ) return fault_process(JsonMainControllerFault::kInvalidCommandSource);

    // Prove every terminal frame fits before the callback can access the bus.
    if (!build_main_json_error_response(
        result.request_id, result.command, JsonErrorResponseStatus::kFailed,
        "modbus_error", "Modbus write failed", nullptr,
        maximum_payload_bytes, output_, sizeof(output_)
      ) || !build_main_json_error_response(
        result.request_id, result.command, JsonErrorResponseStatus::kCancelled,
        "emergency_stop", "emergency stop interrupted Modbus write", nullptr,
        maximum_payload_bytes, output_, sizeof(output_)
      ) || !build_main_json_completed_response(
        result.request_id, result.command, maximum_payload_bytes,
        output_, sizeof(output_)
      )) return fault_process(JsonMainControllerFault::kResponseSerializationFailure);
    const JsonMainModbusWriteOutcome outcome = source->execute(
      operation, *parameters, source->context);
    if (outcome == JsonMainModbusWriteOutcome::kCompleted) return finish_response(
      JsonMainControllerResponseKind::kModbusWriteCompleted, maximum_payload_bytes);
    const bool stopped = outcome == JsonMainModbusWriteOutcome::kEmergencyStop;
    if (stopped || outcome == JsonMainModbusWriteOutcome::kModbusError) {
      if (!build_main_json_error_response(
          result.request_id, result.command,
          stopped ? JsonErrorResponseStatus::kCancelled : JsonErrorResponseStatus::kFailed,
          stopped ? "emergency_stop" : "modbus_error",
          stopped ? "emergency stop interrupted Modbus write" : "Modbus write failed",
          nullptr, maximum_payload_bytes, output_, sizeof(output_)
      )) return fault_process(JsonMainControllerFault::kResponseSerializationFailure);
      return finish_response(
        stopped ? JsonMainControllerResponseKind::kModbusWriteCancelled
          : JsonMainControllerResponseKind::kModbusWriteFailed,
        maximum_payload_bytes);
    }
    return fault_process(JsonMainControllerFault::kInvalidCommandSource);
  }

  JsonMainControllerProcessStatus process_calibration(
    const JsonMainRequestParseResult &result,
    const JsonMainCalibrationCommandSource *source,
    JsonMainControllerAdmissionStatus admission,
    size_t maximum_payload_bytes
  ) {
    JsonMainControllerProcessStatus admission_status =
      JsonMainControllerProcessStatus::kControllerFault;
    if (!process_admission(
        result,
        admission,
        maximum_payload_bytes,
        admission_status
    )) {
      return admission_status;
    }
    if (!session_bound_) {
      if (!build_rejected_response(
          result.request_id,
          result.command,
          "session_not_established",
          "hello must complete before calibration",
          maximum_payload_bytes
      )) {
        return fault_process(
          JsonMainControllerFault::kResponseSerializationFailure
        );
      }
      return finish_response(
        JsonMainControllerResponseKind::kCalibrationRejected,
        maximum_payload_bytes
      );
    }
    if (configuration_sync_required_) {
      if (!build_rejected_response(
          result.request_id,
          result.command,
          "configuration_sync_required",
          "set_position must complete before calibration",
          maximum_payload_bytes
      )) {
        return fault_process(
          JsonMainControllerFault::kResponseSerializationFailure
        );
      }
      return finish_response(
        JsonMainControllerResponseKind::kCalibrationRejected,
        maximum_payload_bytes
      );
    }
    if (
      source == nullptr
      || source->execute == nullptr
      || range_aliases_internal_storage(source, sizeof(*source))
      || range_aliases_internal_storage(source->context, 1)
    ) {
      return fault_process(JsonMainControllerFault::kInvalidCommandSource);
    }
    const JsonMainCalibrationParameters *parameters =
      result.payload.calibration();
    if (parameters == nullptr) {
      return fault_process(JsonMainControllerFault::kInvalidCommandSource);
    }
    // Response capacity must be guaranteed before calibration can pulse a motor.
    if (
      maximum_payload_bytes
        < kJsonCalibrationTerminalPayloadReservationBytes
    ) {
      return fault_process(
        JsonMainControllerFault::kResponseSerializationFailure
      );
    }
    JsonMainCalibrationExecutionResult execution_result = {};
    if (!source->execute(
        *parameters,
        execution_result,
        source->context
    )) {
      return fault_process(JsonMainControllerFault::kInvalidCommandSource);
    }
    if (!json_main_calibration_execution_result_valid(execution_result)) {
      return fault_process(JsonMainControllerFault::kInvalidCommandSource);
    }
    if (!build_main_json_calibration_response(
        result.request_id,
        execution_result,
        maximum_payload_bytes,
        output_,
        sizeof(output_)
    )) {
      return fault_process(
        JsonMainControllerFault::kResponseSerializationFailure
      );
    }
    JsonMainControllerResponseKind response_kind =
      JsonMainControllerResponseKind::kNone;
    switch (execution_result.outcome) {
      case JsonMainCalibrationOutcome::kCompleted:
        response_kind = JsonMainControllerResponseKind::kCalibrationCompleted;
        break;
      case JsonMainCalibrationOutcome::kNotRepresentable:
        response_kind = JsonMainControllerResponseKind::kCalibrationRejected;
        break;
      case JsonMainCalibrationOutcome::kEmergencyStop:
        response_kind = JsonMainControllerResponseKind::kCalibrationCancelled;
        break;
      case JsonMainCalibrationOutcome::kCalibrationFailed:
      case JsonMainCalibrationOutcome::kPositionUnavailable:
        response_kind = JsonMainControllerResponseKind::kCalibrationFailed;
        break;
      case JsonMainCalibrationOutcome::kInvalid:
        return fault_process(JsonMainControllerFault::kInvalidCommandSource);
    }
    return finish_response(response_kind, maximum_payload_bytes);
  }

  JsonMainControllerProcessStatus process_live_jog(
    const JsonMainRequestParseResult &result,
    const JsonMainLiveJogCommandSource *source,
    JsonMainControllerAdmissionStatus admission,
    size_t maximum_payload_bytes
  ) {
    JsonMainControllerProcessStatus admission_status =
      JsonMainControllerProcessStatus::kControllerFault;
    if (!process_admission(
        result,
        admission,
        maximum_payload_bytes,
        admission_status
    )) {
      return admission_status;
    }
    if (!session_bound_) {
      if (!build_rejected_response(
          result.request_id,
          result.command,
          "session_not_established",
          "hello must complete before live jogging",
          maximum_payload_bytes
      )) {
        return fault_process(
          JsonMainControllerFault::kResponseSerializationFailure
        );
      }
      return finish_response(
        JsonMainControllerResponseKind::kCorrelatedRejection,
        maximum_payload_bytes
      );
    }
    if (configuration_sync_required_) {
      if (!build_rejected_response(
          result.request_id,
          result.command,
          "configuration_sync_required",
          "set_position must complete before live jogging",
          maximum_payload_bytes
      )) {
        return fault_process(
          JsonMainControllerFault::kResponseSerializationFailure
        );
      }
      return finish_response(
        JsonMainControllerResponseKind::kCorrelatedRejection,
        maximum_payload_bytes
      );
    }
    const JsonMainLiveJogParameters *parameters = result.payload.live_jog();
    if (
      source == nullptr
      || source->begin == nullptr
      || source->stop == nullptr
      || source->renew == nullptr
      || source->active_motion_id != 0
      || range_aliases_internal_storage(source, sizeof(*source))
      || range_aliases_internal_storage(source->context, 1)
      || active_live_motion_id_ != 0
      || active_live_kind_ != JsonLiveJogKind::kInvalid
      || pending_live_motion_id_ != 0
      || pending_live_kind_ != JsonLiveJogKind::kInvalid
      || pending_playback_request_id_ != 0
      || active_playback_request_id_ != 0
      || playback_execution_active_
      || parameters == nullptr
      || !json_live_jog_detail::parameters_valid(*parameters)
      || json_live_jog_detail::command_name(
        parameters->kind
      ) == nullptr
      || strcmp(
        result.command,
        json_live_jog_detail::command_name(parameters->kind)
      ) != 0
    ) {
      return fault_process(JsonMainControllerFault::kInvalidCommandSource);
    }
    if (
      maximum_payload_bytes
        < kJsonLiveJogTerminalPayloadReservationBytes
      || !build_main_json_accepted_response(
        result.request_id,
        result.command,
        maximum_payload_bytes,
        output_,
        sizeof(output_)
      )
    ) {
      return fault_process(
        JsonMainControllerFault::kResponseSerializationFailure
      );
    }
    if (!source->begin(
        result.request_id,
        *parameters,
        source->context
    )) {
      return fault_process(JsonMainControllerFault::kInvalidCommandSource);
    }
    pending_live_motion_id_ = result.request_id;
    pending_live_kind_ = parameters->kind;
    return finish_response(
      JsonMainControllerResponseKind::kLiveJogAccepted,
      maximum_payload_bytes
    );
  }

  JsonMainControllerProcessStatus process_stop(
    const JsonMainRequestParseResult &result,
    const JsonMainLiveJogCommandSource *source,
    JsonMainControllerAdmissionStatus admission,
    size_t maximum_payload_bytes
  ) {
    JsonMainControllerProcessStatus admission_status =
      JsonMainControllerProcessStatus::kControllerFault;
    if (!process_admission(
        result,
        admission,
        maximum_payload_bytes,
        admission_status
    )) {
      return admission_status;
    }
    if (!session_bound_) {
      if (!build_rejected_response(
          result.request_id,
          result.command,
          "session_not_established",
          "hello must complete before live-motion stop",
          maximum_payload_bytes
      )) {
        return fault_process(
          JsonMainControllerFault::kResponseSerializationFailure
        );
      }
      return finish_response(
        JsonMainControllerResponseKind::kStopRejected,
        maximum_payload_bytes
      );
    }
    if (
      source == nullptr
      || source->begin == nullptr
      || source->stop == nullptr
      || source->renew == nullptr
      || range_aliases_internal_storage(source, sizeof(*source))
      || range_aliases_internal_storage(source->context, 1)
      || source->active_motion_id != active_live_motion_id_
    ) {
      return fault_process(JsonMainControllerFault::kInvalidCommandSource);
    }
    const JsonMainStopParameters *parameters = result.payload.stop();
    if (parameters == nullptr) {
      return fault_process(JsonMainControllerFault::kInvalidCommandSource);
    }
    if (active_live_motion_id_ == 0) {
      if (!build_rejected_response(
          result.request_id,
          result.command,
          "no_live_motion",
          "no live motion is active",
          maximum_payload_bytes
      )) {
        return fault_process(
          JsonMainControllerFault::kResponseSerializationFailure
        );
      }
      return finish_response(
        JsonMainControllerResponseKind::kStopRejected,
        maximum_payload_bytes
      );
    }
    if (result.request_id == active_live_motion_id_) {
      const JsonStringErrorDetail detail = {"field", "params"};
      if (!build_main_json_error_response(
          result.request_id,
          result.command,
          JsonErrorResponseStatus::kRejected,
          "invalid_parameter",
          "stop request must use a distinct correlation identifier",
          &detail,
          maximum_payload_bytes,
          output_,
          sizeof(output_)
      )) {
        return fault_process(
          JsonMainControllerFault::kResponseSerializationFailure
        );
      }
      return finish_response(
        JsonMainControllerResponseKind::kStopRejected,
        maximum_payload_bytes
      );
    }
    if (parameters->motion_id != active_live_motion_id_) {
      if (!build_main_json_stop_mismatch_response(
          result.request_id,
          active_live_motion_id_,
          maximum_payload_bytes,
          output_,
          sizeof(output_)
      )) {
        return fault_process(
          JsonMainControllerFault::kResponseSerializationFailure
        );
      }
      return finish_response(
        JsonMainControllerResponseKind::kStopRejected,
        maximum_payload_bytes
      );
    }
    if (!build_main_json_stop_completed_response(
        result.request_id,
        parameters->motion_id,
        maximum_payload_bytes,
        output_,
        sizeof(output_)
    )) {
      return fault_process(
        JsonMainControllerFault::kResponseSerializationFailure
      );
    }
    if (!source->stop(parameters->motion_id, source->context)) {
      return fault_process(JsonMainControllerFault::kInvalidCommandSource);
    }
    return finish_response(
      JsonMainControllerResponseKind::kStopCompleted,
      maximum_payload_bytes
    );
  }

  JsonMainControllerProcessStatus process_renew_live_motion(
    const JsonMainRequestParseResult &result,
    const JsonMainLiveJogCommandSource *source,
    JsonMainControllerAdmissionStatus admission,
    size_t maximum_payload_bytes
  ) {
    JsonMainControllerProcessStatus admission_status =
      JsonMainControllerProcessStatus::kControllerFault;
    if (!process_admission(
        result,
        admission,
        maximum_payload_bytes,
        admission_status
    )) {
      return admission_status;
    }
    if (!session_bound_) {
      if (!build_rejected_response(
          result.request_id,
          result.command,
          "session_not_established",
          "hello must complete before live-motion renewal",
          maximum_payload_bytes
      )) {
        return fault_process(
          JsonMainControllerFault::kResponseSerializationFailure
        );
      }
      return finish_response(
        JsonMainControllerResponseKind::kRenewLiveMotionRejected,
        maximum_payload_bytes
      );
    }
    if (
      source == nullptr
      || source->begin == nullptr
      || source->stop == nullptr
      || source->renew == nullptr
      || range_aliases_internal_storage(source, sizeof(*source))
      || range_aliases_internal_storage(source->context, 1)
      || source->active_motion_id != active_live_motion_id_
    ) {
      return fault_process(JsonMainControllerFault::kInvalidCommandSource);
    }
    const JsonMainRenewLiveMotionParameters *parameters =
      result.payload.renew_live_motion();
    if (parameters == nullptr) {
      return fault_process(JsonMainControllerFault::kInvalidCommandSource);
    }
    if (active_live_motion_id_ == 0) {
      if (!build_rejected_response(
          result.request_id,
          result.command,
          "no_live_motion",
          "no live motion is active",
          maximum_payload_bytes
      )) {
        return fault_process(
          JsonMainControllerFault::kResponseSerializationFailure
        );
      }
      return finish_response(
        JsonMainControllerResponseKind::kRenewLiveMotionRejected,
        maximum_payload_bytes
      );
    }
    if (result.request_id == active_live_motion_id_) {
      const JsonStringErrorDetail detail = {"field", "params"};
      if (!build_main_json_error_response(
          result.request_id,
          result.command,
          JsonErrorResponseStatus::kRejected,
          "invalid_parameter",
          "renewal request must use a distinct correlation identifier",
          &detail,
          maximum_payload_bytes,
          output_,
          sizeof(output_)
      )) {
        return fault_process(
          JsonMainControllerFault::kResponseSerializationFailure
        );
      }
      return finish_response(
        JsonMainControllerResponseKind::kRenewLiveMotionRejected,
        maximum_payload_bytes
      );
    }
    if (parameters->motion_id != active_live_motion_id_) {
      if (!build_main_json_live_motion_control_mismatch_response(
          "renew_live_motion",
          result.request_id,
          active_live_motion_id_,
          maximum_payload_bytes,
          output_,
          sizeof(output_)
      )) {
        return fault_process(
          JsonMainControllerFault::kResponseSerializationFailure
        );
      }
      return finish_response(
        JsonMainControllerResponseKind::kRenewLiveMotionRejected,
        maximum_payload_bytes
      );
    }
    const JsonMainLiveJogRenewStatus renew_status = source->renew(
      parameters->motion_id,
      source->context
    );
    switch (renew_status) {
      case JsonMainLiveJogRenewStatus::kRenewed:
        if (!build_main_json_renew_live_motion_completed_response(
            result.request_id,
            parameters->motion_id,
            maximum_payload_bytes,
            output_,
            sizeof(output_)
        )) {
          return fault_process(
            JsonMainControllerFault::kResponseSerializationFailure
          );
        }
        return finish_response(
          JsonMainControllerResponseKind::kRenewLiveMotionCompleted,
          maximum_payload_bytes
        );
      case JsonMainLiveJogRenewStatus::kLeaseExpired:
        if (!build_rejected_response(
            result.request_id,
            result.command,
            "control_lease_expired",
            "live-motion control lease expired",
            maximum_payload_bytes
        )) {
          return fault_process(
            JsonMainControllerFault::kResponseSerializationFailure
          );
        }
        return finish_response(
          JsonMainControllerResponseKind::kRenewLiveMotionRejected,
          maximum_payload_bytes
        );
      case JsonMainLiveJogRenewStatus::kMotionSettled:
        if (!build_rejected_response(
            result.request_id,
            result.command,
            "no_live_motion",
            "no renewable live motion is active",
            maximum_payload_bytes
        )) {
          return fault_process(
            JsonMainControllerFault::kResponseSerializationFailure
          );
        }
        return finish_response(
          JsonMainControllerResponseKind::kRenewLiveMotionRejected,
          maximum_payload_bytes
        );
      case JsonMainLiveJogRenewStatus::kFault:
        return fault_process(JsonMainControllerFault::kInvalidCommandSource);
    }
    return fault_process(JsonMainControllerFault::kInvalidCommandSource);
  }

  JsonMainControllerProcessStatus process_jog_tool(
    const JsonMainRequestParseResult &result,
    const JsonMainToolJogCommandSource *source,
    JsonMainControllerAdmissionStatus admission,
    size_t maximum_payload_bytes
  ) {
    JsonMainControllerProcessStatus admission_status =
      JsonMainControllerProcessStatus::kControllerFault;
    if (!process_admission(
        result,
        admission,
        maximum_payload_bytes,
        admission_status
    )) {
      return admission_status;
    }
    if (!session_bound_) {
      if (!build_rejected_response(
          result.request_id,
          result.command,
          "session_not_established",
          "hello must complete before tool-frame jogging",
          maximum_payload_bytes
      )) {
        return fault_process(
          JsonMainControllerFault::kResponseSerializationFailure
        );
      }
      return finish_response(
        JsonMainControllerResponseKind::kJogToolRejected,
        maximum_payload_bytes
      );
    }
    if (configuration_sync_required_) {
      if (!build_rejected_response(
          result.request_id,
          result.command,
          "configuration_sync_required",
          "set_position must complete before tool-frame jogging",
          maximum_payload_bytes
      )) {
        return fault_process(
          JsonMainControllerFault::kResponseSerializationFailure
        );
      }
      return finish_response(
        JsonMainControllerResponseKind::kJogToolRejected,
        maximum_payload_bytes
      );
    }
    if (
      source == nullptr
      || source->execute == nullptr
      || range_aliases_internal_storage(source, sizeof(*source))
      || range_aliases_internal_storage(source->context, 1)
    ) {
      return fault_process(JsonMainControllerFault::kInvalidCommandSource);
    }
    const JsonMainToolJogParameters *parameters = result.payload.jog_tool();
    if (parameters == nullptr) {
      return fault_process(JsonMainControllerFault::kInvalidCommandSource);
    }
    // Terminal capacity is reserved before physical mutation begins.
    if (
      maximum_payload_bytes
        < kJsonToolJogTerminalPayloadReservationBytes
    ) {
      return fault_process(
        JsonMainControllerFault::kResponseSerializationFailure
      );
    }
    JsonMainToolJogExecutionResult execution_result = {};
    if (!source->execute(
        *parameters,
        execution_result,
        source->context
    )) {
      return fault_process(JsonMainControllerFault::kInvalidCommandSource);
    }
    if (!json_main_tool_jog_execution_result_valid(
        execution_result
    )) {
      return fault_process(JsonMainControllerFault::kInvalidCommandSource);
    }
    if (!build_main_json_tool_jog_response(
        result.request_id,
        execution_result,
        maximum_payload_bytes,
        output_,
        sizeof(output_)
    )) {
      return fault_process(
        JsonMainControllerFault::kResponseSerializationFailure
      );
    }
    JsonMainControllerResponseKind response_kind =
      JsonMainControllerResponseKind::kNone;
    switch (execution_result.outcome) {
      case JsonMainToolJogOutcome::kCompleted:
        response_kind = JsonMainControllerResponseKind::kJogToolCompleted;
        break;
      case JsonMainToolJogOutcome::kKinematicsUnreachable:
      case JsonMainToolJogOutcome::kJointLimitViolation:
      case JsonMainToolJogOutcome::kPositionNotRepresentable:
        response_kind = JsonMainControllerResponseKind::kJogToolRejected;
        break;
      case JsonMainToolJogOutcome::kEmergencyStop:
        response_kind = JsonMainControllerResponseKind::kJogToolCancelled;
        break;
      case JsonMainToolJogOutcome::kPositionUnavailable:
      case JsonMainToolJogOutcome::kMotionExecutionFailed:
      case JsonMainToolJogOutcome::kEncoderCollision:
      case JsonMainToolJogOutcome::kEncoderStateUnavailable:
        response_kind = JsonMainControllerResponseKind::kJogToolFailed;
        break;
      case JsonMainToolJogOutcome::kInvalid:
        return fault_process(JsonMainControllerFault::kInvalidCommandSource);
    }
    return finish_response(response_kind, maximum_payload_bytes);
  }

  JsonMainControllerProcessStatus process_move_cartesian(
    const JsonMainRequestParseResult &result,
    const JsonMainMoveCartesianCommandSource *source,
    JsonMainControllerAdmissionStatus admission,
    size_t maximum_payload_bytes
  ) {
    JsonMainControllerProcessStatus admission_status =
      JsonMainControllerProcessStatus::kControllerFault;
    if (!process_admission(
        result,
        admission,
        maximum_payload_bytes,
        admission_status
    )) {
      return admission_status;
    }
    if (!session_bound_) {
      if (!build_rejected_response(
          result.request_id,
          result.command,
          "session_not_established",
          "hello must complete before Cartesian motion",
          maximum_payload_bytes
      )) {
        return fault_process(
          JsonMainControllerFault::kResponseSerializationFailure
        );
      }
      return finish_response(
        JsonMainControllerResponseKind::kMoveCartesianRejected,
        maximum_payload_bytes
      );
    }
    if (configuration_sync_required_) {
      if (!build_rejected_response(
          result.request_id,
          result.command,
          "configuration_sync_required",
          "set_position must complete before Cartesian motion",
          maximum_payload_bytes
      )) {
        return fault_process(
          JsonMainControllerFault::kResponseSerializationFailure
        );
      }
      return finish_response(
        JsonMainControllerResponseKind::kMoveCartesianRejected,
        maximum_payload_bytes
      );
    }
    if (
      source == nullptr
      || source->execute == nullptr
      || range_aliases_internal_storage(source, sizeof(*source))
      || range_aliases_internal_storage(source->context, 1)
    ) {
      return fault_process(JsonMainControllerFault::kInvalidCommandSource);
    }
    const JsonMainMoveCartesianParameters *parameters =
      result.payload.move_cartesian();
    if (parameters == nullptr) {
      return fault_process(JsonMainControllerFault::kInvalidCommandSource);
    }
    // Terminal capacity is reserved before physical mutation begins.
    if (
      maximum_payload_bytes
        < kJsonCartesianMotionTerminalPayloadReservationBytes
    ) {
      return fault_process(
        JsonMainControllerFault::kResponseSerializationFailure
      );
    }
    JsonMainMoveCartesianExecutionResult execution_result = {};
    if (!source->execute(
        *parameters,
        execution_result,
        source->context
    )) {
      return fault_process(JsonMainControllerFault::kInvalidCommandSource);
    }
    if (!json_main_move_cartesian_execution_result_valid(
        execution_result
    )) {
      return fault_process(JsonMainControllerFault::kInvalidCommandSource);
    }
    if (!build_main_json_move_cartesian_response(
        result.request_id,
        execution_result,
        maximum_payload_bytes,
        output_,
        sizeof(output_)
    )) {
      return fault_process(
        JsonMainControllerFault::kResponseSerializationFailure
      );
    }
    JsonMainControllerResponseKind response_kind =
      JsonMainControllerResponseKind::kNone;
    switch (execution_result.outcome) {
      case JsonMainMoveCartesianOutcome::kCompleted:
        response_kind =
          JsonMainControllerResponseKind::kMoveCartesianCompleted;
        break;
      case JsonMainMoveCartesianOutcome::kKinematicsUnreachable:
      case JsonMainMoveCartesianOutcome::kJointLimitViolation:
      case JsonMainMoveCartesianOutcome::kPositionNotRepresentable:
        response_kind =
          JsonMainControllerResponseKind::kMoveCartesianRejected;
        break;
      case JsonMainMoveCartesianOutcome::kEmergencyStop:
        response_kind =
          JsonMainControllerResponseKind::kMoveCartesianCancelled;
        break;
      case JsonMainMoveCartesianOutcome::kPositionUnavailable:
      case JsonMainMoveCartesianOutcome::kMotionExecutionFailed:
      case JsonMainMoveCartesianOutcome::kEncoderCollision:
      case JsonMainMoveCartesianOutcome::kEncoderStateUnavailable:
        response_kind =
          JsonMainControllerResponseKind::kMoveCartesianFailed;
        break;
      case JsonMainMoveCartesianOutcome::kInvalid:
        return fault_process(JsonMainControllerFault::kInvalidCommandSource);
    }
    return finish_response(response_kind, maximum_payload_bytes);
  }

  JsonMainControllerProcessStatus process_move_joints(
    const JsonMainRequestParseResult &result,
    const JsonMainMoveJointsCommandSource *source,
    JsonMainControllerAdmissionStatus admission,
    size_t maximum_payload_bytes
  ) {
    JsonMainControllerProcessStatus admission_status =
      JsonMainControllerProcessStatus::kControllerFault;
    if (!process_admission(
        result,
        admission,
        maximum_payload_bytes,
        admission_status
    )) {
      return admission_status;
    }
    if (!session_bound_) {
      if (!build_rejected_response(
          result.request_id,
          result.command,
          "session_not_established",
          "hello must complete before joint motion",
          maximum_payload_bytes
      )) {
        return fault_process(
          JsonMainControllerFault::kResponseSerializationFailure
        );
      }
      return finish_response(
        JsonMainControllerResponseKind::kMoveJointsRejected,
        maximum_payload_bytes
      );
    }
    if (configuration_sync_required_) {
      if (!build_rejected_response(
          result.request_id,
          result.command,
          "configuration_sync_required",
          "set_position must complete before joint motion",
          maximum_payload_bytes
      )) {
        return fault_process(
          JsonMainControllerFault::kResponseSerializationFailure
        );
      }
      return finish_response(
        JsonMainControllerResponseKind::kMoveJointsRejected,
        maximum_payload_bytes
      );
    }
    if (
      source == nullptr
      || source->execute == nullptr
      || range_aliases_internal_storage(source, sizeof(*source))
      || range_aliases_internal_storage(source->context, 1)
    ) {
      return fault_process(JsonMainControllerFault::kInvalidCommandSource);
    }
    const JsonMainMoveJointsParameters *parameters =
      result.payload.move_joints();
    if (parameters == nullptr) {
      return fault_process(JsonMainControllerFault::kInvalidCommandSource);
    }
    // Terminal capacity is reserved before physical mutation begins.
    if (
      maximum_payload_bytes
        < kJsonJointMotionTerminalPayloadReservationBytes
    ) {
      return fault_process(
        JsonMainControllerFault::kResponseSerializationFailure
      );
    }
    JsonMainMoveJointsExecutionResult execution_result = {};
    if (!source->execute(
        result.request_id,
        *parameters,
        execution_result,
        source->context
    )) {
      return fault_process(JsonMainControllerFault::kInvalidCommandSource);
    }
    if (!json_main_move_joints_execution_result_valid(execution_result)) {
      return fault_process(JsonMainControllerFault::kInvalidCommandSource);
    }
    if (!build_main_json_move_joints_response(
        result.request_id,
        execution_result,
        maximum_payload_bytes,
        output_,
        sizeof(output_)
    )) {
      return fault_process(
        JsonMainControllerFault::kResponseSerializationFailure
      );
    }
    JsonMainControllerResponseKind response_kind =
      JsonMainControllerResponseKind::kNone;
    switch (execution_result.outcome) {
      case JsonMainMoveJointsOutcome::kCompleted:
        response_kind = JsonMainControllerResponseKind::kMoveJointsCompleted;
        break;
      case JsonMainMoveJointsOutcome::kJointLimitViolation:
      case JsonMainMoveJointsOutcome::kPositionNotRepresentable:
        response_kind = JsonMainControllerResponseKind::kMoveJointsRejected;
        break;
      case JsonMainMoveJointsOutcome::kEmergencyStop:
        response_kind = JsonMainControllerResponseKind::kMoveJointsCancelled;
        break;
      case JsonMainMoveJointsOutcome::kPositionUnavailable:
      case JsonMainMoveJointsOutcome::kMotionExecutionFailed:
      case JsonMainMoveJointsOutcome::kEncoderCollision:
      case JsonMainMoveJointsOutcome::kEncoderStateUnavailable:
        response_kind = JsonMainControllerResponseKind::kMoveJointsFailed;
        break;
      case JsonMainMoveJointsOutcome::kInvalid:
        return fault_process(JsonMainControllerFault::kInvalidCommandSource);
    }
    return finish_response(response_kind, maximum_payload_bytes);
  }

  JsonMainControllerProcessStatus process_direct(
    const JsonMainRequestParseResult &result,
    const JsonMainDirectCommandSource *source,
    JsonMainControllerAdmissionStatus admission,
    size_t maximum_payload_bytes
  ) {
    JsonMainControllerProcessStatus admission_status =
      JsonMainControllerProcessStatus::kControllerFault;
    if (!process_admission(
        result,
        admission,
        maximum_payload_bytes,
        admission_status
    )) {
      return admission_status;
    }
    if (!session_bound_) {
      if (!build_rejected_response(
          result.request_id,
          result.command,
          "session_not_established",
          "hello must complete before commands",
          maximum_payload_bytes
      )) {
        return fault_process(
          JsonMainControllerFault::kResponseSerializationFailure
        );
      }
      return finish_response(
        JsonMainControllerResponseKind::kCorrelatedRejection,
        maximum_payload_bytes
      );
    }
    if (configuration_sync_required_) {
      if (!build_rejected_response(
          result.request_id,
          result.command,
          "configuration_sync_required",
          "set_position must complete before commands",
          maximum_payload_bytes
      )) {
        return fault_process(
          JsonMainControllerFault::kResponseSerializationFailure
        );
      }
      return finish_response(
        JsonMainControllerResponseKind::kCorrelatedRejection,
        maximum_payload_bytes
      );
    }
    if (
      source == nullptr
      || source->execute == nullptr
      || range_aliases_internal_storage(source, sizeof(*source))
      || range_aliases_internal_storage(source->context, 1)
    ) {
      return fault_process(JsonMainControllerFault::kInvalidCommandSource);
    }
    const JsonMainDirectParameters *parameters = result.payload.direct();
    if (parameters == nullptr) {
      return fault_process(JsonMainControllerFault::kInvalidCommandSource);
    }
    if (
      maximum_payload_bytes
        < kJsonCartesianMotionTerminalPayloadReservationBytes
    ) {
      return fault_process(
        JsonMainControllerFault::kResponseSerializationFailure
      );
    }
    const char *command =
      json_main_request_command_name(result.payload.command_kind());
    if (command == nullptr) {
      return fault_process(JsonMainControllerFault::kInvalidCommandSource);
    }
    if (strcmp(command, "play_gcode_file") == 0) {
      if (
        parameters->kind != JsonMainDirectParameterKind::kStorageFile
        || pending_playback_request_id_ != 0
        || active_playback_request_id_ != 0
        || playback_execution_active_
        || !build_main_json_accepted_response(
          result.request_id,
          command,
          maximum_payload_bytes,
          output_,
          sizeof(output_)
        )
      ) {
        return fault_process(JsonMainControllerFault::kInvalidCommandSource);
      }
      playback_parameters_ = *parameters;
      pending_playback_request_id_ = result.request_id;
      return finish_response(
        JsonMainControllerResponseKind::kDirectAccepted,
        maximum_payload_bytes
      );
    }
    const JsonMainDirectResponseStatus response_status = source->execute(
      command,
      *parameters,
      result.request_id,
      maximum_payload_bytes,
      output_,
      sizeof(output_),
      source->context
    );
    JsonMainControllerResponseKind response_kind =
      JsonMainControllerResponseKind::kNone;
    switch (response_status) {
      case JsonMainDirectResponseStatus::kCompleted:
        response_kind = JsonMainControllerResponseKind::kDirectCompleted;
        break;
      case JsonMainDirectResponseStatus::kRejected:
        response_kind = JsonMainControllerResponseKind::kDirectRejected;
        break;
      case JsonMainDirectResponseStatus::kCancelled:
        response_kind = JsonMainControllerResponseKind::kDirectCancelled;
        break;
      case JsonMainDirectResponseStatus::kFailed:
        response_kind = JsonMainControllerResponseKind::kDirectFailed;
        break;
      case JsonMainDirectResponseStatus::kInvalid:
        return fault_process(JsonMainControllerFault::kInvalidCommandSource);
    }
    if (output_[0] == '\0') {
      return fault_process(
        JsonMainControllerFault::kResponseSerializationFailure
      );
    }
    return finish_response(
      response_kind,
      maximum_payload_bytes
    );
  }

  JsonMainControllerProcessStatus finish_response(
    JsonMainControllerResponseKind response_kind,
    size_t maximum_payload_bytes
  ) {
    size_t payload_length = 0;
    while (
      payload_length < sizeof(output_)
      && output_[payload_length] != '\0'
    ) {
      const unsigned char value =
        static_cast<unsigned char>(output_[payload_length]);
      if (value < 0x20 || value > 0x7E) {
        return fault_process(
          JsonMainControllerFault::kResponseSerializationFailure
        );
      }
      ++payload_length;
    }
    if (
      response_kind == JsonMainControllerResponseKind::kNone
      || payload_length == 0
      || payload_length > maximum_payload_bytes
      || payload_length + 2 > sizeof(output_)
    ) {
      return fault_process(
        JsonMainControllerFault::kResponseSerializationFailure
      );
    }
    output_[payload_length] = '\n';
    output_[payload_length + 1] = '\0';
    frame_length_ = payload_length + 1;
    response_kind_ = response_kind;
    state_ = JsonMainControllerOwnerState::kResponseReady;
    fault_ = JsonMainControllerFault::kNone;
    return JsonMainControllerProcessStatus::kResponseReady;
  }

  JsonMainControllerProcessStatus fault_process(
    JsonMainControllerFault fault
  ) {
    fault_owner(fault);
    return JsonMainControllerProcessStatus::kControllerFault;
  }

  void fault_owner(JsonMainControllerFault fault) {
    clear_frame();
    pending_live_motion_id_ = 0;
    pending_live_kind_ = JsonLiveJogKind::kInvalid;
    active_live_motion_id_ = 0;
    active_live_kind_ = JsonLiveJogKind::kInvalid;
    pending_playback_request_id_ = 0;
    active_playback_request_id_ = 0;
    playback_execution_active_ = false;
    playback_parameters_ = {};
    fault_ = fault;
    state_ = JsonMainControllerOwnerState::kFaulted;
  }

  void clear_frame() {
    output_[0] = '\0';
    frame_length_ = 0;
    response_kind_ = JsonMainControllerResponseKind::kNone;
  }

  JsonMainRequestParserWorkspace parser_workspace_;
  size_t semantic_allocation_limit_;
  JsonMainControllerOwnerState state_;
  JsonMainControllerFault fault_;
  JsonMainControllerResponseKind response_kind_;
  bool session_bound_;
  bool configuration_sync_required_;
  uint32_t pending_live_motion_id_;
  JsonLiveJogKind pending_live_kind_;
  uint32_t active_live_motion_id_;
  JsonLiveJogKind active_live_kind_;
  uint32_t pending_playback_request_id_;
  uint32_t active_playback_request_id_;
  bool playback_execution_active_;
  JsonMainDirectParameters playback_parameters_;
  size_t frame_length_;
  char output_[kJsonMainControllerOutputCapacity];
};

}  // namespace ar4_protocol

#endif
