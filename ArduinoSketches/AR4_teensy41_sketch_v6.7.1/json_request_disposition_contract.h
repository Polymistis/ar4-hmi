#ifndef AR4_JSON_REQUEST_DISPOSITION_CONTRACT_H
#define AR4_JSON_REQUEST_DISPOSITION_CONTRACT_H

#include "json_request_contract.h"
#include "json_response_contract.h"

namespace ar4_protocol {

enum class JsonMainRequestDispositionKind {
  kDispatchHello,
  kDispatchGetHomeReference,
  kDispatchGetPositionDisposition,
  kDispatchTestLimitSwitches,
  kDispatchSetEncoders,
  kDispatchReadEncoders,
  kDispatchGetMotionTrace,
  kDispatchCorrectPosition,
  kDispatchZeroExternalAxis,
  kDispatchSetPosition,
  kDispatchUpdateParams,
  kDispatchConfigExtAxis,
  kDispatchControllerWait,
  kDispatchModbusRead,
  kDispatchModbusWrite,
  kDispatchCalibrate,
  kDispatchJogTool,
  kDispatchLiveJointJog,
  kDispatchLiveCartJog,
  kDispatchLiveToolJog,
  kDispatchMoveCartesian,
  kDispatchMoveJoints,
  kDispatchDirect,
  kDispatchStop,
  kDispatchRenewLiveMotion,
  kCorrelatedRejection,
  kProtocolError,
  kControllerFault,
};

struct JsonMainRequestDisposition {
  JsonMainRequestDispositionKind kind;
  const char *error_code;
  const char *message;
  const char *detail_field;
  const char *detail_value;
};

enum class JsonMainRequestParseResponseStatus {
  kDispatchHello,
  kDispatchGetHomeReference,
  kDispatchGetPositionDisposition,
  kDispatchTestLimitSwitches,
  kDispatchSetEncoders,
  kDispatchReadEncoders,
  kDispatchGetMotionTrace,
  kDispatchCorrectPosition,
  kDispatchZeroExternalAxis,
  kDispatchSetPosition,
  kDispatchUpdateParams,
  kDispatchConfigExtAxis,
  kDispatchControllerWait,
  kDispatchModbusRead,
  kDispatchModbusWrite,
  kDispatchCalibrate,
  kDispatchJogTool,
  kDispatchLiveJointJog,
  kDispatchLiveCartJog,
  kDispatchLiveToolJog,
  kDispatchMoveCartesian,
  kDispatchMoveJoints,
  kDispatchDirect,
  kDispatchStop,
  kDispatchRenewLiveMotion,
  kCorrelatedRejectionBuilt,
  kProtocolErrorBuilt,
  kControllerFault,
  kSerializationFailure,
};

inline JsonMainRequestDisposition json_main_controller_fault_disposition() {
  const JsonMainRequestDisposition disposition = {
    JsonMainRequestDispositionKind::kControllerFault,
    nullptr,
    nullptr,
    nullptr,
    nullptr,
  };
  return disposition;
}

inline bool json_main_request_parse_result_shape_valid(
  const JsonMainRequestParseResult &result
) {
  const bool correlation_shape_valid = result.correlation_valid
    ? result.request_id != 0 && json_protocol_name_valid(result.command)
    : result.request_id == 0 && result.command[0] == '\0';
  if (!correlation_shape_valid) return false;

  if (result.status == JsonMainRequestParseStatus::kReady) {
    if (!result.correlation_valid) return false;
    const JsonMainRequestCommand command_kind =
      result.payload.command_kind();
    const char *expected_command =
      json_main_request_command_name(command_kind);
    if (
      expected_command == nullptr
      || strcmp(result.command, expected_command) != 0
    ) return false;
    switch (command_kind) {
      case JsonMainRequestCommand::kLiveJointJog:
      case JsonMainRequestCommand::kLiveCartJog:
      case JsonMainRequestCommand::kLiveToolJog: {
        const JsonMainLiveJogParameters *parameters =
          result.payload.live_jog();
        if (parameters == nullptr) return false;
        return json_live_jog_detail::parameters_valid(
          *parameters
        )
          && parameters->kind
            == (
              command_kind == JsonMainRequestCommand::kLiveJointJog
                ? JsonLiveJogKind::kJoint
                : command_kind
                    == JsonMainRequestCommand::kLiveCartJog
                  ? JsonLiveJogKind::kCartesian
                  : JsonLiveJogKind::kTool
            );
      }
      case JsonMainRequestCommand::kStop: {
        const JsonMainStopParameters *parameters = result.payload.stop();
        return parameters != nullptr && parameters->motion_id != 0;
      }
      case JsonMainRequestCommand::kRenewLiveMotion: {
        const JsonMainRenewLiveMotionParameters *parameters =
          result.payload.renew_live_motion();
        return parameters != nullptr && parameters->motion_id != 0;
      }
      case JsonMainRequestCommand::kSetPosition:
        return result.payload.set_position() != nullptr;
      case JsonMainRequestCommand::kUpdateParams:
        return result.payload.update_params() != nullptr;
      case JsonMainRequestCommand::kConfigExtAxis:
        return result.payload.config_ext_axis() != nullptr;
      case JsonMainRequestCommand::kControllerWait: {
        const JsonMainControllerWaitParameters *parameters =
          result.payload.controller_wait();
        return parameters != nullptr
          && parameters->duration_milliseconds
            <= static_cast<uint32_t>(kMainFirmwareWaitMaxSeconds) * 1000U;
      }
      case JsonMainRequestCommand::kModbusReadHoldingRegister:
      case JsonMainRequestCommand::kModbusReadCoil:
      case JsonMainRequestCommand::kModbusReadDiscreteInput:
      case JsonMainRequestCommand::kModbusReadInputRegister: {
        const JsonMainModbusReadParameters *parameters =
          result.payload.modbus_read();
        return parameters != nullptr
          && json_main_modbus_read_parameters_valid(
            command_kind,
            *parameters
          );
      }
      case JsonMainRequestCommand::kModbusWriteCoil:
      case JsonMainRequestCommand::kModbusWriteRegister: {
        const JsonMainModbusWriteParameters *parameters =
          result.payload.modbus_write();
        return parameters != nullptr
          && json_main_modbus_write_parameters_valid(
            command_kind,
            *parameters
          );
      }
      case JsonMainRequestCommand::kCalibrate:
        return result.payload.calibration() != nullptr;
      case JsonMainRequestCommand::kJogTool:
        return result.payload.jog_tool() != nullptr;
      case JsonMainRequestCommand::kMoveCartesian:
        return result.payload.move_cartesian() != nullptr;
      case JsonMainRequestCommand::kMoveJoints:
        return result.payload.move_joints() != nullptr;
      case JsonMainRequestCommand::kGetMotionTrace:
        return result.payload.motion_trace() != nullptr;
      case JsonMainRequestCommand::kMoveLinear:
      case JsonMainRequestCommand::kMoveVision:
      case JsonMainRequestCommand::kWaitModbusCoil:
      case JsonMainRequestCommand::kWaitModbusDiscreteInput:
      case JsonMainRequestCommand::kDeleteSdProgram:
      case JsonMainRequestCommand::kListSdPrograms:
      case JsonMainRequestCommand::kWriteGcodeMove:
      case JsonMainRequestCommand::kPlayGcodeFile:
      case JsonMainRequestCommand::kWaitModbusHoldingRegister:
      case JsonMainRequestCommand::kMoveArc:
      case JsonMainRequestCommand::kMoveCircle:
      case JsonMainRequestCommand::kMoveSpline:
        return result.payload.direct() != nullptr;
      case JsonMainRequestCommand::kHello:
      case JsonMainRequestCommand::kGetHomeReference:
      case JsonMainRequestCommand::kGetPositionDisposition:
      case JsonMainRequestCommand::kTestLimitSwitches:
      case JsonMainRequestCommand::kSetEncoders:
      case JsonMainRequestCommand::kReadEncoders:
      case JsonMainRequestCommand::kCorrectPosition:
      case JsonMainRequestCommand::kZeroJ7:
      case JsonMainRequestCommand::kZeroJ8:
      case JsonMainRequestCommand::kZeroJ9:
        return true;
      case JsonMainRequestCommand::kUnknown:
        return false;
    }
    return false;
  }
  if (result.payload.command_kind() != JsonMainRequestCommand::kUnknown) {
    return false;
  }

  switch (result.status) {
    case JsonMainRequestParseStatus::kInvalidArgument:
    case JsonMainRequestParseStatus::kInvalidPayload:
    case JsonMainRequestParseStatus::kInvalidRequestIdentifier:
    case JsonMainRequestParseStatus::kInvalidCommandName:
      return !result.correlation_valid;
    case JsonMainRequestParseStatus::kUnknownCommand:
      return result.correlation_valid
        && json_main_request_command_from_name(result.command)
          == JsonMainRequestCommand::kUnknown;
    case JsonMainRequestParseStatus::kReady:
      return false;
    case JsonMainRequestParseStatus::kMalformedJson:
    case JsonMainRequestParseStatus::kParserResourceExhausted:
    case JsonMainRequestParseStatus::kDuplicateField:
    case JsonMainRequestParseStatus::kNestingLimitExceeded:
    case JsonMainRequestParseStatus::kContainerLimitExceeded:
    case JsonMainRequestParseStatus::kInvalidFieldName:
    case JsonMainRequestParseStatus::kInvalidStringValue:
    case JsonMainRequestParseStatus::kInvalidNumber:
    case JsonMainRequestParseStatus::kInvalidEnvelope:
    case JsonMainRequestParseStatus::kUnsupportedVersion:
    case JsonMainRequestParseStatus::kUnsupportedMessageType:
    case JsonMainRequestParseStatus::kInvalidParameters:
    case JsonMainRequestParseStatus::kAllocatorStateInvalid:
      return true;
  }
  return false;
}

inline JsonMainRequestDisposition classify_main_json_request_disposition(
  const JsonMainRequestParseResult &result
) {
  if (!json_main_request_parse_result_shape_valid(result)) {
    return json_main_controller_fault_disposition();
  }

  if (result.status == JsonMainRequestParseStatus::kReady) {
    const JsonMainRequestCommand command_kind =
      result.payload.command_kind();
    switch (command_kind) {
      case JsonMainRequestCommand::kHello: {
        const JsonMainRequestDisposition disposition = {
          JsonMainRequestDispositionKind::kDispatchHello,
          nullptr,
          nullptr,
          nullptr,
          nullptr,
        };
        return disposition;
      }
      case JsonMainRequestCommand::kGetHomeReference: {
        const JsonMainRequestDisposition disposition = {
          JsonMainRequestDispositionKind::kDispatchGetHomeReference,
          nullptr,
          nullptr,
          nullptr,
          nullptr,
        };
        return disposition;
      }
      case JsonMainRequestCommand::kGetPositionDisposition: {
        const JsonMainRequestDisposition disposition = {
          JsonMainRequestDispositionKind::kDispatchGetPositionDisposition,
          nullptr,
          nullptr,
          nullptr,
          nullptr,
        };
        return disposition;
      }
      case JsonMainRequestCommand::kGetMotionTrace: {
        const JsonMainRequestDisposition disposition = {
          JsonMainRequestDispositionKind::kDispatchGetMotionTrace,
          nullptr,
          nullptr,
          nullptr,
          nullptr,
        };
        return disposition;
      }
      case JsonMainRequestCommand::kTestLimitSwitches:
      case JsonMainRequestCommand::kSetEncoders:
      case JsonMainRequestCommand::kReadEncoders: {
        const JsonMainRequestDisposition disposition = {
          command_kind == JsonMainRequestCommand::kTestLimitSwitches
            ? JsonMainRequestDispositionKind::kDispatchTestLimitSwitches
            : command_kind == JsonMainRequestCommand::kSetEncoders
              ? JsonMainRequestDispositionKind::kDispatchSetEncoders
              : JsonMainRequestDispositionKind::kDispatchReadEncoders,
          nullptr, nullptr, nullptr, nullptr,
        };
        return disposition;
      }
      case JsonMainRequestCommand::kSetPosition: {
        const JsonMainRequestDisposition disposition = {
          JsonMainRequestDispositionKind::kDispatchSetPosition,
          nullptr,
          nullptr,
          nullptr,
          nullptr,
        };
        return disposition;
      }
      case JsonMainRequestCommand::kCorrectPosition: {
        const JsonMainRequestDisposition disposition = {
          JsonMainRequestDispositionKind::kDispatchCorrectPosition,
          nullptr,
          nullptr,
          nullptr,
          nullptr,
        };
        return disposition;
      }
      case JsonMainRequestCommand::kZeroJ7:
      case JsonMainRequestCommand::kZeroJ8:
      case JsonMainRequestCommand::kZeroJ9: {
        const JsonMainRequestDisposition disposition = {
          JsonMainRequestDispositionKind::kDispatchZeroExternalAxis,
          nullptr, nullptr, nullptr, nullptr,
        };
        return disposition;
      }
      case JsonMainRequestCommand::kUpdateParams: {
        const JsonMainRequestDisposition disposition = {
          JsonMainRequestDispositionKind::kDispatchUpdateParams,
          nullptr,
          nullptr,
          nullptr,
          nullptr,
        };
        return disposition;
      }
      case JsonMainRequestCommand::kConfigExtAxis: {
        const JsonMainRequestDisposition disposition = {
          JsonMainRequestDispositionKind::kDispatchConfigExtAxis,
          nullptr,
          nullptr,
          nullptr,
          nullptr,
        };
        return disposition;
      }
      case JsonMainRequestCommand::kControllerWait: {
        const JsonMainRequestDisposition disposition = {
          JsonMainRequestDispositionKind::kDispatchControllerWait,
          nullptr,
          nullptr,
          nullptr,
          nullptr,
        };
        return disposition;
      }
      case JsonMainRequestCommand::kModbusReadHoldingRegister:
      case JsonMainRequestCommand::kModbusReadCoil:
      case JsonMainRequestCommand::kModbusReadDiscreteInput:
      case JsonMainRequestCommand::kModbusReadInputRegister: {
        const JsonMainRequestDisposition disposition = {
          JsonMainRequestDispositionKind::kDispatchModbusRead,
          nullptr, nullptr, nullptr, nullptr,
        };
        return disposition;
      }
      case JsonMainRequestCommand::kModbusWriteCoil:
      case JsonMainRequestCommand::kModbusWriteRegister: {
        const JsonMainRequestDisposition disposition = {
          JsonMainRequestDispositionKind::kDispatchModbusWrite,
          nullptr, nullptr, nullptr, nullptr,
        };
        return disposition;
      }
      case JsonMainRequestCommand::kCalibrate: {
        const JsonMainRequestDisposition disposition = {
          JsonMainRequestDispositionKind::kDispatchCalibrate,
          nullptr,
          nullptr,
          nullptr,
          nullptr,
        };
        return disposition;
      }
      case JsonMainRequestCommand::kJogTool: {
        const JsonMainRequestDisposition disposition = {
          JsonMainRequestDispositionKind::kDispatchJogTool,
          nullptr,
          nullptr,
          nullptr,
          nullptr,
        };
        return disposition;
      }
      case JsonMainRequestCommand::kLiveJointJog: {
        const JsonMainRequestDisposition disposition = {
          JsonMainRequestDispositionKind::kDispatchLiveJointJog,
          nullptr,
          nullptr,
          nullptr,
          nullptr,
        };
        return disposition;
      }
      case JsonMainRequestCommand::kLiveCartJog: {
        const JsonMainRequestDisposition disposition = {
          JsonMainRequestDispositionKind::kDispatchLiveCartJog,
          nullptr,
          nullptr,
          nullptr,
          nullptr,
        };
        return disposition;
      }
      case JsonMainRequestCommand::kLiveToolJog: {
        const JsonMainRequestDisposition disposition = {
          JsonMainRequestDispositionKind::kDispatchLiveToolJog,
          nullptr,
          nullptr,
          nullptr,
          nullptr,
        };
        return disposition;
      }
      case JsonMainRequestCommand::kMoveCartesian: {
        const JsonMainRequestDisposition disposition = {
          JsonMainRequestDispositionKind::kDispatchMoveCartesian,
          nullptr,
          nullptr,
          nullptr,
          nullptr,
        };
        return disposition;
      }
      case JsonMainRequestCommand::kMoveJoints: {
        const JsonMainRequestDisposition disposition = {
          JsonMainRequestDispositionKind::kDispatchMoveJoints,
          nullptr,
          nullptr,
          nullptr,
          nullptr,
        };
        return disposition;
      }
      case JsonMainRequestCommand::kMoveLinear:
      case JsonMainRequestCommand::kMoveVision:
      case JsonMainRequestCommand::kWaitModbusCoil:
      case JsonMainRequestCommand::kWaitModbusDiscreteInput:
      case JsonMainRequestCommand::kDeleteSdProgram:
      case JsonMainRequestCommand::kListSdPrograms:
      case JsonMainRequestCommand::kWriteGcodeMove:
      case JsonMainRequestCommand::kPlayGcodeFile:
      case JsonMainRequestCommand::kWaitModbusHoldingRegister:
      case JsonMainRequestCommand::kMoveArc:
      case JsonMainRequestCommand::kMoveCircle:
      case JsonMainRequestCommand::kMoveSpline: {
        const JsonMainRequestDisposition disposition = {
          JsonMainRequestDispositionKind::kDispatchDirect,
          nullptr, nullptr, nullptr, nullptr,
        };
        return disposition;
      }
      case JsonMainRequestCommand::kStop: {
        const JsonMainRequestDisposition disposition = {
          JsonMainRequestDispositionKind::kDispatchStop,
          nullptr,
          nullptr,
          nullptr,
          nullptr,
        };
        return disposition;
      }
      case JsonMainRequestCommand::kRenewLiveMotion: {
        const JsonMainRequestDisposition disposition = {
          JsonMainRequestDispositionKind::kDispatchRenewLiveMotion,
          nullptr,
          nullptr,
          nullptr,
          nullptr,
        };
        return disposition;
      }
      case JsonMainRequestCommand::kUnknown:
        return json_main_controller_fault_disposition();
    }
    return json_main_controller_fault_disposition();
  }
  if (
    result.status == JsonMainRequestParseStatus::kInvalidArgument
    || result.status == JsonMainRequestParseStatus::kAllocatorStateInvalid
  ) {
    return json_main_controller_fault_disposition();
  }

  const char *error_code = nullptr;
  const char *message = nullptr;
  const char *detail_field = nullptr;
  const char *detail_value = nullptr;
  switch (result.status) {
    case JsonMainRequestParseStatus::kInvalidPayload:
      error_code = "malformed_frame";
      message = "request payload is invalid";
      break;
    case JsonMainRequestParseStatus::kMalformedJson:
      error_code = "malformed_frame";
      message = "request JSON is malformed";
      break;
    case JsonMainRequestParseStatus::kParserResourceExhausted:
      error_code = "parser_resource_exhausted";
      message = "request exceeded parser resources";
      break;
    case JsonMainRequestParseStatus::kDuplicateField:
      error_code = "duplicate_field";
      message = "request contains a duplicate field";
      break;
    case JsonMainRequestParseStatus::kNestingLimitExceeded:
      error_code = "nesting_limit_exceeded";
      message = "request nesting exceeds protocol limits";
      break;
    case JsonMainRequestParseStatus::kContainerLimitExceeded:
      error_code = "container_limit_exceeded";
      message = "request container exceeds protocol limits";
      break;
    case JsonMainRequestParseStatus::kInvalidFieldName:
      error_code = "invalid_field_name";
      message = "request field name is invalid";
      break;
    case JsonMainRequestParseStatus::kInvalidStringValue:
      error_code = "invalid_string_value";
      message = "request string value is invalid";
      break;
    case JsonMainRequestParseStatus::kInvalidNumber:
      error_code = "invalid_number";
      message = "request numeric value is invalid";
      break;
    case JsonMainRequestParseStatus::kInvalidEnvelope:
      error_code = "invalid_envelope";
      message = "request envelope is invalid";
      break;
    case JsonMainRequestParseStatus::kUnsupportedVersion:
      error_code = "unsupported_version";
      message = "request protocol version is unsupported";
      detail_field = "field";
      detail_value = "v";
      break;
    case JsonMainRequestParseStatus::kUnsupportedMessageType:
      error_code = "unsupported_message_type";
      message = "request message type is unsupported";
      detail_field = "field";
      detail_value = "type";
      break;
    case JsonMainRequestParseStatus::kInvalidRequestIdentifier:
      error_code = "invalid_request_identifier";
      message = "request identifier is invalid";
      detail_field = "field";
      detail_value = "id";
      break;
    case JsonMainRequestParseStatus::kInvalidCommandName:
      error_code = "invalid_command_name";
      message = "request command name is invalid";
      detail_field = "field";
      detail_value = "cmd";
      break;
    case JsonMainRequestParseStatus::kUnknownCommand:
      error_code = "unknown_command";
      message = "request command is unsupported";
      detail_field = "field";
      detail_value = "cmd";
      break;
    case JsonMainRequestParseStatus::kInvalidParameters:
      error_code = "invalid_parameter";
      message = "request parameters are invalid";
      detail_field = "field";
      detail_value = "params";
      break;
    case JsonMainRequestParseStatus::kReady:
    case JsonMainRequestParseStatus::kInvalidArgument:
    case JsonMainRequestParseStatus::kAllocatorStateInvalid:
      return json_main_controller_fault_disposition();
  }
  if (error_code == nullptr || message == nullptr) {
    return json_main_controller_fault_disposition();
  }

  const JsonMainRequestDisposition disposition = {
    result.correlation_valid
      ? JsonMainRequestDispositionKind::kCorrelatedRejection
      : JsonMainRequestDispositionKind::kProtocolError,
    error_code,
    message,
    detail_field,
    detail_value,
  };
  return disposition;
}

// Parse-result storage and serializer output must not overlap.
inline JsonMainRequestParseResponseStatus build_main_json_parse_response(
  const JsonMainRequestParseResult &result,
  size_t maximum_payload_bytes,
  char *output,
  size_t output_capacity
) {
  if (output != nullptr && output_capacity > 0) output[0] = '\0';
  const JsonMainRequestDisposition disposition =
    classify_main_json_request_disposition(result);
  if (disposition.kind == JsonMainRequestDispositionKind::kDispatchHello) {
    return JsonMainRequestParseResponseStatus::kDispatchHello;
  }
  if (
    disposition.kind
      == JsonMainRequestDispositionKind::kDispatchGetHomeReference
  ) {
    return JsonMainRequestParseResponseStatus::kDispatchGetHomeReference;
  }
  if (
    disposition.kind
      == JsonMainRequestDispositionKind::kDispatchGetPositionDisposition
  ) {
    return JsonMainRequestParseResponseStatus::kDispatchGetPositionDisposition;
  }
  if (
    disposition.kind
      == JsonMainRequestDispositionKind::kDispatchTestLimitSwitches
  ) return JsonMainRequestParseResponseStatus::kDispatchTestLimitSwitches;
  if (
    disposition.kind == JsonMainRequestDispositionKind::kDispatchSetEncoders
  ) return JsonMainRequestParseResponseStatus::kDispatchSetEncoders;
  if (
    disposition.kind == JsonMainRequestDispositionKind::kDispatchReadEncoders
  ) return JsonMainRequestParseResponseStatus::kDispatchReadEncoders;
  if (
    disposition.kind
      == JsonMainRequestDispositionKind::kDispatchGetMotionTrace
  ) return JsonMainRequestParseResponseStatus::kDispatchGetMotionTrace;
  if (
    disposition.kind
      == JsonMainRequestDispositionKind::kDispatchSetPosition
  ) {
    return JsonMainRequestParseResponseStatus::kDispatchSetPosition;
  }
  if (
    disposition.kind
      == JsonMainRequestDispositionKind::kDispatchCorrectPosition
  ) {
    return JsonMainRequestParseResponseStatus::kDispatchCorrectPosition;
  }
  if (
    disposition.kind
      == JsonMainRequestDispositionKind::kDispatchZeroExternalAxis
  ) return JsonMainRequestParseResponseStatus::kDispatchZeroExternalAxis;
  if (
    disposition.kind
      == JsonMainRequestDispositionKind::kDispatchUpdateParams
  ) {
    return JsonMainRequestParseResponseStatus::kDispatchUpdateParams;
  }
  if (
    disposition.kind
      == JsonMainRequestDispositionKind::kDispatchConfigExtAxis
  ) {
    return JsonMainRequestParseResponseStatus::kDispatchConfigExtAxis;
  }
  if (
    disposition.kind
      == JsonMainRequestDispositionKind::kDispatchControllerWait
  ) {
    return JsonMainRequestParseResponseStatus::kDispatchControllerWait;
  }
  if (
    disposition.kind == JsonMainRequestDispositionKind::kDispatchModbusRead
  ) {
    return JsonMainRequestParseResponseStatus::kDispatchModbusRead;
  }
  if (
    disposition.kind == JsonMainRequestDispositionKind::kDispatchModbusWrite
  ) {
    return JsonMainRequestParseResponseStatus::kDispatchModbusWrite;
  }
  if (
    disposition.kind
      == JsonMainRequestDispositionKind::kDispatchCalibrate
  ) {
    return JsonMainRequestParseResponseStatus::kDispatchCalibrate;
  }
  if (
    disposition.kind
      == JsonMainRequestDispositionKind::kDispatchJogTool
  ) {
    return JsonMainRequestParseResponseStatus::kDispatchJogTool;
  }
  if (
    disposition.kind
      == JsonMainRequestDispositionKind::kDispatchLiveJointJog
  ) {
    return JsonMainRequestParseResponseStatus::kDispatchLiveJointJog;
  }
  if (
    disposition.kind
      == JsonMainRequestDispositionKind::kDispatchLiveCartJog
  ) {
    return JsonMainRequestParseResponseStatus::kDispatchLiveCartJog;
  }
  if (
    disposition.kind
      == JsonMainRequestDispositionKind::kDispatchLiveToolJog
  ) {
    return JsonMainRequestParseResponseStatus::kDispatchLiveToolJog;
  }
  if (
    disposition.kind
      == JsonMainRequestDispositionKind::kDispatchMoveCartesian
  ) {
    return JsonMainRequestParseResponseStatus::kDispatchMoveCartesian;
  }
  if (
    disposition.kind
      == JsonMainRequestDispositionKind::kDispatchMoveJoints
  ) {
    return JsonMainRequestParseResponseStatus::kDispatchMoveJoints;
  }
  if (disposition.kind == JsonMainRequestDispositionKind::kDispatchDirect) {
    return JsonMainRequestParseResponseStatus::kDispatchDirect;
  }
  if (
    disposition.kind == JsonMainRequestDispositionKind::kDispatchStop
  ) {
    return JsonMainRequestParseResponseStatus::kDispatchStop;
  }
  if (
    disposition.kind
      == JsonMainRequestDispositionKind::kDispatchRenewLiveMotion
  ) {
    return JsonMainRequestParseResponseStatus::kDispatchRenewLiveMotion;
  }
  if (disposition.kind == JsonMainRequestDispositionKind::kControllerFault) {
    return JsonMainRequestParseResponseStatus::kControllerFault;
  }

  const bool detail_present = disposition.detail_field != nullptr;
  if (detail_present != (disposition.detail_value != nullptr)) {
    return JsonMainRequestParseResponseStatus::kControllerFault;
  }
  const JsonStringErrorDetail detail = {
    disposition.detail_field,
    disposition.detail_value,
  };
  const JsonStringErrorDetail *detail_pointer = detail_present
    ? &detail
    : nullptr;
  if (
    disposition.kind
      == JsonMainRequestDispositionKind::kCorrelatedRejection
  ) {
    if (!build_main_json_error_response(
        result.request_id,
        result.command,
        JsonErrorResponseStatus::kRejected,
        disposition.error_code,
        disposition.message,
        detail_pointer,
        maximum_payload_bytes,
        output,
        output_capacity
    )) {
      return JsonMainRequestParseResponseStatus::kSerializationFailure;
    }
    return JsonMainRequestParseResponseStatus::kCorrelatedRejectionBuilt;
  }
  if (disposition.kind == JsonMainRequestDispositionKind::kProtocolError) {
    if (!build_json_protocol_error_response(
        disposition.error_code,
        disposition.message,
        detail_pointer,
        maximum_payload_bytes,
        output,
        output_capacity
    )) {
      return JsonMainRequestParseResponseStatus::kSerializationFailure;
    }
    return JsonMainRequestParseResponseStatus::kProtocolErrorBuilt;
  }
  return JsonMainRequestParseResponseStatus::kControllerFault;
}

}  // namespace ar4_protocol

#endif
