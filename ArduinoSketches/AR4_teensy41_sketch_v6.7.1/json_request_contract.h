#ifndef AR4_JSON_REQUEST_CONTRACT_H
#define AR4_JSON_REQUEST_CONTRACT_H

#if \
  defined(__GNUC__) \
  && !defined(__clang__) \
  && !defined(AR4_TEST_UNSUPPRESSED_ARDUINOJSON)
#pragma GCC diagnostic push
// ArduinoJson 7.4.3 leaves unused nextId_ uninitialized in an empty iterator.
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#endif
#include <ArduinoJson.h>
#if \
  defined(__GNUC__) \
  && !defined(__clang__) \
  && !defined(AR4_TEST_UNSUPPRESSED_ARDUINOJSON)
#pragma GCC diagnostic pop
#endif

#include <float.h>
#include <math.h>
#include <new>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <type_traits>

#include "controller_domain_contract.h"
#include "controller_motion_trace_contract.h"
#include "json_bounded_allocator_contract.h"
#include "json_calibration_contract.h"
#include "json_cartesian_motion_contract.h"
#include "json_controller_configuration_contract.h"
#include "json_direct_command_contract.h"
#include "json_joint_motion_contract.h"
#include "json_live_jog_contract.h"
#include "json_session_contract.h"
#include "json_tool_jog_contract.h"

#if \
  ARDUINOJSON_VERSION_MAJOR != 7 \
  || ARDUINOJSON_VERSION_MINOR != 4 \
  || ARDUINOJSON_VERSION_REVISION != 3
#error "AR4 JSON parsing requires ArduinoJson 7.4.3"
#endif

namespace ar4_protocol {

constexpr size_t kJsonProtocolMaximumDepth = 8;
constexpr size_t kJsonProtocolMaximumContainerEntries = 128;
constexpr size_t kJsonRequestSemanticStorageBytes = 48U * 1024U;
constexpr size_t kJsonRequestEnvelopeFieldCount = 5;
constexpr size_t kJsonRequestMaximumTrackedKeys =
  kJsonProtocolMaximumPayloadBytes / 4 + 1;
constexpr size_t kJsonRequestMaximumDelimiterDepth =
  kJsonProtocolMaximumPayloadBytes / 2 + 1;
constexpr size_t kJsonArduinoMaximumNumericTokenLength = 63;
// Scanner and ArduinoJson nesting include the root and params containers.
constexpr size_t kJsonArduinoDeserializerNestingLimit =
  kJsonProtocolMaximumDepth + 2;

static_assert(
  kJsonProtocolMaximumPayloadBytes <= UINT16_MAX,
  "JSON parser key offsets require a 16-bit payload range"
);
static_assert(
  kJsonArduinoDeserializerNestingLimit <= UINT8_MAX,
  "ArduinoJson nesting-limit input requires an 8-bit range"
);

enum class JsonMainRequestCommand {
  kHello,
  kGetHomeReference,
  kGetPositionDisposition,
  kTestLimitSwitches,
  kSetEncoders,
  kReadEncoders,
  kGetMotionTrace,
  kCorrectPosition,
  kZeroJ7,
  kZeroJ8,
  kZeroJ9,
  kSetPosition,
  kUpdateParams,
  kConfigExtAxis,
  kControllerWait,
  kModbusReadHoldingRegister,
  kModbusReadCoil,
  kModbusReadDiscreteInput,
  kModbusReadInputRegister,
  kModbusWriteCoil,
  kModbusWriteRegister,
  kCalibrate,
  kJogTool,
  kLiveJointJog,
  kLiveCartJog,
  kLiveToolJog,
  kMoveCartesian,
  kMoveJoints,
  kMoveLinear,
  kMoveVision,
  kWaitModbusCoil,
  kWaitModbusDiscreteInput,
  kDeleteSdProgram,
  kListSdPrograms,
  kWriteGcodeMove,
  kPlayGcodeFile,
  kWaitModbusHoldingRegister,
  kMoveArc,
  kMoveCircle,
  kMoveSpline,
  kStop,
  kRenewLiveMotion,
  kUnknown,
};

inline const char *json_main_request_command_name(
  JsonMainRequestCommand command
) {
  switch (command) {
    case JsonMainRequestCommand::kHello:
      return "hello";
    case JsonMainRequestCommand::kGetHomeReference:
      return "get_home_reference";
    case JsonMainRequestCommand::kGetPositionDisposition:
      return "get_position_disposition";
    case JsonMainRequestCommand::kTestLimitSwitches:
      return "test_limit_switches";
    case JsonMainRequestCommand::kSetEncoders:
      return "set_encoders";
    case JsonMainRequestCommand::kReadEncoders:
      return "read_encoders";
    case JsonMainRequestCommand::kGetMotionTrace:
      return "get_motion_trace";
    case JsonMainRequestCommand::kCorrectPosition:
      return "correct_position";
    case JsonMainRequestCommand::kZeroJ7:
      return "zero_j7";
    case JsonMainRequestCommand::kZeroJ8:
      return "zero_j8";
    case JsonMainRequestCommand::kZeroJ9:
      return "zero_j9";
    case JsonMainRequestCommand::kSetPosition:
      return "set_position";
    case JsonMainRequestCommand::kUpdateParams:
      return "update_params";
    case JsonMainRequestCommand::kConfigExtAxis:
      return "config_ext_axis";
    case JsonMainRequestCommand::kControllerWait:
      return "controller_wait";
    case JsonMainRequestCommand::kModbusReadHoldingRegister:
      return "modbus_read_holding_register";
    case JsonMainRequestCommand::kModbusReadCoil:
      return "modbus_read_coil";
    case JsonMainRequestCommand::kModbusReadDiscreteInput:
      return "modbus_read_discrete_input";
    case JsonMainRequestCommand::kModbusReadInputRegister:
      return "modbus_read_input_register";
    case JsonMainRequestCommand::kModbusWriteCoil:
      return "modbus_write_coil";
    case JsonMainRequestCommand::kModbusWriteRegister:
      return "modbus_write_register";
    case JsonMainRequestCommand::kCalibrate:
      return "calibrate";
    case JsonMainRequestCommand::kJogTool:
      return "jog_tool";
    case JsonMainRequestCommand::kLiveJointJog:
      return "live_joint_jog";
    case JsonMainRequestCommand::kLiveCartJog:
      return "live_cart_jog";
    case JsonMainRequestCommand::kLiveToolJog:
      return "live_tool_jog";
    case JsonMainRequestCommand::kMoveCartesian:
      return "move_cartesian";
    case JsonMainRequestCommand::kMoveJoints:
      return "move_joints";
    case JsonMainRequestCommand::kMoveLinear:
      return "move_linear";
    case JsonMainRequestCommand::kMoveVision:
      return "move_vision";
    case JsonMainRequestCommand::kWaitModbusCoil:
      return "wait_modbus_coil";
    case JsonMainRequestCommand::kWaitModbusDiscreteInput:
      return "wait_modbus_discrete_input";
    case JsonMainRequestCommand::kDeleteSdProgram:
      return "delete_sd_program";
    case JsonMainRequestCommand::kListSdPrograms:
      return "list_sd_programs";
    case JsonMainRequestCommand::kWriteGcodeMove:
      return "write_gcode_move";
    case JsonMainRequestCommand::kPlayGcodeFile:
      return "play_gcode_file";
    case JsonMainRequestCommand::kWaitModbusHoldingRegister:
      return "wait_modbus_holding_register";
    case JsonMainRequestCommand::kMoveArc:
      return "move_arc";
    case JsonMainRequestCommand::kMoveCircle:
      return "move_circle";
    case JsonMainRequestCommand::kMoveSpline:
      return "move_spline";
    case JsonMainRequestCommand::kStop:
      return "stop";
    case JsonMainRequestCommand::kRenewLiveMotion:
      return "renew_live_motion";
    case JsonMainRequestCommand::kUnknown:
      return nullptr;
  }
  return nullptr;
}

inline JsonMainRequestCommand json_main_request_command_from_name(
  const char *command
) {
  if (command == nullptr) return JsonMainRequestCommand::kUnknown;
  if (strcmp(command, "hello") == 0) {
    return JsonMainRequestCommand::kHello;
  }
  if (strcmp(command, "get_home_reference") == 0) {
    return JsonMainRequestCommand::kGetHomeReference;
  }
  if (strcmp(command, "get_position_disposition") == 0) {
    return JsonMainRequestCommand::kGetPositionDisposition;
  }
  if (strcmp(command, "test_limit_switches") == 0) {
    return JsonMainRequestCommand::kTestLimitSwitches;
  }
  if (strcmp(command, "set_encoders") == 0) {
    return JsonMainRequestCommand::kSetEncoders;
  }
  if (strcmp(command, "read_encoders") == 0) {
    return JsonMainRequestCommand::kReadEncoders;
  }
  if (strcmp(command, "get_motion_trace") == 0) {
    return JsonMainRequestCommand::kGetMotionTrace;
  }
  if (strcmp(command, "correct_position") == 0) {
    return JsonMainRequestCommand::kCorrectPosition;
  }
  if (strcmp(command, "zero_j7") == 0) return JsonMainRequestCommand::kZeroJ7;
  if (strcmp(command, "zero_j8") == 0) return JsonMainRequestCommand::kZeroJ8;
  if (strcmp(command, "zero_j9") == 0) return JsonMainRequestCommand::kZeroJ9;
  if (strcmp(command, "set_position") == 0) {
    return JsonMainRequestCommand::kSetPosition;
  }
  if (strcmp(command, "update_params") == 0) {
    return JsonMainRequestCommand::kUpdateParams;
  }
  if (strcmp(command, "config_ext_axis") == 0) {
    return JsonMainRequestCommand::kConfigExtAxis;
  }
  if (strcmp(command, "controller_wait") == 0) {
    return JsonMainRequestCommand::kControllerWait;
  }
  if (strcmp(command, "modbus_read_holding_register") == 0)
    return JsonMainRequestCommand::kModbusReadHoldingRegister;
  if (strcmp(command, "modbus_read_coil") == 0)
    return JsonMainRequestCommand::kModbusReadCoil;
  if (strcmp(command, "modbus_read_discrete_input") == 0)
    return JsonMainRequestCommand::kModbusReadDiscreteInput;
  if (strcmp(command, "modbus_read_input_register") == 0)
    return JsonMainRequestCommand::kModbusReadInputRegister;
  if (strcmp(command, "modbus_write_coil") == 0)
    return JsonMainRequestCommand::kModbusWriteCoil;
  if (strcmp(command, "modbus_write_register") == 0)
    return JsonMainRequestCommand::kModbusWriteRegister;
  if (strcmp(command, "calibrate") == 0) {
    return JsonMainRequestCommand::kCalibrate;
  }
  if (strcmp(command, "jog_tool") == 0) {
    return JsonMainRequestCommand::kJogTool;
  }
  if (strcmp(command, "live_joint_jog") == 0) {
    return JsonMainRequestCommand::kLiveJointJog;
  }
  if (strcmp(command, "live_cart_jog") == 0) {
    return JsonMainRequestCommand::kLiveCartJog;
  }
  if (strcmp(command, "live_tool_jog") == 0) {
    return JsonMainRequestCommand::kLiveToolJog;
  }
  if (strcmp(command, "move_cartesian") == 0) {
    return JsonMainRequestCommand::kMoveCartesian;
  }
  if (strcmp(command, "move_joints") == 0) {
    return JsonMainRequestCommand::kMoveJoints;
  }
  if (strcmp(command, "move_linear") == 0)
    return JsonMainRequestCommand::kMoveLinear;
  if (strcmp(command, "move_vision") == 0)
    return JsonMainRequestCommand::kMoveVision;
  if (strcmp(command, "wait_modbus_coil") == 0)
    return JsonMainRequestCommand::kWaitModbusCoil;
  if (strcmp(command, "wait_modbus_discrete_input") == 0)
    return JsonMainRequestCommand::kWaitModbusDiscreteInput;
  if (strcmp(command, "delete_sd_program") == 0)
    return JsonMainRequestCommand::kDeleteSdProgram;
  if (strcmp(command, "list_sd_programs") == 0)
    return JsonMainRequestCommand::kListSdPrograms;
  if (strcmp(command, "write_gcode_move") == 0)
    return JsonMainRequestCommand::kWriteGcodeMove;
  if (strcmp(command, "play_gcode_file") == 0)
    return JsonMainRequestCommand::kPlayGcodeFile;
  if (strcmp(command, "wait_modbus_holding_register") == 0)
    return JsonMainRequestCommand::kWaitModbusHoldingRegister;
  if (strcmp(command, "move_arc") == 0)
    return JsonMainRequestCommand::kMoveArc;
  if (strcmp(command, "move_circle") == 0)
    return JsonMainRequestCommand::kMoveCircle;
  if (strcmp(command, "move_spline") == 0)
    return JsonMainRequestCommand::kMoveSpline;
  if (strcmp(command, "stop") == 0) {
    return JsonMainRequestCommand::kStop;
  }
  if (strcmp(command, "renew_live_motion") == 0) {
    return JsonMainRequestCommand::kRenewLiveMotion;
  }
  return JsonMainRequestCommand::kUnknown;
}

inline bool json_main_modbus_read_operation(
  JsonMainRequestCommand command, ModbusOperation &operation
) {
  if (command == JsonMainRequestCommand::kModbusReadHoldingRegister)
    operation = ModbusOperation::kReadHoldingRegisters;
  else if (command == JsonMainRequestCommand::kModbusReadCoil)
    operation = ModbusOperation::kReadCoil;
  else if (command == JsonMainRequestCommand::kModbusReadDiscreteInput)
    operation = ModbusOperation::kReadDiscreteInput;
  else if (command == JsonMainRequestCommand::kModbusReadInputRegister)
    operation = ModbusOperation::kReadInputRegisters;
  else
    return false;
  return true;
}

inline bool json_main_modbus_write_operation(
  JsonMainRequestCommand command, ModbusOperation &operation
) {
  if (command == JsonMainRequestCommand::kModbusWriteCoil)
    operation = ModbusOperation::kWriteCoil;
  else if (command == JsonMainRequestCommand::kModbusWriteRegister)
    operation = ModbusOperation::kWriteRegister;
  else
    return false;
  return true;
}

enum class JsonMainRequestParseStatus {
  kReady,
  kInvalidArgument,
  kInvalidPayload,
  kMalformedJson,
  kParserResourceExhausted,
  kAllocatorStateInvalid,
  kDuplicateField,
  kNestingLimitExceeded,
  kContainerLimitExceeded,
  kInvalidFieldName,
  kInvalidStringValue,
  kInvalidNumber,
  kInvalidEnvelope,
  kUnsupportedVersion,
  kUnsupportedMessageType,
  kInvalidRequestIdentifier,
  kInvalidCommandName,
  kUnknownCommand,
  kInvalidParameters,
};

struct JsonRequestKeyReference {
  uint16_t start;
  uint16_t end;
  uint16_t decoded_length;
  uint16_t hash;
};

using JsonMainRequestSemanticAllocator =
  JsonBoundedAllocator<kJsonRequestSemanticStorageBytes>;

struct JsonMainRequestParserWorkspace {
  // Persistent caller ownership keeps parser storage out of the motion stack.
  JsonRequestKeyReference keys[kJsonRequestMaximumTrackedKeys];
  union {
    char number_buffer[kJsonProtocolMaximumPayloadBytes + 1];
    char delimiter_stack[kJsonRequestMaximumDelimiterDepth];
  } scratch;
  JsonMainRequestSemanticAllocator semantic_allocator;
};

struct JsonMainRequestParseOptions {
  size_t semantic_allocation_limit;
};

constexpr JsonMainRequestParseOptions kJsonMainRequestDefaultParseOptions = {
  kJsonRequestSemanticStorageBytes,
};

struct JsonMainSetPositionParameters {
  int32_t robot_joints_millidegrees[6];
  int32_t external_axes_milliunits[3];
};

struct JsonMainControllerWaitParameters {
  uint32_t duration_milliseconds;
};

struct JsonMainModbusReadParameters {
  int slave_id;
  int address;
  int count;
};

inline bool json_main_modbus_read_parameters_valid(
  JsonMainRequestCommand command, const JsonMainModbusReadParameters &parameters
) {
  ModbusOperation operation = ModbusOperation::kReadCoil;
  return json_main_modbus_read_operation(command, operation)
    && validate_modbus_request(
      operation, parameters.slave_id, parameters.address, parameters.count);
}

struct JsonMainModbusWriteParameters {
  int slave_id;
  int address;
  int value;
};

inline bool json_main_modbus_write_parameters_valid(
  JsonMainRequestCommand command,
  const JsonMainModbusWriteParameters &parameters
) {
  ModbusOperation operation = ModbusOperation::kWriteCoil;
  return json_main_modbus_write_operation(command, operation)
    && validate_modbus_request(
      operation, parameters.slave_id, parameters.address, parameters.value);
}

class JsonMainRequestPayload {
 public:
  JsonMainRequestPayload()
    : command_kind_(JsonMainRequestCommand::kUnknown) {
    storage_.none = 0;
  }

  JsonMainRequestPayload(const JsonMainRequestPayload &) = default;
  JsonMainRequestPayload &operator=(const JsonMainRequestPayload &) = default;

  void reset() {
    storage_.none = 0;
    command_kind_ = JsonMainRequestCommand::kUnknown;
  }

  JsonMainRequestCommand command_kind() const {
    return command_kind_;
  }

  bool assign_empty(JsonMainRequestCommand command_kind) {
    switch (command_kind) {
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
        storage_.none = 0;
        command_kind_ = command_kind;
        return true;
      case JsonMainRequestCommand::kSetPosition:
      case JsonMainRequestCommand::kGetMotionTrace:
      case JsonMainRequestCommand::kUpdateParams:
      case JsonMainRequestCommand::kConfigExtAxis:
      case JsonMainRequestCommand::kControllerWait:
      case JsonMainRequestCommand::kModbusReadHoldingRegister:
      case JsonMainRequestCommand::kModbusReadCoil:
      case JsonMainRequestCommand::kModbusReadDiscreteInput:
      case JsonMainRequestCommand::kModbusReadInputRegister:
      case JsonMainRequestCommand::kModbusWriteCoil:
      case JsonMainRequestCommand::kModbusWriteRegister:
      case JsonMainRequestCommand::kCalibrate:
      case JsonMainRequestCommand::kJogTool:
      case JsonMainRequestCommand::kLiveJointJog:
      case JsonMainRequestCommand::kLiveCartJog:
      case JsonMainRequestCommand::kLiveToolJog:
      case JsonMainRequestCommand::kMoveCartesian:
      case JsonMainRequestCommand::kMoveJoints:
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
      case JsonMainRequestCommand::kStop:
      case JsonMainRequestCommand::kRenewLiveMotion:
      case JsonMainRequestCommand::kUnknown:
        return false;
    }
    return false;
  }

  void assign_set_position(const JsonMainSetPositionParameters &parameters) {
    assign_parameters(&storage_.set_position, parameters);
    command_kind_ = JsonMainRequestCommand::kSetPosition;
  }

  void assign_update_params(const JsonMainUpdateParameters &parameters) {
    assign_parameters(&storage_.update_params, parameters);
    command_kind_ = JsonMainRequestCommand::kUpdateParams;
  }

  void assign_config_ext_axis(
    const JsonMainExternalAxisParameters &parameters
  ) {
    assign_parameters(&storage_.config_ext_axis, parameters);
    command_kind_ = JsonMainRequestCommand::kConfigExtAxis;
  }

  void assign_controller_wait(
    const JsonMainControllerWaitParameters &parameters
  ) {
    assign_parameters(&storage_.controller_wait, parameters);
    command_kind_ = JsonMainRequestCommand::kControllerWait;
  }

  bool assign_modbus_read(
    JsonMainRequestCommand command_kind, const JsonMainModbusReadParameters &parameters
  ) {
    if (!json_main_modbus_read_parameters_valid(command_kind, parameters)) {
      return false;
    }
    assign_parameters(&storage_.modbus_read, parameters);
    command_kind_ = command_kind;
    return true;
  }

  bool assign_modbus_write(
    JsonMainRequestCommand command_kind,
    const JsonMainModbusWriteParameters &parameters
  ) {
    if (!json_main_modbus_write_parameters_valid(command_kind, parameters)) {
      return false;
    }
    assign_parameters(&storage_.modbus_write, parameters);
    command_kind_ = command_kind;
    return true;
  }

  void assign_jog_tool(const JsonMainToolJogParameters &parameters) {
    assign_parameters(&storage_.jog_tool, parameters);
    command_kind_ = JsonMainRequestCommand::kJogTool;
  }

  void assign_calibration(
    const JsonMainCalibrationParameters &parameters
  ) {
    assign_parameters(&storage_.calibration, parameters);
    command_kind_ = JsonMainRequestCommand::kCalibrate;
  }

  bool assign_live_jog(
    JsonMainRequestCommand command_kind,
    const JsonMainLiveJogParameters &parameters
  ) {
    if (
      command_kind != JsonMainRequestCommand::kLiveJointJog
      && command_kind != JsonMainRequestCommand::kLiveCartJog
      && command_kind != JsonMainRequestCommand::kLiveToolJog
    ) {
      return false;
    }
    assign_parameters(&storage_.live_jog, parameters);
    command_kind_ = command_kind;
    return true;
  }

  void assign_move_cartesian(
    const JsonMainMoveCartesianParameters &parameters
  ) {
    assign_parameters(&storage_.move_cartesian, parameters);
    command_kind_ = JsonMainRequestCommand::kMoveCartesian;
  }

  void assign_move_joints(const JsonMainMoveJointsParameters &parameters) {
    assign_parameters(&storage_.move_joints, parameters);
    command_kind_ = JsonMainRequestCommand::kMoveJoints;
  }

  void assign_motion_trace(const JsonMainMotionTraceParameters &parameters) {
    assign_parameters(&storage_.motion_trace, parameters);
    command_kind_ = JsonMainRequestCommand::kGetMotionTrace;
  }

  void assign_direct(
    JsonMainRequestCommand command_kind,
    const JsonMainDirectParameters &parameters
  ) {
    assign_parameters(&storage_.direct, parameters);
    command_kind_ = command_kind;
  }

  void assign_stop(const JsonMainStopParameters &parameters) {
    assign_parameters(&storage_.stop, parameters);
    command_kind_ = JsonMainRequestCommand::kStop;
  }

  void assign_renew_live_motion(
    const JsonMainRenewLiveMotionParameters &parameters
  ) {
    assign_parameters(&storage_.renew_live_motion, parameters);
    command_kind_ = JsonMainRequestCommand::kRenewLiveMotion;
  }

  const JsonMainSetPositionParameters *set_position() const {
    return command_kind_ == JsonMainRequestCommand::kSetPosition
      ? &storage_.set_position : nullptr;
  }

  const JsonMainUpdateParameters *update_params() const {
    return command_kind_ == JsonMainRequestCommand::kUpdateParams
      ? &storage_.update_params : nullptr;
  }

  const JsonMainExternalAxisParameters *config_ext_axis() const {
    return command_kind_ == JsonMainRequestCommand::kConfigExtAxis
      ? &storage_.config_ext_axis : nullptr;
  }

  const JsonMainControllerWaitParameters *controller_wait() const {
    return command_kind_ == JsonMainRequestCommand::kControllerWait
      ? &storage_.controller_wait : nullptr;
  }

  const JsonMainModbusReadParameters *modbus_read() const {
    ModbusOperation operation = ModbusOperation::kReadCoil;
    return json_main_modbus_read_operation(command_kind_, operation)
      ? &storage_.modbus_read : nullptr;
  }

  const JsonMainModbusWriteParameters *modbus_write() const {
    ModbusOperation operation = ModbusOperation::kWriteCoil;
    return json_main_modbus_write_operation(command_kind_, operation)
      ? &storage_.modbus_write : nullptr;
  }

  const JsonMainToolJogParameters *jog_tool() const {
    return command_kind_ == JsonMainRequestCommand::kJogTool
      ? &storage_.jog_tool : nullptr;
  }

  const JsonMainCalibrationParameters *calibration() const {
    return command_kind_ == JsonMainRequestCommand::kCalibrate
      ? &storage_.calibration : nullptr;
  }

  const JsonMainLiveJogParameters *live_jog() const {
    return command_kind_ == JsonMainRequestCommand::kLiveJointJog
        || command_kind_ == JsonMainRequestCommand::kLiveCartJog
        || command_kind_ == JsonMainRequestCommand::kLiveToolJog
      ? &storage_.live_jog : nullptr;
  }

  const JsonMainMoveCartesianParameters *move_cartesian() const {
    return command_kind_ == JsonMainRequestCommand::kMoveCartesian
      ? &storage_.move_cartesian : nullptr;
  }

  const JsonMainMoveJointsParameters *move_joints() const {
    return command_kind_ == JsonMainRequestCommand::kMoveJoints
      ? &storage_.move_joints : nullptr;
  }

  const JsonMainMotionTraceParameters *motion_trace() const {
    return command_kind_ == JsonMainRequestCommand::kGetMotionTrace
      ? &storage_.motion_trace : nullptr;
  }

  const JsonMainDirectParameters *direct() const {
    switch (command_kind_) {
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
        return &storage_.direct;
      default:
        return nullptr;
    }
  }

  const JsonMainStopParameters *stop() const {
    return command_kind_ == JsonMainRequestCommand::kStop
      ? &storage_.stop : nullptr;
  }

  const JsonMainRenewLiveMotionParameters *renew_live_motion() const {
    return command_kind_ == JsonMainRequestCommand::kRenewLiveMotion
      ? &storage_.renew_live_motion : nullptr;
  }

 private:
  union Storage {
    uint8_t none;
    JsonMainSetPositionParameters set_position;
    JsonMainUpdateParameters update_params;
    JsonMainExternalAxisParameters config_ext_axis;
    JsonMainControllerWaitParameters controller_wait;
    JsonMainModbusReadParameters modbus_read;
    JsonMainModbusWriteParameters modbus_write;
    JsonMainCalibrationParameters calibration;
    JsonMainToolJogParameters jog_tool;
    JsonMainLiveJogParameters live_jog;
    JsonMainMoveCartesianParameters move_cartesian;
    JsonMainMoveJointsParameters move_joints;
    JsonMainMotionTraceParameters motion_trace;
    JsonMainDirectParameters direct;
    JsonMainStopParameters stop;
    JsonMainRenewLiveMotionParameters renew_live_motion;
  };

  template <typename Parameters>
  static void assign_parameters(
    Parameters *storage,
    const Parameters &parameters
  ) {
    static_assert(
      std::is_trivial<Parameters>::value,
      "JSON request union members must remain trivial"
    );
    ::new (static_cast<void *>(storage)) Parameters(parameters);
  }

  JsonMainRequestCommand command_kind_;
  Storage storage_;
};

struct JsonMainRequestParseResult {
  JsonMainRequestParseStatus status;
  bool correlation_valid;
  uint32_t request_id;
  char command[kJsonProtocolMaximumNameLength + 1];
  JsonMainRequestPayload payload;
};

namespace json_request_detail {

struct JsonStringToken {
  size_t start;
  size_t end;
  size_t decoded_length;
  uint16_t hash;
  bool printable_ascii;
  bool field_name_valid;
  bool protocol_name_valid;
};

struct JsonSoleNumericParameterObservation {
  bool present;
  double value;
};

inline bool ascii_letter(uint16_t value) {
  return (value >= 'A' && value <= 'Z')
    || (value >= 'a' && value <= 'z');
}

inline bool ascii_lowercase_letter(uint16_t value) {
  return value >= 'a' && value <= 'z';
}

inline bool ascii_digit(uint16_t value) {
  return value >= '0' && value <= '9';
}

inline bool hexadecimal_value(char value, uint16_t &decoded) {
  if (value >= '0' && value <= '9') {
    decoded = static_cast<uint16_t>(value - '0');
    return true;
  }
  if (value >= 'A' && value <= 'F') {
    decoded = static_cast<uint16_t>(value - 'A' + 10);
    return true;
  }
  if (value >= 'a' && value <= 'f') {
    decoded = static_cast<uint16_t>(value - 'a' + 10);
    return true;
  }
  return false;
}

inline bool decode_json_escape(
  const char *payload,
  size_t length,
  size_t &index,
  uint16_t &decoded
) {
  if (index >= length || payload[index] != '\\') return false;
  ++index;
  if (index >= length) return false;
  const char escape = payload[index++];
  switch (escape) {
    case '"': decoded = '"'; return true;
    case '\\': decoded = '\\'; return true;
    case '/': decoded = '/'; return true;
    case 'b': decoded = 0x08; return true;
    case 'f': decoded = 0x0C; return true;
    case 'n': decoded = 0x0A; return true;
    case 'r': decoded = 0x0D; return true;
    case 't': decoded = 0x09; return true;
    case 'u': break;
    default: return false;
  }
  if (length - index < 4) return false;
  uint16_t value = 0;
  for (size_t offset = 0; offset < 4; ++offset) {
    uint16_t digit = 0;
    if (!hexadecimal_value(payload[index + offset], digit)) return false;
    value = static_cast<uint16_t>(value * 16 + digit);
  }
  index += 4;
  decoded = value;
  return true;
}

inline bool scan_json_string(
  const char *payload,
  size_t length,
  size_t &index,
  JsonStringToken &token
) {
  if (index >= length || payload[index] != '"') return false;
  ++index;
  token.start = index;
  token.decoded_length = 0;
  token.hash = 0x811C;
  token.printable_ascii = true;
  token.field_name_valid = true;
  token.protocol_name_valid = true;
  bool closed = false;
  while (index < length) {
    uint16_t decoded = 0;
    if (payload[index] == '"') {
      token.end = index;
      ++index;
      closed = true;
      break;
    }
    if (payload[index] == '\\') {
      if (!decode_json_escape(payload, length, index, decoded)) return false;
    } else {
      decoded = static_cast<unsigned char>(payload[index++]);
    }
    const size_t character_index = token.decoded_length++;
    token.hash = static_cast<uint16_t>(
      (token.hash ^ decoded) * static_cast<uint16_t>(0x0193)
    );
    if (decoded < 0x20 || decoded > 0x7E) {
      token.printable_ascii = false;
    }
    const bool field_character_valid = character_index == 0
      ? ascii_letter(decoded)
      : ascii_letter(decoded) || ascii_digit(decoded) || decoded == '_';
    const bool name_character_valid = character_index == 0
      ? ascii_lowercase_letter(decoded)
      : ascii_lowercase_letter(decoded)
        || ascii_digit(decoded)
        || decoded == '_';
    token.field_name_valid =
      token.field_name_valid && field_character_valid;
    token.protocol_name_valid =
      token.protocol_name_valid && name_character_valid;
  }
  if (!closed) return false;
  if (
    token.decoded_length == 0
    || token.decoded_length > kJsonProtocolMaximumNameLength
  ) {
    token.field_name_valid = false;
    token.protocol_name_valid = false;
  }
  return true;
}

inline bool next_decoded_character(
  const char *payload,
  size_t end,
  size_t &index,
  uint16_t &decoded
) {
  if (index >= end) return false;
  if (payload[index] == '\\') {
    return decode_json_escape(payload, end, index, decoded);
  }
  decoded = static_cast<unsigned char>(payload[index++]);
  return true;
}

inline bool token_equals(
  const char *payload,
  const JsonStringToken &token,
  const char *expected
) {
  if (expected == nullptr) return false;
  size_t input_index = token.start;
  size_t expected_index = 0;
  while (input_index < token.end && expected[expected_index] != '\0') {
    uint16_t decoded = 0;
    if (
      !next_decoded_character(payload, token.end, input_index, decoded)
      || decoded != static_cast<unsigned char>(expected[expected_index])
    ) {
      return false;
    }
    ++expected_index;
  }
  return input_index == token.end && expected[expected_index] == '\0';
}

inline bool tokens_equal(
  const char *payload,
  const JsonStringToken &left,
  const JsonRequestKeyReference &right
) {
  if (
    left.decoded_length != right.decoded_length
    || left.hash != right.hash
  ) {
    return false;
  }
  size_t left_index = left.start;
  size_t right_index = right.start;
  while (left_index < left.end && right_index < right.end) {
    uint16_t left_value = 0;
    uint16_t right_value = 0;
    if (
      !next_decoded_character(payload, left.end, left_index, left_value)
      || !next_decoded_character(
        payload,
        right.end,
        right_index,
        right_value
      )
      || left_value != right_value
    ) {
      return false;
    }
  }
  return left_index == left.end && right_index == right.end;
}

inline bool copy_token(
  const char *payload,
  const JsonStringToken &token,
  char *output,
  size_t output_capacity
) {
  if (
    output == nullptr
    || output_capacity == 0
    || token.decoded_length >= output_capacity
  ) {
    return false;
  }
  size_t input_index = token.start;
  size_t output_index = 0;
  while (input_index < token.end) {
    uint16_t decoded = 0;
    if (
      !next_decoded_character(payload, token.end, input_index, decoded)
      || decoded > 0x7F
    ) {
      output[0] = '\0';
      return false;
    }
    output[output_index++] = static_cast<char>(decoded);
  }
  output[output_index] = '\0';
  return true;
}

inline void skip_spaces(
  const char *payload,
  size_t length,
  size_t &index
) {
  while (index < length && payload[index] == ' ') ++index;
}

inline bool primitive_delimiter(char value) {
  return value == ' ' || value == ',' || value == '}' || value == ']';
}

inline bool skip_correlation_value(
  const char *payload,
  size_t length,
  size_t &index,
  JsonMainRequestParserWorkspace &workspace
) {
  if (index >= length) return false;
  if (payload[index] == '"') {
    JsonStringToken token = {};
    return scan_json_string(payload, length, index, token);
  }
  if (payload[index] != '{' && payload[index] != '[') {
    const size_t start = index;
    while (index < length && !primitive_delimiter(payload[index])) ++index;
    return index > start;
  }

  size_t delimiter_count = 0;
  workspace.scratch.delimiter_stack[delimiter_count++] = payload[index++];
  while (index < length && delimiter_count > 0) {
    if (payload[index] == '"') {
      JsonStringToken token = {};
      if (!scan_json_string(payload, length, index, token)) return false;
      continue;
    }
    const char value = payload[index++];
    if (value == '{' || value == '[') {
      if (delimiter_count >= kJsonRequestMaximumDelimiterDepth) return false;
      workspace.scratch.delimiter_stack[delimiter_count++] = value;
      continue;
    }
    if (value == '}' || value == ']') {
      const char opening = workspace.scratch.delimiter_stack[
        delimiter_count - 1
      ];
      if (
        (opening == '{' && value != '}')
        || (opening == '[' && value != ']')
      ) {
        return false;
      }
      --delimiter_count;
    }
  }
  return delimiter_count == 0;
}

inline bool parse_uint32_token(
  const char *payload,
  size_t start,
  size_t end,
  uint32_t &output
) {
  if (start >= end) return false;
  uint64_t staged = 0;
  if (payload[start] == '0' && end - start != 1) return false;
  for (size_t index = start; index < end; ++index) {
    const char value = payload[index];
    if (value < '0' || value > '9') return false;
    staged = staged * 10 + static_cast<uint64_t>(value - '0');
    if (staged > UINT32_MAX) return false;
  }
  if (staged == 0) return false;
  output = static_cast<uint32_t>(staged);
  return true;
}

inline bool extract_request_correlation(
  const char *payload,
  size_t length,
  JsonMainRequestParserWorkspace &workspace,
  JsonMainRequestParseResult &result
) {
  // Correlation extraction owns only correlation_valid, request_id, and command.
  result.correlation_valid = false;
  result.request_id = 0;
  result.command[0] = '\0';
  size_t index = 0;
  skip_spaces(payload, length, index);
  if (index >= length || payload[index++] != '{') return false;
  skip_spaces(payload, length, index);
  size_t id_count = 0;
  size_t command_count = 0;
  bool id_valid = false;
  bool command_valid = false;
  uint32_t request_id = 0;
  char command[kJsonProtocolMaximumNameLength + 1] = {0};
  if (index < length && payload[index] == '}') {
    ++index;
  } else {
    for (;;) {
      JsonStringToken key = {};
      if (!scan_json_string(payload, length, index, key)) return false;
      const bool is_id = token_equals(payload, key, "id");
      const bool is_command = token_equals(payload, key, "cmd");
      skip_spaces(payload, length, index);
      if (index >= length || payload[index++] != ':') return false;
      skip_spaces(payload, length, index);
      const size_t value_start = index;
      if (!skip_correlation_value(payload, length, index, workspace)) {
        return false;
      }
      const size_t value_end = index;
      if (is_id) {
        ++id_count;
        uint32_t staged = 0;
        if (parse_uint32_token(payload, value_start, value_end, staged)) {
          request_id = staged;
          id_valid = true;
        } else {
          id_valid = false;
        }
      }
      if (is_command) {
        ++command_count;
        size_t string_index = value_start;
        JsonStringToken value = {};
        if (
          scan_json_string(payload, value_end, string_index, value)
          && string_index == value_end
          && value.printable_ascii
          && value.protocol_name_valid
          && copy_token(payload, value, command, sizeof(command))
        ) {
          command_valid = true;
        } else {
          command_valid = false;
          command[0] = '\0';
        }
      }
      skip_spaces(payload, length, index);
      if (index >= length) return false;
      if (payload[index] == '}') {
        ++index;
        break;
      }
      if (payload[index++] != ',') return false;
      skip_spaces(payload, length, index);
    }
  }
  skip_spaces(payload, length, index);
  if (index != length) return false;
  if (
    id_count != 1
    || command_count != 1
    || !id_valid
    || !command_valid
  ) {
    return true;
  }
  result.correlation_valid = true;
  result.request_id = request_id;
  memcpy(result.command, command, sizeof(command));
  return true;
}

inline bool magnitude_within_limit(
  const char *payload,
  size_t start,
  size_t end,
  const char *limit
) {
  const size_t digit_count = end - start;
  const size_t limit_length = strlen(limit);
  if (digit_count != limit_length) return digit_count < limit_length;
  for (size_t index = 0; index < digit_count; ++index) {
    if (payload[start + index] == limit[index]) continue;
    return payload[start + index] < limit[index];
  }
  return true;
}

class StrictJsonScanner {
 public:
  StrictJsonScanner(
    const char *payload,
    size_t length,
    JsonMainRequestParserWorkspace &workspace
  ) :
    payload_(payload),
    length_(length),
    workspace_(workspace),
    index_(0),
    key_count_(0),
    first_error_(JsonMainRequestParseStatus::kReady),
    sole_numeric_parameter_{} {}

  JsonMainRequestParseStatus scan() {
    skip_spaces(payload_, length_, index_);
    if (!parse_value(0, -1)) {
      return JsonMainRequestParseStatus::kMalformedJson;
    }
    skip_spaces(payload_, length_, index_);
    if (index_ != length_) {
      return JsonMainRequestParseStatus::kMalformedJson;
    }
    return first_error_;
  }

  const JsonSoleNumericParameterObservation &sole_numeric_parameter() const {
    return sole_numeric_parameter_;
  }

 private:
  void record(JsonMainRequestParseStatus status) {
    if (first_error_ == JsonMainRequestParseStatus::kReady) {
      first_error_ = status;
    }
  }

  bool parse_value(
    size_t container_depth,
    int domain_depth,
    JsonSoleNumericParameterObservation *number = nullptr
  ) {
    if (number != nullptr) number->present = false;
    if (
      domain_depth > static_cast<int>(kJsonProtocolMaximumDepth)
    ) {
      record(JsonMainRequestParseStatus::kNestingLimitExceeded);
    }
    skip_spaces(payload_, length_, index_);
    if (index_ >= length_) return false;
    switch (payload_[index_]) {
      case '{': return parse_object(container_depth, domain_depth);
      case '[': return parse_array(container_depth, domain_depth);
      case '"': return parse_string_value();
      case 't': return parse_literal("true");
      case 'f': return parse_literal("false");
      case 'n': return parse_literal("null");
      default: return parse_number(number);
    }
  }

  bool parse_string_value() {
    JsonStringToken token = {};
    if (!scan_json_string(payload_, length_, index_, token)) return false;
    if (
      !token.printable_ascii
      || token.decoded_length > kJsonProtocolMaximumStringLength
    ) {
      record(JsonMainRequestParseStatus::kInvalidStringValue);
    }
    return true;
  }

  bool parse_literal(const char *literal) {
    const size_t literal_length = strlen(literal);
    if (
      length_ - index_ < literal_length
      || memcmp(payload_ + index_, literal, literal_length) != 0
    ) {
      return false;
    }
    index_ += literal_length;
    return true;
  }

  bool parse_number(JsonSoleNumericParameterObservation *number) {
    const size_t start = index_;
    if (payload_[index_] == '-') {
      ++index_;
      if (index_ >= length_) return false;
    }
    const size_t integer_start = index_;
    if (payload_[index_] == '0') {
      ++index_;
      if (index_ < length_ && ascii_digit(payload_[index_])) return false;
    } else {
      if (payload_[index_] < '1' || payload_[index_] > '9') return false;
      while (index_ < length_ && ascii_digit(payload_[index_])) ++index_;
    }
    const size_t integer_end = index_;
    bool floating = false;
    bool nonzero_mantissa = false;
    for (size_t digit = integer_start; digit < integer_end; ++digit) {
      nonzero_mantissa = nonzero_mantissa || payload_[digit] != '0';
    }
    if (index_ < length_ && payload_[index_] == '.') {
      floating = true;
      ++index_;
      const size_t fraction_start = index_;
      while (index_ < length_ && ascii_digit(payload_[index_])) {
        nonzero_mantissa = nonzero_mantissa || payload_[index_] != '0';
        ++index_;
      }
      if (index_ == fraction_start) return false;
    }
    if (
      index_ < length_
      && (payload_[index_] == 'e' || payload_[index_] == 'E')
    ) {
      floating = true;
      ++index_;
      if (
        index_ < length_
        && (payload_[index_] == '+' || payload_[index_] == '-')
      ) {
        ++index_;
      }
      const size_t exponent_start = index_;
      while (index_ < length_ && ascii_digit(payload_[index_])) ++index_;
      if (index_ == exponent_start) return false;
    }
    const size_t token_length = index_ - start;
    if (token_length > kJsonArduinoMaximumNumericTokenLength) {
      record(JsonMainRequestParseStatus::kInvalidNumber);
      return true;
    }
    if (!floating) {
      const bool negative = payload_[start] == '-';
      const char *limit = negative
        ? "9223372036854775808"
        : "18446744073709551615";
      if (!magnitude_within_limit(
          payload_,
          integer_start,
          integer_end,
          limit
      )) {
        record(JsonMainRequestParseStatus::kInvalidNumber);
      }
      if (number == nullptr) return true;
    }

    memcpy(workspace_.scratch.number_buffer, payload_ + start, token_length);
    workspace_.scratch.number_buffer[token_length] = '\0';
    char *end = nullptr;
    const double parsed = strtod(workspace_.scratch.number_buffer, &end);
    if (
      end == workspace_.scratch.number_buffer
      || *end != '\0'
      || !isfinite(parsed)
      || (parsed == 0.0 && nonzero_mantissa)
    ) {
      record(JsonMainRequestParseStatus::kInvalidNumber);
    } else if (number != nullptr) {
      number->present = true;
      number->value = parsed;
    }
    return true;
  }

  bool parse_object(size_t container_depth, int domain_depth) {
    if (container_depth >= kJsonArduinoDeserializerNestingLimit) {
      record(JsonMainRequestParseStatus::kNestingLimitExceeded);
      return skip_correlation_value(
        payload_,
        length_,
        index_,
        workspace_
      );
    }
    ++index_;
    skip_spaces(payload_, length_, index_);
    const size_t scope_start = key_count_;
    size_t entry_count = 0;
    if (index_ < length_ && payload_[index_] == '}') {
      ++index_;
      return true;
    }
    for (;;) {
      JsonStringToken key = {};
      if (!scan_json_string(payload_, length_, index_, key)) {
        key_count_ = scope_start;
        return false;
      }
      ++entry_count;
      if (entry_count > kJsonProtocolMaximumContainerEntries) {
        record(JsonMainRequestParseStatus::kContainerLimitExceeded);
      }
      if (!key.printable_ascii || !key.field_name_valid) {
        record(JsonMainRequestParseStatus::kInvalidFieldName);
      }
      for (size_t prior = scope_start; prior < key_count_; ++prior) {
        if (tokens_equal(payload_, key, workspace_.keys[prior])) {
          record(JsonMainRequestParseStatus::kDuplicateField);
          break;
        }
      }
      if (key_count_ >= kJsonRequestMaximumTrackedKeys) {
        record(JsonMainRequestParseStatus::kParserResourceExhausted);
      } else {
        JsonRequestKeyReference &stored = workspace_.keys[key_count_++];
        stored.start = static_cast<uint16_t>(key.start);
        stored.end = static_cast<uint16_t>(key.end);
        stored.decoded_length = static_cast<uint16_t>(key.decoded_length);
        stored.hash = key.hash;
      }
      skip_spaces(payload_, length_, index_);
      if (index_ >= length_ || payload_[index_++] != ':') {
        key_count_ = scope_start;
        return false;
      }
      int child_domain_depth = domain_depth < 0 ? -1 : domain_depth + 1;
      if (
        container_depth == 0
        && token_equals(payload_, key, "params")
      ) {
        child_domain_depth = 0;
      }
      JsonSoleNumericParameterObservation *number = nullptr;
      if (domain_depth == 0) {
        if (entry_count == 1) number = &sole_numeric_parameter_;
        else sole_numeric_parameter_.present = false;
      }
      if (!parse_value(
          container_depth + 1,
          child_domain_depth,
          number
      )) {
        key_count_ = scope_start;
        return false;
      }
      skip_spaces(payload_, length_, index_);
      if (index_ >= length_) {
        key_count_ = scope_start;
        return false;
      }
      if (payload_[index_] == '}') {
        ++index_;
        key_count_ = scope_start;
        return true;
      }
      if (payload_[index_++] != ',') {
        key_count_ = scope_start;
        return false;
      }
      skip_spaces(payload_, length_, index_);
    }
  }

  bool parse_array(size_t container_depth, int domain_depth) {
    if (container_depth >= kJsonArduinoDeserializerNestingLimit) {
      record(JsonMainRequestParseStatus::kNestingLimitExceeded);
      return skip_correlation_value(
        payload_,
        length_,
        index_,
        workspace_
      );
    }
    ++index_;
    skip_spaces(payload_, length_, index_);
    size_t entry_count = 0;
    if (index_ < length_ && payload_[index_] == ']') {
      ++index_;
      return true;
    }
    for (;;) {
      ++entry_count;
      if (entry_count > kJsonProtocolMaximumContainerEntries) {
        record(JsonMainRequestParseStatus::kContainerLimitExceeded);
      }
      const int child_domain_depth =
        domain_depth < 0 ? -1 : domain_depth + 1;
      if (!parse_value(container_depth + 1, child_domain_depth)) return false;
      skip_spaces(payload_, length_, index_);
      if (index_ >= length_) return false;
      if (payload_[index_] == ']') {
        ++index_;
        return true;
      }
      if (payload_[index_++] != ',') return false;
      skip_spaces(payload_, length_, index_);
    }
  }

  const char *payload_;
  size_t length_;
  JsonMainRequestParserWorkspace &workspace_;
  size_t index_;
  size_t key_count_;
  JsonMainRequestParseStatus first_error_;
  JsonSoleNumericParameterObservation sole_numeric_parameter_;
};

inline bool json_string_equals(
  ArduinoJson::JsonString value,
  const char *expected
) {
  if (!value || expected == nullptr) return false;
  const size_t expected_length = strlen(expected);
  return value.size() == expected_length
    && memcmp(value.c_str(), expected, expected_length) == 0;
}

inline bool json_protocol_name_valid(ArduinoJson::JsonString value) {
  if (
    !value
    || value.size() == 0
    || value.size() > kJsonProtocolMaximumNameLength
  ) {
    return false;
  }
  for (size_t index = 0; index < value.size(); ++index) {
    const uint16_t character = static_cast<unsigned char>(value.c_str()[index]);
    const bool valid = index == 0
      ? ascii_lowercase_letter(character)
      : ascii_lowercase_letter(character)
        || ascii_digit(character)
        || character == '_';
    if (!valid) return false;
  }
  return true;
}

struct JsonRequestEnvelopeView {
  ArduinoJson::JsonVariantConst command;
  ArduinoJson::JsonVariantConst identifier;
  ArduinoJson::JsonVariantConst parameters;
  ArduinoJson::JsonVariantConst type;
  ArduinoJson::JsonVariantConst version;
};

inline bool extract_request_envelope(
  ArduinoJson::JsonObjectConst root,
  JsonRequestEnvelopeView &view
) {
  if (root.size() != kJsonRequestEnvelopeFieldCount) return false;
  view = {};
  bool command_present = false;
  bool identifier_present = false;
  bool parameters_present = false;
  bool type_present = false;
  bool version_present = false;
  for (ArduinoJson::JsonPairConst pair : root) {
    const ArduinoJson::JsonString key = pair.key();
    if (json_string_equals(key, "cmd")) {
      view.command = pair.value();
      command_present = true;
    } else if (json_string_equals(key, "id")) {
      view.identifier = pair.value();
      identifier_present = true;
    } else if (json_string_equals(key, "params")) {
      view.parameters = pair.value();
      parameters_present = true;
    } else if (json_string_equals(key, "type")) {
      view.type = pair.value();
      type_present = true;
    } else if (json_string_equals(key, "v")) {
      view.version = pair.value();
      version_present = true;
    } else {
      return false;
    }
  }
  return command_present
    && identifier_present
    && parameters_present
    && type_present
    && version_present;
}

inline bool extract_signed_int32_array(
  ArduinoJson::JsonVariantConst value,
  int32_t *output,
  size_t expected_count
) {
  if (
    output == nullptr
    || !value.is<ArduinoJson::JsonArrayConst>()
  ) {
    return false;
  }
  const ArduinoJson::JsonArrayConst values =
    value.as<ArduinoJson::JsonArrayConst>();
  if (values.size() != expected_count) return false;
  size_t index = 0;
  for (ArduinoJson::JsonVariantConst item : values) {
    if (!item.is<int32_t>()) return false;
    output[index++] = item.as<int32_t>();
  }
  return index == expected_count;
}

inline bool extract_set_position_parameters(
  ArduinoJson::JsonObjectConst params,
  JsonMainSetPositionParameters &output
) {
  if (params.size() != 2) return false;
  JsonMainSetPositionParameters staged = {};
  bool robot_joints_present = false;
  bool external_axes_present = false;
  for (ArduinoJson::JsonPairConst pair : params) {
    const ArduinoJson::JsonString key = pair.key();
    if (json_string_equals(key, "robot_joints_millidegrees")) {
      if (
        robot_joints_present
        || !extract_signed_int32_array(
          pair.value(),
          staged.robot_joints_millidegrees,
          6
        )
      ) {
        return false;
      }
      robot_joints_present = true;
    } else if (json_string_equals(key, "external_axes_milliunits")) {
      if (
        external_axes_present
        || !extract_signed_int32_array(
          pair.value(),
          staged.external_axes_milliunits,
          3
        )
      ) {
        return false;
      }
      external_axes_present = true;
    } else {
      return false;
    }
  }
  if (!robot_joints_present || !external_axes_present) return false;
  output = staged;
  return true;
}

inline bool normalize_controller_float(double parsed, float &output) {
  if (!isfinite(parsed) || fabs(parsed) > FLT_MAX) return false;
  const float converted = static_cast<float>(parsed);
  if (
    !isfinite(converted)
    || (parsed != 0.0 && converted == 0.0f)
  ) {
    return false;
  }
  output = converted;
  return true;
}

inline bool extract_controller_float(
  ArduinoJson::JsonVariantConst value,
  float &output
) {
  return !value.is<bool>()
    && value.is<double>()
    && normalize_controller_float(value.as<double>(), output);
}

inline bool extract_controller_wait_parameters(
  ArduinoJson::JsonObjectConst params,
  const JsonSoleNumericParameterObservation &observed_number,
  JsonMainControllerWaitParameters &output
) {
  const ArduinoJson::JsonVariantConst seconds_value = params["seconds"];
  if (
    params.size() != 1
    || seconds_value.is<bool>()
    || !seconds_value.is<double>()
    || !observed_number.present
  ) return false;
  float seconds = 0.0f;
  uint32_t duration_milliseconds = 0;
  if (
    observed_number.value < 0.0
    || observed_number.value > kMainFirmwareWaitMaxSeconds
    || !normalize_controller_float(observed_number.value, seconds)
    || !wait_seconds_to_milliseconds(seconds, duration_milliseconds)
  ) {
    return false;
  }
  output.duration_milliseconds = duration_milliseconds;
  return true;
}

inline bool extract_modbus_read_parameters(
  ArduinoJson::JsonObjectConst params, JsonMainModbusReadParameters &output
) {
  if (params.size() != 3) return false;
  const ArduinoJson::JsonVariantConst slave_id = params["slave_id"];
  const ArduinoJson::JsonVariantConst address = params["address"];
  const ArduinoJson::JsonVariantConst count = params["count"];
  if (
    slave_id.is<bool>() || !slave_id.is<int>()
    || address.is<bool>() || !address.is<int>()
    || count.is<bool>() || !count.is<int>()
  ) return false;
  const JsonMainModbusReadParameters staged = {
    slave_id.as<int>(), address.as<int>(), count.as<int>(),
  };
  output = staged;
  return true;
}

inline bool extract_modbus_write_parameters(
  ArduinoJson::JsonObjectConst params, JsonMainModbusWriteParameters &output
) {
  if (params.size() != 3) return false;
  const ArduinoJson::JsonVariantConst slave_id = params["slave_id"];
  const ArduinoJson::JsonVariantConst address = params["address"];
  const ArduinoJson::JsonVariantConst value = params["value"];
  if (
    slave_id.is<bool>() || !slave_id.is<int>()
    || address.is<bool>() || !address.is<int>()
    || value.is<bool>() || !value.is<int>()
  ) return false;
  const JsonMainModbusWriteParameters staged = {
    slave_id.as<int>(), address.as<int>(), value.as<int>(),
  };
  output = staged;
  return true;
}

inline bool extract_controller_float_array(
  ArduinoJson::JsonVariantConst value,
  float *output,
  size_t expected_count
) {
  if (
    output == nullptr
    || !value.is<ArduinoJson::JsonArrayConst>()
  ) {
    return false;
  }
  const ArduinoJson::JsonArrayConst values =
    value.as<ArduinoJson::JsonArrayConst>();
  if (values.size() != expected_count) return false;
  size_t index = 0;
  for (ArduinoJson::JsonVariantConst item : values) {
    if (!extract_controller_float(item, output[index++])) return false;
  }
  return index == expected_count;
}

inline bool extract_binary_int_array(
  ArduinoJson::JsonVariantConst value,
  int *output,
  size_t expected_count
) {
  if (
    output == nullptr
    || !value.is<ArduinoJson::JsonArrayConst>()
  ) {
    return false;
  }
  const ArduinoJson::JsonArrayConst values =
    value.as<ArduinoJson::JsonArrayConst>();
  if (values.size() != expected_count) return false;
  size_t index = 0;
  for (ArduinoJson::JsonVariantConst item : values) {
    if (item.is<bool>() || !item.is<int>()) return false;
    const int parsed = item.as<int>();
    if (parsed != 0 && parsed != 1) return false;
    output[index++] = parsed;
  }
  return index == expected_count;
}

inline bool extract_bool_array(
  ArduinoJson::JsonVariantConst value,
  bool *output,
  size_t expected_count
) {
  if (
    output == nullptr
    || !value.is<ArduinoJson::JsonArrayConst>()
  ) {
    return false;
  }
  const ArduinoJson::JsonArrayConst values =
    value.as<ArduinoJson::JsonArrayConst>();
  if (values.size() != expected_count) return false;
  size_t index = 0;
  for (ArduinoJson::JsonVariantConst item : values) {
    if (!item.is<bool>()) return false;
    output[index++] = item.as<bool>();
  }
  return index == expected_count;
}

inline bool extract_update_parameters(
  ArduinoJson::JsonObjectConst params,
  JsonMainUpdateParameters &output
) {
  if (params.size() != 13) return false;
  JsonMainUpdateParameters staged = {};
  bool tool_translation_present = false;
  bool tool_rotation_present = false;
  bool motor_directions_present = false;
  bool calibration_directions_present = false;
  bool calibration_switches_present = false;
  bool positive_limits_present = false;
  bool negative_limits_present = false;
  bool steps_per_degree_present = false;
  bool encoder_counts_present = false;
  bool dh_theta_present = false;
  bool dh_alpha_present = false;
  bool dh_d_present = false;
  bool dh_a_present = false;
  for (ArduinoJson::JsonPairConst pair : params) {
    const ArduinoJson::JsonString key = pair.key();
    if (json_string_equals(key, "tool_translation_millimeters")) {
      if (
        tool_translation_present
        || !extract_controller_float_array(
          pair.value(),
          staged.tool_translation_millimeters,
          3
        )
      ) return false;
      tool_translation_present = true;
    } else if (json_string_equals(key, "tool_rotation_degrees")) {
      if (
        tool_rotation_present
        || !extract_controller_float_array(
          pair.value(),
          staged.tool_rotation_degrees,
          3
        )
      ) return false;
      tool_rotation_present = true;
    } else if (json_string_equals(key, "motor_directions")) {
      if (
        motor_directions_present
        || !extract_binary_int_array(
          pair.value(),
          staged.motor_directions,
          kJsonControllerAxisCount
        )
      ) return false;
      motor_directions_present = true;
    } else if (json_string_equals(key, "calibration_directions")) {
      if (
        calibration_directions_present
        || !extract_binary_int_array(
          pair.value(),
          staged.calibration_directions,
          kJsonControllerAxisCount
        )
      ) return false;
      calibration_directions_present = true;
    } else if (json_string_equals(
        key,
        "calibration_switch_active_high"
    )) {
      if (
        calibration_switches_present
        || !extract_bool_array(
          pair.value(),
          staged.calibration_switch_active_high,
          kJsonControllerAxisCount
        )
      ) return false;
      calibration_switches_present = true;
    } else if (json_string_equals(
        key,
        "positive_joint_limits_degrees"
    )) {
      if (
        positive_limits_present
        || !extract_controller_float_array(
          pair.value(),
          staged.positive_joint_limits_degrees,
          kJsonPrimaryJointCount
        )
      ) return false;
      positive_limits_present = true;
    } else if (json_string_equals(
        key,
        "negative_joint_limits_degrees"
    )) {
      if (
        negative_limits_present
        || !extract_controller_float_array(
          pair.value(),
          staged.negative_joint_limits_degrees,
          kJsonPrimaryJointCount
        )
      ) return false;
      negative_limits_present = true;
    } else if (json_string_equals(key, "steps_per_degree")) {
      if (
        steps_per_degree_present
        || !extract_controller_float_array(
          pair.value(),
          staged.steps_per_degree,
          kJsonPrimaryJointCount
        )
      ) return false;
      steps_per_degree_present = true;
    } else if (json_string_equals(key, "encoder_counts_per_step")) {
      if (
        encoder_counts_present
        || !extract_controller_float_array(
          pair.value(),
          staged.encoder_counts_per_step,
          kJsonPrimaryJointCount
        )
      ) return false;
      encoder_counts_present = true;
    } else if (json_string_equals(key, "dh_theta_degrees")) {
      if (
        dh_theta_present
        || !extract_controller_float_array(
          pair.value(),
          staged.dh_theta_degrees,
          kJsonPrimaryJointCount
        )
      ) return false;
      dh_theta_present = true;
    } else if (json_string_equals(key, "dh_alpha_degrees")) {
      if (
        dh_alpha_present
        || !extract_controller_float_array(
          pair.value(),
          staged.dh_alpha_degrees,
          kJsonPrimaryJointCount
        )
      ) return false;
      dh_alpha_present = true;
    } else if (json_string_equals(key, "dh_d_millimeters")) {
      if (
        dh_d_present
        || !extract_controller_float_array(
          pair.value(),
          staged.dh_d_millimeters,
          kJsonPrimaryJointCount
        )
      ) return false;
      dh_d_present = true;
    } else if (json_string_equals(key, "dh_a_millimeters")) {
      if (
        dh_a_present
        || !extract_controller_float_array(
          pair.value(),
          staged.dh_a_millimeters,
          kJsonPrimaryJointCount
        )
      ) return false;
      dh_a_present = true;
    } else {
      return false;
    }
  }
  if (
    !tool_translation_present
    || !tool_rotation_present
    || !motor_directions_present
    || !calibration_directions_present
    || !calibration_switches_present
    || !positive_limits_present
    || !negative_limits_present
    || !steps_per_degree_present
    || !encoder_counts_present
    || !dh_theta_present
    || !dh_alpha_present
    || !dh_d_present
    || !dh_a_present
  ) {
    return false;
  }
  output = staged;
  return true;
}

inline bool extract_external_axis_parameters(
  ArduinoJson::JsonObjectConst params,
  JsonMainExternalAxisParameters &output
) {
  if (params.size() != 3) return false;
  JsonMainExternalAxisParameters staged = {};
  bool travel_present = false;
  bool rotations_present = false;
  bool steps_present = false;
  for (ArduinoJson::JsonPairConst pair : params) {
    const ArduinoJson::JsonString key = pair.key();
    if (json_string_equals(key, "travel_units")) {
      if (
        travel_present
        || !extract_controller_float_array(
          pair.value(),
          staged.travel_units,
          kJsonExternalAxisCount
        )
      ) return false;
      travel_present = true;
    } else if (json_string_equals(key, "drive_rotations")) {
      if (
        rotations_present
        || !extract_controller_float_array(
          pair.value(),
          staged.drive_rotations,
          kJsonExternalAxisCount
        )
      ) return false;
      rotations_present = true;
    } else if (json_string_equals(key, "motor_steps")) {
      if (
        steps_present
        || !extract_controller_float_array(
          pair.value(),
          staged.motor_steps,
          kJsonExternalAxisCount
        )
      ) return false;
      steps_present = true;
    } else {
      return false;
    }
  }
  if (!travel_present || !rotations_present || !steps_present) return false;
  output = staged;
  return true;
}

inline JsonMainRequestParseStatus classify_semantic_deserialization(
  const ArduinoJson::DeserializationError &error,
  const JsonMainRequestSemanticAllocator &allocator
) {
  // A distinct fault status protects future dependency or allocator drift.
  if (allocator.invalid_operation()) {
    return JsonMainRequestParseStatus::kAllocatorStateInvalid;
  }
  // Any denied allocation taints the parse even after dependency recovery.
  if (
    error == ArduinoJson::DeserializationError::NoMemory
    || allocator.failed()
  ) {
    return JsonMainRequestParseStatus::kParserResourceExhausted;
  }
  return error
    ? JsonMainRequestParseStatus::kMalformedJson
    : JsonMainRequestParseStatus::kReady;
}

}  // namespace json_request_detail

inline JsonMainRequestParseStatus parse_main_json_request(
  const char *payload,
  size_t payload_length,
  JsonMainRequestParserWorkspace &workspace,
  JsonMainRequestParseResult &result,
  const JsonMainRequestParseOptions &options =
    kJsonMainRequestDefaultParseOptions
) {
  result.status = JsonMainRequestParseStatus::kInvalidArgument;
  result.correlation_valid = false;
  result.request_id = 0;
  result.command[0] = '\0';
  result.payload.reset();
  if (
    payload == nullptr
    || !JsonMainRequestSemanticAllocator::valid_limit(
      options.semantic_allocation_limit
    )
  ) {
    return result.status;
  }
  if (!workspace.semantic_allocator.reset(
      options.semantic_allocation_limit
  )) {
    result.status = JsonMainRequestParseStatus::kAllocatorStateInvalid;
    return result.status;
  }
  if (
    payload_length == 0
    || payload_length > kJsonProtocolMaximumPayloadBytes
  ) {
    result.status = JsonMainRequestParseStatus::kInvalidPayload;
    return result.status;
  }
  for (size_t index = 0; index < payload_length; ++index) {
    const unsigned char value = static_cast<unsigned char>(payload[index]);
    if (value < 0x20 || value > 0x7E) {
      result.status = JsonMainRequestParseStatus::kInvalidPayload;
      return result.status;
    }
  }

  // Correlation is best-effort; strict scanning owns payload validity.
  json_request_detail::extract_request_correlation(
    payload,
    payload_length,
    workspace,
    result
  );

  json_request_detail::StrictJsonScanner scanner(
    payload,
    payload_length,
    workspace
  );
  const JsonMainRequestParseStatus structural_status = scanner.scan();
  if (structural_status != JsonMainRequestParseStatus::kReady) {
    result.status = structural_status;
    return result.status;
  }

  ArduinoJson::JsonDocument document(&workspace.semantic_allocator);
  const ArduinoJson::DeserializationError error = ArduinoJson::deserializeJson(
    document,
    payload,
    payload_length,
    ArduinoJson::DeserializationOption::NestingLimit(
      kJsonArduinoDeserializerNestingLimit
    )
  );
  const JsonMainRequestParseStatus deserialization_status =
    json_request_detail::classify_semantic_deserialization(
      error,
      workspace.semantic_allocator
    );
  if (deserialization_status != JsonMainRequestParseStatus::kReady) {
    result.status = deserialization_status;
    return result.status;
  }

  const ArduinoJson::JsonObjectConst root =
    document.as<ArduinoJson::JsonObjectConst>();
  json_request_detail::JsonRequestEnvelopeView envelope = {};
  if (
    root.isNull()
    || !json_request_detail::extract_request_envelope(root, envelope)
  ) {
    result.status = JsonMainRequestParseStatus::kInvalidEnvelope;
    return result.status;
  }
  if (
    !envelope.version.is<uint32_t>()
    || envelope.version.as<uint32_t>() != kJsonProtocolVersion
  ) {
    result.status = JsonMainRequestParseStatus::kUnsupportedVersion;
    return result.status;
  }
  const ArduinoJson::JsonString type =
    envelope.type.as<ArduinoJson::JsonString>();
  if (!json_request_detail::json_string_equals(type, "request")) {
    result.status = JsonMainRequestParseStatus::kUnsupportedMessageType;
    return result.status;
  }
  if (
    !envelope.identifier.is<uint32_t>()
    || envelope.identifier.as<uint32_t>() == 0
  ) {
    result.status = JsonMainRequestParseStatus::kInvalidRequestIdentifier;
    return result.status;
  }
  const ArduinoJson::JsonString command =
    envelope.command.as<ArduinoJson::JsonString>();
  if (!json_request_detail::json_protocol_name_valid(command)) {
    result.status = JsonMainRequestParseStatus::kInvalidCommandName;
    return result.status;
  }
  const JsonMainRequestCommand parsed_command_kind =
    json_main_request_command_from_name(command.c_str());
  if (!envelope.parameters.is<ArduinoJson::JsonObjectConst>()) {
    result.status = JsonMainRequestParseStatus::kInvalidParameters;
    return result.status;
  }

  if (!result.correlation_valid) {
    result.correlation_valid = true;
    result.request_id = envelope.identifier.as<uint32_t>();
    memcpy(result.command, command.c_str(), command.size());
    result.command[command.size()] = '\0';
  }
  const ArduinoJson::JsonObjectConst params =
    envelope.parameters.as<ArduinoJson::JsonObjectConst>();
  if (parsed_command_kind == JsonMainRequestCommand::kUnknown) {
    result.status = JsonMainRequestParseStatus::kUnknownCommand;
    return result.status;
  }
  bool parameters_valid = false;
  switch (parsed_command_kind) {
    case JsonMainRequestCommand::kSetPosition: {
      JsonMainSetPositionParameters parameters = {};
      parameters_valid = json_request_detail::extract_set_position_parameters(
        params,
        parameters
      );
      if (parameters_valid) result.payload.assign_set_position(parameters);
      break;
    }
    case JsonMainRequestCommand::kUpdateParams: {
      JsonMainUpdateParameters parameters = {};
      parameters_valid = json_request_detail::extract_update_parameters(
        params,
        parameters
      );
      if (parameters_valid) result.payload.assign_update_params(parameters);
      break;
    }
    case JsonMainRequestCommand::kConfigExtAxis: {
      JsonMainExternalAxisParameters parameters = {};
      parameters_valid =
        json_request_detail::extract_external_axis_parameters(
          params,
          parameters
        );
      if (parameters_valid) {
        result.payload.assign_config_ext_axis(parameters);
      }
      break;
    }
    case JsonMainRequestCommand::kControllerWait: {
      JsonMainControllerWaitParameters parameters = {};
      parameters_valid =
        json_request_detail::extract_controller_wait_parameters(
          params,
          scanner.sole_numeric_parameter(),
          parameters
        );
      if (parameters_valid) result.payload.assign_controller_wait(parameters);
      break;
    }
    case JsonMainRequestCommand::kModbusReadHoldingRegister:
    case JsonMainRequestCommand::kModbusReadCoil:
    case JsonMainRequestCommand::kModbusReadDiscreteInput:
    case JsonMainRequestCommand::kModbusReadInputRegister: {
      JsonMainModbusReadParameters parameters = {};
      parameters_valid = json_request_detail::extract_modbus_read_parameters(
        params,
        parameters
      ) && result.payload.assign_modbus_read(
        parsed_command_kind,
        parameters
      );
      break;
    }
    case JsonMainRequestCommand::kModbusWriteCoil:
    case JsonMainRequestCommand::kModbusWriteRegister: {
      JsonMainModbusWriteParameters parameters = {};
      parameters_valid = json_request_detail::extract_modbus_write_parameters(
        params,
        parameters
      ) && result.payload.assign_modbus_write(
        parsed_command_kind,
        parameters
      );
      break;
    }
    case JsonMainRequestCommand::kCalibrate: {
      JsonMainCalibrationParameters parameters = {};
      parameters_valid = extract_main_calibration_parameters(
        params,
        parameters
      );
      if (parameters_valid) result.payload.assign_calibration(parameters);
      break;
    }
    case JsonMainRequestCommand::kJogTool: {
      JsonMainToolJogParameters parameters = {};
      parameters_valid = extract_main_tool_jog_parameters(
        params,
        parameters
      );
      if (parameters_valid) result.payload.assign_jog_tool(parameters);
      break;
    }
    case JsonMainRequestCommand::kLiveJointJog: {
      JsonMainLiveJogParameters parameters = {};
      parameters_valid = extract_main_live_jog_parameters(
        params,
        JsonLiveJogKind::kJoint,
        parameters
      );
      if (parameters_valid) {
        parameters_valid = result.payload.assign_live_jog(
          parsed_command_kind,
          parameters
        );
      }
      break;
    }
    case JsonMainRequestCommand::kLiveCartJog: {
      JsonMainLiveJogParameters parameters = {};
      parameters_valid = extract_main_live_jog_parameters(
        params,
        JsonLiveJogKind::kCartesian,
        parameters
      );
      if (parameters_valid) {
        parameters_valid = result.payload.assign_live_jog(
          parsed_command_kind,
          parameters
        );
      }
      break;
    }
    case JsonMainRequestCommand::kLiveToolJog: {
      JsonMainLiveJogParameters parameters = {};
      parameters_valid = extract_main_live_jog_parameters(
        params,
        JsonLiveJogKind::kTool,
        parameters
      );
      if (parameters_valid) {
        parameters_valid = result.payload.assign_live_jog(
          parsed_command_kind,
          parameters
        );
      }
      break;
    }
    case JsonMainRequestCommand::kMoveCartesian: {
      JsonMainMoveCartesianParameters parameters = {};
      parameters_valid = extract_main_move_cartesian_parameters(
        params,
        parameters
      );
      if (parameters_valid) result.payload.assign_move_cartesian(parameters);
      break;
    }
    case JsonMainRequestCommand::kMoveJoints: {
      JsonMainMoveJointsParameters parameters = {};
      parameters_valid = extract_main_move_joints_parameters(
        params,
        parameters
      );
      if (parameters_valid) result.payload.assign_move_joints(parameters);
      break;
    }
    case JsonMainRequestCommand::kGetMotionTrace: {
      JsonMainMotionTraceParameters parameters = {};
      parameters_valid = extract_main_motion_trace_parameters(
        params,
        parameters
      );
      if (parameters_valid) result.payload.assign_motion_trace(parameters);
      break;
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
      JsonMainDirectParameters parameters = {};
      parameters_valid = extract_main_direct_parameters(
        result.command,
        params,
        parameters
      );
      if (parameters_valid) {
        result.payload.assign_direct(parsed_command_kind, parameters);
      }
      break;
    }
    case JsonMainRequestCommand::kStop: {
      JsonMainStopParameters parameters = {};
      parameters_valid = extract_main_stop_parameters(
        params,
        parameters
      );
      if (parameters_valid) result.payload.assign_stop(parameters);
      break;
    }
    case JsonMainRequestCommand::kRenewLiveMotion: {
      JsonMainRenewLiveMotionParameters parameters = {};
      parameters_valid = extract_main_renew_live_motion_parameters(
        params,
        parameters
      );
      if (parameters_valid) {
        result.payload.assign_renew_live_motion(parameters);
      }
      break;
    }
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
      parameters_valid = params.size() == 0
        && result.payload.assign_empty(parsed_command_kind);
      break;
    case JsonMainRequestCommand::kUnknown:
      parameters_valid = false;
      break;
  }
  if (!parameters_valid) {
    result.status = JsonMainRequestParseStatus::kInvalidParameters;
    return result.status;
  }
  result.status = JsonMainRequestParseStatus::kReady;
  return result.status;
}

}  // namespace ar4_protocol

#endif
