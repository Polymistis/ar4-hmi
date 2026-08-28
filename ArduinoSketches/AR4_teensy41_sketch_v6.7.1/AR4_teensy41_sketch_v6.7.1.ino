/*  AR4 Robot Control Software
    Copyright (c) 2024, Chris Annin
    All rights reserved.

    You are free to share, copy and redistribute in any medium
    or format.  You are free to remix, transform and build upon
    this material.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:

          Redistributions of source code must retain the above copyright
          notice, this list of conditions and the following disclaimer.
          Redistribution of this software in source or binary forms shall be free
          of all charges or fees to the recipient of this software.
          Redistributions in binary form must reproduce the above copyright
          notice, this list of conditions and the following disclaimer in the
          documentation and/or other materials provided with the distribution.
          you must give appropriate credit and indicate if changes were made. You may do
          so in any reasonable manner, but not in any way that suggests the
          licensor endorses you or your use.
          Selling Annin Robotics software, robots, robot parts, or any versions of robots or software based on this
          work is strictly prohibited.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
    ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL CHRIS ANNIN BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

    chris.annin@gmail.com

*/


// VERSION LOG
// 1.0 - 2/6/21 - initial release
// 1.1 - 2/20/21 - bug fix, calibration offset on negative axis calibration direction axis 2,4,5
// 2.0 - 10/1/22 - added lookahead and spline functionality
// 2.2 - 11/6/22 - added Move V for open cv integrated vision
// 3.0 - 2/3/23 - open loop bypass moved to teensy board / add external axis 8 & 9 / bug fix live jog drift
// 3.1 - 5/10/23 - gcode initial
// 3.2 - 5/12/23 - remove RoboDK kinematics
// 3.3 - 6/4/23 - update geometric kinematics
// 4.0 - 11/5/23 - .txt .ar4 extension, gcode tab, kinematics tab. Initial MK2 release.
// 4.1 - 11/23/23 - bug fix added - R06_neg_matrix[2][3] = -DHparams[5][2]; added to UPdate CMD & GCC diagnostic
// 4.2 - 1/12/24 - bug fix - step direction delay
// 4.3 - 1/21/24 - Gcode to SD card.  Estop button interrupt.
// 4.3.1 - 2/1/24 bug fix - vision snap and find drop down
// 4.4 - 3/2/24 added kinematic error handling
// 4.5 - 6/29/24 simplified drive motors functions with arrays
// 5.0 - 7/14/24 updated kinematics
// 5.1 - 2/15/25 Modbus option
// 5.2 - 6/7/25 Modbus option
// 6.0 - 6/7/25 Virtual Robot
// 6.1 - 8/29/25 updated accel and decel, auto calibrate & microsteps
// 6.2 - 8/29/25 changed bootstrap theme, xbox upgrade
// 6.3 - 10/8/25 - JK - added beta Linux support
// 6.4 - 10/29/25 - added set robot command to store HW and version to eprom, MK4 update, fixed tool jog, re-added 2 step calibration, add servo amp test
// 6.5 - 2/15/26 - gcode bug fix / update program flow with run once and run program loop
// 6.6 - 2/22/26 - update kinematic solver to reduce J4/6 wrap | reimplement wrist N/F config
// 6.7 - 3/11/26 MB holding reg bug fix
// 6.7.1 - 3/11/26 bug fix calibration debounce
const char *FIRMWARE_VERSION = "6.7.1-ar4hmi.38";
const char *JSON_FIRMWARE_NAME = "AR4 Teensy";
const char *JSON_FIRMWARE_BUILD = "tracked";

//////////////////////////////////////////////////////////////////////////////
//DEBUGGING
//////////////////////////////////////////////////////////////////////////////
// Define function aliases for debugging but only if debugging is enabled

#include <math.h>
#include <limits>
#include <string.h>
#include <avr/pgmspace.h>
#include <Encoder.h>
#include <SPI.h>
#include <SD.h>
#include <stdexcept>
#include <ModbusMaster.h>
#include <EEPROM.h>
#include "angle_conversion_contract.h"
#include "calibration_switch_contract.h"
#include "command_queue_contract.h"
#include "controller_domain_contract.h"
#include "cartesian_pose_contract.h"
#include "debug_contract.h"
#include "home_reference_contract.h"
#include "identity_contract.h"
#include "json_event_contract.h"
#include "json_main_controller_dispatch_contract.h"
#include "json_runtime_contract.h"
#include "json_session_identity_contract.h"
#include "joint_telemetry_contract.h"
#include "joint_speed_target_contract.h"
#include "motion_command_parse_contract.h"
#include "motion_mode_transaction.h"
#include "numeric_parse_contract.h"
#include "persistence_contract.h"
#include "serial_frame_contract.h"
#include "tool_jog_contract.h"
#include "wrist_selection_contract.h"
const char *const PROTOCOL_CAPABILITIES[] = {
  ar4_protocol::kJsonProtocolCapability,
  ar4_protocol::kJsonRequestCorrelationCapability,
  ar4_protocol::kJsonEventStreamCapability,
};
constexpr size_t PROTOCOL_CAPABILITY_COUNT =
  sizeof(PROTOCOL_CAPABILITIES) / sizeof(PROTOCOL_CAPABILITIES[0]);
const char *const JSON_COMMANDS[] = {
  "hello", "get_home_reference", "get_position_disposition",
  "correct_position", "set_position", "test_limit_switches", "set_encoders",
  "read_encoders", "update_params", "config_ext_axis", "zero_j7", "zero_j8",
  "zero_j9", "controller_wait", "calibrate", "move_joints", "move_cartesian",
  "move_linear", "move_vision", "jog_tool", "live_joint_jog", "live_cart_jog",
  "live_tool_jog", "stop", "renew_live_motion", "modbus_read_holding_register",
  "modbus_read_coil", "modbus_read_discrete_input", "modbus_read_input_register",
  "modbus_write_coil", "modbus_write_register", "wait_modbus_coil",
  "wait_modbus_discrete_input", "delete_sd_program", "list_sd_programs",
  "write_gcode_move", "play_gcode_file", "wait_modbus_holding_register",
  "move_arc", "move_circle", "move_spline",
};
constexpr size_t JSON_COMMAND_COUNT = sizeof(JSON_COMMANDS) / sizeof(JSON_COMMANDS[0]);
static_assert(
  PROTOCOL_CAPABILITY_COUNT
    <= ar4_protocol::kProtocolCapabilityMaximumCount,
  "Advertised protocol capabilities exceed the identity contract"
);
#pragma GCC diagnostic ignored "-Warray-bounds"
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wsequence-point"
#pragma GCC diagnostic ignored "-Wunused-value"
#pragma GCC diagnostic ignored "-Wunused-function"
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#pragma GCC diagnostic ignored "-Waddress"
#pragma GCC diagnostic ignored "-Wall"

#define Table_Size 6
typedef float Matrix4x4[16];
typedef float tRobot[66];

String recData;
bool serialFrameDiscarding = false;
ar4_protocol::SerialFrameReceiveDeadline serialFrameReceiveDeadline = {};
bool jsonRuntimeFault = false;
bool jsonResponseWritePrepared = false;
size_t jsonResponseWriteOffset = 0;
bool jsonEventWritePrepared = false;
size_t jsonEventWriteOffset = 0;
bool jsonEstopAdmissionResponsePending = false;
bool jsonEstopAdmissionTelemetryBlocked = false;
ar4_protocol::EstopAdmissionDecision jsonEstopAdmissionDecision = {
  false,
  0,
};
ar4_protocol::JsonMainControllerFrameView jsonResponseWriteView = {};
ar4_protocol::JsonEventFrameView jsonEventWriteView = {};
ar4_protocol::JsonMainControllerRequestOwner jsonMainControllerOwner;
ar4_protocol::JsonControllerEventOutputOwner jsonControllerEventOwner;
uint32_t jsonJointTelemetrySequence = 0;
using ar4_protocol::JsonLiveMotionRuntimeState;
struct JsonLiveMotionRuntime {
  ar4_protocol::JsonLiveMotionRuntimeOwner owner;
  ar4_protocol::JsonMainLiveJogParameters parameters;
  ar4_protocol::JsonMainLiveJogExecutionResult terminal;
  bool speed_limited;
};
JsonLiveMotionRuntime jsonLiveMotion = {};

void processSerial();
bool service_json_output();
uint32_t json_live_motion_microseconds(void *context);
void json_live_motion_delay_microseconds(
  uint32_t durationMicroseconds,
  void *context
);
bool json_joint_telemetry_output_available(
  const ar4_protocol::JsonLiveMotionContinuationSource *continuationSource
);

const int J1stepPin = 0;
const int J1dirPin = 1;
const int J2stepPin = 2;
const int J2dirPin = 3;
const int J3stepPin = 4;
const int J3dirPin = 5;
const int J4stepPin = 6;
const int J4dirPin = 7;
const int J5stepPin = 8;
const int J5dirPin = 9;
const int J6stepPin = 10;
const int J6dirPin = 11;
const int J7stepPin = 12;
const int J7dirPin = 13;
const int J8stepPin = 32;
const int J8dirPin = 33;
const int J9stepPin = 40;
const int J9dirPin = 41;

const int J1calPin = 26;
const int J2calPin = 27;
const int J3calPin = 28;
const int J4calPin = 29;
const int J5calPin = 30;
const int J6calPin = 31;
const int J7calPin = 36;
const int J8calPin = 37;
const int J9calPin = 38;

const int EstopPin = 39;


//set encoder multiplier
float J1encMult = 5;
float J2encMult = 5;
float J3encMult = 5;
float J4encMult = 5;
float J5encMult = 2.5;
float J6encMult = 5;
int encOffset = 50;

//set encoder pins
Encoder J1encPos(14, 15);
Encoder J2encPos(17, 16);
Encoder J3encPos(19, 18);
Encoder J4encPos(20, 21);
Encoder J5encPos(23, 22);
Encoder J6encPos(24, 25);

ModbusMaster node;


// GLOBAL VARS //

//define axis limits in degrees
float J1axisLimPos = 170;
float J1axisLimNeg = 170;
float J2axisLimPos = 90;
float J2axisLimNeg = 42;
float J3axisLimPos = 52;
float J3axisLimNeg = 89;
float J4axisLimPos = 180;
float J4axisLimNeg = 180;
float J5axisLimPos = 105;
float J5axisLimNeg = 105;
float J6axisLimPos = 180;
float J6axisLimNeg = 180;
float J7axisLimPos = 3450;
float J7axisLimNeg = 0;
float J8axisLimPos = 3450;
float J8axisLimNeg = 0;
float J9axisLimPos = 3450;
float J9axisLimNeg = 0;

int J1MotDir = 0;
int J2MotDir = 1;
int J3MotDir = 1;
int J4MotDir = 1;
int J5MotDir = 1;
int J6MotDir = 1;
int J7MotDir = 1;
int J8MotDir = 1;
int J9MotDir = 1;

int J1CalDir = 1;
int J2CalDir = 0;
int J3CalDir = 1;
int J4CalDir = 0;
int J5CalDir = 0;
int J6CalDir = 1;
int J7CalDir = 0;
int J8CalDir = 0;
int J9CalDir = 0;

//define total axis travel
float J1axisLim = J1axisLimPos + J1axisLimNeg;
float J2axisLim = J2axisLimPos + J2axisLimNeg;
float J3axisLim = J3axisLimPos + J3axisLimNeg;
float J4axisLim = J4axisLimPos + J4axisLimNeg;
float J5axisLim = J5axisLimPos + J5axisLimNeg;
float J6axisLim = J6axisLimPos + J6axisLimNeg;
float J7axisLim = J7axisLimPos + J7axisLimNeg;
float J8axisLim = J8axisLimPos + J8axisLimNeg;
float J9axisLim = J9axisLimPos + J9axisLimNeg;

//motor steps per degree
float J1StepDeg = 88.888;
float J2StepDeg = 111.111;
float J3StepDeg = 111.111;
float J4StepDeg = 99.555;
float J5StepDeg = 43.720;
float J6StepDeg = 44.444;
float J7StepDeg = 14.2857;
float J8StepDeg = 14.2857;
float J9StepDeg = 14.2857;

//steps full movement of each axis
int J1StepLim = J1axisLim * J1StepDeg;
int J2StepLim = J2axisLim * J2StepDeg;
int J3StepLim = J3axisLim * J3StepDeg;
int J4StepLim = J4axisLim * J4StepDeg;
int J5StepLim = J5axisLim * J5StepDeg;
int J6StepLim = J6axisLim * J6StepDeg;
int J7StepLim = J7axisLim * J7StepDeg;
int J8StepLim = J8axisLim * J8StepDeg;
int J9StepLim = J9axisLim * J9StepDeg;

//step at axis zero
int J1zeroStep = J1axisLimNeg * J1StepDeg;
int J2zeroStep = J2axisLimNeg * J2StepDeg;
int J3zeroStep = J3axisLimNeg * J3StepDeg;
int J4zeroStep = J4axisLimNeg * J4StepDeg;
int J5zeroStep = J5axisLimNeg * J5StepDeg;
int J6zeroStep = J6axisLimNeg * J6StepDeg;
int J7zeroStep = J7axisLimNeg * J7StepDeg;
int J8zeroStep = J8axisLimNeg * J8StepDeg;
int J9zeroStep = J9axisLimNeg * J9StepDeg;

//start master step count at Jzerostep
int J1StepM = J1zeroStep;
int J2StepM = J2zeroStep;
int J3StepM = J3zeroStep;
int J4StepM = J4zeroStep;
int J5StepM = J5zeroStep;
int J6StepM = J6zeroStep;
int J7StepM = J7zeroStep;
int J8StepM = J8zeroStep;
int J9StepM = J9zeroStep;



//FIRMWARE CALIBRATION OFFSET - this is combined with the HMI software offset
float J1calBaseOff = -6.2;
float J2calBaseOff = 3.8;
float J3calBaseOff = 1.4;
float J4calBaseOff = -.8;
float J5calBaseOff = 5.6;
float J6calBaseOff = .5;
float J7calBaseOff = 0;
float J8calBaseOff = 0;
float J9calBaseOff = 0;
ar4_protocol::PrimaryHomeReferenceState primaryHomeReference = {
  { false, false, false },
  { 0, 0, 0 },
};

//reset collision indicators
int J1collisionTrue = 0;
int J2collisionTrue = 0;
int J3collisionTrue = 0;
int J4collisionTrue = 0;
int J5collisionTrue = 0;
int J6collisionTrue = 0;
int TotalCollision = 0;
int KinematicError = 0;

float J7length;
float J7rot;
float J7steps;

float J8length;
float J8rot;
float J8steps;

float J9length;
float J9rot;
float J9steps;

float lineDist;

String WristCon;
int Quadrant;

unsigned long J1DebounceTime = 0;
unsigned long J2DebounceTime = 0;
unsigned long J3DebounceTime = 0;
unsigned long J4DebounceTime = 0;
unsigned long J5DebounceTime = 0;
unsigned long J6DebounceTime = 0;
unsigned long debounceDelay = 50;
String Alarm = "0";
String speedViolation = "0";
float minSpeedDelay = 200;
float maxMMperSec = 192;
float linWayDistSP = 1;
String debug = "";
String flag = "";
const int TRACKrotdir = 0;
float JogStepInc = 1;

int J1EncSteps;
int J2EncSteps;
int J3EncSteps;
int J4EncSteps;
int J5EncSteps;
int J6EncSteps;

#define ROBOT_nDOFs 6
int JointLoopModes[ROBOT_nDOFs];
typedef ar4_protocol::MotionModeTransaction<
  String,
  ROBOT_nDOFs
> FirmwareMotionModeTransaction;
const int numJoints = 9;
static_assert(
  numJoints == ROBOT_nDOFs + 3,
  "JSON set-position mapping requires three external axes"
);
static_assert(
  ROBOT_nDOFs == ar4_protocol::kJsonPrimaryJointCount,
  "JSON configuration mapping requires six primary joints"
);
static_assert(
  numJoints == ar4_protocol::kJsonControllerAxisCount,
  "JSON configuration mapping requires nine controller axes"
);
uint8_t calibrationLimitSensor[numJoints] = {
  HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH,
};
typedef float tRobotJoints[ROBOT_nDOFs];
typedef float tRobotPose[ROBOT_nDOFs];

//declare in out vars
float xyzuvw_Out[ROBOT_nDOFs];
float xyzuvw_In[ROBOT_nDOFs];

float JangleOut[ROBOT_nDOFs];
float JangleIn[ROBOT_nDOFs];
float joints_estimate[ROBOT_nDOFs];
static_assert(
  ROBOT_nDOFs == ar4_protocol::kWristJointCount,
  "Wrist selection requires the six-axis robot matrix"
);
float SolutionMatrix[
  ar4_protocol::kWristJointCount
][ar4_protocol::kMaximumWristSolutions];

//external axis
float J7_pos;
float J8_pos;
float J9_pos;

float pose[16];

//define rounding vars
bool rndTrue;
float rndSpeed;
volatile bool estopActive;
volatile ar4_protocol::JointTelemetryResponseOwnership
  telemetryResponseOwnership = { false, false, false };
volatile ar4_protocol::ControllerResponseOwnership
  controllerResponseOwnership = { false, false };
volatile ar4_protocol::EstopAdmissionOwnership
  estopAdmissionOwnership = { 0, false };

float Xtool = 0;
float Ytool = 0;
float Ztool = 0;
float RZtool = 0;
float RYtool = 0;
float RXtool = 0;

//DENAVIT HARTENBERG PARAMETERS

float DHparams[6][4] = {
  { 0, 0, 169.77, 0 },
  { -90, -90, 0, 64.2 },
  { 0, 0, 0, 305 },
  { 0, -90, 222.63, 0 },
  { 0, 90, 0, 0 },
  { 180, -90, 41, 0 }
};

using ar4_protocol::parse_float_marker_fields;
using ar4_protocol::parse_float_span;
using ar4_protocol::parse_float_spans;
using ar4_protocol::parse_int_marker_fields;
using ar4_protocol::parse_int_span;

void load_axis_calibration(
  float (&negative_limits)[numJoints],
  float (&positive_limits)[numJoints],
  float (&steps_per_unit)[numJoints],
  int (&step_limits)[numJoints]
) {
  const float staged_negative_limits[numJoints] = {
    J1axisLimNeg, J2axisLimNeg, J3axisLimNeg,
    J4axisLimNeg, J5axisLimNeg, J6axisLimNeg,
    J7axisLimNeg, J8axisLimNeg, J9axisLimNeg,
  };
  const float staged_positive_limits[numJoints] = {
    J1axisLimPos, J2axisLimPos, J3axisLimPos,
    J4axisLimPos, J5axisLimPos, J6axisLimPos,
    J7axisLimPos, J8axisLimPos, J9axisLimPos,
  };
  const float staged_steps_per_unit[numJoints] = {
    J1StepDeg, J2StepDeg, J3StepDeg,
    J4StepDeg, J5StepDeg, J6StepDeg,
    J7StepDeg, J8StepDeg, J9StepDeg,
  };
  const int staged_step_limits[numJoints] = {
    J1StepLim, J2StepLim, J3StepLim,
    J4StepLim, J5StepLim, J6StepLim,
    J7StepLim, J8StepLim, J9StepLim,
  };
  for (int axis = 0; axis < numJoints; ++axis) {
    negative_limits[axis] = staged_negative_limits[axis];
    positive_limits[axis] = staged_positive_limits[axis];
    steps_per_unit[axis] = staged_steps_per_unit[axis];
    step_limits[axis] = staged_step_limits[axis];
  }
}

bool joint_positions_to_future_steps(
  const float (&positions)[numJoints],
  int (&future_steps)[numJoints]
) {
  float negative_limits[numJoints];
  float positive_limits[numJoints];
  float steps_per_unit[numJoints];
  int step_limits[numJoints];
  load_axis_calibration(
    negative_limits,
    positive_limits,
    steps_per_unit,
    step_limits
  );
  return ar4_protocol::calibrated_positions_to_steps(
    positions,
    negative_limits,
    positive_limits,
    steps_per_unit,
    step_limits,
    future_steps
  );
}

bool build_configured_position_rebase(
  const int (&step_monitors)[numJoints],
  ar4_protocol::ControllerPositionRebase &output
) {
  float negative_limits[numJoints];
  float positive_limits[numJoints];
  float steps_per_unit[numJoints];
  int step_limits[numJoints];
  load_axis_calibration(
    negative_limits,
    positive_limits,
    steps_per_unit,
    step_limits
  );
  const float encoder_counts_per_step[ROBOT_nDOFs] = {
    J1encMult, J2encMult, J3encMult,
    J4encMult, J5encMult, J6encMult,
  };
  return ar4_protocol::build_controller_position_rebase(
    step_monitors,
    step_limits,
    encoder_counts_per_step,
    output
  );
}

void commit_configured_position_rebase(
  const ar4_protocol::ControllerPositionRebase &state
) {
  J1StepM = state.step_monitors[0];
  J2StepM = state.step_monitors[1];
  J3StepM = state.step_monitors[2];
  J4StepM = state.step_monitors[3];
  J5StepM = state.step_monitors[4];
  J6StepM = state.step_monitors[5];
  J7StepM = state.step_monitors[6];
  J8StepM = state.step_monitors[7];
  J9StepM = state.step_monitors[8];
  J1encPos.write(state.encoder_counts[0]);
  J2encPos.write(state.encoder_counts[1]);
  J3encPos.write(state.encoder_counts[2]);
  J4encPos.write(state.encoder_counts[3]);
  J5encPos.write(state.encoder_counts[4]);
  J6encPos.write(state.encoder_counts[5]);
}

bool inverse_solution_to_future_steps(
  float J7_target,
  float J8_target,
  float J9_target,
  int (&future_steps)[numJoints]
) {
  const float positions[numJoints] = {
    JangleOut[0], JangleOut[1], JangleOut[2],
    JangleOut[3], JangleOut[4], JangleOut[5],
    J7_target, J8_target, J9_target,
  };
  return joint_positions_to_future_steps(positions, future_steps);
}

bool external_positions_to_future_steps(
  float J7_target,
  float J8_target,
  float J9_target,
  int (&future_steps)[3]
) {
  float negative_limits[numJoints];
  float positive_limits[numJoints];
  float steps_per_unit[numJoints];
  int step_limits[numJoints];
  load_axis_calibration(
    negative_limits,
    positive_limits,
    steps_per_unit,
    step_limits
  );
  const float targets[3] = { J7_target, J8_target, J9_target };
  int staged[3];
  for (int axis = 0; axis < 3; ++axis) {
    const int calibration_index = axis + ROBOT_nDOFs;
    if (!ar4_protocol::calibrated_position_to_step(
        targets[axis],
        negative_limits[calibration_index],
        positive_limits[calibration_index],
        steps_per_unit[calibration_index],
        step_limits[calibration_index],
        staged[axis]
    )) {
      return false;
    }
  }
  for (int axis = 0; axis < 3; ++axis) future_steps[axis] = staged[axis];
  return true;
}

bool primary_inverse_solution_to_future_steps(
  int (&future_steps)[ROBOT_nDOFs]
) {
  float negative_limits[numJoints];
  float positive_limits[numJoints];
  float steps_per_unit[numJoints];
  int step_limits[numJoints];
  load_axis_calibration(
    negative_limits,
    positive_limits,
    steps_per_unit,
    step_limits
  );
  int staged[ROBOT_nDOFs];
  for (int axis = 0; axis < ROBOT_nDOFs; ++axis) {
    if (!ar4_protocol::calibrated_position_to_step(
        JangleOut[axis],
        negative_limits[axis],
        positive_limits[axis],
        steps_per_unit[axis],
        step_limits[axis],
        staged[axis]
    )) {
      return false;
    }
  }
  for (int axis = 0; axis < ROBOT_nDOFs; ++axis) {
    future_steps[axis] = staged[axis];
  }
  return true;
}

bool future_step_is_outside_limit(int future_step, int step_limit) {
  return future_step < 0 || future_step > step_limit;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//MATRIX OPERATION
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//This allow to return a array as an argument instead of using global pointer

#define Matrix_Multiply(out, inA, inB) \
  (out)[0] = (inA)[0] * (inB)[0] + (inA)[4] * (inB)[1] + (inA)[8] * (inB)[2]; \
  (out)[1] = (inA)[1] * (inB)[0] + (inA)[5] * (inB)[1] + (inA)[9] * (inB)[2]; \
  (out)[2] = (inA)[2] * (inB)[0] + (inA)[6] * (inB)[1] + (inA)[10] * (inB)[2]; \
  (out)[3] = 0; \
  (out)[4] = (inA)[0] * (inB)[4] + (inA)[4] * (inB)[5] + (inA)[8] * (inB)[6]; \
  (out)[5] = (inA)[1] * (inB)[4] + (inA)[5] * (inB)[5] + (inA)[9] * (inB)[6]; \
  (out)[6] = (inA)[2] * (inB)[4] + (inA)[6] * (inB)[5] + (inA)[10] * (inB)[6]; \
  (out)[7] = 0; \
  (out)[8] = (inA)[0] * (inB)[8] + (inA)[4] * (inB)[9] + (inA)[8] * (inB)[10]; \
  (out)[9] = (inA)[1] * (inB)[8] + (inA)[5] * (inB)[9] + (inA)[9] * (inB)[10]; \
  (out)[10] = (inA)[2] * (inB)[8] + (inA)[6] * (inB)[9] + (inA)[10] * (inB)[10]; \
  (out)[11] = 0; \
  (out)[12] = (inA)[0] * (inB)[12] + (inA)[4] * (inB)[13] + (inA)[8] * (inB)[14] + (inA)[12]; \
  (out)[13] = (inA)[1] * (inB)[12] + (inA)[5] * (inB)[13] + (inA)[9] * (inB)[14] + (inA)[13]; \
  (out)[14] = (inA)[2] * (inB)[12] + (inA)[6] * (inB)[13] + (inA)[10] * (inB)[14] + (inA)[14]; \
  (out)[15] = 1;

#define Matrix_Inv(out, in) \
  (out)[0] = (in)[0]; \
  (out)[1] = (in)[4]; \
  (out)[2] = (in)[8]; \
  (out)[3] = 0; \
  (out)[4] = (in)[1]; \
  (out)[5] = (in)[5]; \
  (out)[6] = (in)[9]; \
  (out)[7] = 0; \
  (out)[8] = (in)[2]; \
  (out)[9] = (in)[6]; \
  (out)[10] = (in)[10]; \
  (out)[11] = 0; \
  (out)[12] = -((in)[0] * (in)[12] + (in)[1] * (in)[13] + (in)[2] * (in)[14]); \
  (out)[13] = -((in)[4] * (in)[12] + (in)[5] * (in)[13] + (in)[6] * (in)[14]); \
  (out)[14] = -((in)[8] * (in)[12] + (in)[9] * (in)[13] + (in)[10] * (in)[14]); \
  (out)[15] = 1;

#define Matrix_Copy(out, in) \
  (out)[0] = (in)[0]; \
  (out)[1] = (in)[1]; \
  (out)[2] = (in)[2]; \
  (out)[3] = (in)[3]; \
  (out)[4] = (in)[4]; \
  (out)[5] = (in)[5]; \
  (out)[6] = (in)[6]; \
  (out)[7] = (in)[7]; \
  (out)[8] = (in)[8]; \
  (out)[9] = (in)[9]; \
  (out)[10] = (in)[10]; \
  (out)[11] = (in)[11]; \
  (out)[12] = (in)[12]; \
  (out)[13] = (in)[13]; \
  (out)[14] = (in)[14]; \
  (out)[15] = (in)[15];

#define Matrix_Eye(inout) \
  (inout)[0] = 1; \
  (inout)[1] = 0; \
  (inout)[2] = 0; \
  (inout)[3] = 0; \
  (inout)[4] = 0; \
  (inout)[5] = 1; \
  (inout)[6] = 0; \
  (inout)[7] = 0; \
  (inout)[8] = 0; \
  (inout)[9] = 0; \
  (inout)[10] = 1; \
  (inout)[11] = 0; \
  (inout)[12] = 0; \
  (inout)[13] = 0; \
  (inout)[14] = 0; \
  (inout)[15] = 1;

#define Matrix_Multiply_Cumul(inout, inB) \
  { \
    Matrix4x4 out; \
    Matrix_Multiply(out, inout, inB); \
    Matrix_Copy(inout, out); \
  }


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//DECLARATION OF VARIABLES
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/// DHM Table parameters
#define DHM_Alpha 0
#define DHM_A 1
#define DHM_Theta 2
#define DHM_D 3


/// Custom robot base (user frame)
Matrix4x4 Robot_BaseFrame = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };

/// Custom robot tool (tool frame, end of arm tool or TCP)
Matrix4x4 Robot_ToolFrame = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };

/// Robot parameters
/// All robot data is held in a large array
tRobot Robot_Data = { 0 };


//These global variable are also pointers, allowing to put the variables inside the Robot_Data
/// DHM table
float *Robot_Kin_DHM_Table = Robot_Data + 0 * Table_Size;

/// xyzwpr of the base
float *Robot_Kin_Base = Robot_Data + 6 * Table_Size;

/// xyzwpr of the tool
float *Robot_Kin_Tool = Robot_Data + 7 * Table_Size;

float *Robot_JointLimits_Upper = Robot_Data + 8 * Table_Size;

float *Robot_JointLimits_Lower = Robot_Data + 9 * Table_Size;

/// Robot axis senses
float *Robot_Senses = Robot_Data + 10 * Table_Size;

// A value mappings

float *Robot_Kin_DHM_L1 = Robot_Kin_DHM_Table + 0 * Table_Size;
float *Robot_Kin_DHM_L2 = Robot_Kin_DHM_Table + 1 * Table_Size;
float *Robot_Kin_DHM_L3 = Robot_Kin_DHM_Table + 2 * Table_Size;
float *Robot_Kin_DHM_L4 = Robot_Kin_DHM_Table + 3 * Table_Size;
float *Robot_Kin_DHM_L5 = Robot_Kin_DHM_Table + 4 * Table_Size;
float *Robot_Kin_DHM_L6 = Robot_Kin_DHM_Table + 5 * Table_Size;


float &Robot_Kin_DHM_A2(Robot_Kin_DHM_Table[1 * Table_Size + 1]);
float &Robot_Kin_DHM_A3(Robot_Kin_DHM_Table[2 * Table_Size + 1]);
float &Robot_Kin_DHM_A4(Robot_Kin_DHM_Table[3 * Table_Size + 1]);

// D value mappings
float &Robot_Kin_DHM_D1(Robot_Kin_DHM_Table[0 * Table_Size + 3]);
float &Robot_Kin_DHM_D2(Robot_Kin_DHM_Table[1 * Table_Size + 3]);
float &Robot_Kin_DHM_D4(Robot_Kin_DHM_Table[3 * Table_Size + 3]);
float &Robot_Kin_DHM_D6(Robot_Kin_DHM_Table[5 * Table_Size + 3]);

// Theta value mappings (mastering)
float &Robot_Kin_DHM_Theta1(Robot_Kin_DHM_Table[0 * Table_Size + 2]);
float &Robot_Kin_DHM_Theta2(Robot_Kin_DHM_Table[1 * Table_Size + 2]);
float &Robot_Kin_DHM_Theta3(Robot_Kin_DHM_Table[2 * Table_Size + 2]);
float &Robot_Kin_DHM_Theta4(Robot_Kin_DHM_Table[3 * Table_Size + 2]);
float &Robot_Kin_DHM_Theta5(Robot_Kin_DHM_Table[4 * Table_Size + 2]);
float &Robot_Kin_DHM_Theta6(Robot_Kin_DHM_Table[5 * Table_Size + 2]);



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool containsNullByte(float value) {
  const unsigned char *bytes = reinterpret_cast<const unsigned char *>(&value);
  for (size_t i = 0; i < sizeof(float); ++i) {
    if (bytes[i] == 0x00) {
      return true;
    }
  }
  return false;
}

bool isValidResult(float value) {
  // Check for NaN or Inf, which are typical results of invalid operations
  return !std::isnan(value) && !std::isinf(value);
}

void apply_robot_native_configuration(
  const float (&nativeTheta)[ROBOT_nDOFs],
  const float (&nativeAlpha)[ROBOT_nDOFs]
) {
  robot_data_reset();
  const float lowerLimits[ROBOT_nDOFs] = {
    J1axisLimNeg,
    J2axisLimNeg,
    J3axisLimNeg,
    J4axisLimNeg,
    J5axisLimNeg,
    J6axisLimNeg,
  };
  const float upperLimits[ROBOT_nDOFs] = {
    J1axisLimPos,
    J2axisLimPos,
    J3axisLimPos,
    J4axisLimPos,
    J5axisLimPos,
    J6axisLimPos,
  };
  for (int joint = 0; joint < ROBOT_nDOFs; ++joint) {
    float* link = Robot_Kin_DHM_Table + joint * Table_Size;
    link[DHM_Alpha] = nativeAlpha[joint];
    link[DHM_Theta] = nativeTheta[joint];
    link[DHM_A] = DHparams[joint][3];
    link[DHM_D] = DHparams[joint][2];
    Robot_JointLimits_Lower[joint] = lowerLimits[joint];
    Robot_JointLimits_Upper[joint] = upperLimits[joint];
  }
}

bool robot_set_AR() {
  float nativeTheta[ROBOT_nDOFs] = {};
  float nativeAlpha[ROBOT_nDOFs] = {};
  for (int joint = 0; joint < ROBOT_nDOFs; ++joint) {
    if (
      !ar4_protocol::degrees_to_radians(
        DHparams[joint][0],
        nativeTheta[joint]
      )
      || !ar4_protocol::degrees_to_radians(
        DHparams[joint][1],
        nativeAlpha[joint]
      )
    ) {
      return false;
    }
  }

  apply_robot_native_configuration(nativeTheta, nativeAlpha);
  return true;
}

void robot_data_reset() {
  Matrix_Eye(Robot_BaseFrame);
  Matrix_Eye(Robot_ToolFrame);

  for (int i = 0; i < 6; i++) {
    Robot_Kin_Base[i] = 0.0;
  }

  for (int i = 0; i < ROBOT_nDOFs; i++) {
    Robot_Senses[i] = +1.0;
  }
}

// ============================================================================
// EEPROM Configuration
// ============================================================================

// Defaults represent an identity record whose commit marker is absent.
const char *DEFAULT_ROBOT_MODEL = "Unset";
const char *DEFAULT_ROBOT_VERSION = "Unset";
const char *DEFAULT_DRIVER_BOARD = "Unset";
const char *DEFAULT_SERIAL_NUMBER = "Unset";
const char *DEFAULT_ASSET_TAG = "Unset";

String robot_model = DEFAULT_ROBOT_MODEL;
String robot_version = DEFAULT_ROBOT_VERSION;
String driver_board = DEFAULT_DRIVER_BOARD;
String serial_number = DEFAULT_SERIAL_NUMBER;
String asset_tag = DEFAULT_ASSET_TAG;
ar4_protocol::IdentityRecordStatus identity_record_status =
  ar4_protocol::IdentityRecordStatus::kUninitialized;
ar4_protocol::JsonSessionIdentityStatus json_session_identity_status =
  ar4_protocol::JsonSessionIdentityStatus::kPersistenceUnavailable;
char controller_hardware_id[
  ar4_protocol::kControllerHardwareIdCapacity
] = { 0 };
char json_session_id[
  ar4_protocol::kJsonSessionIdentifierLength + 1
] = { 0 };


void use_default_robot_identity() {
  robot_model = DEFAULT_ROBOT_MODEL;
  robot_version = DEFAULT_ROBOT_VERSION;
  driver_board = DEFAULT_DRIVER_BOARD;
  serial_number = DEFAULT_SERIAL_NUMBER;
  asset_tag = DEFAULT_ASSET_TAG;
}

void clear_robot_identity() {
  robot_model = "";
  robot_version = "";
  driver_board = "";
  serial_number = "";
  asset_tag = "";
}

// ============================================================================
// EEPROM Functions
// ============================================================================

void load_robot_id_from_eeprom() {
  char stored_robot_model[ar4_protocol::kIdentityFieldStorageSize] = { 0 };
  char stored_robot_version[ar4_protocol::kIdentityFieldStorageSize] = { 0 };
  char stored_driver_board[ar4_protocol::kIdentityFieldStorageSize] = { 0 };
  char stored_serial_number[ar4_protocol::kIdentityFieldStorageSize] = { 0 };
  char stored_asset_tag[ar4_protocol::kIdentityFieldStorageSize] = { 0 };
  identity_record_status = ar4_protocol::load_identity_record(
    EEPROM,
    stored_robot_model,
    stored_robot_version,
    stored_driver_board,
    stored_serial_number,
    stored_asset_tag
  );

  if (
    identity_record_status
    == ar4_protocol::IdentityRecordStatus::kUninitialized
  ) {
    use_default_robot_identity();
    return;
  }
  if (
    identity_record_status
    != ar4_protocol::IdentityRecordStatus::kValid
  ) {
    identity_record_status = ar4_protocol::IdentityRecordStatus::kCorrupt;
    clear_robot_identity();
    return;
  }

  robot_model = stored_robot_model;
  robot_version = stored_robot_version;
  driver_board = stored_driver_board;
  serial_number = stored_serial_number;
  asset_tag = stored_asset_tag;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//MATRICE OPERATIONS
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//This function returns a 4x4 matrix as an argument (pose) following the modified DH rules for the inputs T rx, T tx, T rz and T tz source : https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters
template<typename T>
void DHM_2_pose(T rx, T tx, T rz, T tz, Matrix4x4 pose) {
  T crx;
  T srx;
  T crz;
  T srz;
  crx = cos(rx);
  srx = sin(rx);
  crz = cos(rz);
  srz = sin(rz);
  pose[0] = crz;
  pose[4] = -srz;
  pose[8] = 0.0;
  pose[12] = tx;
  pose[1] = crx * srz;
  pose[5] = crx * crz;
  pose[9] = -srx;
  pose[13] = -tz * srx;
  pose[2] = srx * srz;
  pose[6] = crz * srx;
  pose[10] = crx;
  pose[14] = tz * crx;
  pose[3] = 0.0;
  pose[7] = 0.0;
  pose[11] = 0.0;
  pose[15] = 1.0;
}


//This function tranforms a coordinate system xyzwpr into a 4x4 matrix and return it as an argument.
template<typename T>
void xyzwpr_2_pose(const T xyzwpr[6], Matrix4x4 pose) {
  T srx;
  T crx;
  T sry;
  T cry;
  T srz;
  T crz;
  T H_tmp;
  srx = sin(xyzwpr[3]);
  crx = cos(xyzwpr[3]);
  sry = sin(xyzwpr[4]);
  cry = cos(xyzwpr[4]);
  srz = sin(xyzwpr[5]);
  crz = cos(xyzwpr[5]);
  pose[0] = cry * crz;
  pose[4] = -cry * srz;
  pose[8] = sry;
  pose[12] = xyzwpr[0];
  H_tmp = crz * srx;
  pose[1] = crx * srz + H_tmp * sry;
  crz *= crx;
  pose[5] = crz - srx * sry * srz;
  pose[9] = -cry * srx;
  pose[13] = xyzwpr[1];
  pose[2] = srx * srz - crz * sry;
  pose[6] = H_tmp + crx * sry * srz;
  pose[10] = crx * cry;
  pose[14] = xyzwpr[2];
  pose[3] = 0.0;
  pose[7] = 0.0;
  pose[11] = 0.0;
  pose[15] = 1.0;
}


/// Calculate the [x,y,z,u,v,w] position with rotation vector for a pose target
template<typename T>
void pose_2_xyzuvw(const Matrix4x4 pose, T out[6]) {
  T sin_angle;
  T angle;
  T vector[3];
  int iidx;
  int vector_tmp;
  signed char b_I[9];
  out[0] = pose[12];
  out[1] = pose[13];
  out[2] = pose[14];
  sin_angle = (((pose[0] + pose[5]) + pose[10]) - 1.0) * 0.5;
  if (sin_angle <= -1.0) {
    sin_angle = -1.0;
  }

  if (sin_angle >= 1.0) {
    sin_angle = 1.0;
  }

  angle = acos(sin_angle);
  if (angle < 1.0E-6) {
    vector[0] = 0.0;
    vector[1] = 0.0;
    vector[2] = 0.0;
  } else {
    sin_angle = sin(angle);
    if (abs(sin_angle) < 1.0E-6) {  //IMPOTANT : cosinus of 90 give a really small number instead of 0, the result is forced back to what it should
      sin_angle = pose[0];
      iidx = 0;
      if (pose[0] < pose[5]) {
        sin_angle = pose[5];
        iidx = 1;
      }

      if (sin_angle < pose[10]) {
        sin_angle = pose[10];
        iidx = 2;
      }

      for (vector_tmp = 0; vector_tmp < 9; vector_tmp++) {
        b_I[vector_tmp] = 0;
      }

      b_I[0] = 1;
      b_I[4] = 1;
      b_I[8] = 1;
      sin_angle = 2.0 * (1.0 + sin_angle);
      if (sin_angle <= 0.0) {
        sin_angle = 0.0;
      } else {
        sin_angle = sqrt(sin_angle);
      }

      vector_tmp = iidx << 2;
      vector[0] = (pose[vector_tmp] + static_cast<T>(b_I[3 * iidx])) / sin_angle;
      vector[1] = (pose[1 + vector_tmp] + static_cast<T>(b_I[1 + 3 * iidx]))
                  / sin_angle;
      vector[2] = (pose[2 + vector_tmp] + static_cast<T>(b_I[2 + 3 * iidx]))
                  / sin_angle;
      angle = M_PI;
    } else {
      sin_angle = 1.0 / (2.0 * sin_angle);
      vector[0] = (pose[6] - pose[9]) * sin_angle;
      vector[1] = (pose[8] - pose[2]) * sin_angle;
      vector[2] = (pose[1] - pose[4]) * sin_angle;
    }
  }

  sin_angle = angle * 180.0 / M_PI;
  out[3] = vector[0] * sin_angle * M_PI / 180.0;
  out[4] = vector[1] * sin_angle * M_PI / 180.0;
  out[5] = vector[2] * sin_angle * M_PI / 180.0;
}


//This function tranforms a coordinate system xyzwpr into a 4x4 matrix using UR euler rules and return it as an argument.
template<typename T>
void xyzuvw_2_pose(const T xyzuvw[6], Matrix4x4 pose) {
  T s;
  T angle;
  T axisunit[3];
  T ex;
  T c;
  T pose_tmp;
  T b_pose_tmp;
  s = sqrt((xyzuvw[3] * xyzuvw[3] + xyzuvw[4] * xyzuvw[4]) + xyzuvw[5] * xyzuvw[5]);
  angle = s * 180.0 / M_PI;
  if (abs(angle) < 1.0E-6) {  //IMPOTANT : cosinus of 90 give a really small number instead of 0, the result is forced back to what it should
    memset(&pose[0], 0, sizeof(T) << 4);
    pose[0] = 1.0;
    pose[5] = 1.0;
    pose[10] = 1.0;
    pose[15] = 1.0;
  } else {
    axisunit[1] = abs(xyzuvw[4]);
    axisunit[2] = abs(xyzuvw[5]);
    ex = abs(xyzuvw[3]);
    if (abs(xyzuvw[3]) < axisunit[1]) {
      ex = axisunit[1];
    }

    if (ex < axisunit[2]) {
      ex = axisunit[2];
    }

    if (ex < 1.0E-6) {  //IMPOTANT : cosinus of 90 give a really small number instead of 0, the result is forced back to what it should
      memset(&pose[0], 0, sizeof(T) << 4);
      pose[0] = 1.0;
      pose[5] = 1.0;
      pose[10] = 1.0;
      pose[15] = 1.0;
    } else {
      axisunit[0] = xyzuvw[3] / s;
      axisunit[1] = xyzuvw[4] / s;
      axisunit[2] = xyzuvw[5] / s;
      s = angle * 3.1415926535897931 / 180.0;
      c = cos(s);
      s = sin(s);
      angle = axisunit[0] * axisunit[0];
      pose[0] = angle + c * (1.0 - angle);
      angle = axisunit[0] * axisunit[1] * (1.0 - c);
      ex = axisunit[2] * s;
      pose[4] = angle - ex;
      pose_tmp = axisunit[0] * axisunit[2] * (1.0 - c);
      b_pose_tmp = axisunit[1] * s;
      pose[8] = pose_tmp + b_pose_tmp;
      pose[1] = angle + ex;
      angle = axisunit[1] * axisunit[1];
      pose[5] = angle + (1.0 - angle) * c;
      angle = axisunit[1] * axisunit[2] * (1.0 - c);
      ex = axisunit[0] * s;
      pose[9] = angle - ex;
      pose[2] = pose_tmp - b_pose_tmp;
      pose[6] = angle + ex;
      angle = axisunit[2] * axisunit[2];
      pose[10] = angle + (1.0 - angle) * c;
      pose[3] = 0.0;
      pose[7] = 0.0;
      pose[11] = 0.0;
      pose[15] = 1.0;
    }
  }

  pose[12] = xyzuvw[0];
  pose[13] = xyzuvw[1];
  pose[14] = xyzuvw[2];
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//FOWARD KINEMATICS
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void SolveFowardKinematics() {

  if (!robot_set_AR()) {
    KinematicError = 1;
    return;
  }

  float target_xyzuvw[6];
  float joints[ROBOT_nDOFs];

  for (int i = 0; i < ROBOT_nDOFs; i++) {
    joints[i] = JangleIn[i];
  }

  forward_kinematics_robot_xyzuvw(joints, target_xyzuvw);

  float external_xyzuvw[ar4_protocol::kCartesianPoseSize];
  if (!ar4_protocol::native_cartesian_pose_to_external(
      target_xyzuvw,
      external_xyzuvw
  )) {
    KinematicError = 1;
    return;
  }

  xyzuvw_Out[0] = external_xyzuvw[0];
  xyzuvw_Out[1] = external_xyzuvw[1];
  xyzuvw_Out[2] = external_xyzuvw[2];
  xyzuvw_Out[3] = external_xyzuvw[3] / M_PI * 180;
  xyzuvw_Out[4] = external_xyzuvw[4] / M_PI * 180;
  xyzuvw_Out[5] = external_xyzuvw[5] / M_PI * 180;
}



template<typename T>
void forward_kinematics_arm(const T *joints, Matrix4x4 pose) {
  xyzwpr_2_pose(Robot_Kin_Base, pose);
  for (int i = 0; i < ROBOT_nDOFs; i++) {
    Matrix4x4 hi;
    float *dhm_i = Robot_Kin_DHM_Table + i * Table_Size;
    T ji_rad = joints[i] * Robot_Senses[i] * M_PI / 180.0;
    DHM_2_pose(dhm_i[0], dhm_i[1], dhm_i[2] + ji_rad, dhm_i[3], hi);
    Matrix_Multiply_Cumul(pose, hi);
  }
  Matrix4x4 tool_pose;
  xyzwpr_2_pose(Robot_Kin_Tool, tool_pose);
  Matrix_Multiply_Cumul(pose, tool_pose);
}


template<typename T>
void forward_kinematics_robot_xyzuvw(const T joints[ROBOT_nDOFs], T target_xyzuvw[6]) {
  Matrix4x4 pose;
  forward_kinematics_robot(joints, pose);
  pose_2_xyzuvw(pose, target_xyzuvw);
}

template<typename T>
void forward_kinematics_robot(const T joints[ROBOT_nDOFs], Matrix4x4 target) {
  Matrix4x4 invBaseFrame;
  Matrix4x4 pose_arm;
  Matrix_Inv(invBaseFrame, Robot_BaseFrame);
  forward_kinematics_arm(joints, pose_arm);
  Matrix_Multiply(target, invBaseFrame, pose_arm);
  Matrix_Multiply_Cumul(target, Robot_ToolFrame);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//REVERSE KINEMATICS
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void JointEstimate() {

  for (int i = 0; i < ROBOT_nDOFs; i++) {
    joints_estimate[i] = JangleIn[i];
  }
}

void SolveInverseKinematics(char wrist_config) {

  float target[6];

  int NumberOfSol = 0;
  int solVal = -1;

  KinematicError = 0;

  JointEstimate();
  if (!ar4_protocol::external_cartesian_pose_to_native_radians(
      xyzuvw_In,
      target
  )) {
    KinematicError = 1;
    return;
  }


  NumberOfSol = ar4_protocol::generate_wrist_solutions(
    SolutionMatrix,
    target,
    joints_estimate,
    Robot_JointLimits_Upper,
    Robot_JointLimits_Lower,
    [](
      const float* solver_target,
      const float* candidate,
      float position_tolerance,
      float rotation_tolerance
    ) {
      Matrix4x4 target_pose;
      Matrix4x4 candidate_pose;
      xyzuvw_2_pose(solver_target, target_pose);
      forward_kinematics_robot(candidate, candidate_pose);
      return ar4_protocol::wrist_pose_matches(
        candidate_pose,
        target_pose,
        position_tolerance,
        rotation_tolerance
      );
    },
    [](const float* solver_target, float* candidate, const float* seed) {
      return inverse_kinematics_robot_xyzuvw<float>(
        solver_target,
        candidate,
        seed
      ) != 0;
    }
  );

  solVal = ar4_protocol::select_wrist_solution(
    SolutionMatrix,
    NumberOfSol,
    joints_estimate,
    wrist_config
  );
  if (solVal < 0) {
    KinematicError = 1;
    return;
  }

  for (int i = 0; i < ROBOT_nDOFs; i++) {
    JangleOut[i] = SolutionMatrix[i][solVal];
  }
}





template<typename T>
int inverse_kinematics_robot(const Matrix4x4 target, T joints[ROBOT_nDOFs], const T *joints_estimate) {
  Matrix4x4 invToolFrame;
  Matrix4x4 pose_arm;
  int nsol;
  Matrix_Inv(invToolFrame, Robot_ToolFrame);
  Matrix_Multiply(pose_arm, Robot_BaseFrame, target);
  Matrix_Multiply_Cumul(pose_arm, invToolFrame);
  if (joints_estimate != nullptr) {
    inverse_kinematics_raw(pose_arm, Robot_Data, joints_estimate, joints, &nsol);
  } else {
    // Warning! This is dangerous if joints does not have a valid/reasonable result
    T joints_approx[6];
    memcpy(joints_approx, joints, ROBOT_nDOFs * sizeof(T));
    inverse_kinematics_raw(pose_arm, Robot_Data, joints_approx, joints, &nsol);
  }
  if (nsol == 0) {
    return 0;
  }

  return 1;
}


template<typename T>
int inverse_kinematics_robot_xyzuvw(const T target_xyzuvw1[6], T joints[ROBOT_nDOFs], const T *joints_estimate) {

  Matrix4x4 pose;
  xyzuvw_2_pose(target_xyzuvw1, pose);
  return inverse_kinematics_robot(pose, joints, joints_estimate);
}


template<typename T>
void inverse_kinematics_raw(const T pose[16], const tRobot DK, const T joints_approx_in[6], T joints[6], int *nsol) {
  int i0;
  T base[16];
  T joints_approx[6];
  T tool[16];
  int i;
  T Hout[16];
  T b_Hout[9];
  T dv0[4];
  bool guard1 = false;
  T make_sqrt;
  T P04[4];
  T q1;
  int i1;
  T c_Hout[16];
  T k2;
  T k1;
  T ai;
  T B;
  T C;
  T s31;
  T c31;
  T q13_idx_2;
  T bb_div_cc;
  T q13_idx_0;
  for (i0 = 0; i0 < 6; i0++) {
    joints_approx[i0] = DK[60 + i0] * joints_approx_in[i0];
  }

  //debug = String(Robot_Data[13]) + " * " + String(Robot_Data[19]) + " * " + String(Robot_Data[21]);

  xyzwpr_2_pose(*(T(*)[6]) & DK[36], base);
  xyzwpr_2_pose(*(T(*)[6]) & DK[42], tool);
  for (i0 = 0; i0 < 4; i0++) {
    i = i0 << 2;
    Hout[i] = base[i0];
    Hout[1 + i] = base[i0 + 4];
    Hout[2 + i] = base[i0 + 8];
    Hout[3 + i] = base[i0 + 12];
  }

  for (i0 = 0; i0 < 3; i0++) {
    i = i0 << 2;
    Hout[3 + i] = 0.0;
    b_Hout[3 * i0] = -Hout[i];
    b_Hout[1 + 3 * i0] = -Hout[1 + i];
    b_Hout[2 + 3 * i0] = -Hout[2 + i];
  }

  for (i0 = 0; i0 < 3; i0++) {
    Hout[12 + i0] = (b_Hout[i0] * base[12] + b_Hout[i0 + 3] * base[13]) + b_Hout[i0 + 6] * base[14];
  }

  for (i0 = 0; i0 < 4; i0++) {
    i = i0 << 2;
    base[i] = tool[i0];
    base[1 + i] = tool[i0 + 4];
    base[2 + i] = tool[i0 + 8];
    base[3 + i] = tool[i0 + 12];
  }

  for (i0 = 0; i0 < 3; i0++) {
    i = i0 << 2;
    base[3 + i] = 0.0;
    b_Hout[3 * i0] = -base[i];
    b_Hout[1 + 3 * i0] = -base[1 + i];
    b_Hout[2 + 3 * i0] = -base[2 + i];
  }

  for (i0 = 0; i0 < 3; i0++) {
    base[12 + i0] = (b_Hout[i0] * tool[12] + b_Hout[i0 + 3] * tool[13]) + b_Hout[i0 + 6] * tool[14];
  }

  dv0[0] = 0.0;
  dv0[1] = 0.0;
  dv0[2] = -DK[33];
  dv0[3] = 1.0;
  for (i0 = 0; i0 < 4; i0++) {
    for (i = 0; i < 4; i++) {
      i1 = i << 2;
      c_Hout[i0 + i1] = ((Hout[i0] * pose[i1] + Hout[i0 + 4] * pose[1 + i1]) + Hout[i0 + 8] * pose[2 + i1]) + Hout[i0 + 12] * pose[3 + i1];
    }

    P04[i0] = 0.0;
    for (i = 0; i < 4; i++) {
      i1 = i << 2;
      make_sqrt = ((c_Hout[i0] * base[i1] + c_Hout[i0 + 4] * base[1 + i1]) + c_Hout[i0 + 8] * base[2 + i1]) + c_Hout[i0 + 12] * base[3 + i1];
      tool[i0 + i1] = make_sqrt;
      P04[i0] += make_sqrt * dv0[i];
    }
  }

  guard1 = false;
  if (DK[9] == 0.0) {
    q1 = atan2(P04[1], P04[0]);
    guard1 = true;
  } else {
    make_sqrt = (P04[0] * P04[0] + P04[1] * P04[1]) - DK[9] * DK[9];
    if (make_sqrt < 0.0) {
      for (i = 0; i < 6; i++) {
        joints[i] = 0.0;
      }

      *nsol = 0;
    } else {
      q1 = atan2(P04[1], P04[0]) - atan2(DK[9], sqrt(make_sqrt));
      guard1 = true;
    }
  }

  if (guard1) {
    k2 = P04[2] - DK[3];
    k1 = (cos(q1) * P04[0] + sin(q1) * P04[1]) - DK[7];
    ai = (((k1 * k1 + k2 * k2) - DK[13] * DK[13]) - DK[21] * DK[21]) - DK[19] * DK[19];
    B = 2.0 * DK[21] * DK[13];
    C = 2.0 * DK[19] * DK[13];
    s31 = 0.0;
    c31 = 0.0;
    if (C == 0.0) {
      s31 = -ai / B;
      make_sqrt = 1.0 - s31 * s31;
      if (make_sqrt >= 0.0) {
        c31 = sqrt(make_sqrt);
      }
    } else {
      q13_idx_2 = C * C;
      bb_div_cc = B * B / q13_idx_2;
      make_sqrt = 2.0 * ai * B / q13_idx_2;
      make_sqrt = make_sqrt * make_sqrt - 4.0 * ((1.0 + bb_div_cc) * (ai * ai / q13_idx_2 - 1.0));
      if (make_sqrt >= 0.0) {
        s31 = (-2.0 * ai * B / q13_idx_2 + sqrt(make_sqrt)) / (2.0 * (1.0 + bb_div_cc));
        c31 = (ai + B * s31) / C;
      }
    }

    if ((make_sqrt >= 0.0) && (abs(s31) <= 1.0)) {
      B = atan2(s31, c31);
      make_sqrt = cos(B);
      ai = sin(B);
      C = (DK[13] - DK[21] * ai) + DK[19] * make_sqrt;
      make_sqrt = DK[21] * make_sqrt + DK[19] * ai;
      q13_idx_0 = q1 + -DK[2];
      k2 = atan2(C * k1 - make_sqrt * k2, C * k2 + make_sqrt * k1) + (-DK[8] - M_PI / 2);
      q13_idx_2 = B + -DK[14];
      bb_div_cc = joints_approx[3] * M_PI / 180.0 - (-DK[20]);
      q1 = q13_idx_0 + DK[2];
      B = k2 + DK[8];
      C = q13_idx_2 + DK[14];
      make_sqrt = B + C;
      s31 = cos(make_sqrt);
      c31 = cos(q1);
      Hout[0] = s31 * c31;
      ai = sin(q1);
      Hout[4] = s31 * ai;
      make_sqrt = sin(make_sqrt);
      Hout[8] = -make_sqrt;
      Hout[12] = (DK[3] * make_sqrt - DK[7] * s31) - DK[13] * cos(C);
      Hout[1] = -sin(B + C) * c31;
      Hout[5] = -sin(B + C) * ai;
      Hout[9] = -s31;
      Hout[13] = (DK[3] * s31 + DK[7] * make_sqrt) + DK[13] * sin(C);
      Hout[2] = -ai;
      Hout[6] = c31;
      Hout[10] = 0.0;
      Hout[14] = 0.0;
      Hout[3] = 0.0;
      Hout[7] = 0.0;
      Hout[11] = 0.0;
      Hout[15] = 1.0;
      for (i0 = 0; i0 < 4; i0++) {
        for (i = 0; i < 4; i++) {
          i1 = i << 2;
          base[i0 + i1] = ((Hout[i0] * tool[i1] + Hout[i0 + 4] * tool[1 + i1]) + Hout[i0 + 8] * tool[2 + i1]) + Hout[i0 + 12] * tool[3 + i1];
        }
      }

      make_sqrt = 1.0 - base[9] * base[9];
      if (make_sqrt <= 0.0) {
        make_sqrt = 0.0;
      } else {
        make_sqrt = sqrt(make_sqrt);
      }

      if (make_sqrt < 1.0E-6) {
        C = atan2(make_sqrt, base[9]);
        make_sqrt = sin(bb_div_cc);
        ai = cos(bb_div_cc);
        make_sqrt = atan2(make_sqrt * base[0] + ai * base[2], make_sqrt * base[2] - ai * base[0]);
      } else if (joints_approx[4] >= 0.0) {
        bb_div_cc = atan2(base[10] / make_sqrt, -base[8] / make_sqrt);
        C = atan2(make_sqrt, base[9]);
        make_sqrt = sin(C);
        make_sqrt = atan2(base[5] / make_sqrt, -base[1] / make_sqrt);
      } else {
        bb_div_cc = atan2(-base[10] / make_sqrt, base[8] / make_sqrt);
        C = atan2(-make_sqrt, base[9]);
        make_sqrt = sin(C);
        make_sqrt = atan2(base[5] / make_sqrt, -base[1] / make_sqrt);
      }

      joints[0] = q13_idx_0;
      joints[3] = bb_div_cc + -DK[20];
      joints[1] = k2;
      joints[4] = C + -DK[26];
      joints[2] = q13_idx_2;
      joints[5] = make_sqrt + (-DK[32] + M_PI);
      make_sqrt = joints[5];
      if (joints[5] > 3.1415926535897931) {
        make_sqrt = joints[5] - M_PI * 2;
      } else {
        if (joints[5] <= -M_PI) {
          make_sqrt = joints[5] + M_PI * 2;
        }
      }

      joints[5] = make_sqrt;
      for (i0 = 0; i0 < 6; i0++) {
        joints[i0] = DK[60 + i0] * (joints[i0] * 180.0 / M_PI);
      }

      *nsol = 1.0;
    } else {
      for (i = 0; i < 6; i++) {
        joints[i] = 0.0;
      }

      *nsol = 0;
    }
  }
}




/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//CALCULATE POSITIONS
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void updatePos() {

  JangleIn[0] = (J1StepM - J1zeroStep) / J1StepDeg;
  JangleIn[1] = (J2StepM - J2zeroStep) / J2StepDeg;
  JangleIn[2] = (J3StepM - J3zeroStep) / J3StepDeg;
  JangleIn[3] = (J4StepM - J4zeroStep) / J4StepDeg;
  JangleIn[4] = (J5StepM - J5zeroStep) / J5StepDeg;
  JangleIn[5] = (J6StepM - J6zeroStep) / J6StepDeg;

  J7_pos = (J7StepM - J7zeroStep) / J7StepDeg;
  J8_pos = (J8StepM - J8zeroStep) / J8StepDeg;
  J9_pos = (J9StepM - J9zeroStep) / J9StepDeg;

  SolveFowardKinematics();
}


bool read_configured_encoder_steps(
  int (&output)[ROBOT_nDOFs],
  uint8_t &failureMask
);
void latch_encoder_conversion_fault(uint8_t failureMask);

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//SD CARD
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


static bool sd_ok = false;
static char mounted_sd_media_id[
  ar4_protocol::kControllerMediaIdCapacity
] = { 0 };

bool readSDMediaId(
  char (&output)[ar4_protocol::kControllerMediaIdCapacity]
) {
  cid_t cid = {};
  if (SD.sdfs.card() == nullptr || !SD.sdfs.card()->readCID(&cid)) {
    sd_ok = false;
    return false;
  }
  if (!ar4_protocol::format_controller_media_id(
      reinterpret_cast<const uint8_t *>(&cid),
      sizeof(cid),
      output,
      sizeof(output)
  )) {
    sd_ok = false;
    return false;
  }
  return true;
}

bool sameSDMediaId(
  const char (&left)[ar4_protocol::kControllerMediaIdCapacity],
  const char (&right)[ar4_protocol::kControllerMediaIdCapacity]
) {
  for (
    size_t index = 0;
    index < ar4_protocol::kControllerMediaIdCapacity;
    ++index
  ) {
    if (left[index] != right[index]) return false;
  }
  return true;
}

bool copyMountedSDMediaId(
  char (&output)[ar4_protocol::kControllerMediaIdCapacity]
) {
  if (!sd_ok) return false;
  for (
    size_t index = 0;
    index < ar4_protocol::kControllerMediaIdCapacity;
    ++index
  ) {
    output[index] = mounted_sd_media_id[index];
  }
  return true;
}

bool initSD() {
  char current_media_id[
    ar4_protocol::kControllerMediaIdCapacity
  ] = { 0 };
  if (
    sd_ok
    && readSDMediaId(current_media_id)
    && sameSDMediaId(current_media_id, mounted_sd_media_id)
  ) {
    return true;
  }

  sd_ok = false;
  if (!SD.begin(BUILTIN_SDCARD)) {
    return false;
  }

  char remounted_media_id[
    ar4_protocol::kControllerMediaIdCapacity
  ] = { 0 };
  if (!readSDMediaId(remounted_media_id)) return false;
  for (
    size_t index = 0;
    index < ar4_protocol::kControllerMediaIdCapacity;
    ++index
  ) {
    mounted_sd_media_id[index] = remounted_media_id[index];
  }
  sd_ok = true;
  return true;
}

bool verifyExpectedSDMediaId(const String &expected_media_id) {
  if (
    !ar4_protocol::valid_controller_media_id(
      expected_media_id,
      0,
      static_cast<int>(expected_media_id.length())
    )
  ) {
    return false;
  }
  char current_media_id[
    ar4_protocol::kControllerMediaIdCapacity
  ] = { 0 };
  char mounted_media_id[
    ar4_protocol::kControllerMediaIdCapacity
  ] = { 0 };
  if (
    !copyMountedSDMediaId(mounted_media_id)
    || expected_media_id != mounted_media_id
  ) {
    return false;
  }
  if (!readSDMediaId(current_media_id)) return false;
  if (expected_media_id != current_media_id) {
    return false;
  }
  return true;
}

bool writeSD(
  const String &filename,
  const String &info,
  const String &expected_media_id
) {
  if (!initSD()) return false;
  if (!verifyExpectedSDMediaId(expected_media_id)) return false;

  File f = SD.open(filename.c_str(), FILE_WRITE);
  if (!f) {
    sd_ok = false;
    return false;
  }
  const size_t written = f.println(info);
  f.flush();
  const bool succeeded = written == info.length() + 2
    && f.getWriteError() == 0;
  f.close();
  if (!succeeded) {
    sd_ok = false;
    return false;
  }
  return verifyExpectedSDMediaId(expected_media_id);
}

bool deleteSD(
  const String &filename,
  const String &expected_media_id
) {
  if (!initSD()) return false;
  if (!verifyExpectedSDMediaId(expected_media_id)) return false;
  if (!SD.remove(filename.c_str())) {
    sd_ok = false;
    return false;
  }
  return verifyExpectedSDMediaId(expected_media_id);
}

ar4_protocol::SDFileLookupStatus findSDFile(const String &filename) {
  FsFile root = SD.sdfs.open("/");
  if (!root) {
    sd_ok = false;
    return ar4_protocol::SDFileLookupStatus::kError;
  }

  while (true) {
    FsFile entry = root.openNextFile();
    if (!entry) {
      const bool read_failed = root.getError() != 0;
      root.close();
      if (read_failed) {
        sd_ok = false;
        return ar4_protocol::SDFileLookupStatus::kError;
      }
      return ar4_protocol::SDFileLookupStatus::kAbsent;
    }

    char entry_name[
      ar4_protocol::kControllerFilenameMaxLength + 1
    ] = { 0 };
    size_t entry_name_length = 0;
    const bool entry_name_read =
      ar4_protocol::read_controller_directory_entry_name(
        entry,
        entry_name,
        sizeof(entry_name),
        entry_name_length
      );
    const bool is_directory = entry.isDirectory();
    entry.close();
    if (
      !entry_name_read
      || entry_name_length
        > ar4_protocol::kControllerFilenameMaxLength
    ) {
      root.close();
      sd_ok = false;
      return ar4_protocol::SDFileLookupStatus::kError;
    }
    if (
      !is_directory
      && ar4_protocol::controller_filenames_equal_ignore_case(
        entry_name,
        filename.c_str()
      )
    ) {
      root.close();
      return ar4_protocol::SDFileLookupStatus::kPresent;
    }
  }
}

ar4_protocol::StoredRowReadStatus read_stored_command_row(
  File &file,
  String &row
) {
  row = "";
  while (file.available()) {
    const ar4_protocol::StoredRowReadStatus status =
      ar4_protocol::append_stored_row_byte(row, file.read());
    if (status != ar4_protocol::StoredRowReadStatus::kPending) {
      return status;
    }
  }
  return ar4_protocol::finish_stored_row(row);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//DRIVE LIMIT
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool driveLimit(
  const int steps[],
  const int requested[],
  float SpeedVal
) {

  if (
    !isfinite(SpeedVal)
    || !isfinite(minSpeedDelay)
    || SpeedVal <= 0.0f
    || minSpeedDelay <= 0.0f
  ) {
    return false;
  }
  unsigned long firstActiveUs[numJoints] = { 0 };

  int calcStepGap = minSpeedDelay / (SpeedVal / 100);
  if (calcStepGap <= 0) return false;

  // Define arrays for calibration directions, motor directions, and direction pins
  int calDir[numJoints] = { J1CalDir, J2CalDir, J3CalDir, J4CalDir, J5CalDir, J6CalDir, J7CalDir, J8CalDir, J9CalDir };
  int motDir[numJoints] = { J1MotDir, J2MotDir, J3MotDir, J4MotDir, J5MotDir, J6MotDir, J7MotDir, J8MotDir, J9MotDir };
  int dirPins[numJoints] = { J1dirPin, J2dirPin, J3dirPin, J4dirPin, J5dirPin, J6dirPin, J7dirPin, J8dirPin, J9dirPin };

  // Define arrays for current state, calibration pins, step pins, completion status, and steps done
  int curState[numJoints] = { 0 };
  int calPins[numJoints] = { J1calPin, J2calPin, J3calPin, J4calPin, J5calPin, J6calPin, J7calPin, J8calPin, J9calPin };
  int stepPins[numJoints] = { J1stepPin, J2stepPin, J3stepPin, J4stepPin, J5stepPin, J6stepPin, J7stepPin, J8stepPin, J9stepPin };
  int *stepMonitors[numJoints] = {
    &J1StepM, &J2StepM, &J3StepM, &J4StepM, &J5StepM,
    &J6StepM, &J7StepM, &J8StepM, &J9StepM,
  };

  int stepsDone[numJoints] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  int complete[numJoints] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  int limitConfirmed[numJoints] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  // Once the sensor is first seen, stop that axis immediately.
  // Debounce confirms the sensor while the axis is stationary.
  int limitSeen[numJoints] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  for (int i = 0; i < numJoints; i++) {
    if ((calDir[i] == 1 && motDir[i] == 1) || (calDir[i] == 0 && motDir[i] == 0)) {
      digitalWrite(dirPins[i], HIGH);
    } else {
      digitalWrite(dirPins[i], LOW);
    }

    // Make sure step pins start from a known idle state
    digitalWrite(stepPins[i], LOW);
  }

  int64_t progressStepBudget = 0;
  for (int i = 0; i < numJoints; i++) {
    if (requested[i] != 0 && requested[i] != 1) return false;
    // Unrequested joints must not participate in completion or switch checks.
    if (requested[i] == 0) {
      complete[i] = 1;
    } else {
      if (steps[i] < 0) return false;
      progressStepBudget += steps[i];
    }
  }

  uint64_t maximumIterations = 0;
  if (!ar4_protocol::calibration_stage_maximum_iterations(
      progressStepBudget,
      ar4_protocol::kCalibrationLimitSearchLoopDelayMicroseconds,
      maximumIterations
  )) {
    return false;
  }
  uint64_t iterations = 0;

  // DRIVE MOTORS FOR CALIBRATION
  int DriveLimInProc = 1;

  while (DriveLimInProc == 1 && estopActive == false) {
    if (iterations >= maximumIterations) return false;
    ++iterations;

    for (int i = 0; i < numJoints; i++) {

      if (complete[i] == 1) {
        continue;
      }

      curState[i] = digitalRead(calPins[i]);

      // Debounced limit detection, but stop immediately on first detection
      if (ar4_protocol::calibration_switch_is_active(
          curState[i],
          calibrationLimitSensor[i]
      )) {

        if (limitSeen[i] == 0) {
          firstActiveUs[i] = micros();
          limitSeen[i] = 1;  // stop stepping this axis immediately
        }

        if (
          (micros() - firstActiveUs[i])
            >= ar4_protocol::kCalibrationSwitchStableMicroseconds
        ) {
          complete[i] = 1;
          limitConfirmed[i] = 1;
        }

      } else {
        firstActiveUs[i] = 0;
        limitSeen[i] = 0;
      }

      // Step the motor only if the limit has not been seen yet
      if (stepsDone[i] < steps[i] && complete[i] == 0 && limitSeen[i] == 0) {
        const int logicalIncrement = calDir[i] == 1 ? 1 : -1;
        if (
          (logicalIncrement > 0
            && *stepMonitors[i] == std::numeric_limits<int>::max())
          || (logicalIncrement < 0
            && *stepMonitors[i] == std::numeric_limits<int>::min())
        ) {
          return false;
        }
        digitalWrite(stepPins[i], HIGH);
        delayMicroseconds(ar4_protocol::kCalibrationStepPulseMicroseconds);
        digitalWrite(stepPins[i], LOW);

        *stepMonitors[i] += logicalIncrement;
        stepsDone[i]++;

        delayMicroseconds(calcStepGap);

      } else if (
        stepsDone[i] >= steps[i]
        && complete[i] == 0
        && limitSeen[i] == 0
      ) {
        // Steps exceeded, sensor never triggered
        complete[i] = 1;
      }
    }

    // Check if all joints are complete
    int allComplete = 1;

    for (int i = 0; i < numJoints; i++) {
      if (complete[i] == 0) {
        allComplete = 0;
        break;
      }
    }

    if (allComplete == 1) {
      DriveLimInProc = 0;
    }

    delayMicroseconds(
      ar4_protocol::kCalibrationLimitSearchLoopDelayMicroseconds
    );
  }

  if (estopActive) return false;
  for (int i = 0; i < numJoints; ++i) {
    if (requested[i] == 1 && limitConfirmed[i] == 0) return false;
  }
  return true;
}


bool backOff(uint8_t J1req, uint8_t J2req, uint8_t J3req, uint8_t J4req, uint8_t J5req,
             uint8_t J6req, uint8_t J7req, uint8_t J8req, uint8_t J9req,
             float SpeedVal,
             float maximumBackoffTravel) {

  if (
    !isfinite(SpeedVal)
    || !isfinite(minSpeedDelay)
    || SpeedVal <= 0.0f
    || minSpeedDelay <= 0.0f
  ) {
    return false;
  }
  int calcStepGap = minSpeedDelay / (SpeedVal / 100);
  if (calcStepGap <= 0) return false;

  // SET DIRECTIONS
  digitalWrite(J1dirPin, (J1CalDir == J1MotDir) ? LOW : HIGH);
  digitalWrite(J2dirPin, (J2CalDir == J2MotDir) ? LOW : HIGH);
  digitalWrite(J3dirPin, (J3CalDir == J3MotDir) ? LOW : HIGH);
  digitalWrite(J4dirPin, (J4CalDir == J4MotDir) ? LOW : HIGH);
  digitalWrite(J5dirPin, (J5CalDir == J5MotDir) ? LOW : HIGH);
  digitalWrite(J6dirPin, (J6CalDir == J6MotDir) ? LOW : HIGH);
  digitalWrite(J7dirPin, (J7CalDir == J7MotDir) ? LOW : HIGH);
  digitalWrite(J8dirPin, (J8CalDir == J8MotDir) ? LOW : HIGH);
  digitalWrite(J9dirPin, (J9CalDir == J9MotDir) ? LOW : HIGH);

  // Make sure step pins start LOW
  digitalWrite(J1stepPin, LOW);
  digitalWrite(J2stepPin, LOW);
  digitalWrite(J3stepPin, LOW);
  digitalWrite(J4stepPin, LOW);
  digitalWrite(J5stepPin, LOW);
  digitalWrite(J6stepPin, LOW);
  digitalWrite(J7stepPin, LOW);
  digitalWrite(J8stepPin, LOW);
  digitalWrite(J9stepPin, LOW);

  const uint8_t requested[numJoints] = {
    J1req, J2req, J3req, J4req, J5req,
    J6req, J7req, J8req, J9req,
  };
  const int calPins[numJoints] = {
    J1calPin, J2calPin, J3calPin, J4calPin, J5calPin,
    J6calPin, J7calPin, J8calPin, J9calPin,
  };
  const int stepPins[numJoints] = {
    J1stepPin, J2stepPin, J3stepPin, J4stepPin, J5stepPin,
    J6stepPin, J7stepPin, J8stepPin, J9stepPin,
  };
  const int calibrationDirections[numJoints] = {
    J1CalDir, J2CalDir, J3CalDir, J4CalDir, J5CalDir,
    J6CalDir, J7CalDir, J8CalDir, J9CalDir,
  };
  int *stepMonitors[numJoints] = {
    &J1StepM, &J2StepM, &J3StepM, &J4StepM, &J5StepM,
    &J6StepM, &J7StepM, &J8StepM, &J9StepM,
  };
  const float stepsPerUnit[numJoints] = {
    J1StepDeg, J2StepDeg, J3StepDeg, J4StepDeg, J5StepDeg,
    J6StepDeg, J7StepDeg, J8StepDeg, J9StepDeg,
  };
  const int configuredStepLimits[numJoints] = {
    J1StepLim, J2StepLim, J3StepLim, J4StepLim, J5StepLim,
    J6StepLim, J7StepLim, J8StepLim, J9StepLim,
  };
  int maximumSteps[numJoints] = { 0 };
  int completedSteps[numJoints] = { 0 };
  bool releaseCandidate[numJoints] = { false };
  bool releaseConfirmed[numJoints] = { false };
  unsigned long releaseStarted[numJoints] = { 0 };

  int64_t progressStepBudget = 0;
  for (int axis = 0; axis < numJoints; ++axis) {
    if (requested[axis] == 0) {
      releaseConfirmed[axis] = true;
      continue;
    }
    if (!ar4_protocol::calibration_release_step_limit(
        stepsPerUnit[axis],
        maximumBackoffTravel,
        configuredStepLimits[axis],
        maximumSteps[axis]
    )) {
      return false;
    }
    progressStepBudget += maximumSteps[axis];
  }

  uint64_t maximumIterations = 0;
  if (!ar4_protocol::calibration_stage_maximum_iterations(
      progressStepBudget,
      static_cast<uint32_t>(calcStepGap),
      maximumIterations
  )) {
    return false;
  }
  uint64_t iterations = 0;

  auto pulseStep = [](int pin) {
    digitalWrite(pin, HIGH);
    delayMicroseconds(ar4_protocol::kCalibrationStepPulseMicroseconds);
    digitalWrite(pin, LOW);
  };

  while (true) {
    if (estopActive) return false;
    if (iterations >= maximumIterations) return false;
    ++iterations;
    bool allReleased = true;
    const unsigned long now = micros();
    for (int axis = 0; axis < numJoints; ++axis) {
      if (releaseConfirmed[axis]) continue;
      allReleased = false;
      if (ar4_protocol::calibration_switch_is_released(
          digitalRead(calPins[axis]),
          calibrationLimitSensor[axis]
      )) {
        if (!releaseCandidate[axis]) {
          releaseCandidate[axis] = true;
          releaseStarted[axis] = now;
        } else if (
          (now - releaseStarted[axis])
          >= ar4_protocol::kCalibrationSwitchStableMicroseconds
        ) {
          releaseConfirmed[axis] = true;
        }
        continue;
      }
      releaseCandidate[axis] = false;
      if (completedSteps[axis] >= maximumSteps[axis]) return false;
      const int logicalIncrement = calibrationDirections[axis] == 1 ? -1 : 1;
      if (
        (logicalIncrement > 0
          && *stepMonitors[axis] == std::numeric_limits<int>::max())
        || (logicalIncrement < 0
          && *stepMonitors[axis] == std::numeric_limits<int>::min())
      ) {
        return false;
      }
      pulseStep(stepPins[axis]);
      *stepMonitors[axis] += logicalIncrement;
      ++completedSteps[axis];
    }
    if (allReleased) return true;
    delayMicroseconds(calcStepGap);
  }
}




/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//CHECK ENCODERS
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


bool read_configured_encoder_steps(
  int (&output)[ROBOT_nDOFs],
  uint8_t &failureMask
) {
  const int32_t encoderCounts[ROBOT_nDOFs] = {
    J1encPos.read(), J2encPos.read(), J3encPos.read(),
    J4encPos.read(), J5encPos.read(), J6encPos.read(),
  };
  const float encoderScales[ROBOT_nDOFs] = {
    J1encMult, J2encMult, J3encMult,
    J4encMult, J5encMult, J6encMult,
  };
  int staged[ROBOT_nDOFs] = {};
  failureMask = 0;
  for (int axis = 0; axis < ROBOT_nDOFs; ++axis) {
    if (!ar4_protocol::configured_encoder_count_to_step(
        encoderCounts[axis],
        encoderScales[axis],
        staged[axis]
    )) {
      failureMask |= static_cast<uint8_t>(1u << axis);
    }
  }
  if (failureMask != 0) return false;
  for (int axis = 0; axis < ROBOT_nDOFs; ++axis) {
    output[axis] = staged[axis];
  }
  return true;
}


bool build_configured_encoder_counts(
  int32_t (&output)[ROBOT_nDOFs],
  uint8_t &failureMask
) {
  const int stepMonitors[ROBOT_nDOFs] = {
    J1StepM, J2StepM, J3StepM, J4StepM, J5StepM, J6StepM,
  };
  const float encoderScales[ROBOT_nDOFs] = {
    J1encMult, J2encMult, J3encMult,
    J4encMult, J5encMult, J6encMult,
  };
  int32_t staged[ROBOT_nDOFs] = {};
  failureMask = 0;
  for (int axis = 0; axis < ROBOT_nDOFs; ++axis) {
    if (!ar4_protocol::configured_step_to_encoder_count(
        stepMonitors[axis],
        encoderScales[axis],
        staged[axis]
    )) {
      failureMask |= static_cast<uint8_t>(1u << axis);
    }
  }
  if (failureMask != 0) return false;
  for (int axis = 0; axis < ROBOT_nDOFs; ++axis) {
    output[axis] = staged[axis];
  }
  return true;
}


void latch_encoder_conversion_fault(uint8_t failureMask) {
  int *collisionFlags[ROBOT_nDOFs] = {
    &J1collisionTrue, &J2collisionTrue, &J3collisionTrue,
    &J4collisionTrue, &J5collisionTrue, &J6collisionTrue,
  };
  for (int axis = 0; axis < ROBOT_nDOFs; ++axis) {
    if ((failureMask & static_cast<uint8_t>(1u << axis)) != 0) {
      *collisionFlags[axis] = 1;
    }
  }
  TotalCollision = J1collisionTrue + J2collisionTrue + J3collisionTrue
    + J4collisionTrue + J5collisionTrue + J6collisionTrue;
  flag = "EC" + String(J1collisionTrue) + String(J2collisionTrue)
    + String(J3collisionTrue) + String(J4collisionTrue)
    + String(J5collisionTrue) + String(J6collisionTrue);
}


bool resetEncoders() {

  int32_t encoderCounts[ROBOT_nDOFs] = {};
  uint8_t conversionFailureMask = 0;
  if (!build_configured_encoder_counts(
      encoderCounts,
      conversionFailureMask
  )) {
    latch_encoder_conversion_fault(conversionFailureMask);
    return false;
  }

  J1collisionTrue = 0;
  J2collisionTrue = 0;
  J3collisionTrue = 0;
  J4collisionTrue = 0;
  J5collisionTrue = 0;
  J6collisionTrue = 0;
  TotalCollision = 0;

  //set encoders to current position
  J1encPos.write(encoderCounts[0]);
  J2encPos.write(encoderCounts[1]);
  J3encPos.write(encoderCounts[2]);
  J4encPos.write(encoderCounts[3]);
  J5encPos.write(encoderCounts[4]);
  J6encPos.write(encoderCounts[5]);
  //delayMicroseconds(5);
  return true;
}

bool checkEncoders() {
  //read encoders
  int encoderSteps[ROBOT_nDOFs] = {};
  uint8_t conversionFailureMask = 0;
  if (!read_configured_encoder_steps(
      encoderSteps,
      conversionFailureMask
  )) {
    latch_encoder_conversion_fault(conversionFailureMask);
    return false;
  }
  J1EncSteps = encoderSteps[0];
  J2EncSteps = encoderSteps[1];
  J3EncSteps = encoderSteps[2];
  J4EncSteps = encoderSteps[3];
  J5EncSteps = encoderSteps[4];
  J6EncSteps = encoderSteps[5];

  if (ar4_protocol::encoder_step_difference_reaches_threshold(J1EncSteps, J1StepM, encOffset)) {
    if (JointLoopModes[0] == 0) {
      J1collisionTrue = 1;
      J1StepM = J1EncSteps;
    }
  }
  if (ar4_protocol::encoder_step_difference_reaches_threshold(J2EncSteps, J2StepM, encOffset)) {
    if (JointLoopModes[1] == 0) {
      J2collisionTrue = 1;
      J2StepM = J2EncSteps;
    }
  }
  if (ar4_protocol::encoder_step_difference_reaches_threshold(J3EncSteps, J3StepM, encOffset)) {
    if (JointLoopModes[2] == 0) {
      J3collisionTrue = 1;
      J3StepM = J3EncSteps;
    }
  }
  if (ar4_protocol::encoder_step_difference_reaches_threshold(J4EncSteps, J4StepM, encOffset)) {
    if (JointLoopModes[3] == 0) {
      J4collisionTrue = 1;
      J4StepM = J4EncSteps;
    }
  }
  if (ar4_protocol::encoder_step_difference_reaches_threshold(J5EncSteps, J5StepM, encOffset)) {
    if (JointLoopModes[4] == 0) {
      J5collisionTrue = 1;
      J5StepM = J5EncSteps;
    }
  }
  if (ar4_protocol::encoder_step_difference_reaches_threshold(J6EncSteps, J6StepM, encOffset)) {
    if (JointLoopModes[5] == 0) {
      J6collisionTrue = 1;
      J6StepM = J6EncSteps;
    }
  }

  TotalCollision = J1collisionTrue + J2collisionTrue + J3collisionTrue + J4collisionTrue + J5collisionTrue + J6collisionTrue;
  if (TotalCollision > 0) {
    flag = "EC" + String(J1collisionTrue) + String(J2collisionTrue) + String(J3collisionTrue) + String(J4collisionTrue) + String(J5collisionTrue) + String(J6collisionTrue);
  }
  return true;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//DRIVE MOTORS J
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void store_step_monitors(const int (&stepMonitors)[numJoints]) {
  J1StepM = stepMonitors[0];
  J2StepM = stepMonitors[1];
  J3StepM = stepMonitors[2];
  J4StepM = stepMonitors[3];
  J5StepM = stepMonitors[4];
  J6StepM = stepMonitors[5];
  J7StepM = stepMonitors[6];
  J8StepM = stepMonitors[7];
  J9StepM = stepMonitors[8];
}

bool emit_joint_telemetry() {
  if (
    !telemetryResponseOwnership.active
    || Serial.availableForWrite()
    < ar4_protocol::kJointTelemetryMinimumWriteCapacity
  ) {
    return false;
  }

  const int32_t encoderCounts[ar4_protocol::kJointTelemetryAxisCount] = {
    J1encPos.read(),
    J2encPos.read(),
    J3encPos.read(),
    J4encPos.read(),
    J5encPos.read(),
    J6encPos.read(),
  };
  const float encoderMultipliers[ar4_protocol::kJointTelemetryAxisCount] = {
    J1encMult,
    J2encMult,
    J3encMult,
    J4encMult,
    J5encMult,
    J6encMult,
  };
  const int32_t zeroSteps[ar4_protocol::kJointTelemetryAxisCount] = {
    J1zeroStep,
    J2zeroStep,
    J3zeroStep,
    J4zeroStep,
    J5zeroStep,
    J6zeroStep,
  };
  const float stepsPerDegree[ar4_protocol::kJointTelemetryAxisCount] = {
    J1StepDeg,
    J2StepDeg,
    J3StepDeg,
    J4StepDeg,
    J5StepDeg,
    J6StepDeg,
  };
  int32_t millidegrees[ar4_protocol::kJointTelemetryAxisCount] = {};
  if (!ar4_protocol::encoder_counts_to_joint_millidegrees(
      encoderCounts,
      encoderMultipliers,
      zeroSteps,
      stepsPerDegree,
      millidegrees
  )) {
    return false;
  }

  char frame[ar4_protocol::kJsonJointPositionTelemetryFrameCapacity] = {};
  size_t frameLength = 0;
  const bool frameBuilt =
    ar4_protocol::build_json_joint_position_telemetry_frame(
      jsonJointTelemetrySequence,
      millidegrees,
      frame,
      sizeof(frame),
      frameLength
    );
  const size_t terminalReserve =
    ar4_protocol::kJsonJointMotionTerminalPayloadReservationBytes;
  if (
    !frameBuilt
    || estopActive
    || Serial.availableForWrite()
      < static_cast<int>(frameLength + terminalReserve)
  ) {
    return false;
  }
  const bool written = Serial.write(
    reinterpret_cast<const uint8_t *>(frame),
    frameLength
  ) == frameLength;
  if (written) {
    jsonJointTelemetrySequence =
      jsonJointTelemetrySequence == UINT32_MAX
        ? 0
        : jsonJointTelemetrySequence + 1;
  }
  return written;
}

void begin_telemetry_response_ownership(bool telemetryRequested) {
  noInterrupts();
  ar4_protocol::begin_joint_telemetry_response_ownership(
    telemetryRequested,
    telemetryResponseOwnership
  );
  interrupts();
}

bool driveMotorsJ(int J1step, int J2step, int J3step, int J4step, int J5step, int J6step, int J7step, int J8step, int J9step,
                  int J1dir, int J2dir, int J3dir, int J4dir, int J5dir, int J6dir, int J7dir, int J8dir, int J9dir,
                  String SpeedType, float SpeedVal, float ACCspd, float DCCspd, float ACCramp,
                  FirmwareMotionModeTransaction *motionModes,
                  bool telemetryRequested,
                  const ar4_protocol::JsonLiveMotionContinuationSource *
                    continuationSource = nullptr,
                  bool roundingContinuationEnabled = true) {
  if (!roundingContinuationEnabled) {
    ar4_protocol::consume_motion_rounding_continuation(false, rndTrue);
  }
  if (!ar4_protocol::json_live_motion_continuation_source_valid(
      continuationSource
  )) return false;
  // Array of steps and directions
  int steps[9] = { J1step, J2step, J3step, J4step, J5step, J6step, J7step, J8step, J9step };
  int dirs[9] = { J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, J7dir, J8dir, J9dir };

  // Array of active joints, current steps, PE, SE, LO, and their current states
  int active[9] = { 0 };
  int cur[9] = { 0 };
  int PE[9] = { 0 }, SE_1[9] = { 0 }, SE_2[9] = { 0 }, LO_1[9] = { 0 }, LO_2[9] = { 0 };
  int PEcur[9] = { 0 }, SE_1cur[9] = { 0 }, SE_2cur[9] = { 0 };

  // Array of step and direction pins
  int stepPins[9] = { J1stepPin, J2stepPin, J3stepPin, J4stepPin, J5stepPin, J6stepPin, J7stepPin, J8stepPin, J9stepPin };
  int dirPins[9] = { J1dirPin, J2dirPin, J3dirPin, J4dirPin, J5dirPin, J6dirPin, J7dirPin, J8dirPin, J9dirPin };
  int motDirs[9] = { J1MotDir, J2MotDir, J3MotDir, J4MotDir, J5MotDir, J6MotDir, J7MotDir, J8MotDir, J9MotDir };

  // Initialize step monitors
  int stepMonitors[9] = { J1StepM, J2StepM, J3StepM, J4StepM, J5StepM, J6StepM, J7StepM, J8StepM, J9StepM };

  int HighStep = 0;
  int Jactive = 0;

  // FIND HIGHEST STEP
  for (int i = 0; i < numJoints; i++) {
    if (steps[i] < 0 || (dirs[i] != 0 && dirs[i] != 1)) return false;
    if (steps[i] > HighStep) {
      HighStep = steps[i];
    }
    if (steps[i] >= 1) {
      active[i] = 1;
      Jactive++;
    }
  }
  if (HighStep == 0) return true;

  /////CALC SPEEDS//////
  float calcStepGap = 0.0f;  // cruise delay (µs between highStep ticks)
  double speedSP = 0.0;      // target total time in µs for the move
  if (
    SpeedType.length() != 1
    || !ar4_protocol::valid_motion_profile(
      SpeedType.charAt(0),
      SpeedVal,
      ACCspd,
      DCCspd,
      ACCramp
    )
    || minSpeedDelay <= 0
  ) {
    return false;
  }

  // DETERMINE STEPS
  float ACCStep = HighStep * (ACCspd / 100.0f);
  float DCCStep = HighStep * (DCCspd / 100.0f);
  float NORStep = HighStep - ACCStep - DCCStep;

  // SET SPEED FOR SECONDS OR MM PER SEC
  if (SpeedType == "s") {
    speedSP = static_cast<double>(SpeedVal) * 1000000.0;
  } else if (SpeedType == "m") {
    const double x = static_cast<double>(xyzuvw_In[0]) - xyzuvw_Out[0];
    const double y = static_cast<double>(xyzuvw_In[1]) - xyzuvw_Out[1];
    const double z = static_cast<double>(xyzuvw_In[2]) - xyzuvw_Out[2];
    const double lineDist = sqrt(x * x + y * y + z * z);
    if (!isfinite(lineDist) || lineDist <= 0.0) return false;
    speedSP = lineDist / static_cast<double>(SpeedVal) * 1000000.0;
  }

  // fixed ramp factors (start/end slower than cruise)
  if (ACCramp < 10) {
    ACCramp = 10;
  }
  const float k_acc = ACCramp / 10;
  const float k_dec = ACCramp / 10;

  if (SpeedType == "s" || SpeedType == "m") {
    // Solve cruise delay so total time matches speedSP.
    //
    // Total time T for a trapezoid (linear accel/decel):
    // T = ACCStep * (start+cruise)/2 + NORStep * (cruise) + DCCStep * (cruise+end)/2
    // Let start = k_acc * cruise, end = k_dec * cruise => solve for cruise:
    //
    // T = cruise * [ NORStep + (ACCStep*(1+k_acc) + DCCStep*(1+k_dec))/2 ]
    //
    const double denom = NORStep + (
      ACCStep * (1.0f + k_acc) + DCCStep * (1.0f + k_dec)
    ) * 0.5;

    if (denom <= 0.0f) {
      // Fallback to constant speed if accel+decel consume everything
      calcStepGap = static_cast<float>(speedSP / HighStep);
    } else {
      calcStepGap = static_cast<float>(speedSP / denom);
    }

    if (calcStepGap < minSpeedDelay) {
      calcStepGap = minSpeedDelay;
      speedViolation = "1";
    }
  } else if (SpeedType == "p") {
    // Percentage mode unchanged
    calcStepGap = minSpeedDelay / (SpeedVal / 100.0f);
  }

  // With cruise known, define start/end delays and per-step increments
  float startDelay = calcStepGap * k_acc;  // slower than cruise
  float endDelay = calcStepGap * k_dec;    // slower than cruise
  const bool roundingContinuationSelected =
    roundingContinuationEnabled && rndTrue;
  if (!ar4_protocol::valid_delay_envelope(
      calcStepGap,
      startDelay,
      endDelay,
      roundingContinuationSelected,
      rndSpeed
  )) {
    if (!roundingContinuationEnabled) rndTrue = false;
    return false;
  }
  const bool useRoundingContinuation =
    ar4_protocol::consume_motion_rounding_continuation(
      roundingContinuationEnabled,
      rndTrue
    );
  if (estopActive) return false;
  if (!ar4_protocol::service_json_live_motion_continuation(
      continuationSource
  )) return false;
  if (motionModes != nullptr) motionModes->commit();

  // Timing rejection must precede every output-pin mutation.
  for (int i = 0; i < numJoints; i++) {
    digitalWrite(dirPins[i], dirs[i] == motDirs[i] ? HIGH : LOW);
  }
  delayMicroseconds(15);

  // Linear ramp decrements/increments per step
  float calcACCstepInc = (ACCStep > 0.0f) ? (startDelay - calcStepGap) / ACCStep : 0.0f;  // subtract each step
  float calcDCCstepInc = (DCCStep > 0.0f) ? (endDelay - calcStepGap) / DCCStep : 0.0f;    // add each step

  // Start at the slow end of accel (or keep rounding behavior)
  float calcACCstartDel = startDelay;
  float curDelay = useRoundingContinuation ? rndSpeed : calcACCstartDel;

  ///// DRIVE MOTORS /////
  unsigned long moveStart = micros();
  uint32_t lastTelemetryAttempt = moveStart;
  int highStepCur = 0;

  while ((cur[0] < steps[0] || cur[1] < steps[1] || cur[2] < steps[2] || cur[3] < steps[3] || cur[4] < steps[4] || cur[5] < steps[5] || cur[6] < steps[6] || cur[7] < steps[7] || cur[8] < steps[8]) && estopActive == false) {

    if (!ar4_protocol::service_json_live_motion_continuation(
        continuationSource
    )) {
      store_step_monitors(stepMonitors);
      return false;
    }

    ////DELAY CALC/////
    if (highStepCur < ACCStep) {
      // During accel, move from startDelay down to cruise
      curDelay = fmax(
        calcStepGap,
        curDelay - calcACCstepInc
      );
    } else if (highStepCur >= (HighStep - DCCStep)) {
      // During decel, move from cruise up to endDelay
      curDelay = fmin(
        endDelay,
        curDelay + calcDCCstepInc
      );
    } else {
      curDelay = calcStepGap;  // cruise
    }

    float distDelay = 30;
    float disDelayCur = 0;

    for (int i = 0; i < 9; i++) {
      if (cur[i] < steps[i]) {
        PE[i] = (HighStep / steps[i]);
        LO_1[i] = (HighStep - (steps[i] * PE[i]));
        SE_1[i] = (LO_1[i] > 0) ? (HighStep / LO_1[i]) : 0;
        LO_2[i] = (SE_1[i] > 0) ? (HighStep - ((steps[i] * PE[i]) + ((steps[i] * PE[i]) / SE_1[i]))) : 0;
        SE_2[i] = (LO_2[i] > 0) ? (HighStep / LO_2[i]) : 0;

        if (SE_2[i] == 0) {
          SE_2cur[i] = SE_2[i] + 1;
        }

        if (SE_2cur[i] != SE_2[i]) {
          SE_2cur[i]++;
          if (SE_1[i] == 0) {
            SE_1cur[i] = SE_1[i] + 1;
          }

          if (SE_1cur[i] != SE_1[i]) {
            SE_1cur[i]++;
            PEcur[i]++;

            if (PEcur[i] == PE[i]) {
              cur[i]++;
              PEcur[i] = 0;
              digitalWrite(stepPins[i], LOW);
              delayMicroseconds(distDelay);
              disDelayCur += distDelay;

              if (dirs[i] == 0) {
                stepMonitors[i]--;
              } else {
                stepMonitors[i]++;
              }
            }
          } else {
            SE_1cur[i] = 0;
          }
        } else {
          SE_2cur[i] = 0;
        }
      }
    }

    // Increment current step
    highStepCur++;
    for (int i = 0; i < 9; i++) {
      digitalWrite(stepPins[i], HIGH);
    }

    uint32_t pulseDelay = 0;
    if (!ar4_protocol::pulse_delay_microseconds(
        curDelay,
        disDelayCur,
        minSpeedDelay,
        pulseDelay
    )) {
      store_step_monitors(stepMonitors);
      return false;
    }
    const uint32_t pulseWaitStarted = micros();
    if (
      continuationSource != nullptr
      && !ar4_protocol::service_json_live_motion_continuation(
        continuationSource
      )
    ) {
      store_step_monitors(stepMonitors);
      return false;
    }
    if (
      telemetryRequested
      && json_joint_telemetry_output_available(continuationSource)
      && ar4_protocol::joint_telemetry_due(
        pulseWaitStarted,
        lastTelemetryAttempt
      )
    ) {
      lastTelemetryAttempt = pulseWaitStarted;
      emit_joint_telemetry();
    }
    const uint32_t telemetryWorkMicroseconds =
      static_cast<uint32_t>(micros() - pulseWaitStarted);
    if (continuationSource == nullptr) {
      if (telemetryWorkMicroseconds < pulseDelay) {
        delayMicroseconds(pulseDelay - telemetryWorkMicroseconds);
      }
    } else {
      const ar4_protocol::JsonLiveMotionPulseClockSource pulseClock = {
        json_live_motion_microseconds,
        json_live_motion_delay_microseconds,
        nullptr,
      };
      if (!ar4_protocol::wait_json_live_motion_pulse(
          pulseWaitStarted,
          pulseDelay,
          *continuationSource,
          pulseClock
      )) {
        store_step_monitors(stepMonitors);
        return false;
      }
    }
  }
  unsigned long moveEnd = micros();
  float elapsedSeconds = (moveEnd - moveStart) / 1000000.0f;
  //debug = String(elapsedSeconds);

  // Set rounding speed to last move speed
  rndSpeed = curDelay;

  // Update the original step monitor variables
  store_step_monitors(stepMonitors);
  return true;
}

uint32_t json_live_motion_microseconds(void *context) {
  (void)context;
  return static_cast<uint32_t>(micros());
}

void json_live_motion_delay_microseconds(
  uint32_t durationMicroseconds,
  void *context
) {
  (void)context;
  delayMicroseconds(durationMicroseconds);
}

bool json_joint_telemetry_output_available(
  const ar4_protocol::JsonLiveMotionContinuationSource *continuationSource
) {
  if (continuationSource == nullptr) return true;
  return jsonMainControllerOwner.state()
      == ar4_protocol::JsonMainControllerOwnerState::kIdle
    && !jsonResponseWritePrepared
    && Serial.available() == 0
    && recData.length() == 0
    && !serialFrameDiscarding;
}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//DRIVE MOTORS G
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool driveMotorsG(int J1step, int J2step, int J3step, int J4step, int J5step, int J6step, int J7step, int J8step, int J9step, int J1dir, int J2dir, int J3dir, int J4dir, int J5dir, int J6dir, int J7dir, int J8dir, int J9dir, String SpeedType, float SpeedVal, float ACCspd, float DCCspd, float ACCramp, FirmwareMotionModeTransaction *motionModes) {
  int steps[] = { J1step, J2step, J3step, J4step, J5step, J6step, J7step, J8step, J9step };
  int dirs[] = { J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, J7dir, J8dir, J9dir };
  int motDirs[] = { J1MotDir, J2MotDir, J3MotDir, J4MotDir, J5MotDir, J6MotDir, J7MotDir, J8MotDir, J9MotDir };
  int stepPins[] = { J1stepPin, J2stepPin, J3stepPin, J4stepPin, J5stepPin, J6stepPin, J7stepPin, J8stepPin, J9stepPin };
  int dirPins[] = { J1dirPin, J2dirPin, J3dirPin, J4dirPin, J5dirPin, J6dirPin, J7dirPin, J8dirPin, J9dirPin };
  int stepMonitors[] = { J1StepM, J2StepM, J3StepM, J4StepM, J5StepM, J6StepM, J7StepM, J8StepM, J9StepM };

  // FIND HIGHEST STEP
  int HighStep = 0;
  for (int i = 0; i < numJoints; i++) {
    if (steps[i] < 0 || (dirs[i] != 0 && dirs[i] != 1)) return false;
    if (steps[i] > HighStep) {
      HighStep = steps[i];
    }
  }
  if (HighStep == 0) return !estopActive;

  // FIND ACTIVE JOINTS
  int Jactive = 0;
  for (int i = 0; i < 9; i++) {
    if (steps[i] >= 1) {
      Jactive++;
    }
  }

  // Array of active joints, current steps, PE, SE, LO, and their current states
  int active[9] = { 0 };
  int cur[9] = { 0 };
  int PE[9] = { 0 }, SE_1[9] = { 0 }, SE_2[9] = { 0 }, LO_1[9] = { 0 }, LO_2[9] = { 0 };
  int PEcur[9] = { 0 }, SE_1cur[9] = { 0 }, SE_2cur[9] = { 0 };

  int highStepCur = 0;
  float curDelay = 0;
  double speedSP = 0.0;
  float moveDist;

  ///// CALC SPEEDS /////
  float calcStepGap = 0.0f;
  speedViolation = "0";  // Reset speed violation flag
  if (
    SpeedType.length() != 1
    || !ar4_protocol::valid_motion_profile(
      SpeedType.charAt(0),
      SpeedVal,
      ACCspd,
      DCCspd,
      ACCramp
    )
    || minSpeedDelay <= 0
    || !isfinite(maxMMperSec)
    || maxMMperSec <= 0.0f
  ) {
    return false;
  }

  // Set speed for seconds or mm per sec
  if (SpeedType == "s") {
    speedSP = static_cast<double>(SpeedVal) * 1000000.0 * 1.2;
    calcStepGap = static_cast<float>(speedSP / HighStep);
  } else if (SpeedType == "m") {
    if (SpeedVal >= maxMMperSec) {
      SpeedVal = maxMMperSec;
      speedViolation = "1";
    }
    SpeedVal = ((SpeedVal / maxMMperSec) * 100);
    calcStepGap = minSpeedDelay / (SpeedVal / 100);
  } else if (SpeedType == "p") {
    calcStepGap = minSpeedDelay / (SpeedVal / 100);
  }

  // Ensure calcStepGap is not less than minSpeedDelay
  if (calcStepGap <= minSpeedDelay) {
    calcStepGap = minSpeedDelay;
    speedViolation = "1";
  }
  if (!ar4_protocol::valid_delay_envelope(
      calcStepGap,
      calcStepGap,
      calcStepGap,
      false,
      0.0
  )) {
    return false;
  }
  if (estopActive) return false;
  if (motionModes != nullptr) motionModes->commit();

  // Timing rejection must precede every output-pin mutation.
  for (int i = 0; i < numJoints; i++) {
    digitalWrite(dirPins[i], dirs[i] == motDirs[i] ? HIGH : LOW);
  }
  delayMicroseconds(15);

  ///// DRIVE MOTORS /////
  while ((cur[0] != steps[0] || cur[1] != steps[1] || cur[2] != steps[2] || cur[3] != steps[3] || cur[4] != steps[4] || cur[5] != steps[5] || cur[6] != steps[6] || cur[7] != steps[7] || cur[8] != steps[8]) && estopActive == false) {
    curDelay = calcStepGap;

    float distDelay = 30;
    float disDelayCur = 0;

    for (int i = 0; i < 9; i++) {
      if (cur[i] < steps[i]) {
        PE[i] = (HighStep / steps[i]);
        LO_1[i] = (HighStep - (steps[i] * PE[i]));
        SE_1[i] = LO_1[i] > 0 ? (HighStep / LO_1[i]) : 0;
        LO_2[i] = SE_1[i] > 0 ? HighStep - ((steps[i] * PE[i]) + ((steps[i] * PE[i]) / SE_1[i])) : 0;
        SE_2[i] = LO_2[i] > 0 ? (HighStep / LO_2[i]) : 0;

        if (SE_2[i] == 0) SE_2cur[i] = SE_2[i] + 1;
        if (SE_2cur[i] != SE_2[i]) {
          SE_2cur[i]++;
          if (SE_1[i] == 0) SE_1cur[i] = SE_1[i] + 1;
          if (SE_1cur[i] != SE_1[i]) {
            SE_1cur[i]++;
            PEcur[i]++;
            if (PEcur[i] == PE[i]) {
              cur[i]++;
              PEcur[i] = 0;
              digitalWrite(stepPins[i], LOW);
              delayMicroseconds(distDelay);
              disDelayCur += distDelay;
              stepMonitors[i] += (dirs[i] == 0) ? -1 : 1;
            }
          } else {
            SE_1cur[i] = 0;
          }
        } else {
          SE_2cur[i] = 0;
        }
      }
    }

    highStepCur++;
    for (int i = 0; i < 9; i++) {
      digitalWrite(stepPins[i], HIGH);
    }
    uint32_t pulseDelay = 0;
    if (!ar4_protocol::pulse_delay_microseconds(
        curDelay,
        disDelayCur,
        minSpeedDelay,
        pulseDelay
    )) {
      store_step_monitors(stepMonitors);
      return false;
    }
    delayMicroseconds(pulseDelay);
  }

  // set rounding speed to last move speed
  rndSpeed = curDelay;

  // assign the updated values back to the original step monitors
  store_step_monitors(stepMonitors);
  if (estopActive) return false;
  return true;
}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//DRIVE MOTORS L
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool driveMotorsL(int J1step, int J2step, int J3step, int J4step, int J5step, int J6step, int J7step, int J8step, int J9step, int J1dir, int J2dir, int J3dir, int J4dir, int J5dir, int J6dir, int J7dir, int J8dir, int J9dir, float curDelay, FirmwareMotionModeTransaction *motionModes) {
  // Array of steps, directions, pins, motor directions, and step counters
  int steps[9] = { J1step, J2step, J3step, J4step, J5step, J6step, J7step, J8step, J9step };
  int dirs[9] = { J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, J7dir, J8dir, J9dir };
  int dirPins[9] = { J1dirPin, J2dirPin, J3dirPin, J4dirPin, J5dirPin, J6dirPin, J7dirPin, J8dirPin, J9dirPin };
  int stepPins[9] = { J1stepPin, J2stepPin, J3stepPin, J4stepPin, J5stepPin, J6stepPin, J7stepPin, J8stepPin, J9stepPin };
  int motDirs[9] = { J1MotDir, J2MotDir, J3MotDir, J4MotDir, J5MotDir, J6MotDir, J7MotDir, J8MotDir, J9MotDir };
  int stepMonitors[9] = { J1StepM, J2StepM, J3StepM, J4StepM, J5StepM, J6StepM, J7StepM, J8StepM, J9StepM };

  // Array of active joints, current steps, PE, SE, LO, and their current states
  int active[9] = { 0 };
  int cur[9] = { 0 };
  int PE[9] = { 0 }, SE_1[9] = { 0 }, SE_2[9] = { 0 }, LO_1[9] = { 0 }, LO_2[9] = { 0 };
  int PEcur[9] = { 0 }, SE_1cur[9] = { 0 }, SE_2cur[9] = { 0 };

  // FIND HIGHEST STEP
  int HighStep = 0;
  for (int i = 0; i < numJoints; i++) {
    if (steps[i] < 0 || (dirs[i] != 0 && dirs[i] != 1)) return false;
    if (steps[i] > HighStep) {
      HighStep = steps[i];
    }
  }
  if (HighStep == 0) return !estopActive;
  if (!ar4_protocol::valid_delay_envelope(
      curDelay,
      curDelay,
      curDelay,
      false,
      0.0
  ) || minSpeedDelay <= 0) {
    return false;
  }

  // FIND ACTIVE JOINTS
  for (int i = 0; i < 9; i++) {
    if (steps[i] >= 1) {
      active[i] = 1;
    }
  }

  if (estopActive) return false;
  if (motionModes != nullptr) motionModes->commit();

  // SET DIRECTIONS
  for (int i = 0; i < 9; i++) {
    if (dirs[i] == motDirs[i]) {
      digitalWrite(dirPins[i], HIGH);
    } else {
      digitalWrite(dirPins[i], LOW);
    }
  }

  delayMicroseconds(15);

  int highStepCur = 0;

  // DRIVE MOTORS
  while ((cur[0] < steps[0] || cur[1] < steps[1] || cur[2] < steps[2] || cur[3] < steps[3] || cur[4] < steps[4] || cur[5] < steps[5] || cur[6] < steps[6] || cur[7] < steps[7] || cur[8] < steps[8]) && !estopActive) {
    float distDelay = 30;
    float disDelayCur = 0;

    // Iterate through each joint
    for (int i = 0; i < 9; i++) {
      if (cur[i] < steps[i]) {
        PE[i] = (HighStep / steps[i]);
        LO_1[i] = (HighStep - (steps[i] * PE[i]));
        SE_1[i] = (LO_1[i] > 0) ? (HighStep / LO_1[i]) : 0;
        LO_2[i] = (SE_1[i] > 0) ? HighStep - ((steps[i] * PE[i]) + ((steps[i] * PE[i]) / SE_1[i])) : 0;
        SE_2[i] = (LO_2[i] > 0) ? (HighStep / LO_2[i]) : 0;

        if (SE_2[i] == 0) {
          SE_2cur[i] = 1;
        }
        if (SE_2cur[i] != SE_2[i]) {
          SE_2cur[i]++;
          if (SE_1[i] == 0) {
            SE_1cur[i] = 1;
          }
          if (SE_1cur[i] != SE_1[i]) {
            SE_1cur[i]++;
            PEcur[i]++;
            if (PEcur[i] == PE[i]) {
              cur[i]++;
              PEcur[i] = 0;
              digitalWrite(stepPins[i], LOW);
              delayMicroseconds(distDelay);
              disDelayCur += distDelay;
              stepMonitors[i] += (dirs[i] == 0) ? -1 : 1;
            }
          } else {
            SE_1cur[i] = 0;
          }
        } else {
          SE_2cur[i] = 0;
        }
      }
    }

    // Increment current step
    highStepCur++;
    for (int i = 0; i < 9; i++) {
      digitalWrite(stepPins[i], HIGH);
    }
    uint32_t pulseDelay = 0;
    if (!ar4_protocol::pulse_delay_microseconds(
        curDelay,
        disDelayCur,
        minSpeedDelay,
        pulseDelay
    )) {
      store_step_monitors(stepMonitors);
      return false;
    }
    delayMicroseconds(pulseDelay);
  }

  // Assign the updated values back to the original step monitors
  store_step_monitors(stepMonitors);
  if (estopActive) return false;
  return true;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//MOVE J
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//COMMUNICATIONS
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



const int32_t MODBUS_PARSE_ERROR = -2;

int32_t execute_modbus_operation(
  ar4_protocol::ModbusOperation operation,
  int slave_id,
  int address,
  int value
) {
  if (!ar4_protocol::validate_modbus_request(
      operation, slave_id, address, value
  )) return MODBUS_PARSE_ERROR;
  node = ModbusMaster();
  node.begin(slave_id, Serial8);
  uint8_t result = node.ku8MBInvalidFunction;
  switch (operation) {
    case ar4_protocol::ModbusOperation::kReadCoil:
      result = node.readCoils(address, value);
      break;
    case ar4_protocol::ModbusOperation::kReadDiscreteInput:
      result = node.readDiscreteInputs(address, value);
      break;
    case ar4_protocol::ModbusOperation::kReadHoldingRegisters:
      result = node.readHoldingRegisters(address, value);
      break;
    case ar4_protocol::ModbusOperation::kReadInputRegisters:
      result = node.readInputRegisters(address, value);
      break;
    case ar4_protocol::ModbusOperation::kWriteCoil:
      result = node.writeSingleCoil(address, value);
      break;
    case ar4_protocol::ModbusOperation::kWriteRegister:
      result = node.writeSingleRegister(address, value);
      break;
  }
  if (result != node.ku8MBSuccess) return -1;
  if (
    operation == ar4_protocol::ModbusOperation::kWriteCoil
    || operation == ar4_protocol::ModbusOperation::kWriteRegister
  ) return 1;
  return node.getResponseBuffer(0);
}




/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//READ DATA
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void abandon_json_estop_admission_response() {
  if (!jsonEstopAdmissionResponsePending) return;
  noInterrupts();
  // A failed JSON admission response has not reported the stop. Release only
  // admission gates; the stop latch and controller response owner must survive
  // while the JSON runtime remains fail-closed.
  if (jsonEstopAdmissionTelemetryBlocked) {
    ar4_protocol::clear_joint_telemetry_estop_admission_block(
      telemetryResponseOwnership
    );
  }
  ar4_protocol::abandon_estop_admission_response(
    estopAdmissionOwnership
  );
  jsonEstopAdmissionDecision = {false, 0};
  jsonEstopAdmissionTelemetryBlocked = false;
  jsonEstopAdmissionResponsePending = false;
  interrupts();
}


void clear_json_live_motion_runtime() {
  jsonLiveMotion = {};
}


void latch_json_runtime_fault() {
  clear_json_live_motion_runtime();
  abandon_json_estop_admission_response();
  jsonRuntimeFault = true;
}


void apply_json_process_status(
  ar4_protocol::JsonMainControllerProcessStatus status
) {
  switch (status) {
    case ar4_protocol::JsonMainControllerProcessStatus::kResponseReady:
      return;
    case ar4_protocol::JsonMainControllerProcessStatus::kBusy:
    case ar4_protocol::JsonMainControllerProcessStatus::kControllerFault:
    case ar4_protocol::JsonMainControllerProcessStatus::kSessionFaulted:
      latch_json_runtime_fault();
      return;
  }
  latch_json_runtime_fault();
}


void stage_json_estop_admission_response() {
  noInterrupts();
  if (jsonEstopAdmissionResponsePending) {
    interrupts();
    latch_json_runtime_fault();
    return;
  }
  const bool telemetry_blocked =
    ar4_protocol::joint_telemetry_estop_admission_blocked(
      telemetryResponseOwnership
    );
  const ar4_protocol::EstopAdmissionDecision decision =
    ar4_protocol::begin_estop_admission(
      telemetry_blocked,
      estopActive,
      digitalRead(EstopPin) == LOW,
      estopAdmissionOwnership
    );
  if (decision.blocked) {
    jsonEstopAdmissionDecision = decision;
    jsonEstopAdmissionTelemetryBlocked = telemetry_blocked;
    jsonEstopAdmissionResponsePending = true;
  }
  interrupts();
}


void complete_json_estop_admission_response() {
  if (!jsonEstopAdmissionResponsePending) return;
  noInterrupts();
  if (jsonEstopAdmissionTelemetryBlocked) {
    ar4_protocol::clear_joint_telemetry_estop_admission_block(
      telemetryResponseOwnership
    );
  }
  const bool clear_estop_latch =
    ar4_protocol::complete_estop_admission_response(
      jsonEstopAdmissionDecision,
      digitalRead(EstopPin) == LOW,
      estopAdmissionOwnership,
      controllerResponseOwnership
    );
  jsonEstopAdmissionDecision = {false, 0};
  jsonEstopAdmissionTelemetryBlocked = false;
  jsonEstopAdmissionResponsePending = false;
  if (clear_estop_latch) estopActive = false;
  interrupts();
}


bool controller_mutation_estop_blocked() {
  noInterrupts();
  const bool blocked =
    estopActive
    || digitalRead(EstopPin) == LOW
    || ar4_protocol::joint_telemetry_estop_admission_blocked(
      telemetryResponseOwnership
    );
  interrupts();
  return blocked;
}


bool wait_for_controller_duration(
  uint32_t duration_ms,
  bool include_admission_blocks
);


bool prepare_json_position_snapshot(
  ar4_protocol::JsonMainPositionSnapshot &snapshot
) {
  KinematicError = 0;
  updatePos();
  if (KinematicError != 0) return false;
  const float external_axes[3] = {J7_pos, J8_pos, J9_pos};
  const float cartesian[3] = {
    xyzuvw_Out[0],
    xyzuvw_Out[1],
    xyzuvw_Out[2],
  };
  const float orientation[3] = {
    xyzuvw_Out[3],
    xyzuvw_Out[4],
    xyzuvw_Out[5],
  };
  return ar4_protocol::build_json_main_position_snapshot(
    JangleIn,
    external_axes,
    cartesian,
    orientation,
    snapshot
  );
}


bool prepare_json_position_disposition_fields(
  ar4_protocol::JsonMainPositionResponseSource &source
) {
  source.status = ar4_protocol::JsonMainPositionSourceStatus::kAvailable;
  source.speed_limited = speedViolation == "1";
  if (
    (speedViolation != "0" && speedViolation != "1")
    || debug.length()
      > ar4_protocol::kJsonPositionControllerDebugMaximumLength
    || flag.length() > ar4_protocol::kJsonPositionMotionFaultMaximumLength
  ) {
    source.status =
      ar4_protocol::JsonMainPositionSourceStatus::kDispositionUnavailable;
    return false;
  }
  debug.toCharArray(source.controller_debug, sizeof(source.controller_debug));
  flag.toCharArray(source.motion_fault, sizeof(source.motion_fault));
  if (
    !ar4_protocol::json_position_controller_debug_valid(
      source.controller_debug
    )
    || !ar4_protocol::json_position_motion_fault_valid(source.motion_fault)
  ) {
    source.status =
      ar4_protocol::JsonMainPositionSourceStatus::kDispositionUnavailable;
    return false;
  }
  return true;
}


bool prepare_json_position_source(
  ar4_protocol::JsonMainPositionResponseSource &source
) {
  source = {};
  if (!prepare_json_position_snapshot(source.snapshot)) {
    source.status =
      ar4_protocol::JsonMainPositionSourceStatus::kPositionUnavailable;
    return false;
  }
  if (Alarm != "0") {
    if (
      Alarm.length()
        > ar4_protocol::kJsonPositionControllerAlarmMaximumLength
    ) {
      source.status =
        ar4_protocol::JsonMainPositionSourceStatus::kDispositionUnavailable;
      return true;
    }
    Alarm.toCharArray(
      source.controller_alarm,
      sizeof(source.controller_alarm)
    );
    if (!ar4_protocol::json_position_controller_alarm_valid(
        source.controller_alarm
    )) {
      source.status =
        ar4_protocol::JsonMainPositionSourceStatus::kDispositionUnavailable;
      return true;
    }
    source.status =
      ar4_protocol::JsonMainPositionSourceStatus::kControllerAlarm;
    return true;
  }
  prepare_json_position_disposition_fields(source);
  return true;
}


bool stage_json_external_axis_zero_post_zero_position(
  uint8_t axis,
  ar4_protocol::JsonMainPositionResponseSource &postZeroPosition,
  void *context
) {
  const ar4_protocol::JsonMainPositionResponseSource *const currentPosition =
    static_cast<const ar4_protocol::JsonMainPositionResponseSource *>(context);
  if (
    currentPosition == nullptr
    || currentPosition->status
      != ar4_protocol::JsonMainPositionSourceStatus::kAvailable
    || axis < 7
    || axis > 9
  ) {
    return false;
  }
  const int zeroSteps[3] = {J7zeroStep, J8zeroStep, J9zeroStep};
  const float stepsPerUnit[3] = {J7StepDeg, J8StepDeg, J9StepDeg};
  const size_t index = static_cast<size_t>(axis - 7);
  ar4_protocol::JsonMainPositionResponseSource staged = *currentPosition;
  const float zeroPosition =
    -static_cast<float>(zeroSteps[index]) / stepsPerUnit[index];
  if (!ar4_protocol::scale_json_position_value(
      zeroPosition,
      1000.0,
      staged.snapshot.external_axes_milliunits[index]
  )) {
    return false;
  }
  postZeroPosition = staged;
  return true;
}


ar4_protocol::JsonMainExternalAxisZeroApplyStatus
apply_json_external_axis_zero(uint8_t axis, void *context) {
  (void)context;
  if (axis < 7 || axis > 9) {
    return ar4_protocol::JsonMainExternalAxisZeroApplyStatus::kInvalid;
  }
  int *stepMonitors[3] = {&J7StepM, &J8StepM, &J9StepM};
  noInterrupts();
  const bool estopBlocked =
    estopActive
    || digitalRead(EstopPin) == LOW
    || ar4_protocol::joint_telemetry_estop_admission_blocked(
      telemetryResponseOwnership
    );
  if (!estopBlocked) *stepMonitors[axis - 7] = 0;
  interrupts();
  return estopBlocked
    ? ar4_protocol::JsonMainExternalAxisZeroApplyStatus::kEmergencyStopActive
    : ar4_protocol::JsonMainExternalAxisZeroApplyStatus::kApplied;
}


struct PreparedJsonPositionCorrection {
  bool ready;
  int primarySteps[ROBOT_nDOFs];
  float joints[ROBOT_nDOFs];
  float cartesian[3];
  float orientation[3];
};

PreparedJsonPositionCorrection preparedJsonPositionCorrection = {};


bool prepare_json_corrected_position_snapshot(
  const int (&primarySteps)[ROBOT_nDOFs],
  PreparedJsonPositionCorrection &prepared,
  ar4_protocol::JsonMainPositionSnapshot &snapshot
) {
  prepared = {};
  const int zeroSteps[ROBOT_nDOFs] = {
    J1zeroStep, J2zeroStep, J3zeroStep,
    J4zeroStep, J5zeroStep, J6zeroStep,
  };
  const float stepsPerDegree[ROBOT_nDOFs] = {
    J1StepDeg, J2StepDeg, J3StepDeg,
    J4StepDeg, J5StepDeg, J6StepDeg,
  };
  for (int axis = 0; axis < ROBOT_nDOFs; ++axis) {
    const double scale = static_cast<double>(stepsPerDegree[axis]);
    if (!isfinite(scale) || scale == 0.0) return false;
    const double position = (
      static_cast<double>(primarySteps[axis])
      - static_cast<double>(zeroSteps[axis])
    ) / scale;
    if (
      !isfinite(position)
      || fabs(position) > std::numeric_limits<float>::max()
    ) return false;
    prepared.primarySteps[axis] = primarySteps[axis];
    prepared.joints[axis] = static_cast<float>(position);
  }

  if (!robot_set_AR()) return false;
  float nativePose[6] = {};
  float externalPose[ar4_protocol::kCartesianPoseSize] = {};
  forward_kinematics_robot_xyzuvw(prepared.joints, nativePose);
  if (!ar4_protocol::native_cartesian_pose_to_external(
      nativePose,
      externalPose
  )) return false;
  for (int axis = 0; axis < 3; ++axis) {
    prepared.cartesian[axis] = externalPose[axis];
    prepared.orientation[axis] = externalPose[axis + 3] / M_PI * 180.0f;
  }
  const float externalAxes[3] = {J7_pos, J8_pos, J9_pos};
  if (!ar4_protocol::build_json_main_position_snapshot(
      prepared.joints,
      externalAxes,
      prepared.cartesian,
      prepared.orientation,
      snapshot
  )) return false;
  prepared.ready = true;
  return true;
}


bool prepare_json_position_correction(
  ar4_protocol::JsonMainCorrectPositionResult &result,
  void *context
) {
  PreparedJsonPositionCorrection *prepared =
    static_cast<PreparedJsonPositionCorrection *>(context);
  if (prepared == nullptr) return false;
  *prepared = {};
  result = {};

  int encoderSteps[ROBOT_nDOFs] = {};
  uint8_t failureMask = 0;
  if (!read_configured_encoder_steps(encoderSteps, failureMask)) {
    latch_encoder_conversion_fault(failureMask);
    if (!prepare_json_position_snapshot(result.position.snapshot)) {
      result.outcome =
        ar4_protocol::JsonMainCorrectPositionOutcome::kPositionUnavailable;
      result.position.status =
        ar4_protocol::JsonMainPositionSourceStatus::kPositionUnavailable;
      return true;
    }
    const int encoderFaults[ROBOT_nDOFs] = {
      J1collisionTrue, J2collisionTrue, J3collisionTrue,
      J4collisionTrue, J5collisionTrue, J6collisionTrue,
    };
    result.outcome = ar4_protocol::JsonMainCorrectPositionOutcome::
      kEncoderStateUnavailable;
    result.position.status =
      ar4_protocol::JsonMainPositionSourceStatus::kAvailable;
    for (int axis = 0; axis < ROBOT_nDOFs; ++axis) {
      result.axes[axis] = encoderFaults[axis] != 0;
    }
    return true;
  }

  if (!prepare_json_corrected_position_snapshot(
      encoderSteps,
      *prepared,
      result.position.snapshot
  )) {
    result.outcome =
      ar4_protocol::JsonMainCorrectPositionOutcome::kPositionUnavailable;
    result.position.status =
      ar4_protocol::JsonMainPositionSourceStatus::kPositionUnavailable;
    return true;
  }
  if (!prepare_json_position_disposition_fields(result.position)) {
    *prepared = {};
    result.outcome =
      ar4_protocol::JsonMainCorrectPositionOutcome::kPositionUnavailable;
    result.position.status =
      ar4_protocol::JsonMainPositionSourceStatus::kPositionUnavailable;
    return true;
  }
  result.outcome =
    ar4_protocol::JsonMainCorrectPositionOutcome::kCompleted;
  return true;
}


ar4_protocol::JsonMainCorrectPositionApplyStatus
apply_json_position_correction(void *context) {
  PreparedJsonPositionCorrection *prepared =
    static_cast<PreparedJsonPositionCorrection *>(context);
  if (prepared == nullptr || !prepared->ready) {
    return ar4_protocol::JsonMainCorrectPositionApplyStatus::kInvalid;
  }
  int *stepMonitors[ROBOT_nDOFs] = {
    &J1StepM, &J2StepM, &J3StepM, &J4StepM, &J5StepM, &J6StepM,
  };
  noInterrupts();
  const bool estopBlocked =
    estopActive
    || digitalRead(EstopPin) == LOW
    || ar4_protocol::joint_telemetry_estop_admission_blocked(
      telemetryResponseOwnership
    );
  if (!estopBlocked) {
    // The correction changes no output pins. Keeping the late-stop admission
    // and state replacement in the same short critical section prevents a
    // stop ISR from landing between the safety decision and the logical pose
    // commit.
    for (int axis = 0; axis < ROBOT_nDOFs; ++axis) {
      *stepMonitors[axis] = prepared->primarySteps[axis];
      JangleIn[axis] = prepared->joints[axis];
    }
    for (int axis = 0; axis < 3; ++axis) {
      xyzuvw_Out[axis] = prepared->cartesian[axis];
      xyzuvw_Out[axis + 3] = prepared->orientation[axis];
    }
    KinematicError = 0;
  }
  interrupts();
  *prepared = {};
  if (estopBlocked) {
    return ar4_protocol::JsonMainCorrectPositionApplyStatus::
      kEmergencyStopActive;
  }
  return ar4_protocol::JsonMainCorrectPositionApplyStatus::kApplied;
}


template <typename JsonMotionResult>
bool capture_json_motion_position(JsonMotionResult &result) {
  return prepare_json_position_snapshot(result.position);
}


struct PreparedCalibrationOperation {
  int requested[numJoints];
  int searchSteps[numJoints];
  int masterSteps[numJoints];
  int centerSteps[numJoints];
  int centerDirections[numJoints];
  int32_t primaryHomeReference[
    ar4_protocol::kPrimaryHomeReferenceAxisCount
  ];
};


bool prepare_calibration_operation(
  const ar4_protocol::JsonMainCalibrationParameters &params,
  PreparedCalibrationOperation &prepared,
  bool (&invalidAxes)[numJoints]
) {
  prepared = {};
  for (int axis = 0; axis < numJoints; ++axis) invalidAxes[axis] = false;

  const int stepLimits[numJoints] = {
    J1StepLim, J2StepLim, J3StepLim, J4StepLim, J5StepLim,
    J6StepLim, J7StepLim, J8StepLim, J9StepLim,
  };
  const int calibrationDirections[numJoints] = {
    J1CalDir, J2CalDir, J3CalDir, J4CalDir, J5CalDir,
    J6CalDir, J7CalDir, J8CalDir, J9CalDir,
  };
  const float positiveLimits[numJoints] = {
    J1axisLimPos, J2axisLimPos, J3axisLimPos,
    J4axisLimPos, J5axisLimPos, J6axisLimPos,
    J7axisLimPos, J8axisLimPos, J9axisLimPos,
  };
  const float negativeLimits[numJoints] = {
    J1axisLimNeg, J2axisLimNeg, J3axisLimNeg,
    J4axisLimNeg, J5axisLimNeg, J6axisLimNeg,
    J7axisLimNeg, J8axisLimNeg, J9axisLimNeg,
  };
  const float stepsPerUnit[numJoints] = {
    J1StepDeg, J2StepDeg, J3StepDeg,
    J4StepDeg, J5StepDeg, J6StepDeg,
    J7StepDeg, J8StepDeg, J9StepDeg,
  };
  const float baseOffsets[numJoints] = {
    J1calBaseOff, J2calBaseOff, J3calBaseOff,
    J4calBaseOff, J5calBaseOff, J6calBaseOff,
    J7calBaseOff, J8calBaseOff, J9calBaseOff,
  };
  const int zeroSteps[numJoints] = {
    J1zeroStep, J2zeroStep, J3zeroStep,
    J4zeroStep, J5zeroStep, J6zeroStep,
    J7zeroStep, J8zeroStep, J9zeroStep,
  };
  bool valid = true;
  for (int axis = 0; axis < numJoints; ++axis) {
    prepared.requested[axis] = params.axes[axis] ? 1 : 0;
    int stagedCenterSteps = 0;
    int stagedJointFiveSteps = 0;
    if (!ar4_protocol::calibration_reference_steps(
        prepared.requested[axis],
        calibrationDirections[axis],
        positiveLimits[axis],
        negativeLimits[axis],
        stepsPerUnit[axis],
        stepLimits[axis],
        baseOffsets[axis],
        params.offsets[axis],
        axis == 4,
        prepared.masterSteps[axis],
        stagedCenterSteps,
        stagedJointFiveSteps
    )) {
      invalidAxes[axis] = true;
      valid = false;
      continue;
    }
    prepared.searchSteps[axis] = prepared.requested[axis] == 1
      ? stepLimits[axis]
      : 0;
    prepared.centerSteps[axis] = axis == 4
      ? stagedJointFiveSteps
      : stagedCenterSteps;
    prepared.centerDirections[axis] = calibrationDirections[axis] == 1
      ? 0
      : 1;
    if (
      static_cast<size_t>(axis)
        < ar4_protocol::kPrimaryHomeReferenceAxisCount
      && prepared.requested[axis] == 1
      && !ar4_protocol::primary_parking_reference_from_steps(
        static_cast<size_t>(axis),
        prepared.masterSteps[axis],
        zeroSteps[axis],
        stepsPerUnit[axis],
        negativeLimits[axis],
        positiveLimits[axis],
        prepared.primaryHomeReference[axis]
      )
    ) {
      invalidAxes[axis] = true;
      valid = false;
    }
  }
  return valid;
}


void finish_calibration_execution_failure(
  ar4_protocol::JsonMainCalibrationStage stage,
  ar4_protocol::JsonMainCalibrationExecutionResult &result
) {
  const bool estopBlocked = controller_mutation_estop_blocked();
  result.outcome = estopBlocked
    ? ar4_protocol::JsonMainCalibrationOutcome::kEmergencyStop
    : ar4_protocol::JsonMainCalibrationOutcome::kCalibrationFailed;
  result.stage = estopBlocked
    ? ar4_protocol::JsonMainCalibrationStage::kNone
    : stage;
  if (!capture_json_motion_position(result)) {
    result = {};
    result.outcome =
      ar4_protocol::JsonMainCalibrationOutcome::kPositionUnavailable;
  }
}


bool execute_calibration_operation(
  const ar4_protocol::JsonMainCalibrationParameters &params,
  ar4_protocol::JsonMainCalibrationExecutionResult &result
) {
  result = {};
  ar4_protocol::consume_motion_rounding_continuation(false, rndTrue);
  PreparedCalibrationOperation prepared = {};
  bool invalidAxes[numJoints] = {};
  if (!prepare_calibration_operation(params, prepared, invalidAxes)) {
    result.outcome =
      ar4_protocol::JsonMainCalibrationOutcome::kNotRepresentable;
    for (int axis = 0; axis < numJoints; ++axis) {
      result.axes[axis] = invalidAxes[axis];
    }
    return true;
  }
  if (controller_mutation_estop_blocked()) {
    finish_calibration_execution_failure(
      ar4_protocol::JsonMainCalibrationStage::kFastLimitSearch,
      result
    );
    return true;
  }

  ar4_protocol::PrimaryHomeReferenceState invalidatedHomeReference =
    primaryHomeReference;
  for (
    size_t axis = 0;
    axis < ar4_protocol::kPrimaryHomeReferenceAxisCount;
    ++axis
  ) {
    if (prepared.requested[axis] == 1) {
      ar4_protocol::invalidate_primary_home_reference_axis(
        invalidatedHomeReference,
        axis
      );
    }
  }
  primaryHomeReference = invalidatedHomeReference;
  speedViolation = "0";
  flag = "";

  if (!driveLimit(
      prepared.searchSteps,
      prepared.requested,
      ar4_protocol::kCalibrationFastSearchSpeedPercent
  )) {
    finish_calibration_execution_failure(
      ar4_protocol::JsonMainCalibrationStage::kFastLimitSearch,
      result
    );
    return true;
  }
  if (!backOff(
      prepared.requested[0],
      prepared.requested[1],
      prepared.requested[2],
      prepared.requested[3],
      prepared.requested[4],
      prepared.requested[5],
      prepared.requested[6],
      prepared.requested[7],
      prepared.requested[8],
      ar4_protocol::kCalibrationReleaseSpeedPercent,
      ar4_protocol::kCalibrationReleaseMaximumTravelUnits
  )) {
    finish_calibration_execution_failure(
      ar4_protocol::JsonMainCalibrationStage::kSwitchRelease,
      result
    );
    return true;
  }
  if (!driveLimit(
      prepared.searchSteps,
      prepared.requested,
      ar4_protocol::kCalibrationSlowSearchSpeedPercent
  )) {
    finish_calibration_execution_failure(
      ar4_protocol::JsonMainCalibrationStage::kSlowLimitSearch,
      result
    );
    return true;
  }

  int *stepMonitors[numJoints] = {
    &J1StepM, &J2StepM, &J3StepM, &J4StepM, &J5StepM,
    &J6StepM, &J7StepM, &J8StepM, &J9StepM,
  };
  for (int axis = 0; axis < numJoints; ++axis) {
    if (prepared.requested[axis] == 1) {
      *stepMonitors[axis] = prepared.masterSteps[axis];
    }
  }
  if (!driveMotorsJ(
      prepared.centerSteps[0],
      prepared.centerSteps[1],
      prepared.centerSteps[2],
      prepared.centerSteps[3],
      prepared.centerSteps[4],
      prepared.centerSteps[5],
      prepared.centerSteps[6],
      prepared.centerSteps[7],
      prepared.centerSteps[8],
      prepared.centerDirections[0],
      prepared.centerDirections[1],
      prepared.centerDirections[2],
      prepared.centerDirections[3],
      prepared.centerDirections[4],
      prepared.centerDirections[5],
      prepared.centerDirections[6],
      prepared.centerDirections[7],
      prepared.centerDirections[8],
      "p",
      ar4_protocol::kCalibrationCenterSpeedPercent,
      ar4_protocol::kCalibrationCenterAccelerationPercent,
      ar4_protocol::kCalibrationCenterDecelerationPercent,
      ar4_protocol::kCalibrationCenterRampPercent,
      nullptr,
      false,
      nullptr,
      false
  ) || controller_mutation_estop_blocked()) {
    finish_calibration_execution_failure(
      ar4_protocol::JsonMainCalibrationStage::kCenterMove,
      result
    );
    return true;
  }

  ar4_protocol::PrimaryHomeReferenceState committedHomeReference =
    primaryHomeReference;
  for (
    size_t axis = 0;
    axis < ar4_protocol::kPrimaryHomeReferenceAxisCount;
    ++axis
  ) {
    if (prepared.requested[axis] == 1) {
      ar4_protocol::set_primary_home_reference(
        committedHomeReference,
        axis,
        prepared.primaryHomeReference[axis]
      );
    }
  }
  primaryHomeReference = committedHomeReference;
  result.outcome = ar4_protocol::JsonMainCalibrationOutcome::kCompleted;
  result.speed_limited = speedViolation == "1";
  if (!capture_json_motion_position(result)) {
    result = {};
    result.outcome =
      ar4_protocol::JsonMainCalibrationOutcome::kPositionUnavailable;
  }
  return true;
}


bool execute_json_calibration(
  const ar4_protocol::JsonMainCalibrationParameters &params,
  ar4_protocol::JsonMainCalibrationExecutionResult &result,
  void *context
) {
  if (context != nullptr) return false;
  const bool executed = execute_calibration_operation(params, result);
  speedViolation = "0";
  flag = "";
  return executed;
}


bool complete_json_telemetry_ownership(
  bool telemetryRequested,
  bool driveSucceeded
) {
  if (!telemetryRequested) return true;
  noInterrupts();
  const ar4_protocol::JointTelemetryTerminalDecision decision =
    ar4_protocol::decide_joint_telemetry_terminal(
      telemetryRequested,
      driveSucceeded,
      estopActive,
      telemetryResponseOwnership
    );
  const bool owned =
    decision.kind != ar4_protocol::JointTelemetryTerminalKind::kNotOwned;
  if (owned) {
    ar4_protocol::commit_joint_telemetry_terminal(
      decision,
      telemetryResponseOwnership
    );
  }
  interrupts();
  return owned;
}


enum class PreparedJsonMotionOutcome : uint8_t {
  kCompleted,
  kEmergencyStop,
  kPositionUnavailable,
  kMotionExecutionFailed,
  kEncoderCollision,
  kEncoderStateUnavailable,
};


struct PreparedJsonMotionResult {
  PreparedJsonMotionOutcome outcome;
  bool encoder_axes[ROBOT_nDOFs];
  bool speed_limited;
};


bool execute_prepared_json_motion(
  const int (&futureSteps)[numJoints],
  const String &speedMode,
  float speedValue,
  float accelerationPercent,
  float decelerationPercent,
  float rampPercent,
  const String &wristConfiguration,
  const int (&loopModes)[ROBOT_nDOFs],
  bool telemetryRequested,
  PreparedJsonMotionResult &result,
  const ar4_protocol::JsonLiveMotionContinuationSource *
    continuationSource,
  bool gcodeTiming = false
) {
  result = {};
  if (!ar4_protocol::json_live_motion_continuation_source_valid(
      continuationSource
  )) return false;
  const int currentSteps[numJoints] = {
    J1StepM,
    J2StepM,
    J3StepM,
    J4StepM,
    J5StepM,
    J6StepM,
    J7StepM,
    J8StepM,
    J9StepM,
  };
  const int stepLimits[numJoints] = {
    J1StepLim,
    J2StepLim,
    J3StepLim,
    J4StepLim,
    J5StepLim,
    J6StepLim,
    J7StepLim,
    J8StepLim,
    J9StepLim,
  };
  int moveSteps[numJoints] = {};
  int directions[numJoints] = {};
  for (int axis = 0; axis < numJoints; ++axis) {
    if (
      future_step_is_outside_limit(futureSteps[axis], stepLimits[axis])
      || future_step_is_outside_limit(currentSteps[axis], stepLimits[axis])
    ) {
      result.outcome = PreparedJsonMotionOutcome::kPositionUnavailable;
      return true;
    }
    const int64_t difference =
      static_cast<int64_t>(currentSteps[axis]) - futureSteps[axis];
    const int64_t magnitude = difference < 0 ? -difference : difference;
    if (magnitude > std::numeric_limits<int>::max()) {
      result.outcome = PreparedJsonMotionOutcome::kPositionUnavailable;
      return true;
    }
    moveSteps[axis] = static_cast<int>(magnitude);
    directions[axis] = difference <= 0 ? 1 : 0;
  }

  if (controller_mutation_estop_blocked()) {
    result.outcome = PreparedJsonMotionOutcome::kEmergencyStop;
    return true;
  }

  int32_t encoderCounts[ROBOT_nDOFs] = {};
  uint8_t encoderFailureMask = 0;
  if (!build_configured_encoder_counts(
      encoderCounts,
      encoderFailureMask
  )) {
    latch_encoder_conversion_fault(encoderFailureMask);
    result.outcome = PreparedJsonMotionOutcome::kEncoderStateUnavailable;
    const int encoderFaults[ROBOT_nDOFs] = {
      J1collisionTrue,
      J2collisionTrue,
      J3collisionTrue,
      J4collisionTrue,
      J5collisionTrue,
      J6collisionTrue,
    };
    for (int axis = 0; axis < ROBOT_nDOFs; ++axis) {
      result.encoder_axes[axis] = encoderFaults[axis] != 0;
    }
    return true;
  }

  FirmwareMotionModeTransaction motionModes(
    WristCon,
    JointLoopModes,
    wristConfiguration,
    loopModes
  );
  speedViolation = "0";
  flag = "";
  if (!resetEncoders()) {
    result.outcome = PreparedJsonMotionOutcome::kEncoderStateUnavailable;
    const int encoderFaults[ROBOT_nDOFs] = {
      J1collisionTrue,
      J2collisionTrue,
      J3collisionTrue,
      J4collisionTrue,
      J5collisionTrue,
      J6collisionTrue,
    };
    for (int axis = 0; axis < ROBOT_nDOFs; ++axis) {
      result.encoder_axes[axis] = encoderFaults[axis] != 0;
    }
    flag = "";
    return true;
  }

  begin_telemetry_response_ownership(telemetryRequested);
  const bool driveSucceeded = gcodeTiming ? driveMotorsG(
    moveSteps[0], moveSteps[1], moveSteps[2], moveSteps[3], moveSteps[4],
    moveSteps[5], moveSteps[6], moveSteps[7], moveSteps[8],
    directions[0], directions[1], directions[2], directions[3],
    directions[4], directions[5], directions[6], directions[7],
    directions[8], speedMode, speedValue, accelerationPercent,
    decelerationPercent, rampPercent, &motionModes
  ) : driveMotorsJ(
    moveSteps[0],
    moveSteps[1],
    moveSteps[2],
    moveSteps[3],
    moveSteps[4],
    moveSteps[5],
    moveSteps[6],
    moveSteps[7],
    moveSteps[8],
    directions[0],
    directions[1],
    directions[2],
    directions[3],
    directions[4],
    directions[5],
    directions[6],
    directions[7],
    directions[8],
    speedMode,
    speedValue,
    accelerationPercent,
    decelerationPercent,
    rampPercent,
    &motionModes,
    telemetryRequested,
    continuationSource
  );
  const bool telemetryOwnershipCompleted =
    complete_json_telemetry_ownership(
      telemetryRequested,
      driveSucceeded
    );
  const bool encoderStateAvailable = checkEncoders();
  const bool estopBlocked = controller_mutation_estop_blocked();

  if (estopBlocked) {
    result.outcome = PreparedJsonMotionOutcome::kEmergencyStop;
  } else if (!telemetryOwnershipCompleted || !driveSucceeded) {
    result.outcome = PreparedJsonMotionOutcome::kMotionExecutionFailed;
  } else if (!encoderStateAvailable) {
    result.outcome = PreparedJsonMotionOutcome::kEncoderStateUnavailable;
  } else if (TotalCollision > 0) {
    result.outcome = PreparedJsonMotionOutcome::kEncoderCollision;
  } else {
    result.outcome = PreparedJsonMotionOutcome::kCompleted;
    result.speed_limited = speedViolation == "1";
  }
  if (
    result.outcome == PreparedJsonMotionOutcome::kEncoderCollision
    || result.outcome
      == PreparedJsonMotionOutcome::kEncoderStateUnavailable
  ) {
    const int encoderFaults[ROBOT_nDOFs] = {
      J1collisionTrue,
      J2collisionTrue,
      J3collisionTrue,
      J4collisionTrue,
      J5collisionTrue,
      J6collisionTrue,
    };
    for (int axis = 0; axis < ROBOT_nDOFs; ++axis) {
      result.encoder_axes[axis] = encoderFaults[axis] != 0;
    }
  }
  speedViolation = "0";
  flag = "";
  return true;
}


class MotionKinematicsTransaction {
 public:
  MotionKinematicsTransaction()
    : savedKinematicError_(KinematicError) {
    for (int axis = 0; axis < ROBOT_nDOFs; ++axis) {
      savedCartesianInput_[axis] = xyzuvw_In[axis];
      savedJointOutput_[axis] = JangleOut[axis];
      savedJointEstimate_[axis] = joints_estimate[axis];
      for (
        int solution = 0;
        solution < ar4_protocol::kMaximumWristSolutions;
        ++solution
      ) {
        savedSolutionMatrix_[axis][solution] =
          SolutionMatrix[axis][solution];
      }
    }
  }

  ~MotionKinematicsTransaction() {
    for (int axis = 0; axis < ROBOT_nDOFs; ++axis) {
      xyzuvw_In[axis] = savedCartesianInput_[axis];
      JangleOut[axis] = savedJointOutput_[axis];
      joints_estimate[axis] = savedJointEstimate_[axis];
      for (
        int solution = 0;
        solution < ar4_protocol::kMaximumWristSolutions;
        ++solution
      ) {
        SolutionMatrix[axis][solution] =
          savedSolutionMatrix_[axis][solution];
      }
    }
    KinematicError = savedKinematicError_;
  }

  MotionKinematicsTransaction(
    const MotionKinematicsTransaction &
  ) = delete;
  MotionKinematicsTransaction &operator=(
    const MotionKinematicsTransaction &
  ) = delete;

 private:
  int savedKinematicError_;
  float savedCartesianInput_[ROBOT_nDOFs];
  float savedJointOutput_[ROBOT_nDOFs];
  float savedJointEstimate_[ROBOT_nDOFs];
  float savedSolutionMatrix_[
    ar4_protocol::kWristJointCount
  ][ar4_protocol::kMaximumWristSolutions];
};


class MotionToolFrameTransaction {
 public:
  MotionToolFrameTransaction() {
    for (int axis = 0; axis < ROBOT_nDOFs; ++axis) {
      savedToolFrame_[axis] = Robot_Kin_Tool[axis];
    }
  }

  ~MotionToolFrameTransaction() {
    for (int axis = 0; axis < ROBOT_nDOFs; ++axis) {
      Robot_Kin_Tool[axis] = savedToolFrame_[axis];
    }
  }

  MotionToolFrameTransaction(
    const MotionToolFrameTransaction &
  ) = delete;
  MotionToolFrameTransaction &operator=(
    const MotionToolFrameTransaction &
  ) = delete;

  bool apply_offset(int frame_index, float frame_offset) {
    if (frame_index < 0 || frame_index >= ROBOT_nDOFs) return false;
    float staged = 0.0f;
    if (!ar4_protocol::stage_json_tool_frame_offset(
        Robot_Kin_Tool[frame_index],
        frame_offset,
        staged
    )) {
      return false;
    }
    Robot_Kin_Tool[frame_index] = staged;
    return true;
  }

 private:
  float savedToolFrame_[ROBOT_nDOFs];
};


bool refresh_motion_source_position() {
  KinematicError = 0;
  updatePos();
  return KinematicError == 0;
}


bool prepare_joint_speed_target(
  const float *robotJointsDegrees,
  bool millimetersPerSecond
) {
  if (!millimetersPerSecond) return true;
  if (robotJointsDegrees == nullptr) return false;
  if (!refresh_motion_source_position()) return false;

  float externalTarget[ar4_protocol::kCartesianPoseSize] = {};
  if (!ar4_protocol::joint_speed_target_from_joints(
    robotJointsDegrees,
    &forward_kinematics_robot_xyzuvw<float>,
    externalTarget
  )) {
    return false;
  }
  for (int axis = 0; axis < ROBOT_nDOFs; ++axis) {
    xyzuvw_In[axis] = externalTarget[axis];
  }
  return true;
}


bool execute_json_move_cartesian_with_timing(
  const ar4_protocol::JsonMainMoveCartesianParameters &params,
  ar4_protocol::JsonMainMoveCartesianExecutionResult &result,
  void *context,
  bool gcodeTiming,
  bool capturePosition
) {
  const ar4_protocol::JsonLiveMotionContinuationSource *
    continuationSource = static_cast<
      const ar4_protocol::JsonLiveMotionContinuationSource *
    >(context);
  if (!ar4_protocol::json_live_motion_continuation_source_valid(
      continuationSource
  )) return false;
  result = {};

  // Millimeters-per-second scheduling reads xyzuvw_In during driveMotorsJ;
  // restoration therefore occurs only after every motion exit path settles.
  MotionKinematicsTransaction kinematicsTransaction;
  xyzuvw_In[0] = params.translation_millimeters[0];
  xyzuvw_In[1] = params.translation_millimeters[1];
  xyzuvw_In[2] = params.translation_millimeters[2];
  // Firmware kinematics use Rz, Ry, Rx; JSON uses Rx, Ry, Rz.
  xyzuvw_In[3] = params.orientation_degrees[2];
  xyzuvw_In[4] = params.orientation_degrees[1];
  xyzuvw_In[5] = params.orientation_degrees[0];

  char wristConfiguration = 'A';
  switch (params.wrist_configuration) {
    case ar4_protocol::JsonCartesianMotionWristConfiguration::kAutomatic:
      wristConfiguration = 'A';
      break;
    case ar4_protocol::JsonCartesianMotionWristConfiguration::kNear:
      wristConfiguration = 'N';
      break;
    case ar4_protocol::JsonCartesianMotionWristConfiguration::kFar:
      wristConfiguration = 'F';
      break;
  }
  SolveInverseKinematics(wristConfiguration);
  const bool kinematicsSolved = KinematicError == 0;
  float primaryTargets[ROBOT_nDOFs] = {};
  if (kinematicsSolved) {
    for (int axis = 0; axis < ROBOT_nDOFs; ++axis) {
      primaryTargets[axis] = JangleOut[axis];
    }
  }
  if (!kinematicsSolved) {
    result.outcome =
      ar4_protocol::JsonMainMoveCartesianOutcome::kKinematicsUnreachable;
    return true;
  }

  const float targets[numJoints] = {
    primaryTargets[0],
    primaryTargets[1],
    primaryTargets[2],
    primaryTargets[3],
    primaryTargets[4],
    primaryTargets[5],
    params.external_axes_units[0],
    params.external_axes_units[1],
    params.external_axes_units[2],
  };
  float negativeLimits[numJoints] = {};
  float positiveLimits[numJoints] = {};
  float stepsPerUnit[numJoints] = {};
  int stepLimits[numJoints] = {};
  load_axis_calibration(
    negativeLimits,
    positiveLimits,
    stepsPerUnit,
    stepLimits
  );
  int futureSteps[numJoints] = {};
  bool representationAxes[numJoints] = {};
  bool limitAxes[numJoints] = {};
  bool representationFailure = false;
  bool limitFailure = false;
  for (int axis = 0; axis < numJoints; ++axis) {
    int derivedStepLimit = 0;
    int zeroStep = 0;
    if (
      !ar4_protocol::validate_axis_calibration(
        negativeLimits[axis],
        positiveLimits[axis],
        stepsPerUnit[axis],
        derivedStepLimit,
        zeroStep
      )
      || stepLimits[axis] < 0
      || stepLimits[axis] != derivedStepLimit
    ) {
      representationAxes[axis] = true;
      representationFailure = true;
      continue;
    }
    if (
      targets[axis] < -negativeLimits[axis]
      || targets[axis] > positiveLimits[axis]
    ) {
      limitAxes[axis] = true;
      limitFailure = true;
      continue;
    }
    if (!ar4_protocol::calibrated_position_to_step(
        targets[axis],
        negativeLimits[axis],
        positiveLimits[axis],
        stepsPerUnit[axis],
        stepLimits[axis],
        futureSteps[axis]
    )) {
      representationAxes[axis] = true;
      representationFailure = true;
    }
  }
  if (representationFailure) {
    result.outcome =
      ar4_protocol::JsonMainMoveCartesianOutcome::kPositionNotRepresentable;
    for (int axis = 0; axis < numJoints; ++axis) {
      result.axes[axis] = representationAxes[axis];
    }
    return true;
  }
  if (limitFailure) {
    result.outcome =
      ar4_protocol::JsonMainMoveCartesianOutcome::kJointLimitViolation;
    for (int axis = 0; axis < numJoints; ++axis) {
      result.axes[axis] = limitAxes[axis];
    }
    return true;
  }
  if (
    params.speed_mode
      == ar4_protocol::JsonCartesianMotionSpeedMode::
        kMillimetersPerSecond
    && !refresh_motion_source_position()
  ) {
    result.outcome =
      ar4_protocol::JsonMainMoveCartesianOutcome::kPositionUnavailable;
    return true;
  }

  int loopModes[ROBOT_nDOFs] = {};
  for (int axis = 0; axis < ROBOT_nDOFs; ++axis) {
    loopModes[axis] = params.loop_modes[axis] ? 1 : 0;
  }
  String speedMode = "p";
  switch (params.speed_mode) {
    case ar4_protocol::JsonCartesianMotionSpeedMode::kPercent:
      speedMode = "p";
      break;
    case ar4_protocol::JsonCartesianMotionSpeedMode::kSeconds:
      speedMode = "s";
      break;
    case ar4_protocol::JsonCartesianMotionSpeedMode::kMillimetersPerSecond:
      speedMode = "m";
      break;
  }
  PreparedJsonMotionResult prepared = {};
  if (!execute_prepared_json_motion(
      futureSteps,
      speedMode,
      params.speed_value,
      params.acceleration_percent,
      params.deceleration_percent,
      params.ramp_percent,
      String(wristConfiguration),
      loopModes,
      params.telemetry_enabled,
      prepared,
      continuationSource,
      gcodeTiming
  )) {
    return false;
  }

  switch (prepared.outcome) {
    case PreparedJsonMotionOutcome::kCompleted:
      result.outcome =
        ar4_protocol::JsonMainMoveCartesianOutcome::kCompleted;
      result.speed_limited = prepared.speed_limited;
      break;
    case PreparedJsonMotionOutcome::kEmergencyStop:
      result.outcome =
        ar4_protocol::JsonMainMoveCartesianOutcome::kEmergencyStop;
      break;
    case PreparedJsonMotionOutcome::kPositionUnavailable:
      result.outcome =
        ar4_protocol::JsonMainMoveCartesianOutcome::kPositionUnavailable;
      return true;
    case PreparedJsonMotionOutcome::kMotionExecutionFailed:
      result.outcome =
        ar4_protocol::JsonMainMoveCartesianOutcome::kMotionExecutionFailed;
      break;
    case PreparedJsonMotionOutcome::kEncoderCollision:
      result.outcome =
        ar4_protocol::JsonMainMoveCartesianOutcome::kEncoderCollision;
      break;
    case PreparedJsonMotionOutcome::kEncoderStateUnavailable:
      result.outcome = ar4_protocol::JsonMainMoveCartesianOutcome::
        kEncoderStateUnavailable;
      break;
  }
  if (
    prepared.outcome == PreparedJsonMotionOutcome::kEncoderCollision
    || prepared.outcome
      == PreparedJsonMotionOutcome::kEncoderStateUnavailable
  ) {
    for (int axis = 0; axis < ROBOT_nDOFs; ++axis) {
      result.axes[axis] = prepared.encoder_axes[axis];
    }
  }
  if (capturePosition && !capture_json_motion_position(result)) {
    result = {};
    result.outcome =
      ar4_protocol::JsonMainMoveCartesianOutcome::kPositionUnavailable;
  }
  return true;
}

bool execute_json_move_cartesian(
  const ar4_protocol::JsonMainMoveCartesianParameters &params,
  ar4_protocol::JsonMainMoveCartesianExecutionResult &result,
  void *context
) {
  return execute_json_move_cartesian_with_timing(
    params, result, context, false, true
  );
}


bool execute_json_move_joints(
  const ar4_protocol::JsonMainMoveJointsParameters &params,
  ar4_protocol::JsonMainMoveJointsExecutionResult &result,
  void *context
) {
  const ar4_protocol::JsonLiveMotionContinuationSource *
    continuationSource = static_cast<
      const ar4_protocol::JsonLiveMotionContinuationSource *
    >(context);
  if (!ar4_protocol::json_live_motion_continuation_source_valid(
      continuationSource
  )) return false;
  result = {};

  float positions[numJoints] = {};
  for (int axis = 0; axis < ROBOT_nDOFs; ++axis) {
    positions[axis] = params.robot_joints_degrees[axis];
  }
  for (int axis = 0; axis < 3; ++axis) {
    positions[axis + ROBOT_nDOFs] = params.external_axes_units[axis];
  }

  float negativeLimits[numJoints] = {};
  float positiveLimits[numJoints] = {};
  float stepsPerUnit[numJoints] = {};
  int stepLimits[numJoints] = {};
  load_axis_calibration(
    negativeLimits,
    positiveLimits,
    stepsPerUnit,
    stepLimits
  );
  int futureSteps[numJoints] = {};
  bool representationFailure = false;
  bool limitFailure = false;
  bool representationAxes[numJoints] = {};
  bool limitAxes[numJoints] = {};
  for (int axis = 0; axis < numJoints; ++axis) {
    int derivedStepLimit = 0;
    int zeroStep = 0;
    if (
      !ar4_protocol::validate_axis_calibration(
        negativeLimits[axis],
        positiveLimits[axis],
        stepsPerUnit[axis],
        derivedStepLimit,
        zeroStep
      )
      || stepLimits[axis] < 0
      || stepLimits[axis] != derivedStepLimit
    ) {
      representationAxes[axis] = true;
      representationFailure = true;
      continue;
    }
    if (
      positions[axis] < -negativeLimits[axis]
      || positions[axis] > positiveLimits[axis]
    ) {
      limitAxes[axis] = true;
      limitFailure = true;
      continue;
    }
    if (!ar4_protocol::calibrated_position_to_step(
        positions[axis],
        negativeLimits[axis],
        positiveLimits[axis],
        stepsPerUnit[axis],
        stepLimits[axis],
        futureSteps[axis]
    )) {
      representationAxes[axis] = true;
      representationFailure = true;
    }
  }
  if (representationFailure) {
    result.outcome =
      ar4_protocol::JsonMainMoveJointsOutcome::kPositionNotRepresentable;
    for (int axis = 0; axis < numJoints; ++axis) {
      result.axes[axis] = representationAxes[axis];
    }
    return true;
  }
  if (limitFailure) {
    result.outcome =
      ar4_protocol::JsonMainMoveJointsOutcome::kJointLimitViolation;
    for (int axis = 0; axis < numJoints; ++axis) {
      result.axes[axis] = limitAxes[axis];
    }
    return true;
  }

  MotionKinematicsTransaction kinematicsTransaction;
  if (!prepare_joint_speed_target(
      params.robot_joints_degrees,
      params.speed_mode
        == ar4_protocol::JsonJointMotionSpeedMode::kMillimetersPerSecond
  )) {
    result.outcome =
      ar4_protocol::JsonMainMoveJointsOutcome::kPositionUnavailable;
    return true;
  }

  int loopModes[ROBOT_nDOFs] = {};
  for (int axis = 0; axis < ROBOT_nDOFs; ++axis) {
    loopModes[axis] = params.loop_modes[axis] ? 1 : 0;
  }
  String wristConfiguration = "A";
  switch (params.wrist_configuration) {
    case ar4_protocol::JsonJointMotionWristConfiguration::kNear:
      wristConfiguration = "N";
      break;
    case ar4_protocol::JsonJointMotionWristConfiguration::kFar:
      wristConfiguration = "F";
      break;
    case ar4_protocol::JsonJointMotionWristConfiguration::kAutomatic:
      wristConfiguration = "A";
      break;
  }
  String speedMode = "p";
  switch (params.speed_mode) {
    case ar4_protocol::JsonJointMotionSpeedMode::kPercent:
      speedMode = "p";
      break;
    case ar4_protocol::JsonJointMotionSpeedMode::kSeconds:
      speedMode = "s";
      break;
    case ar4_protocol::JsonJointMotionSpeedMode::kMillimetersPerSecond:
      speedMode = "m";
      break;
  }
  PreparedJsonMotionResult prepared = {};
  if (!execute_prepared_json_motion(
    futureSteps,
    speedMode,
    params.speed_value,
    params.acceleration_percent,
    params.deceleration_percent,
    params.ramp_percent,
    wristConfiguration,
    loopModes,
    params.telemetry_enabled,
    prepared,
    continuationSource
  )) {
    return false;
  }
  switch (prepared.outcome) {
    case PreparedJsonMotionOutcome::kCompleted:
      result.outcome = ar4_protocol::JsonMainMoveJointsOutcome::kCompleted;
      result.speed_limited = prepared.speed_limited;
      break;
    case PreparedJsonMotionOutcome::kEmergencyStop:
      result.outcome =
        ar4_protocol::JsonMainMoveJointsOutcome::kEmergencyStop;
      break;
    case PreparedJsonMotionOutcome::kPositionUnavailable:
      result.outcome =
        ar4_protocol::JsonMainMoveJointsOutcome::kPositionUnavailable;
      return true;
    case PreparedJsonMotionOutcome::kMotionExecutionFailed:
      result.outcome =
        ar4_protocol::JsonMainMoveJointsOutcome::kMotionExecutionFailed;
      break;
    case PreparedJsonMotionOutcome::kEncoderCollision:
      result.outcome =
        ar4_protocol::JsonMainMoveJointsOutcome::kEncoderCollision;
      break;
    case PreparedJsonMotionOutcome::kEncoderStateUnavailable:
      result.outcome =
        ar4_protocol::JsonMainMoveJointsOutcome::kEncoderStateUnavailable;
      break;
  }
  if (
    prepared.outcome == PreparedJsonMotionOutcome::kEncoderCollision
    || prepared.outcome
      == PreparedJsonMotionOutcome::kEncoderStateUnavailable
  ) {
    for (int axis = 0; axis < ROBOT_nDOFs; ++axis) {
      result.axes[axis] = prepared.encoder_axes[axis];
    }
  }
  if (!capture_json_motion_position(result)) {
    result = {};
    result.outcome =
      ar4_protocol::JsonMainMoveJointsOutcome::kPositionUnavailable;
  }
  return true;
}


bool execute_json_jog_tool_with_telemetry(
  const ar4_protocol::JsonMainToolJogParameters &params,
  ar4_protocol::JsonMainToolJogExecutionResult &result,
  bool telemetryRequested,
  ar4_protocol::JsonLiveMotionContinuationSource *
    continuationSource
) {
  if (!ar4_protocol::json_live_motion_continuation_source_valid(
      continuationSource
  )) return false;
  result = {};

  int toolFrameIndex = -1;
  float toolFrameOffset = 0.0f;
  if (!ar4_protocol::decode_json_tool_jog_offset(
      params,
      toolFrameIndex,
      toolFrameOffset
  )) {
    return false;
  }

  float primaryTargets[ROBOT_nDOFs] = {};
  {
    MotionKinematicsTransaction kinematicsTransaction;
    if (!refresh_motion_source_position()) {
      result.outcome =
        ar4_protocol::JsonMainToolJogOutcome::kPositionUnavailable;
      return true;
    }
    MotionToolFrameTransaction toolFrameTransaction;
    if (!toolFrameTransaction.apply_offset(
        toolFrameIndex,
        toolFrameOffset
    )) {
      result.outcome =
        ar4_protocol::JsonMainToolJogOutcome::kPositionNotRepresentable;
      return true;
    }
    for (int axis = 0; axis < ROBOT_nDOFs; ++axis) {
      xyzuvw_In[axis] = xyzuvw_Out[axis];
    }

    char wristConfiguration = 'A';
    switch (params.wrist_configuration) {
      case ar4_protocol::JsonCartesianMotionWristConfiguration::kAutomatic:
        wristConfiguration = 'A';
        break;
      case ar4_protocol::JsonCartesianMotionWristConfiguration::kNear:
        wristConfiguration = 'N';
        break;
      case ar4_protocol::JsonCartesianMotionWristConfiguration::kFar:
        wristConfiguration = 'F';
        break;
    }
    SolveInverseKinematics(wristConfiguration);
    if (KinematicError != 0) {
      result.outcome =
        ar4_protocol::JsonMainToolJogOutcome::kKinematicsUnreachable;
      return true;
    }
    for (int axis = 0; axis < ROBOT_nDOFs; ++axis) {
      primaryTargets[axis] = JangleOut[axis];
    }
  }

  ar4_protocol::JsonMainMoveJointsParameters jointParams = {};
  for (int axis = 0; axis < ROBOT_nDOFs; ++axis) {
    jointParams.robot_joints_degrees[axis] = primaryTargets[axis];
    jointParams.loop_modes[axis] = params.loop_modes[axis];
  }
  jointParams.external_axes_units[0] = J7_pos;
  jointParams.external_axes_units[1] = J8_pos;
  jointParams.external_axes_units[2] = J9_pos;
  switch (params.speed_mode) {
    case ar4_protocol::JsonCartesianMotionSpeedMode::kPercent:
      jointParams.speed_mode =
        ar4_protocol::JsonJointMotionSpeedMode::kPercent;
      break;
    case ar4_protocol::JsonCartesianMotionSpeedMode::kSeconds:
      jointParams.speed_mode =
        ar4_protocol::JsonJointMotionSpeedMode::kSeconds;
      break;
    case ar4_protocol::JsonCartesianMotionSpeedMode::
        kMillimetersPerSecond:
      return false;
  }
  jointParams.speed_value = params.speed_value;
  jointParams.acceleration_percent = params.acceleration_percent;
  jointParams.deceleration_percent = params.deceleration_percent;
  jointParams.ramp_percent = params.ramp_percent;
  switch (params.wrist_configuration) {
    case ar4_protocol::JsonCartesianMotionWristConfiguration::kAutomatic:
      jointParams.wrist_configuration =
        ar4_protocol::JsonJointMotionWristConfiguration::kAutomatic;
      break;
    case ar4_protocol::JsonCartesianMotionWristConfiguration::kNear:
      jointParams.wrist_configuration =
        ar4_protocol::JsonJointMotionWristConfiguration::kNear;
      break;
    case ar4_protocol::JsonCartesianMotionWristConfiguration::kFar:
      jointParams.wrist_configuration =
        ar4_protocol::JsonJointMotionWristConfiguration::kFar;
      break;
  }
  jointParams.telemetry_enabled = telemetryRequested;

  ar4_protocol::JsonMainMoveJointsExecutionResult jointResult = {};
  if (!execute_json_move_joints(
      jointParams,
      jointResult,
      continuationSource
  )) {
    return false;
  }
  switch (jointResult.outcome) {
    case ar4_protocol::JsonMainMoveJointsOutcome::kCompleted:
      result.outcome = ar4_protocol::JsonMainToolJogOutcome::kCompleted;
      break;
    case ar4_protocol::JsonMainMoveJointsOutcome::kJointLimitViolation:
      result.outcome =
        ar4_protocol::JsonMainToolJogOutcome::kJointLimitViolation;
      break;
    case ar4_protocol::JsonMainMoveJointsOutcome::kPositionNotRepresentable:
      result.outcome =
        ar4_protocol::JsonMainToolJogOutcome::kPositionNotRepresentable;
      break;
    case ar4_protocol::JsonMainMoveJointsOutcome::kEmergencyStop:
      result.outcome = ar4_protocol::JsonMainToolJogOutcome::kEmergencyStop;
      break;
    case ar4_protocol::JsonMainMoveJointsOutcome::kPositionUnavailable:
      result.outcome =
        ar4_protocol::JsonMainToolJogOutcome::kPositionUnavailable;
      break;
    case ar4_protocol::JsonMainMoveJointsOutcome::kMotionExecutionFailed:
      result.outcome =
        ar4_protocol::JsonMainToolJogOutcome::kMotionExecutionFailed;
      break;
    case ar4_protocol::JsonMainMoveJointsOutcome::kEncoderCollision:
      result.outcome =
        ar4_protocol::JsonMainToolJogOutcome::kEncoderCollision;
      break;
    case ar4_protocol::JsonMainMoveJointsOutcome::kEncoderStateUnavailable:
      result.outcome =
        ar4_protocol::JsonMainToolJogOutcome::kEncoderStateUnavailable;
      break;
    case ar4_protocol::JsonMainMoveJointsOutcome::kInvalid:
      return false;
  }
  result.position = jointResult.position;
  for (size_t axis = 0; axis < ar4_protocol::kJsonCartesianMotionAxisCount;
       ++axis) {
    result.axes[axis] = jointResult.axes[axis];
  }
  result.speed_limited = jointResult.speed_limited;
  for (size_t index = 0; index < sizeof(result.controller_debug); ++index) {
    result.controller_debug[index] = jointResult.controller_debug[index];
  }
  return true;
}


bool execute_json_jog_tool(
  const ar4_protocol::JsonMainToolJogParameters &params,
  ar4_protocol::JsonMainToolJogExecutionResult &result,
  void *context
) {
  (void)context;
  return execute_json_jog_tool_with_telemetry(
    params,
    result,
    false,
    nullptr
  );
}


bool begin_json_live_jog(
  uint32_t motionId,
  const ar4_protocol::JsonMainLiveJogParameters &params,
  void *context
) {
  (void)context;
  if (
    motionId == 0
    || !ar4_protocol::json_live_motion_owner_empty(jsonLiveMotion.owner)
    || !ar4_protocol::json_live_jog_detail::parameters_valid(params)
  ) return false;
  JsonLiveMotionRuntime pending = {};
  if (!ar4_protocol::begin_json_live_motion(
      motionId,
      params.lease_milliseconds,
      pending.owner
  )) return false;
  pending.parameters = params;
  jsonLiveMotion = pending;
  return true;
}


bool request_json_live_stop(uint32_t motionId, void *context) {
  (void)context;
  if (
    jsonLiveMotion.owner.motion_id == motionId
    && jsonLiveMotion.owner.state
      == JsonLiveMotionRuntimeState::kTerminalReady
  ) {
    return true;
  }
  return ar4_protocol::request_json_live_motion_stop(
    motionId,
    jsonLiveMotion.owner
  );
}


ar4_protocol::JsonMainLiveJogRenewStatus renew_json_live_motion_callback(
  uint32_t motionId,
  void *context
) {
  (void)context;
  if (ar4_protocol::renew_json_live_motion(
      motionId,
      static_cast<uint32_t>(millis()),
      jsonLiveMotion.owner
  )) {
    return ar4_protocol::JsonMainLiveJogRenewStatus::kRenewed;
  }
  if (
    jsonLiveMotion.owner.motion_id == motionId
    && (
      jsonLiveMotion.owner.state
        == JsonLiveMotionRuntimeState::kLeaseExpired
      || (
        jsonLiveMotion.owner.state
          == JsonLiveMotionRuntimeState::kTerminalReady
        && jsonLiveMotion.terminal.outcome
          == ar4_protocol::JsonMainLiveJogOutcome::kControlLeaseExpired
      )
    )
  ) {
    return ar4_protocol::JsonMainLiveJogRenewStatus::kLeaseExpired;
  }
  if (
    jsonLiveMotion.owner.motion_id == motionId
    && (
      jsonLiveMotion.owner.state
        == JsonLiveMotionRuntimeState::kStopRequested
      || jsonLiveMotion.owner.state
        == JsonLiveMotionRuntimeState::kTerminalReady
    )
  ) {
    return ar4_protocol::JsonMainLiveJogRenewStatus::kMotionSettled;
  }
  return ar4_protocol::JsonMainLiveJogRenewStatus::kFault;
}


bool json_live_motion_continuation(void *context) {
  (void)context;
  return ar4_protocol::json_live_motion_may_continue(
    jsonLiveMotion.owner.motion_id,
    static_cast<uint32_t>(millis()),
    jsonLiveMotion.owner
  );
}


bool json_live_motion_control_input_pending(void *context) {
  (void)context;
  return !jsonRuntimeFault
    && jsonMainControllerOwner.state()
      == ar4_protocol::JsonMainControllerOwnerState::kIdle
    && Serial.available() > 0;
}


bool service_one_json_live_motion_control_byte(void *context) {
  (void)context;
  processSerial();
  return !jsonRuntimeFault;
}


bool service_json_live_motion_control(void *context) {
  (void)context;
  const ar4_protocol::JsonLiveMotionControlInputSource controlInput = {
    json_live_motion_control_input_pending,
    service_one_json_live_motion_control_byte,
    nullptr,
  };
  if (!ar4_protocol::service_bounded_json_live_motion_control_input(
      controlInput
  )) return false;
  service_json_output();
  return !jsonRuntimeFault;
}


template <typename SourceResult>
void copy_json_live_result_fields(
  const SourceResult &source,
  ar4_protocol::JsonMainLiveJogExecutionResult &target
) {
  target.position = source.position;
  for (
    size_t axis = 0;
    axis < ar4_protocol::kJsonLiveJogControllerAxisCount;
    ++axis
  ) {
    target.axes[axis] = source.axes[axis];
  }
  target.speed_limited = source.speed_limited;
  for (size_t index = 0; index < sizeof(target.controller_debug); ++index) {
    target.controller_debug[index] = source.controller_debug[index];
  }
}


ar4_protocol::JsonMainLiveJogExecutionResult convert_json_live_result(
  const ar4_protocol::JsonMainMoveJointsExecutionResult &source
) {
  ar4_protocol::JsonMainLiveJogExecutionResult target = {};
  copy_json_live_result_fields(source, target);
  switch (source.outcome) {
    case ar4_protocol::JsonMainMoveJointsOutcome::kCompleted:
      target.outcome = ar4_protocol::JsonMainLiveJogOutcome::kCompleted;
      break;
    case ar4_protocol::JsonMainMoveJointsOutcome::kJointLimitViolation:
      target.outcome =
        ar4_protocol::JsonMainLiveJogOutcome::kJointLimitReached;
      break;
    case ar4_protocol::JsonMainMoveJointsOutcome::kPositionNotRepresentable:
      target.outcome =
        ar4_protocol::JsonMainLiveJogOutcome::kPositionNotRepresentable;
      break;
    case ar4_protocol::JsonMainMoveJointsOutcome::kEmergencyStop:
      target.outcome = ar4_protocol::JsonMainLiveJogOutcome::kEmergencyStop;
      break;
    case ar4_protocol::JsonMainMoveJointsOutcome::kPositionUnavailable:
      target.outcome =
        ar4_protocol::JsonMainLiveJogOutcome::kPositionUnavailable;
      break;
    case ar4_protocol::JsonMainMoveJointsOutcome::kMotionExecutionFailed:
      target.outcome =
        ar4_protocol::JsonMainLiveJogOutcome::kMotionExecutionFailed;
      break;
    case ar4_protocol::JsonMainMoveJointsOutcome::kEncoderCollision:
      target.outcome =
        ar4_protocol::JsonMainLiveJogOutcome::kEncoderCollision;
      break;
    case ar4_protocol::JsonMainMoveJointsOutcome::kEncoderStateUnavailable:
      target.outcome =
        ar4_protocol::JsonMainLiveJogOutcome::kEncoderStateUnavailable;
      break;
    case ar4_protocol::JsonMainMoveJointsOutcome::kInvalid:
      target.outcome = ar4_protocol::JsonMainLiveJogOutcome::kInvalid;
      break;
  }
  return target;
}


ar4_protocol::JsonMainLiveJogExecutionResult convert_json_live_result(
  const ar4_protocol::JsonMainMoveCartesianExecutionResult &source
) {
  ar4_protocol::JsonMainLiveJogExecutionResult target = {};
  copy_json_live_result_fields(source, target);
  switch (source.outcome) {
    case ar4_protocol::JsonMainMoveCartesianOutcome::kCompleted:
      target.outcome = ar4_protocol::JsonMainLiveJogOutcome::kCompleted;
      break;
    case ar4_protocol::JsonMainMoveCartesianOutcome::kKinematicsUnreachable:
      target.outcome =
        ar4_protocol::JsonMainLiveJogOutcome::kKinematicsUnreachable;
      break;
    case ar4_protocol::JsonMainMoveCartesianOutcome::kJointLimitViolation:
      target.outcome =
        ar4_protocol::JsonMainLiveJogOutcome::kJointLimitReached;
      break;
    case ar4_protocol::JsonMainMoveCartesianOutcome::
        kPositionNotRepresentable:
      target.outcome =
        ar4_protocol::JsonMainLiveJogOutcome::kPositionNotRepresentable;
      break;
    case ar4_protocol::JsonMainMoveCartesianOutcome::kEmergencyStop:
      target.outcome = ar4_protocol::JsonMainLiveJogOutcome::kEmergencyStop;
      break;
    case ar4_protocol::JsonMainMoveCartesianOutcome::kPositionUnavailable:
      target.outcome =
        ar4_protocol::JsonMainLiveJogOutcome::kPositionUnavailable;
      break;
    case ar4_protocol::JsonMainMoveCartesianOutcome::kMotionExecutionFailed:
      target.outcome =
        ar4_protocol::JsonMainLiveJogOutcome::kMotionExecutionFailed;
      break;
    case ar4_protocol::JsonMainMoveCartesianOutcome::kEncoderCollision:
      target.outcome =
        ar4_protocol::JsonMainLiveJogOutcome::kEncoderCollision;
      break;
    case ar4_protocol::JsonMainMoveCartesianOutcome::
        kEncoderStateUnavailable:
      target.outcome =
        ar4_protocol::JsonMainLiveJogOutcome::kEncoderStateUnavailable;
      break;
    case ar4_protocol::JsonMainMoveCartesianOutcome::kInvalid:
      target.outcome = ar4_protocol::JsonMainLiveJogOutcome::kInvalid;
      break;
  }
  return target;
}


void settle_json_live_terminal(
  ar4_protocol::JsonMainLiveJogExecutionResult terminal
) {
  if (
    terminal.outcome
      != ar4_protocol::JsonMainLiveJogOutcome::kPositionUnavailable
    && !capture_json_motion_position(terminal)
  ) {
    terminal = {};
    terminal.outcome =
      ar4_protocol::JsonMainLiveJogOutcome::kPositionUnavailable;
  }
  jsonLiveMotion.terminal = terminal;
  if (!ar4_protocol::mark_json_live_motion_terminal_ready(
      jsonLiveMotion.owner
  )) {
    latch_json_runtime_fault();
  }
}


void record_json_live_segment_result(
  ar4_protocol::JsonMainLiveJogExecutionResult result
) {
  if (
    jsonLiveMotion.owner.state
      == JsonLiveMotionRuntimeState::kLeaseExpired
    && result.outcome
      == ar4_protocol::JsonMainLiveJogOutcome::kMotionExecutionFailed
  ) {
    result = {};
    result.outcome =
      ar4_protocol::JsonMainLiveJogOutcome::kControlLeaseExpired;
  }
  if (result.outcome == ar4_protocol::JsonMainLiveJogOutcome::kCompleted) {
    jsonLiveMotion.speed_limited =
      jsonLiveMotion.speed_limited || result.speed_limited;
    jsonLiveMotion.terminal = result;
    jsonLiveMotion.terminal.speed_limited = jsonLiveMotion.speed_limited;
    return;
  }
  settle_json_live_terminal(result);
}


bool execute_json_live_joint_segment() {
  ar4_protocol::JsonMainPositionSnapshot position = {};
  if (!prepare_json_position_snapshot(position)) {
    ar4_protocol::JsonMainLiveJogExecutionResult terminal = {};
    terminal.outcome =
      ar4_protocol::JsonMainLiveJogOutcome::kPositionUnavailable;
    settle_json_live_terminal(terminal);
    return true;
  }
  ar4_protocol::JsonMainMoveJointsParameters params = {};
  for (size_t axis = 0; axis < ROBOT_nDOFs; ++axis) {
    params.robot_joints_degrees[axis] =
      static_cast<float>(position.robot_joints_millidegrees[axis])
        / 1000.0f;
    params.loop_modes[axis] = jsonLiveMotion.parameters.loop_modes[axis];
  }
  for (size_t axis = 0; axis < 3; ++axis) {
    params.external_axes_units[axis] =
      static_cast<float>(position.external_axes_milliunits[axis])
        / 1000.0f;
  }
  const float direction =
    jsonLiveMotion.parameters.direction
        == ar4_protocol::JsonToolJogDirection::kPositive
      ? 1.0f
      : -1.0f;
  const size_t axis = jsonLiveMotion.parameters.axis_index;
  if (axis < ROBOT_nDOFs) {
    params.robot_joints_degrees[axis] += direction * 0.25f;
  } else if (axis < numJoints) {
    params.external_axes_units[axis - ROBOT_nDOFs] += direction * 0.25f;
  } else {
    return false;
  }
  params.speed_mode = jsonLiveMotion.parameters.speed_mode;
  params.speed_value = jsonLiveMotion.parameters.speed_value;
  params.acceleration_percent =
    jsonLiveMotion.parameters.acceleration_percent;
  params.deceleration_percent =
    jsonLiveMotion.parameters.deceleration_percent;
  params.ramp_percent = jsonLiveMotion.parameters.ramp_percent;
  params.wrist_configuration =
    jsonLiveMotion.parameters.wrist_configuration;
  params.telemetry_enabled = jsonLiveMotion.parameters.telemetry_enabled;
  ar4_protocol::JsonMainMoveJointsExecutionResult result = {};
  ar4_protocol::JsonLiveMotionContinuationSource continuation = {
    json_live_motion_continuation,
    service_json_live_motion_control,
    nullptr,
  };
  const bool executed = execute_json_move_joints(
    params,
    result,
    &continuation
  );
  if (ar4_protocol::json_live_motion_interrupted_by_control(
      jsonLiveMotion.owner.state
  )) return true;
  if (!executed) return false;
  record_json_live_segment_result(convert_json_live_result(result));
  return true;
}


bool execute_json_live_cartesian_segment() {
  if (!refresh_motion_source_position()) {
    ar4_protocol::JsonMainLiveJogExecutionResult terminal = {};
    terminal.outcome =
      ar4_protocol::JsonMainLiveJogOutcome::kPositionUnavailable;
    settle_json_live_terminal(terminal);
    return true;
  }
  ar4_protocol::JsonMainMoveCartesianParameters params = {};
  params.translation_millimeters[0] = xyzuvw_Out[0];
  params.translation_millimeters[1] = xyzuvw_Out[1];
  params.translation_millimeters[2] = xyzuvw_Out[2];
  params.orientation_degrees[0] = xyzuvw_Out[5];
  params.orientation_degrees[1] = xyzuvw_Out[4];
  params.orientation_degrees[2] = xyzuvw_Out[3];
  params.external_axes_units[0] = J7_pos;
  params.external_axes_units[1] = J8_pos;
  params.external_axes_units[2] = J9_pos;
  const float direction =
    jsonLiveMotion.parameters.direction
        == ar4_protocol::JsonToolJogDirection::kPositive
      ? 1.0f
      : -1.0f;
  const size_t axis = jsonLiveMotion.parameters.axis_index;
  if (axis < 3) {
    params.translation_millimeters[axis] += direction * 0.25f;
  } else if (axis < ROBOT_nDOFs) {
    params.orientation_degrees[axis - 3] += direction * 0.25f;
  } else {
    return false;
  }
  params.speed_mode =
    ar4_protocol::JsonCartesianMotionSpeedMode::kPercent;
  params.speed_value = jsonLiveMotion.parameters.speed_value;
  params.acceleration_percent =
    jsonLiveMotion.parameters.acceleration_percent;
  params.deceleration_percent =
    jsonLiveMotion.parameters.deceleration_percent;
  params.ramp_percent = jsonLiveMotion.parameters.ramp_percent;
  switch (jsonLiveMotion.parameters.wrist_configuration) {
    case ar4_protocol::JsonJointMotionWristConfiguration::kNear:
      params.wrist_configuration =
        ar4_protocol::JsonCartesianMotionWristConfiguration::kNear;
      break;
    case ar4_protocol::JsonJointMotionWristConfiguration::kFar:
      params.wrist_configuration =
        ar4_protocol::JsonCartesianMotionWristConfiguration::kFar;
      break;
    case ar4_protocol::JsonJointMotionWristConfiguration::kAutomatic:
      params.wrist_configuration =
        ar4_protocol::JsonCartesianMotionWristConfiguration::kAutomatic;
      break;
  }
  for (size_t joint = 0; joint < ROBOT_nDOFs; ++joint) {
    params.loop_modes[joint] = jsonLiveMotion.parameters.loop_modes[joint];
  }
  params.telemetry_enabled = jsonLiveMotion.parameters.telemetry_enabled;
  ar4_protocol::JsonMainMoveCartesianExecutionResult result = {};
  ar4_protocol::JsonLiveMotionContinuationSource continuation = {
    json_live_motion_continuation,
    service_json_live_motion_control,
    nullptr,
  };
  const bool executed = execute_json_move_cartesian(
    params,
    result,
    &continuation
  );
  if (ar4_protocol::json_live_motion_interrupted_by_control(
      jsonLiveMotion.owner.state
  )) return true;
  if (!executed) return false;
  record_json_live_segment_result(convert_json_live_result(result));
  return true;
}


bool execute_json_live_tool_segment() {
  ar4_protocol::JsonMainToolJogParameters params = {};
  params.axis = static_cast<ar4_protocol::JsonToolJogAxis>(
    jsonLiveMotion.parameters.axis_index
  );
  params.direction = jsonLiveMotion.parameters.direction;
  params.distance = 0.25f;
  params.speed_mode = ar4_protocol::JsonCartesianMotionSpeedMode::kPercent;
  params.speed_value = jsonLiveMotion.parameters.speed_value;
  params.acceleration_percent =
    jsonLiveMotion.parameters.acceleration_percent;
  params.deceleration_percent =
    jsonLiveMotion.parameters.deceleration_percent;
  params.ramp_percent = jsonLiveMotion.parameters.ramp_percent;
  switch (jsonLiveMotion.parameters.wrist_configuration) {
    case ar4_protocol::JsonJointMotionWristConfiguration::kNear:
      params.wrist_configuration =
        ar4_protocol::JsonCartesianMotionWristConfiguration::kNear;
      break;
    case ar4_protocol::JsonJointMotionWristConfiguration::kFar:
      params.wrist_configuration =
        ar4_protocol::JsonCartesianMotionWristConfiguration::kFar;
      break;
    case ar4_protocol::JsonJointMotionWristConfiguration::kAutomatic:
      params.wrist_configuration =
        ar4_protocol::JsonCartesianMotionWristConfiguration::kAutomatic;
      break;
  }
  for (size_t joint = 0; joint < ROBOT_nDOFs; ++joint) {
    params.loop_modes[joint] = jsonLiveMotion.parameters.loop_modes[joint];
  }
  ar4_protocol::JsonMainToolJogExecutionResult result = {};
  ar4_protocol::JsonLiveMotionContinuationSource continuation = {
    json_live_motion_continuation,
    service_json_live_motion_control,
    nullptr,
  };
  const bool executed = execute_json_jog_tool_with_telemetry(
      params,
      result,
      jsonLiveMotion.parameters.telemetry_enabled,
      &continuation
  );
  if (ar4_protocol::json_live_motion_interrupted_by_control(
      jsonLiveMotion.owner.state
  )) return true;
  if (!executed) return false;
  record_json_live_segment_result(convert_json_live_result(result));
  return true;
}


bool stage_json_live_terminal() {
  jsonLiveMotion.terminal.speed_limited = jsonLiveMotion.speed_limited
    || jsonLiveMotion.terminal.speed_limited;
  const ar4_protocol::JsonMainControllerProcessStatus status =
    jsonMainControllerOwner.stage_live_terminal(
      jsonLiveMotion.owner.motion_id,
      jsonLiveMotion.parameters.kind,
      jsonLiveMotion.terminal,
      ar4_protocol::kJsonProtocolMaximumPayloadBytes
    );
  if (
    status
      != ar4_protocol::JsonMainControllerProcessStatus::kResponseReady
  ) {
    apply_json_process_status(status);
    return false;
  }
  if (!ar4_protocol::mark_json_live_motion_terminal_staged(
      jsonLiveMotion.owner
  )) {
    latch_json_runtime_fault();
    return false;
  }
  return true;
}


bool service_json_live_motion(bool advanceSegment) {
  const uint32_t ownerMotionId =
    jsonMainControllerOwner.active_live_motion_id();
  const bool ownerIdle = jsonMainControllerOwner.state()
    == ar4_protocol::JsonMainControllerOwnerState::kIdle;
  const bool activeKindMatches = ownerMotionId != 0
    && jsonMainControllerOwner.active_live_kind()
      == jsonLiveMotion.parameters.kind;
  const ar4_protocol::JsonLiveMotionServiceAction action =
    ar4_protocol::plan_json_live_motion_service(
      static_cast<uint32_t>(millis()),
      ownerMotionId,
      activeKindMatches,
      ownerIdle,
      jsonRuntimeFault,
      controller_mutation_estop_blocked(),
      Serial.available() > 0
        || recData.length() != 0
        || serialFrameDiscarding,
      advanceSegment,
      jsonLiveMotion.owner
    );
  switch (action) {
    case ar4_protocol::JsonLiveMotionServiceAction::kIdle:
      return false;
    case ar4_protocol::JsonLiveMotionServiceAction::kClear:
      clear_json_live_motion_runtime();
      return false;
    case ar4_protocol::JsonLiveMotionServiceAction::kFault:
      latch_json_runtime_fault();
      return false;
    case ar4_protocol::JsonLiveMotionServiceAction::kWait:
      return true;
    case ar4_protocol::JsonLiveMotionServiceAction::kStageTerminal:
      stage_json_live_terminal();
      return true;
    case ar4_protocol::JsonLiveMotionServiceAction::kSettleStop: {
      ar4_protocol::JsonMainLiveJogExecutionResult terminal = {};
      terminal.outcome = ar4_protocol::JsonMainLiveJogOutcome::kCompleted;
      terminal.speed_limited = jsonLiveMotion.speed_limited;
      settle_json_live_terminal(terminal);
      return true;
    }
    case ar4_protocol::JsonLiveMotionServiceAction::kSettleLeaseExpiry: {
      ar4_protocol::JsonMainLiveJogExecutionResult terminal = {};
      terminal.outcome =
        ar4_protocol::JsonMainLiveJogOutcome::kControlLeaseExpired;
      settle_json_live_terminal(terminal);
      return true;
    }
    case ar4_protocol::JsonLiveMotionServiceAction::kSettleEmergencyStop: {
      ar4_protocol::JsonMainLiveJogExecutionResult terminal = {};
      terminal.outcome =
        ar4_protocol::JsonMainLiveJogOutcome::kEmergencyStop;
      settle_json_live_terminal(terminal);
      return true;
    }
    case ar4_protocol::JsonLiveMotionServiceAction::kAdvanceSegment:
      break;
  }
  bool executed = false;
  switch (jsonLiveMotion.parameters.kind) {
    case ar4_protocol::JsonLiveJogKind::kJoint:
      executed = execute_json_live_joint_segment();
      break;
    case ar4_protocol::JsonLiveJogKind::kCartesian:
      executed = execute_json_live_cartesian_segment();
      break;
    case ar4_protocol::JsonLiveJogKind::kTool:
      executed = execute_json_live_tool_segment();
      break;
    case ar4_protocol::JsonLiveJogKind::kInvalid:
      executed = false;
      break;
  }
  if (!executed) {
    latch_json_runtime_fault();
    return false;
  }
  return true;
}


ar4_protocol::JsonMainSetPositionApplyStatus apply_json_set_position(
  const ar4_protocol::JsonMainSetPositionParameters &params,
  void *context
) {
  (void)context;
  float positions[numJoints] = {};
  for (size_t axis = 0; axis < ROBOT_nDOFs; ++axis) {
    positions[axis] =
      static_cast<float>(params.robot_joints_millidegrees[axis]) / 1000.0f;
  }
  for (size_t axis = 0; axis < 3; ++axis) {
    positions[axis + ROBOT_nDOFs] =
      static_cast<float>(params.external_axes_milliunits[axis]) / 1000.0f;
  }
  int stagedStepMonitors[numJoints] = {};
  ar4_protocol::ControllerPositionRebase stagedRebase = {};
  if (
    !joint_positions_to_future_steps(positions, stagedStepMonitors)
    || !build_configured_position_rebase(
      stagedStepMonitors,
      stagedRebase
    )
  ) {
    return ar4_protocol::JsonMainSetPositionApplyStatus::
      kPositionNotRepresentable;
  }
  ar4_protocol::PrimaryHomeReferenceState invalidatedHomeReference =
    primaryHomeReference;
  ar4_protocol::invalidate_primary_home_reference(
    invalidatedHomeReference
  );
  if (controller_mutation_estop_blocked()) {
    return ar4_protocol::JsonMainSetPositionApplyStatus::
      kEmergencyStopActive;
  }
  commit_configured_position_rebase(stagedRebase);
  primaryHomeReference = invalidatedHomeReference;
  return ar4_protocol::JsonMainSetPositionApplyStatus::kApplied;
}


ar4_protocol::JsonMainConfigurationApplyStatus apply_json_update_params(
  const ar4_protocol::JsonMainUpdateParameters &params,
  void *context
) {
  (void)context;
  ar4_protocol::JsonMainUpdateConfiguration staged = {};
  if (!ar4_protocol::build_json_main_update_configuration(params, staged)) {
    return ar4_protocol::JsonMainConfigurationApplyStatus::
      kConfigurationNotRepresentable;
  }
  ar4_protocol::PrimaryHomeReferenceState invalidatedHomeReference =
    primaryHomeReference;
  ar4_protocol::invalidate_primary_home_reference(
    invalidatedHomeReference
  );
  if (controller_mutation_estop_blocked()) {
    return ar4_protocol::JsonMainConfigurationApplyStatus::
      kEmergencyStopActive;
  }

  for (int field = 0; field < ROBOT_nDOFs; ++field) {
    Robot_Kin_Tool[field] = staged.tool[field];
  }

  int *motorDirections[numJoints] = {
    &J1MotDir, &J2MotDir, &J3MotDir, &J4MotDir, &J5MotDir,
    &J6MotDir, &J7MotDir, &J8MotDir, &J9MotDir,
  };
  int *calibrationDirections[numJoints] = {
    &J1CalDir, &J2CalDir, &J3CalDir, &J4CalDir, &J5CalDir,
    &J6CalDir, &J7CalDir, &J8CalDir, &J9CalDir,
  };
  for (int axis = 0; axis < numJoints; ++axis) {
    *motorDirections[axis] = staged.motor_directions[axis];
    *calibrationDirections[axis] = staged.calibration_directions[axis];
  }

  J1axisLimPos = staged.positive_joint_limits_degrees[0];
  J2axisLimPos = staged.positive_joint_limits_degrees[1];
  J3axisLimPos = staged.positive_joint_limits_degrees[2];
  J4axisLimPos = staged.positive_joint_limits_degrees[3];
  J5axisLimPos = staged.positive_joint_limits_degrees[4];
  J6axisLimPos = staged.positive_joint_limits_degrees[5];
  J1axisLimNeg = staged.negative_joint_limits_degrees[0];
  J2axisLimNeg = staged.negative_joint_limits_degrees[1];
  J3axisLimNeg = staged.negative_joint_limits_degrees[2];
  J4axisLimNeg = staged.negative_joint_limits_degrees[3];
  J5axisLimNeg = staged.negative_joint_limits_degrees[4];
  J6axisLimNeg = staged.negative_joint_limits_degrees[5];

  J1StepDeg = staged.steps_per_degree[0];
  J2StepDeg = staged.steps_per_degree[1];
  J3StepDeg = staged.steps_per_degree[2];
  J4StepDeg = staged.steps_per_degree[3];
  J5StepDeg = staged.steps_per_degree[4];
  J6StepDeg = staged.steps_per_degree[5];
  J1encMult = staged.encoder_counts_per_step[0];
  J2encMult = staged.encoder_counts_per_step[1];
  J3encMult = staged.encoder_counts_per_step[2];
  J4encMult = staged.encoder_counts_per_step[3];
  J5encMult = staged.encoder_counts_per_step[4];
  J6encMult = staged.encoder_counts_per_step[5];

  for (int joint = 0; joint < ROBOT_nDOFs; ++joint) {
    for (int field = 0; field < 4; ++field) {
      DHparams[joint][field] = staged.dh_degrees[joint][field];
    }
  }

  J1axisLim = staged.joint_travel_degrees[0];
  J2axisLim = staged.joint_travel_degrees[1];
  J3axisLim = staged.joint_travel_degrees[2];
  J4axisLim = staged.joint_travel_degrees[3];
  J5axisLim = staged.joint_travel_degrees[4];
  J6axisLim = staged.joint_travel_degrees[5];
  J1StepLim = staged.step_limits[0];
  J2StepLim = staged.step_limits[1];
  J3StepLim = staged.step_limits[2];
  J4StepLim = staged.step_limits[3];
  J5StepLim = staged.step_limits[4];
  J6StepLim = staged.step_limits[5];
  J1zeroStep = staged.zero_steps[0];
  J2zeroStep = staged.zero_steps[1];
  J3zeroStep = staged.zero_steps[2];
  J4zeroStep = staged.zero_steps[3];
  J5zeroStep = staged.zero_steps[4];
  J6zeroStep = staged.zero_steps[5];
  for (int axis = 0; axis < numJoints; ++axis) {
    calibrationLimitSensor[axis] =
      staged.calibration_switch_active_high[axis] ? HIGH : LOW;
  }

  primaryHomeReference = invalidatedHomeReference;
  apply_robot_native_configuration(
    staged.dh_theta_radians,
    staged.dh_alpha_radians
  );
  return ar4_protocol::JsonMainConfigurationApplyStatus::kApplied;
}


ar4_protocol::JsonMainConfigurationApplyStatus apply_json_config_ext_axis(
  const ar4_protocol::JsonMainExternalAxisParameters &params,
  void *context
) {
  (void)context;
  ar4_protocol::JsonMainExternalAxisConfiguration staged = {};
  if (!ar4_protocol::build_json_main_external_axis_configuration(
      params,
      staged
  )) {
    return ar4_protocol::JsonMainConfigurationApplyStatus::
      kConfigurationNotRepresentable;
  }
  if (controller_mutation_estop_blocked()) {
    return ar4_protocol::JsonMainConfigurationApplyStatus::
      kEmergencyStopActive;
  }

  J7length = staged.travel_units[0];
  J7rot = staged.drive_rotations[0];
  J7steps = staged.motor_steps[0];
  J8length = staged.travel_units[1];
  J8rot = staged.drive_rotations[1];
  J8steps = staged.motor_steps[1];
  J9length = staged.travel_units[2];
  J9rot = staged.drive_rotations[2];
  J9steps = staged.motor_steps[2];

  J7axisLimNeg = 0.0f;
  J7axisLimPos = staged.axes[0].positive_limit;
  J7axisLim = J7axisLimPos;
  J7StepDeg = staged.axes[0].steps_per_unit;
  J7StepLim = staged.axes[0].step_limit;
  J7zeroStep = staged.axes[0].zero_step;
  J8axisLimNeg = 0.0f;
  J8axisLimPos = staged.axes[1].positive_limit;
  J8axisLim = J8axisLimPos;
  J8StepDeg = staged.axes[1].steps_per_unit;
  J8StepLim = staged.axes[1].step_limit;
  J8zeroStep = staged.axes[1].zero_step;
  J9axisLimNeg = 0.0f;
  J9axisLimPos = staged.axes[2].positive_limit;
  J9axisLim = J9axisLimPos;
  J9StepDeg = staged.axes[2].steps_per_unit;
  J9StepLim = staged.axes[2].step_limit;
  J9zeroStep = staged.axes[2].zero_step;
  return ar4_protocol::JsonMainConfigurationApplyStatus::kApplied;
}


ar4_protocol::JsonMainDiagnosticOutcome execute_json_diagnostic(
  ar4_protocol::JsonMainRequestCommand command,
  bool *active,
  int32_t *counts,
  void *context
) {
  (void)context;
  if (active == nullptr || counts == nullptr) {
    return ar4_protocol::JsonMainDiagnosticOutcome::kUnavailable;
  }
  if (command == ar4_protocol::JsonMainRequestCommand::kTestLimitSwitches) {
    const int calibrationPins[ROBOT_nDOFs] = {
      J1calPin, J2calPin, J3calPin, J4calPin, J5calPin, J6calPin,
    };
    for (size_t axis = 0; axis < ROBOT_nDOFs; ++axis) {
      active[axis] = ar4_protocol::calibration_switch_is_active(
        digitalRead(calibrationPins[axis]),
        calibrationLimitSensor[axis]
      );
    }
    return ar4_protocol::JsonMainDiagnosticOutcome::kCompleted;
  }
  if (command == ar4_protocol::JsonMainRequestCommand::kReadEncoders) {
    counts[0] = static_cast<int32_t>(J1encPos.read());
    counts[1] = static_cast<int32_t>(J2encPos.read());
    counts[2] = static_cast<int32_t>(J3encPos.read());
    counts[3] = static_cast<int32_t>(J4encPos.read());
    counts[4] = static_cast<int32_t>(J5encPos.read());
    counts[5] = static_cast<int32_t>(J6encPos.read());
    return ar4_protocol::JsonMainDiagnosticOutcome::kCompleted;
  }
  if (command == ar4_protocol::JsonMainRequestCommand::kSetEncoders) {
    if (controller_mutation_estop_blocked()) {
      return ar4_protocol::JsonMainDiagnosticOutcome::kEmergencyStopActive;
    }
    J1encPos.write(1000);
    J2encPos.write(1000);
    J3encPos.write(1000);
    J4encPos.write(1000);
    J5encPos.write(1000);
    J6encPos.write(1000);
    return ar4_protocol::JsonMainDiagnosticOutcome::kCompleted;
  }
  return ar4_protocol::JsonMainDiagnosticOutcome::kUnavailable;
}


ar4_protocol::JsonMainControllerWaitOutcome execute_json_controller_wait(
  uint32_t duration_milliseconds,
  void *context
) {
  (void)context;
  return wait_for_controller_duration(duration_milliseconds, true)
    ? ar4_protocol::JsonMainControllerWaitOutcome::kCompleted
    : ar4_protocol::JsonMainControllerWaitOutcome::kEmergencyStop;
}


ar4_protocol::JsonMainModbusReadOutcome execute_json_modbus_read(
  ar4_protocol::ModbusOperation operation,
  const ar4_protocol::JsonMainModbusReadParameters &parameters,
  int32_t &value,
  void *context
) {
  (void)context;
  const bool read_operation =
    operation == ar4_protocol::ModbusOperation::kReadCoil
    || operation == ar4_protocol::ModbusOperation::kReadDiscreteInput
    || operation == ar4_protocol::ModbusOperation::kReadHoldingRegisters
    || operation == ar4_protocol::ModbusOperation::kReadInputRegisters;
  // Keep the active JSON source read-only even if dispatcher mapping drifts.
  if (!read_operation) return ar4_protocol::JsonMainModbusReadOutcome::kInvalid;
  if (controller_mutation_estop_blocked()) {
    return ar4_protocol::JsonMainModbusReadOutcome::kEmergencyStop;
  }
  const int32_t result = execute_modbus_operation(
    operation, parameters.slave_id, parameters.address, parameters.count
  );
  if (controller_mutation_estop_blocked()) {
    return ar4_protocol::JsonMainModbusReadOutcome::kEmergencyStop;
  }
  if (result < 0) {
    return ar4_protocol::JsonMainModbusReadOutcome::kModbusError;
  }
  if (
    result > 1
    && (
      operation == ar4_protocol::ModbusOperation::kReadCoil
      || operation == ar4_protocol::ModbusOperation::kReadDiscreteInput
    )
  ) return ar4_protocol::JsonMainModbusReadOutcome::kModbusError;
  value = result;
  return ar4_protocol::JsonMainModbusReadOutcome::kCompleted;
}


ar4_protocol::JsonMainModbusWriteOutcome execute_json_modbus_write(
  ar4_protocol::ModbusOperation operation,
  const ar4_protocol::JsonMainModbusWriteParameters &parameters,
  void *context
) {
  (void)context;
  const bool write_operation =
    operation == ar4_protocol::ModbusOperation::kWriteCoil
    || operation == ar4_protocol::ModbusOperation::kWriteRegister;
  // Keep the active JSON source write-only even if dispatcher mapping drifts.
  if (!write_operation) {
    return ar4_protocol::JsonMainModbusWriteOutcome::kInvalid;
  }
  if (controller_mutation_estop_blocked()) {
    return ar4_protocol::JsonMainModbusWriteOutcome::kEmergencyStop;
  }
  const int32_t result = execute_modbus_operation(
    operation, parameters.slave_id, parameters.address, parameters.value
  );
  // Once bus access begins, the response owner emits any concurrent stop
  // after the shared-executor outcome instead of replacing that result.
  if (result != 1) {
    return ar4_protocol::JsonMainModbusWriteOutcome::kModbusError;
  }
  return ar4_protocol::JsonMainModbusWriteOutcome::kCompleted;
}

#include "json_direct_command_runtime.h"

ar4_protocol::JsonMainHelloResponseSource prepare_json_hello_source() {
  ar4_protocol::JsonMainHelloSourceStatus status =
    ar4_protocol::JsonMainHelloSourceStatus::kAvailable;
  if (
    identity_record_status
      == ar4_protocol::IdentityRecordStatus::kCorrupt
    || !ar4_protocol::controller_hardware_id_valid(controller_hardware_id)
  ) {
    status = ar4_protocol::JsonMainHelloSourceStatus::kIdentityUnavailable;
  } else if (
    json_session_identity_status
      != ar4_protocol::JsonSessionIdentityStatus::kAvailable
  ) {
    status = ar4_protocol::JsonMainHelloSourceStatus::kSessionUnavailable;
  }
  return {
    status,
    json_session_id,
    JSON_FIRMWARE_NAME,
    FIRMWARE_VERSION,
    JSON_FIRMWARE_BUILD,
    controller_hardware_id,
    driver_board.c_str(),
    robot_model.c_str(),
    robot_version.c_str(),
    serial_number.c_str(),
    asset_tag.c_str(),
    PROTOCOL_CAPABILITIES,
    PROTOCOL_CAPABILITY_COUNT,
    JSON_COMMANDS,
    JSON_COMMAND_COUNT,
  };
}


void queue_json_transport_error(
  const char *error_code,
  const char *message
) {
  const ar4_protocol::JsonMainControllerProcessStatus status =
    jsonMainControllerOwner.process_protocol_error(
      error_code,
      message,
      ar4_protocol::kJsonProtocolMaximumPayloadBytes
    );
  apply_json_process_status(status);
}


void process_json_serial_frame(const String &frame) {
  if (jsonRuntimeFault) return;

  ar4_protocol::JsonSerialFrameView frame_view = {};
  const ar4_protocol::JsonSerialFrameStatus frame_status =
    ar4_protocol::validate_json_serial_frame(
      frame.c_str(),
      frame.length(),
      frame_view
    );
  if (frame_status != ar4_protocol::JsonSerialFrameStatus::kComplete) {
    queue_json_transport_error(
      "malformed_frame",
      "request payload is invalid"
    );
    return;
  }

  ar4_protocol::JsonMainHelloResponseSource hello =
    prepare_json_hello_source();
  const ar4_protocol::JsonMainHomeReferenceResponseSource homeReference = {
    ar4_protocol::JsonMainHomeReferenceSourceStatus::kAvailable,
    primaryHomeReference,
  };
  ar4_protocol::JsonMainPositionResponseSource position = {};
  if (jsonMainControllerOwner.configuration_sync_required()) {
    position.status =
      ar4_protocol::JsonMainPositionSourceStatus::kPositionUnavailable;
  } else {
    prepare_json_position_source(position);
  }
  const ar4_protocol::JsonMainSetPositionCommandSource setPosition = {
    apply_json_set_position,
    nullptr,
  };
  const ar4_protocol::JsonMainUpdateParametersCommandSource updateParams = {
    apply_json_update_params,
    nullptr,
  };
  const ar4_protocol::JsonMainExternalAxisCommandSource configExtAxis = {
    apply_json_config_ext_axis,
    nullptr,
  };
  const ar4_protocol::JsonMainMoveJointsCommandSource moveJoints = {
    execute_json_move_joints,
    nullptr,
  };
  const ar4_protocol::JsonMainMoveCartesianCommandSource moveCartesian = {
    execute_json_move_cartesian,
    nullptr,
  };
  const ar4_protocol::JsonMainToolJogCommandSource jogTool = {
    execute_json_jog_tool,
    nullptr,
  };
  const ar4_protocol::JsonMainLiveJogCommandSource liveJog = {
    jsonLiveMotion.owner.state == JsonLiveMotionRuntimeState::kIdle
      ? 0
      : jsonLiveMotion.owner.motion_id,
    begin_json_live_jog,
    request_json_live_stop,
    renew_json_live_motion_callback,
    nullptr,
  };
  const ar4_protocol::JsonMainDiagnosticCommandSource diagnostics = {
    execute_json_diagnostic,
    nullptr,
  };
  const ar4_protocol::JsonMainCalibrationCommandSource calibration = {
    execute_json_calibration,
    nullptr,
  };
  const ar4_protocol::JsonMainCorrectPositionCommandSource correctPosition = {
    prepare_json_position_correction,
    apply_json_position_correction,
    &preparedJsonPositionCorrection,
  };
  const ar4_protocol::JsonMainExternalAxisZeroCommandSource zeroExternalAxis = {
    stage_json_external_axis_zero_post_zero_position,
    apply_json_external_axis_zero,
    &position,
  };
  const ar4_protocol::JsonMainControllerWaitCommandSource controllerWait = {
    execute_json_controller_wait,
    nullptr,
  };
  const ar4_protocol::JsonMainModbusReadCommandSource modbusRead = {
    execute_json_modbus_read,
    nullptr,
  };
  const ar4_protocol::JsonMainModbusWriteCommandSource modbusWrite = {
    execute_json_modbus_write,
    nullptr,
  };
  const ar4_protocol::JsonMainDirectCommandSource direct = {
    ar4_json_direct_runtime::execute,
    nullptr,
  };
  const bool estop_blocked = controller_mutation_estop_blocked();
  const ar4_protocol::JsonMainControllerDispatchSources sources = {
    &hello,
    &homeReference,
    &position,
    estop_blocked
      ? ar4_protocol::JsonMainControllerAdmissionStatus::
        kEmergencyStopActive
      : ar4_protocol::JsonMainControllerAdmissionStatus::kAvailable,
    &setPosition,
    &updateParams,
    &configExtAxis,
    &moveJoints,
    &moveCartesian,
    &jogTool,
    &liveJog,
    &diagnostics,
    &calibration,
    &correctPosition,
    &zeroExternalAxis,
    &controllerWait,
    &modbusRead,
    &modbusWrite,
    &direct,
  };
  const ar4_protocol::JsonMainControllerProcessStatus status =
    jsonMainControllerOwner.process_payload(
      frame_view.payload,
      frame_view.payload_length,
      sources,
      ar4_protocol::kJsonProtocolMaximumPayloadBytes
    );
  apply_json_process_status(status);
  if (
    status == ar4_protocol::JsonMainControllerProcessStatus::kResponseReady
    && jsonMainControllerOwner.response_kind()
      == ar4_protocol::JsonMainControllerResponseKind::kAdmissionRejected
  ) {
    stage_json_estop_admission_response();
  }
}


bool service_json_response_output() {
  const ar4_protocol::JsonMainControllerOwnerState state =
    jsonMainControllerOwner.state();
  if (state == ar4_protocol::JsonMainControllerOwnerState::kFaulted) {
    latch_json_runtime_fault();
    return true;
  }
  if (
    state == ar4_protocol::JsonMainControllerOwnerState::kIdle
    && !jsonResponseWritePrepared
  ) {
    return false;
  }
  if (!jsonResponseWritePrepared) {
    const ar4_protocol::JsonMainControllerOutputBeginStatus begin_status =
      jsonMainControllerOwner.begin_response_write(jsonResponseWriteView);
    if (
      begin_status
        != ar4_protocol::JsonMainControllerOutputBeginStatus::kReady
    ) {
      latch_json_runtime_fault();
      return true;
    }
    jsonResponseWritePrepared = true;
    jsonResponseWriteOffset = 0;
  }
  size_t admitted = 0;
  const ar4_protocol::JsonOutputProgressStatus plan_status =
    ar4_protocol::plan_json_output_chunk(
      jsonResponseWriteView.length,
      jsonResponseWriteOffset,
      Serial.availableForWrite(),
      admitted
    );
  if (plan_status == ar4_protocol::JsonOutputProgressStatus::kBlocked) {
    return true;
  }
  if (plan_status != ar4_protocol::JsonOutputProgressStatus::kProgress) {
    latch_json_runtime_fault();
    return true;
  }
  const size_t written = Serial.write(
    reinterpret_cast<const uint8_t *>(jsonResponseWriteView.data)
      + jsonResponseWriteOffset,
    admitted
  );
  const ar4_protocol::JsonOutputProgressStatus write_status =
    ar4_protocol::record_json_output_chunk(
      jsonResponseWriteView.length,
      admitted,
      written,
      jsonResponseWriteOffset
    );
  if (
    write_status == ar4_protocol::JsonOutputProgressStatus::kBlocked
    || write_status == ar4_protocol::JsonOutputProgressStatus::kProgress
  ) {
    return true;
  }
  if (write_status != ar4_protocol::JsonOutputProgressStatus::kCompleted) {
    latch_json_runtime_fault();
    return true;
  }
  const ar4_protocol::JsonMainControllerResponseKind completed_response_kind =
    jsonResponseWriteView.kind;
  const ar4_protocol::JsonMainControllerOutputCompletionStatus
    completion_status = jsonMainControllerOwner.complete_response_write(
      jsonResponseWriteOffset
    );
  jsonResponseWritePrepared = false;
  jsonResponseWriteOffset = 0;
  jsonResponseWriteView = {};
  if (
    completion_status
      != ar4_protocol::JsonMainControllerOutputCompletionStatus::kCompleted
  ) {
    latch_json_runtime_fault();
  } else {
    if (
      completed_response_kind
        == ar4_protocol::JsonMainControllerResponseKind::
          kPositionDispositionCompleted
      || completed_response_kind
        == ar4_protocol::JsonMainControllerResponseKind::
          kCorrectPositionCompleted
      || completed_response_kind
        == ar4_protocol::JsonMainControllerResponseKind::
          kExternalAxisZeroCompleted
    ) {
      speedViolation = "0";
      flag = "";
    } else if (
      completed_response_kind
        == ar4_protocol::JsonMainControllerResponseKind::
          kCorrectPositionEncoderFailed
    ) {
      flag = "";
    } else if (
      completed_response_kind
        == ar4_protocol::JsonMainControllerResponseKind::kPositionAlarmFailed
      && Alarm != "0"
    ) {
      Alarm = "0";
    }
    complete_json_estop_admission_response();
  }
  return true;
}


bool service_json_event_output() {
  const ar4_protocol::JsonEventOutputState state =
    jsonControllerEventOwner.state();
  if (state == ar4_protocol::JsonEventOutputState::kFaulted) {
    latch_json_runtime_fault();
    return true;
  }
  if (state == ar4_protocol::JsonEventOutputState::kIdle) return false;
  if (!jsonEventWritePrepared) {
    const ar4_protocol::JsonEventOutputBeginStatus begin_status =
      jsonControllerEventOwner.begin_write(jsonEventWriteView);
    if (begin_status != ar4_protocol::JsonEventOutputBeginStatus::kReady) {
      latch_json_runtime_fault();
      return true;
    }
    jsonEventWritePrepared = true;
    jsonEventWriteOffset = 0;
  }
  size_t admitted = 0;
  const ar4_protocol::JsonOutputProgressStatus plan_status =
    ar4_protocol::plan_json_output_chunk(
      jsonEventWriteView.length,
      jsonEventWriteOffset,
      Serial.availableForWrite(),
      admitted
    );
  if (plan_status == ar4_protocol::JsonOutputProgressStatus::kBlocked) {
    return true;
  }
  if (plan_status != ar4_protocol::JsonOutputProgressStatus::kProgress) {
    latch_json_runtime_fault();
    return true;
  }
  const size_t written = Serial.write(
    reinterpret_cast<const uint8_t *>(jsonEventWriteView.data)
      + jsonEventWriteOffset,
    admitted
  );
  const ar4_protocol::JsonOutputProgressStatus write_status =
    ar4_protocol::record_json_output_chunk(
      jsonEventWriteView.length,
      admitted,
      written,
      jsonEventWriteOffset
    );
  if (
    write_status == ar4_protocol::JsonOutputProgressStatus::kBlocked
    || write_status == ar4_protocol::JsonOutputProgressStatus::kProgress
  ) {
    return true;
  }
  if (write_status != ar4_protocol::JsonOutputProgressStatus::kCompleted) {
    latch_json_runtime_fault();
    return true;
  }
  const ar4_protocol::JsonEventOutputCompletionStatus completion_status =
    jsonControllerEventOwner.complete_write(jsonEventWriteOffset);
  jsonEventWritePrepared = false;
  jsonEventWriteOffset = 0;
  jsonEventWriteView = {};
  if (
    completion_status
      != ar4_protocol::JsonEventOutputCompletionStatus::kCompleted
  ) {
    latch_json_runtime_fault();
    return true;
  }
  noInterrupts();
  ar4_protocol::acknowledge_controller_estop_response(
    controllerResponseOwnership
  );
  interrupts();
  return true;
}




bool service_json_output() {
  const bool response_owned = jsonResponseWritePrepared
    || jsonMainControllerOwner.state()
      != ar4_protocol::JsonMainControllerOwnerState::kIdle;
  const bool event_owned = jsonEventWritePrepared
    || jsonControllerEventOwner.state()
      != ar4_protocol::JsonEventOutputState::kIdle;
  switch (ar4_protocol::select_json_output_route(
      false,
      jsonRuntimeFault,
      jsonResponseWritePrepared,
      jsonEventWritePrepared,
      response_owned,
      event_owned
  )) {
    case ar4_protocol::JsonOutputRoute::kFaultSignal:
    case ar4_protocol::JsonOutputRoute::kFaulted:
    case ar4_protocol::JsonOutputRoute::kIdle:
      return false;
    case ar4_protocol::JsonOutputRoute::kResponse:
      return service_json_response_output();
    case ar4_protocol::JsonOutputRoute::kEvent:
      return service_json_event_output();
    case ar4_protocol::JsonOutputRoute::kInvalid:
      latch_json_runtime_fault();
      return true;
  }
  latch_json_runtime_fault();
  return true;
}



void processSerial() {
  if (jsonMainControllerOwner.playback_execution_active()) return;
  if (Serial.available() <= 0) return;
  const ar4_protocol::SerialFrameReadStatus status =
    ar4_protocol::append_serial_frame_byte(
      recData, serialFrameDiscarding, Serial.read());
  ar4_protocol::update_serial_frame_receive_deadline(
    static_cast<uint32_t>(millis()), status, recData.length(),
    serialFrameDiscarding, serialFrameReceiveDeadline);
  if (status == ar4_protocol::SerialFrameReadStatus::kOverflow) {
    queue_json_transport_error(
      "frame_too_large", "request frame exceeds protocol limit");
  } else if (status == ar4_protocol::SerialFrameReadStatus::kComplete) {
    process_json_serial_frame(recData);
    recData = "";
  }
}


bool service_json_gcode_playback() {
  if (jsonMainControllerOwner.active_playback_request_id() == 0) {
    return false;
  }
  if (
    jsonMainControllerOwner.state()
      != ar4_protocol::JsonMainControllerOwnerState::kIdle
  ) {
    return true;
  }
  const ar4_protocol::JsonMainDirectCommandSource direct = {
    ar4_json_direct_runtime::execute,
    nullptr,
  };
  const ar4_protocol::JsonMainControllerProcessStatus status =
    jsonMainControllerOwner.stage_playback_terminal(
      &direct,
      ar4_protocol::kJsonProtocolMaximumPayloadBytes
    );
  apply_json_process_status(status);
  return true;
}









void EstopProg() {
  ar4_protocol::record_estop_interrupt(
    estopAdmissionOwnership,
    estopActive,
    controllerResponseOwnership,
    telemetryResponseOwnership
  );
}




bool wait_for_controller_duration(
  uint32_t duration_ms,
  bool include_admission_blocks
) {
  const unsigned long started_at = millis();
  while (millis() - started_at < duration_ms) {
    if (estopActive) break;
    if (
      include_admission_blocks
      && controller_mutation_estop_blocked()
    ) return false;
    delay(1);
  }
  return !estopActive
    && (
      !include_admission_blocks
      || !controller_mutation_estop_blocked()
    );
}



void complete_controller_response_scope() {
  noInterrupts();
  const bool completed =
    ar4_protocol::complete_controller_response_ownership(
      controllerResponseOwnership
    );
  const bool publishEstop = completed
    && controllerResponseOwnership.estop_response_pending;
  interrupts();
  if (!publishEstop) return;
  if (jsonRuntimeFault) return;
  if (
    jsonMainControllerOwner.state()
      != ar4_protocol::JsonMainControllerOwnerState::kIdle
    || jsonResponseWritePrepared
    || jsonControllerEventOwner.state()
      != ar4_protocol::JsonEventOutputState::kIdle
    || jsonEventWritePrepared
  ) {
    return;
  }
  const ar4_protocol::JsonEventQueueStatus queue_status =
    jsonControllerEventOwner.queue_emergency_stop(
      true,
      ar4_protocol::kJsonProtocolMaximumPayloadBytes
    );
  if (queue_status != ar4_protocol::JsonEventQueueStatus::kResponseReady) {
    latch_json_runtime_fault();
    return;
  }
  service_json_event_output();
}


class ControllerResponseScope {
 public:
  ControllerResponseScope() : active_(false) {
    noInterrupts();
    active_ = ar4_protocol::begin_controller_response_ownership(
      controllerResponseOwnership
    );
    interrupts();
  }

  ~ControllerResponseScope() {
    if (active_) complete_controller_response_scope();
  }

 private:
  bool active_;
};



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//MAIN
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  // run once:
  Serial.begin(9600);
  Serial8.begin(38400);  // Use Serial8 (pins 34 and 35)
  // There is no Serial.print before this line
  const ar4_protocol::PersistenceMigrationStatus migration_status =
    ar4_protocol::migrate_legacy_persistence(EEPROM);
  if (
    migration_status
    == ar4_protocol::PersistenceMigrationStatus::kFailed
  ) {
    identity_record_status = ar4_protocol::IdentityRecordStatus::kCorrupt;
    clear_robot_identity();
  } else {
    load_robot_id_from_eeprom();
  }
  const bool hardware_id_available =
    ar4_protocol::format_controller_hardware_id(
      HW_OCOTP_MAC0 & 0xFFFFFFu,
      controller_hardware_id,
      sizeof(controller_hardware_id)
    );
  if (!hardware_id_available) {
    json_session_identity_status =
      ar4_protocol::JsonSessionIdentityStatus::kInvalidHardwareIdentity;
  } else {
    json_session_identity_status =
      ar4_protocol::advance_json_session_identity(
        EEPROM,
        controller_hardware_id,
        json_session_id,
        sizeof(json_session_id)
      );
  }


  // Initialize Modbus communication
  node.begin(1, Serial8);

  pinMode(J1stepPin, OUTPUT);
  pinMode(J1dirPin, OUTPUT);
  pinMode(J2stepPin, OUTPUT);
  pinMode(J2dirPin, OUTPUT);
  pinMode(J3stepPin, OUTPUT);
  pinMode(J3dirPin, OUTPUT);
  pinMode(J4stepPin, OUTPUT);
  pinMode(J4dirPin, OUTPUT);
  pinMode(J5stepPin, OUTPUT);
  pinMode(J5dirPin, OUTPUT);
  pinMode(J6stepPin, OUTPUT);
  pinMode(J6dirPin, OUTPUT);
  pinMode(J7stepPin, OUTPUT);
  pinMode(J7dirPin, OUTPUT);
  pinMode(J8stepPin, OUTPUT);
  pinMode(J8dirPin, OUTPUT);
  pinMode(J9stepPin, OUTPUT);
  pinMode(J9dirPin, OUTPUT);

  pinMode(J1calPin, INPUT);
  pinMode(J2calPin, INPUT);
  pinMode(J3calPin, INPUT);
  pinMode(J4calPin, INPUT);
  pinMode(J5calPin, INPUT);
  pinMode(J6calPin, INPUT);
  pinMode(J7calPin, INPUT);
  pinMode(J8calPin, INPUT);
  pinMode(J9calPin, INPUT);

  pinMode(EstopPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(EstopPin), EstopProg, LOW);

  digitalWrite(J1stepPin, HIGH);
  digitalWrite(J2stepPin, HIGH);
  digitalWrite(J3stepPin, HIGH);
  digitalWrite(J4stepPin, HIGH);
  digitalWrite(J5stepPin, HIGH);
  digitalWrite(J6stepPin, HIGH);
  digitalWrite(J7stepPin, HIGH);
  digitalWrite(J8stepPin, HIGH);
  digitalWrite(J9stepPin, HIGH);

  flag = "";
  rndTrue = false;
}

void loop() {
  ControllerResponseScope responseScope;
  if (ar4_protocol::serial_frame_receive_timed_out(
      static_cast<uint32_t>(millis()), serialFrameReceiveDeadline
  )) {
    const bool liveMotionActive =
      jsonLiveMotion.owner.state != JsonLiveMotionRuntimeState::kIdle;
    ar4_protocol::expire_serial_frame_receive(
      recData, serialFrameDiscarding, serialFrameReceiveDeadline
    );
    if (liveMotionActive) latch_json_runtime_fault();
    else queue_json_transport_error(
      "frame_timeout", "request frame receive deadline expired"
    );
  }
  bool outputOwned = service_json_output();
  if (jsonRuntimeFault) return;
  bool playbackOwned = false;
  if (!outputOwned) {
    playbackOwned = service_json_gcode_playback();
    outputOwned = service_json_output();
  }
  bool liveOwned = false;
  if (!outputOwned && !playbackOwned) {
    liveOwned = service_json_live_motion(false);
    outputOwned = service_json_output();
  }
  const bool controlPending = Serial.available() > 0
    || recData.length() != 0 || serialFrameDiscarding;
  if (
    !outputOwned
    && !playbackOwned
    && (!liveOwned || controlPending)
  ) processSerial();
  if (!outputOwned && !playbackOwned && liveOwned) {
    service_json_live_motion(!controlPending);
  }
  service_json_output();
}
