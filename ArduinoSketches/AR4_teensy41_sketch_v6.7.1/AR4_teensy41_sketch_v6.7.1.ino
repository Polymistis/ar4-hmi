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
const char *FIRMWARE_VERSION = "6.7.1-ar4hmi.1";
const char *JT_WRIST_CONFIG_CAPABILITY = "JT_WRIST_CONFIG_V1";

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
#include "command_queue_contract.h"
#include "controller_domain_contract.h"
#include "cartesian_pose_contract.h"
#include "debug_contract.h"
#include "identity_contract.h"
#include "motion_command_parse_contract.h"
#include "motion_mode_transaction.h"
#include "numeric_parse_contract.h"
#include "persistence_contract.h"
#include "serial_frame_contract.h"
#include "spline_response_contract.h"
#include "tool_jog_contract.h"
#include "wrist_selection_contract.h"
#pragma GCC diagnostic ignored "-Warray-bounds"
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wsequence-point"
#pragma GCC diagnostic ignored "-Wunused-value"
#pragma GCC diagnostic ignored "-Wunused-function"
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#pragma GCC diagnostic ignored "-Waddress"
#pragma GCC diagnostic ignored "-Wall"

bool DEBUG = false;
// These Debug printers do nothing unless DEBUG = true
#define DEBUG_PRINT(x) \
  do { \
    if (DEBUG) Serial.print(x); \
  } while (0)
#define DEBUG_PRINTLN(x) \
  do { \
    if (DEBUG) Serial.println(x); \
  } while (0)

#define Table_Size 6
typedef float Matrix4x4[16];
typedef float tRobot[66];

String cmdBuffer1;
String cmdBuffer2;
String cmdBuffer3;
String inData;
String recData;
String checkData;
String function;
bool serialFrameDiscarding = false;
volatile byte state = LOW;

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
typedef float tRobotJoints[ROBOT_nDOFs];
typedef float tRobotPose[ROBOT_nDOFs];

//declare in out vars
float xyzuvw_Out[ROBOT_nDOFs];
float xyzuvw_In[ROBOT_nDOFs];
float xyzuvw_Temp[ROBOT_nDOFs];

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

float J7_In;
float J8_In;
float J9_In;

float pose[16];

String moveSequence;

//define rounding vars
float rndArcStart[6];
float rndArcMid[6];
float rndArcEnd[6];
float rndCalcCen[6];
String rndData;
bool rndTrue;
float rndSpeed;
bool splineTrue;
bool splineEndReceived;
volatile bool estopActive;

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
using ar4_protocol::parse_int_spans;

bool parse_loop_mode_span(
  const String &command,
  int begin,
  int end,
  int (&outputs)[ROBOT_nDOFs]
) {
  return ar4_protocol::parse_binary_digit_span(
    command,
    begin,
    end,
    outputs
  );
}

bool parse_loop_modes(
  const String &command,
  int marker,
  int (&outputs)[ROBOT_nDOFs]
) {
  if (marker < 0) return false;
  return parse_loop_mode_span(
    command,
    marker + 2,
    static_cast<int>(command.length()),
    outputs
  );
}

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

bool primary_positions_to_future_steps(
  const float (&positions)[ROBOT_nDOFs],
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
        positions[axis],
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

// EEPROM Memory Map
constexpr int EEPROM_ROBOT_MODEL_ADDR = ar4_protocol::kRobotModelAddress;
constexpr int EEPROM_ROBOT_VERSION_ADDR = ar4_protocol::kRobotVersionAddress;
constexpr int EEPROM_DRIVER_BOARD_ADDR = ar4_protocol::kDriverBoardAddress;
constexpr int EEPROM_SERIAL_NUMBER_ADDR = ar4_protocol::kSerialNumberAddress;
constexpr int EEPROM_ASSET_TAG_ADDR = ar4_protocol::kAssetTagAddress;

// Defaults represent an identity record whose commit marker is absent.
const char *DEFAULT_ROBOT_MODEL = "Unset";
const char *DEFAULT_ROBOT_VERSION = "Unset";
const char *DEFAULT_DRIVER_BOARD = "Unset";
const char *DEFAULT_SERIAL_NUMBER = "Unset";
const char *DEFAULT_ASSET_TAG = "Unset";
const bool DEFAULT_DEBUG = DEBUG;

String robot_model = DEFAULT_ROBOT_MODEL;
String robot_version = DEFAULT_ROBOT_VERSION;
String driver_board = DEFAULT_DRIVER_BOARD;
String serial_number = DEFAULT_SERIAL_NUMBER;
String asset_tag = DEFAULT_ASSET_TAG;
ar4_protocol::IdentityRecordStatus identity_record_status =
  ar4_protocol::IdentityRecordStatus::kUninitialized;


String validated_identity_field(String value) {
  if (!ar4_protocol::identity_field_valid(value.c_str())) return "";
  return value;
}

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

bool write_identity_field_to_eeprom(int address, const String& value) {
  return ar4_protocol::write_identity_field(
    EEPROM,
    address,
    value.c_str()
  );
}

// ============================================================================
// EEPROM Functions
// ============================================================================

void load_debug_from_eeprom() {
  bool debugBuf = false;
  if (!ar4_protocol::load_debug_record(EEPROM, debugBuf)) {
    Serial.println("Debug persistence not initialized or invalid in load_debug");
    return;
  }
  DEBUG = debugBuf;
  if (DEBUG) {
    DEBUG_PRINTLN("Loaded DEBUG=True from EEPROM - Setting DEBUG to True");
  }
}

bool save_debug_to_eeprom(bool value) {
  if (!ar4_protocol::save_debug_record(EEPROM, value)) {
    Serial.println("Error saving Debug Persistence - Transaction failed");
    return false;
  }
  return true;
}

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
    Serial.println("EEPROM not initialized in load_robot_id");
    use_default_robot_identity();
    return;
  }
  if (
    identity_record_status
    != ar4_protocol::IdentityRecordStatus::kValid
  ) {
    Serial.println("EEPROM identity record corrupt in load_robot_id");
    identity_record_status = ar4_protocol::IdentityRecordStatus::kCorrupt;
    clear_robot_identity();
    return;
  }

  robot_model = stored_robot_model;
  robot_version = stored_robot_version;
  driver_board = stored_driver_board;
  serial_number = stored_serial_number;
  asset_tag = stored_asset_tag;
  DEBUG_PRINT("Debug - Loaded Robot Model from EEPROM: ");
  DEBUG_PRINTLN(robot_model);
  DEBUG_PRINT("Debug - Loaded Robot Version from EEPROM: ");
  DEBUG_PRINTLN(robot_version);
  DEBUG_PRINT("Debug - Loaded Driver Board from EEPROM: ");
  DEBUG_PRINTLN(driver_board);
  DEBUG_PRINT("Debug - Loaded Serial Number from EEPROM: ");
  DEBUG_PRINTLN(serial_number);
  DEBUG_PRINT("Debug - Loaded Asset Tag from EEPROM: ");
  DEBUG_PRINTLN(asset_tag);
}

bool save_robot_id_to_eeprom(const String robot_model, const String robot_version, const String driver_board, const String serial_number, const String asset_tag) {
  /*
     * Save robot model and version to EEPROM.
     * 
     * Args:
     *   robot_model: Robot model string (max 31 chars)
     *   robot_version: Robot version string (max 31 chars)
     *   driver_board: Driver board string (max 31 chars)
     *   serial_number: Serial number string (max 31 chars)
     *   asset_tag: Asset Tag string (max 31 chars)
     */
  if (
    !ar4_protocol::identity_field_valid(robot_model.c_str())
    || !ar4_protocol::identity_field_valid(robot_version.c_str())
    || !ar4_protocol::identity_field_valid(driver_board.c_str())
    || !ar4_protocol::identity_field_valid(serial_number.c_str())
    || !ar4_protocol::identity_field_valid(asset_tag.c_str())
  ) {
    return false;
  }
  return ar4_protocol::save_identity_record(EEPROM, [&]() {
    return write_identity_field_to_eeprom(EEPROM_ROBOT_MODEL_ADDR, robot_model)
      && write_identity_field_to_eeprom(EEPROM_ROBOT_VERSION_ADDR, robot_version)
      && write_identity_field_to_eeprom(EEPROM_DRIVER_BOARD_ADDR, driver_board)
      && write_identity_field_to_eeprom(EEPROM_SERIAL_NUMBER_ADDR, serial_number)
      && write_identity_field_to_eeprom(EEPROM_ASSET_TAG_ADDR, asset_tag);
  });
}

void reboot() {
  DEBUG_PRINT("Rebooting Driver Board: ");
  DEBUG_PRINTLN(driver_board);
  if (driver_board.indexOf("Teensy") >= 0) {
    DEBUG_PRINTLN("Teensy 3.x / 4.x: ARM system reset");
    SCB_AIRCR = 0x05FA0004;
    while (true)
      ;
  } else {
    // Unknown type — fallback or safe no-op
    Serial.println("Unknown board type, no reboot performed.");
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////
// Persistent Hardware / Query Functions
/////////////////////////////////////////////////////////////////////////////////////////////////////

void handle_hello_command() {
  if (
    identity_record_status
    == ar4_protocol::IdentityRecordStatus::kCorrupt
  ) {
    Serial.println("ER");
    return;
  }
  char response[ar4_protocol::kIdentityJsonCapacity] = { 0 };
  if (!ar4_protocol::build_identity_json(
      driver_board.c_str(),
      FIRMWARE_VERSION,
      robot_model.c_str(),
      robot_version.c_str(),
      serial_number.c_str(),
      asset_tag.c_str(),
      JT_WRIST_CONFIG_CAPABILITY,
      response,
      sizeof(response)
  )) {
    Serial.println("ER");
    return;
  }
  Serial.println(response);
}


void handle_set_robot_id_command(String new_robot_model, String new_robot_version, String new_driver_board, String new_serial_number, String new_asset_tag) {
  new_robot_model = validated_identity_field(new_robot_model);
  new_robot_version = validated_identity_field(new_robot_version);
  new_driver_board = validated_identity_field(new_driver_board);
  new_serial_number = validated_identity_field(new_serial_number);
  new_asset_tag = validated_identity_field(new_asset_tag);
  if (
    new_robot_model.length() == 0
    || new_robot_version.length() == 0
    || new_driver_board.length() == 0
    || new_serial_number.length() == 0
    || new_asset_tag.length() == 0
  ) {
    Serial.println("Error: Invalid robot identity field");
    return;
  }
  if (!save_robot_id_to_eeprom(
      new_robot_model,
      new_robot_version,
      new_driver_board,
      new_serial_number,
      new_asset_tag
  )) {
    identity_record_status = ar4_protocol::IdentityRecordStatus::kCorrupt;
    clear_robot_identity();
    Serial.println("Error: Robot identity persistence failed");
    return;
  }
  robot_model = new_robot_model;
  robot_version = new_robot_version;
  driver_board = new_driver_board;
  serial_number = new_serial_number;
  asset_tag = new_asset_tag;
  identity_record_status = ar4_protocol::IdentityRecordStatus::kValid;
  Serial.println("Done");
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
  // Serial.println("Sol : " + String(solVal) + " Nb sol : " + String(NumberOfSol));

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

void sendRobotPos() {

  updatePos();

  String sendPos = "A" + String(JangleIn[0], 3) + "B" + String(JangleIn[1], 3) + "C" + String(JangleIn[2], 3) + "D" + String(JangleIn[3], 3) + "E" + String(JangleIn[4], 3) + "F" + String(JangleIn[5], 3) + "G" + String(xyzuvw_Out[0], 3) + "H" + String(xyzuvw_Out[1], 3) + "I" + String(xyzuvw_Out[2], 3) + "J" + String(xyzuvw_Out[3], 3) + "K" + String(xyzuvw_Out[4], 3) + "L" + String(xyzuvw_Out[5], 3) + "M" + speedViolation + "N" + debug + "O" + flag + "P" + J7_pos + "Q" + J8_pos + "R" + J9_pos;
  delay(5);
  Serial.println(sendPos);
  speedViolation = "0";
  flag = "";
}

void sendRobotPosSpline() {

  updatePos();

  String sendPos = "A" + String(JangleIn[0], 3) + "B" + String(JangleIn[1], 3) + "C" + String(JangleIn[2], 3) + "D" + String(JangleIn[3], 3) + "E" + String(JangleIn[4], 3) + "F" + String(JangleIn[5], 3) + "G" + String(xyzuvw_Out[0], 3) + "H" + String(xyzuvw_Out[1], 3) + "I" + String(xyzuvw_Out[2], 3) + "J" + String(xyzuvw_Out[3], 3) + "K" + String(xyzuvw_Out[4], 3) + "L" + String(xyzuvw_Out[5], 3) + "M" + speedViolation + "N" + debug + "O" + flag + "P" + J7_pos + "Q" + J8_pos + "R" + J9_pos;
  delay(5);
  Serial.println(sendPos);
  speedViolation = "0";
}

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



void correctRobotPos() {

  J1StepM = J1encPos.read() / J1encMult;
  J2StepM = J2encPos.read() / J2encMult;
  J3StepM = J3encPos.read() / J3encMult;
  J4StepM = J4encPos.read() / J4encMult;
  J5StepM = J5encPos.read() / J5encMult;
  J6StepM = J6encPos.read() / J6encMult;

  JangleIn[0] = (J1StepM - J1zeroStep) / J1StepDeg;
  JangleIn[1] = (J2StepM - J2zeroStep) / J2StepDeg;
  JangleIn[2] = (J3StepM - J3zeroStep) / J3StepDeg;
  JangleIn[3] = (J4StepM - J4zeroStep) / J4StepDeg;
  JangleIn[4] = (J5StepM - J5zeroStep) / J5StepDeg;
  JangleIn[5] = (J6StepM - J6zeroStep) / J6StepDeg;


  SolveFowardKinematics();

  String sendPos = "A" + String(JangleIn[0], 3) + "B" + String(JangleIn[1], 3) + "C" + String(JangleIn[2], 3) + "D" + String(JangleIn[3], 3) + "E" + String(JangleIn[4], 3) + "F" + String(JangleIn[5], 3) + "G" + String(xyzuvw_Out[0], 3) + "H" + String(xyzuvw_Out[1], 3) + "I" + String(xyzuvw_Out[2], 3) + "J" + String(xyzuvw_Out[3], 3) + "K" + String(xyzuvw_Out[4], 3) + "L" + String(xyzuvw_Out[5], 3) + "M" + speedViolation + "N" + debug + "O" + flag + "P" + J7_pos + "Q" + J8_pos + "R" + J9_pos;
  delay(5);
  Serial.println(sendPos);
  speedViolation = "0";
  flag = "";
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//SD CARD
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


static bool sd_ok = false;

const char *sdCodeMeaning(uint8_t code) {
  switch (code) {
    case 0x04: return "CMD17 read block failed (card error response)";
    case 0x0A: return "ACMD41 init timeout";
    case 0x02: return "CMD8 not accepted (not SD/unsupported)";
    default: return "see code/data";
  }
}

String egSD(const char *stage) {
  uint8_t code = SD.sdfs.sdErrorCode();
  uint8_t data = SD.sdfs.sdErrorData();

  String s = "EG: ";
  s += stage;
  s += " | code=0x";
  s += String(code, HEX);
  s += " data=0x";
  s += String(data, HEX);
  s += " | ";
  s += sdCodeMeaning(code);
  return s;
}

bool initSD() {
  if (sd_ok) return true;
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println(egSD("begin fail"));
    return false;
  }
  sd_ok = true;
  return true;
}

bool writeSD(const String &filename, const String &info) {
  if (!initSD()) return false;

  File f = SD.open(filename.c_str(), FILE_WRITE);
  if (!f) {
    sd_ok = false;
    Serial.println(egSD("open fail"));
    return false;
  }
  const size_t written = f.println(info);
  f.flush();
  const bool succeeded = written == info.length() + 2
    && f.getWriteError() == 0;
  f.close();
  if (!succeeded) {
    sd_ok = false;
    Serial.println(egSD("write fail"));
  }
  return succeeded;
}

bool deleteSD(const String &filename) {
  if (!initSD()) return false;
  if (!SD.remove(filename.c_str())) {
    sd_ok = false;
    Serial.println(egSD("remove fail"));
    return false;
  }
  return true;
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

void printDirectory(File dir, int numTabs) {
  String filesSD;
  while (true) {

    File entry = dir.openNextFile();
    if (!entry) {
      // no more files
      Serial.println(filesSD);
      break;
    }
    if (entry.name() != "System Volume Information") {
      filesSD += entry.name();
      filesSD += ",";
    }
    entry.close();
  }
}




/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//DRIVE LIMIT
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void driveLimit(const int steps[], float SpeedVal) {

  const unsigned long DEBOUNCE_US = 3000;  // 3 ms
  unsigned long firstHighUs[numJoints] = { 0 };

  int calcStepGap = minSpeedDelay / (SpeedVal / 100);

  // Define arrays for calibration directions, motor directions, and direction pins
  const uint8_t limitSensor[numJoints] = { HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH };
  int calDir[numJoints] = { J1CalDir, J2CalDir, J3CalDir, J4CalDir, J5CalDir, J6CalDir, J7CalDir, J8CalDir, J9CalDir };
  int motDir[numJoints] = { J1MotDir, J2MotDir, J3MotDir, J4MotDir, J5MotDir, J6MotDir, J7MotDir, J8MotDir, J9MotDir };
  int dirPins[numJoints] = { J1dirPin, J2dirPin, J3dirPin, J4dirPin, J5dirPin, J6dirPin, J7dirPin, J8dirPin, J9dirPin };

  // Define arrays for current state, calibration pins, step pins, completion status, and steps done
  int curState[numJoints] = { 0 };
  int calPins[numJoints] = { J1calPin, J2calPin, J3calPin, J4calPin, J5calPin, J6calPin, J7calPin, J8calPin, J9calPin };
  int stepPins[numJoints] = { J1stepPin, J2stepPin, J3stepPin, J4stepPin, J5stepPin, J6stepPin, J7stepPin, J8stepPin, J9stepPin };

  int stepsDone[numJoints] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  int complete[numJoints] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };

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

  for (int i = 0; i < numJoints; i++) {
    // Set complete if joint was not sent a limit value
    if (steps[i] == 0) {
      complete[i] = 1;
    }
  }

  // DRIVE MOTORS FOR CALIBRATION
  int DriveLimInProc = 1;

  while (DriveLimInProc == 1 && estopActive == false) {

    for (int i = 0; i < numJoints; i++) {

      if (complete[i] == 1) {
        continue;
      }

      curState[i] = digitalRead(calPins[i]);

      // Debounced limit detection, but stop immediately on first detection
      if (curState[i] == limitSensor[i]) {

        if (firstHighUs[i] == 0) {
          firstHighUs[i] = micros();
          limitSeen[i] = 1;  // stop stepping this axis immediately
        }

        if ((micros() - firstHighUs[i]) >= DEBOUNCE_US) {
          complete[i] = 1;
        }

      } else {
        firstHighUs[i] = 0;
        limitSeen[i] = 0;
      }

      // Step the motor only if the limit has not been seen yet
      if (stepsDone[i] < steps[i] && complete[i] == 0 && limitSeen[i] == 0) {
        digitalWrite(stepPins[i], HIGH);
        delayMicroseconds(5);
        digitalWrite(stepPins[i], LOW);

        stepsDone[i]++;

        delayMicroseconds(calcStepGap);

      } else if (stepsDone[i] >= steps[i] && complete[i] == 0) {
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

    delayMicroseconds(100);
  }
}


void backOff(uint8_t J1req, uint8_t J2req, uint8_t J3req, uint8_t J4req, uint8_t J5req,
             uint8_t J6req, uint8_t J7req, uint8_t J8req, uint8_t J9req,
             float SpeedVal,
             int backoffSteps) {

  int calcStepGap = minSpeedDelay / (SpeedVal / 100);

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

  auto pulseStep = [](uint8_t pin) {
    digitalWrite(pin, HIGH);
    delayMicroseconds(5);
    digitalWrite(pin, LOW);
  };

  for (int step = 0; step < backoffSteps && estopActive == false; step++) {

    if (J1req == 1) pulseStep(J1stepPin);
    if (J2req == 1) pulseStep(J2stepPin);
    if (J3req == 1) pulseStep(J3stepPin);
    if (J4req == 1) pulseStep(J4stepPin);
    if (J5req == 1) pulseStep(J5stepPin);
    if (J6req == 1) pulseStep(J6stepPin);
    if (J7req == 1) pulseStep(J7stepPin);
    if (J8req == 1) pulseStep(J8stepPin);
    if (J9req == 1) pulseStep(J9stepPin);

    delayMicroseconds(calcStepGap);
  }
}




/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//CHECK ENCODERS
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void resetEncoders() {

  J1collisionTrue = 0;
  J2collisionTrue = 0;
  J3collisionTrue = 0;
  J4collisionTrue = 0;
  J5collisionTrue = 0;
  J6collisionTrue = 0;

  //set encoders to current position
  J1encPos.write(J1StepM * J1encMult);
  J2encPos.write(J2StepM * J2encMult);
  J3encPos.write(J3StepM * J3encMult);
  J4encPos.write(J4StepM * J4encMult);
  J5encPos.write(J5StepM * J5encMult);
  J6encPos.write(J6StepM * J6encMult);
  //delayMicroseconds(5);
}

void checkEncoders() {
  //read encoders
  J1EncSteps = J1encPos.read() / J1encMult;
  J2EncSteps = J2encPos.read() / J2encMult;
  J3EncSteps = J3encPos.read() / J3encMult;
  J4EncSteps = J4encPos.read() / J4encMult;
  J5EncSteps = J5encPos.read() / J5encMult;
  J6EncSteps = J6encPos.read() / J6encMult;

  if (abs((J1EncSteps - J1StepM)) >= encOffset) {
    if (JointLoopModes[0] == 0) {
      J1collisionTrue = 1;
      J1StepM = J1encPos.read() / J1encMult;
    }
  }
  if (abs((J2EncSteps - J2StepM)) >= encOffset) {
    if (JointLoopModes[1] == 0) {
      J2collisionTrue = 1;
      J2StepM = J2encPos.read() / J2encMult;
    }
  }
  if (abs((J3EncSteps - J3StepM)) >= encOffset) {
    if (JointLoopModes[2] == 0) {
      J3collisionTrue = 1;
      J3StepM = J3encPos.read() / J3encMult;
    }
  }
  if (abs((J4EncSteps - J4StepM)) >= encOffset) {
    if (JointLoopModes[3] == 0) {
      J4collisionTrue = 1;
      J4StepM = J4encPos.read() / J4encMult;
    }
  }
  if (abs((J5EncSteps - J5StepM)) >= encOffset) {
    if (JointLoopModes[4] == 0) {
      J5collisionTrue = 1;
      J5StepM = J5encPos.read() / J5encMult;
    }
  }
  if (abs((J6EncSteps - J6StepM)) >= encOffset) {
    if (JointLoopModes[5] == 0) {
      J6collisionTrue = 1;
      J6StepM = J6encPos.read() / J6encMult;
    }
  }

  TotalCollision = J1collisionTrue + J2collisionTrue + J3collisionTrue + J4collisionTrue + J5collisionTrue + J6collisionTrue;
  if (TotalCollision > 0) {
    flag = "EC" + String(J1collisionTrue) + String(J2collisionTrue) + String(J3collisionTrue) + String(J4collisionTrue) + String(J5collisionTrue) + String(J6collisionTrue);
  }
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

bool driveMotorsJ(int J1step, int J2step, int J3step, int J4step, int J5step, int J6step, int J7step, int J8step, int J9step,
                  int J1dir, int J2dir, int J3dir, int J4dir, int J5dir, int J6dir, int J7dir, int J8dir, int J9dir,
                  String SpeedType, float SpeedVal, float ACCspd, float DCCspd, float ACCramp,
                  FirmwareMotionModeTransaction *motionModes) {
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
  if (!ar4_protocol::valid_delay_envelope(
      calcStepGap,
      startDelay,
      endDelay,
      rndTrue,
      rndSpeed
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

  // Linear ramp decrements/increments per step
  float calcACCstepInc = (ACCStep > 0.0f) ? (startDelay - calcStepGap) / ACCStep : 0.0f;  // subtract each step
  float calcDCCstepInc = (DCCStep > 0.0f) ? (endDelay - calcStepGap) / DCCStep : 0.0f;    // add each step

  // Start at the slow end of accel (or keep rounding behavior)
  float calcACCstartDel = startDelay;
  float curDelay = (rndTrue == true) ? rndSpeed : calcACCstartDel;
  rndTrue = false;

  ///// DRIVE MOTORS /////
  unsigned long moveStart = micros();
  int highStepCur = 0;

  while ((cur[0] < steps[0] || cur[1] < steps[1] || cur[2] < steps[2] || cur[3] < steps[3] || cur[4] < steps[4] || cur[5] < steps[5] || cur[6] < steps[6] || cur[7] < steps[7] || cur[8] < steps[8]) && estopActive == false) {

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
    delayMicroseconds(pulseDelay);
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
  if (HighStep == 0) return true;

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
  if (HighStep == 0) return true;
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

  // Process lookahead
  if (splineTrue) {
    processSerial();
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

    // Process lookahead
    if (splineTrue) {
      processSerial();
    }

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
  return true;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//MOVE J
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

ar4_protocol::MotionCommandStatus moveJ(
  String inData,
  bool response,
  bool precalc,
  bool simspeed
) {

  int J1dir;
  int J2dir;
  int J3dir;
  int J4dir;
  int J5dir;
  int J6dir;
  int J7dir;
  int J8dir;
  int J9dir;

  int J1axisFault = 0;
  int J2axisFault = 0;
  int J3axisFault = 0;
  int J4axisFault = 0;
  int J5axisFault = 0;
  int J6axisFault = 0;
  int J7axisFault = 0;
  int J8axisFault = 0;
  int J9axisFault = 0;
  int TotalAxisFault = 0;

  ar4_protocol::CartesianMoveCommandFields commandFields = {};
  if (!ar4_protocol::parse_cartesian_move_command(inData, commandFields)) {
    return ar4_protocol::MotionCommandStatus::kRejected;
  }
  int staged_external_steps[3];
  if (!external_positions_to_future_steps(
      commandFields.auxiliary[0],
      commandFields.auxiliary[1],
      commandFields.auxiliary[2],
      staged_external_steps
  )) {
    return ar4_protocol::MotionCommandStatus::kRejected;
  }

  for (int axis = 0; axis < ROBOT_nDOFs; ++axis) {
    xyzuvw_In[axis] = commandFields.pose[axis];
  }
  J7_In = commandFields.auxiliary[0];
  J8_In = commandFields.auxiliary[1];
  J9_In = commandFields.auxiliary[2];
  String SpeedType(commandFields.speed_mode);
  float SpeedVal = commandFields.speed;
  float ACCspd = commandFields.acceleration;
  float DCCspd = commandFields.deceleration;
  float ACCramp = commandFields.ramp;
  ar4_protocol::MotionModeTransaction<String, ROBOT_nDOFs> motionModes(
    WristCon,
    JointLoopModes,
    String(commandFields.wrist_config),
    commandFields.loop_modes
  );


  SolveInverseKinematics(commandFields.wrist_config);

  //calc destination motor steps
  int future_steps[numJoints] = {};
  int primary_future_steps[ROBOT_nDOFs] = {};
  if (
    KinematicError != 0
    || !primary_inverse_solution_to_future_steps(primary_future_steps)
  ) {
    return ar4_protocol::MotionCommandStatus::kRejected;
  }
  for (int axis = 0; axis < ROBOT_nDOFs; ++axis) {
    future_steps[axis] = primary_future_steps[axis];
  }
  for (int axis = 0; axis < 3; ++axis) {
    future_steps[axis + ROBOT_nDOFs] = staged_external_steps[axis];
  }
  int J1futStepM = future_steps[0];
  int J2futStepM = future_steps[1];
  int J3futStepM = future_steps[2];
  int J4futStepM = future_steps[3];
  int J5futStepM = future_steps[4];
  int J6futStepM = future_steps[5];
  int J7futStepM = future_steps[6];
  int J8futStepM = future_steps[7];
  int J9futStepM = future_steps[8];

  if (precalc) {
    J1StepM = J1futStepM;
    J2StepM = J2futStepM;
    J3StepM = J3futStepM;
    J4StepM = J4futStepM;
    J5StepM = J5futStepM;
    J6StepM = J6futStepM;
    J7StepM = J7futStepM;
    J8StepM = J8futStepM;
    J9StepM = J9futStepM;
  }

  else {
    //calc delta from current to destination
    int J1stepDif = J1StepM - J1futStepM;
    int J2stepDif = J2StepM - J2futStepM;
    int J3stepDif = J3StepM - J3futStepM;
    int J4stepDif = J4StepM - J4futStepM;
    int J5stepDif = J5StepM - J5futStepM;
    int J6stepDif = J6StepM - J6futStepM;
    int J7stepDif = J7StepM - J7futStepM;
    int J8stepDif = J8StepM - J8futStepM;
    int J9stepDif = J9StepM - J9futStepM;

    //determine motor directions
    J1dir = (J1stepDif <= 0) ? 1 : 0;
    J2dir = (J2stepDif <= 0) ? 1 : 0;
    J3dir = (J3stepDif <= 0) ? 1 : 0;
    J4dir = (J4stepDif <= 0) ? 1 : 0;
    J5dir = (J5stepDif <= 0) ? 1 : 0;
    J6dir = (J6stepDif <= 0) ? 1 : 0;
    J7dir = (J7stepDif <= 0) ? 1 : 0;
    J8dir = (J8stepDif <= 0) ? 1 : 0;
    J9dir = (J9stepDif <= 0) ? 1 : 0;

    int StepLim[numJoints] = { J1StepLim, J2StepLim, J3StepLim, J4StepLim, J5StepLim, J6StepLim, J7StepLim, J8StepLim, J9StepLim };
    int axisFault[numJoints] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };

    // Future counters are the safety boundary; signed deltas are not positions.
    for (int i = 0; i < numJoints; ++i) {
      if (future_step_is_outside_limit(future_steps[i], StepLim[i])) {
        axisFault[i] = 1;
      }
    }

    // Assign fault values back to individual variables
    J1axisFault = axisFault[0];
    J2axisFault = axisFault[1];
    J3axisFault = axisFault[2];
    J4axisFault = axisFault[3];
    J5axisFault = axisFault[4];
    J6axisFault = axisFault[5];
    J7axisFault = axisFault[6];
    J8axisFault = axisFault[7];
    J9axisFault = axisFault[8];

    // Calculate total axis fault
    TotalAxisFault = 0;
    for (int i = 0; i < numJoints; ++i) {
      TotalAxisFault += axisFault[i];
    }

    if (TotalAxisFault == 0 && KinematicError == 0) {
      resetEncoders();
      bool drive_succeeded = false;
      if (simspeed) {
        drive_succeeded = driveMotorsG(abs(J1stepDif), abs(J2stepDif), abs(J3stepDif), abs(J4stepDif), abs(J5stepDif), abs(J6stepDif), abs(J7stepDif), abs(J8stepDif), abs(J9stepDif), J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, J7dir, J8dir, J9dir, SpeedType, SpeedVal, ACCspd, DCCspd, ACCramp, &motionModes);
      } else {
        drive_succeeded = driveMotorsJ(abs(J1stepDif), abs(J2stepDif), abs(J3stepDif), abs(J4stepDif), abs(J5stepDif), abs(J6stepDif), abs(J7stepDif), abs(J8stepDif), abs(J9stepDif), J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, J7dir, J8dir, J9dir, SpeedType, SpeedVal, ACCspd, DCCspd, ACCramp, &motionModes);
      }
      if (!drive_succeeded) {
        return ar4_protocol::MotionCommandStatus::kRejected;
      }
      checkEncoders();
      if (TotalCollision > 0) {
        sendRobotPos();
        return ar4_protocol::MotionCommandStatus::kTerminalFaultReported;
      }
      if (response == true) {
        sendRobotPos();
      }
    } else if (KinematicError == 1) {
      Alarm = "ER";
      delay(5);
      Serial.println(Alarm);
      Alarm = "0";
      return ar4_protocol::MotionCommandStatus::kTerminalFaultReported;
    } else {
      Alarm = "EL" + String(J1axisFault) + String(J2axisFault) + String(J3axisFault) + String(J4axisFault) + String(J5axisFault) + String(J6axisFault) + String(J7axisFault) + String(J8axisFault) + String(J9axisFault);
      delay(5);
      Serial.println(Alarm);
      Alarm = "0";
      return ar4_protocol::MotionCommandStatus::kTerminalFaultReported;
    }

    inData = "";  // Clear recieved buffer
                  ////////MOVE COMPLETE///////////
  }
  return ar4_protocol::MotionCommandStatus::kCompleted;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//COMMUNICATIONS
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



const int32_t MODBUS_PARSE_ERROR = -2;

int32_t modbusQuerry(String inData, int function) {
  int32_t result;
  int32_t response;
  int32_t response2;
  int slaveIdIndex = inData.indexOf('A');
  int MBaddressIndex = inData.indexOf('B');
  int MBvalIndex = inData.indexOf('C');
  const int markers[] = { slaveIdIndex, MBaddressIndex, MBvalIndex };
  const int begins[] = {
    slaveIdIndex + 1,
    MBaddressIndex + 1,
    MBvalIndex + 1,
  };
  const int ends[] = {
    MBaddressIndex,
    MBvalIndex,
    static_cast<int>(inData.length()),
  };
  int parsed[3];
  if (
    !ar4_protocol::marker_positions_are_ordered_from(
      inData.length(),
      markers,
      0
    )
    || !parse_int_spans(inData, begins, ends, parsed)
  ) {
    return MODBUS_PARSE_ERROR;
  }
  int SlaveID = parsed[0];
  int MBaddress = parsed[1];
  int MBval = parsed[2];
  ar4_protocol::ModbusOperation operation;
  if (function == 1) {
    operation = ar4_protocol::ModbusOperation::kReadCoil;
  } else if (function == 2) {
    operation = ar4_protocol::ModbusOperation::kReadDiscreteInput;
  } else if (function == 3) {
    operation = ar4_protocol::ModbusOperation::kReadHoldingRegisters;
  } else if (function == 4) {
    operation = ar4_protocol::ModbusOperation::kReadInputRegisters;
  } else if (function == 15) {
    operation = ar4_protocol::ModbusOperation::kWriteCoil;
  } else if (function == 6) {
    operation = ar4_protocol::ModbusOperation::kWriteRegister;
  } else {
    return MODBUS_PARSE_ERROR;
  }
  if (!ar4_protocol::validate_modbus_request(
      operation,
      SlaveID,
      MBaddress,
      MBval
  )) {
    return MODBUS_PARSE_ERROR;
  }
  node = ModbusMaster();
  node.begin(SlaveID, Serial8);

  if (function == 1) {
    result = node.readCoils(MBaddress, 1);
    if (result == node.ku8MBSuccess) {
      response = node.getResponseBuffer(0);
      return response;
    } else {
      response = -1;
      return response;
    }
  } else if (function == 2) {
    result = node.readDiscreteInputs(MBaddress, 1);
    if (result == node.ku8MBSuccess) {
      response = node.getResponseBuffer(0);
      return response;
    } else {
      response = -1;
      return response;
    }
  } else if (function == 3) {
    result = node.readHoldingRegisters(MBaddress, MBval);
    if (result == node.ku8MBSuccess) {
      response = node.getResponseBuffer(0);
      return response;
    } else {
      response = -1;
      return response;
    }
  } else if (function == 4) {
    result = node.readInputRegisters(MBaddress, MBval);
    if (result == node.ku8MBSuccess) {
      response = node.getResponseBuffer(0);
      return response;
    } else {
      response = -1;
      return response;
    }
  } else if (function == 15) {
    result = node.writeSingleCoil(MBaddress, MBval);
    if (result == node.ku8MBSuccess) {
      response = 1;
      return response;
    } else {
      response = -1;
      return response;
    }
  } else if (function == 6) {
    result = node.writeSingleRegister(MBaddress, MBval);
    if (result == node.ku8MBSuccess) {
      response = 1;
      return response;
    } else {
      response = -1;
      return response;
    }
  } else {
    response = -1;
    return response;
  }
}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//READ DATA
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


ar4_protocol::SerialFrameReadStatus read_serial_frame_byte(String &frame) {
  if (Serial.available() <= 0) {
    return ar4_protocol::SerialFrameReadStatus::kPending;
  }
  return ar4_protocol::append_serial_frame_byte(
    frame,
    serialFrameDiscarding,
    Serial.read()
  );
}


void processSerial() {
  if (Serial.available() > 0 and cmdBuffer3 == "") {
    const ar4_protocol::SerialFrameReadStatus frameStatus =
      read_serial_frame_byte(recData);
    if (frameStatus == ar4_protocol::SerialFrameReadStatus::kOverflow) {
      Serial.println("ER");
      return;
    }
    // Process message when new line character is recieved
    if (frameStatus == ar4_protocol::SerialFrameReadStatus::kComplete) {
      //place data in last position
      cmdBuffer3 = recData;
      //determine if move command
      String commandPayload;
      String procCMDtype = "";
      if (ar4_protocol::extract_serial_command_payload(
          recData,
          commandPayload
      )) {
        procCMDtype = commandPayload.substring(0, 2);
      }
      if (procCMDtype == "SS") {
        splineTrue = false;
        splineEndReceived = true;
      }
      if (ar4_protocol::should_emit_spline_preface(
          splineTrue,
          procCMDtype.c_str()
      )) {
        if (moveSequence == "") {
          moveSequence = "firsMoveActive";
        }
        //close serial so next command can be read in
        if (Alarm == "0") {
          sendRobotPosSpline();
        } else {
          Serial.println(Alarm);
          Alarm = "0";
        }
      }

      recData = "";  // Clear recieved buffer

      shiftCMDarray();


      //if second position is empty and first move command read in process second move ahead of time
      if (procCMDtype == "MS" and moveSequence == "firsMoveActive" and cmdBuffer2 == "" and cmdBuffer1 != "" and splineTrue == true) {
        moveSequence = "secondMoveProcessed";
        while (cmdBuffer2 == "") {
          if (Serial.available() > 0) {
            const ar4_protocol::SerialFrameReadStatus lookaheadStatus =
              read_serial_frame_byte(recData);
            if (
              lookaheadStatus
              == ar4_protocol::SerialFrameReadStatus::kOverflow
            ) {
              Serial.println("ER");
              return;
            }
            if (
              lookaheadStatus
              == ar4_protocol::SerialFrameReadStatus::kComplete
            ) {
              cmdBuffer2 = recData;
              commandPayload = "";
              procCMDtype = "";
              if (ar4_protocol::extract_serial_command_payload(
                  recData,
                  commandPayload
              )) {
                procCMDtype = commandPayload.substring(0, 2);
              }
              if (procCMDtype == "MS") {
                //close serial so next command can be read in
                delay(5);
                if (Alarm == "0") {
                  sendRobotPosSpline();
                } else {
                  Serial.println(Alarm);
                  Alarm = "0";
                }
              }
              recData = "";  // Clear recieved buffer
            }
          }
        }
      }
    }
  }
}


void shiftCMDarray() {
  if (cmdBuffer1 == "") {
    //shift 2 to 1
    cmdBuffer1 = cmdBuffer2;
    cmdBuffer2 = "";
  }
  if (cmdBuffer2 == "") {
    //shift 3 to 2
    cmdBuffer2 = cmdBuffer3;
    cmdBuffer3 = "";
  }
  if (cmdBuffer1 == "") {
    //shift 2 to 1
    cmdBuffer1 = cmdBuffer2;
    cmdBuffer2 = "";
  }
}


void consume_current_command() {
  ar4_protocol::consume_command_queue(
    inData,
    cmdBuffer1,
    cmdBuffer2,
    cmdBuffer3
  );
}


ar4_protocol::LiveControlFrameStatus read_live_control_frame() {
  const ar4_protocol::SerialFrameReadStatus status =
    read_serial_frame_byte(inData);
  return ar4_protocol::classify_live_control_frame(
    status,
    inData.c_str(),
    inData.length()
  );
}


void send_live_terminal_response(
  ar4_protocol::LiveControlFrameStatus control_status,
  int kinematic_error,
  int axis_fault,
  const String &axis_limit_response
) {
  switch (ar4_protocol::select_live_terminal_response(
      control_status,
      kinematic_error,
      axis_fault
  )) {
    case ar4_protocol::LiveTerminalResponseKind::kPosition:
      sendRobotPos();
      return;
    case ar4_protocol::LiveTerminalResponseKind::kError:
      delay(5);
      Serial.println("ER");
      return;
    case ar4_protocol::LiveTerminalResponseKind::kAxisLimit:
      delay(5);
      Serial.println(axis_limit_response);
      return;
  }
}


void EstopProg() {
  estopActive = true;
  flag = "EB";
  sendRobotPos();
}



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
    Serial.println("EEPROM legacy persistence migration failed");
    identity_record_status = ar4_protocol::IdentityRecordStatus::kCorrupt;
    clear_robot_identity();
  } else {
    load_debug_from_eeprom();
    load_robot_id_from_eeprom();
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

  //clear command buffer array
  cmdBuffer1 = "";
  cmdBuffer2 = "";
  cmdBuffer3 = "";
  //reset move command flag
  moveSequence = "";
  flag = "";
  rndTrue = false;
  splineTrue = false;
  splineEndReceived = false;
}

void loop() {

  ////////////////////////////////////
  ///////////start loop///////////////

  if (splineEndReceived == false) {
    processSerial();
  }
  //dont start unless at least one command has been read in
  if (cmdBuffer1 != "") {
    //process data
    estopActive = false;
    if (!ar4_protocol::extract_serial_command_payload(cmdBuffer1, inData)) {
      Serial.println("ER");
      consume_current_command();
      return;
    }
    String function = inData.substring(0, 2);
    inData = inData.substring(2);
    KinematicError = 0;
    debug = "";

    if (function == "HO") {
      handle_hello_command();
    }

    if (function == "RB") {
      Serial.println("System Restarting");
      reboot();
    }

    if (function == "DB") {
      String help = "Command DB - Set Debug Parameters\n";
      help += "required [D] - Debug State 0/1 (off/on) Enables / Disabled Serial Debug Mode\n";
      help += "optional [P] - Persistence 0/1 Disable / Enable debug mode persist accross reboots\n\n";
      help += "Example: DB[D]1[P]1 - Enabled Debug mode with persist\n";
      help += "Example: DB[D]0 - Disable debug mode, don't change current persisted value\n\n";

      ar4_protocol::DebugCommand debugCommand = {false, false, false};
      const ar4_protocol::DebugCommandStatus debugStatus =
        ar4_protocol::parse_debug_command(inData.c_str(), debugCommand);
      if (
        debugStatus == ar4_protocol::DebugCommandStatus::kMissingDebugField
        || debugStatus == ar4_protocol::DebugCommandStatus::kInvalidFormat
      ) {
        if (debugStatus == ar4_protocol::DebugCommandStatus::kInvalidFormat) {
          Serial.println("Invalid DB command format");
        }
        Serial.println(help);
        consume_current_command();
        return;
      }
      if (debugStatus == ar4_protocol::DebugCommandStatus::kInvalidDebugValue) {
        Serial.println("Valid values for debug are 0 and 1\n");
        Serial.println(help);
        consume_current_command();
        return;
      }
      if (
        debugStatus
        == ar4_protocol::DebugCommandStatus::kInvalidPersistenceValue
      ) {
        Serial.println("Valid values for persist are 0 and 1\n");
        Serial.println(help);
        consume_current_command();
        return;
      }
      if (!ar4_protocol::apply_debug_command(
        debugCommand,
        DEBUG,
        save_debug_to_eeprom
      )) {
        consume_current_command();
        return;
      }

      Serial.println("Done");
      consume_current_command();
      return;
    }

    if (function == "SR") {
      ar4_protocol::IdentitySetCommandFields commandFields = {};
      if (!ar4_protocol::parse_identity_set_command(
          inData.c_str(),
          commandFields
      )) {
        Serial.println("Error: Invalid format (SR)");
        consume_current_command();
        return;
      }

      handle_set_robot_id_command(
        String(commandFields.robot_model),
        String(commandFields.robot_version),
        String(commandFields.driver_board),
        String(commandFields.serial_number),
        String(commandFields.asset_tag)
      );
    }

    //-----MODBUS READ HOLDING REGISTER - FUNCTION 03--------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "BA") {
      int32_t result = modbusQuerry(inData, 3);
      if (result == MODBUS_PARSE_ERROR) {
        Serial.println("ER");
      } else if (result == -1) {
        Serial.println("Modbus Error");
      } else {
        Serial.println(result);
      }
    }

    //-----MODBUS READ COIL - FUNCTION 01--------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "BB") {
      int32_t result = modbusQuerry(inData, 1);
      if (result == MODBUS_PARSE_ERROR) {
        Serial.println("ER");
      } else if (result == -1) {
        Serial.println("Modbus Error");
      } else {
        Serial.println(result);
      }
    }

    //-----MODBUS READ INPUT - FUNCTION 02--------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "BC") {
      int32_t result = modbusQuerry(inData, 2);
      if (result == MODBUS_PARSE_ERROR) {
        Serial.println("ER");
      } else if (result == -1) {
        Serial.println("Modbus Error");
      } else {
        Serial.println(result);
      }
    }

    //-----MODBUS READ HOLDING REGISTER - FUNCTION 03--------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "BH") {
      int32_t result = modbusQuerry(inData, 3);
      if (result == MODBUS_PARSE_ERROR) {
        Serial.println("ER");
      } else if (result == -1) {
        Serial.println("Modbus Error");
      } else {
        Serial.println(result);
      }
    }

    //-----MODBUS READ INPUT REGISTER - FUNCTION 04--------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "BD") {
      int32_t result = modbusQuerry(inData, 4);
      if (result == MODBUS_PARSE_ERROR) {
        Serial.println("ER");
      } else if (result == -1) {
        Serial.println("Modbus Error");
      } else {
        Serial.println(result);
      }
    }

    //-----MODBUS WRITE COIL - FUNCTION 15--------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "BE") {
      int32_t result = modbusQuerry(inData, 15);
      if (result == MODBUS_PARSE_ERROR) {
        Serial.println("ER");
      } else if (result == -1) {
        Serial.println("Modbus Error");
      } else {
        Serial.println("Write Success");
      }
    }

    //-----MODBUS WRITE REGISTER - FUNCTION 6--------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "BF") {
      int32_t result = modbusQuerry(inData, 6);
      if (result == MODBUS_PARSE_ERROR) {
        Serial.println("ER");
      } else if (result == -1) {
        Serial.println("Modbus Error");
      } else {
        Serial.println("Write Success");
      }
    }

    //-----QUERRY DRIVE MODBUS--------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "MQ") {
      uint8_t result;
      int16_t highRegister;

      // Modbus read
      result = node.readHoldingRegisters(0x1207, 2);

      if (result == node.ku8MBSuccess) {

        highRegister = node.getResponseBuffer(0);
        Serial.println(highRegister);

      } else {
        Serial.println("Modbus error: ");
        //Serial.println(result, HEX);
      }

      delay(1000);
    }

    //-----HOME MOTOR DRIVE MODBUS--------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "HD") {
      uint8_t result;

      // Address and value to write
      uint16_t registerAddress1 = 0x020D;  // P0213 - DI3
      uint16_t registerAddress2 = 0x020C;  // P0212 - DI2
      //uint16_t registerAddress = 0x1207;  // P1807 - absolute position counter
      uint16_t valueOn = 1;   // Value to write to the register
      uint16_t valueOff = 0;  // Value to write to the register

      // Write the value to the register
      result = node.writeSingleRegister(registerAddress1, valueOn);
      delay(50);
      result = node.writeSingleRegister(registerAddress2, valueOn);
      delay(50);
      result = node.writeSingleRegister(registerAddress1, valueOff);
      delay(50);
      result = node.writeSingleRegister(registerAddress2, valueOff);

      if (result == node.ku8MBSuccess) {
        Serial.println("Write successful");
      } else {
        //Serial.println("Modbus Error: ");
        Serial.println(result, HEX);
      }

      delay(50);
    }

    //-----RESET DRIVE MODBUS--------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "RR") {
      uint8_t result;

      // Address and value to write
      uint16_t registerAddress1 = 0x020D;  // P0213 - DI3 INPUT
      uint16_t registerAddress2 = 0x0203;  // P0203 - DI3 FUNCTION SELECTION

      uint16_t valueOn = 1;
      uint16_t valueOff = 0;
      uint16_t homingMode = 33;
      uint16_t resetMode = 2;


      result = node.writeSingleRegister(registerAddress2, resetMode);
      delay(50);
      result = node.writeSingleRegister(registerAddress1, valueOn);
      delay(50);
      result = node.writeSingleRegister(registerAddress2, homingMode);
      delay(50);


      if (result == node.ku8MBSuccess) {
        Serial.println("Write successful");
      } else {
        Serial.println("fail");
      }

      delay(50);
    }

    //-----RESET DRIVE MODBUS--------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "FR") {
      uint8_t result;

      // Address and value to write
      uint16_t registerAddress1 = 0x0B01;  // P1101 - fault reset

      uint16_t valueOn = 1;
      uint16_t valueOff = 0;

      result = node.writeSingleRegister(registerAddress1, valueOn);
      delay(50);
      result = node.writeSingleRegister(registerAddress1, valueOff);
      delay(50);


      if (result == node.ku8MBSuccess) {
        Serial.println("Write successful");
      } else {
        Serial.println("fail");
      }

      delay(50);
    }

    //-----SPLINE START------------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "SL") {
      splineTrue = true;
      delay(5);
      Serial.print("SL");
      moveSequence = "";
      flag = "";
      rndTrue = false;
      splineEndReceived = false;
    }

    //----- SPLINE STOP  ----------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "SS") {
      delay(5);
      sendRobotPos();
      splineTrue = false;
      splineEndReceived = false;
    }

    //-----COMMAND TO CLOSE---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "CL") {
      delay(5);
      Serial.end();
    }

    //-----COMMAND TEST LIMIT SWITCHES---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "TL") {

      String J1calTest = "0";
      String J2calTest = "0";
      String J3calTest = "0";
      String J4calTest = "0";
      String J5calTest = "0";
      String J6calTest = "0";

      if (digitalRead(J1calPin) == HIGH) {
        J1calTest = "1";
      }
      if (digitalRead(J2calPin) == HIGH) {
        J2calTest = "1";
      }
      if (digitalRead(J3calPin) == HIGH) {
        J3calTest = "1";
      }
      if (digitalRead(J4calPin) == HIGH) {
        J4calTest = "1";
      }
      if (digitalRead(J5calPin) == HIGH) {
        J5calTest = "1";
      }
      if (digitalRead(J6calPin) == HIGH) {
        J6calTest = "1";
      }
      String TestLim = " J1 = " + J1calTest + "   J2 = " + J2calTest + "   J3 = " + J3calTest + "   J4 = " + J4calTest + "   J5 = " + J5calTest + "   J6 = " + J6calTest;
      delay(5);
      Serial.println(TestLim);
    }


    //-----COMMAND SET ENCODERS TO 1000---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "SE") {
      J1encPos.write(1000);
      J2encPos.write(1000);
      J3encPos.write(1000);
      J4encPos.write(1000);
      J5encPos.write(1000);
      J6encPos.write(1000);
      delay(5);
      Serial.print("Done");
    }

    //-----COMMAND READ ENCODERS---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "RE") {
      J1EncSteps = J1encPos.read();
      J2EncSteps = J2encPos.read();
      J3EncSteps = J3encPos.read();
      J4EncSteps = J4encPos.read();
      J5EncSteps = J5encPos.read();
      J6EncSteps = J6encPos.read();
      String Read = " J1 = " + String(J1EncSteps) + "   J2 = " + String(J2EncSteps) + "   J3 = " + String(J3EncSteps) + "   J4 = " + String(J4EncSteps) + "   J5 = " + String(J5EncSteps) + "   J6 = " + String(J6EncSteps);
      delay(5);
      Serial.println(Read);
    }

    //-----COMMAND REQUEST POSITION---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "RP") {
      //close serial so next command can be read in
      delay(5);
      if (Alarm == "0") {
        sendRobotPos();
      } else {
        Serial.println(Alarm);
        Alarm = "0";
      }
    }



    //-----COMMAND HOME POSITION---------------------------------------------------
    //-----------------------------------------------------------------------

    //For debugging
    if (function == "HM") {

      int J1dir;
      int J2dir;
      int J3dir;
      int J4dir;
      int J5dir;
      int J6dir;
      int J7dir;
      int J8dir;
      int J9dir;


      String SpeedType = "p";
      float SpeedVal = 25.0;
      float ACCspd = 10.0;
      float DCCspd = 10.0;
      float ACCramp = 20.0;

      const float home_positions[ROBOT_nDOFs] = {
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
      };
      int home_steps[ROBOT_nDOFs] = {};
      if (!primary_positions_to_future_steps(home_positions, home_steps)) {
        Serial.println("ER");
        consume_current_command();
        return;
      }
      for (int axis = 0; axis < ROBOT_nDOFs; ++axis) {
        JangleIn[axis] = home_positions[axis];
      }


      //calc destination motor steps
      int J1futStepM = home_steps[0];
      int J2futStepM = home_steps[1];
      int J3futStepM = home_steps[2];
      int J4futStepM = home_steps[3];
      int J5futStepM = home_steps[4];
      int J6futStepM = home_steps[5];

      //calc delta from current to destination
      int J1stepDif = J1StepM - J1futStepM;
      int J2stepDif = J2StepM - J2futStepM;
      int J3stepDif = J3StepM - J3futStepM;
      int J4stepDif = J4StepM - J4futStepM;
      int J5stepDif = J5StepM - J5futStepM;
      int J6stepDif = J6StepM - J6futStepM;
      int J7stepDif = 0;
      int J8stepDif = 0;
      int J9stepDif = 0;

      //determine motor directions
      J1dir = (J1stepDif <= 0) ? 1 : 0;
      J2dir = (J2stepDif <= 0) ? 1 : 0;
      J3dir = (J3stepDif <= 0) ? 1 : 0;
      J4dir = (J4stepDif <= 0) ? 1 : 0;
      J5dir = (J5stepDif <= 0) ? 1 : 0;
      J6dir = (J6stepDif <= 0) ? 1 : 0;
      J7dir = 0;
      J8dir = 0;
      J9dir = 0;



      resetEncoders();
      if (!driveMotorsJ(abs(J1stepDif), abs(J2stepDif), abs(J3stepDif), abs(J4stepDif), abs(J5stepDif), abs(J6stepDif), abs(J7stepDif), abs(J8stepDif), abs(J9stepDif), J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, J7dir, J8dir, J9dir, SpeedType, SpeedVal, ACCspd, DCCspd, ACCramp, nullptr)) {
        Serial.println("ER");
        consume_current_command();
        return;
      }
      checkEncoders();
      sendRobotPos();
      delay(5);
      Serial.println("Done");
    }


    //-----COMMAND CORRECT POSITION---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "CP") {
      correctRobotPos();
    }

    //-----COMMAND UPDATE PARAMS---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "UP") {
      int TFxStart = inData.indexOf('A');
      int TFyStart = inData.indexOf('B');
      int TFzStart = inData.indexOf('C');
      int TFrzStart = inData.indexOf('D');
      int TFryStart = inData.indexOf('E');
      int TFrxStart = inData.indexOf('F');

      int J1motDirStart = inData.indexOf('G');
      int J2motDirStart = inData.indexOf('H');
      int J3motDirStart = inData.indexOf('I');
      int J4motDirStart = inData.indexOf('J');
      int J5motDirStart = inData.indexOf('K');
      int J6motDirStart = inData.indexOf('L');
      int J7motDirStart = inData.indexOf('M');
      int J8motDirStart = inData.indexOf('N');
      int J9motDirStart = inData.indexOf('O');

      int J1calDirStart = inData.indexOf('P');
      int J2calDirStart = inData.indexOf('Q');
      int J3calDirStart = inData.indexOf('R');
      int J4calDirStart = inData.indexOf('S');
      int J5calDirStart = inData.indexOf('T');
      int J6calDirStart = inData.indexOf('U');
      int J7calDirStart = inData.indexOf('V');
      int J8calDirStart = inData.indexOf('W');
      int J9calDirStart = inData.indexOf('X');

      int J1PosLimStart = inData.indexOf('Y');
      int J1NegLimStart = inData.indexOf('Z');
      int J2PosLimStart = inData.indexOf('a');
      int J2NegLimStart = inData.indexOf('b');
      int J3PosLimStart = inData.indexOf('c');
      int J3NegLimStart = inData.indexOf('d');
      int J4PosLimStart = inData.indexOf('e');
      int J4NegLimStart = inData.indexOf('f');
      int J5PosLimStart = inData.indexOf('g');
      int J5NegLimStart = inData.indexOf('h');
      int J6PosLimStart = inData.indexOf('i');
      int J6NegLimStart = inData.indexOf('j');

      int J1StepDegStart = inData.indexOf('k');
      int J2StepDegStart = inData.indexOf('l');
      int J3StepDegStart = inData.indexOf('m');
      int J4StepDegStart = inData.indexOf('n');
      int J5StepDegStart = inData.indexOf('o');
      int J6StepDegStart = inData.indexOf('p');

      int J1EncMultStart = inData.indexOf('q');
      int J2EncMultStart = inData.indexOf('r');
      int J3EncMultStart = inData.indexOf('s');
      int J4EncMultStart = inData.indexOf('t');
      int J5EncMultStart = inData.indexOf('u');
      int J6EncMultStart = inData.indexOf('v');

      int J1tDHparStart = inData.indexOf('w');
      int J2tDHparStart = inData.indexOf('x');
      int J3tDHparStart = inData.indexOf('y');
      int J4tDHparStart = inData.indexOf('z');
      int J5tDHparStart = inData.indexOf('!');
      int J6tDHparStart = inData.indexOf('@');

      int J1uDHparStart = inData.indexOf('#');
      int J2uDHparStart = inData.indexOf('$');
      int J3uDHparStart = inData.indexOf('%');
      int J4uDHparStart = inData.indexOf('^');
      int J5uDHparStart = inData.indexOf('&');
      int J6uDHparStart = inData.indexOf('*');

      int J1dDHparStart = inData.indexOf('(');
      int J2dDHparStart = inData.indexOf(')');
      int J3dDHparStart = inData.indexOf('+');
      int J4dDHparStart = inData.indexOf('=');
      int J5dDHparStart = inData.indexOf(',');
      int J6dDHparStart = inData.indexOf('_');

      int J1aDHparStart = inData.indexOf('<');
      int J2aDHparStart = inData.indexOf('>');
      int J3aDHparStart = inData.indexOf('?');
      int J4aDHparStart = inData.indexOf('{');
      int J5aDHparStart = inData.indexOf('}');
      int J6aDHparStart = inData.indexOf('~');

      const int positions[] = {
        TFxStart,
        TFyStart,
        TFzStart,
        TFrzStart,
        TFryStart,
        TFrxStart,
        J1motDirStart,
        J2motDirStart,
        J3motDirStart,
        J4motDirStart,
        J5motDirStart,
        J6motDirStart,
        J7motDirStart,
        J8motDirStart,
        J9motDirStart,
        J1calDirStart,
        J2calDirStart,
        J3calDirStart,
        J4calDirStart,
        J5calDirStart,
        J6calDirStart,
        J7calDirStart,
        J8calDirStart,
        J9calDirStart,
        J1PosLimStart,
        J1NegLimStart,
        J2PosLimStart,
        J2NegLimStart,
        J3PosLimStart,
        J3NegLimStart,
        J4PosLimStart,
        J4NegLimStart,
        J5PosLimStart,
        J5NegLimStart,
        J6PosLimStart,
        J6NegLimStart,
        J1StepDegStart,
        J2StepDegStart,
        J3StepDegStart,
        J4StepDegStart,
        J5StepDegStart,
        J6StepDegStart,
        J1EncMultStart,
        J2EncMultStart,
        J3EncMultStart,
        J4EncMultStart,
        J5EncMultStart,
        J6EncMultStart,
        J1tDHparStart,
        J2tDHparStart,
        J3tDHparStart,
        J4tDHparStart,
        J5tDHparStart,
        J6tDHparStart,
        J1uDHparStart,
        J2uDHparStart,
        J3uDHparStart,
        J4uDHparStart,
        J5uDHparStart,
        J6uDHparStart,
        J1dDHparStart,
        J2dDHparStart,
        J3dDHparStart,
        J4dDHparStart,
        J5dDHparStart,
        J6dDHparStart,
        J1aDHparStart,
        J2aDHparStart,
        J3aDHparStart,
        J4aDHparStart,
        J5aDHparStart,
        J6aDHparStart,
        static_cast<int>(inData.length()),
      };
      float stagedTool[ROBOT_nDOFs];
      int stagedDirections[18];
      float stagedLimits[12];
      float stagedStepDegrees[ROBOT_nDOFs];
      float stagedEncoderMultipliers[ROBOT_nDOFs];
      float stagedDHTheta[ROBOT_nDOFs];
      float stagedDHAlpha[ROBOT_nDOFs];
      float stagedDHD[ROBOT_nDOFs];
      float stagedDHA[ROBOT_nDOFs];
      if (
        !ar4_protocol::field_boundaries_cover_command(
          inData.length(),
          positions
        )
        || !parse_float_marker_fields(inData, positions, stagedTool)
        || !parse_int_marker_fields(inData, positions + 6, stagedDirections)
        || !ar4_protocol::values_are_binary(stagedDirections)
        || !parse_float_marker_fields(inData, positions + 24, stagedLimits)
        || !parse_float_marker_fields(inData, positions + 36, stagedStepDegrees)
        || !parse_float_marker_fields(
          inData,
          positions + 42,
          stagedEncoderMultipliers
        )
        || !parse_float_marker_fields(inData, positions + 48, stagedDHTheta)
        || !parse_float_marker_fields(inData, positions + 54, stagedDHAlpha)
        || !parse_float_marker_fields(inData, positions + 60, stagedDHD)
        || !parse_float_marker_fields(inData, positions + 66, stagedDHA)
      ) {
        Serial.println("ER");
        consume_current_command();
        return;
      }

      int stagedStepLimits[ROBOT_nDOFs];
      int stagedZeroSteps[ROBOT_nDOFs];
      for (int joint = 0; joint < ROBOT_nDOFs; ++joint) {
        if (
          stagedEncoderMultipliers[joint] <= 0.0f
          || !ar4_protocol::validate_axis_calibration(
            stagedLimits[joint * 2 + 1],
            stagedLimits[joint * 2],
            stagedStepDegrees[joint],
            stagedStepLimits[joint],
            stagedZeroSteps[joint]
          )
        ) {
          Serial.println("ER");
          consume_current_command();
          return;
        }
      }

      float nativeToolRz = 0.0f;
      float nativeToolRy = 0.0f;
      float nativeToolRx = 0.0f;
      if (
        !ar4_protocol::degrees_to_radians(
          stagedTool[3],
          nativeToolRz
        )
        || !ar4_protocol::degrees_to_radians(
          stagedTool[4],
          nativeToolRy
        )
        || !ar4_protocol::degrees_to_radians(
          stagedTool[5],
          nativeToolRx
        )
      ) {
        Serial.println("ER");
        consume_current_command();
        return;
      }
      for (int joint = 0; joint < ROBOT_nDOFs; ++joint) {
        float nativeTheta = 0.0f;
        float nativeAlpha = 0.0f;
        if (
          !ar4_protocol::degrees_to_radians(
            stagedDHTheta[joint],
            nativeTheta
          )
          || !ar4_protocol::degrees_to_radians(
            stagedDHAlpha[joint],
            nativeAlpha
          )
        ) {
          Serial.println("ER");
          consume_current_command();
          return;
        }
      }
      Robot_Kin_Tool[0] = stagedTool[0];
      Robot_Kin_Tool[1] = stagedTool[1];
      Robot_Kin_Tool[2] = stagedTool[2];
      Robot_Kin_Tool[5] = nativeToolRz;
      Robot_Kin_Tool[4] = nativeToolRy;
      Robot_Kin_Tool[3] = nativeToolRx;
      J1MotDir = stagedDirections[0];
      J2MotDir = stagedDirections[1];
      J3MotDir = stagedDirections[2];
      J4MotDir = stagedDirections[3];
      J5MotDir = stagedDirections[4];
      J6MotDir = stagedDirections[5];
      J7MotDir = stagedDirections[6];
      J8MotDir = stagedDirections[7];
      J9MotDir = stagedDirections[8];
      J1CalDir = stagedDirections[9];
      J2CalDir = stagedDirections[10];
      J3CalDir = stagedDirections[11];
      J4CalDir = stagedDirections[12];
      J5CalDir = stagedDirections[13];
      J6CalDir = stagedDirections[14];
      J7CalDir = stagedDirections[15];
      J8CalDir = stagedDirections[16];
      J9CalDir = stagedDirections[17];
      J1axisLimPos = stagedLimits[0];
      J1axisLimNeg = stagedLimits[1];
      J2axisLimPos = stagedLimits[2];
      J2axisLimNeg = stagedLimits[3];
      J3axisLimPos = stagedLimits[4];
      J3axisLimNeg = stagedLimits[5];
      J4axisLimPos = stagedLimits[6];
      J4axisLimNeg = stagedLimits[7];
      J5axisLimPos = stagedLimits[8];
      J5axisLimNeg = stagedLimits[9];
      J6axisLimPos = stagedLimits[10];
      J6axisLimNeg = stagedLimits[11];

      J1StepDeg = stagedStepDegrees[0];
      J2StepDeg = stagedStepDegrees[1];
      J3StepDeg = stagedStepDegrees[2];
      J4StepDeg = stagedStepDegrees[3];
      J5StepDeg = stagedStepDegrees[4];
      J6StepDeg = stagedStepDegrees[5];

      J1encMult = stagedEncoderMultipliers[0];
      J2encMult = stagedEncoderMultipliers[1];
      J3encMult = stagedEncoderMultipliers[2];
      J4encMult = stagedEncoderMultipliers[3];
      J5encMult = stagedEncoderMultipliers[4];
      J6encMult = stagedEncoderMultipliers[5];

      for (int joint = 0; joint < ROBOT_nDOFs; ++joint) {
        DHparams[joint][0] = stagedDHTheta[joint];
        DHparams[joint][1] = stagedDHAlpha[joint];
        DHparams[joint][2] = stagedDHD[joint];
        DHparams[joint][3] = stagedDHA[joint];
      }


      //define total axis travel
      J1axisLim = J1axisLimPos + J1axisLimNeg;
      J2axisLim = J2axisLimPos + J2axisLimNeg;
      J3axisLim = J3axisLimPos + J3axisLimNeg;
      J4axisLim = J4axisLimPos + J4axisLimNeg;
      J5axisLim = J5axisLimPos + J5axisLimNeg;
      J6axisLim = J6axisLimPos + J6axisLimNeg;

      //steps full movement of each axis
      J1StepLim = stagedStepLimits[0];
      J2StepLim = stagedStepLimits[1];
      J3StepLim = stagedStepLimits[2];
      J4StepLim = stagedStepLimits[3];
      J5StepLim = stagedStepLimits[4];
      J6StepLim = stagedStepLimits[5];

      //step and axis zero
      J1zeroStep = stagedZeroSteps[0];
      J2zeroStep = stagedZeroSteps[1];
      J3zeroStep = stagedZeroSteps[2];
      J4zeroStep = stagedZeroSteps[3];
      J5zeroStep = stagedZeroSteps[4];
      J6zeroStep = stagedZeroSteps[5];

      if (!robot_set_AR()) {
        Serial.println("ER");
        consume_current_command();
        return;
      }
      Serial.print("Done");
    }

    //-----COMMAND CALIBRATE EXTERNAL AXIS---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "CE") {
      int J7lengthStart = inData.indexOf('A');
      int J7rotStart = inData.indexOf('B');
      int J7stepsStart = inData.indexOf('C');
      int J8lengthStart = inData.indexOf('D');
      int J8rotStart = inData.indexOf('E');
      int J8stepsStart = inData.indexOf('F');
      int J9lengthStart = inData.indexOf('G');
      int J9rotStart = inData.indexOf('H');
      int J9stepsStart = inData.indexOf('I');

      const int positions[] = {
        J7lengthStart,
        J7rotStart,
        J7stepsStart,
        J8lengthStart,
        J8rotStart,
        J8stepsStart,
        J9lengthStart,
        J9rotStart,
        J9stepsStart,
        static_cast<int>(inData.length()),
      };
      float parsed[9];
      if (
        !ar4_protocol::field_boundaries_cover_command(
          inData.length(),
          positions
        )
        || !parse_float_marker_fields(inData, positions, parsed)
      ) {
        Serial.println("ER");
        consume_current_command();
        return;
      }

      ar4_protocol::ExternalAxisCalibration stagedCalibration[3];
      for (int axis = 0; axis < 3; ++axis) {
        if (!ar4_protocol::validate_external_axis_calibration(
            parsed[axis * 3],
            parsed[axis * 3 + 1],
            parsed[axis * 3 + 2],
            stagedCalibration[axis]
        )) {
          Serial.println("ER");
          consume_current_command();
          return;
        }
      }

      J7length = parsed[0];
      J7rot = parsed[1];
      J7steps = parsed[2];
      J8length = parsed[3];
      J8rot = parsed[4];
      J8steps = parsed[5];
      J9length = parsed[6];
      J9rot = parsed[7];
      J9steps = parsed[8];

      J7axisLimNeg = 0;
      J7axisLimPos = stagedCalibration[0].positive_limit;
      J7axisLim = J7axisLimPos + J7axisLimNeg;
      J7StepDeg = stagedCalibration[0].steps_per_unit;
      J7StepLim = stagedCalibration[0].step_limit;
      J7zeroStep = stagedCalibration[0].zero_step;

      J8axisLimNeg = 0;
      J8axisLimPos = stagedCalibration[1].positive_limit;
      J8axisLim = J8axisLimPos + J8axisLimNeg;
      J8StepDeg = stagedCalibration[1].steps_per_unit;
      J8StepLim = stagedCalibration[1].step_limit;
      J8zeroStep = stagedCalibration[1].zero_step;

      J9axisLimNeg = 0;
      J9axisLimPos = stagedCalibration[2].positive_limit;
      J9axisLim = J9axisLimPos + J9axisLimNeg;
      J9StepDeg = stagedCalibration[2].steps_per_unit;
      J9StepLim = stagedCalibration[2].step_limit;
      J9zeroStep = stagedCalibration[2].zero_step;

      delay(5);
      Serial.print("Done");
    }

    //-----COMMAND ZERO J7---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "Z7") {
      J7StepM = 0;
      sendRobotPos();
    }

    //-----COMMAND ZERO J8---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "Z8") {
      J8StepM = 0;
      sendRobotPos();
    }

    //-----COMMAND ZERO J9---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "Z9") {
      J9StepM = 0;
      sendRobotPos();
    }


    //-----COMMAND TO WAIT TIME---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "WT") {
      int WTstart = inData.indexOf('S');
      float WaitTime = 0.0f;
      uint32_t WaitTimeMS = 0;
      if (
        WTstart != 0
        || !parse_float_span(
          inData,
          WTstart + 1,
          static_cast<int>(inData.length()),
          WaitTime
        )
        || !ar4_protocol::wait_seconds_to_milliseconds(
          WaitTime,
          WaitTimeMS
        )
      ) {
        Serial.println("ER");
        consume_current_command();
        return;
      }
      delay(WaitTimeMS);
      Serial.println("WTdone");
    }


    // The Teensy GPIO map has no general-purpose output profile. ON/OF remain
    // auxiliary-controller commands until a board-specific map is defined.
    if (function == "ON" || function == "OF") {
      Serial.println("ER");
      consume_current_command();
      return;
    }

    //-----COMMAND TO WAIT MODBUS COIL---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "WJ") {
      int32_t result = -2;
      String MBquery = "";
      int slaveIndex = inData.indexOf('A');
      int inputIndex = inData.indexOf('B');
      int valueIndex = inData.indexOf('C');
      int timoutIndex = inData.indexOf('D');
      const int positions[] = {
        slaveIndex,
        inputIndex,
        valueIndex,
        timoutIndex,
        static_cast<int>(inData.length()),
      };
      int parsed[4];
      uint32_t timeoutMillis = 0;
      if (
        !ar4_protocol::field_boundaries_cover_command(
          inData.length(),
          positions
        )
        || !parse_int_marker_fields(inData, positions, parsed)
        || !ar4_protocol::validate_modbus_wait(
          ar4_protocol::ModbusOperation::kReadCoil,
          parsed[0],
          parsed[1],
          parsed[2],
          parsed[3],
          timeoutMillis
        )
      ) {
        Serial.println("ER");
        consume_current_command();
        return;
      }
      int slaveID = parsed[0];
      int input = parsed[1];
      int value = parsed[2];
      unsigned long startTime = millis();
      MBquery = "A" + String(slaveID) + "B" + String(input) + "C1";
      while ((millis() - startTime < timeoutMillis) && (result != value)) {
        result = modbusQuerry(MBquery, 1);
        delay(100);
      }
      delay(5);
      if (result == value) {
        Serial.println("Done");
      } else if (result == -1) {
        Serial.println("Modbus Error");
      } else {
        Serial.println("ER");
      }
    }

    //-----COMMAND TO WAIT MODBUS INPUT---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "WK") {
      int32_t result = -2;
      String MBquery = "";
      int slaveIndex = inData.indexOf('A');
      int inputIndex = inData.indexOf('B');
      int valueIndex = inData.indexOf('C');
      int timoutIndex = inData.indexOf('D');
      const int positions[] = {
        slaveIndex,
        inputIndex,
        valueIndex,
        timoutIndex,
        static_cast<int>(inData.length()),
      };
      int parsed[4];
      uint32_t timeoutMillis = 0;
      if (
        !ar4_protocol::field_boundaries_cover_command(
          inData.length(),
          positions
        )
        || !parse_int_marker_fields(inData, positions, parsed)
        || !ar4_protocol::validate_modbus_wait(
          ar4_protocol::ModbusOperation::kReadDiscreteInput,
          parsed[0],
          parsed[1],
          parsed[2],
          parsed[3],
          timeoutMillis
        )
      ) {
        Serial.println("ER");
        consume_current_command();
        return;
      }
      int slaveID = parsed[0];
      int input = parsed[1];
      int value = parsed[2];
      unsigned long startTime = millis();
      MBquery = "A" + String(slaveID) + "B" + String(input) + "C1";
      while ((millis() - startTime < timeoutMillis) && (result != value)) {
        result = modbusQuerry(MBquery, 2);
        delay(100);
      }
      delay(5);
      if (result == value) {
        Serial.println("Done");
      } else if (result == -1) {
        Serial.println("Modbus Error");
      } else {
        Serial.println("ER");
      }
    }

    //-----COMMAND TO SET MODBUS COIL---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "SC") {
      int32_t result = -2;
      String MBquery = "";
      int slaveIndex = inData.indexOf('A');
      int inputIndex = inData.indexOf('B');
      int valueIndex = inData.indexOf('C');
      const int positions[] = {
        slaveIndex,
        inputIndex,
        valueIndex,
        static_cast<int>(inData.length()),
      };
      int parsed[3];
      if (
        !ar4_protocol::field_boundaries_cover_command(
          inData.length(),
          positions
        )
        || !parse_int_marker_fields(inData, positions, parsed)
        || !ar4_protocol::validate_modbus_request(
          ar4_protocol::ModbusOperation::kWriteCoil,
          parsed[0],
          parsed[1],
          parsed[2]
        )
      ) {
        Serial.println("ER");
        consume_current_command();
        return;
      }
      int slaveID = parsed[0];
      int input = parsed[1];
      int value = parsed[2];
      MBquery = "A" + String(slaveID) + "B" + String(input) + "C" + String(value);
      result = modbusQuerry(MBquery, 15);
      delay(5);
      Serial.println(result);
    }

    //-----COMMAND TO SET MODBUS OUTPUT REGISTER---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "SO") {
      int32_t result = -2;
      String MBquery = "";
      int slaveIndex = inData.indexOf('A');
      int inputIndex = inData.indexOf('B');
      int valueIndex = inData.indexOf('C');
      const int positions[] = {
        slaveIndex,
        inputIndex,
        valueIndex,
        static_cast<int>(inData.length()),
      };
      int parsed[3];
      if (
        !ar4_protocol::field_boundaries_cover_command(
          inData.length(),
          positions
        )
        || !parse_int_marker_fields(inData, positions, parsed)
        || !ar4_protocol::validate_modbus_request(
          ar4_protocol::ModbusOperation::kWriteRegister,
          parsed[0],
          parsed[1],
          parsed[2]
        )
      ) {
        Serial.println("ER");
        consume_current_command();
        return;
      }
      int slaveID = parsed[0];
      int input = parsed[1];
      int value = parsed[2];
      MBquery = "A" + String(slaveID) + "B" + String(input) + "C" + String(value);
      result = modbusQuerry(MBquery, 6);
      delay(5);
      Serial.println(result);
    }


    //-----COMMAND SEND POSITION---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "SP") {
      int J1angStart = inData.indexOf('A');
      int J2angStart = inData.indexOf('B');
      int J3angStart = inData.indexOf('C');
      int J4angStart = inData.indexOf('D');
      int J5angStart = inData.indexOf('E');
      int J6angStart = inData.indexOf('F');
      int J7angStart = inData.indexOf('G');
      int J8angStart = inData.indexOf('H');
      int J9angStart = inData.indexOf('I');
      const int positions[] = {
        J1angStart,
        J2angStart,
        J3angStart,
        J4angStart,
        J5angStart,
        J6angStart,
        J7angStart,
        J8angStart,
        J9angStart,
        static_cast<int>(inData.length()),
      };
      float parsed[9];
      int stagedStepMonitors[numJoints];
      if (
        !ar4_protocol::field_boundaries_cover_command(
          inData.length(),
          positions
        )
        || !parse_float_marker_fields(inData, positions, parsed)
        || !joint_positions_to_future_steps(parsed, stagedStepMonitors)
      ) {
        Serial.println("ER");
        consume_current_command();
        return;
      }
      J1StepM = stagedStepMonitors[0];
      J2StepM = stagedStepMonitors[1];
      J3StepM = stagedStepMonitors[2];
      J4StepM = stagedStepMonitors[3];
      J5StepM = stagedStepMonitors[4];
      J6StepM = stagedStepMonitors[5];
      J7StepM = stagedStepMonitors[6];
      J8StepM = stagedStepMonitors[7];
      J9StepM = stagedStepMonitors[8];
      delay(5);
      Serial.println("Done");
    }


    //-----COMMAND ECHO TEST MESSAGE---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "TM") {
      int J1start = inData.indexOf('A');
      int J2start = inData.indexOf('B');
      int J3start = inData.indexOf('C');
      int J4start = inData.indexOf('D');
      int J5start = inData.indexOf('E');
      int J6start = inData.indexOf('F');
      int WristConStart = inData.indexOf('W');
      const int positions[] = {
        J1start,
        J2start,
        J3start,
        J4start,
        J5start,
        J6start,
        WristConStart,
      };
      float parsed[ROBOT_nDOFs];
      if (
        !ar4_protocol::marker_positions_are_ordered_from(
          inData.length(),
          positions,
          0
        )
        || WristConStart + 2 != static_cast<int>(inData.length())
        || !ar4_protocol::valid_wrist_config(inData.charAt(WristConStart + 1))
        || !parse_float_marker_fields(inData, positions, parsed)
      ) {
        Serial.println("ER");
        consume_current_command();
        return;
      }
      for (int joint = 0; joint < ROBOT_nDOFs; ++joint) {
        JangleIn[joint] = parsed[joint];
      }
      WristCon = inData.substring(WristConStart + 1);

      SolveInverseKinematics(inData.charAt(WristConStart + 1));

      String echo = "";
      delay(5);
      Serial.println(inData);
    }


    //-----COMMAND TO CALIBRATE---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "LL") {
      int J1start = inData.indexOf('A');
      int J2start = inData.indexOf('B');
      int J3start = inData.indexOf('C');
      int J4start = inData.indexOf('D');
      int J5start = inData.indexOf('E');
      int J6start = inData.indexOf('F');
      int J7start = inData.indexOf('G');
      int J8start = inData.indexOf('H');
      int J9start = inData.indexOf('I');

      int J1calstart = inData.indexOf('J');
      int J2calstart = inData.indexOf('K');
      int J3calstart = inData.indexOf('L');
      int J4calstart = inData.indexOf('M');
      int J5calstart = inData.indexOf('N');
      int J6calstart = inData.indexOf('O');
      int J7calstart = inData.indexOf('P');
      int J8calstart = inData.indexOf('Q');
      int J9calstart = inData.indexOf('R');

      const int positions[] = {
        J1start,
        J2start,
        J3start,
        J4start,
        J5start,
        J6start,
        J7start,
        J8start,
        J9start,
        J1calstart,
        J2calstart,
        J3calstart,
        J4calstart,
        J5calstart,
        J6calstart,
        J7calstart,
        J8calstart,
        J9calstart,
        static_cast<int>(inData.length()),
      };
      int requested[9];
      float calibrationOffsets[9];
      if (
        !ar4_protocol::field_boundaries_cover_command(
          inData.length(),
          positions
        )
        || !parse_int_marker_fields(inData, positions, requested)
        || !ar4_protocol::values_are_binary(requested)
        || !parse_float_marker_fields(
          inData,
          positions + 9,
          calibrationOffsets
        )
      ) {
        Serial.println("ER");
        consume_current_command();
        return;
      }
      int J1req = requested[0];
      int J2req = requested[1];
      int J3req = requested[2];
      int J4req = requested[3];
      int J5req = requested[4];
      int J6req = requested[5];
      int J7req = requested[6];
      int J8req = requested[7];
      int J9req = requested[8];

      ///
      float SpeedIn;
      ///
      int J1stepCen = 0;
      int J2stepCen = 0;
      int J3stepCen = 0;
      int J4stepCen = 0;
      int J5step45 = 0;
      int J6stepCen = 0;
      int J7stepCen = 0;
      int J8stepCen = 0;
      int J9stepCen = 0;
      ///
      int J1dir;
      int J2dir;
      int J3dir;
      int J4dir;
      int J5dir;
      int J6dir;
      int J7dir;
      int J8dir;
      int J9dir;

      int Jreq[9] = { J1req, J2req, J3req, J4req, J5req, J6req, J7req, J8req, J9req };
      int JStepLim[9] = { J1StepLim, J2StepLim, J3StepLim, J4StepLim, J5StepLim, J6StepLim, J7StepLim, J8StepLim, J9StepLim };
      int JStep[9] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };

      const int calibrationDirections[9] = {
        J1CalDir, J2CalDir, J3CalDir, J4CalDir, J5CalDir,
        J6CalDir, J7CalDir, J8CalDir, J9CalDir,
      };
      const float positiveLimits[9] = {
        J1axisLimPos, J2axisLimPos, J3axisLimPos,
        J4axisLimPos, J5axisLimPos, J6axisLimPos,
        J7axisLimPos, J8axisLimPos, J9axisLimPos,
      };
      const float negativeLimits[9] = {
        J1axisLimNeg, J2axisLimNeg, J3axisLimNeg,
        J4axisLimNeg, J5axisLimNeg, J6axisLimNeg,
        J7axisLimNeg, J8axisLimNeg, J9axisLimNeg,
      };
      const float stepsPerUnit[9] = {
        J1StepDeg, J2StepDeg, J3StepDeg,
        J4StepDeg, J5StepDeg, J6StepDeg,
        J7StepDeg, J8StepDeg, J9StepDeg,
      };
      const float baseOffsets[9] = {
        J1calBaseOff, J2calBaseOff, J3calBaseOff,
        J4calBaseOff, J5calBaseOff, J6calBaseOff,
        J7calBaseOff, J8calBaseOff, J9calBaseOff,
      };
      int stagedMasterSteps[9];
      int stagedCenterSteps[9];
      int stagedJointFiveSteps[9];
      for (int axis = 0; axis < 9; ++axis) {
        if (!ar4_protocol::calibration_reference_steps(
            Jreq[axis],
            calibrationDirections[axis],
            positiveLimits[axis],
            negativeLimits[axis],
            stepsPerUnit[axis],
            JStepLim[axis],
            baseOffsets[axis],
            calibrationOffsets[axis],
            axis == 4,
            stagedMasterSteps[axis],
            stagedCenterSteps[axis],
            stagedJointFiveSteps[axis]
        )) {
          Serial.println("ER");
          consume_current_command();
          return;
        }
      }

      for (int i = 0; i < 9; i++) {
        if (Jreq[i] == 1) {
          JStep[i] = JStepLim[i];
        }
      }

      //DRIVE TO LIMITS FAST
      SpeedIn = 25;
      driveLimit(JStep, SpeedIn);

      //Backoff 
      backOff(J1req, J2req, J3req, J4req, J5req,
        J6req, J7req, J8req, J9req,
        5,      // speed
        700);   // steps

      //DRIVE TO LIMITS MED
      SpeedIn = 2;
      driveLimit(JStep, SpeedIn);






      int *stepMonitors[9] = {
        &J1StepM, &J2StepM, &J3StepM, &J4StepM, &J5StepM,
        &J6StepM, &J7StepM, &J8StepM, &J9StepM,
      };
      for (int axis = 0; axis < 9; ++axis) {
        if (Jreq[axis] == 1) *stepMonitors[axis] = stagedMasterSteps[axis];
      }
      J1stepCen = stagedCenterSteps[0];
      J2stepCen = stagedCenterSteps[1];
      J3stepCen = stagedCenterSteps[2];
      J4stepCen = stagedCenterSteps[3];
      J5step45 = stagedJointFiveSteps[4];
      J6stepCen = stagedCenterSteps[5];
      J7stepCen = stagedCenterSteps[6];
      J8stepCen = stagedCenterSteps[7];
      J9stepCen = stagedCenterSteps[8];


      //move to center
      /// J1 ///
      if (J1CalDir) {
        J1dir = 0;
      } else {
        J1dir = 1;
      }
      /// J2 ///
      if (J2CalDir) {
        J2dir = 0;
      } else {
        J2dir = 1;
      }
      /// J3 ///
      if (J3CalDir) {
        J3dir = 0;
      } else {
        J3dir = 1;
      }
      /// J4 ///
      if (J4CalDir) {
        J4dir = 0;
      } else {
        J4dir = 1;
      }
      /// J5 ///
      if (J5CalDir) {
        J5dir = 0;
      } else {
        J5dir = 1;
      }
      /// J6 ///
      if (J6CalDir) {
        J6dir = 0;
      } else {
        J6dir = 1;
      }
      /// J7 ///
      if (J7CalDir) {
        J7dir = 0;
      } else {
        J7dir = 1;
      }
      /// J8 ///
      if (J8CalDir) {
        J8dir = 0;
      } else {
        J8dir = 1;
      }
      /// J9 ///
      if (J9CalDir) {
        J9dir = 0;
      } else {
        J9dir = 1;
      }

      float ACCspd = 10;
      float DCCspd = 10;
      String SpeedType = "p";
      //float SpeedVal = 50;
      float SpeedVal = 100;
      float ACCramp = 50;

      if (!driveMotorsJ(J1stepCen, J2stepCen, J3stepCen, J4stepCen, J5step45, J6stepCen, J7stepCen, J8stepCen, J9stepCen, J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, J7dir, J8dir, J9dir, SpeedType, SpeedVal, ACCspd, DCCspd, ACCramp, nullptr)) {
        Serial.println("ER");
        consume_current_command();
        return;
      }
      sendRobotPos();
      inData = "";  // Clear recieved buffer
    }


    //----- LIVE CARTESIAN JOG  ---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "LC") {
      int J1dir;
      int J2dir;
      int J3dir;
      int J4dir;
      int J5dir;
      int J6dir;
      int J7dir;
      int J8dir;
      int J9dir;

      int J1axisFault = 0;
      int J2axisFault = 0;
      int J3axisFault = 0;
      int J4axisFault = 0;
      int J5axisFault = 0;
      int J6axisFault = 0;
      int TotalAxisFault = 0;
      ar4_protocol::LiveControlFrameStatus liveControlStatus =
        ar4_protocol::LiveControlFrameStatus::kPending;

      bool JogInPoc = true;
      Alarm = "0";


      ar4_protocol::LiveJogCommandFields commandFields = {};
      if (!ar4_protocol::parse_live_jog_command(
          inData,
          ar4_protocol::LiveJogCommandKind::kCartesian,
          commandFields
      )) {
        Serial.println("ER");
        consume_current_command();
        return;
      }

      int Vector = commandFields.vector;
      String SpeedType(commandFields.speed_mode);
      float SpeedVal = commandFields.speed;
      float ACCspd = commandFields.acceleration;
      float DCCspd = commandFields.deceleration;
      float ACCramp = commandFields.ramp;
      ar4_protocol::MotionModeTransaction<String, ROBOT_nDOFs> motionModes(
        WristCon,
        JointLoopModes,
        String(commandFields.wrist_config),
        commandFields.loop_modes
      );

      inData = "";  // Clear recieved buffer

      delay(5);
      Serial.println();
      updatePos();

      xyzuvw_In[0] = xyzuvw_Out[0];
      xyzuvw_In[1] = xyzuvw_Out[1];
      xyzuvw_In[2] = xyzuvw_Out[2];
      xyzuvw_In[3] = xyzuvw_Out[3];
      xyzuvw_In[4] = xyzuvw_Out[4];
      xyzuvw_In[5] = xyzuvw_Out[5];


      while (JogInPoc == true) {

        if (Vector == 10) {
          xyzuvw_In[0] = xyzuvw_Out[0] - JogStepInc;
        }
        if (Vector == 11) {
          xyzuvw_In[0] = xyzuvw_Out[0] + JogStepInc;
        }

        if (Vector == 20) {
          xyzuvw_In[1] = xyzuvw_Out[1] - JogStepInc;
        }
        if (Vector == 21) {
          xyzuvw_In[1] = xyzuvw_Out[1] + JogStepInc;
        }

        if (Vector == 30) {
          xyzuvw_In[2] = xyzuvw_Out[2] - JogStepInc;
        }
        if (Vector == 31) {
          xyzuvw_In[2] = xyzuvw_Out[2] + JogStepInc;
        }

        if (Vector == 40) {
          xyzuvw_In[3] = xyzuvw_Out[3] - JogStepInc;
        }
        if (Vector == 41) {
          xyzuvw_In[3] = xyzuvw_Out[3] + JogStepInc;
        }

        if (Vector == 50) {
          xyzuvw_In[4] = xyzuvw_Out[4] - JogStepInc;
        }
        if (Vector == 51) {
          xyzuvw_In[4] = xyzuvw_Out[4] + JogStepInc;
        }

        if (Vector == 60) {
          xyzuvw_In[5] = xyzuvw_Out[5] - JogStepInc;
        }
        if (Vector == 61) {
          xyzuvw_In[5] = xyzuvw_Out[5] + JogStepInc;
        }

        SolveInverseKinematics(commandFields.wrist_config);

        //calc destination motor steps
        int future_steps[ROBOT_nDOFs] = {};
        if (
          KinematicError != 0
          || !primary_inverse_solution_to_future_steps(future_steps)
        ) {
          KinematicError = 1;
          break;
        }
        int J1futStepM = future_steps[0];
        int J2futStepM = future_steps[1];
        int J3futStepM = future_steps[2];
        int J4futStepM = future_steps[3];
        int J5futStepM = future_steps[4];
        int J6futStepM = future_steps[5];

        //calc delta from current to destination
        int J1stepDif = J1StepM - J1futStepM;
        int J2stepDif = J2StepM - J2futStepM;
        int J3stepDif = J3StepM - J3futStepM;
        int J4stepDif = J4StepM - J4futStepM;
        int J5stepDif = J5StepM - J5futStepM;
        int J6stepDif = J6StepM - J6futStepM;
        int J7stepDif = 0;
        int J8stepDif = 0;
        int J9stepDif = 0;

        //determine motor directions
        J1dir = (J1stepDif <= 0) ? 1 : 0;
        J2dir = (J2stepDif <= 0) ? 1 : 0;
        J3dir = (J3stepDif <= 0) ? 1 : 0;
        J4dir = (J4stepDif <= 0) ? 1 : 0;
        J5dir = (J5stepDif <= 0) ? 1 : 0;
        J6dir = (J6stepDif <= 0) ? 1 : 0;
        J7dir = 0;
        J8dir = 0;
        J9dir = 0;


        //determine if requested position is within axis limits
        if (future_step_is_outside_limit(J1futStepM, J1StepLim)) {
          J1axisFault = 1;
        }
        if (future_step_is_outside_limit(J2futStepM, J2StepLim)) {
          J2axisFault = 1;
        }
        if (future_step_is_outside_limit(J3futStepM, J3StepLim)) {
          J3axisFault = 1;
        }
        if (future_step_is_outside_limit(J4futStepM, J4StepLim)) {
          J4axisFault = 1;
        }
        if (future_step_is_outside_limit(J5futStepM, J5StepLim)) {
          J5axisFault = 1;
        }
        if (future_step_is_outside_limit(J6futStepM, J6StepLim)) {
          J6axisFault = 1;
        }
        TotalAxisFault = J1axisFault + J2axisFault + J3axisFault + J4axisFault + J5axisFault + J6axisFault;


        if (TotalAxisFault == 0 && KinematicError == 0) {
          if (!driveMotorsJ(abs(J1stepDif), abs(J2stepDif), abs(J3stepDif), abs(J4stepDif), abs(J5stepDif), abs(J6stepDif), abs(J7stepDif), abs(J8stepDif), abs(J9stepDif), J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, J7dir, J8dir, J9dir, SpeedType, SpeedVal, ACCspd, DCCspd, ACCramp, &motionModes)) {
            KinematicError = 1;
            break;
          }
          updatePos();
        }

        //stop loop if any serial command is recieved - but the expected command is "S" to stop the loop.

        liveControlStatus = read_live_control_frame();
        if (
          liveControlStatus
          != ar4_protocol::LiveControlFrameStatus::kPending
        ) {
          break;
        }

        //end loop
      }

      TotalCollision = J1collisionTrue + J2collisionTrue + J3collisionTrue + J4collisionTrue + J5collisionTrue + J6collisionTrue;
      if (TotalCollision > 0) {
        flag = "EC" + String(J1collisionTrue) + String(J2collisionTrue) + String(J3collisionTrue) + String(J4collisionTrue) + String(J5collisionTrue) + String(J6collisionTrue);
      }

      send_live_terminal_response(
        liveControlStatus,
        KinematicError,
        TotalAxisFault,
        "EL" + String(J1axisFault) + String(J2axisFault)
          + String(J3axisFault) + String(J4axisFault)
          + String(J5axisFault) + String(J6axisFault)
      );

      inData = "";  // Clear recieved buffer
      ////////MOVE COMPLETE///////////
    }



    //----- LIVE JOINT JOG  ---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "LJ") {

      int J1dir;
      int J2dir;
      int J3dir;
      int J4dir;
      int J5dir;
      int J6dir;
      int J7dir;
      int J8dir;
      int J9dir;

      int J1axisFault = 0;
      int J2axisFault = 0;
      int J3axisFault = 0;
      int J4axisFault = 0;
      int J5axisFault = 0;
      int J6axisFault = 0;
      int J7axisFault = 0;
      int J8axisFault = 0;
      int J9axisFault = 0;
      int TotalAxisFault = 0;
      ar4_protocol::LiveControlFrameStatus liveControlStatus =
        ar4_protocol::LiveControlFrameStatus::kPending;

      bool JogInPoc = true;
      Alarm = "0";


      ar4_protocol::LiveJogCommandFields commandFields = {};
      if (!ar4_protocol::parse_live_jog_command(
          inData,
          ar4_protocol::LiveJogCommandKind::kJoint,
          commandFields
      )) {
        Serial.println("ER");
        consume_current_command();
        return;
      }

      int Vector = commandFields.vector;
      String SpeedType(commandFields.speed_mode);
      float SpeedVal = commandFields.speed;
      float ACCspd = commandFields.acceleration;
      float DCCspd = commandFields.deceleration;
      float ACCramp = commandFields.ramp;
      ar4_protocol::MotionModeTransaction<String, ROBOT_nDOFs> motionModes(
        WristCon,
        JointLoopModes,
        String(commandFields.wrist_config),
        commandFields.loop_modes
      );

      inData = "";  // Clear recieved buffer

      //clear serial
      delay(5);
      Serial.println();
      updatePos();

      float J1Angle = JangleIn[0];
      float J2Angle = JangleIn[1];
      float J3Angle = JangleIn[2];
      float J4Angle = JangleIn[3];
      float J5Angle = JangleIn[4];
      float J6Angle = JangleIn[5];
      float J7Angle = J7_pos;
      float J8Angle = J8_pos;
      float J9Angle = J9_pos;
      float xyzuvw_In[6];

      while (JogInPoc == true) {

        if (Vector == 10) {
          J1Angle = JangleIn[0] - .25;
        }
        if (Vector == 11) {
          J1Angle = JangleIn[0] + .25;
        }

        if (Vector == 20) {
          J2Angle = JangleIn[1] - .25;
        }
        if (Vector == 21) {
          J2Angle = JangleIn[1] + .25;
        }

        if (Vector == 30) {
          J3Angle = JangleIn[2] - .25;
        }
        if (Vector == 31) {
          J3Angle = JangleIn[2] + .25;
        }

        if (Vector == 40) {
          J4Angle = JangleIn[3] - .25;
        }
        if (Vector == 41) {
          J4Angle = JangleIn[3] + .25;
        }

        if (Vector == 50) {
          J5Angle = JangleIn[4] - .25;
        }
        if (Vector == 51) {
          J5Angle = JangleIn[4] + .25;
        }

        if (Vector == 60) {
          J6Angle = JangleIn[5] - .25;
        }
        if (Vector == 61) {
          J6Angle = JangleIn[5] + .25;
        }
        if (Vector == 70) {
          J7Angle = J7_pos - .25;
        }
        if (Vector == 71) {
          J7Angle = J7_pos + .25;
        }
        if (Vector == 80) {
          J8Angle = J8_pos - .25;
        }
        if (Vector == 81) {
          J8Angle = J8_pos + .25;
        }
        if (Vector == 90) {
          J9Angle = J9_pos - .25;
        }
        if (Vector == 91) {
          J9Angle = J9_pos + .25;
        }

        //calc destination motor steps
        const float positions[numJoints] = {
          J1Angle, J2Angle, J3Angle, J4Angle, J5Angle, J6Angle,
          J7Angle, J8Angle, J9Angle,
        };
        int future_steps[numJoints] = {};
        if (!joint_positions_to_future_steps(positions, future_steps)) {
          KinematicError = 1;
          break;
        }
        int J1futStepM = future_steps[0];
        int J2futStepM = future_steps[1];
        int J3futStepM = future_steps[2];
        int J4futStepM = future_steps[3];
        int J5futStepM = future_steps[4];
        int J6futStepM = future_steps[5];
        int J7futStepM = future_steps[6];
        int J8futStepM = future_steps[7];
        int J9futStepM = future_steps[8];

        //calc delta from current to destination
        int J1stepDif = J1StepM - J1futStepM;
        int J2stepDif = J2StepM - J2futStepM;
        int J3stepDif = J3StepM - J3futStepM;
        int J4stepDif = J4StepM - J4futStepM;
        int J5stepDif = J5StepM - J5futStepM;
        int J6stepDif = J6StepM - J6futStepM;
        int J7stepDif = J7StepM - J7futStepM;
        int J8stepDif = J8StepM - J8futStepM;
        int J9stepDif = J9StepM - J9futStepM;

        //determine motor directions
        J1dir = (J1stepDif <= 0) ? 1 : 0;
        J2dir = (J2stepDif <= 0) ? 1 : 0;
        J3dir = (J3stepDif <= 0) ? 1 : 0;
        J4dir = (J4stepDif <= 0) ? 1 : 0;
        J5dir = (J5stepDif <= 0) ? 1 : 0;
        J6dir = (J6stepDif <= 0) ? 1 : 0;
        J7dir = (J7stepDif <= 0) ? 1 : 0;
        J8dir = (J8stepDif <= 0) ? 1 : 0;
        J9dir = (J9stepDif <= 0) ? 1 : 0;

        //determine if requested position is within axis limits
        if (future_step_is_outside_limit(J1futStepM, J1StepLim)) {
          J1axisFault = 1;
        }
        if (future_step_is_outside_limit(J2futStepM, J2StepLim)) {
          J2axisFault = 1;
        }
        if (future_step_is_outside_limit(J3futStepM, J3StepLim)) {
          J3axisFault = 1;
        }
        if (future_step_is_outside_limit(J4futStepM, J4StepLim)) {
          J4axisFault = 1;
        }
        if (future_step_is_outside_limit(J5futStepM, J5StepLim)) {
          J5axisFault = 1;
        }
        if (future_step_is_outside_limit(J6futStepM, J6StepLim)) {
          J6axisFault = 1;
        }
        if (future_step_is_outside_limit(J7futStepM, J7StepLim)) {
          J7axisFault = 1;
        }
        if (future_step_is_outside_limit(J8futStepM, J8StepLim)) {
          J8axisFault = 1;
        }
        if (future_step_is_outside_limit(J9futStepM, J9StepLim)) {
          J9axisFault = 1;
        }
        TotalAxisFault = J1axisFault + J2axisFault + J3axisFault + J4axisFault + J5axisFault + J6axisFault + J7axisFault + J8axisFault + J9axisFault;

        if (TotalAxisFault == 0 && KinematicError == 0) {
          if (!driveMotorsJ(abs(J1stepDif), abs(J2stepDif), abs(J3stepDif), abs(J4stepDif), abs(J5stepDif), abs(J6stepDif), abs(J7stepDif), abs(J8stepDif), abs(J9stepDif), J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, J7dir, J8dir, J9dir, SpeedType, SpeedVal, ACCspd, DCCspd, ACCramp, &motionModes)) {
            KinematicError = 1;
            break;
          }
          updatePos();
        }

        //stop loop if any serial command is recieved - but the expected command is "S" to stop the loop.

        liveControlStatus = read_live_control_frame();
        if (
          liveControlStatus
          != ar4_protocol::LiveControlFrameStatus::kPending
        ) {
          break;
        }

        //end loop
      }

      TotalCollision = J1collisionTrue + J2collisionTrue + J3collisionTrue + J4collisionTrue + J5collisionTrue + J6collisionTrue;
      if (TotalCollision > 0) {
        flag = "EC" + String(J1collisionTrue) + String(J2collisionTrue) + String(J3collisionTrue) + String(J4collisionTrue) + String(J5collisionTrue) + String(J6collisionTrue);
      }

      send_live_terminal_response(
        liveControlStatus,
        KinematicError,
        TotalAxisFault,
        "EL" + String(J1axisFault) + String(J2axisFault)
          + String(J3axisFault) + String(J4axisFault)
          + String(J5axisFault) + String(J6axisFault)
          + String(J7axisFault) + String(J8axisFault)
          + String(J9axisFault)
      );

      inData = "";  // Clear recieved buffer
      ////////MOVE COMPLETE///////////
    }



    //----- LIVE TOOL JOG  ---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "LT") {
      int J1dir;
      int J2dir;
      int J3dir;
      int J4dir;
      int J5dir;
      int J6dir;
      int J7dir;
      int J8dir;
      int J9dir;

      int J1axisFault = 0;
      int J2axisFault = 0;
      int J3axisFault = 0;
      int J4axisFault = 0;
      int J5axisFault = 0;
      int J6axisFault = 0;
      int TRaxisFault = 0;
      int TotalAxisFault = 0;
      ar4_protocol::LiveControlFrameStatus liveControlStatus =
        ar4_protocol::LiveControlFrameStatus::kPending;

      float Xtool = Robot_Kin_Tool[0];
      float Ytool = Robot_Kin_Tool[1];
      float Ztool = Robot_Kin_Tool[2];
      float RZtool = Robot_Kin_Tool[5];
      float RYtool = Robot_Kin_Tool[4];
      float RXtool = Robot_Kin_Tool[3];

      bool JogInPoc = true;
      Alarm = "0";

      ar4_protocol::LiveJogCommandFields commandFields = {};
      if (!ar4_protocol::parse_live_jog_command(
          inData,
          ar4_protocol::LiveJogCommandKind::kTool,
          commandFields
      )) {
        Serial.println("ER");
        consume_current_command();
        return;
      }

      int Vector = commandFields.vector;
      String SpeedType(commandFields.speed_mode);
      float SpeedVal = commandFields.speed;
      float ACCspd = commandFields.acceleration;
      float DCCspd = commandFields.deceleration;
      float ACCramp = commandFields.ramp;
      int toolFrameIndex = -1;
      float toolFrameOffset = 0.0f;
      if (
        !ar4_protocol::decode_live_tool_offset(
          Vector,
          ar4_protocol::kLiveToolJogIncrement,
          toolFrameIndex,
          toolFrameOffset
        )
      ) {
        Serial.println("ER");
        consume_current_command();
        return;
      }
      ar4_protocol::MotionModeTransaction<String, ROBOT_nDOFs> motionModes(
        WristCon,
        JointLoopModes,
        String(commandFields.wrist_config),
        commandFields.loop_modes
      );

      inData = "";  // Clear recieved buffer

      delay(5);
      Serial.println();
      updatePos();

      Xtool = Robot_Kin_Tool[0];
      Ytool = Robot_Kin_Tool[1];
      Ztool = Robot_Kin_Tool[2];
      RXtool = Robot_Kin_Tool[3];
      RYtool = Robot_Kin_Tool[4];
      RZtool = Robot_Kin_Tool[5];

      JangleIn[0] = (J1StepM - J1zeroStep) / J1StepDeg;
      JangleIn[1] = (J2StepM - J2zeroStep) / J2StepDeg;
      JangleIn[2] = (J3StepM - J3zeroStep) / J3StepDeg;
      JangleIn[3] = (J4StepM - J4zeroStep) / J4StepDeg;
      JangleIn[4] = (J5StepM - J5zeroStep) / J5StepDeg;
      JangleIn[5] = (J6StepM - J6zeroStep) / J6StepDeg;

      while (JogInPoc == true && KinematicError == 0) {

        Xtool = Robot_Kin_Tool[0];
        Ytool = Robot_Kin_Tool[1];
        Ztool = Robot_Kin_Tool[2];
        RXtool = Robot_Kin_Tool[3];
        RYtool = Robot_Kin_Tool[4];
        RZtool = Robot_Kin_Tool[5];

        Robot_Kin_Tool[toolFrameIndex] += toolFrameOffset;



        xyzuvw_In[0] = xyzuvw_Out[0];
        xyzuvw_In[1] = xyzuvw_Out[1];
        xyzuvw_In[2] = xyzuvw_Out[2];
        xyzuvw_In[3] = xyzuvw_Out[3];
        xyzuvw_In[4] = xyzuvw_Out[4];
        xyzuvw_In[5] = xyzuvw_Out[5];

        SolveInverseKinematics(commandFields.wrist_config);

        Robot_Kin_Tool[0] = Xtool;
        Robot_Kin_Tool[1] = Ytool;
        Robot_Kin_Tool[2] = Ztool;
        Robot_Kin_Tool[3] = RXtool;
        Robot_Kin_Tool[4] = RYtool;
        Robot_Kin_Tool[5] = RZtool;

        //calc destination motor steps
        int future_steps[ROBOT_nDOFs] = {};
        if (
          KinematicError != 0
          || !primary_inverse_solution_to_future_steps(future_steps)
        ) {
          KinematicError = 1;
          break;
        }
        int J1futStepM = future_steps[0];
        int J2futStepM = future_steps[1];
        int J3futStepM = future_steps[2];
        int J4futStepM = future_steps[3];
        int J5futStepM = future_steps[4];
        int J6futStepM = future_steps[5];

        //calc delta from current to destination
        int J1stepDif = J1StepM - J1futStepM;
        int J2stepDif = J2StepM - J2futStepM;
        int J3stepDif = J3StepM - J3futStepM;
        int J4stepDif = J4StepM - J4futStepM;
        int J5stepDif = J5StepM - J5futStepM;
        int J6stepDif = J6StepM - J6futStepM;
        int J7stepDif = 0;
        int J8stepDif = 0;
        int J9stepDif = 0;

        //determine motor directions
        J1dir = (J1stepDif <= 0) ? 1 : 0;
        J2dir = (J2stepDif <= 0) ? 1 : 0;
        J3dir = (J3stepDif <= 0) ? 1 : 0;
        J4dir = (J4stepDif <= 0) ? 1 : 0;
        J5dir = (J5stepDif <= 0) ? 1 : 0;
        J6dir = (J6stepDif <= 0) ? 1 : 0;
        J7dir = 0;
        J8dir = 0;
        J9dir = 0;


        //determine if requested position is within axis limits
        if (future_step_is_outside_limit(J1futStepM, J1StepLim)) {
          J1axisFault = 1;
        }
        if (future_step_is_outside_limit(J2futStepM, J2StepLim)) {
          J2axisFault = 1;
        }
        if (future_step_is_outside_limit(J3futStepM, J3StepLim)) {
          J3axisFault = 1;
        }
        if (future_step_is_outside_limit(J4futStepM, J4StepLim)) {
          J4axisFault = 1;
        }
        if (future_step_is_outside_limit(J5futStepM, J5StepLim)) {
          J5axisFault = 1;
        }
        if (future_step_is_outside_limit(J6futStepM, J6StepLim)) {
          J6axisFault = 1;
        }
        TotalAxisFault = J1axisFault + J2axisFault + J3axisFault + J4axisFault + J5axisFault + J6axisFault;


        if (TotalAxisFault == 0 && KinematicError == 0) {
          if (!driveMotorsJ(abs(J1stepDif), abs(J2stepDif), abs(J3stepDif), abs(J4stepDif), abs(J5stepDif), abs(J6stepDif), abs(J7stepDif), abs(J8stepDif), abs(J9stepDif), J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, J7dir, J8dir, J9dir, SpeedType, SpeedVal, ACCspd, DCCspd, ACCramp, &motionModes)) {
            KinematicError = 1;
            break;
          }
          updatePos();
        }

        //stop loop if any serial command is recieved - but the expected command is "S" to stop the loop.

        liveControlStatus = read_live_control_frame();
        if (
          liveControlStatus
          != ar4_protocol::LiveControlFrameStatus::kPending
        ) {
          break;
        }

        //end loop
      }

      TotalCollision = J1collisionTrue + J2collisionTrue + J3collisionTrue + J4collisionTrue + J5collisionTrue + J6collisionTrue;
      if (TotalCollision > 0) {
        flag = "EC" + String(J1collisionTrue) + String(J2collisionTrue) + String(J3collisionTrue) + String(J4collisionTrue) + String(J5collisionTrue) + String(J6collisionTrue);
      }

      send_live_terminal_response(
        liveControlStatus,
        KinematicError,
        TotalAxisFault,
        "EL" + String(J1axisFault) + String(J2axisFault)
          + String(J3axisFault) + String(J4axisFault)
          + String(J5axisFault) + String(J6axisFault)
      );

      inData = "";  // Clear recieved buffer
      ////////MOVE COMPLETE///////////
    }



    //----- Jog T ---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "JT") {
      int J1dir;
      int J2dir;
      int J3dir;
      int J4dir;
      int J5dir;
      int J6dir;
      int J7dir;
      int J8dir;
      int J9dir;

      float Xtool = Robot_Kin_Tool[0];
      float Ytool = Robot_Kin_Tool[1];
      float Ztool = Robot_Kin_Tool[2];
      float RZtool = Robot_Kin_Tool[5];
      float RYtool = Robot_Kin_Tool[4];
      float RXtool = Robot_Kin_Tool[3];

      int J1axisFault = 0;
      int J2axisFault = 0;
      int J3axisFault = 0;
      int J4axisFault = 0;
      int J5axisFault = 0;
      int J6axisFault = 0;
      int TotalAxisFault = 0;

      String Alarm = "0";

      ar4_protocol::ToolJogCommandFields commandFields = {};
      if (!ar4_protocol::parse_tool_jog_command(inData, commandFields)) {
        Serial.println("ER");
        consume_current_command();
        return;
      }
      float Dist = commandFields.distance;
      String SpeedType(commandFields.speed_mode);
      float SpeedVal = commandFields.speed;
      float ACCspd = commandFields.acceleration;
      float DCCspd = commandFields.deceleration;
      float ACCramp = commandFields.ramp;

      int toolFrameIndex = -1;
      float toolFrameOffset = 0.0f;
      if (
        !ar4_protocol::decode_discrete_tool_offset(
          commandFields.axis,
          static_cast<char>('0' + commandFields.direction),
          Dist,
          toolFrameIndex,
          toolFrameOffset
        )
      ) {
        Serial.println("ER");
        consume_current_command();
        return;
      }
      ar4_protocol::MotionModeTransaction<String, ROBOT_nDOFs> motionModes(
        WristCon,
        JointLoopModes,
        String(commandFields.wrist_config),
        commandFields.loop_modes
      );
      Robot_Kin_Tool[toolFrameIndex] += toolFrameOffset;


      JangleIn[0] = (J1StepM - J1zeroStep) / J1StepDeg;
      JangleIn[1] = (J2StepM - J2zeroStep) / J2StepDeg;
      JangleIn[2] = (J3StepM - J3zeroStep) / J3StepDeg;
      JangleIn[3] = (J4StepM - J4zeroStep) / J4StepDeg;
      JangleIn[4] = (J5StepM - J5zeroStep) / J5StepDeg;
      JangleIn[5] = (J6StepM - J6zeroStep) / J6StepDeg;


      xyzuvw_In[0] = xyzuvw_Out[0];
      xyzuvw_In[1] = xyzuvw_Out[1];
      xyzuvw_In[2] = xyzuvw_Out[2];
      xyzuvw_In[3] = xyzuvw_Out[3];
      xyzuvw_In[4] = xyzuvw_Out[4];
      xyzuvw_In[5] = xyzuvw_Out[5];

      SolveInverseKinematics(commandFields.wrist_config);

      Robot_Kin_Tool[0] = Xtool;
      Robot_Kin_Tool[1] = Ytool;
      Robot_Kin_Tool[2] = Ztool;
      Robot_Kin_Tool[3] = RXtool;
      Robot_Kin_Tool[4] = RYtool;
      Robot_Kin_Tool[5] = RZtool;


      //calc destination motor steps
      int future_steps[ROBOT_nDOFs] = {};
      if (
        KinematicError == 0
        && !primary_inverse_solution_to_future_steps(future_steps)
      ) {
        KinematicError = 1;
      }
      int J1futStepM = future_steps[0];
      int J2futStepM = future_steps[1];
      int J3futStepM = future_steps[2];
      int J4futStepM = future_steps[3];
      int J5futStepM = future_steps[4];
      int J6futStepM = future_steps[5];

      //calc delta from current to destination
      int J1stepDif = J1StepM - J1futStepM;
      int J2stepDif = J2StepM - J2futStepM;
      int J3stepDif = J3StepM - J3futStepM;
      int J4stepDif = J4StepM - J4futStepM;
      int J5stepDif = J5StepM - J5futStepM;
      int J6stepDif = J6StepM - J6futStepM;
      int J7stepDif = 0;
      int J8stepDif = 0;
      int J9stepDif = 0;

      //determine motor directions
      J1dir = (J1stepDif <= 0) ? 1 : 0;
      J2dir = (J2stepDif <= 0) ? 1 : 0;
      J3dir = (J3stepDif <= 0) ? 1 : 0;
      J4dir = (J4stepDif <= 0) ? 1 : 0;
      J5dir = (J5stepDif <= 0) ? 1 : 0;
      J6dir = (J6stepDif <= 0) ? 1 : 0;
      J7dir = 0;
      J8dir = 0;
      J9dir = 0;

      //determine if requested position is within axis limits
      if (future_step_is_outside_limit(J1futStepM, J1StepLim)) {
        J1axisFault = 1;
      }
      if (future_step_is_outside_limit(J2futStepM, J2StepLim)) {
        J2axisFault = 1;
      }
      if (future_step_is_outside_limit(J3futStepM, J3StepLim)) {
        J3axisFault = 1;
      }
      if (future_step_is_outside_limit(J4futStepM, J4StepLim)) {
        J4axisFault = 1;
      }
      if (future_step_is_outside_limit(J5futStepM, J5StepLim)) {
        J5axisFault = 1;
      }
      if (future_step_is_outside_limit(J6futStepM, J6StepLim)) {
        J6axisFault = 1;
      }
      TotalAxisFault = J1axisFault + J2axisFault + J3axisFault + J4axisFault + J5axisFault + J6axisFault;

      debug = String(SpeedVal);
      if (TotalAxisFault == 0 && KinematicError == 0) {
        resetEncoders();
        if (!driveMotorsJ(abs(J1stepDif), abs(J2stepDif), abs(J3stepDif), abs(J4stepDif), abs(J5stepDif), abs(J6stepDif), abs(J7stepDif), abs(J8stepDif), abs(J9stepDif), J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, J7dir, J8dir, J9dir, SpeedType, SpeedVal, ACCspd, DCCspd, ACCramp, &motionModes)) {
          Serial.println("ER");
          consume_current_command();
          return;
        }
        checkEncoders();
        sendRobotPos();
      } else if (KinematicError == 1) {
        Alarm = "ER";
        delay(5);
        Serial.println(Alarm);
        Alarm = "0";
      } else {
        Alarm = "EL" + String(J1axisFault) + String(J2axisFault) + String(J3axisFault) + String(J4axisFault) + String(J5axisFault) + String(J6axisFault);
        delay(5);
        Serial.println(Alarm);
        Alarm = "0";
      }

      inData = "";  // Clear recieved buffer
      ////////MOVE COMPLETE///////////
    }





    //----- MOVE V ------ VISION OFFSET ----------------------------------
    //-----------------------------------------------------------------------
    if (function == "MV") {
      int J1dir;
      int J2dir;
      int J3dir;
      int J4dir;
      int J5dir;
      int J6dir;
      int J7dir;
      int J8dir;
      int J9dir;

      int J1axisFault = 0;
      int J2axisFault = 0;
      int J3axisFault = 0;
      int J4axisFault = 0;
      int J5axisFault = 0;
      int J6axisFault = 0;
      int J7axisFault = 0;
      int J8axisFault = 0;
      int J9axisFault = 0;
      int TotalAxisFault = 0;

      ar4_protocol::CartesianMoveCommandFields commandFields = {};
      if (!ar4_protocol::parse_vision_move_command(inData, commandFields)) {
        Serial.println("ER");
        consume_current_command();
        return;
      }

      float vision_rotation_radians = 0.0f;
      if (!ar4_protocol::degrees_to_radians(
          commandFields.vision_rotation_degrees,
          vision_rotation_radians
      )) {
        Serial.println("ER");
        consume_current_command();
        return;
      }

      for (int axis = 0; axis < ROBOT_nDOFs; ++axis) {
        xyzuvw_In[axis] = commandFields.pose[axis];
      }
      J7_In = commandFields.auxiliary[0];
      J8_In = commandFields.auxiliary[1];
      J9_In = commandFields.auxiliary[2];
      String SpeedType(commandFields.speed_mode);
      float SpeedVal = commandFields.speed;
      float ACCspd = commandFields.acceleration;
      float DCCspd = commandFields.deceleration;
      float ACCramp = commandFields.ramp;
      ar4_protocol::MotionModeTransaction<String, ROBOT_nDOFs> motionModes(
        WristCon,
        JointLoopModes,
        String(commandFields.wrist_config),
        commandFields.loop_modes
      );

      //get current tool rotation
      float RXtool = Robot_Kin_Tool[3];


      // offset tool rotation by the found vision angle
      Robot_Kin_Tool[3] = Robot_Kin_Tool[3] - vision_rotation_radians;

      //solve kinematics
      SolveInverseKinematics(commandFields.wrist_config);

      //calc destination motor steps
      int future_steps[numJoints] = {};
      if (
        KinematicError == 0
        && !inverse_solution_to_future_steps(J7_In, J8_In, J9_In, future_steps)
      ) {
        KinematicError = 1;
      }
      int J1futStepM = future_steps[0];
      int J2futStepM = future_steps[1];
      int J3futStepM = future_steps[2];
      int J4futStepM = future_steps[3];
      int J5futStepM = future_steps[4];
      int J6futStepM = future_steps[5];
      int J7futStepM = future_steps[6];
      int J8futStepM = future_steps[7];
      int J9futStepM = future_steps[8];


      //calc delta from current to destination
      int J1stepDif = J1StepM - J1futStepM;
      int J2stepDif = J2StepM - J2futStepM;
      int J3stepDif = J3StepM - J3futStepM;
      int J4stepDif = J4StepM - J4futStepM;
      int J5stepDif = J5StepM - J5futStepM;
      int J6stepDif = J6StepM - J6futStepM;
      int J7stepDif = J7StepM - J7futStepM;
      int J8stepDif = J8StepM - J8futStepM;
      int J9stepDif = J9StepM - J9futStepM;

      // put tool roation back where it was
      Robot_Kin_Tool[3] = RXtool;

      //determine motor directions
      J1dir = (J1stepDif <= 0) ? 1 : 0;
      J2dir = (J2stepDif <= 0) ? 1 : 0;
      J3dir = (J3stepDif <= 0) ? 1 : 0;
      J4dir = (J4stepDif <= 0) ? 1 : 0;
      J5dir = (J5stepDif <= 0) ? 1 : 0;
      J6dir = (J6stepDif <= 0) ? 1 : 0;
      J7dir = (J7stepDif <= 0) ? 1 : 0;
      J8dir = (J8stepDif <= 0) ? 1 : 0;
      J9dir = (J9stepDif <= 0) ? 1 : 0;



      //determine if requested position is within axis limits
      if (future_step_is_outside_limit(J1futStepM, J1StepLim)) {
        J1axisFault = 1;
      }
      if (future_step_is_outside_limit(J2futStepM, J2StepLim)) {
        J2axisFault = 1;
      }
      if (future_step_is_outside_limit(J3futStepM, J3StepLim)) {
        J3axisFault = 1;
      }
      if (future_step_is_outside_limit(J4futStepM, J4StepLim)) {
        J4axisFault = 1;
      }
      if (future_step_is_outside_limit(J5futStepM, J5StepLim)) {
        J5axisFault = 1;
      }
      if (future_step_is_outside_limit(J6futStepM, J6StepLim)) {
        J6axisFault = 1;
      }
      if (future_step_is_outside_limit(J7futStepM, J7StepLim)) {
        J7axisFault = 1;
      }
      if (future_step_is_outside_limit(J8futStepM, J8StepLim)) {
        J8axisFault = 1;
      }
      if (future_step_is_outside_limit(J9futStepM, J9StepLim)) {
        J9axisFault = 1;
      }
      TotalAxisFault = J1axisFault + J2axisFault + J3axisFault + J4axisFault + J5axisFault + J6axisFault + J7axisFault + J8axisFault + J9axisFault;


      if (TotalAxisFault == 0 && KinematicError == 0) {
        resetEncoders();
        if (!driveMotorsJ(abs(J1stepDif), abs(J2stepDif), abs(J3stepDif), abs(J4stepDif), abs(J5stepDif), abs(J6stepDif), abs(J7stepDif), abs(J8stepDif), abs(J9stepDif), J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, J7dir, J8dir, J9dir, SpeedType, SpeedVal, ACCspd, DCCspd, ACCramp, &motionModes)) {
          Serial.println("ER");
          consume_current_command();
          return;
        }
        checkEncoders();
        sendRobotPos();
      } else if (KinematicError == 1) {
        Alarm = "ER";
        delay(5);
        Serial.println(Alarm);
        Alarm = "0";
      } else {
        Alarm = "EL" + String(J1axisFault) + String(J2axisFault) + String(J3axisFault) + String(J4axisFault) + String(J5axisFault) + String(J6axisFault) + String(J7axisFault) + String(J8axisFault) + String(J9axisFault);
        delay(5);
        Serial.println(Alarm);
        Alarm = "0";
      }



      inData = "";  // Clear recieved buffer
      ////////MOVE COMPLETE///////////
    }




    //----- MOVE IN JOINTS ROTATION  ---------------------------------------------------
    //-----------------------------------------------------------------------

    if (function == "RJ") {
      int J1dir;
      int J2dir;
      int J3dir;
      int J4dir;
      int J5dir;
      int J6dir;
      int J7dir;
      int J8dir;
      int J9dir;

      int J1axisFault = 0;
      int J2axisFault = 0;
      int J3axisFault = 0;
      int J4axisFault = 0;
      int J5axisFault = 0;
      int J6axisFault = 0;
      int J7axisFault = 0;
      int J8axisFault = 0;
      int J9axisFault = 0;
      int TotalAxisFault = 0;

      ar4_protocol::JointMoveCommandFields commandFields = {};
      if (!ar4_protocol::parse_joint_move_command(inData, commandFields)) {
        Serial.println("ER");
        consume_current_command();
        return;
      }
      int future_steps[numJoints] = {};
      if (!joint_positions_to_future_steps(
          commandFields.positions,
          future_steps
      )) {
        Serial.println("ER");
        consume_current_command();
        return;
      }
      J7_In = commandFields.positions[6];
      J8_In = commandFields.positions[7];
      J9_In = commandFields.positions[8];
      String SpeedType(commandFields.speed_mode);
      float SpeedVal = commandFields.speed;
      float ACCspd = commandFields.acceleration;
      float DCCspd = commandFields.deceleration;
      float ACCramp = commandFields.ramp;
      ar4_protocol::MotionModeTransaction<String, ROBOT_nDOFs> motionModes(
        WristCon,
        JointLoopModes,
        String(commandFields.wrist_config),
        commandFields.loop_modes
      );

      int J1futStepM = future_steps[0];
      int J2futStepM = future_steps[1];
      int J3futStepM = future_steps[2];
      int J4futStepM = future_steps[3];
      int J5futStepM = future_steps[4];
      int J6futStepM = future_steps[5];
      int J7futStepM = future_steps[6];
      int J8futStepM = future_steps[7];
      int J9futStepM = future_steps[8];

      //calc delta from current to destination
      int J1stepDif = J1StepM - J1futStepM;
      int J2stepDif = J2StepM - J2futStepM;
      int J3stepDif = J3StepM - J3futStepM;
      int J4stepDif = J4StepM - J4futStepM;
      int J5stepDif = J5StepM - J5futStepM;
      int J6stepDif = J6StepM - J6futStepM;
      int J7stepDif = J7StepM - J7futStepM;
      int J8stepDif = J8StepM - J8futStepM;
      int J9stepDif = J9StepM - J9futStepM;


      //determine motor directions
      J1dir = (J1stepDif <= 0) ? 1 : 0;
      J2dir = (J2stepDif <= 0) ? 1 : 0;
      J3dir = (J3stepDif <= 0) ? 1 : 0;
      J4dir = (J4stepDif <= 0) ? 1 : 0;
      J5dir = (J5stepDif <= 0) ? 1 : 0;
      J6dir = (J6stepDif <= 0) ? 1 : 0;
      J7dir = (J7stepDif <= 0) ? 1 : 0;
      J8dir = (J8stepDif <= 0) ? 1 : 0;
      J9dir = (J9stepDif <= 0) ? 1 : 0;


      //determine if requested position is within axis limits
      if (future_step_is_outside_limit(J1futStepM, J1StepLim)) {
        J1axisFault = 1;
      }
      if (future_step_is_outside_limit(J2futStepM, J2StepLim)) {
        J2axisFault = 1;
      }
      if (future_step_is_outside_limit(J3futStepM, J3StepLim)) {
        J3axisFault = 1;
      }
      if (future_step_is_outside_limit(J4futStepM, J4StepLim)) {
        J4axisFault = 1;
      }
      if (future_step_is_outside_limit(J5futStepM, J5StepLim)) {
        J5axisFault = 1;
      }
      if (future_step_is_outside_limit(J6futStepM, J6StepLim)) {
        J6axisFault = 1;
      }
      if (future_step_is_outside_limit(J7futStepM, J7StepLim)) {
        J7axisFault = 1;
      }
      if (future_step_is_outside_limit(J8futStepM, J8StepLim)) {
        J8axisFault = 1;
      }
      if (future_step_is_outside_limit(J9futStepM, J9StepLim)) {
        J9axisFault = 1;
      }
      TotalAxisFault = J1axisFault + J2axisFault + J3axisFault + J4axisFault + J5axisFault + J6axisFault + J7axisFault + J8axisFault + J9axisFault;


      if (TotalAxisFault == 0 && KinematicError == 0) {
        resetEncoders();
        if (!driveMotorsJ(abs(J1stepDif), abs(J2stepDif), abs(J3stepDif), abs(J4stepDif), abs(J5stepDif), abs(J6stepDif), abs(J7stepDif), abs(J8stepDif), abs(J9stepDif), J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, J7dir, J8dir, J9dir, SpeedType, SpeedVal, ACCspd, DCCspd, ACCramp, &motionModes)) {
          Serial.println("ER");
          consume_current_command();
          return;
        }
        checkEncoders();
        sendRobotPos();
      } else if (KinematicError == 1) {
        Alarm = "ER";
        delay(5);
        Serial.println(Alarm);
      } else {
        Alarm = "EL" + String(J1axisFault) + String(J2axisFault) + String(J3axisFault) + String(J4axisFault) + String(J5axisFault) + String(J6axisFault) + String(J7axisFault) + String(J8axisFault) + String(J9axisFault);
        delay(5);
        Serial.println(Alarm);
      }


      inData = "";  // Clear recieved buffer
      ////////MOVE COMPLETE///////////
    }



    //----- MOVE L ---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "ML" and flag == "") {
      int J1dir;
      int J2dir;
      int J3dir;
      int J4dir;
      int J5dir;
      int J6dir;
      int J7dir;
      int J8dir;
      int J9dir;

      float curDelay;

      String nextCMDtype;
      String test;

      int J1axisFault = 0;
      int J2axisFault = 0;
      int J3axisFault = 0;
      int J4axisFault = 0;
      int J5axisFault = 0;
      int J6axisFault = 0;
      int J7axisFault = 0;
      int J8axisFault = 0;
      int J9axisFault = 0;
      int TotalAxisFault = 0;

      //String Alarm = "0";

      float curWayDis;
      float speedSP;

      float Xvect;
      float Yvect;
      float Zvect;
      float RZvect;
      float RYvect;
      float RXvect;

      ar4_protocol::CartesianMoveCommandFields commandFields = {};
      if (!ar4_protocol::parse_linear_move_command(
          inData,
          commandFields
      )) {
        Serial.println("ER");
        consume_current_command();
        return;
      }
      int external_target_steps[3] = {};
      if (!external_positions_to_future_steps(
          commandFields.auxiliary[0],
          commandFields.auxiliary[1],
          commandFields.auxiliary[2],
          external_target_steps
      )) {
        Serial.println("ER");
        consume_current_command();
        return;
      }

      for (int axis = 0; axis < ROBOT_nDOFs; ++axis) {
        xyzuvw_Temp[axis] = commandFields.pose[axis];
      }
      J7_In = commandFields.auxiliary[0];
      J8_In = commandFields.auxiliary[1];
      J9_In = commandFields.auxiliary[2];
      String SpeedType(commandFields.speed_mode);
      float SpeedVal = commandFields.speed;
      float ACCspd = commandFields.acceleration;
      float DCCspd = commandFields.deceleration;
      float ACCramp = commandFields.ramp;
      float Rounding = commandFields.rounding;
      ar4_protocol::MotionModeTransaction<String, ROBOT_nDOFs> motionModes(
        WristCon,
        JointLoopModes,
        String(commandFields.wrist_config),
        commandFields.loop_modes
      );


      ///// rounding logic /////
      if (cmdBuffer2 != "") {
        if (ar4_protocol::extract_serial_command_payload(
            cmdBuffer2,
            checkData
        )) {
          nextCMDtype = checkData.substring(0, 1);
          checkData = checkData.substring(2);
        } else {
          nextCMDtype = "";
          checkData = "";
        }
      }
      if (splineTrue == true and Rounding > 0 and nextCMDtype == "M") {
        //calculate new end point before rounding arc
        updatePos();
        //vector
        float Xvect = xyzuvw_Temp[0] - xyzuvw_Out[0];
        float Yvect = xyzuvw_Temp[1] - xyzuvw_Out[1];
        float Zvect = xyzuvw_Temp[2] - xyzuvw_Out[2];
        float RZvect = xyzuvw_Temp[3] - xyzuvw_Out[3];
        float RYvect = xyzuvw_Temp[4] - xyzuvw_Out[4];
        float RXvect = xyzuvw_Temp[5] - xyzuvw_Out[5];
        //start pos
        float Xstart = xyzuvw_Out[0];
        float Ystart = xyzuvw_Out[1];
        float Zstart = xyzuvw_Out[2];
        float RZstart = xyzuvw_Out[3];
        float RYstart = xyzuvw_Out[4];
        float RXstart = xyzuvw_Out[5];
        //line dist
        float lineDist = pow((pow((Xvect), 2) + pow((Yvect), 2) + pow((Zvect), 2) + pow((RZvect), 2) + pow((RYvect), 2) + pow((RXvect), 2)), .5);
        if (!isfinite(lineDist) || lineDist <= 0.0f) {
          Serial.println("ER");
          consume_current_command();
          return;
        }
        if (Rounding > (lineDist * .45)) {
          Rounding = lineDist * .45;
        }
        float newDistPerc = 1 - (Rounding / lineDist);
        //cropped destination (new end point before rounding arc)
        xyzuvw_In[0] = Xstart + (Xvect * newDistPerc);
        xyzuvw_In[1] = Ystart + (Yvect * newDistPerc);
        xyzuvw_In[2] = Zstart + (Zvect * newDistPerc);
        xyzuvw_In[3] = RZstart + (RZvect * newDistPerc);
        xyzuvw_In[4] = RYstart + (RYvect * newDistPerc);
        xyzuvw_In[5] = RXstart + (RXvect * newDistPerc);
        ar4_protocol::CartesianMoveCommandFields nextCommandFields = {};
        if (!ar4_protocol::parse_linear_move_command(
            checkData,
            nextCommandFields
        )) {
          Serial.println("ER");
          consume_current_command();
          return;
        }
        for (int axis = 0; axis < ROBOT_nDOFs; ++axis) {
          rndArcEnd[axis] = nextCommandFields.pose[axis];
        }
        //arc vector
        Xvect = rndArcEnd[0] - xyzuvw_Temp[0];
        Yvect = rndArcEnd[1] - xyzuvw_Temp[1];
        Zvect = rndArcEnd[2] - xyzuvw_Temp[2];
        RZvect = rndArcEnd[3] - xyzuvw_Temp[3];
        RYvect = rndArcEnd[4] - xyzuvw_Temp[4];
        RXvect = rndArcEnd[5] - xyzuvw_Temp[5];
        //end arc start pos
        Xstart = xyzuvw_Temp[0];
        Ystart = xyzuvw_Temp[1];
        Zstart = xyzuvw_Temp[2];
        RZstart = xyzuvw_Temp[3];
        RYstart = xyzuvw_Temp[4];
        RXstart = xyzuvw_Temp[5];
        //line dist
        lineDist = pow((pow((Xvect), 2) + pow((Yvect), 2) + pow((Zvect), 2) + pow((RZvect), 2) + pow((RYvect), 2) + pow((RXvect), 2)), .5);
        if (!isfinite(lineDist) || lineDist <= 0.0f) {
          Serial.println("ER");
          consume_current_command();
          return;
        }
        if (Rounding > (lineDist * .45)) {
          Rounding = lineDist * .45;
        }
        newDistPerc = (Rounding / lineDist);
        //calculated arc end postion
        rndArcEnd[0] = Xstart + (Xvect * newDistPerc);
        rndArcEnd[1] = Ystart + (Yvect * newDistPerc);
        rndArcEnd[2] = Zstart + (Zvect * newDistPerc);
        rndArcEnd[3] = RZstart + (RZvect * newDistPerc);
        rndArcEnd[4] = RYstart + (RYvect * newDistPerc);
        rndArcEnd[5] = RXstart + (RXvect * newDistPerc);
        //calculate arc center point
        rndCalcCen[0] = (xyzuvw_In[0] + rndArcEnd[0]) / 2;
        rndCalcCen[1] = (xyzuvw_In[1] + rndArcEnd[1]) / 2;
        rndCalcCen[2] = (xyzuvw_In[2] + rndArcEnd[2]) / 2;
        rndCalcCen[3] = (xyzuvw_In[3] + rndArcEnd[3]) / 2;
        rndCalcCen[4] = (xyzuvw_In[4] + rndArcEnd[4]) / 2;
        rndCalcCen[5] = (xyzuvw_In[5] + rndArcEnd[5]) / 2;
        rndArcMid[0] = (xyzuvw_Temp[0] + rndCalcCen[0]) / 2;
        rndArcMid[1] = (xyzuvw_Temp[1] + rndCalcCen[1]) / 2;
        rndArcMid[2] = (xyzuvw_Temp[2] + rndCalcCen[2]) / 2;
        rndArcMid[3] = (xyzuvw_Temp[3] + rndCalcCen[3]) / 2;
        rndArcMid[4] = (xyzuvw_Temp[4] + rndCalcCen[4]) / 2;
        rndArcMid[5] = (xyzuvw_Temp[5] + rndCalcCen[5]) / 2;
        //set arc move to be executed
        rndData = "X" + String(rndArcMid[0]) + "Y" + String(rndArcMid[1]) + "Z" + String(rndArcMid[2]) + "Rz" + String(rndArcMid[3]) + "Ry" + String(rndArcMid[4]) + "Rx" + String(rndArcMid[5]) + "Ex" + String(rndArcEnd[0]) + "Ey" + String(rndArcEnd[1]) + "Ez" + String(rndArcEnd[2]) + "Tr0S" + SpeedType + String(SpeedVal) + "Ac" + String(ACCspd) + "Dc" + String(DCCspd) + "Rm" + String(ACCramp) + "W" + String(commandFields.wrist_config) + "Lm" + String(commandFields.loop_modes[0]) + String(commandFields.loop_modes[1]) + String(commandFields.loop_modes[2]) + String(commandFields.loop_modes[3]) + String(commandFields.loop_modes[4]) + String(commandFields.loop_modes[5]);
        function = "MA";
        rndTrue = true;
      } else {
        updatePos();
        xyzuvw_In[0] = xyzuvw_Temp[0];
        xyzuvw_In[1] = xyzuvw_Temp[1];
        xyzuvw_In[2] = xyzuvw_Temp[2];
        xyzuvw_In[3] = xyzuvw_Temp[3];
        xyzuvw_In[4] = xyzuvw_Temp[4];
        xyzuvw_In[5] = xyzuvw_Temp[5];
      }



      //xyz vector
      Xvect = xyzuvw_In[0] - xyzuvw_Out[0];
      Yvect = xyzuvw_In[1] - xyzuvw_Out[1];
      Zvect = xyzuvw_In[2] - xyzuvw_Out[2];
      RZvect = xyzuvw_In[3] - xyzuvw_Out[3];
      RYvect = xyzuvw_In[4] - xyzuvw_Out[4];
      RXvect = xyzuvw_In[5] - xyzuvw_Out[5];


      //start pos
      float Xstart = xyzuvw_Out[0];
      float Ystart = xyzuvw_Out[1];
      float Zstart = xyzuvw_Out[2];
      float RZstart = xyzuvw_Out[3];
      float RYstart = xyzuvw_Out[4];
      float RXstart = xyzuvw_Out[5];


      //line dist and determine way point gap
      float lineDist = pow((pow((Xvect), 2) + pow((Yvect), 2) + pow((Zvect), 2) + pow((RZvect), 2) + pow((RYvect), 2) + pow((RXvect), 2)), .5);
      int waypoint_count = 0;
      if (!ar4_protocol::waypoint_count_for_path(
          lineDist,
          linWayDistSP,
          waypoint_count
      )) {
        Serial.println("ER");
        consume_current_command();
        return;
      }
      if (lineDist > 0) {

        float wayPts = static_cast<float>(waypoint_count);
        float wayPerc = 1.0f / wayPts;

        //pre calculate entire move and speeds

        SolveInverseKinematics(commandFields.wrist_config);
        //calc destination motor steps for precalc
        int destination_steps[ROBOT_nDOFs] = {};
        if (
          KinematicError != 0
          || !primary_inverse_solution_to_future_steps(destination_steps)
        ) {
          Serial.println("ER");
          consume_current_command();
          return;
        }
        int J1futStepM = destination_steps[0];
        int J2futStepM = destination_steps[1];
        int J3futStepM = destination_steps[2];
        int J4futStepM = destination_steps[3];
        int J5futStepM = destination_steps[4];
        int J6futStepM = destination_steps[5];

        //calc delta from current to destination fpr precalc
        int J1stepDif = J1StepM - J1futStepM;
        int J2stepDif = J2StepM - J2futStepM;
        int J3stepDif = J3StepM - J3futStepM;
        int J4stepDif = J4StepM - J4futStepM;
        int J5stepDif = J5StepM - J5futStepM;
        int J6stepDif = J6StepM - J6futStepM;

        //FIND HIGHEST STEP FOR PRECALC
        int HighStep = abs(J1stepDif);
        if (abs(J2stepDif) > HighStep) {
          HighStep = abs(J2stepDif);
        }
        if (abs(J3stepDif) > HighStep) {
          HighStep = abs(J3stepDif);
        }
        if (abs(J4stepDif) > HighStep) {
          HighStep = abs(J4stepDif);
        }
        if (abs(J5stepDif) > HighStep) {
          HighStep = abs(J5stepDif);
        }
        if (abs(J6stepDif) > HighStep) {
          HighStep = abs(J6stepDif);
        }
        const int external_start_steps[3] = { J7StepM, J8StepM, J9StepM };
        for (int axis = 0; axis < 3; ++axis) {
          const int external_difference = abs(
            external_start_steps[axis] - external_target_steps[axis]
          );
          if (external_difference > HighStep) HighStep = external_difference;
        }
        if (HighStep < 1) HighStep = 1;


        /////PRE CALC SPEEDS//////
        float calcStepGap;

        //determine steps
        float ACCStep = HighStep * (ACCspd / 100);
        float NORStep = HighStep * ((100 - ACCspd - DCCspd) / 100);
        float DCCStep = HighStep * (DCCspd / 100);

        //set speed for seconds or mm per sec
        if (SpeedType == "s") {
          speedSP = (SpeedVal * 1000000) * 1.2;
        } else if ((SpeedType == "m")) {
          speedSP = ((lineDist / SpeedVal) * 1000000) * 1.2;
        }
        if (
          (SpeedType == "s" || SpeedType == "m")
          && (!isfinite(speedSP) || speedSP <= 0.0f)
        ) {
          Serial.println("ER");
          consume_current_command();
          return;
        }

        //calc step gap for seconds or mm per sec
        if (SpeedType == "s" or SpeedType == "m") {
          float zeroStepGap = speedSP / HighStep;
          float zeroACCstepInc = (zeroStepGap * (100 / ACCramp)) / ACCStep;
          float zeroACCtime = ((ACCStep)*zeroStepGap) + ((ACCStep - 9) * (((ACCStep) * (zeroACCstepInc / 2))));
          float zeroNORtime = NORStep * zeroStepGap;
          float zeroDCCstepInc = (zeroStepGap * (100 / ACCramp)) / DCCStep;
          float zeroDCCtime = ((DCCStep)*zeroStepGap) + ((DCCStep - 9) * (((DCCStep) * (zeroDCCstepInc / 2))));
          float zeroTOTtime = zeroACCtime + zeroNORtime + zeroDCCtime;
          if (!isfinite(zeroTOTtime) || zeroTOTtime <= 0.0f) {
            calcStepGap = minSpeedDelay;
            speedViolation = "1";
          } else {
            float overclockPerc = speedSP / zeroTOTtime;
            calcStepGap = zeroStepGap * overclockPerc;
          }
          if (calcStepGap <= minSpeedDelay) {
            calcStepGap = minSpeedDelay;
            speedViolation = "1";
          }
        }

        //calc step gap for percentage
        else if (SpeedType == "p") {
          calcStepGap = minSpeedDelay / (SpeedVal / 100);
        }
        if (!isfinite(calcStepGap) || calcStepGap <= 0.0f) {
          Serial.println("ER");
          consume_current_command();
          return;
        }

        //calculate final step increments
        float calcACCstepInc = (calcStepGap * (100 / ACCramp)) / ACCStep;
        float calcDCCstepInc = (calcStepGap * (100 / ACCramp)) / DCCStep;
        float calcACCstartDel = (calcACCstepInc * ACCStep) * 2;
        float calcDCCendDel = (calcDCCstepInc * DCCStep) * 2;


        //calc way pt speeds
        float ACCwayPts = wayPts * (ACCspd / 100);
        float NORwayPts = wayPts * ((100 - ACCspd - DCCspd) / 100);
        float DCCwayPts = wayPts * (DCCspd / 100);

        //calc way inc for lin way steps
        float ACCwayInc = (calcACCstartDel - calcStepGap) / ACCwayPts;
        float DCCwayInc = (calcDCCendDel - calcStepGap) / DCCwayPts;
        if (
          !isfinite(ACCwayInc)
          || !isfinite(DCCwayInc)
          || !ar4_protocol::valid_delay_envelope(
            calcStepGap,
            calcACCstartDel,
            calcDCCendDel,
            rndTrue,
            rndSpeed
          )
        ) {
          Serial.println("ER");
          consume_current_command();
          return;
        }

        //set starting delsy
        if (rndTrue == true) {
          curDelay = rndSpeed;
        } else {
          curDelay = calcACCstartDel;
        }


        resetEncoders();
        /////////////////////////////////////////////////
        //loop through waypoints
        for (int i = 1; i <= waypoint_count; i++) {

          ////DELAY CALC/////
          if (i <= ACCwayPts) {
            curDelay = fmax(calcStepGap, curDelay - ACCwayInc);
          } else if (i >= (wayPts - DCCwayPts)) {
            curDelay = fmin(calcDCCendDel, curDelay + DCCwayInc);
          } else {
            curDelay = calcStepGap;
          }

          float curWayPerc = wayPerc * i;
          xyzuvw_In[0] = Xstart + (Xvect * curWayPerc);
          xyzuvw_In[1] = Ystart + (Yvect * curWayPerc);
          xyzuvw_In[2] = Zstart + (Zvect * curWayPerc);
          xyzuvw_In[3] = RZstart + (RZvect * curWayPerc);
          xyzuvw_In[4] = RYstart + (RYvect * curWayPerc);
          xyzuvw_In[5] = RXstart + (RXvect * curWayPerc);

          SolveInverseKinematics(commandFields.wrist_config);

          //calc destination motor steps
          int future_steps[ROBOT_nDOFs] = {};
          if (
            KinematicError != 0
            || !primary_inverse_solution_to_future_steps(future_steps)
          ) {
            KinematicError = 1;
            break;
          }
          int J1futStepM = future_steps[0];
          int J2futStepM = future_steps[1];
          int J3futStepM = future_steps[2];
          int J4futStepM = future_steps[3];
          int J5futStepM = future_steps[4];
          int J6futStepM = future_steps[5];
          int J7futStepM = 0;
          int J8futStepM = 0;
          int J9futStepM = 0;
          if (
            !ar4_protocol::interpolated_step_target(
              external_start_steps[0], external_target_steps[0],
              i, waypoint_count, J7StepLim, J7futStepM
            )
            || !ar4_protocol::interpolated_step_target(
              external_start_steps[1], external_target_steps[1],
              i, waypoint_count, J8StepLim, J8futStepM
            )
            || !ar4_protocol::interpolated_step_target(
              external_start_steps[2], external_target_steps[2],
              i, waypoint_count, J9StepLim, J9futStepM
            )
          ) {
            KinematicError = 1;
            break;
          }

          //calc delta from current to destination
          int J1stepDif = J1StepM - J1futStepM;
          int J2stepDif = J2StepM - J2futStepM;
          int J3stepDif = J3StepM - J3futStepM;
          int J4stepDif = J4StepM - J4futStepM;
          int J5stepDif = J5StepM - J5futStepM;
          int J6stepDif = J6StepM - J6futStepM;
          int J7stepDif = J7StepM - J7futStepM;
          int J8stepDif = J8StepM - J8futStepM;
          int J9stepDif = J9StepM - J9futStepM;

          //determine motor directions
          J1dir = (J1stepDif <= 0) ? 1 : 0;
          J2dir = (J2stepDif <= 0) ? 1 : 0;
          J3dir = (J3stepDif <= 0) ? 1 : 0;
          J4dir = (J4stepDif <= 0) ? 1 : 0;
          J5dir = (J5stepDif <= 0) ? 1 : 0;
          J6dir = (J6stepDif <= 0) ? 1 : 0;
          J7dir = (J7stepDif <= 0) ? 1 : 0;
          J8dir = (J8stepDif <= 0) ? 1 : 0;
          J9dir = (J9stepDif <= 0) ? 1 : 0;

          //determine if requested position is within axis limits
          if (future_step_is_outside_limit(J1futStepM, J1StepLim)) {
            J1axisFault = 1;
          }
          if (future_step_is_outside_limit(J2futStepM, J2StepLim)) {
            J2axisFault = 1;
          }
          if (future_step_is_outside_limit(J3futStepM, J3StepLim)) {
            J3axisFault = 1;
          }
          if (future_step_is_outside_limit(J4futStepM, J4StepLim)) {
            J4axisFault = 1;
          }
          if (future_step_is_outside_limit(J5futStepM, J5StepLim)) {
            J5axisFault = 1;
          }
          if (future_step_is_outside_limit(J6futStepM, J6StepLim)) {
            J6axisFault = 1;
          }
          if (future_step_is_outside_limit(J7futStepM, J7StepLim)) {
            J7axisFault = 1;
          }
          if (future_step_is_outside_limit(J8futStepM, J8StepLim)) {
            J8axisFault = 1;
          }
          if (future_step_is_outside_limit(J9futStepM, J9StepLim)) {
            J9axisFault = 1;
          }
          TotalAxisFault = J1axisFault + J2axisFault + J3axisFault + J4axisFault + J5axisFault + J6axisFault + J7axisFault + J8axisFault + J9axisFault;

          if (TotalAxisFault != 0) {
            Alarm = "EL" + String(J1axisFault) + String(J2axisFault) + String(J3axisFault) + String(J4axisFault) + String(J5axisFault) + String(J6axisFault) + String(J7axisFault) + String(J8axisFault) + String(J9axisFault);
            break;
          }

          if (!driveMotorsL(abs(J1stepDif), abs(J2stepDif), abs(J3stepDif), abs(J4stepDif), abs(J5stepDif), abs(J6stepDif), abs(J7stepDif), abs(J8stepDif), abs(J9stepDif), J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, J7dir, J8dir, J9dir, curDelay, &motionModes)) {
            KinematicError = 1;
            break;
          }
          updatePos();
          rndSpeed = curDelay;
        }
      }

      if (KinematicError == 1) {
        Alarm = "ER";
        if (splineTrue == false) {
          delay(5);
          Serial.println(Alarm);
        }
      } else if (TotalAxisFault != 0) {
        if (splineTrue == false) {
          delay(5);
          Serial.println(Alarm);
        }
      } else {
        checkEncoders();
      }
      if (
        splineTrue == false
        && KinematicError == 0
        && TotalAxisFault == 0
      ) {
        sendRobotPos();
      }
      inData = "";  // Clear recieved buffer
      ////////MOVE COMPLETE///////////
    }




    //----- MOVE J ---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "MJ") {
      const ar4_protocol::MotionCommandStatus status =
        moveJ(inData, true, false, false);
      if (ar4_protocol::should_emit_generic_motion_error(status)) {
        Serial.println("ER");
      }
    }

    //----- MOVE G ---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "MG") {
      const ar4_protocol::MotionCommandStatus status =
        moveJ(inData, true, false, true);
      if (ar4_protocol::should_emit_generic_motion_error(status)) {
        Serial.println("ER");
      }
    }


    //----- DELETE PROG FROM SD CARD ---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "DG") {
      int fileStart = inData.indexOf("Fn");
      if (
        fileStart != 0
        || !ar4_protocol::valid_controller_filename(
          inData,
          fileStart + 2,
          static_cast<int>(inData.length())
        )
      ) {
        Serial.println("ER");
        consume_current_command();
        return;
      }
      if (!initSD()) {
        consume_current_command();
        return;
      }
      String filename = inData.substring(fileStart + 2);
      const char *fn = filename.c_str();
      if (SD.exists(fn)) {
        if (deleteSD(filename)) Serial.println("P");
      } else {
        Serial.println("F");
      }
    }

    //----- READ FILES FROM SD CARD ---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "RG") {
      File root;
      if (!initSD()) {
        consume_current_command();
        return;
      }
      root = SD.open("/");
      if (!root) {
        Serial.println(egSD("open root fail"));
        consume_current_command();
        return;
      }
      printDirectory(root, 0);
      root.close();
    }


    //----- WRITE COMMAND TO SD CARD ---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "WC") {
      int fileStart = inData.indexOf("Fn");
      if (
        fileStart <= 0
        || !ar4_protocol::valid_controller_filename(
          inData,
          fileStart + 2,
          static_cast<int>(inData.length())
        )
      ) {
        Serial.println("ER");
        consume_current_command();
        return;
      }
      String filename = inData.substring(fileStart + 2);
      String info = inData.substring(0, fileStart);
      ar4_protocol::CartesianMoveCommandFields stored_fields = {};
      int stored_external_steps[3] = {};
      if (
        !ar4_protocol::parse_cartesian_move_command(info, stored_fields)
        || !external_positions_to_future_steps(
          stored_fields.auxiliary[0],
          stored_fields.auxiliary[1],
          stored_fields.auxiliary[2],
          stored_external_steps
        )
      ) {
        Serial.println("ER");
        consume_current_command();
        return;
      }
      if (writeSD(filename, info)) sendRobotPos();
    }

    //----- PLAY FILE ON SD CARD ---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "PG") {
      File gcFile;
      String Cmd;
      String storedRow;
      int fileStart = inData.indexOf("Fn");
      if (
        fileStart != 0
        || !ar4_protocol::valid_controller_filename(
          inData,
          fileStart + 2,
          static_cast<int>(inData.length())
        )
      ) {
        Serial.println("ER");
        consume_current_command();
        return;
      }
      if (!initSD()) {
        consume_current_command();
        return;
      }
      String filename = inData.substring(fileStart + 2);
      const char *fn = filename.c_str();
      gcFile = SD.open(fn);
      if (!gcFile) {
        Serial.println("EG");
        consume_current_command();
        return;
      }
      while (gcFile.available() && estopActive == false) {
        if (
          read_stored_command_row(gcFile, storedRow)
              != ar4_protocol::StoredRowReadStatus::kComplete
          || !ar4_protocol::extract_stored_command_payload(storedRow, Cmd)
        ) {
          Serial.println("ER");
          gcFile.close();
          consume_current_command();
          return;
        }
        //CARTESIAN CMD
        if (Cmd.substring(0, 1) == "X") {
          updatePos();
          const ar4_protocol::MotionCommandStatus status =
            moveJ(Cmd, false, false, true);
          if (!ar4_protocol::should_continue_stored_playback(status)) {
            if (ar4_protocol::should_emit_generic_motion_error(status)) {
              Serial.println("ER");
            }
            gcFile.close();
            consume_current_command();
            return;
          }
        }
        //PRECALC'D CMD - not currently used, needs position handling
        else {
          int i1 = Cmd.indexOf(',');
          int i2 = Cmd.indexOf(',', i1 + 1);
          int i3 = Cmd.indexOf(',', i2 + 1);
          int i4 = Cmd.indexOf(',', i3 + 1);
          int i5 = Cmd.indexOf(',', i4 + 1);
          int i6 = Cmd.indexOf(',', i5 + 1);
          int i7 = Cmd.indexOf(',', i6 + 1);
          int i8 = Cmd.indexOf(',', i7 + 1);
          int i9 = Cmd.indexOf(',', i8 + 1);
          int i10 = Cmd.indexOf(',', i9 + 1);
          int i11 = Cmd.indexOf(',', i10 + 1);
          int i12 = Cmd.indexOf(',', i11 + 1);
          int i13 = Cmd.indexOf(',', i12 + 1);
          int i14 = Cmd.indexOf(',', i13 + 1);
          int i15 = Cmd.indexOf(',', i14 + 1);
          int i16 = Cmd.indexOf(',', i15 + 1);
          int i17 = Cmd.indexOf(',', i16 + 1);
          int i18 = Cmd.indexOf(',', i17 + 1);
          int i19 = Cmd.indexOf(',', i18 + 1);
          int i20 = Cmd.indexOf(',', i19 + 1);
          int i21 = Cmd.indexOf(',', i20 + 1);
          int i22 = Cmd.indexOf(',', i21 + 1);
          const int markers[] = {
            i1,
            i2,
            i3,
            i4,
            i5,
            i6,
            i7,
            i8,
            i9,
            i10,
            i11,
            i12,
            i13,
            i14,
            i15,
            i16,
            i17,
            i18,
            i19,
            i20,
            i21,
            i22,
          };
          const int intBegins[] = {
            0,
            i1 + 1,
            i2 + 1,
            i3 + 1,
            i4 + 1,
            i5 + 1,
            i6 + 1,
            i7 + 1,
            i8 + 1,
            i9 + 1,
            i10 + 1,
            i11 + 1,
            i12 + 1,
            i13 + 1,
            i14 + 1,
            i15 + 1,
            i16 + 1,
            i17 + 1,
          };
          const int intEnds[] = {
            i1,
            i2,
            i3,
            i4,
            i5,
            i6,
            i7,
            i8,
            i9,
            i10,
            i11,
            i12,
            i13,
            i14,
            i15,
            i16,
            i17,
            i18,
          };
          const int floatBegins[] = {
            i19 + 1,
            i20 + 1,
            i21 + 1,
            i22 + 1,
          };
          const int floatEnds[] = {
            i20,
            i21,
            i22,
            static_cast<int>(Cmd.length()),
          };
          int intFields[18];
          float floatFields[4];
          if (
            !ar4_protocol::marker_positions_are_ordered(Cmd.length(), markers)
            || Cmd.indexOf(',', i22 + 1) != -1
            || i19 - i18 != 2
            || !parse_int_spans(Cmd, intBegins, intEnds, intFields)
            || !ar4_protocol::values_are_binary(intFields + 9, 9)
            || !parse_float_spans(Cmd, floatBegins, floatEnds, floatFields)
          ) {
            Serial.println("ER");
            gcFile.close();
            consume_current_command();
            return;
          }
          int J1step = intFields[0];
          int J2step = intFields[1];
          int J3step = intFields[2];
          int J4step = intFields[3];
          int J5step = intFields[4];
          int J6step = intFields[5];
          int J7step = intFields[6];
          int J8step = intFields[7];
          int J9step = intFields[8];
          int J1dir = intFields[9];
          int J2dir = intFields[10];
          int J3dir = intFields[11];
          int J4dir = intFields[12];
          int J5dir = intFields[13];
          int J6dir = intFields[14];
          int J7dir = intFields[15];
          int J8dir = intFields[16];
          int J9dir = intFields[17];
          String SpeedType = Cmd.substring(i18 + 1, i19);
          float SpeedVal = floatFields[0];
          float ACCspd = floatFields[1];
          float DCCspd = floatFields[2];
          float ACCramp = floatFields[3];
          const char speed_mode = SpeedType.charAt(0);
          const int step_counts[numJoints] = {
            J1step, J2step, J3step, J4step, J5step,
            J6step, J7step, J8step, J9step,
          };
          const int directions[numJoints] = {
            J1dir, J2dir, J3dir, J4dir, J5dir,
            J6dir, J7dir, J8dir, J9dir,
          };
          const int current_steps[numJoints] = {
            J1StepM, J2StepM, J3StepM, J4StepM, J5StepM,
            J6StepM, J7StepM, J8StepM, J9StepM,
          };
          const int step_limits[numJoints] = {
            J1StepLim, J2StepLim, J3StepLim, J4StepLim, J5StepLim,
            J6StepLim, J7StepLim, J8StepLim, J9StepLim,
          };
          int future_steps[numJoints] = {};
          bool stored_move_is_valid = ar4_protocol::valid_motion_profile(
            speed_mode,
            SpeedVal,
            ACCspd,
            DCCspd,
            ACCramp
          );
          for (int axis = 0; axis < numJoints && stored_move_is_valid; ++axis) {
            stored_move_is_valid = ar4_protocol::stored_step_target(
              current_steps[axis],
              step_counts[axis],
              directions[axis],
              step_limits[axis],
              future_steps[axis]
            );
          }
          if (!stored_move_is_valid) {
            Serial.println("ER");
            gcFile.close();
            consume_current_command();
            return;
          }
          if (!driveMotorsG(J1step, J2step, J3step, J4step, J5step, J6step, J7step, J8step, J9step, J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, J7dir, J8dir, J9dir, SpeedType, SpeedVal, ACCspd, DCCspd, ACCramp, nullptr)) {
            Serial.println("ER");
            gcFile.close();
            consume_current_command();
            return;
          }
        }
      }
      gcFile.close();
      sendRobotPos();
    }



    //----- WRITE PRE-CALC'D MOVE TO SD CARD ---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "WG") {
      int J1dir;
      int J2dir;
      int J3dir;
      int J4dir;
      int J5dir;
      int J6dir;
      int J7dir;
      int J8dir;
      int J9dir;

      int J1axisFault = 0;
      int J2axisFault = 0;
      int J3axisFault = 0;
      int J4axisFault = 0;
      int J5axisFault = 0;
      int J6axisFault = 0;
      int J7axisFault = 0;
      int J8axisFault = 0;
      int J9axisFault = 0;
      int TotalAxisFault = 0;

      String info;

      int fileStart = inData.indexOf("Fn");
      if (
        fileStart <= 0
        || !ar4_protocol::valid_controller_filename(
          inData,
          fileStart + 2,
          static_cast<int>(inData.length())
        )
      ) {
        Serial.println("ER");
        consume_current_command();
        return;
      }
      String motionInfo = inData.substring(0, fileStart);
      ar4_protocol::CartesianMoveCommandFields commandFields = {};
      if (!ar4_protocol::parse_cartesian_move_command(
          motionInfo,
          commandFields
      )) {
        Serial.println("ER");
        consume_current_command();
        return;
      }
      String filename = inData.substring(fileStart + 2);
      for (int axis = 0; axis < ROBOT_nDOFs; ++axis) {
        xyzuvw_In[axis] = commandFields.pose[axis];
      }
      J7_In = commandFields.auxiliary[0];
      J8_In = commandFields.auxiliary[1];
      J9_In = commandFields.auxiliary[2];
      String SpeedType(commandFields.speed_mode);
      float SpeedVal = commandFields.speed;
      float ACCspd = commandFields.acceleration;
      float DCCspd = commandFields.deceleration;
      float ACCramp = commandFields.ramp;
      ar4_protocol::MotionModeTransaction<String, ROBOT_nDOFs> motionModes(
        WristCon,
        JointLoopModes,
        String(commandFields.wrist_config),
        commandFields.loop_modes
      );

      SolveInverseKinematics(commandFields.wrist_config);

      //calc destination motor steps
      int future_steps[numJoints] = {};
      if (
        KinematicError != 0
        || !inverse_solution_to_future_steps(
          J7_In,
          J8_In,
          J9_In,
          future_steps
        )
      ) {
        Serial.println("ER");
        consume_current_command();
        return;
      }
      int J1futStepM = future_steps[0];
      int J2futStepM = future_steps[1];
      int J3futStepM = future_steps[2];
      int J4futStepM = future_steps[3];
      int J5futStepM = future_steps[4];
      int J6futStepM = future_steps[5];
      int J7futStepM = future_steps[6];
      int J8futStepM = future_steps[7];
      int J9futStepM = future_steps[8];


      //calc delta from current to destination
      int J1stepDif = J1StepM - J1futStepM;
      int J2stepDif = J2StepM - J2futStepM;
      int J3stepDif = J3StepM - J3futStepM;
      int J4stepDif = J4StepM - J4futStepM;
      int J5stepDif = J5StepM - J5futStepM;
      int J6stepDif = J6StepM - J6futStepM;
      int J7stepDif = J7StepM - J7futStepM;
      int J8stepDif = J8StepM - J8futStepM;
      int J9stepDif = J9StepM - J9futStepM;

      //determine motor directions
      J1dir = (J1stepDif <= 0) ? 1 : 0;
      J2dir = (J2stepDif <= 0) ? 1 : 0;
      J3dir = (J3stepDif <= 0) ? 1 : 0;
      J4dir = (J4stepDif <= 0) ? 1 : 0;
      J5dir = (J5stepDif <= 0) ? 1 : 0;
      J6dir = (J6stepDif <= 0) ? 1 : 0;
      J7dir = (J7stepDif <= 0) ? 1 : 0;
      J8dir = (J8stepDif <= 0) ? 1 : 0;
      J9dir = (J9stepDif <= 0) ? 1 : 0;


      //determine if requested position is within axis limits
      if (future_step_is_outside_limit(J1futStepM, J1StepLim)) {
        J1axisFault = 1;
      }
      if (future_step_is_outside_limit(J2futStepM, J2StepLim)) {
        J2axisFault = 1;
      }
      if (future_step_is_outside_limit(J3futStepM, J3StepLim)) {
        J3axisFault = 1;
      }
      if (future_step_is_outside_limit(J4futStepM, J4StepLim)) {
        J4axisFault = 1;
      }
      if (future_step_is_outside_limit(J5futStepM, J5StepLim)) {
        J5axisFault = 1;
      }
      if (future_step_is_outside_limit(J6futStepM, J6StepLim)) {
        J6axisFault = 1;
      }
      if (future_step_is_outside_limit(J7futStepM, J7StepLim)) {
        J7axisFault = 1;
      }
      if (future_step_is_outside_limit(J8futStepM, J8StepLim)) {
        J8axisFault = 1;
      }
      if (future_step_is_outside_limit(J9futStepM, J9StepLim)) {
        J9axisFault = 1;
      }
      TotalAxisFault = J1axisFault + J2axisFault + J3axisFault + J4axisFault + J5axisFault + J6axisFault + J7axisFault + J8axisFault + J9axisFault;


      if (TotalAxisFault == 0 && KinematicError == 0) {
        info = String(abs(J1stepDif)) + "," + String(abs(J2stepDif)) + "," + String(abs(J3stepDif)) + "," + String(abs(J4stepDif)) + "," + String(abs(J5stepDif)) + "," + String(abs(J6stepDif)) + "," + String(abs(J7stepDif)) + "," + String(abs(J8stepDif)) + "," + String(abs(J9stepDif)) + "," + String(J1dir) + "," + String(J2dir) + "," + String(J3dir) + "," + String(J4dir) + "," + String(J5dir) + "," + String(J6dir) + "," + String(J7dir) + "," + String(J8dir) + "," + String(J9dir) + "," + String(SpeedType) + "," + String(SpeedVal) + "," + String(ACCspd) + "," + String(DCCspd) + "," + String(ACCramp);
        if (writeSD(filename, info)) {
          J1StepM = J1futStepM;
          J2StepM = J2futStepM;
          J3StepM = J3futStepM;
          J4StepM = J4futStepM;
          J5StepM = J5futStepM;
          J6StepM = J6futStepM;
          J7StepM = J7futStepM;
          J8StepM = J8futStepM;
          J9StepM = J9futStepM;
          sendRobotPos();
        }
      } else if (KinematicError == 1) {
        Alarm = "ER";
        delay(5);
        Serial.println(Alarm);
        Alarm = "0";
      } else {
        Alarm = "EL" + String(J1axisFault) + String(J2axisFault) + String(J3axisFault) + String(J4axisFault) + String(J5axisFault) + String(J6axisFault) + String(J7axisFault) + String(J8axisFault) + String(J9axisFault);
        delay(5);
        Serial.println(Alarm);
        Alarm = "0";
      }

      inData = "";  // Clear recieved buffer
      ////////MOVE COMPLETE///////////
    }



    //----- MOVE C (Cirlce) ---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "MC") {

      int J1dir;
      int J2dir;
      int J3dir;
      int J4dir;
      int J5dir;
      int J6dir;
      int J7dir;
      int J8dir;
      int J9dir;

      int J1axisFault = 0;
      int J2axisFault = 0;
      int J3axisFault = 0;
      int J4axisFault = 0;
      int J5axisFault = 0;
      int J6axisFault = 0;
      int TotalAxisFault = 0;

      String Alarm = "0";
      float curWayDis;
      float speedSP;
      float Xvect;
      float Yvect;
      float Zvect;
      float calcStepGap;
      float theta;
      int Cdir;
      float axis[3];
      float axisTemp[3];
      float startVect[3];
      float Rotation[3][3];
      float DestPt[3];
      float a;
      float b;
      float c;
      float d;
      float aa;
      float bb;
      float cc;
      float dd;
      float bc;
      float ad;
      float ac;
      float ab;
      float bd;
      float cd;

      int xStart = inData.indexOf("Cx");
      int yStart = inData.indexOf("Cy");
      int zStart = inData.indexOf("Cz");
      int rzStart = inData.indexOf("Rz");
      int ryStart = inData.indexOf("Ry");
      int rxStart = inData.indexOf("Rx");
      int xMidIndex = inData.indexOf("Bx");
      int yMidIndex = inData.indexOf("By");
      int zMidIndex = inData.indexOf("Bz");
      int xEndIndex = inData.indexOf("Px");
      int yEndIndex = inData.indexOf("Py");
      int zEndIndex = inData.indexOf("Pz");
      int tStart = inData.indexOf("Tr");
      int SPstart = inData.indexOf("S");
      int AcStart = inData.indexOf("Ac");
      int DcStart = inData.indexOf("Dc");
      int RmStart = inData.indexOf("Rm");
      int WristConStart = inData.indexOf("W");
      int LoopModeStart = inData.indexOf("Lm");

      const int markers[] = {
        xStart,
        yStart,
        zStart,
        rzStart,
        ryStart,
        rxStart,
        xMidIndex,
        yMidIndex,
        zMidIndex,
        xEndIndex,
        yEndIndex,
        zEndIndex,
        tStart,
        SPstart,
        AcStart,
        DcStart,
        RmStart,
        WristConStart,
        LoopModeStart,
      };
      const int begins[] = {
        xStart + 2,
        yStart + 2,
        zStart + 2,
        rzStart + 2,
        ryStart + 2,
        rxStart + 2,
        xMidIndex + 2,
        yMidIndex + 2,
        zMidIndex + 2,
        xEndIndex + 2,
        yEndIndex + 2,
        zEndIndex + 2,
        tStart + 2,
        SPstart + 2,
        AcStart + 2,
        DcStart + 2,
        RmStart + 2,
      };
      const int ends[] = {
        yStart,
        zStart,
        rzStart,
        ryStart,
        rxStart,
        xMidIndex,
        yMidIndex,
        zMidIndex,
        xEndIndex,
        yEndIndex,
        zEndIndex,
        tStart,
        SPstart,
        AcStart,
        DcStart,
        RmStart,
        WristConStart,
      };
      float parsed[17];
      int loopModes[ROBOT_nDOFs];
      const char speed_mode = inData.charAt(SPstart + 1);
      const char wrist_config = inData.charAt(WristConStart + 1);
      if (
        !ar4_protocol::marker_positions_are_ordered_from(
          inData.length(),
          markers,
          0
        )
        || WristConStart + 2 != LoopModeStart
        || LoopModeStart + 2 + ROBOT_nDOFs
          != static_cast<int>(inData.length())
        || !parse_float_spans(inData, begins, ends, parsed)
        || !ar4_protocol::supported_trajectory_rotation(parsed[12])
        || !parse_loop_modes(inData, LoopModeStart, loopModes)
        || !ar4_protocol::valid_motion_profile(
          speed_mode,
          parsed[13],
          parsed[14],
          parsed[15],
          parsed[16]
        )
        || !ar4_protocol::valid_wrist_config(wrist_config)
      ) {
        Serial.println("ER");
        consume_current_command();
        return;
      }

      float xBeg = parsed[0];
      float yBeg = parsed[1];
      float zBeg = parsed[2];
      float rzBeg = parsed[3];
      float ryBeg = parsed[4];
      float rxBeg = parsed[5];
      float xMid = parsed[6];
      float yMid = parsed[7];
      float zMid = parsed[8];
      float xEnd = parsed[9];
      float yEnd = parsed[10];
      float zEnd = parsed[11];
      String SpeedType(speed_mode);
      float SpeedVal = parsed[13];
      float ACCspd = parsed[14];
      float DCCspd = parsed[15];
      float ACCramp = parsed[16];
      ar4_protocol::MotionModeTransaction<String, ROBOT_nDOFs> motionModes(
        WristCon,
        JointLoopModes,
        String(wrist_config),
        loopModes
      );

      const float circle_center[3] = { xBeg, yBeg, zBeg };
      const float circle_start[3] = { xMid, yMid, zMid };
      const float circle_end[3] = { xEnd, yEnd, zEnd };
      if (!ar4_protocol::valid_circle_geometry(
          circle_center,
          circle_start,
          circle_end,
          linWayDistSP
      )) {
        Serial.println("ER");
        consume_current_command();
        return;
      }

      //calc vector from start point of circle (mid) to center of circle (beg)
      Xvect = xMid - xBeg;
      Yvect = yMid - yBeg;
      Zvect = zMid - zBeg;
      //get radius - distance from first point (center of circle) to second point (start point of circle)
      float Radius = pow((pow((Xvect), 2) + pow((Yvect), 2) + pow((Zvect), 2)), .5);

      //set center coordinates of circle to first point (beg) as this is the center of our circle
      float Px = xBeg;
      float Py = yBeg;
      float Pz = zBeg;

      //define start vetor (mid) point is start of circle
      startVect[0] = (xMid - Px);
      startVect[1] = (yMid - Py);
      startVect[2] = (zMid - Pz);
      //get vectors from center of circle to  mid target (start) and end target then normalize
      float vect_Bmag = pow((pow((xMid - Px), 2) + pow((yMid - Py), 2) + pow((zMid - Pz), 2)), .5);
      float vect_Bx = (xMid - Px) / vect_Bmag;
      float vect_By = (yMid - Py) / vect_Bmag;
      float vect_Bz = (zMid - Pz) / vect_Bmag;
      float vect_Cmag = pow((pow((xEnd - Px), 2) + pow((yEnd - Py), 2) + pow((zEnd - Pz), 2)), .5);
      float vect_Cx = (xEnd - Px) / vect_Cmag;
      float vect_Cy = (yEnd - Py) / vect_Cmag;
      float vect_Cz = (zEnd - Pz) / vect_Cmag;
      //get cross product of vectors b & c than apply to axis matrix
      float CrossX = (vect_By * vect_Cz) - (vect_Bz * vect_Cy);
      float CrossY = (vect_Bz * vect_Cx) - (vect_Bx * vect_Cz);
      float CrossZ = (vect_Bx * vect_Cy) - (vect_By * vect_Cx);
      axis[0] = CrossX / sqrt((CrossX * CrossX) + (CrossY * CrossY) + (CrossZ * CrossZ));
      axis[1] = CrossY / sqrt((CrossX * CrossX) + (CrossY * CrossY) + (CrossZ * CrossZ));
      axis[2] = CrossZ / sqrt((CrossX * CrossX) + (CrossY * CrossY) + (CrossZ * CrossZ));
      //get radian angle between vectors using acos of dot product
      float circle_dot = (
        vect_Bx * vect_Cx + vect_By * vect_Cy + vect_Bz * vect_Cz
      ) / (
        sqrt(pow(vect_Bx, 2) + pow(vect_By, 2) + pow(vect_Bz, 2))
        * sqrt(pow(vect_Cx, 2) + pow(vect_Cy, 2) + pow(vect_Cz, 2))
      );
      circle_dot = fmax(-1.0f, fmin(1.0f, circle_dot));
      float BCradians = acos(circle_dot);
      //get arc degree
      float ABdegrees = degrees(BCradians);
      //get direction from angle
      if (ABdegrees > 0) {
        Cdir = 1;
      } else {
        Cdir = -1;
      }

      //get circumference and calc way pt gap
      float lineDist = 2 * 3.14159265359 * Radius;
      int waypoint_count = 0;
      int HighStep = 0;
      if (
        !ar4_protocol::waypoint_count_for_path(
          lineDist,
          linWayDistSP,
          waypoint_count
        )
        || !ar4_protocol::waypoint_count_for_path(
          lineDist,
          0.05f,
          HighStep
        )
      ) {
        Serial.println("ER");
        consume_current_command();
        return;
      }
      float wayPts = static_cast<float>(waypoint_count);

      float wayPerc = 1.0f / wayPts;
      //cacl way pt angle
      float theta_Deg = ((360 * Cdir) / (wayPts));

      //determine steps
      float ACCStep = HighStep * (ACCspd / 100);
      float NORStep = HighStep * ((100 - ACCspd - DCCspd) / 100);
      float DCCStep = HighStep * (DCCspd / 100);

      //set speed for seconds or mm per sec
      if (SpeedType == "s") {
        speedSP = (SpeedVal * 1000000) * 1.75;
      } else if (SpeedType == "m") {
        speedSP = ((lineDist / SpeedVal) * 1000000) * 1.75;
      }
      if (
        (SpeedType == "s" || SpeedType == "m")
        && (!isfinite(speedSP) || speedSP <= 0.0f)
      ) {
        Serial.println("ER");
        consume_current_command();
        return;
      }

      //calc step gap for seconds or mm per sec
      if (SpeedType == "s" or SpeedType == "m") {
        float zeroStepGap = speedSP / HighStep;
        float zeroACCstepInc = (zeroStepGap * (100 / ACCramp)) / ACCStep;
        float zeroACCtime = ((ACCStep)*zeroStepGap) + ((ACCStep - 9) * (((ACCStep) * (zeroACCstepInc / 2))));
        float zeroNORtime = NORStep * zeroStepGap;
        float zeroDCCstepInc = (zeroStepGap * (100 / ACCramp)) / DCCStep;
        float zeroDCCtime = ((DCCStep)*zeroStepGap) + ((DCCStep - 9) * (((DCCStep) * (zeroDCCstepInc / 2))));
        float zeroTOTtime = zeroACCtime + zeroNORtime + zeroDCCtime;
        if (!isfinite(zeroTOTtime) || zeroTOTtime <= 0.0f) {
          calcStepGap = minSpeedDelay;
          speedViolation = "1";
        } else {
          float overclockPerc = speedSP / zeroTOTtime;
          calcStepGap = zeroStepGap * overclockPerc;
        }
        if (calcStepGap <= minSpeedDelay) {
          calcStepGap = minSpeedDelay;
          speedViolation = "1";
        }
      }

      //calc step gap for percentage
      else if (SpeedType == "p") {
        calcStepGap = minSpeedDelay / (SpeedVal / 100);
      }
      if (!isfinite(calcStepGap) || calcStepGap <= 0.0f) {
        Serial.println("ER");
        consume_current_command();
        return;
      }

      //calculate final step increments
      float calcACCstepInc = (calcStepGap * (100 / ACCramp)) / ACCStep;
      float calcDCCstepInc = (calcStepGap * (100 / ACCramp)) / DCCStep;
      float calcACCstartDel = (calcACCstepInc * ACCStep) * 2;
      float calcDCCendDel = (calcDCCstepInc * DCCStep) * 2;


      //calc way pt speeds
      float ACCwayPts = wayPts * (ACCspd / 100);
      float NORwayPts = wayPts * ((100 - ACCspd - DCCspd) / 100);
      float DCCwayPts = wayPts * (DCCspd / 100);

      //calc way inc for lin way steps
      float ACCwayInc = (calcACCstartDel - calcStepGap) / ACCwayPts;
      float DCCwayInc = (calcDCCendDel - calcStepGap) / DCCwayPts;
      if (
        !isfinite(ACCwayInc)
        || !isfinite(DCCwayInc)
        || !ar4_protocol::valid_delay_envelope(
          calcStepGap,
          calcACCstartDel,
          calcDCCendDel,
          false,
          rndSpeed
        )
      ) {
        Serial.println("ER");
        consume_current_command();
        return;
      }

      //set starting delsy
      float curDelay = calcACCstartDel;

      //set starting angle first way pt
      float cur_deg = theta_Deg;

      /////////////////////////////////////
      //loop through waypoints
      ////////////////////////////////////

      resetEncoders();

      for (int i = 1; i <= waypoint_count; i++) {

        theta = radians(cur_deg);
        //use euler rodrigues formula to find rotation vector
        a = cos(theta / 2.0);
        b = -axis[0] * sin(theta / 2.0);
        c = -axis[1] * sin(theta / 2.0);
        d = -axis[2] * sin(theta / 2.0);
        aa = a * a;
        bb = b * b;
        cc = c * c;
        dd = d * d;
        bc = b * c;
        ad = a * d;
        ac = a * c;
        ab = a * b;
        bd = b * d;
        cd = c * d;
        Rotation[0][0] = aa + bb - cc - dd;
        Rotation[0][1] = 2 * (bc + ad);
        Rotation[0][2] = 2 * (bd - ac);
        Rotation[1][0] = 2 * (bc - ad);
        Rotation[1][1] = aa + cc - bb - dd;
        Rotation[1][2] = 2 * (cd + ab);
        Rotation[2][0] = 2 * (bd + ac);
        Rotation[2][1] = 2 * (cd - ab);
        Rotation[2][2] = aa + dd - bb - cc;

        //get product of current rotation and start vector
        DestPt[0] = (Rotation[0][0] * startVect[0]) + (Rotation[0][1] * startVect[1]) + (Rotation[0][2] * startVect[2]);
        DestPt[1] = (Rotation[1][0] * startVect[0]) + (Rotation[1][1] * startVect[1]) + (Rotation[1][2] * startVect[2]);
        DestPt[2] = (Rotation[2][0] * startVect[0]) + (Rotation[2][1] * startVect[1]) + (Rotation[2][2] * startVect[2]);

        ////DELAY CALC/////
        if (i <= ACCwayPts) {
          curDelay = fmax(calcStepGap, curDelay - ACCwayInc);
        } else if (i >= (wayPts - DCCwayPts)) {
          curDelay = fmin(calcDCCendDel, curDelay + DCCwayInc);
        } else {
          curDelay = calcStepGap;
        }

        //shift way pts back to orignal origin and calc kinematics for way pt movement
        xyzuvw_In[0] = (DestPt[0]) + Px;
        xyzuvw_In[1] = (DestPt[1]) + Py;
        xyzuvw_In[2] = (DestPt[2]) + Pz;
        xyzuvw_In[3] = rzBeg;
        xyzuvw_In[4] = ryBeg;
        xyzuvw_In[5] = rxBeg;

        SolveInverseKinematics(wrist_config);

        //calc destination motor steps
        int future_steps[ROBOT_nDOFs] = {};
        if (
          KinematicError != 0
          || !primary_inverse_solution_to_future_steps(future_steps)
        ) {
          KinematicError = 1;
          break;
        }
        int J1futStepM = future_steps[0];
        int J2futStepM = future_steps[1];
        int J3futStepM = future_steps[2];
        int J4futStepM = future_steps[3];
        int J5futStepM = future_steps[4];
        int J6futStepM = future_steps[5];

        //calc delta from current to destination
        int J1stepDif = J1StepM - J1futStepM;
        int J2stepDif = J2StepM - J2futStepM;
        int J3stepDif = J3StepM - J3futStepM;
        int J4stepDif = J4StepM - J4futStepM;
        int J5stepDif = J5StepM - J5futStepM;
        int J6stepDif = J6StepM - J6futStepM;
        int J7stepDif = 0;
        int J8stepDif = 0;
        int J9stepDif = 0;

        //determine motor directions
        J1dir = (J1stepDif <= 0) ? 1 : 0;
        J2dir = (J2stepDif <= 0) ? 1 : 0;
        J3dir = (J3stepDif <= 0) ? 1 : 0;
        J4dir = (J4stepDif <= 0) ? 1 : 0;
        J5dir = (J5stepDif <= 0) ? 1 : 0;
        J6dir = (J6stepDif <= 0) ? 1 : 0;
        J7dir = 0;
        J8dir = 0;
        J9dir = 0;

        //determine if requested position is within axis limits
        if (future_step_is_outside_limit(J1futStepM, J1StepLim)) {
          J1axisFault = 1;
        }
        if (future_step_is_outside_limit(J2futStepM, J2StepLim)) {
          J2axisFault = 1;
        }
        if (future_step_is_outside_limit(J3futStepM, J3StepLim)) {
          J3axisFault = 1;
        }
        if (future_step_is_outside_limit(J4futStepM, J4StepLim)) {
          J4axisFault = 1;
        }
        if (future_step_is_outside_limit(J5futStepM, J5StepLim)) {
          J5axisFault = 1;
        }
        if (future_step_is_outside_limit(J6futStepM, J6StepLim)) {
          J6axisFault = 1;
        }
        TotalAxisFault = J1axisFault + J2axisFault + J3axisFault + J4axisFault + J5axisFault + J6axisFault;

        if (TotalAxisFault != 0) {
          Alarm = "EL" + String(J1axisFault) + String(J2axisFault) + String(J3axisFault) + String(J4axisFault) + String(J5axisFault) + String(J6axisFault);
          break;
        }

        if (!driveMotorsL(abs(J1stepDif), abs(J2stepDif), abs(J3stepDif), abs(J4stepDif), abs(J5stepDif), abs(J6stepDif), abs(J7stepDif), abs(J8stepDif), abs(J9stepDif), J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, J7dir, J8dir, J9dir, curDelay, &motionModes)) {
          KinematicError = 1;
          break;
        }
        updatePos();

        //increment angle
        cur_deg += theta_Deg;
      }

      if (KinematicError == 1) {
        Alarm = "ER";
        delay(5);
        Serial.println(Alarm);
      } else if (TotalAxisFault != 0) {
        delay(5);
        Serial.println(Alarm);
      } else {
        checkEncoders();
        sendRobotPos();
      }


      inData = "";  // Clear recieved buffer
      ////////MOVE COMPLETE///////////
    }




    //----- MOVE A (Arc) ---------------------------------------------------
    //-----------------------------------------------------------------------
    if (function == "MA" and flag == "") {

      if (rndTrue == true) {
        inData = rndData;
      }

      float curDelay;

      int J1dir;
      int J2dir;
      int J3dir;
      int J4dir;
      int J5dir;
      int J6dir;
      int J7dir;
      int J8dir;
      int J9dir;

      int J1axisFault = 0;
      int J2axisFault = 0;
      int J3axisFault = 0;
      int J4axisFault = 0;
      int J5axisFault = 0;
      int J6axisFault = 0;
      int TotalAxisFault = 0;

      //String Alarm = "0";
      float curWayDis;
      float speedSP;
      float Xvect;
      float Yvect;
      float Zvect;
      float calcStepGap;
      float theta;
      float axis[3];
      float axisTemp[3];
      float startVect[3];
      float Rotation[3][3];
      float DestPt[3];
      float a;
      float b;
      float c;
      float d;
      float aa;
      float bb;
      float cc;
      float dd;
      float bc;
      float ad;
      float ac;
      float ab;
      float bd;
      float cd;

      int xMidIndex = inData.indexOf("X");
      int yMidIndex = inData.indexOf("Y");
      int zMidIndex = inData.indexOf("Z");
      int rzIndex = inData.indexOf("Rz");
      int ryIndex = inData.indexOf("Ry");
      int rxIndex = inData.indexOf("Rx");

      int xEndIndex = inData.indexOf("Ex");
      int yEndIndex = inData.indexOf("Ey");
      int zEndIndex = inData.indexOf("Ez");
      int tStart = inData.indexOf("Tr");
      int SPstart = inData.indexOf("S");
      int AcStart = inData.indexOf("Ac");
      int DcStart = inData.indexOf("Dc");
      int RmStart = inData.indexOf("Rm");
      int WristConStart = inData.indexOf("W");
      int LoopModeStart = inData.indexOf("Lm");

      const int markers[] = {
        xMidIndex,
        yMidIndex,
        zMidIndex,
        rzIndex,
        ryIndex,
        rxIndex,
        xEndIndex,
        yEndIndex,
        zEndIndex,
        tStart,
        SPstart,
        AcStart,
        DcStart,
        RmStart,
        WristConStart,
        LoopModeStart,
      };
      const int begins[] = {
        xMidIndex + 1,
        yMidIndex + 1,
        zMidIndex + 1,
        rzIndex + 2,
        ryIndex + 2,
        rxIndex + 2,
        xEndIndex + 2,
        yEndIndex + 2,
        zEndIndex + 2,
        tStart + 2,
        SPstart + 2,
        AcStart + 2,
        DcStart + 2,
        RmStart + 2,
      };
      const int ends[] = {
        yMidIndex,
        zMidIndex,
        rzIndex,
        ryIndex,
        rxIndex,
        xEndIndex,
        yEndIndex,
        zEndIndex,
        tStart,
        SPstart,
        AcStart,
        DcStart,
        RmStart,
        WristConStart,
      };
      float parsed[14];
      int loopModes[ROBOT_nDOFs];
      const char speed_mode = inData.charAt(SPstart + 1);
      const char wrist_config = inData.charAt(WristConStart + 1);
      if (
        !ar4_protocol::marker_positions_are_ordered_from(
          inData.length(),
          markers,
          0
        )
        || WristConStart + 2 != LoopModeStart
        || LoopModeStart + 2 + ROBOT_nDOFs
          != static_cast<int>(inData.length())
        || !parse_float_spans(inData, begins, ends, parsed)
        || !ar4_protocol::supported_trajectory_rotation(parsed[9])
        || !parse_loop_modes(inData, LoopModeStart, loopModes)
        || !ar4_protocol::valid_motion_profile(
          speed_mode,
          parsed[10],
          parsed[11],
          parsed[12],
          parsed[13]
        )
        || !ar4_protocol::valid_wrist_config(wrist_config)
      ) {
        Serial.println("ER");
        consume_current_command();
        return;
      }

      updatePos();

      float xBeg = xyzuvw_Out[0];
      float yBeg = xyzuvw_Out[1];
      float zBeg = xyzuvw_Out[2];
      float rzBeg = xyzuvw_Out[3];
      float ryBeg = xyzuvw_Out[4];
      float rxBeg = xyzuvw_Out[5];

      float xMid = parsed[0];
      float yMid = parsed[1];
      float zMid = parsed[2];
      float rz = parsed[3];
      float ry = parsed[4];
      float rx = parsed[5];


      float RZvect = rzBeg - rz;
      float RYvect = ryBeg - ry;
      float RXvect = rxBeg - rx;

      float xEnd = parsed[6];
      float yEnd = parsed[7];
      float zEnd = parsed[8];
      String SpeedType(speed_mode);
      float SpeedVal = parsed[10];
      float ACCspd = parsed[11];
      float DCCspd = parsed[12];
      float ACCramp = parsed[13];
      ar4_protocol::MotionModeTransaction<String, ROBOT_nDOFs> motionModes(
        WristCon,
        JointLoopModes,
        String(wrist_config),
        loopModes
      );

      const float arc_start[3] = { xBeg, yBeg, zBeg };
      const float arc_middle[3] = { xMid, yMid, zMid };
      const float arc_end[3] = { xEnd, yEnd, zEnd };
      ar4_protocol::OrderedArcGeometry arc_geometry = {};
      if (!ar4_protocol::valid_arc_geometry(
          arc_start,
          arc_middle,
          arc_end,
          linWayDistSP,
          &arc_geometry
      )) {
        Serial.println("ER");
        consume_current_command();
        return;
      }

      const float Px = static_cast<float>(arc_geometry.center[0]);
      const float Py = static_cast<float>(arc_geometry.center[1]);
      const float Pz = static_cast<float>(arc_geometry.center[2]);
      startVect[0] = xBeg - Px;
      startVect[1] = yBeg - Py;
      startVect[2] = zBeg - Pz;
      axis[0] = static_cast<float>(arc_geometry.axis[0]);
      axis[1] = static_cast<float>(arc_geometry.axis[1]);
      axis[2] = static_cast<float>(arc_geometry.axis[2]);
      const float ABdegrees = degrees(
        static_cast<float>(arc_geometry.radians)
      );
      const float lineDist = static_cast<float>(
        arc_geometry.radius * arc_geometry.radians
      );
      int waypoint_count = 0;
      int HighStep = 0;
      if (
        !ar4_protocol::waypoint_count_for_path(
          lineDist,
          linWayDistSP,
          waypoint_count
        )
        || !ar4_protocol::waypoint_count_for_path(
          lineDist,
          0.05f,
          HighStep
        )
      ) {
        Serial.println("ER");
        consume_current_command();
        return;
      }
      float wayPts = static_cast<float>(waypoint_count);

      float wayPerc = 1.0f / wayPts;
      //cacl way pt angle
      float theta_Deg = (ABdegrees / wayPts);

      //determine steps
      float ACCStep = HighStep * (ACCspd / 100);
      float NORStep = HighStep * ((100 - ACCspd - DCCspd) / 100);
      float DCCStep = HighStep * (DCCspd / 100);

      //set speed for seconds or mm per sec
      if (SpeedType == "s") {
        speedSP = (SpeedVal * 1000000) * 1.2;
      } else if (SpeedType == "m") {
        speedSP = ((lineDist / SpeedVal) * 1000000) * 1.2;
      }
      if (
        (SpeedType == "s" || SpeedType == "m")
        && (!isfinite(speedSP) || speedSP <= 0.0f)
      ) {
        Serial.println("ER");
        consume_current_command();
        return;
      }

      //calc step gap for seconds or mm per sec
      if (SpeedType == "s" or SpeedType == "m") {
        float zeroStepGap = speedSP / HighStep;
        float zeroACCstepInc = (zeroStepGap * (100 / ACCramp)) / ACCStep;
        float zeroACCtime = ((ACCStep)*zeroStepGap) + ((ACCStep - 9) * (((ACCStep) * (zeroACCstepInc / 2))));
        float zeroNORtime = NORStep * zeroStepGap;
        float zeroDCCstepInc = (zeroStepGap * (100 / ACCramp)) / DCCStep;
        float zeroDCCtime = ((DCCStep)*zeroStepGap) + ((DCCStep - 9) * (((DCCStep) * (zeroDCCstepInc / 2))));
        float zeroTOTtime = zeroACCtime + zeroNORtime + zeroDCCtime;
        if (!isfinite(zeroTOTtime) || zeroTOTtime <= 0.0f) {
          calcStepGap = minSpeedDelay;
          speedViolation = "1";
        } else {
          float overclockPerc = speedSP / zeroTOTtime;
          calcStepGap = zeroStepGap * overclockPerc;
        }
        if (calcStepGap <= minSpeedDelay) {
          calcStepGap = minSpeedDelay;
          speedViolation = "1";
        }
      }

      //calc step gap for percentage
      else if (SpeedType == "p") {
        calcStepGap = minSpeedDelay / (SpeedVal / 100);
      }
      if (!isfinite(calcStepGap) || calcStepGap <= 0.0f) {
        Serial.println("ER");
        consume_current_command();
        return;
      }

      //calculate final step increments
      float calcACCstepInc = (calcStepGap * (100 / ACCramp)) / ACCStep;
      float calcDCCstepInc = (calcStepGap * (100 / ACCramp)) / DCCStep;
      float calcACCstartDel = (calcACCstepInc * ACCStep) * 2;
      float calcDCCendDel = (calcDCCstepInc * DCCStep) * 2;


      //calc way pt speeds
      float ACCwayPts = wayPts * (ACCspd / 100);
      float NORwayPts = wayPts * ((100 - ACCspd - DCCspd) / 100);
      float DCCwayPts = wayPts * (DCCspd / 100);

      //calc way inc for lin way steps
      float ACCwayInc = (calcACCstartDel - calcStepGap) / ACCwayPts;
      float DCCwayInc = (calcDCCendDel - calcStepGap) / DCCwayPts;
      if (
        !isfinite(ACCwayInc)
        || !isfinite(DCCwayInc)
        || !ar4_protocol::valid_delay_envelope(
          calcStepGap,
          calcACCstartDel,
          calcDCCendDel,
          rndTrue,
          rndSpeed
        )
      ) {
        Serial.println("ER");
        consume_current_command();
        return;
      }

      //set starting delsy
      if (rndTrue == true) {
        curDelay = rndSpeed;
      } else {
        curDelay = calcACCstartDel;
      }


      //set starting angle first way pt
      float cur_deg = theta_Deg;

      /////////////////////////////////////
      //loop through waypoints
      ////////////////////////////////////


      ////debug values to sd card//////////////
      //SD.begin(BUILTIN_SDCARD);
      //String filename = "arc_debug";
      //const char *fn = filename.c_str();
      //if (SD.exists(fn)) {
      //  deleteSD(filename);
      //}

      resetEncoders();

      for (int i = 1; i <= waypoint_count; i++) {

        theta = radians(cur_deg);
        //use euler rodrigues formula to find rotation vector
        a = cos(theta / 2.0);
        b = -axis[0] * sin(theta / 2.0);
        c = -axis[1] * sin(theta / 2.0);
        d = -axis[2] * sin(theta / 2.0);
        aa = a * a;
        bb = b * b;
        cc = c * c;
        dd = d * d;
        bc = b * c;
        ad = a * d;
        ac = a * c;
        ab = a * b;
        bd = b * d;
        cd = c * d;
        Rotation[0][0] = aa + bb - cc - dd;
        Rotation[0][1] = 2 * (bc + ad);
        Rotation[0][2] = 2 * (bd - ac);
        Rotation[1][0] = 2 * (bc - ad);
        Rotation[1][1] = aa + cc - bb - dd;
        Rotation[1][2] = 2 * (cd + ab);
        Rotation[2][0] = 2 * (bd + ac);
        Rotation[2][1] = 2 * (cd - ab);
        Rotation[2][2] = aa + dd - bb - cc;

        //get product of current rotation and start vector
        DestPt[0] = (Rotation[0][0] * startVect[0]) + (Rotation[0][1] * startVect[1]) + (Rotation[0][2] * startVect[2]);
        DestPt[1] = (Rotation[1][0] * startVect[0]) + (Rotation[1][1] * startVect[1]) + (Rotation[1][2] * startVect[2]);
        DestPt[2] = (Rotation[2][0] * startVect[0]) + (Rotation[2][1] * startVect[1]) + (Rotation[2][2] * startVect[2]);

        ////DELAY CALC/////
        if (rndTrue == true) {
          curDelay = rndSpeed;
        } else if (i <= ACCwayPts) {
          curDelay = fmax(calcStepGap, curDelay - ACCwayInc);
        } else if (i >= (wayPts - DCCwayPts)) {
          curDelay = fmin(calcDCCendDel, curDelay + DCCwayInc);
        } else {
          curDelay = calcStepGap;
        }

        //shift way pts back to orignal origin and calc kinematics for way pt movement
        float curWayPerc = wayPerc * i;
        xyzuvw_In[0] = (DestPt[0]) + Px;
        xyzuvw_In[1] = (DestPt[1]) + Py;
        xyzuvw_In[2] = (DestPt[2]) + Pz;
        xyzuvw_In[3] = rzBeg - (RZvect * curWayPerc);
        xyzuvw_In[4] = ryBeg - (RYvect * curWayPerc);
        xyzuvw_In[5] = rxBeg - (RXvect * curWayPerc);


        SolveInverseKinematics(wrist_config);

        //calc destination motor steps
        int future_steps[ROBOT_nDOFs] = {};
        if (
          KinematicError != 0
          || !primary_inverse_solution_to_future_steps(future_steps)
        ) {
          KinematicError = 1;
          break;
        }
        int J1futStepM = future_steps[0];
        int J2futStepM = future_steps[1];
        int J3futStepM = future_steps[2];
        int J4futStepM = future_steps[3];
        int J5futStepM = future_steps[4];
        int J6futStepM = future_steps[5];

        //calc delta from current to destination
        int J1stepDif = J1StepM - J1futStepM;
        int J2stepDif = J2StepM - J2futStepM;
        int J3stepDif = J3StepM - J3futStepM;
        int J4stepDif = J4StepM - J4futStepM;
        int J5stepDif = J5StepM - J5futStepM;
        int J6stepDif = J6StepM - J6futStepM;
        int J7stepDif = 0;
        int J8stepDif = 0;
        int J9stepDif = 0;


        //determine motor directions
        J1dir = (J1stepDif <= 0) ? 1 : 0;
        J2dir = (J2stepDif <= 0) ? 1 : 0;
        J3dir = (J3stepDif <= 0) ? 1 : 0;
        J4dir = (J4stepDif <= 0) ? 1 : 0;
        J5dir = (J5stepDif <= 0) ? 1 : 0;
        J6dir = (J6stepDif <= 0) ? 1 : 0;
        J7dir = 0;
        J8dir = 0;
        J9dir = 0;

        //determine if requested position is within axis limits
        if (future_step_is_outside_limit(J1futStepM, J1StepLim)) {
          J1axisFault = 1;
        }
        if (future_step_is_outside_limit(J2futStepM, J2StepLim)) {
          J2axisFault = 1;
        }
        if (future_step_is_outside_limit(J3futStepM, J3StepLim)) {
          J3axisFault = 1;
        }
        if (future_step_is_outside_limit(J4futStepM, J4StepLim)) {
          J4axisFault = 1;
        }
        if (future_step_is_outside_limit(J5futStepM, J5StepLim)) {
          J5axisFault = 1;
        }
        if (future_step_is_outside_limit(J6futStepM, J6StepLim)) {
          J6axisFault = 1;
        }
        TotalAxisFault = J1axisFault + J2axisFault + J3axisFault + J4axisFault + J5axisFault + J6axisFault;

        if (TotalAxisFault != 0) {
          Alarm = "EL" + String(J1axisFault) + String(J2axisFault) + String(J3axisFault) + String(J4axisFault) + String(J5axisFault) + String(J6axisFault);
          break;
        }

        if (!driveMotorsL(abs(J1stepDif), abs(J2stepDif), abs(J3stepDif), abs(J4stepDif), abs(J5stepDif), abs(J6stepDif), abs(J7stepDif), abs(J8stepDif), abs(J9stepDif), J1dir, J2dir, J3dir, J4dir, J5dir, J6dir, J7dir, J8dir, J9dir, curDelay, &motionModes)) {
          KinematicError = 1;
          break;
        }
        updatePos();

        //increment angle
        cur_deg += theta_Deg;



        ///////////debug values to sd card
        //updatePos();
        //String cart_val = "Cartesian Value = " + String(xyzuvw_Out[0]) + " - " + String(xyzuvw_Out[1]) + " - " + String(xyzuvw_Out[2]) + " - " + String(xyzuvw_Out[3]) + " - " + String(xyzuvw_Out[4]) + " - " + String(xyzuvw_Out[5]);
        //String joint_val = "Joint Value = " + String(JangleIn[0]) + " - " + String(JangleIn[1]) + " - " + String(JangleIn[2]) + " - " + String(JangleIn[3]) + " - " + String(JangleIn[4]) + " - " + String(JangleIn[5]);
        //SD.begin(BUILTIN_SDCARD);
        //const char *fn = filename.c_str();
        //writeSD(fn, cart_val);
        //writeSD(fn, joint_val);
      }
      if (KinematicError == 1) {
        Alarm = "ER";
        if (splineTrue == false) {
          delay(5);
          Serial.println(Alarm);
        }
      } else if (TotalAxisFault != 0) {
        if (splineTrue == false) {
          delay(5);
          Serial.println(Alarm);
        }
      } else {
        checkEncoders();
      }
      rndTrue = false;
      inData = "";  // Clear recieved buffer
      if (
        splineTrue == false
        && KinematicError == 0
        && TotalAxisFault == 0
      ) {
        sendRobotPos();
      }
      ////////MOVE COMPLETE///////////
    }


    consume_current_command();
  }
}
