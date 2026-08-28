/*  AR4 Annin Robot Control Software Arduino Mega sketch
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
          Selling AR2 software, robots, robot parts, or any versions of robots or software based on this
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

    Log:

*/


#include <Servo.h>

#include "auxiliary_protocol_contract.h"

static const ar4_auxiliary::BoardProfile kBoardProfile =
  ar4_auxiliary::kMegaBoard;
static const uint8_t kServoCount = 7;
static const uint8_t kServoPins[kServoCount] = {
  A0,
  A1,
  A2,
  A3,
  A4,
  A5,
  A6,
};
static const uint8_t kInputPins[] = {
  2, 3, 4, 5, 6, 7,
  8, 9, 10, 11, 12, 13, 14, 15,
  16, 17, 18, 19, 20, 21, 22, 23,
  24, 25, 26, 27,
};
static const uint8_t kOutputPins[] = {
  28, 29, 30, 31, 32, 33, 34, 35,
  36, 37, 38, 39, 40, 41, 42, 43,
  44, 45, 46, 47, 48, 49, 50, 51,
  52, 53,
};

static const uint8_t kCurrentSensorPin = A7;
static const float kAnalogReferenceVolts = 5.0f;
static const float kCurrentSensorVoltsPerAmp = 0.185f;
static const uint16_t kCurrentZeroSampleCount = 200;
static const uint8_t kCurrentReadSampleCount = 20;

Servo servoChannels[kServoCount];
bool servoAttached[kServoCount] = {
  false,
  false,
  false,
  false,
  false,
  false,
  false,
};
ar4_auxiliary::FrameBuffer commandFrames;
int currentSensorZeroRaw = 512;
ar4_auxiliary::WaitState waitOperation = {false, 0, 0, 0, 0, 0};

void calibrateCurrentSensor() {
  long sampleTotal = 0;
  for (
    uint16_t sample = 0;
    sample < kCurrentZeroSampleCount;
    ++sample
  ) {
    sampleTotal += analogRead(kCurrentSensorPin);
    delay(2);
  }
  currentSensorZeroRaw = (
    sampleTotal / static_cast<long>(kCurrentZeroSampleCount)
  );
}

float readCurrentAmps() {
  long sampleTotal = 0;
  for (
    uint8_t sample = 0;
    sample < kCurrentReadSampleCount;
    ++sample
  ) {
    sampleTotal += analogRead(kCurrentSensorPin);
  }
  const float raw = (
    static_cast<float>(sampleTotal)
    / static_cast<float>(kCurrentReadSampleCount)
  );
  const float volts = raw * kAnalogReferenceVolts / 1023.0f;
  const float zeroVolts = (
    static_cast<float>(currentSensorZeroRaw)
    * kAnalogReferenceVolts
    / 1023.0f
  );
  float amps = (
    (volts - zeroVolts) / kCurrentSensorVoltsPerAmp
  );
  if (amps < 0.0f) {
    amps = -amps;
  }
  return amps;
}

bool writeServo(uint8_t channel, uint16_t position) {
  if (channel >= kServoCount) {
    return false;
  }

  if (!servoAttached[channel]) {
    // Keep the timer ISR masked until the admitted target replaces its default.
    const uint8_t interruptState = SREG;
    cli();
    const uint8_t servoIndex = servoChannels[channel].attach(
      kServoPins[channel]
    );
    if (servoIndex != INVALID_SERVO) {
      servoChannels[channel].write(position);
    }
    SREG = interruptState;
    if (servoIndex == INVALID_SERVO) {
      return false;
    }
    servoAttached[channel] = true;
  } else {
    servoChannels[channel].write(position);
  }
  return true;
}

void printCommandName(ar4_auxiliary::CommandKind kind) {
  switch (kind) {
    case ar4_auxiliary::kHelloCommand:
      Serial.print(F("hello"));
      return;
    case ar4_auxiliary::kServoCommand:
      Serial.print(F("servo"));
      return;
    case ar4_auxiliary::kInputReadCommand:
      Serial.print(F("input_read"));
      return;
    case ar4_auxiliary::kSetOutputCommand:
      Serial.print(F("set_output"));
      return;
    case ar4_auxiliary::kWaitInputCommand:
      Serial.print(F("wait_input"));
      return;
    case ar4_auxiliary::kGripperCurrentCommand:
      Serial.print(F("test_gripper_amps"));
      return;
    case ar4_auxiliary::kStopCommand:
      Serial.print(F("stop"));
      return;
    case ar4_auxiliary::kGripperDetachCommand:
      Serial.print(F("gripper_detach"));
      return;
    case ar4_auxiliary::kUnknownCommand:
      return;
  }
}

void beginCompletedResponse(
  ar4_auxiliary::CommandKind kind,
  uint32_t requestId
) {
  Serial.print(F("{\"cmd\":\""));
  printCommandName(kind);
  Serial.print(F("\",\"id\":"));
  Serial.print(static_cast<unsigned long>(requestId));
  Serial.print(F(",\"result\":"));
}

void finishCompletedResponse() {
  Serial.println(
    F(",\"status\":\"completed\",\"type\":\"response\",\"v\":1}")
  );
}

void sendCompletedEmpty(
  ar4_auxiliary::CommandKind kind,
  uint32_t requestId
) {
  beginCompletedResponse(kind, requestId);
  Serial.print(F("{}"));
  finishCompletedResponse();
}

void sendCommandError(
  ar4_auxiliary::CommandKind kind,
  uint32_t requestId,
  const __FlashStringHelper* status,
  const __FlashStringHelper* code,
  const __FlashStringHelper* message
) {
  Serial.print(F("{\"cmd\":\""));
  printCommandName(kind);
  Serial.print(F("\",\"error\":{\"code\":\""));
  Serial.print(code);
  Serial.print(F("\",\"details\":{},\"message\":\""));
  Serial.print(message);
  Serial.print(F("\"},\"id\":"));
  Serial.print(static_cast<unsigned long>(requestId));
  Serial.print(F(",\"status\":\""));
  Serial.print(status);
  Serial.println(F("\",\"type\":\"response\",\"v\":1}"));
}

void sendProtocolError() {
  Serial.println(
    F("{\"error\":{\"code\":\"invalid_request\",\"details\":{},"
      "\"message\":\"Invalid request\"},\"type\":\"protocol_error\","
      "\"v\":1}")
  );
}

void sendInvalidParameters(
  ar4_auxiliary::CommandKind kind,
  uint32_t requestId
) {
  Serial.print(F("{\"cmd\":\""));
  printCommandName(kind);
  Serial.print(
    F("\",\"error\":{\"code\":\"invalid_parameter\","
      "\"details\":{\"field\":\"params\"},"
      "\"message\":\"Invalid command parameters\"},\"id\":")
  );
  Serial.print(static_cast<unsigned long>(requestId));
  Serial.println(
    F(",\"status\":\"rejected\",\"type\":\"response\",\"v\":1}")
  );
}

void sendHello(uint32_t requestId) {
  beginCompletedResponse(ar4_auxiliary::kHelloCommand, requestId);
  Serial.print(
    F("{\"board\":\"mega\",\"commands\":[\"hello\",\"servo\","
      "\"input_read\",\"set_output\",\"wait_input\","
      "\"test_gripper_amps\",\"stop\",\"gripper_detach\"],"
      "\"device\":\"auxiliary_controller\",\"firmware\":{"
      "\"build\":\"ar4hmi\",\"name\":\"AR4 Mega IO\","
      "\"version\":\"2.0\"},\"protocol\":{"
      "\"max_payload_bytes\":")
  );
  Serial.print(
    static_cast<unsigned long>(ar4_auxiliary::kMaximumPayloadLength)
  );
  Serial.print(F(",\"name\":\"ar4_json\",\"version\":1}}"));
  finishCompletedResponse();
}

void stopWait(const ar4_auxiliary::ParsedCommand& stopCommand) {
  const uint32_t waitRequestId = waitOperation.requestId;
  ar4_auxiliary::cancelWait(&waitOperation);
  sendCompletedEmpty(stopCommand.kind, stopCommand.requestId);
  sendCommandError(
    ar4_auxiliary::kWaitInputCommand,
    waitRequestId,
    F("cancelled"),
    F("stop_requested"),
    F("Input wait stopped")
  );
}

void serviceWait() {
  if (!waitOperation.active) {
    return;
  }

  const uint32_t now = static_cast<uint32_t>(millis());
  const uint8_t observedState = (
    digitalRead(waitOperation.pin) == HIGH ? 1 : 0
  );
  const ar4_auxiliary::WaitResult result = ar4_auxiliary::updateWait(
    &waitOperation,
    observedState,
    now
  );
  switch (result) {
    case ar4_auxiliary::kWaitPending:
      return;
    case ar4_auxiliary::kWaitMatched:
      sendCompletedEmpty(
        ar4_auxiliary::kWaitInputCommand,
        waitOperation.requestId
      );
      return;
    case ar4_auxiliary::kWaitTimedOut:
      sendCommandError(
        ar4_auxiliary::kWaitInputCommand,
        waitOperation.requestId,
        F("failed"),
        F("timeout"),
        F("Input wait timed out")
      );
      return;
    case ar4_auxiliary::kWaitInactive:
      ar4_auxiliary::cancelWait(&waitOperation);
      return;
  }
}

bool beginWait(const ar4_auxiliary::ParsedCommand& command) {
  return ar4_auxiliary::startWait(
    &waitOperation,
    command,
    static_cast<uint32_t>(millis())
  );
}

void handleFrame(const ar4_auxiliary::Frame& frame) {
  ar4_auxiliary::ParsedCommand command = {
    ar4_auxiliary::kUnknownCommand,
    0,
    0,
    0,
    0,
    0,
    0,
  };
  const ar4_auxiliary::RequestParseStatus status = (
    ar4_auxiliary::parseRequest(
      frame.data,
      frame.length,
      kBoardProfile,
      &command
    )
  );
  if (status == ar4_auxiliary::kRequestInvalidParameters) {
    sendInvalidParameters(command.kind, command.requestId);
    return;
  }
  if (status != ar4_auxiliary::kRequestReady) {
    sendProtocolError();
    return;
  }

  if (waitOperation.active) {
    if (command.kind == ar4_auxiliary::kStopCommand) {
      stopWait(command);
    } else {
      sendCommandError(
        command.kind,
        command.requestId,
        F("rejected"),
        F("busy"),
        F("Input wait is active")
      );
    }
    return;
  }

  switch (command.kind) {
    case ar4_auxiliary::kHelloCommand:
      sendHello(command.requestId);
      break;
    case ar4_auxiliary::kServoCommand:
      if (writeServo(command.channel, command.position)) {
        sendCompletedEmpty(command.kind, command.requestId);
      } else {
        sendCommandError(
          command.kind,
          command.requestId,
          F("failed"),
          F("servo_unavailable"),
          F("Servo attachment failed")
        );
      }
      break;
    case ar4_auxiliary::kInputReadCommand:
      beginCompletedResponse(command.kind, command.requestId);
      Serial.print(
        digitalRead(command.pin) == HIGH
          ? F("{\"state\":true}")
          : F("{\"state\":false}")
      );
      finishCompletedResponse();
      break;
    case ar4_auxiliary::kSetOutputCommand:
      digitalWrite(command.pin, command.state == 1 ? HIGH : LOW);
      sendCompletedEmpty(command.kind, command.requestId);
      break;
    case ar4_auxiliary::kWaitInputCommand:
      if (!beginWait(command)) {
        sendCommandError(
          command.kind,
          command.requestId,
          F("rejected"),
          F("busy"),
          F("Input wait is active")
        );
      }
      break;
    case ar4_auxiliary::kGripperCurrentCommand:
      beginCompletedResponse(command.kind, command.requestId);
      Serial.print(F("{\"amps\":"));
      Serial.print(readCurrentAmps(), 3);
      Serial.print(F("}"));
      finishCompletedResponse();
      break;
    case ar4_auxiliary::kStopCommand:
      sendCompletedEmpty(command.kind, command.requestId);
      break;
    case ar4_auxiliary::kGripperDetachCommand:
      if (servoAttached[0]) {
        servoChannels[0].detach();
      }
      servoAttached[0] = false;
      sendCompletedEmpty(command.kind, command.requestId);
      break;
    case ar4_auxiliary::kUnknownCommand:
      break;
  }
}

void setup() {
  pinMode(kCurrentSensorPin, INPUT);

  for (
    size_t index = 0;
    index < sizeof(kInputPins) / sizeof(kInputPins[0]);
    ++index
  ) {
    pinMode(kInputPins[index], INPUT_PULLUP);
  }
  for (uint8_t pin = 28; pin <= 35; ++pin) {
    digitalWrite(pin, HIGH);
  }
  for (
    size_t index = 0;
    index < sizeof(kOutputPins) / sizeof(kOutputPins[0]);
    ++index
  ) {
    pinMode(kOutputPins[index], OUTPUT);
  }

  Serial.begin(9600);
  calibrateCurrentSensor();
}

void loop() {
  serviceWait();
  while (Serial.available() > 0) {
    const int received = Serial.read();
    if (received < 0) {
      break;
    }

    ar4_auxiliary::Frame frame;
    const ar4_auxiliary::FrameStatus status = commandFrames.push(
      static_cast<char>(received),
      &frame
    );
    if (status == ar4_auxiliary::kFrameReady) {
      handleFrame(frame);
    } else if (status == ar4_auxiliary::kFrameRejected) {
      sendProtocolError();
    }
    serviceWait();
  }
  serviceWait();
}
