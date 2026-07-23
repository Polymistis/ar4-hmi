#!/usr/bin/env python

############################################################################
## Version AR4 6.7 CA LC                                        ############
############################################################################
""" AR4 - robot control software
    Copyright (c) 2024, Chris Annin
    All rights reserved.

    You are free to share, copy and redistribute in any medium
    or format.  You are free to remix, transform and build upon
    this material.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:

        * Redistributions of source code must retain the above copyright
          notice, this list of conditions and the following disclaimer.
        * Redistribution of this software in source or binary forms shall be free
          of all charges or fees to the recipient of this software.
        * Redistributions in binary form must reproduce the above copyright
          notice, this list of conditions and the following disclaimer in the
          documentation and/or other materials provided with the distribution.
        * you must give appropriate credit and indicate if changes were made. You may do
          so in any reasonable manner, but not in any way that suggests the
          licensor endorses you or your use.
		* Selling robots, robot parts, or any versions of robots or software based on this 
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
"""
##########################################################################
### VERSION DOC ##########################################################
##########################################################################
''' 
**VERSION 1.0 INITIAL RELEASE
  VERSION 1.1 3/5/22 bug fix, position register function 
  VERSION 1.2 4/21/22 added timeout to ser com
  VERSION 1.3 6/17/22 removed timeout ser com - modified cal file
  VERSION 2.0 10/1/22 added spline lookahead
  VERSION 2.2 11/6/22 added opencv integrated vision tab
  VERSION 3.0 2/3/23 move open loop bypass to teensy / add J8 & J9
  VERSION 3.1 5/1/23 gcode initial development
  VERSION 3.2 6/3/23 remove RoboDK kinematics
  VERSION 3.3 6/4/23 update geometric kinematics
  VERSION 4.0 11/5/23 .txt .ar4 extension, gcode tab, kinematics tab. Initial MK2 release.
  VERSION 4.3 1/21/24 Gcode to SD card.  Estop button interrupt.
  VERSION 4.3.1 2/1/24 bug fix - vision snap and find drop down
  VERSION 4.4 3/2/24 added kinematic error handling
  VERSION 4.4 6/29/24 simplified drive motors functions with arrays
  VERSION 5.0 7/14/24 updating kinematics
  VERSION 5.1 1/22/25 bug fix stopping after calibration from CMD window / added Modbus RS-485
  VERSION 5.2 3/23/25 add auto calibrate for individual axis
  VERSION 6.0 6/12/25 add virtual robot
  VERSION 6.1 8/29/25 updated accel and decel, auto calibrate & microsteps
  VERSION 6.2 9/12/25 changed bootstrap theme, xbox upgrade
  VERSION 6.2.1 9/24/25 fixed slider position update
  VERSION 6.3 10/8/25 changed COM entry to dropdown, added beta Linux support, added basic config module
  VERSION 6.4 1/3/26 MK4 update, fixed tool jog, re-added 2 step calibration, add servo amp test
  VERSION 6.4.1 1/24/26 added path resolve for sys.executable
  VERSION 6.5 2/15/26 - gcode bug fix / update program flow with run once and run program loop
  VERSION 6.6 2/22/26 - update kinematic solver to reduce J4/6 wrap | reimplement wrist N/F config
  VERSION 6.7 3/11/26 - fix MB read hold reg bug
'''
##########################################################################

import sys
import os
from datetime import datetime

################################################################################################
## Logging Configuration
import logging
from ARrobots.Logging import CustomOutputHandler, ModuleFilter, dump_logger_info
'''
Check for debug mode from environment variable
Linux - 'export DEBUG=true'
Windows - 'set DEBUG=true'
'''
DEBUG = os.getenv("DEBUG", "").lower() in ("1", "true", "yes", "on") #Check if DEBUG env var is set

logger = logging.getLogger("ARrobots")
logger.setLevel(logging.DEBUG if DEBUG else logging.INFO)

# Add console handler
console = logging.StreamHandler(sys.stdout)
console.setFormatter(logging.Formatter("%(name)s: %(asctime)s [%(levelname)s] %(message)s"))
logger.addHandler(console)
logger.propagate = False

# Function to log to Pane 8
def pane8_log(message):
    if tab8 and hasattr(tab8, "ElogView"):
      Curtime = datetime.now().strftime("%B %d %Y - %I:%M%p")
      try:
          # Schedule insertion on Tkinter main thread
          tab8.ElogView.after(0, lambda: tab8.ElogView.insert(END, f"{Curtime} - {message}"))
      except tk.TclError:
          pass  # widget likely gone

# Setup Pane8 as a logging handler and log there
pane8_handler = CustomOutputHandler(pane8_log)
pane8_handler.setFormatter(logging.Formatter("%(levelname)s - %(message)s"))
# Pan8 should not try to log modules that init before pane8 is available.
pane8_handler.addFilter(ModuleFilter("ARrobots.AR4config"))
pane8_handler.addFilter(ModuleFilter("ARrobots.Logging"))
logger.addHandler(pane8_handler)

if DEBUG:
  dump_logger_info("ARrobots")
  dump_logger_info("ARrobots.AR4config")
  dump_logger_info("ARrobots.Calibration")
  dump_logger_info("ARrobots.Logging")

## End Logging Configuration #######################################################################################

from os import path, execv
import pathlib
import subprocess
from multiprocessing.resource_sharer import stop
from dataclasses import dataclass
from contextlib import contextmanager
import threading
from threading import Lock, Thread
from queue import Empty, Queue

import time

from functools import partial, wraps
import ctypes

import math
import numpy as np
from numpy import mean

import pickle
import serial

from matplotlib import pyplot as plt

from tkinter import *
# Import ttkbootstrap widgets to replace ttk widgets
import ttkbootstrap as ttk_bootstrap
from ttkbootstrap import Style as BootstrapStyle
from ttkbootstrap import *  # This makes ttkbootstrap widgets available globally
from tkinter import simpledialog, messagebox
import tkinter as tk
from tkinter import ttk, Misc
from tkinter import filedialog as fd
import tkinter.messagebox
from PIL import Image, ImageTk

import vtk
from vtkmodules.tk.vtkTkRenderWindowInteractor import vtkTkRenderWindowInteractor
import vtkmodules.vtkInteractionStyle as vtkIS

import webbrowser
import cv2

import re

import ARrobots.robot_kinematics as robot
from ARrobots.Calibration import load_calibration, save_calibration
from ARrobots.HMI.Calibration import apply_calibration
from ARrobots.HMI.joint_motion import (
  AUXILIARY_BOARD_MEGA,
  AUXILIARY_BOARD_NANO,
  AUXILIARY_BOARD_NONE,
  CONTROLLER_CAPABILITY_JT_WRIST_CONFIG_V1,
  CoalescingJointDispatcher,
  CONTROL_POLL_INTERVAL_SECONDS,
  ControllerJointCalibration,
  DeferredLiveMotionArbiter,
  LiveMotionScheduleResult,
  DeferredJointAdjustments,
  MAX_COMMAND_LENGTH,
  MAX_RESPONSE_FRAME_LENGTH,
  MAX_RESPONSE_PAYLOAD_LENGTH,
  MotionInputError,
  MotionQueueFault,
  MotionProfile,
  MotionRequestLease,
  MotionRequestRegistry,
  MotionTransportBusy,
  PositionResponse,
  ProtocolResponseError,
  SerialTransportQuarantinedError,
  SerialTransportTimeout,
  SerialActivityRegistry,
  SerialActivityRejected,
  VirtualMotionOperation,
  auxiliary_pneumatic_output_pin,
  build_virtual_joint_command,
  canonicalize_serial_command,
  canonicalize_virtual_command,
  command_response_timeout,
  controller_degree_to_native_radians,
  controller_number,
  controller_protocol_decimal,
  controller_ratio,
  decode_serial_response_line,
  exchange_serial_line,
  exchange_serial_line_until_cancelled,
  finite_number,
  motion_timing_response_timeout,
  normalize_auxiliary_board_profile,
  parse_auxiliary_output_command,
  parse_auxiliary_servo_command,
  parse_command_timing,
  parse_controller_identity_response,
  parse_controller_modbus_response,
  parse_motion_wrist_config,
  parse_position_response,
  parse_virtual_command_timing,
  quarantine_serial_transport,
  read_serial_exact_response,
  read_serial_line_response,
  read_serial_line_response_with_optional_followup,
  serial_transport_quarantined,
  validate_auxiliary_output_command,
  validate_auxiliary_servo_command,
  validate_controller_filename,
  write_serial_control,
)

#####################################################################################
# Cross-Compat Patch
# We need platform awareness, port enumeration, and some typing imports

from pathlib import Path
import platform
from serial.tools import list_ports
from typing import List, Optional

from ARrobots.AR4config import AR4_Configuration

global Config, CE, CAL, RUN
Config = AR4_Configuration()
CE = Config.Environment
CAL = Config.Calibration
RUN = Config.RuntimeState # Not implemented yet


def _load_startup_calibration():
  calibration_values = load_calibration()
  if not isinstance(calibration_values, dict):
    raise RuntimeError(
      "startup calibration is unavailable or invalid; controller motion remains disabled"
    )
  return calibration_values

if CE['Platform']['IS_WINDOWS']:
  from pygrabber.dshow_graph import FilterGraph


robot.robot_set()

#DIR = pathlib.Path(__file__).parent.resolve()
#os.chdir(DIR)

if getattr(sys, "frozen", False):
    DIR = pathlib.Path(sys.executable).resolve().parent   # folder containing the .exe
else:
    DIR = pathlib.Path(__file__).resolve().parent         # folder containing AR4.py

os.chdir(DIR)    

RUN['cropping'] = False

root = Tk()
root.wm_title("AR4 Software Ver 6.7")
root.iconphoto(True, tk.PhotoImage(file="AR.png"))

# Make headless RPi fit app on screen better
if CE['Platform']['IS_RPI'] and CE['Platform']['IS_HEADLESS']:
  rpi_scale = 0.75
  rpi_x_size = 1590
  rpi_y_size = 800
  logger.debug(f"Running on headless Raspberry Pi - Adjusting scale to {rpi_scale} and window size to {rpi_x_size}x{rpi_y_size}")
  root.tk.call('tk', 'scaling', rpi_scale)
  root.geometry(f'{rpi_x_size}x{rpi_y_size}+0+0')
else:
  root.geometry('1600x900+0+0')  # Adjusted for RPI compatibility (1600x900 minimum)
  #root.geometry("1850x980+0+0")  # Original size

root.resizable(width=True, height=True)

#UI_SCALE = 1.25  

#_orig_place = tk.Widget.place
#def _place_scaled(self, *args, **kw):
#    # scale only absolute pixel arguments
#    for k in ("x", "y", "width", "height"):
#        if k in kw and kw[k] is not None:
#            try:
#                kw[k] = int(float(kw[k]) * UI_SCALE)
#            except Exception:
#                pass
#    return _orig_place(self, *args, **kw)

#tk.Widget.place = _place_scaled




nb = ttk_bootstrap.Notebook(root)

# Configure root window for resizing
root.grid_rowconfigure(0, weight=1)
root.grid_columnconfigure(0, weight=1)
nb.grid(row=0, column=0, sticky='nsew')

tab1 = ttk_bootstrap.Frame(nb)
nb.add(tab1, text=' Main Controls ')

tab2 = ttk_bootstrap.Frame(nb)
nb.add(tab2, text='  Config Settings  ')

tab3 = ttk_bootstrap.Frame(nb)
nb.add(tab3, text='   Kinematics    ')

tab4 = ttk_bootstrap.Frame(nb)
nb.add(tab4, text=' Inputs Outputs ')

tab5 = ttk_bootstrap.Frame(nb)
nb.add(tab5, text='   Registers    ')

tab6 = ttk_bootstrap.Frame(nb)
nb.add(tab6, text='   Vision    ')

tab7 = ttk_bootstrap.Frame(nb)
nb.add(tab7, text='    G-Code     ')

tab8 = ttk_bootstrap.Frame(nb)
nb.add(tab8, text='      Log      ')

tab9 = ttk_bootstrap.Frame(nb)
#nb.add(tab9, text='   Info    ')

def on_closing():
  closing_event = globals().get('application_closing')
  with application_lifecycle_lock:
    if closing_event is not None and closing_event.is_set():
      return False
    serial_activity_registry.begin_shutdown()
    if closing_event is not None:
      closing_event.set()
  runtime_state = globals().get('RUN')
  if isinstance(runtime_state, dict):
    runtime_state['xboxUse'] = 0

  live_pending = globals().get('live_serial_result_pending')
  live_stop = globals().get('live_jog_stop_requested')
  if (
    live_pending is not None
    and live_pending.is_set()
    and live_stop is not None
  ):
    live_stop.set()

  offline_state_lock = globals().get('offline_live_jog_state_lock')
  offline_stop = globals().get('offline_live_jog_stop_event')
  if offline_state_lock is not None and offline_stop is not None:
    with offline_state_lock:
      offline_stop.set()

  dispatcher = globals().get('joint_motion_dispatcher')
  if dispatcher is not None and dispatcher.active:
    dispatcher.close()

  almStatusLab.config(text="SHUTDOWN WAITING FOR CONTROLLER", style="Warn.TLabel")
  almStatusLab2.config(text="SHUTDOWN WAITING FOR CONTROLLER", style="Warn.TLabel")
  _poll_application_close()
  return True

root.wm_protocol("WM_DELETE_WINDOW", on_closing)

root.runTrue = 0
root.GCrunTrue = 0


RUN['selectedTemplate'] = StringVar()
RUN['selectedTemplate'].set("")

live_jog_lock = threading.Lock()
live_cartesian_lock = threading.Lock()
live_tool_lock = threading.Lock()
application_lifecycle_lock = threading.Lock()
offline_live_jog_lock = threading.Lock()
offline_live_jog_state_lock = threading.Lock()
offline_live_jog_stop_event = threading.Event()
offline_live_jog_stop_event.set()
offline_live_jog_pose_snapshot = None
drive_lock = threading.Lock()
serial_lock = threading.Lock()
main_serial_operation_state = threading.local()
manual_motion_request_state = threading.local()
motion_request_admission_state = threading.local()
serial_write_lock = threading.Lock()
serial_event_queue = Queue()
calibration_serial_event_queue = Queue()
calibration_operation_lock = threading.Lock()
calibration_operation = None
calibration_next_request_id = 0
calibration_terminal_owner_lock = threading.Lock()
calibration_terminal_response_pending = threading.Event()
calibration_serial_write_committed = threading.Event()
auxiliary_serial_lock = threading.Lock()
auxiliary_serial_write_lock = threading.Lock()
auxiliary_serial_event_queue = Queue()
program_stop_status_event_queue = Queue()
manual_auxiliary_event_queue = Queue()
manual_auxiliary_request_queue = []
manual_auxiliary_active_request = None
manual_auxiliary_next_request_id = 0
manual_auxiliary_state_lock = threading.Lock()
manual_auxiliary_stop_barrier = threading.Event()
startup_auxiliary_cleanup_lock = threading.Lock()
startup_auxiliary_cleanup_pending = {}
startup_auxiliary_cleanup_worker = None
startup_controller_cleanup_lock = threading.Lock()
startup_controller_cleanup_pending = {}
startup_controller_cleanup_worker = None
xbox_auxiliary_event_queue = Queue()
auxiliary_stop_requested = threading.Event()
auxiliary_stop_state_lock = threading.Lock()
program_stop_state_lock = threading.RLock()
auxiliary_stop_next_request_id = 0
auxiliary_stop_pending_request_id = None
auxiliary_stop_active_request_id = None
auxiliary_stop_owner_waiting = False
auxiliary_stop_owner_result = None
auxiliary_stop_owner_result_event = threading.Event()
auxiliary_stop_injected_event = threading.Event()
auxiliary_stop_acknowledgement_deadline = None
xbox_auxiliary_next_request_id = 0
controller_correction_requested = threading.Event()
controller_correction_state_lock = threading.Lock()
manual_motion_pose_pending = threading.Event()
controller_position_resynchronization_required = threading.Event()
kinematics_configuration_ready = threading.Event()
acknowledged_forced_position_lock = threading.Lock()
acknowledged_forced_position_target = None
virtual_motion_event_queue = Queue()
offline_live_jog_operation = None
legacy_serial_result_pending = threading.Event()
live_serial_result_pending = threading.Event()
live_jog_stop_requested = threading.Event()
application_closing = threading.Event()
application_shutdown_started_at = None
shutdown_serial_cancel_requested = set()
serial_activity_registry = SerialActivityRegistry(
  ("ser", "ser2"),
  single_owner_names=("ser2",),
)
motion_request_registry = MotionRequestRegistry()
joint_motion_request_lock = threading.Lock()
joint_motion_request_lease = None
offline_live_jog_motion_lease = None


class CalibrationCancellationBoundary:
  def __init__(self, shutdown_event, write_started_event):
    if not callable(getattr(shutdown_event, "is_set", None)):
      raise TypeError("calibration shutdown event must satisfy the event contract")
    if not callable(getattr(write_started_event, "is_set", None)):
      raise TypeError("calibration write-start event must satisfy the event contract")
    self._shutdown_event = shutdown_event
    self._write_started_event = write_started_event

  def is_set(self):
    write_started = self._write_started_event.is_set()
    if not isinstance(write_started, bool):
      raise TypeError("calibration write-start state must be boolean")
    if write_started:
      return False
    shutdown_started = self._shutdown_event.is_set()
    if not isinstance(shutdown_started, bool):
      raise TypeError("calibration shutdown state must be boolean")
    return shutdown_started


class CalibrationWriteCommitment:
  def __init__(self, shared_event):
    if not all(
      callable(getattr(shared_event, method_name, None))
      for method_name in ("set", "clear", "is_set")
    ):
      raise TypeError("calibration commitment event contract is invalid")
    self._shared_event = shared_event
    self._local_event = threading.Event()

  def set(self):
    self._shared_event.set()
    self._local_event.set()

  def is_set(self):
    committed = self._local_event.is_set()
    if not isinstance(committed, bool):
      raise TypeError("calibration commitment state must be boolean")
    return committed

SERIAL_BASE_RESPONSE_TIMEOUT_SECONDS = 120
SERIAL_RESPONSE_MARGIN_SECONDS = 10
SERIAL_EVENT_APPLICATION_MARGIN_SECONDS = 5
SERIAL_WRITE_TIMEOUT_SECONDS = 5
SERIAL_AUXILIARY_RESPONSE_TIMEOUT_SECONDS = 5
SERIAL_AUXILIARY_WAIT_MARGIN_SECONDS = 5
# Teensy WJ, WK, and WT convert seconds to signed 32-bit milliseconds.
MAIN_FIRMWARE_WAIT_MAX_SECONDS = 2147483
# Firmware emits the live acknowledgement before entering segment execution.
SERIAL_LIVE_ACK_TIMEOUT_SECONDS = 5
SERIAL_STARTUP_READ_TIMEOUT_SECONDS = 10
SERIAL_SHUTDOWN_POLL_MS = 25
SERIAL_SHUTDOWN_RETRY_MS = 1000
SERIAL_SHUTDOWN_ACTIVITY_GRACE_SECONDS = 1.0
FIRMWARE_AXIS_COUNT = 9
FIRMWARE_DISTRIBUTION_DELAY_MICROSECONDS = 30
FIRMWARE_MAX_MILLIMETERS_PER_SECOND = 192
AUXILIARY_FIRMWARE_SIGNED_INT_MAX = 32767
AUXILIARY_WAIT_TERMINAL_RESPONSES = ("Done", "Timeout", "Nano Stopped")
AUXILIARY_WAIT_NATURAL_RESPONSES = ("Done", "Timeout")
AUXILIARY_INACTIVE_STOP_RESPONSE = "Nano Inactive Stopped"
AUXILIARY_STOP_OWNER_RESPONSES = (
  *AUXILIARY_WAIT_TERMINAL_RESPONSES,
  AUXILIARY_INACTIVE_STOP_RESPONSE,
)
AUXILIARY_STOP_NOT_REQUIRED = "not-required"
AUXILIARY_STOP_PENDING = "pending"
AUXILIARY_STOP_DISPATCHED = "dispatched"
XBOX_AUXILIARY_PENDING_KEYS = {
  "_grip_closed": "_grip_pending_request_id",
  "_pneu_open": "_pneu_pending_request_id",
}
XBOX_AUXILIARY_FIXED_TOGGLE_COMMANDS = {
  "_grip_closed": {
    False: ("SV0P50\n", b"Servo Done"),
    True: ("SV0P0\n", b"Servo Done"),
  },
}
XBOX_AUXILIARY_TOGGLE_RESPONSES = {
  "_grip_closed": b"Servo Done",
  "_pneu_open": b"Done",
}
XBOX_AUXILIARY_FIXED_EXCHANGES = frozenset(
  exchange
  for commands in XBOX_AUXILIARY_FIXED_TOGGLE_COMMANDS.values()
  for exchange in commands.values()
)
MANUAL_AUXILIARY_QUEUE_LIMIT = 64
MANUAL_AUXILIARY_CALIBRATION_KEYS = frozenset(
  (
    *(f"Servo{channel}{state}" for channel in range(4) for state in ("on", "off")),
    *(f"DO{row}{state}" for row in range(1, 7) for state in ("on", "off")),
  )
)


@dataclass(frozen=True)
class ManualAuxiliaryRequest:
  request_id: int
  serial_port: object
  command: str
  expected_response: bytes
  calibration_key: str
  calibration_value: str

  def __post_init__(self):
    if (
      isinstance(self.request_id, bool)
      or not isinstance(self.request_id, int)
      or self.request_id <= 0
    ):
      raise MotionInputError("manual auxiliary request ID must be positive")
    if self.serial_port is None or not getattr(self.serial_port, "is_open", False):
      raise ConnectionError(
        "manual auxiliary request requires an open connection"
      )
    if (
      not isinstance(self.command, str)
      or not self.command
      or len(self.command) > MAX_COMMAND_LENGTH
      or not self.command.isascii()
    ):
      raise MotionInputError("manual auxiliary command is invalid")
    if self.expected_response not in (b"Servo Done", b"Done"):
      raise MotionInputError(
        "manual auxiliary expected response is invalid"
      )
    if self.calibration_key not in MANUAL_AUXILIARY_CALIBRATION_KEYS:
      raise MotionInputError("manual auxiliary calibration key is invalid")
    if (
      not isinstance(self.calibration_value, str)
      or not self.calibration_value
      or self.calibration_value != self.calibration_value.strip()
      or not self.calibration_value.isdecimal()
    ):
      raise MotionInputError("manual auxiliary calibration value is invalid")

    if self.expected_response == b"Servo Done":
      _connected_auxiliary_board_profile(self.serial_port)
      channel, position = parse_auxiliary_servo_command(self.command)
      if (
        self.calibration_key
        not in (f"Servo{channel}on", f"Servo{channel}off")
        or int(self.calibration_value) != position
      ):
        raise MotionInputError(
          "manual auxiliary servo persistence contract is invalid"
        )
    else:
      board_profile = _connected_auxiliary_board_profile(self.serial_port)
      validate_auxiliary_output_command(self.command, board_profile)
      prefix, output_pin = parse_auxiliary_output_command(self.command)
      expected_suffix = "on" if prefix == "ON" else "off"
      if (
        not self.calibration_key.startswith("DO")
        or not self.calibration_key.endswith(expected_suffix)
        or int(self.calibration_value) != output_pin
      ):
        raise MotionInputError(
          "manual auxiliary output persistence contract is invalid"
        )


@dataclass(frozen=True)
class ManualAuxiliaryResult:
  request_id: int
  outcome: str
  value: str

  def __post_init__(self):
    if (
      isinstance(self.request_id, bool)
      or not isinstance(self.request_id, int)
      or self.request_id <= 0
    ):
      raise MotionInputError("manual auxiliary result ID must be positive")
    if self.outcome not in ("completed", "failed"):
      raise MotionInputError("manual auxiliary result outcome is invalid")
    if (
      not isinstance(self.value, str)
      or not self.value.strip()
      or self.value != self.value.strip()
    ):
      raise MotionInputError("manual auxiliary result value is invalid")


def _bind_auxiliary_board_profile(serial_port, board_profile):
  if serial_port is None or not getattr(serial_port, "is_open", False):
    raise ConnectionError(
      "auxiliary-board profile requires an open auxiliary connection"
    )
  if RUN.get('ser2') is not serial_port:
    raise ConnectionError(
      "auxiliary-board profile requires the active auxiliary connection"
    )
  profile = normalize_auxiliary_board_profile(board_profile)
  RUN['ser2BoardProfile'] = (serial_port, profile)
  return profile


def _clear_auxiliary_board_profile(serial_port=None):
  binding = RUN.get('ser2BoardProfile')
  if serial_port is None or (
    isinstance(binding, tuple)
    and len(binding) == 2
    and binding[0] is serial_port
  ):
    RUN['ser2BoardProfile'] = None
    return True
  return False


def _connected_auxiliary_board_profile(serial_port=None):
  if serial_port is None:
    serial_port = RUN.get('ser2')
  if serial_port is None or not getattr(serial_port, "is_open", False):
    raise ConnectionError("auxiliary controller serial connection is not open")
  if RUN.get('ser2') is not serial_port:
    raise ConnectionError("auxiliary controller connection ownership changed")
  binding = RUN.get('ser2BoardProfile')
  if (
    not isinstance(binding, tuple)
    or len(binding) != 2
    or binding[0] is not serial_port
  ):
    raise MotionInputError(
      "auxiliary-board profile is not bound to the active connection"
    )
  return normalize_auxiliary_board_profile(binding[1])


def _xbox_auxiliary_toggle_exchange(
  state_name,
  target_state,
  serial_port=None,
):
  if state_name not in XBOX_AUXILIARY_PENDING_KEYS:
    raise MotionInputError("Xbox auxiliary state name is invalid")
  if not isinstance(target_state, bool):
    raise MotionInputError("Xbox auxiliary target state must be boolean")
  profile = _connected_auxiliary_board_profile(serial_port)
  if state_name == "_grip_closed":
    return XBOX_AUXILIARY_FIXED_TOGGLE_COMMANDS[state_name][target_state]
  if state_name != "_pneu_open":
    raise MotionInputError("Xbox pneumatic state name is invalid")
  output_pin = auxiliary_pneumatic_output_pin(profile)
  output_prefix = "OF" if target_state else "ON"
  command = f"{output_prefix}X{output_pin}\n"
  expected_response = _xbox_auxiliary_expected_response(command, serial_port)
  return command, expected_response


def _xbox_auxiliary_expected_response(command, serial_port=None):
  if (
    not isinstance(command, str)
    or not command
    or len(command) > MAX_COMMAND_LENGTH
  ):
    raise MotionInputError("Xbox auxiliary command is invalid")
  fixed_responses = {
    expected_response
    for supported_command, expected_response in XBOX_AUXILIARY_FIXED_EXCHANGES
    if supported_command == command
  }
  if len(fixed_responses) == 1:
    return fixed_responses.pop()
  output_match = re.fullmatch(r"(?:ON|OF)X([0-9]+)\n", command)
  if output_match is None:
    raise MotionInputError("Xbox auxiliary exchange is not supported")
  profile = _connected_auxiliary_board_profile(serial_port)
  validate_auxiliary_output_command(command, profile)
  return b"Done"


def _main_serial_transmit_required(transmit=True):
  return transmit is not False


def _synchronous_motion_request(
  name,
  rejection_result=False,
  requires_kinematics=True,
):
  if not isinstance(requires_kinematics, bool):
    raise TypeError("kinematics admission flag must be boolean")

  def decorate(callback):
    @wraps(callback)
    def guarded(*args, **kwargs):
      request_lease = _acquire_motion_request(
        name,
        requires_kinematics=requires_kinematics,
      )
      if request_lease is None:
        return rejection_result
      try:
        return callback(*args, **kwargs)
      finally:
        _finish_motion_request(request_lease)
    return guarded
  return decorate


@contextmanager
def _reserve_main_serial_operation():
  reservation = getattr(main_serial_operation_state, "reservation", None)
  created = reservation is None
  if created:
    if not serial_lock.acquire(blocking=False):
      raise SerialActivityRejected("controller transport is busy")
    reservation = {"depth": 0, "transferred": False}
    main_serial_operation_state.reservation = reservation
  elif (
    not isinstance(reservation, dict)
    or set(reservation) != {"depth", "transferred"}
    or isinstance(reservation["depth"], bool)
    or not isinstance(reservation["depth"], int)
    or reservation["depth"] <= 0
    or not isinstance(reservation["transferred"], bool)
  ):
    raise RuntimeError("main serial reservation state is invalid")

  if reservation["transferred"]:
    raise SerialActivityRejected(
      "controller transport ownership was transferred to a serial worker"
    )
  reservation["depth"] += 1
  try:
    yield
  finally:
    current = getattr(main_serial_operation_state, "reservation", None)
    if current is not reservation or reservation["depth"] <= 0:
      raise RuntimeError("main serial reservation cleanup state is invalid")
    reservation["depth"] -= 1
    if reservation["depth"] == 0:
      del main_serial_operation_state.reservation
      if not reservation["transferred"]:
        serial_lock.release()


def _transfer_main_serial_reservation():
  reservation = getattr(main_serial_operation_state, "reservation", None)
  if reservation is None:
    return False
  if (
    not isinstance(reservation, dict)
    or reservation.get("depth") != 1
    or reservation.get("transferred") is not False
    or not serial_lock.locked()
  ):
    raise RuntimeError("main serial reservation cannot be transferred")
  reservation["transferred"] = True
  return True


def _restore_main_serial_reservation():
  reservation = getattr(main_serial_operation_state, "reservation", None)
  if (
    not isinstance(reservation, dict)
    or reservation.get("depth") != 1
    or reservation.get("transferred") is not True
    or not serial_lock.locked()
  ):
    raise RuntimeError("main serial reservation cannot be restored")
  reservation["transferred"] = False


def _tracked_serial_operation(
  *serial_names,
  auxiliary_control_injectable=False,
  rejection_result=False,
  on_rejected=None,
  operation_required=None,
):
  if not isinstance(auxiliary_control_injectable, bool):
    raise TypeError("auxiliary_control_injectable must be boolean")
  if auxiliary_control_injectable and "ser2" not in serial_names:
    raise ValueError(
      "auxiliary control injection requires tracked ser2 activity"
    )
  if on_rejected is not None and not callable(on_rejected):
    raise TypeError("on_rejected must be callable")
  if operation_required is not None and not callable(operation_required):
    raise TypeError("operation_required must be callable")
  injectable_names = (
    ("ser2",)
    if auxiliary_control_injectable
    else ()
  )

  def decorator(function):
    @wraps(function)
    def tracked(*args, **kwargs):
      required = (
        True
        if operation_required is None
        else operation_required(*args, **kwargs)
      )
      if not isinstance(required, bool):
        raise TypeError("operation_required must return a boolean")
      if not required:
        return function(*args, **kwargs)

      main_reservation = None
      main_reserved = False
      auxiliary_reserved = False
      activity = None
      try:
        if "ser" in serial_names:
          main_reservation = _reserve_main_serial_operation()
          main_reservation.__enter__()
          main_reserved = True
        if "ser2" in serial_names:
          auxiliary_reserved = auxiliary_serial_lock.acquire(blocking=False)
          if not auxiliary_reserved:
            raise SerialActivityRejected(
              "auxiliary controller transport is busy"
            )
        activity = serial_activity_registry.operations(
          serial_names,
          control_injectable_names=injectable_names,
        )
        activity.__enter__()
      except SerialActivityRejected as exc:
        if auxiliary_reserved:
          auxiliary_serial_lock.release()
        if main_reserved:
          main_reservation.__exit__(None, None, None)
        logger.warning("Serial operation rejected: %s", exc)
        if on_rejected is not None:
          on_rejected()
        return rejection_result
      except Exception:
        if auxiliary_reserved:
          auxiliary_serial_lock.release()
        if main_reserved:
          main_reservation.__exit__(None, None, None)
        raise
      try:
        return function(*args, **kwargs)
      finally:
        try:
          activity.__exit__(None, None, None)
        finally:
          try:
            if auxiliary_reserved:
              auxiliary_serial_lock.release()
          finally:
            if main_reserved:
              main_reservation.__exit__(None, None, None)

    return tracked

  return decorator


@contextmanager
def _tracked_auxiliary_operation(control_injectable=False):
  if not isinstance(control_injectable, bool):
    raise TypeError("control_injectable must be boolean")
  auxiliary_reserved = auxiliary_serial_lock.acquire(blocking=False)
  if not auxiliary_reserved:
    raise SerialActivityRejected(
      "serial operation rejected while the auxiliary transport is busy"
    )
  activity = serial_activity_registry.operations(
    ("ser2",),
    control_injectable_names=("ser2",) if control_injectable else (),
  )
  try:
    activity.__enter__()
  except Exception:
    auxiliary_serial_lock.release()
    raise
  try:
    yield
  finally:
    activity.__exit__(None, None, None)
    auxiliary_serial_lock.release()


def _write_legacy_auxiliary_command(command):
  serial_port = RUN.get('ser2')
  try:
    if isinstance(command, str) and command[:2] in ("ON", "OF", "SV"):
      if command[:2] == "SV":
        _connected_auxiliary_board_profile(serial_port)
        validate_auxiliary_servo_command(command)
      else:
        board_profile = _connected_auxiliary_board_profile(serial_port)
        validate_auxiliary_output_command(command, board_profile)
    return write_serial_control(
      serial_port,
      command,
      write_lock=auxiliary_serial_write_lock,
      reset_input=True,
    )
  finally:
    if (
      RUN.get('ser2') is serial_port
      and not getattr(serial_port, "is_open", False)
    ):
      RUN['ser2'] = None
      _clear_auxiliary_board_profile(serial_port)


def _manual_auxiliary_expected_response(command, serial_port=None):
  if isinstance(command, str) and command.startswith("SV"):
    _connected_auxiliary_board_profile(serial_port)
    validate_auxiliary_servo_command(command)
    return b"Servo Done"
  profile = _connected_auxiliary_board_profile(serial_port)
  validate_auxiliary_output_command(command, profile)
  return b"Done"


@_tracked_serial_operation("ser2")
def _exchange_manual_auxiliary_command(
  command,
  expected_response,
  expected_serial_port,
):
  if not isinstance(expected_response, bytes):
    raise MotionInputError(
      "manual auxiliary exchange response contract is invalid"
    )
  serial_port = RUN.get('ser2')
  if serial_port is not expected_serial_port:
    raise ConnectionError(
      "manual auxiliary connection changed before request dispatch"
    )
  if expected_response != _manual_auxiliary_expected_response(
    command,
    serial_port,
  ):
    raise MotionInputError(
      "manual auxiliary expected response does not match the command"
    )

  try:
    _write_legacy_auxiliary_command(command)
    return read_serial_exact_response(
      serial_port,
      expected_response,
      SERIAL_AUXILIARY_RESPONSE_TIMEOUT_SECONDS,
    )
  except Exception:
    if RUN.get('ser2') is serial_port:
      _close_serial_port('ser2', "manual auxiliary response failure")
    raise


@_tracked_serial_operation("ser2")
def _exchange_xbox_auxiliary_command(
  command,
  expected_response,
  expected_serial_port=None,
):
  if not isinstance(command, str) or not isinstance(expected_response, bytes):
    raise MotionInputError("Xbox auxiliary exchange contract is invalid")
  serial_port = RUN.get('ser2')
  if (
    expected_serial_port is not None
    and serial_port is not expected_serial_port
  ):
    raise ConnectionError(
      "Xbox auxiliary connection changed before request dispatch"
    )
  _connected_auxiliary_board_profile(serial_port)
  if expected_response != _xbox_auxiliary_expected_response(
    command,
    serial_port,
  ):
    raise MotionInputError("Xbox auxiliary expected response is invalid")

  try:
    _write_legacy_auxiliary_command(command)
    response = read_serial_exact_response(
      serial_port,
      expected_response,
      SERIAL_AUXILIARY_RESPONSE_TIMEOUT_SECONDS,
    )
  except Exception:
    if RUN.get('ser2') is serial_port:
      _close_serial_port('ser2', "Xbox auxiliary response failure")
    raise
  return response


def _run_xbox_auxiliary_request(
  request_id,
  state_name,
  target_state,
  serial_port,
  command,
  expected_response,
):
  try:
    response = _exchange_xbox_auxiliary_command(
      command,
      expected_response,
      serial_port,
    )
    if response is False:
      event = (
        "rejected",
        request_id,
        state_name,
        target_state,
        "auxiliary controller transport is busy",
      )
    elif not isinstance(response, str) or not response:
      raise ProtocolResponseError(
        "Xbox auxiliary exchange returned an invalid acknowledgement"
      )
    else:
      event = (
        "completed",
        request_id,
        state_name,
        target_state,
        response,
      )
  except Exception as exc:
    logger.exception("Xbox auxiliary command failed")
    message = str(exc).strip() or "Xbox auxiliary command failed without details"
    event = ("failed", request_id, state_name, target_state, message)
  xbox_auxiliary_event_queue.put(event)


def _start_xbox_auxiliary_request(request_id, state_name, target_state):
  if (
    isinstance(request_id, bool)
    or not isinstance(request_id, int)
    or request_id <= 0
  ):
    raise MotionInputError("Xbox auxiliary request ID must be positive")
  if state_name not in XBOX_AUXILIARY_PENDING_KEYS:
    raise MotionInputError("Xbox auxiliary state name is invalid")
  if not isinstance(target_state, bool):
    raise MotionInputError("Xbox auxiliary target state must be boolean")
  serial_port = RUN.get('ser2')
  command, expected_response = _xbox_auxiliary_toggle_exchange(
    state_name,
    target_state,
    serial_port,
  )
  try:
    thread = threading.Thread(
      target=_run_xbox_auxiliary_request,
      args=(
        request_id,
        state_name,
        target_state,
        serial_port,
        command,
        expected_response,
      ),
      daemon=True,
    )
    thread.start()
  except Exception:
    logger.exception("Unable to start the Xbox auxiliary worker")
    return False
  return True


def _request_xbox_auxiliary_toggle(state_name):
  global xbox_auxiliary_next_request_id

  if state_name not in XBOX_AUXILIARY_PENDING_KEYS:
    raise MotionInputError("Xbox auxiliary state name is invalid")
  current_state = RUN.get(state_name)
  if not isinstance(current_state, bool):
    logger.error("Xbox auxiliary state is unknown: %s", state_name)
    return False
  pending_name = XBOX_AUXILIARY_PENDING_KEYS[state_name]
  pending_request_id = RUN.get(pending_name)
  if pending_request_id is not None:
    logger.warning("Xbox auxiliary toggle already pending: %s", state_name)
    return False
  if (
    isinstance(xbox_auxiliary_next_request_id, bool)
    or not isinstance(xbox_auxiliary_next_request_id, int)
    or xbox_auxiliary_next_request_id < 0
  ):
    raise RuntimeError("Xbox auxiliary request counter is invalid")

  xbox_auxiliary_next_request_id += 1
  request_id = xbox_auxiliary_next_request_id
  target_state = not current_state
  RUN[pending_name] = request_id
  try:
    started = _start_xbox_auxiliary_request(
      request_id,
      state_name,
      target_state,
    )
  except Exception:
    RUN[pending_name] = None
    raise
  if started is not True:
    RUN[pending_name] = None
    return False
  return True


def _manual_auxiliary_stop_in_progress():
  with auxiliary_stop_state_lock:
    controller_stop_pending = (
      auxiliary_stop_pending_request_id is not None
      or auxiliary_stop_active_request_id is not None
    )
  with program_stop_state_lock:
    program_stop_pending = RUN.get('programStopRequestId') is not None
    estop_active = bool(RUN.get('estopActive'))
    position_fault = bool(RUN.get('posOutreach'))
  return (
    manual_auxiliary_stop_barrier.is_set()
    or auxiliary_stop_requested.is_set()
    or controller_stop_pending
    or program_stop_pending
    or estop_active
    or position_fault
  )


def _manual_auxiliary_status_reserved():
  if _manual_auxiliary_stop_in_progress():
    return True
  with program_stop_state_lock:
    return bool(RUN.get('programStopStatusLatched'))


def _manual_auxiliary_program_active():
  program_state = getattr(tab1, "runTrue", 0)
  if isinstance(program_state, bool):
    return program_state
  if (
    isinstance(program_state, int)
    and not isinstance(program_state, bool)
    and program_state in (0, 1)
  ):
    return bool(program_state)
  raise RuntimeError("program execution state is invalid")


def _acknowledge_program_stop_status_for_manual_auxiliary():
  if _manual_auxiliary_stop_in_progress():
    return False
  with program_stop_state_lock:
    if (
      RUN.get('programStopRequestId') is not None
      or RUN.get('estopActive')
      or RUN.get('posOutreach')
    ):
      return False
    RUN['programStopStatusLatched'] = False
  return True


def _set_manual_auxiliary_status(message, style):
  if (
    not isinstance(message, str)
    or not message.strip()
    or message != message.strip()
  ):
    raise TypeError("manual auxiliary status must be normalized text")
  if not isinstance(style, str) or not style:
    raise TypeError("manual auxiliary status style must be non-empty text")
  if _manual_auxiliary_status_reserved():
    return False
  almStatusLab.config(text=message, style=style)
  almStatusLab2.config(text=message, style=style)
  return True


def _set_manual_auxiliary_feedback(message):
  if (
    not isinstance(message, str)
    or not message.strip()
    or message != message.strip()
  ):
    raise TypeError("manual auxiliary feedback must be normalized text")
  cmdRecEntryField.delete(0, 'end')
  cmdRecEntryField.insert(0, message)
  return True


def _render_manual_auxiliary_rejection(message):
  _set_manual_auxiliary_feedback(message)
  _set_manual_auxiliary_status(message, "Alarm.TLabel")
  return False


def _manual_auxiliary_error_detail(error, fallback):
  if (
    not isinstance(fallback, str)
    or not fallback.strip()
    or fallback != fallback.strip()
  ):
    raise TypeError("manual auxiliary error fallback must be normalized text")
  try:
    detail = str(error).strip()
  except Exception:
    return fallback
  return detail or fallback


def _begin_manual_auxiliary_stop(reason):
  if (
    not isinstance(reason, str)
    or not reason.strip()
    or reason != reason.strip()
  ):
    raise TypeError("manual auxiliary cancellation reason must be normalized text")
  with manual_auxiliary_state_lock:
    manual_auxiliary_stop_barrier.set()
    discarded = len(manual_auxiliary_request_queue)
    manual_auxiliary_request_queue.clear()
  if discarded:
    logger.warning(
      "Discarded %s queued manual auxiliary command(s): %s",
      discarded,
      reason,
    )
  return discarded


def _reject_queued_manual_auxiliary_requests(message):
  if (
    not isinstance(message, str)
    or not message.strip()
    or message != message.strip()
  ):
    raise TypeError("manual auxiliary rejection must be normalized text")
  with manual_auxiliary_state_lock:
    if manual_auxiliary_active_request is not None:
      return 0
    discarded = len(manual_auxiliary_request_queue)
    manual_auxiliary_request_queue.clear()
  if discarded:
    logger.warning("%s; discarded %s queued command(s)", message, discarded)
    _render_manual_auxiliary_rejection(message)
  return discarded


def _run_manual_auxiliary_request(request):
  try:
    if not isinstance(request, ManualAuxiliaryRequest):
      raise TypeError("manual auxiliary worker request is invalid")
    response = _exchange_manual_auxiliary_command(
      request.command,
      request.expected_response,
      request.serial_port,
    )
    if response is False:
      raise SerialActivityRejected(
        "auxiliary controller transport changed before dispatch"
      )
    elif response != request.expected_response.decode("ascii"):
      raise ProtocolResponseError(
        "manual auxiliary acknowledgement did not match the request"
      )
    else:
      result = ManualAuxiliaryResult(
        request.request_id,
        "completed",
        response,
      )
  except Exception as exc:
    logger.exception("Manual auxiliary command failed")
    message = _manual_auxiliary_error_detail(
      exc,
      "manual auxiliary command failed without details",
    )
    request_id = getattr(request, "request_id", None)
    if (
      isinstance(request_id, bool)
      or not isinstance(request_id, int)
      or request_id <= 0
    ):
      logger.error("Manual auxiliary worker could not publish an invalid request")
      manual_auxiliary_event_queue.put(None)
      return
    result = ManualAuxiliaryResult(request_id, "failed", message)
  manual_auxiliary_event_queue.put(result)


def _try_dispatch_manual_auxiliary_request():
  global manual_auxiliary_active_request

  if (
    application_closing.is_set()
    or _manual_auxiliary_stop_in_progress()
    or _manual_auxiliary_program_active()
  ):
    return False
  if (
    serial_activity_registry.active("ser2")
    or auxiliary_serial_lock.locked()
  ):
    return False

  with manual_auxiliary_state_lock:
    if application_closing.is_set() or manual_auxiliary_stop_barrier.is_set():
      return False
    if manual_auxiliary_active_request is not None:
      if not isinstance(
        manual_auxiliary_active_request,
        ManualAuxiliaryRequest,
      ):
        raise RuntimeError("manual auxiliary active request is invalid")
      return False
    if not manual_auxiliary_request_queue:
      return False
    request = manual_auxiliary_request_queue.pop(0)
    if not isinstance(request, ManualAuxiliaryRequest):
      raise RuntimeError("manual auxiliary queued request is invalid")
    manual_auxiliary_active_request = request
    try:
      thread = threading.Thread(
        target=_run_manual_auxiliary_request,
        args=(request,),
        daemon=True,
      )
      thread.start()
    except Exception as exc:
      detail = _manual_auxiliary_error_detail(
        exc,
        "worker startup failed without details",
      )
      message = f"Unable to start manual auxiliary worker: {detail}"
      logger.exception(message)
      manual_auxiliary_event_queue.put(
        ManualAuxiliaryResult(request.request_id, "failed", message)
      )

  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0, request.command)
  _set_manual_auxiliary_status(
    "AUXILIARY COMMAND IN PROGRESS",
    "Warn.TLabel",
  )
  return True


def _queue_manual_auxiliary_command(
  command,
  calibration_key,
  calibration_value,
):
  global manual_auxiliary_next_request_id

  if application_closing.is_set():
    return _render_manual_auxiliary_rejection(
      "AUXILIARY COMMAND REJECTED DURING SHUTDOWN"
    )
  if _manual_auxiliary_stop_in_progress():
    message = "AUXILIARY COMMAND REJECTED WHILE STOPPED"
    logger.warning(message)
    return _render_manual_auxiliary_rejection(message)
  if _manual_auxiliary_program_active():
    message = "AUXILIARY COMMAND REJECTED WHILE PROGRAM IS RUNNING"
    logger.warning(message)
    return _render_manual_auxiliary_rejection(message)
  if not _acknowledge_program_stop_status_for_manual_auxiliary():
    message = "AUXILIARY COMMAND REJECTED WHILE STOPPED"
    logger.warning(message)
    return _render_manual_auxiliary_rejection(message)
  if RUN['offlineMode']:
    return _render_manual_auxiliary_rejection(
      "AUXILIARY COMMAND REQUIRES AN ONLINE CONTROLLER"
    )
  with manual_auxiliary_state_lock:
    manual_request_active = manual_auxiliary_active_request is not None
  if not manual_request_active and (
    serial_activity_registry.active("ser2")
    or auxiliary_serial_lock.locked()
  ):
    message = "AUXILIARY COMMAND REJECTED WHILE TRANSPORT IS BUSY"
    logger.warning(message)
    return _render_manual_auxiliary_rejection(message)

  try:
    serial_port = RUN.get('ser2')
    expected_response = _manual_auxiliary_expected_response(
      command,
      serial_port,
    )
  except Exception as exc:
    detail = _manual_auxiliary_error_detail(
      exc,
      "command validation failed without details",
    )
    message = f"Manual auxiliary command rejected: {detail}"
    logger.error(message)
    return _render_manual_auxiliary_rejection(message)

  try:
    with manual_auxiliary_state_lock:
      if (
        application_closing.is_set()
        or manual_auxiliary_stop_barrier.is_set()
        or auxiliary_stop_requested.is_set()
      ):
        raise SerialActivityRejected(
          "manual auxiliary command rejected while a stop is active"
        )
      if manual_auxiliary_active_request is None and (
        serial_activity_registry.active("ser2")
        or auxiliary_serial_lock.locked()
      ):
        raise SerialActivityRejected(
          "manual auxiliary command rejected while transport is busy"
        )
      queued_count = len(manual_auxiliary_request_queue)
      if manual_auxiliary_active_request is not None:
        queued_count += 1
      if queued_count >= MANUAL_AUXILIARY_QUEUE_LIMIT:
        queue_full = True
        request = None
      else:
        queue_full = False
        if (
          isinstance(manual_auxiliary_next_request_id, bool)
          or not isinstance(manual_auxiliary_next_request_id, int)
          or manual_auxiliary_next_request_id < 0
        ):
          raise RuntimeError("manual auxiliary request counter is invalid")
        request_id = manual_auxiliary_next_request_id + 1
        request = ManualAuxiliaryRequest(
          request_id,
          serial_port,
          command,
          expected_response,
          calibration_key,
          calibration_value,
        )
        manual_auxiliary_next_request_id = request_id
        manual_auxiliary_request_queue.append(request)
  except Exception as exc:
    detail = _manual_auxiliary_error_detail(
      exc,
      "command admission failed without details",
    )
    message = f"Manual auxiliary command rejected: {detail}"
    logger.error(message)
    return _render_manual_auxiliary_rejection(message)
  if queue_full:
    return _render_manual_auxiliary_rejection(
      "AUXILIARY COMMAND QUEUE IS FULL"
    )

  if not _try_dispatch_manual_auxiliary_request():
    with manual_auxiliary_state_lock:
      request_queued = request in manual_auxiliary_request_queue
    if request_queued:
      _set_manual_auxiliary_status(
        "AUXILIARY COMMAND QUEUED",
        "Warn.TLabel",
      )
      return True
    message = "AUXILIARY COMMAND CANCELLED BEFORE DISPATCH"
    logger.warning(message)
    return _render_manual_auxiliary_rejection(message)
  return True


def _request_manual_servo(channel, on_state, entry):
  if (
    isinstance(channel, bool)
    or not isinstance(channel, int)
    or not 0 <= channel < 4
  ):
    raise MotionInputError("manual servo channel must be in [0, 3]")
  if not isinstance(on_state, bool):
    raise TypeError("manual servo state must be boolean")
  get_value = getattr(entry, "get", None)
  if not callable(get_value):
    raise TypeError("manual servo entry must provide get()")
  try:
    value = get_value()
  except Exception as exc:
    detail = _manual_auxiliary_error_detail(
      exc,
      "entry read failed without details",
    )
    message = f"Manual servo entry could not be read: {detail}"
    logger.error(message)
    return _render_manual_auxiliary_rejection(message)
  state_name = "on" if on_state else "off"
  return _queue_manual_auxiliary_command(
    f"SV{channel}P{value}\n",
    f"Servo{channel}{state_name}",
    value,
  )


def _request_manual_output(row, on_state, entry):
  if isinstance(row, bool) or not isinstance(row, int) or not 1 <= row <= 6:
    raise MotionInputError("manual output row must be in [1, 6]")
  if not isinstance(on_state, bool):
    raise TypeError("manual output state must be boolean")
  get_value = getattr(entry, "get", None)
  if not callable(get_value):
    raise TypeError("manual output entry must provide get()")
  try:
    value = get_value()
  except Exception as exc:
    detail = _manual_auxiliary_error_detail(
      exc,
      "entry read failed without details",
    )
    message = f"Manual output entry could not be read: {detail}"
    logger.error(message)
    return _render_manual_auxiliary_rejection(message)
  prefix = "ON" if on_state else "OF"
  state_name = "on" if on_state else "off"
  return _queue_manual_auxiliary_command(
    f"{prefix}X{value}\n",
    f"DO{row}{state_name}",
    value,
  )


def _close_serial_port(serial_name, context="application shutdown"):
  serial_port = RUN.get(serial_name)
  if serial_port is None:
    return True
  try:
    serial_port.close()
  except Exception:
    logger.exception("Unable to close %s during %s", serial_name, context)
    return False
  if getattr(serial_port, "is_open", False):
    logger.error("Serial port %s remained open during %s", serial_name, context)
    return False
  if RUN.get(serial_name) is serial_port:
    RUN[serial_name] = None
    if serial_name == 'ser2':
      _clear_auxiliary_board_profile(serial_port)
  return True


def _interrupt_tracked_serial_activity(serial_name):
  if not serial_activity_registry.active(serial_name):
    return False
  serial_port = RUN.get(serial_name)
  if serial_port is None:
    logger.error(
      "Tracked %s activity has no serial connection to interrupt",
      serial_name,
    )
    return False

  if serial_name not in shutdown_serial_cancel_requested:
    cancel_read = getattr(serial_port, "cancel_read", None)
    if callable(cancel_read):
      try:
        cancel_read()
        shutdown_serial_cancel_requested.add(serial_name)
        logger.warning(
          "Cancelled blocking %s read during application shutdown",
          serial_name,
        )
        return True
      except Exception:
        logger.exception(
          "Unable to cancel blocking %s read during application shutdown",
          serial_name,
        )

  logger.warning(
    "Closing %s to release tracked activity during application shutdown",
    serial_name,
  )
  return _close_serial_port(
    serial_name,
    "tracked activity shutdown interruption",
  )


def _release_async_main_serial_transport(activity_lease, request_lease):
  close_activity = getattr(activity_lease, "close", None)
  if not callable(close_activity):
    raise TypeError("main serial activity lease must be closeable")
  if not serial_lock.locked():
    raise RuntimeError("main serial transport reservation was already released")
  if close_activity() is not True:
    raise RuntimeError("main serial activity lease was already released")
  serial_lock.release()
  _finish_motion_request(request_lease)


def _close_failed_controller_startup(
  serial_port,
  activity_lease,
  request_lease,
):
  if not getattr(serial_port, "is_open", False):
    if RUN.get('ser') is serial_port:
      RUN['ser'] = None
    closed = True
  elif RUN.get('ser') is serial_port:
    closed = _close_serial_port('ser', "failed controller connection cleanup")
  else:
    logger.error("Main serial reference changed during failed connection cleanup")
    if not getattr(serial_port, "is_open", False):
      closed = True
    else:
      try:
        serial_port.close()
        closed = not getattr(serial_port, "is_open", False)
      except Exception:
        logger.exception("Unable to close replaced controller startup connection")
        closed = False
  if not closed:
    return False
  try:
    _release_async_main_serial_transport(activity_lease, request_lease)
  except Exception:
    logger.exception("Unable to release failed controller startup ownership")
    return False
  return True


def _run_startup_controller_cleanup():
  global startup_controller_cleanup_worker

  while True:
    with startup_controller_cleanup_lock:
      pending = tuple(startup_controller_cleanup_pending.items())
      if not pending:
        startup_controller_cleanup_worker = None
        return

    completed_any = False
    for key, entry in pending:
      serial_port, activity_lease, request_lease = entry
      if not _close_failed_controller_startup(
        serial_port,
        activity_lease,
        request_lease,
      ):
        continue
      with startup_controller_cleanup_lock:
        current = startup_controller_cleanup_pending.get(key)
        if (
          isinstance(current, tuple)
          and len(current) == 3
          and current[0] is serial_port
          and current[1] is activity_lease
          and current[2] is request_lease
        ):
          del startup_controller_cleanup_pending[key]
      completed_any = True

    if not completed_any:
      time.sleep(SERIAL_SHUTDOWN_RETRY_MS / 1000.0)


def _ensure_startup_controller_cleanup():
  global startup_controller_cleanup_worker

  with startup_controller_cleanup_lock:
    if not startup_controller_cleanup_pending:
      return True
    worker = startup_controller_cleanup_worker
    if worker is not None and worker.is_alive():
      return True
    try:
      worker = threading.Thread(
        target=_run_startup_controller_cleanup,
        name="ar4-startup-controller-cleanup",
        daemon=True,
      )
      startup_controller_cleanup_worker = worker
      worker.start()
    except Exception:
      startup_controller_cleanup_worker = None
      logger.exception("Unable to start controller startup cleanup retry")
      return False
  return True


def _retain_failed_controller_startup(
  serial_port,
  activity_lease,
  request_lease,
):
  entry = (serial_port, activity_lease, request_lease)
  with startup_controller_cleanup_lock:
    key = id(serial_port)
    existing = startup_controller_cleanup_pending.get(key)
    if existing is not None and not (
      isinstance(existing, tuple)
      and len(existing) == 3
      and existing[0] is serial_port
      and existing[1] is activity_lease
      and existing[2] is request_lease
    ):
      raise RuntimeError("controller startup cleanup identity collision")
    startup_controller_cleanup_pending[key] = entry
  _ensure_startup_controller_cleanup()
  return False


def _poll_failed_controller_close(
  serial_port,
  activity_lease,
  request_lease,
):
  if _close_failed_controller_startup(
    serial_port,
    activity_lease,
    request_lease,
  ):
    return True

  message = "CONTROLLER CONNECTION CLEANUP PENDING"
  try:
    almStatusLab.config(text=message, style="Alarm.TLabel")
    almStatusLab2.config(text=message, style="Alarm.TLabel")
  except Exception:
    logger.exception("Unable to report pending controller connection cleanup")
  try:
    root.after(
      SERIAL_SHUTDOWN_RETRY_MS,
      lambda: _poll_failed_controller_close(
        serial_port,
        activity_lease,
        request_lease,
      ),
    )
  except Exception:
    logger.exception("Unable to schedule controller connection cleanup retry")
    _retain_failed_controller_startup(
      serial_port,
      activity_lease,
      request_lease,
    )
  return False


def _abandon_failed_controller_startup(
  serial_port,
  activity_lease,
  request_lease,
):
  if _close_failed_controller_startup(
    serial_port,
    activity_lease,
    request_lease,
  ):
    return True
  logger.error("Controller startup cleanup retained for a durable retry")
  return _retain_failed_controller_startup(
    serial_port,
    activity_lease,
    request_lease,
  )


def _poll_application_close():
  global application_shutdown_started_at

  _poll_serial_events()
  _poll_calibration_events()
  _poll_auxiliary_serial_events()
  _poll_manual_auxiliary_events()
  _poll_joint_motion_events()
  _poll_virtual_motion_events()

  with startup_controller_cleanup_lock:
    controller_cleanup_pending = bool(startup_controller_cleanup_pending)
  if controller_cleanup_pending:
    _ensure_startup_controller_cleanup()
    root.after(SERIAL_SHUTDOWN_POLL_MS, _poll_application_close)
    return False

  with startup_auxiliary_cleanup_lock:
    auxiliary_cleanup_pending = bool(startup_auxiliary_cleanup_pending)
  if auxiliary_cleanup_pending:
    _ensure_startup_auxiliary_cleanup()
    root.after(SERIAL_SHUTDOWN_POLL_MS, _poll_application_close)
    return False

  calibration_shutdown_pending = _calibration_shutdown_pending()
  calibration_write_committed = calibration_serial_write_committed.is_set()
  if not serial_activity_registry.idle():
    now = time.monotonic()
    if application_shutdown_started_at is None:
      application_shutdown_started_at = now
    elif (
      now - application_shutdown_started_at
      >= SERIAL_SHUTDOWN_ACTIVITY_GRACE_SECONDS
    ):
      for serial_name in ("ser", "ser2"):
        if calibration_write_committed and serial_name == "ser":
          continue
        _interrupt_tracked_serial_activity(serial_name)

  if calibration_shutdown_pending:
    message = "SHUTDOWN WAITING FOR CALIBRATION RESPONSE"
    almStatusLab.config(text=message, style="Warn.TLabel")
    almStatusLab2.config(text=message, style="Warn.TLabel")
    root.after(SERIAL_SHUTDOWN_POLL_MS, _poll_application_close)
    return False

  with offline_live_jog_state_lock:
    offline_motion_pending = offline_live_jog_operation is not None
  if offline_motion_pending or _virtual_motion_active():
    root.after(SERIAL_SHUTDOWN_POLL_MS, _poll_application_close)
    return False

  if not serial_lock.acquire(blocking=False):
    root.after(SERIAL_SHUTDOWN_POLL_MS, _poll_application_close)
    return False
  if not auxiliary_serial_lock.acquire(blocking=False):
    serial_lock.release()
    root.after(SERIAL_SHUTDOWN_POLL_MS, _poll_application_close)
    return False
  if not serial_activity_registry.idle():
    auxiliary_serial_lock.release()
    serial_lock.release()
    root.after(SERIAL_SHUTDOWN_POLL_MS, _poll_application_close)
    return False

  persisted = False
  closed = False
  try:
    joint_motion_dispatcher.close()
    try:
      persisted = _flush_calibration_save() is True
    except Exception:
      logger.exception("Unable to flush final controller state during shutdown")
    if persisted:
      closed = _close_serial_port('ser')
      closed = _close_serial_port('ser2') and closed
  finally:
    auxiliary_serial_lock.release()
    serial_lock.release()

  if not persisted:
    almStatusLab.config(
      text="SHUTDOWN WAITING FOR CALIBRATION SAVE",
      style="Alarm.TLabel",
    )
    almStatusLab2.config(
      text="SHUTDOWN WAITING FOR CALIBRATION SAVE",
      style="Alarm.TLabel",
    )
    root.after(SERIAL_SHUTDOWN_RETRY_MS, _poll_application_close)
    return False

  if not closed:
    almStatusLab.config(
      text="SHUTDOWN WAITING FOR SERIAL CLOSE",
      style="Alarm.TLabel",
    )
    almStatusLab2.config(
      text="SHUTDOWN WAITING FOR SERIAL CLOSE",
      style="Alarm.TLabel",
    )
    root.after(SERIAL_SHUTDOWN_RETRY_MS, _poll_application_close)
    return False

  cv2.destroyAllWindows()
  root.quit()
  root.destroy()
  return True

'''
RUN['J1CalStat1'] = IntVar()
RUN['J2CalStat1'] = IntVar()
RUN['J3CalStat1'] = IntVar()
RUN['J4CalStat1'] = IntVar()
RUN['J5CalStat1'] = IntVar()
RUN['J6CalStat1'] = IntVar()
RUN['J8CalStat1'] = IntVar()
RUN['J9CalStat1'] = IntVar()
RUN['J7CalStat1'] = IntVar()

RUN['J1CalStat2'] = IntVar()
RUN['J2CalStat2'] = IntVar()
RUN['J3CalStat2'] = IntVar()
RUN['J4CalStat2'] = IntVar()
RUN['J5CalStat2'] = IntVar()
RUN['J6CalStat2'] = IntVar()
RUN['J7CalStat2'] = IntVar()
RUN['J8CalStat2'] = IntVar()
RUN['J9CalStat2'] = IntVar()
'''

RUN['IncJogStat'] = IntVar()
RUN['fullRot'] = IntVar()
RUN['pick180'] = IntVar()
RUN['pickClosest'] = IntVar()
RUN['autoBG'] = IntVar()
RUN['estopActive'] = False
RUN['posOutreach'] = False
RUN['programStopStatusLatched'] = False
RUN['gcodeSpeed'] = "10"

RUN['inchTrue'] = False
RUN['moveInProc'] = 0
RUN['liveJog'] = False
RUN['progRunning'] = False
RUN['offlineMode'] = False

RUN['color_map'] = {}

RUN['J1StepM'] = None
RUN['J2StepM'] = None
RUN['J3StepM'] = None
RUN['J4StepM'] = None
RUN['J5StepM'] = None
RUN['J6StepM'] = None

RUN['oriImage'] = None
RUN['StepMonitors'] = [0] * 6
RUN['minSpeedDelay'] = 200 #µs
RUN['speedViolation'] = "0"

RUN['xyzuvw_In'] = np.zeros(6)
RUN['KinematicError'] = 0

RUN['cam_on'] = False
RUN['cap'] = None

# Migrated global variables to RUN dictionary
# Robot State & Control
RUN['Alarm'] = None
RUN['VR_angles'] = None
RUN['JangleOut'] = None
RUN['JstepCur'] = None
RUN['JointMin'] = None
RUN['JointMax'] = None
RUN['cur_steps'] = None
RUN['J1axisLimNeg'] = None
RUN['J2axisLimNeg'] = None
RUN['J3axisLimNeg'] = None
RUN['J4axisLimNeg'] = None
RUN['J5axisLimNeg'] = None
RUN['J6axisLimNeg'] = None
RUN['negLim'] = None
RUN['stepDeg'] = None
RUN['flag'] = None
RUN['LineDist'] = None
RUN['Xv'] = None
RUN['Yv'] = None
RUN['Zv'] = None
RUN['xVal'] = None
RUN['yVal'] = None
RUN['zVal'] = None

# Serial Communication
RUN['ser'] = None
RUN['ser2'] = None
RUN['ser2BoardProfile'] = None
RUN['ser3'] = None

# Program Execution
RUN['rowinproc'] = None
RUN['GCrowinproc'] = None

# Live Jog State
RUN['_current'] = None
RUN['_pending_start'] = None
RUN['_cart_current'] = None
RUN['_cart_pending'] = None
RUN['_tool_current'] = None
RUN['_tool_pending'] = None
RUN['_last_input_time'] = None
RUN['_mainMode'] = None
RUN['_smooth'] = None
RUN['_grip_closed'] = None
RUN['_pneu_open'] = None
RUN['_grip_pending_request_id'] = None
RUN['_pneu_pending_request_id'] = None
RUN['cmdType'] = None
RUN['cmdTypeLong'] = None

# Vision System
RUN['cropping'] = False
RUN['button_down'] = None
RUN['box_points'] = None
RUN['x_start'] = None
RUN['y_start'] = None
RUN['x_end'] = None
RUN['y_end'] = None
RUN['mX1'] = None
RUN['mY1'] = None
RUN['mX2'] = None
RUN['mY2'] = None
RUN['prevxVal'] = None
RUN['prevyVal'] = None
RUN['prevzVal'] = None
RUN['xMMpos'] = None
RUN['yMMpos'] = None
RUN['BGavg'] = None

# 3D Visualization
RUN['vtk_running'] = False
RUN['actors'] = {}
RUN['assemblies'] = {}
RUN['base_transforms'] = {}
RUN['joint_transforms'] = {}
RUN['composite_transforms'] = {}
RUN['interactor'] = None
RUN['render_window'] = None

# Input Devices
RUN['xboxUse'] = None
RUN['selectedTemplate'] = None
RUN['selectedCam'] = None


#declare axis limit vars
#! These are probably not necesary anymore but verify
CAL['J1PosLim'] = 0
CAL['J1NegLim'] = 0
CAL['J2PosLim'] = 0
CAL['J2NegLim'] = 0
CAL['J3PosLim'] = 0
CAL['J3NegLim'] = 0
CAL['J4PosLim'] = 0
CAL['J4NegLim'] = 0
CAL['J5PosLim'] = 0
CAL['J5NegLim'] = 0
CAL['J6PosLim'] = 0
CAL['J6NegLim'] = 0
CAL['J7PosLim'] = 0
CAL['J1CalStatVal'] = tk.IntVar(value=0)
CAL['J2CalStatVal'] = tk.IntVar(value=0)
CAL['J3CalStatVal'] = tk.IntVar(value=0)
CAL['J4CalStatVal'] = tk.IntVar(value=0)
CAL['J5CalStatVal'] = tk.IntVar(value=0)
CAL['J6CalStatVal'] = tk.IntVar(value=0)
CAL['J7CalStatVal'] = tk.IntVar(value=0)
CAL['J8CalStatVal'] = tk.IntVar(value=0)
CAL['J9CalStatVal'] = tk.IntVar(value=0)
CAL['J1CalStatVal2'] = tk.IntVar(value=0)
CAL['J2CalStatVal2'] = tk.IntVar(value=0)
CAL['J2CalStatVal2'] = tk.IntVar(value=0)
CAL['J3CalStatVal2'] = tk.IntVar(value=0)
CAL['J4CalStatVal2'] = tk.IntVar(value=0)
CAL['J5CalStatVal2'] = tk.IntVar(value=0)
CAL['J6CalStatVal2'] = tk.IntVar(value=0)
CAL['J7CalStatVal2'] = tk.IntVar(value=0)
CAL['J8CalStatVal2'] = tk.IntVar(value=0)
CAL['J9CalStatVal2'] = tk.IntVar(value=0)
CAL['J1OpenLoopVal'] = tk.IntVar(value=0)
CAL['J2OpenLoopVal'] = tk.IntVar(value=0)
CAL['J3OpenLoopVal'] = tk.IntVar(value=0)
CAL['J4OpenLoopVal'] = tk.IntVar(value=0)
CAL['J5OpenLoopVal'] = tk.IntVar(value=0)
CAL['J6OpenLoopVal'] = tk.IntVar(value=0)
CAL['DisableWristRotVal'] = tk.IntVar(value=0)



#J7NegLim = 0
#J8PosLim = 0
#J8NegLim = 0
#J9PosLim = 0
#J9NegLim = 0


#############################################################################################
### KINEMATICS FOR VIR ROBOT ################################################################
#############################################################################################

#DEG2RAD = np.pi / 180
#RAD2DEG = 180 / np.pi

def _validated_native_ordered_values(values, length, label):
    if not isinstance(values, (list, tuple)):
        raise MotionInputError(f"{label} must be an ordered numeric sequence")
    if len(values) != length:
        count = "six" if length == 6 else str(length)
        raise MotionInputError(f"{label} must contain {count} values")
    return tuple(
        finite_number(value, f"{label} field {index}")
        for index, value in enumerate(values, start=1)
    )


def _validated_native_tool_frame(values, label):
    tool_frame = tuple(
        controller_number(values[key], f"{label} {key}")
        for key in ('TFx', 'TFy', 'TFz', 'TFrx', 'TFry', 'TFrz')
    )
    for key, degrees in zip(('TFrx', 'TFry', 'TFrz'), tool_frame[3:]):
        controller_degree_to_native_radians(
            degrees,
            f"{label} {key}",
        )
    return tool_frame


def _validated_native_kinematics_rotations(values):
    theta = tuple(
        controller_degree_to_native_radians(
            values[f'J{axis}ΘDHpar'],
            f"J{axis} theta",
        )
        for axis in range(1, 7)
    )
    alpha = tuple(
        controller_degree_to_native_radians(
            values[f'J{axis}αDHpar'],
            f"J{axis} alpha",
        )
        for axis in range(1, 7)
    )
    tool_frame = _validated_native_tool_frame(values, "tool-frame")
    return theta, alpha, tool_frame


def _active_tool_frame():
    return _validated_native_tool_frame(CAL, "active tool-frame")


def _prepare_cpp_kinematics_configuration(values):
    (
        theta,
        alpha,
        tool_frame,
    ) = _validated_native_kinematics_rotations(values)
    link_a = tuple(
        controller_number(values[f'J{axis}aDHpar'], f"J{axis} link a")
        for axis in range(1, 7)
    )
    link_d = tuple(
        controller_number(values[f'J{axis}dDHpar'], f"J{axis} link d")
        for axis in range(1, 7)
    )
    positive_limits = tuple(
        controller_number(
            values[f'J{axis}PosLim'],
            f"J{axis} positive limit",
        )
        for axis in range(1, 7)
    )
    negative_limits = tuple(
        controller_number(
            values[f'J{axis}NegLim'],
            f"J{axis} negative limit",
        )
        for axis in range(1, 7)
    )
    for axis, (positive, negative) in enumerate(
        zip(positive_limits, negative_limits),
        start=1,
    ):
        if positive < 0 or negative < 0:
            raise MotionInputError(
                f"J{axis} joint limits must be non-negative magnitudes"
            )

    configuration_writer = getattr(robot, 'set_robot_configuration', None)
    configured_solver = getattr(robot, 'SolveInverseKinematicsConfigured', None)
    missing = tuple(
        name
        for name, function in (
            ('set_robot_configuration', configuration_writer),
            ('SolveInverseKinematicsConfigured', configured_solver),
        )
        if not callable(function)
    )
    if missing:
        raise MotionInputError(
            "native kinematics module lacks the required configured API: "
            + ", ".join(missing)
        )
    return (
        configuration_writer,
        theta + alpha + link_a + link_d,
        positive_limits,
        negative_limits,
        tool_frame,
    )


def _set_cpp_kinematics_from_values(values):
    kinematics_configuration_ready.clear()
    (
        configuration_writer,
        dh_parameters,
        positive_limits,
        negative_limits,
        tool_frame,
    ) = _prepare_cpp_kinematics_configuration(values)
    configuration = (
        dh_parameters,
        positive_limits,
        negative_limits,
        tool_frame,
    )
    configuration_writer(*configuration)
    kinematics_configuration_ready.set()
    return True


def update_CPP_kin_from_entries():
    try:
        values = dict(CAL)
        entry_groups = (
            ('ΘDHpar', (J1ΘEntryField, J2ΘEntryField, J3ΘEntryField, J4ΘEntryField, J5ΘEntryField, J6ΘEntryField)),
            ('αDHpar', (J1αEntryField, J2αEntryField, J3αEntryField, J4αEntryField, J5αEntryField, J6αEntryField)),
            ('aDHpar', (J1aEntryField, J2aEntryField, J3aEntryField, J4aEntryField, J5aEntryField, J6aEntryField)),
            ('dDHpar', (J1dEntryField, J2dEntryField, J3dEntryField, J4dEntryField, J5dEntryField, J6dEntryField)),
        )
        for suffix, fields in entry_groups:
            for axis, field in enumerate(fields, start=1):
                values[f'J{axis}{suffix}'] = field.get()
        values.update({
            'TFx': TFxEntryField.get(),
            'TFy': TFyEntryField.get(),
            'TFz': TFzEntryField.get(),
            'TFrx': TFrxEntryField.get(),
            'TFry': TFryEntryField.get(),
            'TFrz': TFrzEntryField.get(),
        })
        return _set_cpp_kinematics_from_values(values)
    except (KeyError, TypeError, ValueError, MotionInputError) as exc:
        logger.error("Invalid parameter input: %s", exc)
        return False


def setStepMonitorsVR():
    #global StepMonitors
    #global J1StepM, J2StepM, J3StepM, J4StepM, J5StepM, J6StepM
    # global RUN['VR_angles']
    RUN['StepMonitors'][0] = (float(RUN['VR_angles'][0]) + float(CAL['J1NegLim'])) * float(CAL['J1StepDeg'])
    RUN['StepMonitors'][1] = (float(RUN['VR_angles'][1]) + float(CAL['J2NegLim'])) * float(CAL['J2StepDeg'])
    RUN['StepMonitors'][2] = (float(RUN['VR_angles'][2]) + float(CAL['J3NegLim'])) * float(CAL['J3StepDeg'])
    RUN['StepMonitors'][3] = (float(RUN['VR_angles'][3]) + float(CAL['J4NegLim'])) * float(CAL['J4StepDeg'])                                              
    RUN['StepMonitors'][4] = (float(RUN['VR_angles'][4]) + float(CAL['J5NegLim'])) * float(CAL['J5StepDeg'])
    RUN['StepMonitors'][5] = (float(RUN['VR_angles'][5]) + float(CAL['J6NegLim'])) * float(CAL['J6StepDeg'])
    RUN['J1StepM'] = RUN['StepMonitors'][0]
    RUN['J2StepM'] = RUN['StepMonitors'][1]
    RUN['J3StepM'] = RUN['StepMonitors'][2]
    RUN['J4StepM'] = RUN['StepMonitors'][3]
    RUN['J5StepM'] = RUN['StepMonitors'][4]
    RUN['J6StepM'] = RUN['StepMonitors'][5]

                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    
def _joint_entry_is_being_edited(entry):
    return getattr(entry, "_joint_target_editing", False) is True


def _write_joint_position_entry(entry, value):
    if _joint_entry_is_being_edited(entry):
        return False
    entry.delete(0, 'end')
    entry.insert(0, value)
    return True


def _reset_joint_position_entry(entry, value):
    entry._joint_target_editing = False
    entry._joint_target_replace_on_key = True
    entry._joint_target_pointer_focus = False
    entry.delete(0, 'end')
    entry.insert(0, value)
    return True


def refresh_gui_from_joint_angles(joint_angles):
    try:
        joints = _validated_virtual_six_vector(
            joint_angles,
            "virtual GUI refresh joints",
        )
        xyzuvw = _forward_kinematics_display_pose(
            joints,
            "virtual GUI refresh Cartesian pose",
        )
    except Exception as exc:
        raise MotionInputError(f"Forward kinematics refresh failed: {exc}") from exc

    RUN['VR_angles'] = list(joints)
    setStepMonitorsVR()

    cartesian_values = tuple(f"{value:.3f}" for value in xyzuvw)
    (
        CAL['XcurPos'],
        CAL['YcurPos'],
        CAL['ZcurPos'],
        CAL['RzcurPos'],
        CAL['RycurPos'],
        CAL['RxcurPos'],
    ) = cartesian_values
    for field, value in zip(
        (
            XcurEntryField,
            YcurEntryField,
            ZcurEntryField,
            RzcurEntryField,
            RycurEntryField,
            RxcurEntryField,
        ),
        cartesian_values,
    ):
        field.delete(0, 'end')
        field.insert(0, value)

    joint_values = tuple(str(value) for value in joints)
    (
        CAL['J1AngCur'],
        CAL['J2AngCur'],
        CAL['J3AngCur'],
        CAL['J4AngCur'],
        CAL['J5AngCur'],
        CAL['J6AngCur'],
    ) = joint_values
    RUN['WC'] = "F" if joints[4] > 0 else "N"
    logger.info(CAL['J5AngCur'])

    for field, slider, value in zip(
        (
            J1curAngEntryField,
            J2curAngEntryField,
            J3curAngEntryField,
            J4curAngEntryField,
            J5curAngEntryField,
            J6curAngEntryField,
        ),
        (
            J1jogslide,
            J2jogslide,
            J3jogslide,
            J4jogslide,
            J5jogslide,
            J6jogslide,
        ),
        joint_values,
    ):
        _write_joint_position_entry(field, value)
        slider.set(value)
    return True










#############################################################################################
### MOVE LOGIC FOR VIRTUAL ROBOT ############################################################
#############################################################################################


def start_driveMotorsJ_thread(*args):
    if not drive_lock.acquire(blocking=False):
        logger.info("Drive already in progress; command rejected")
        return False
    operation = VirtualMotionOperation()
    try:
        thread = threading.Thread(
            target=run_driveMotorsJ_safe,
            args=(operation,) + args,
            daemon=True,
        )
        thread.start()
    except Exception:
        drive_lock.release()
        raise
    return operation

def run_driveMotorsJ_safe(operation, *args):
    if not isinstance(operation, VirtualMotionOperation):
        raise TypeError("virtual drive operation has an invalid type")
    error = None
    try:
        driveMotorsJ(*args)
    except BaseException as exc:
        detail = str(exc).strip() or type(exc).__name__
        error = f"{type(exc).__name__}: {detail}"
        logger.exception("Virtual drive execution failed")
    finally:
        try:
            drive_lock.release()
        except BaseException as exc:
            detail = str(exc).strip() or type(exc).__name__
            if error is None:
                error = f"{type(exc).__name__}: {detail}"
            logger.exception("Virtual drive ownership release failed")
        if error is None:
            operation.complete(True)
        else:
            operation.complete(False, error)


def _validated_virtual_six_vector(values, label):
    try:
        vector = tuple(values)
    except TypeError as exc:
        raise MotionInputError(f"{label} must be a six-value numeric sequence") from exc
    if len(vector) != 6:
        raise MotionInputError(f"{label} must contain six values")
    return tuple(
        finite_number(value, f"{label} field {index}")
        for index, value in enumerate(vector, start=1)
    )


def _external_cartesian_pose_to_native(values, label):
    external = _validated_virtual_six_vector(values, label)
    # Display and line protocol use Rz/Ry/Rx; native kinematics uses Rx/Ry/Rz.
    return external[:3] + (external[5], external[4], external[3])


def _native_cartesian_pose_to_external(values, label):
    native = _validated_virtual_six_vector(values, label)
    return native[:3] + (native[5], native[4], native[3])


def _forward_kinematics_display_pose(joint_angles, label):
    native_radians = _validated_virtual_six_vector(
        robot.forward_kinematics(joint_angles),
        label,
    )
    native_degrees = native_radians[:3] + tuple(
        math.degrees(value) for value in native_radians[3:]
    )
    return _native_cartesian_pose_to_external(native_degrees, label)


IK_POSITION_TOLERANCE_MILLIMETRES = 0.1
IK_ROTATION_TOLERANCE_DEGREES = 0.1
IK_JOINT_LIMIT_TOLERANCE_DEGREES = 0.001
IK_WRIST_SINGULARITY_DEGREES = 2.0


def _validated_wrist_config(value):
    if not isinstance(value, str):
        raise MotionInputError("wrist configuration must be text")
    wrist_config = value.strip().upper()
    if wrist_config not in ('A', 'F', 'N'):
        raise MotionInputError("wrist configuration must be A, F, or N")
    return wrist_config


def _rotation_vector_matrix(rotation_vector):
    vector = _validated_virtual_six_vector(
        (0.0, 0.0, 0.0, *rotation_vector),
        "rotation vector",
    )[3:]
    angle = math.sqrt(sum(component * component for component in vector))
    if angle <= 1e-12:
        return (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)
    x, y, z = (component / angle for component in vector)
    sine = math.sin(angle)
    cosine = math.cos(angle)
    complement = 1.0 - cosine
    return (
        x * x * complement + cosine,
        x * y * complement - z * sine,
        x * z * complement + y * sine,
        y * x * complement + z * sine,
        y * y * complement + cosine,
        y * z * complement - x * sine,
        z * x * complement - y * sine,
        z * y * complement + x * sine,
        z * z * complement + cosine,
    )


def _rotation_error_degrees(target_rotation, actual_rotation):
    target_matrix = _rotation_vector_matrix(target_rotation)
    actual_matrix = _rotation_vector_matrix(actual_rotation)
    chord = math.sqrt(sum(
        (actual - target) ** 2
        for actual, target in zip(actual_matrix, target_matrix)
    ))
    half_sine = min(1.0, chord / math.sqrt(8.0))
    return math.degrees(2.0 * math.asin(half_sine))


def _validate_inverse_kinematics_result(target, result, wrist_config):
    if result is None:
        return None
    if isinstance(result, (list, tuple)) and not result:
        return None
    joints = _validated_native_ordered_values(
        result,
        6,
        "inverse kinematics result",
    )

    for axis, joint in enumerate(joints, start=1):
        positive = controller_number(
            CAL[f'J{axis}PosLim'],
            f"J{axis} positive limit",
        )
        negative = controller_number(
            CAL[f'J{axis}NegLim'],
            f"J{axis} negative limit",
        )
        if positive < 0 or negative < 0:
            raise MotionInputError(
                f"J{axis} joint limits must be non-negative magnitudes"
            )
        if (
            joint < -negative - IK_JOINT_LIMIT_TOLERANCE_DEGREES
            or joint > positive + IK_JOINT_LIMIT_TOLERANCE_DEGREES
        ):
            raise MotionInputError(
                f"inverse kinematics result exceeds J{axis} limits"
            )

    if abs(joints[4]) > IK_WRIST_SINGULARITY_DEGREES:
        if wrist_config == 'F' and joints[4] < 0:
            raise MotionInputError(
                "inverse kinematics result violates the F wrist branch"
            )
        if wrist_config == 'N' and joints[4] > 0:
            raise MotionInputError(
                "inverse kinematics result violates the N wrist branch"
            )

    actual_pose = _validated_native_ordered_values(
        robot.forward_kinematics(joints),
        6,
        "inverse kinematics round-trip pose",
    )
    position_error = math.sqrt(sum(
        (actual - expected) ** 2
        for actual, expected in zip(actual_pose[:3], target[:3])
    ))
    rotation_error = _rotation_error_degrees(
        tuple(math.radians(value) for value in target[3:]),
        actual_pose[3:],
    )
    if position_error > IK_POSITION_TOLERANCE_MILLIMETRES:
        raise MotionInputError(
            "inverse kinematics result failed Cartesian position validation"
        )
    if rotation_error > IK_ROTATION_TOLERANCE_DEGREES:
        raise MotionInputError(
            "inverse kinematics result failed Cartesian rotation validation"
        )
    return joints


def _solve_inverse_kinematics(target, estimate, wrist_config):
    target_vector = _validated_virtual_six_vector(
        target,
        "inverse kinematics target",
    )
    estimate_vector = _validated_virtual_six_vector(
        estimate,
        "inverse kinematics estimate",
    )
    wrist_config = _validated_wrist_config(wrist_config)

    configured_solver = getattr(
        robot,
        'SolveInverseKinematicsConfigured',
        None,
    )
    if not callable(configured_solver):
        raise MotionInputError(
            "native kinematics module lacks the required configured solver"
        )
    result = configured_solver(
        target_vector,
        estimate_vector,
        wrist_config,
    )

    return _validate_inverse_kinematics_result(
        target_vector,
        result,
        wrist_config,
    )


def driveMotorsJ(
    J1step, J2step, J3step, J4step, J5step, J6step,
    J1dir, J2dir, J3dir, J4dir, J5dir, J6dir,
    SpeedType, SpeedVal, ACCspd, DCCspd, ACCramp,
    cartesian_start=None, cartesian_target=None):

    
    #global J1StepM, J2StepM, J3StepM, J4StepM, J5StepM, J6StepM
    #global xyzuvw_In
    timing_start = None
    timing_target = None
    if SpeedType == "m":
        timing_start = _validated_virtual_six_vector(
            cartesian_start,
            "Cartesian timing start",
        )
        timing_target = _validated_virtual_six_vector(
            cartesian_target,
            "Cartesian timing target",
        )

    steps = [int(round(J1step)), int(round(J2step)), int(round(J3step)),
             int(round(J4step)), int(round(J5step)), int(round(J6step))]
    dirs = [J1dir, J2dir, J3dir, J4dir, J5dir, J6dir]
    RUN['StepMonitors'] = [RUN['J1StepM'], RUN['J2StepM'], RUN['J3StepM'], RUN['J4StepM'], RUN['J5StepM'], RUN['J6StepM']]
    prev_StepMonitors = RUN['StepMonitors'].copy()

    cur = [0] * 6
    PE = [0] * 6
    SE_1 = [0] * 6
    SE_2 = [0] * 6
    LO_1 = [0] * 6
    LO_2 = [0] * 6
    PEcur = [0] * 6
    SE_1cur = [0] * 6
    SE_2cur = [0] * 6

    HighStep = max(steps)
    time.sleep(15e-6)


    if live_cartesian_lock.locked():
        virOffset = VIRTUAL_CARTESIAN_SECONDS_SCALE
    elif live_tool_lock.locked():
        virOffset = VIRTUAL_TOOL_SECONDS_SCALE
    else:
        virOffset = VIRTUAL_JOINT_SECONDS_SCALE

    SpeedVal = SpeedVal * virOffset
    #ACCspd   = ACCspd   * virOffset
    #DCCspd   = DCCspd   * virOffset
    #ACCramp  = ACCramp  * virOffset

    # Steps in each region
    ACCStep = HighStep * (ACCspd / 100.0)
    DCCStep = HighStep * (DCCspd / 100.0)
    NORStep = HighStep - ACCStep - DCCStep

    # Target total time in microseconds (no 1.2 fudge)
    speedSP = 0.0
    if SpeedType == "s":
        speedSP = SpeedVal * 1_000_000.0
    elif SpeedType == "m":
        dx = timing_target[0] - timing_start[0]
        dy = timing_target[1] - timing_start[1]
        dz = timing_target[2] - timing_start[2]
        lineDist = math.sqrt(dx*dx + dy*dy + dz*dz)
        # seconds = distance / (mm/s)
        speedSP = (lineDist / SpeedVal) * 1_000_000.0

    # fixed ramp factors (start/end slower than cruise), same as Teensy:
    # if(ACCramp < 10){ ACCramp = 10; }  k_* = ACCramp / 10
    if ACCramp < 10.0:
        ACCramp = 10.0
    k_acc = ACCramp / 10.0
    k_dec = ACCramp / 10.0

    # Solve cruise delay for trapezoid
    if SpeedType in ("s", "m") and speedSP > 0.0:
        # T = cruise * [ NORStep + (ACCStep*(1+k_acc) + DCCStep*(1+k_dec))/2 ]
        denom = NORStep + 0.5 * (ACCStep * (1.0 + k_acc) + DCCStep * (1.0 + k_dec))
        if denom <= 0.0:
            calcStepGap = speedSP / max(float(HighStep), 1.0)
        else:
            calcStepGap = speedSP / denom

        if calcStepGap < RUN['minSpeedDelay']:
            calcStepGap = RUN['minSpeedDelay']
            try:
                RUN['speedViolation'] = "1"
            except NameError:
                pass  # only if your Python sim doesn't use this flag
    elif SpeedType == "p":
        calcStepGap = RUN['minSpeedDelay'] / (SpeedVal / 100.0)
    else:
        calcStepGap = RUN['minSpeedDelay']

    # With cruise known, define start/end delays and per-step increments
    startDelay = calcStepGap * k_acc  # slower than cruise
    endDelay   = calcStepGap * k_dec  # slower than cruise

    # Linear ramps
    calcACCstepInc = ((startDelay - calcStepGap) / ACCStep) if ACCStep > 0.0 else 0.0  # subtract each accel step
    calcDCCstepInc = ((endDelay   - calcStepGap) / DCCStep) if DCCStep > 0.0 else 0.0  # add each decel step

    # Start delay
    calcACCstartDel = startDelay
    curDelay = calcACCstartDel
    highStepCur = 0


    while any(cur[i] < steps[i] for i in range(6)):
        

        if highStepCur <= ACCStep:
            curDelay -= calcACCstepInc
        elif highStepCur >= (HighStep - DCCStep):
            curDelay += calcDCCstepInc
        else:
            curDelay = calcStepGap

        distDelay = 30
        disDelayCur = 0

        for i in range(6):
            if cur[i] < steps[i]:
                PE[i] = HighStep // steps[i]
                LO_1[i] = HighStep - (steps[i] * PE[i])
                SE_1[i] = (HighStep // LO_1[i]) if LO_1[i] > 0 else 0
                LO_2[i] = (HighStep - ((steps[i] * PE[i]) + ((steps[i] * PE[i]) // SE_1[i]))) if SE_1[i] > 0 else 0
                SE_2[i] = (HighStep // LO_2[i]) if LO_2[i] > 0 else 0

                if SE_2[i] == 0:
                    SE_2cur[i] = SE_2[i] + 1

                if SE_2cur[i] != SE_2[i]:
                    SE_2cur[i] += 1
                    if SE_1[i] == 0:
                        SE_1cur[i] = SE_1[i] + 1

                    if SE_1cur[i] != SE_1[i]:
                        SE_1cur[i] += 1
                        PEcur[i] += 1

                        if PEcur[i] == PE[i]:
                            cur[i] += 1
                            PEcur[i] = 0
                            time.sleep(distDelay / 1_000_000)
                            disDelayCur += distDelay
                            RUN['StepMonitors'][i] += 1 if dirs[i] else -1
                            RUN['VR_angles'][i] = (RUN['StepMonitors'][i] / RUN['stepDeg'][i]) - RUN['negLim'][i]

                            if RUN['StepMonitors'][i] != prev_StepMonitors[i]:
                                prev_StepMonitors[i] = RUN['StepMonitors'][i]
                    else:
                        SE_1cur[i] = 0
                else:
                    SE_2cur[i] = 0

        highStepCur += 1
        time.sleep(max((curDelay - disDelayCur), 0) / 1_000_000)

    RUN['J1StepM'], RUN['J2StepM'], RUN['J3StepM'], RUN['J4StepM'], RUN['J5StepM'], RUN['J6StepM'] = RUN['StepMonitors']


def parse_mj_command(inData):
    try:
        normalized = canonicalize_virtual_command(inData)
    except MotionInputError as exc:
        logger.error("MJ command parse failed: %s", exc)
        return None
    pattern = r"X([-+]?[0-9.]+)Y([-+]?[0-9.]+)Z([-+]?[0-9.]+)Rz([-+]?[0-9.]+)Ry([-+]?[0-9.]+)Rx([-+]?[0-9.]+)S[psm]([-+]?[0-9.]+)Ac([-+]?[0-9.]+)Dc([-+]?[0-9.]+)Rm([-+]?[0-9.]+)"
    match = re.search(pattern, normalized)
    if not match:
        logger.error("MJ command parse failed")
        return None

    vals = [float(v) for v in match.groups()]
    return {
        "xyzuvw": _external_cartesian_pose_to_native(
            vals[:6],
            "MJ Cartesian pose",
        ),
        "SpeedType": normalized[normalized.find("S") + 1],
        "Speed": vals[6],
        "Acc": vals[7],
        "Dec": vals[8],
        "Ramp": vals[9],
        "WristConfig": parse_motion_wrist_config(normalized, virtual=True),
    }


def _vision_rotation_degrees(in_data):
    normalized = canonicalize_virtual_command(in_data)
    vision_start = normalized.find("Vr")
    loop_mode_start = normalized.find("Lm", vision_start + 2)
    if vision_start < 0 or loop_mode_start <= vision_start + 2:
        raise MotionInputError("MV command is missing the vision rotation")
    value = controller_number(
        normalized[vision_start + 2:loop_mode_start],
        "MV vision rotation",
    )
    controller_degree_to_native_radians(value, "MV vision rotation")
    return value


@contextmanager
def _temporary_vision_tool_rotation(vision_rotation_degrees):
    original_tool_frame = _validated_virtual_six_vector(
        robot.get_robot_tool_frame(),
        "MV tool frame",
    )
    adjusted_tool_frame = list(original_tool_frame)
    adjusted_tool_frame[3] = controller_number(
        adjusted_tool_frame[3] - vision_rotation_degrees,
        "MV adjusted tool Rx",
    )
    controller_degree_to_native_radians(
        adjusted_tool_frame[3],
        "MV adjusted tool Rx",
    )
    robot.set_robot_tool_frame(*adjusted_tool_frame)
    try:
        yield
    finally:
        robot.set_robot_tool_frame(*original_tool_frame)

def parse_mt_command(inData):
    try:
        normalized = canonicalize_virtual_command(inData)
    except MotionInputError as exc:
        logger.error("Tool jog command parse failed: %s", exc)
        return None
    axis_map = {
        'JTX': 0, 'JTY': 1, 'JTZ': 2,
        'JTW': 3, 'JTP': 4, 'JTR': 5
    }

    # Extract axis and direction (e.g., JTX1 or JTP0)
    axis_match = re.search(r'(JT[XYZRPW])([01])([-+]?[0-9.]+)', normalized)
    if not axis_match:
        logger.error("Tool jog command parse failed (axis part)")
        return None

    axis_str = axis_match.group(1)
    direction = int(axis_match.group(2))
    value = float(axis_match.group(3))

    if axis_str not in axis_map:
        logger.warning(f"Unknown axis code: {axis_str}")
        return None

    axis_index = axis_map[axis_str]
    offset_vector = [0.0] * 6
    offset_vector[axis_index] = value if direction == 1 else -value

    # Extract speed and ramp values
    try:
        timing = parse_virtual_command_timing(normalized)
        SpeedType = timing.mode
        Speed = timing.speed
        Acc = timing.acceleration
        Dec = timing.deceleration
        Ramp = timing.ramp
        LoopMode = normalized.split("Lm")[1].strip()
        wrist_config = parse_motion_wrist_config(normalized, virtual=True)
    except Exception as e:
        logger.error(f"Tool jog command parse failed (parameters): {e}")
        return None

    return {
        "offset_vector": offset_vector,
        "SpeedType": SpeedType,
        "Speed": Speed,
        "Acc": Acc,
        "Dec": Dec,
        "Ramp": Ramp,
        "LoopMode": LoopMode,
        "WristConfig": wrist_config,
    }


def rj_command(in_data):
    #global J1StepM, J2StepM, J3StepM, J4StepM, J5StepM, J6StepM
    # global RUN['cur_steps'], RUN['Alarm']

    # Find start positions
    Jidx = {label: in_data.find(label) for label in ['A', 'B', 'C', 'D', 'E', 'F']}
    SPstart = in_data.find("S")
    AcStart = in_data.find("Ac")
    DcStart = in_data.find("Dc")
    RmStart = in_data.find("Rm")
    WristConStart = in_data.find("W")
    LoopModeStart = in_data.find("Lm")

    # Parse joint angles
    Jangles = [
        float(in_data[Jidx['A']+1:Jidx['B']]),
        float(in_data[Jidx['B']+1:Jidx['C']]),
        float(in_data[Jidx['C']+1:Jidx['D']]),
        float(in_data[Jidx['D']+1:Jidx['E']]),
        float(in_data[Jidx['E']+1:Jidx['F']]),
        float(in_data[Jidx['F']+1:SPstart]),
    ]

    SpeedType = in_data[SPstart + 1]
    SpeedVal = float(in_data[SPstart + 2:AcStart])
    ACCspd = float(in_data[AcStart + 2:DcStart])
    DCCspd = float(in_data[DcStart + 2:RmStart])
    ACCramp = float(in_data[RmStart + 2:WristConStart])
    WristCon = in_data[WristConStart + 1:LoopModeStart]
    LoopMode = in_data[LoopModeStart + 2:].strip()
    LoopModes = list(map(int, list(LoopMode)))

    fut_steps = [
    int(round((Jangles[0] + RUN['J1axisLimNeg']) * float(CAL['J1StepDeg']))),
    int(round((Jangles[1] + RUN['J2axisLimNeg']) * float(CAL['J2StepDeg']))),
    int(round((Jangles[2] + RUN['J3axisLimNeg']) * float(CAL['J3StepDeg']))),
    int(round((Jangles[3] + RUN['J4axisLimNeg']) * float(CAL['J4StepDeg']))),
    int(round((Jangles[4] + RUN['J5axisLimNeg']) * float(CAL['J5StepDeg']))),
    int(round((Jangles[5] + RUN['J6axisLimNeg']) * float(CAL['J6StepDeg']))),
    ]


    RUN['cur_steps'] = [RUN['J1StepM'], RUN['J2StepM'], RUN['J3StepM'], RUN['J4StepM'], RUN['J5StepM'], RUN['J6StepM']]
    step_degs = [CAL['J1StepDeg'], CAL['J2StepDeg'], CAL['J3StepDeg'], CAL['J4StepDeg'], CAL['J5StepDeg'], CAL['J6StepDeg']]
    step_lims = [J1StepLim, J2StepLim, J3StepLim, J4StepLim, J5StepLim, J6StepLim]

    step_difs = [int(round(cur - fut)) for cur, fut in zip(RUN['cur_steps'], fut_steps)]


    dirs = [1 if diff <= 0 else 0 for diff in step_difs]
    faults = []
    

    for i in range(6):
        if dirs[i] == 1 and (RUN['cur_steps'][i] + abs(step_difs[i]) > step_lims[i]):
            faults.append(1)
        elif dirs[i] == 0 and (RUN['cur_steps'][i] - abs(step_difs[i]) < 0):
            faults.append(1)
        else:
            faults.append(0)

    total_axis_fault = sum(faults)

    if total_axis_fault == 0:
        return start_driveMotorsJ_thread(
            *[abs(d) for d in step_difs],
            *dirs,
            SpeedType,
            SpeedVal,
            ACCspd,
            DCCspd,
            ACCramp
        )
    else:
        if RUN['offlineMode']:
          RUN['Alarm'] = "EL" + ''.join(str(f) for f in faults)
          ErrorHandler(RUN['Alarm'])
    return False


def mj_command(inData):
    #global xyzuvw_In, KinematicError, Robot_Data
    # global RUN['JstepCur'], RUN['JointMin'], RUN['JointMax'], RUN['JangleOut']
    #global J1StepM, J2StepM, J3StepM, J4StepM, J5StepM, J6StepM
    # global RUN['J1axisLimNeg'], RUN['J2axisLimNeg'], RUN['J3axisLimNeg'], RUN['J4axisLimNeg'], RUN['J5axisLimNeg'], RUN['J6axisLimNeg']
    # global RUN['cur_steps'], RUN['Alarm'], RUN['VR_angles']

    logger.info(inData)

    result = parse_mj_command(inData)
    if not result:
        if RUN['offlineMode']:
          ErrorHandler("ER")
        return False

    # Extract values
    target_xyzuvw = np.array(result["xyzuvw"], dtype=float)
    SpeedVal = result["Speed"]
    ACCspd = result["Acc"]
    DCCspd = result["Dec"]
    ACCramp = result["Ramp"]
    SpeedType = result["SpeedType"]

    cartesian_start = None
    cartesian_target = None
    if SpeedType == "m":
        try:
            current_angles = _validated_virtual_six_vector(
                RUN['VR_angles'],
                "Virtual joint timing start",
            )
            native_cartesian_start = _validated_virtual_six_vector(
                robot.forward_kinematics(current_angles),
                "Cartesian timing start",
            )
            cartesian_start = (
                native_cartesian_start[:3]
                + tuple(math.degrees(value) for value in native_cartesian_start[3:])
            )
            cartesian_target = _validated_virtual_six_vector(
                target_xyzuvw,
                "Cartesian timing target",
            )
        except Exception as exc:
            logger.error("MJ Cartesian timing preparation failed: %s", exc)
            if RUN['offlineMode']:
                ErrorHandler("ER")
            return False

    RUN['xyzuvw_In'] = target_xyzuvw

    try:
        RUN['JangleOut'] = _solve_inverse_kinematics(
            RUN['xyzuvw_In'],
            RUN['VR_angles'],
            result["WristConfig"],
        )
    except Exception as exc:
        logger.error("Virtual Cartesian IK failed: %s", exc)
        if RUN['offlineMode']:
            ErrorHandler("ER")
        return False

    if RUN['JangleOut'] is None:
        if RUN['offlineMode']:
          logger.error("Inverse kinematics failed. No solution found.")
          ErrorHandler("ER")
        return False

 
    RUN['JangleOut'] = np.array(RUN['JangleOut'], dtype=np.float64).flatten()

    # Convert angles to steps
    step_degs = [float(CAL['J1StepDeg']), float(CAL['J2StepDeg']), float(CAL['J3StepDeg']),
             float(CAL['J4StepDeg']), float(CAL['J5StepDeg']), float(CAL['J6StepDeg'])]
    axis_neg = [float(RUN['J1axisLimNeg']), float(RUN['J2axisLimNeg']), float(RUN['J3axisLimNeg']),
            float(RUN['J4axisLimNeg']), float(RUN['J5axisLimNeg']), float(RUN['J6axisLimNeg'])]
    fut_steps = [int(round((j + off) * deg)) for j, off, deg in zip(RUN['JangleOut'], axis_neg, step_degs)]

    RUN['cur_steps'] = [RUN['J1StepM'], RUN['J2StepM'], RUN['J3StepM'], RUN['J4StepM'], RUN['J5StepM'], RUN['J6StepM']]
    step_lims = [J1StepLim, J2StepLim, J3StepLim, J4StepLim, J5StepLim, J6StepLim]

    step_difs = [int(round(cur - fut)) for cur, fut in zip(RUN['cur_steps'], fut_steps)]

    dirs = [1 if diff <= 0 else 0 for diff in step_difs]
    faults = []

    for i in range(6):
        if dirs[i] == 1 and (RUN['cur_steps'][i] + abs(step_difs[i]) > step_lims[i]):
            faults.append(1)
        elif dirs[i] == 0 and (RUN['cur_steps'][i] - abs(step_difs[i]) < 0):
            faults.append(1)
        else:
            faults.append(0)

    total_axis_fault = sum(faults)

    if total_axis_fault == 0:
        return start_driveMotorsJ_thread(
            *[abs(d) for d in step_difs],
            *dirs,
            SpeedType,
            SpeedVal,
            ACCspd,
            DCCspd,
            ACCramp,
            cartesian_start,
            cartesian_target,
        )
    else:
        if RUN['offlineMode']:
          RUN['Alarm'] = "EL" + ''.join(str(f) for f in faults)
          ErrorHandler(RUN['Alarm'])
          logger.error(RUN['Alarm'])
    return False


def mv_command(in_data):
    try:
        vision_rotation = _vision_rotation_degrees(in_data)
        with _temporary_vision_tool_rotation(vision_rotation):
            return mj_command(in_data)
    except Exception as exc:
        logger.error("Virtual vision move failed: %s", exc)
        if RUN['offlineMode']:
            ErrorHandler("ER")
        return False




def mt_command(inData):
    #global xyzuvw_In, KinematicError
    # global RUN['JangleOut'], RUN['Alarm'], RUN['VR_angles']
    #global J1StepM, J2StepM, J3StepM, J4StepM, J5StepM, J6StepM
    # global RUN['J1axisLimNeg'], RUN['J2axisLimNeg'], RUN['J3axisLimNeg'], RUN['J4axisLimNeg'], RUN['J5axisLimNeg'], RUN['J6axisLimNeg']
    # global RUN['cur_steps']
    #global offlineMode

    result = parse_mt_command(inData)
    if not result:
        if RUN['offlineMode']:
            ErrorHandler("ER")
        return
    
    offset = [float(v) for v in result["offset_vector"]]
    original_tool_frame = _active_tool_frame()
    jogged_tool_frame = tuple(
        original + delta
        for original, delta in zip(original_tool_frame, offset)
    )
    robot.set_robot_tool_frame(*jogged_tool_frame)
    try:
        RUN['xyzuvw_In'] = np.array(
            _external_cartesian_pose_to_native(
                (
                    CAL['XcurPos'],
                    CAL['YcurPos'],
                    CAL['ZcurPos'],
                    CAL['RzcurPos'],
                    CAL['RycurPos'],
                    CAL['RxcurPos'],
                ),
                "tool-jog current Cartesian pose",
            ),
            dtype=float,
        )
        RUN['JangleOut'] = _solve_inverse_kinematics(
            RUN['xyzuvw_In'],
            RUN['VR_angles'],
            result["WristConfig"],
        )
    finally:
        robot.set_robot_tool_frame(*original_tool_frame)

    if RUN['JangleOut'] is None:
        if RUN['offlineMode']:
            logger.error("Inverse kinematics failed. No solution found.")
            ErrorHandler("ER")
        return

    RUN['JangleOut'] = np.array(RUN['JangleOut'], dtype=np.float64).flatten()

    # Convert angles to steps
    step_degs = [float(CAL['J1StepDeg']), float(CAL['J2StepDeg']), float(CAL['J3StepDeg']),
                 float(CAL['J4StepDeg']), float(CAL['J5StepDeg']), float(CAL['J6StepDeg'])]
    axis_neg = [float(RUN['J1axisLimNeg']), float(RUN['J2axisLimNeg']), float(RUN['J3axisLimNeg']),
                float(RUN['J4axisLimNeg']), float(RUN['J5axisLimNeg']), float(RUN['J6axisLimNeg'])]
    fut_steps = [int(round((j + off) * deg)) for j, off, deg in zip(RUN['JangleOut'], axis_neg, step_degs)]

    RUN['cur_steps'] = [RUN['J1StepM'], RUN['J2StepM'], RUN['J3StepM'], RUN['J4StepM'], RUN['J5StepM'], RUN['J6StepM']]
    step_lims = [J1StepLim, J2StepLim, J3StepLim, J4StepLim, J5StepLim, J6StepLim]

    step_difs = [int(round(cur - fut)) for cur, fut in zip(RUN['cur_steps'], fut_steps)]
    dirs = [1 if diff <= 0 else 0 for diff in step_difs]

    # Check limits
    faults = []
    for i in range(6):
        if dirs[i] == 1 and (RUN['cur_steps'][i] + abs(step_difs[i]) > step_lims[i]):
            faults.append(1)
        elif dirs[i] == 0 and (RUN['cur_steps'][i] - abs(step_difs[i]) < 0):
            faults.append(1)
        else:
            faults.append(0)

    if sum(faults) == 0:
        return start_driveMotorsJ_thread(
            *[abs(d) for d in step_difs],
            *dirs,
            result["SpeedType"],
            result["Speed"],
            result["Acc"],
            result["Dec"],
            result["Ramp"]
        )
    else:
        if RUN['offlineMode']:
            RUN['Alarm'] = "EL" + ''.join(str(f) for f in faults)
            ErrorHandler(RUN['Alarm'])     
    return False
     


def _queue_virtual_motion_error(response):
    virtual_motion_event_queue.put(("error", response))


def _start_offline_virtual_segment(stop_event, args):
    while True:
        with offline_live_jog_state_lock:
            if stop_event.is_set():
                return None
            operation = start_driveMotorsJ_thread(*args)
        if isinstance(operation, VirtualMotionOperation):
            return operation
        if operation is not False:
            raise TypeError("offline virtual segment returned an invalid operation")
        time.sleep(CONTROL_POLL_INTERVAL_SECONDS)


def _await_offline_virtual_segment(operation):
    if not isinstance(operation, VirtualMotionOperation):
        raise TypeError("offline virtual segment has an invalid operation")
    while not operation.completed:
        operation.wait(CONTROL_POLL_INTERVAL_SECONDS)
    succeeded, error = operation.result()
    if not succeeded:
        logger.error("Offline virtual segment failed: %s", error)
        _queue_virtual_motion_error("ER")
    return succeeded


def _parse_live_jog_drive_profile(in_data, expected_opcode):
    if expected_opcode not in ("LC", "LJ", "LT"):
        raise MotionInputError("offline live-jog opcode is invalid")
    if not isinstance(in_data, str) or not in_data.startswith(expected_opcode):
        raise MotionInputError(
            f"offline live-jog command must use {expected_opcode}"
        )
    timing = parse_command_timing(in_data)
    if timing is None or timing.mode != "p":
        raise MotionInputError("live-jog speed mode must be Percent")
    return timing


def live_joint_jog(in_data, stop_event):
    #global J1StepM, J2StepM, J3StepM, J4StepM, J5StepM, J6StepM
    # global RUN['VR_angles'], RUN['J1axisLimNeg'], RUN['J2axisLimNeg'], RUN['J3axisLimNeg'], RUN['J4axisLimNeg'], RUN['J5axisLimNeg'], RUN['J6axisLimNeg']
    # global RUN['Alarm'], RUN['flag']
    #global liveJog, KinematicError

    try:
        timing = _parse_live_jog_drive_profile(in_data, "LJ")
    except MotionInputError as exc:
        logger.error("Offline live joint jog rejected: %s", exc)
        _queue_virtual_motion_error("ER")
        return False

    Vector = float(in_data[in_data.index("V") + 1 : in_data.index("S")])
    SpeedType = timing.mode
    SpeedVal = timing.speed
    ACCspd = timing.acceleration
    DCCspd = timing.deceleration
    ACCramp = timing.ramp

    LoopModeStr = in_data.split("Lm")[1].strip()
    LoopModes = [int(c) for c in LoopModeStr]

    idx = int(Vector // 10) - 1
    direction = 1 if int(Vector) % 10 == 1 else -1

    if not (0 <= idx < 6):
        RUN['Alarm'] = "ER"
        _queue_virtual_motion_error(RUN['Alarm'])
        return False

    while not stop_event.is_set():
        try:
            Jangles = [float(a) for a in RUN['VR_angles'][:6]]
        except Exception as e:
            if RUN['offlineMode']:
              logger.error("Invalid virtual joint angles: %r", RUN['VR_angles'][:6])
              RUN['Alarm'] = "ER"
              _queue_virtual_motion_error(RUN['Alarm'])
            return False

        Jangles[idx] += direction * .1

        axis_lims = [
            float(RUN['J1axisLimNeg']), float(RUN['J2axisLimNeg']), float(RUN['J3axisLimNeg']),
            float(RUN['J4axisLimNeg']), float(RUN['J5axisLimNeg']), float(RUN['J6axisLimNeg'])
        ]
        step_degs = [
            float(CAL['J1StepDeg']), float(CAL['J2StepDeg']), float(CAL['J3StepDeg']),
            float(CAL['J4StepDeg']), float(CAL['J5StepDeg']), float(CAL['J6StepDeg'])
        ]
        step_lims = [J1StepLim, J2StepLim, J3StepLim, J4StepLim, J5StepLim, J6StepLim]
        RUN['cur_steps'] = [RUN['J1StepM'], RUN['J2StepM'], RUN['J3StepM'], RUN['J4StepM'], RUN['J5StepM'], RUN['J6StepM']]

        fut_steps = [int(round((Jangles[i] + axis_lims[i]) * step_degs[i])) for i in range(6)]
        step_difs = [cur - fut for cur, fut in zip(RUN['cur_steps'], fut_steps)]
        dirs = [1 if diff <= 0 else 0 for diff in step_difs]

        faults = []
        for i in range(6):
            if dirs[i] == 1 and (RUN['cur_steps'][i] + abs(step_difs[i]) > step_lims[i]):
                faults.append(1)
            elif dirs[i] == 0 and (RUN['cur_steps'][i] - abs(step_difs[i]) < 0):
                faults.append(1)
            else:
                faults.append(0)

        total_axis_fault = sum(faults)

        if total_axis_fault == 0:
            operation = _start_offline_virtual_segment(
                stop_event,
                (
                    *[abs(d) for d in step_difs],
                    *dirs,
                    SpeedType,
                    SpeedVal,
                    ACCspd,
                    DCCspd,
                    ACCramp,
                ),
            )
            if operation is None:
                break
            if not _await_offline_virtual_segment(operation):
                return False

                
        else:
            if RUN['offlineMode']:
              RUN['Alarm'] = "EL" + ''.join(str(f) for f in faults)
              _queue_virtual_motion_error(RUN['Alarm'])
              RUN['Alarm'] = "0"
            return False
    return True



def live_cartesian_jog(in_data, stop_event):
    #global xyzuvw_In, KinematicError
    #global J1StepM, J2StepM, J3StepM, J4StepM, J5StepM, J6StepM
    # global RUN['J1axisLimNeg'], RUN['J2axisLimNeg'], RUN['J3axisLimNeg'], RUN['J4axisLimNeg'], RUN['J5axisLimNeg'], RUN['J6axisLimNeg']
    #global liveJog

    try:
        timing = _parse_live_jog_drive_profile(in_data, "LC")
    except MotionInputError as exc:
        logger.error("Offline live Cartesian jog rejected: %s", exc)
        _queue_virtual_motion_error("ER")
        return False

    Vector = float(in_data[in_data.index("V") + 1:in_data.index("S")])
    SpeedType = timing.mode
    SpeedVal = timing.speed
    ACCspd = timing.acceleration
    DCCspd = timing.deceleration
    ACCramp = timing.ramp
    LoopModeStr = in_data.split("Lm")[1].strip()
    LoopModes = [int(c) for c in LoopModeStr]
    wrist_config = parse_motion_wrist_config(in_data)

    # Cartesian jog increment
    jog_step = 1  # mm or deg, depending on axis

    RUN['xyzuvw_In'] = np.array(
        _external_cartesian_pose_to_native(
            (
                CAL['XcurPos'],
                CAL['YcurPos'],
                CAL['ZcurPos'],
                CAL['RzcurPos'],
                CAL['RycurPos'],
                CAL['RxcurPos'],
            ),
            "live Cartesian starting pose",
        ),
        dtype=float,
    )

    while not stop_event.is_set():
        vector_axis = {
            10: 0,
            20: 1,
            30: 2,
            40: 5,
            50: 4,
            60: 3,
        }
        idx = vector_axis.get(int(Vector // 10) * 10, -1)
        direction = 1 if int(Vector) % 10 == 1 else -1

        if 0 <= idx < 6:
            RUN['xyzuvw_In'][idx] += direction * jog_step
        else:
            RUN['Alarm'] = "ER"
            _queue_virtual_motion_error(RUN['Alarm'])
            return False

        # Inverse Kinematics
        try:
            RUN['JangleOut'] = _solve_inverse_kinematics(
                RUN['xyzuvw_In'],
                RUN['VR_angles'],
                wrist_config,
            )
        except Exception as e:
            logger.error("Virtual Cartesian jog IK failed: %s", e)
            _queue_virtual_motion_error("ER")
            return False

        if RUN['JangleOut'] is None:
            if RUN['offlineMode']:
              RUN['Alarm'] = "ER"
              _queue_virtual_motion_error(RUN['Alarm'])
            return False
        
        RUN['JangleOut'] = np.array(RUN['JangleOut'], dtype=np.float64).flatten()

        # Convert angles to steps
        step_degs = [
            float(CAL['J1StepDeg']), float(CAL['J2StepDeg']), float(CAL['J3StepDeg']),
            float(CAL['J4StepDeg']), float(CAL['J5StepDeg']), float(CAL['J6StepDeg'])
        ]
        axis_lims = [
            float(RUN['J1axisLimNeg']), float(RUN['J2axisLimNeg']), float(RUN['J3axisLimNeg']),
            float(RUN['J4axisLimNeg']), float(RUN['J5axisLimNeg']), float(RUN['J6axisLimNeg'])
        ]
        #fut_steps = [int(round((float(j) + float(off)) * float(deg))) for j, off, deg in zip(JangleOut, axis_neg, step_degs)]
        fut_steps = [int(round((RUN['JangleOut'][i] + axis_lims[i]) * step_degs[i])) for i in range(6)]

        RUN['cur_steps'] = [RUN['J1StepM'], RUN['J2StepM'], RUN['J3StepM'], RUN['J4StepM'], RUN['J5StepM'], RUN['J6StepM']]
        step_lims = [J1StepLim, J2StepLim, J3StepLim, J4StepLim, J5StepLim, J6StepLim]

        step_difs = [cur - fut for cur, fut in zip(RUN['cur_steps'], fut_steps)]
        dirs = [1 if diff <= 0 else 0 for diff in step_difs]

        # Check axis limits
        faults = []
        for i in range(6):
            if dirs[i] == 1 and (RUN['cur_steps'][i] + abs(step_difs[i]) > step_lims[i]):
                faults.append(1)
            elif dirs[i] == 0 and (RUN['cur_steps'][i] - abs(step_difs[i]) < 0):
                faults.append(1)
            else:
                faults.append(0)

        if (
            sum(faults) == 0
            and RUN['KinematicError'] == 0
        ):
            operation = _start_offline_virtual_segment(
                stop_event,
                (
                    *[abs(d) for d in step_difs],
                    *dirs,
                    SpeedType,
                    SpeedVal,
                    ACCspd,
                    DCCspd,
                    ACCramp,
                ),
            )
            if operation is None:
                break
            if not _await_offline_virtual_segment(operation):
                return False

                 
        else:
            RUN['Alarm'] = "EL" + ''.join(str(f) for f in faults)
            _queue_virtual_motion_error(RUN['Alarm'])
            return False
    return True


def live_tool_jog(in_data, original_tool_frame, stop_event):
    #global xyzuvw_In, KinematicError
    # global RUN['JangleOut'], RUN['Alarm'], RUN['VR_angles']
    #global J1StepM, J2StepM, J3StepM, J4StepM, J5StepM, J6StepM
    # global RUN['J1axisLimNeg'], RUN['J2axisLimNeg'], RUN['J3axisLimNeg'], RUN['J4axisLimNeg'], RUN['J5axisLimNeg'], RUN['J6axisLimNeg']
    # global TFxEntryField, TFyEntryField, TFzEntryField, TFrxEntryField, TFryEntryField, TFrzEntryField
    #global liveJog
    #global offlineMode

    try:
        timing = _parse_live_jog_drive_profile(in_data, "LT")
    except MotionInputError as exc:
        logger.error("Offline live tool jog rejected: %s", exc)
        _queue_virtual_motion_error("ER")
        return False

    Vector = float(in_data[in_data.index("V") + 1:in_data.index("S")])
    SpeedType = timing.mode
    SpeedVal = timing.speed
    ACCspd = timing.acceleration
    DCCspd = timing.deceleration
    ACCramp = timing.ramp
    LoopModeStr = in_data.split("Lm")[1].strip()
    LoopModes = [int(c) for c in LoopModeStr]
    wrist_config = parse_motion_wrist_config(in_data)

    # Tool frame jog step size
    jog_step = LIVE_TOOL_JOG_INCREMENT

    try:
        original_tool_frame = tuple(
            finite_number(value, "tool-frame value")
            for value in original_tool_frame
        )
    except (TypeError, ValueError):
        _queue_virtual_motion_error("ER")
        return False
    if len(original_tool_frame) != 6 or not all(
        math.isfinite(value) for value in original_tool_frame
    ):
        _queue_virtual_motion_error("ER")
        return False

    while not stop_event.is_set():
        vector_axis = {
            10: 0,
            20: 1,
            30: 2,
            40: 5,
            50: 4,
            60: 3,
        }
        idx = vector_axis.get(int(Vector // 10) * 10, -1)
        direction = 1 if int(Vector) % 10 == 0 else -1

        # Build pose from current position
        RUN['xyzuvw_In'] = robot.forward_kinematics(RUN['VR_angles'])
        RUN['xyzuvw_In'] = RUN['xyzuvw_In'][:3] + [math.degrees(v) for v in RUN['xyzuvw_In'][3:]]   

        if 0 <= idx < 6:
            # Modify tool frame temporarily
            jogged_tool_frame = list(original_tool_frame)
            jogged_tool_frame[idx] += direction * jog_step
            robot.set_robot_tool_frame(*jogged_tool_frame)
        else:
            RUN['Alarm'] = "ER"
            _queue_virtual_motion_error(RUN['Alarm'])
            return False

        try:
            RUN['JangleOut'] = _solve_inverse_kinematics(
                RUN['xyzuvw_In'],
                RUN['VR_angles'],
                wrist_config,
            )
        except Exception as e:
            logger.error("Virtual tool jog IK failed: %s", e)
            _queue_virtual_motion_error("ER")
            return False
        finally:
            robot.set_robot_tool_frame(*original_tool_frame)

        if RUN['JangleOut'] is None:
            if RUN['offlineMode']:
                RUN['Alarm'] = "ER"
                _queue_virtual_motion_error(RUN['Alarm'])
            return False

        RUN['JangleOut'] = np.array(RUN['JangleOut'], dtype=np.float64).flatten()

        step_degs = [
            float(CAL['J1StepDeg']), float(CAL['J2StepDeg']), float(CAL['J3StepDeg']),
            float(CAL['J4StepDeg']), float(CAL['J5StepDeg']), float(CAL['J6StepDeg'])
        ]
        axis_lims = [
            float(RUN['J1axisLimNeg']), float(RUN['J2axisLimNeg']), float(RUN['J3axisLimNeg']),
            float(RUN['J4axisLimNeg']), float(RUN['J5axisLimNeg']), float(RUN['J6axisLimNeg'])
        ]
        fut_steps = [int(round((RUN['JangleOut'][i] + axis_lims[i]) * step_degs[i])) for i in range(6)]

        RUN['cur_steps'] = [RUN['J1StepM'], RUN['J2StepM'], RUN['J3StepM'], RUN['J4StepM'], RUN['J5StepM'], RUN['J6StepM']]
        step_lims = [J1StepLim, J2StepLim, J3StepLim, J4StepLim, J5StepLim, J6StepLim]

        step_difs = [cur - fut for cur, fut in zip(RUN['cur_steps'], fut_steps)]
        dirs = [1 if diff <= 0 else 0 for diff in step_difs]

        # Axis limit check
        faults = []
        for i in range(6):
            if dirs[i] == 1 and (RUN['cur_steps'][i] + abs(step_difs[i]) > step_lims[i]):
                faults.append(1)
            elif dirs[i] == 0 and (RUN['cur_steps'][i] - abs(step_difs[i]) < 0):
                faults.append(1)
            else:
                faults.append(0)

        if (
            sum(faults) == 0
            and RUN['KinematicError'] == 0
        ):
            operation = _start_offline_virtual_segment(
                stop_event,
                (
                    *[abs(d) for d in step_difs],
                    *dirs,
                    SpeedType,
                    SpeedVal,
                    ACCspd,
                    DCCspd,
                    ACCramp,
                ),
            )
            if operation is None:
                break
            if not _await_offline_virtual_segment(operation):
                return False
        else:
            RUN['Alarm'] = "EL" + ''.join(str(f) for f in faults)
            _queue_virtual_motion_error(RUN['Alarm'])
            return False
    return True
        



#############################################################################################
### VIRTUAL ROBOT ###########################################################################
#############################################################################################

# Global storage
RUN['vtk_running'] = False
RUN['actors'] = {}
RUN['assemblies'] = {}
RUN['base_transforms'] = {}
RUN['joint_transforms'] = {}
RUN['composite_transforms'] = {}


def _mode_change_is_blocked(request_lease=None, transport_reserved=False):
    if not isinstance(transport_reserved, bool):
        raise TypeError("mode-transition transport reservation flag must be boolean")
    if request_lease is not None and not motion_request_registry.owns(request_lease):
        raise RuntimeError("mode transition requires matching motion ownership")
    if transport_reserved and request_lease is None:
        raise RuntimeError("mode transition transport requires motion ownership")
    if transport_reserved and not serial_lock.locked():
        raise RuntimeError("mode transition transport reservation is missing")

    if application_closing.is_set():
        message = "Mode change rejected during application shutdown"
    elif (
        controller_correction_requested.is_set()
        or manual_motion_pose_pending.is_set()
        or controller_position_resynchronization_required.is_set()
    ):
        message = "Mode change rejected while controller recovery is pending"
    elif (
        _virtual_motion_active(request_lease)
        or live_serial_result_pending.is_set()
        or legacy_serial_result_pending.is_set()
        or joint_motion_dispatcher.active
        or (serial_lock.locked() and not transport_reserved)
    ):
        message = "Mode change rejected while motion or controller ownership is active"
    else:
        return False
    logger.warning(message)
    almStatusLab.config(text=message, style="Warn.TLabel")
    almStatusLab2.config(text=message, style="Warn.TLabel")
    return True


def _set_offline_mode_status(offline):
    if offline:
        offline_button.config(text="Go Online", style="Offline.TButton")
        message = "SYSTEM IN OFFLINE MODE"
        style = "Warn.TLabel"
    else:
        offline_button.config(text="Run Offline", style="Online.TButton")
        message = "SYSTEM IN ONLINE MODE"
        style = "OK.TLabel"
    almStatusLab.config(text=message, style=style)
    almStatusLab2.config(text=message, style=style)


def toggle_offline_mode():
    request_lease = _acquire_motion_request("Mode transition")
    if request_lease is None:
        message = _motion_request_rejection_message(
            "Mode change rejected while motion or recovery ownership is active"
        )
        logger.warning(message)
        almStatusLab.config(text=message, style="Warn.TLabel")
        almStatusLab2.config(text=message, style="Warn.TLabel")
        return False
    try:
        with _reserve_main_serial_operation():
            if _mode_change_is_blocked(
                request_lease,
                transport_reserved=True,
            ):
                return False

            if RUN['offlineMode']:
                virtual_snapshot = list(RUN['VR_angles'])
                if requestPos() is not True:
                    RUN['VR_angles'] = virtual_snapshot
                    _set_offline_mode_status(True)
                    return False
                RUN['offlineMode'] = False
                _set_offline_mode_status(False)
                RUN['VR_angles'] = [float(CAL['J1AngCur']), float(CAL['J2AngCur']), float(CAL['J3AngCur']), float(CAL['J4AngCur']), float(CAL['J5AngCur']), float(CAL['J6AngCur'])]
                setStepMonitorsVR()
            else:
                previous_virtual_pose = list(RUN['VR_angles'])
                RUN['offlineMode'] = True
                _set_offline_mode_status(True)
                RUN['VR_angles'] = [0.000, 0.000, 0.000, 0.000, 90.000, 0.000]
                J1negLimLab.config(text="-"+CAL['J1NegLim'], style="Jointlim.TLabel")
                J1posLimLab.config(text=CAL['J1PosLim'], style="Jointlim.TLabel")
                J1jogslide.config(from_=float("-"+CAL['J1NegLim']), to=float(CAL['J1PosLim']),  length=180, orient=HORIZONTAL,  command=J1sliderUpdate)
                J2negLimLab.config(text="-"+CAL['J2NegLim'], style="Jointlim.TLabel")
                J2posLimLab.config(text=CAL['J2PosLim'], style="Jointlim.TLabel")
                J2jogslide.config(from_=float("-"+CAL['J2NegLim']), to=float(CAL['J2PosLim']),  length=180, orient=HORIZONTAL,  command=J2sliderUpdate)
                J3negLimLab.config(text="-"+CAL['J3NegLim'], style="Jointlim.TLabel")
                J3posLimLab.config(text=CAL['J3PosLim'], style="Jointlim.TLabel")
                J3jogslide.config(from_=float("-"+CAL['J3NegLim']), to=float(CAL['J3PosLim']),  length=180, orient=HORIZONTAL,  command=J3sliderUpdate)
                J4negLimLab.config(text="-"+CAL['J4NegLim'], style="Jointlim.TLabel")
                J4posLimLab.config(text=CAL['J4PosLim'], style="Jointlim.TLabel")
                J4jogslide.config(from_=float("-"+CAL['J4NegLim']), to=float(CAL['J4PosLim']),  length=180, orient=HORIZONTAL,  command=J4sliderUpdate)
                J5negLimLab.config(text="-"+CAL['J5NegLim'], style="Jointlim.TLabel")
                J5posLimLab.config(text=CAL['J5PosLim'], style="Jointlim.TLabel")
                J5jogslide.config(from_=float("-"+CAL['J5NegLim']), to=float(CAL['J5PosLim']),  length=180, orient=HORIZONTAL,  command=J5sliderUpdate)
                J6negLimLab.config(text="-"+CAL['J6NegLim'], style="Jointlim.TLabel")
                J6posLimLab.config(text=CAL['J6PosLim'], style="Jointlim.TLabel")
                J6jogslide.config(from_=float("-"+CAL['J6NegLim']), to=float(CAL['J6PosLim']),  length=180, orient=HORIZONTAL,  command=J6sliderUpdate)
                try:
                    refresh_gui_from_joint_angles(RUN['VR_angles'])
                except MotionInputError as exc:
                    RUN['offlineMode'] = False
                    RUN['VR_angles'] = previous_virtual_pose
                    setStepMonitorsVR()
                    _set_offline_mode_status(False)
                    message = f"Mode change rejected during virtual refresh: {exc}"
                    logger.error(message)
                    almStatusLab.config(text=message, style="Alarm.TLabel")
                    almStatusLab2.config(text=message, style="Alarm.TLabel")
                    return False
            return True
    except SerialActivityRejected as exc:
        message = f"Mode change rejected: {exc}"
        logger.warning(message)
        almStatusLab.config(text=message, style="Warn.TLabel")
        almStatusLab2.config(text=message, style="Warn.TLabel")
        return False
    finally:
        _finish_motion_request(request_lease)

def update_joint_transforms():
    angles = RUN['VR_angles']  # List of 6 joint angles in degrees

    joint_stl_keys = [
        "Link 1-1.STL",
        "Link 2-1.STL",
        "Link 3-1.STL",
        "Link 4-1.STL",
        "Link 5-1.STL",
        "Link 6-1.STL",
    ]

    for i, stl_key in enumerate(joint_stl_keys):
        joint_tf = RUN['joint_transforms'][stl_key]
        joint_tf.Identity()

        if i == 0:
            joint_tf.RotateZ(angles[i])
        elif i == 1:
            joint_tf.RotateY(angles[i])
        elif i == 2:
            joint_tf.RotateY(angles[i])
        elif i == 3:
            joint_tf.RotateX(angles[i])
        elif i == 4:
            joint_tf.RotateY(angles[i])
        elif i == 5:
            joint_tf.RotateX(angles[i])  


def build_robot_actors(renderer):
    # global RUN['actors'], RUN['assemblies'], RUN['base_transforms'], RUN['joint_transforms'], RUN['composite_transforms']
    #global color_map

    # Named colors setup
    colors = vtk.vtkNamedColors()

    # STL files including Link 4-2.STL
    stl_files = [
        "Link Base-1.STL", "Link Base-2.STL", "Link Base-3.STL",
        "Link 1-1.STL", "Link 1-2.STL",
        "Link 2-1.STL", "Link 2-2.STL", "Link 2-3.STL",
        "Link 3-1.STL", "Link 3-2.STL",
        "Link 4-1.STL", "Link 4-2.STL", "Link 4-3.STL",
        "Link 5-1.STL", "Link 5-2.STL",
        "Link 6-1.STL", "Link 6-2.STL"
    ]

    # Clear and initialize the global color map
    RUN['color_map'].clear()
    RUN['color_map'].update({stl: "Silver" for stl in stl_files})
    RUN['color_map'].update({
        "Link Base-2.STL": "Orange",
        "Link Base-3.STL": "DimGray",
        "Link 1-2.STL": "DimGray",
        "Link 2-2.STL": "Orange", "Link 2-3.STL": "DimGray",
        "Link 3-2.STL": "DimGray",
        "Link 4-2.STL": "Orange", "Link 4-3.STL": "DimGray",
        "Link 5-2.STL": "DimGray",
        "Link 6-2.STL": "DimGray"
    })

    # Storage reset
    RUN['actors'].clear()
    RUN['assemblies'].clear()
    RUN['base_transforms'].clear()
    RUN['joint_transforms'].clear()
    RUN['composite_transforms'].clear()

    # Load STL files and create actors
    for stl in stl_files:
        reader = vtk.vtkSTLReader()
        reader.SetFileName(stl)
        reader.Update()

        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(reader.GetOutputPort())

        actor = vtk.vtkActor()
        actor.SetMapper(mapper)

        # Apply initial color from the shared color_map
        actor.GetProperty().SetColor(colors.GetColor3d(RUN['color_map'][stl]))

        base_tf = vtk.vtkTransform()
        joint_tf = vtk.vtkTransform()
        comp_tf = vtk.vtkTransform()

        # Alignment transforms
        if stl == "Link 1-1.STL":
            base_tf.RotateX(180)
            base_tf.Translate(0, 0, -87.5)
        elif stl == "Link 2-1.STL":
            base_tf.RotateZ(180)
            base_tf.RotateX(270)
            base_tf.Translate(-64.15, 77.78, 8.87)
        elif stl == "Link 3-1.STL":
            base_tf.RotateZ(180)
            base_tf.RotateX(180)
            base_tf.Translate(0, 305, -27.84)
        elif stl == "Link 4-1.STL":
            base_tf.RotateY(90)
            base_tf.RotateX(180)
            base_tf.Translate(-36.7, 0, -75.94)
        elif stl == "Link 5-1.STL":
            base_tf.RotateZ(180)
            base_tf.RotateY(90)
            base_tf.Translate(147, 0, 44.88)
        elif stl == "Link 6-1.STL":
            base_tf.RotateY(90)
            base_tf.Translate(43.3, 0, 25)

        comp_tf.Concatenate(base_tf)
        comp_tf.Concatenate(joint_tf)

        asm = vtk.vtkAssembly()
        asm.AddPart(actor)
        asm.SetUserTransform(comp_tf)

        RUN['actors'][stl] = actor
        RUN['assemblies'][stl] = asm
        RUN['base_transforms'][stl] = base_tf
        RUN['joint_transforms'][stl] = joint_tf
        RUN['composite_transforms'][stl] = comp_tf

    # Build hierarchy
    root = RUN['assemblies']["Link Base-1.STL"]
    root.AddPart(RUN['assemblies']["Link Base-2.STL"])
    RUN['assemblies']["Link Base-2.STL"].AddPart(RUN['assemblies']["Link Base-3.STL"])
    RUN['assemblies']["Link Base-3.STL"].AddPart(RUN['assemblies']["Link 1-1.STL"])
    RUN['assemblies']["Link 1-1.STL"].AddPart(RUN['assemblies']["Link 1-2.STL"])
    RUN['assemblies']["Link 1-2.STL"].AddPart(RUN['assemblies']["Link 2-1.STL"])
    RUN['assemblies']["Link 2-1.STL"].AddPart(RUN['assemblies']["Link 2-2.STL"])
    RUN['assemblies']["Link 2-2.STL"].AddPart(RUN['assemblies']["Link 2-3.STL"])
    RUN['assemblies']["Link 2-3.STL"].AddPart(RUN['assemblies']["Link 3-1.STL"])
    RUN['assemblies']["Link 3-1.STL"].AddPart(RUN['assemblies']["Link 3-2.STL"])
    RUN['assemblies']["Link 3-2.STL"].AddPart(RUN['assemblies']["Link 4-1.STL"])
    RUN['assemblies']["Link 4-1.STL"].AddPart(RUN['assemblies']["Link 4-2.STL"])
    RUN['assemblies']["Link 4-2.STL"].AddPart(RUN['assemblies']["Link 4-3.STL"])
    RUN['assemblies']["Link 4-3.STL"].AddPart(RUN['assemblies']["Link 5-1.STL"])
    RUN['assemblies']["Link 5-1.STL"].AddPart(RUN['assemblies']["Link 5-2.STL"])
    RUN['assemblies']["Link 5-2.STL"].AddPart(RUN['assemblies']["Link 6-1.STL"])
    RUN['assemblies']["Link 6-1.STL"].AddPart(RUN['assemblies']["Link 6-2.STL"])

    renderer.AddActor(root)


class CustomInteractorStyle(vtk.vtkInteractorStyleTrackballCamera):
    def __init__(self, renderer):
        self.AddObserver("LeftButtonReleaseEvent", self.on_left_button_up)
        self.renderer = renderer

    def on_left_button_up(self, obj, event):
        self.OnLeftButtonUp()  # <-- CORRECT way to call the base method

def update_joint_angles():
    angles = {
        "Link 1-1.STL": -RUN['VR_angles'][0],
        "Link 2-1.STL": RUN['VR_angles'][1],
        "Link 3-1.STL": -RUN['VR_angles'][2],
        "Link 4-1.STL": -RUN['VR_angles'][3],
        "Link 5-1.STL": -RUN['VR_angles'][4],
        "Link 6-1.STL": RUN['VR_angles'][5]
    }

    for stl, angle in angles.items():
        jt = RUN['joint_transforms'][stl]
        jt.Identity()
        jt.RotateZ(angle)
        ct = RUN['composite_transforms'][stl]
        ct.Identity()
        ct.Concatenate(RUN['base_transforms'][stl])
        ct.Concatenate(jt)

def add_floor_grid(renderer, size=1000, spacing=50):
    grid = vtk.vtkPolyData()
    points = vtk.vtkPoints()
    lines = vtk.vtkCellArray()

    count = 0
    for i in range(-size, size + spacing, spacing):
        # lines parallel to X
        points.InsertNextPoint(i, -size, 0)
        points.InsertNextPoint(i, size, 0)
        lines.InsertNextCell(2)
        lines.InsertCellPoint(count)
        lines.InsertCellPoint(count + 1)
        count += 2

        # lines parallel to Y
        points.InsertNextPoint(-size, i, 0)
        points.InsertNextPoint(size, i, 0)
        lines.InsertNextCell(2)
        lines.InsertCellPoint(count)
        lines.InsertCellPoint(count + 1)
        count += 2

    grid.SetPoints(points)
    grid.SetLines(lines)

    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputData(grid)

    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    actor.GetProperty().SetColor(0.7, 0.7, 0.7)  # Light gray
    actor.GetProperty().SetLineWidth(1)

    RUN['renderer'].AddActor(actor)        

def on_close_event(obj, event):
    # global RUN['vtk_running']
    RUN['vtk_running'] = False
    try:
        obj.GetRenderWindow().Finalize()
        obj.TerminateApp()
    except:
        pass

def update_vtk(render_window, root_widget):
    if not RUN['vtk_running']:
        return

    angles = tuple(
        finite_number(angle, "virtual joint angle")
        for angle in RUN['VR_angles']
    )
    angles_changed = angles != RUN.get('lastRenderedAngles')
    if angles_changed:
        update_joint_angles()
        render_window.Render()
        RUN['lastRenderedAngles'] = angles

    delay_ms = 16 if angles_changed else 100
    root_widget.after(
        delay_ms,
        lambda: update_vtk(render_window, root_widget),
    )


def add_reset_view_button(renderer, interactor, camera):
    # Create a text actor for the button
    text_actor = vtk.vtkTextActor()
    text_actor.SetInput("Reset View")
    text_actor.GetTextProperty().SetFontSize(24)
    text_actor.GetTextProperty().SetColor(1.0, 1.0, 1.0)  # white
    text_actor.SetDisplayPosition(20, 20)  # bottom-left corner
    renderer.AddActor2D(text_actor)

    # Get a rough width/height for click bounds (trial and error)
    click_bounds = {
        'x1': 20,
        'y1': 20,
        'x2': 20 + 150,  # width of text
        'y2': 20 + 40    # height of text
    }

    def click_callback(obj, event):
        click_pos = RUN['interactor'].GetEventPosition()
        x, y = click_pos
        if click_bounds['x1'] <= x <= click_bounds['x2'] and click_bounds['y1'] <= y <= click_bounds['y2']:
            camera.Azimuth(45)
            camera.Elevation(35)
            camera.SetViewUp(0, 0, 1)
            RUN['renderer'].ResetCamera()
            RUN['renderer'].ResetCameraClippingRange()
            RUN['interactor'].GetRenderWindow().Render()

    # Attach click handler
    RUN['interactor'].AddObserver("LeftButtonPressEvent", click_callback)        

def launch_vtk_nonblocking(root_widget):
    #global renderer
    # global RUN['vtk_running'], RUN['interactor'], RUN['render_window'], RUN['VR_angles']

    RUN['vtk_running'] = True

    RUN['renderer'] = vtk.vtkRenderer()
    RUN['render_window'] = vtk.vtkRenderWindow() 
    RUN['render_window'].SetWindowName("AR4 Virtual Robot Viewer")
    RUN['interactor'] = vtk.vtkRenderWindowInteractor()
    RUN['render_window'].AddRenderer(RUN['renderer'])
    RUN['interactor'].SetRenderWindow(RUN['render_window'])

    style = CustomInteractorStyle(RUN['renderer'])
    RUN['interactor'].SetInteractorStyle(style)

    RUN['render_window'].SetSize(1024, 768)
    RUN['renderer'].SetBackground(vtk.vtkNamedColors().GetColor3d("LightSlateGray"))

    build_robot_actors(RUN['renderer'])
    add_floor_grid(RUN['renderer'])

    camera = RUN['renderer'].GetActiveCamera()
    RUN['renderer'].ResetCamera()
    camera.Dolly(3)
    camera.Azimuth(65)
    camera.Elevation(55)
    camera.SetViewUp(0, 0, 1)
    RUN['renderer'].ResetCameraClippingRange()

    #add_reset_view_button(RUN['renderer'], interactor, camera)

    RUN['interactor'].AddObserver("ExitEvent", on_close_event)
    RUN['interactor'].Initialize()
    RUN['render_window'].Render()
    
    set_vtk_topmost_delayed()

    RUN['VR_angles'] = [float(CAL['J1AngCur']), float(CAL['J2AngCur']), float(CAL['J3AngCur']), float(CAL['J4AngCur']), float(CAL['J5AngCur']), float(CAL['J6AngCur'])]
    setStepMonitorsVR()
    update_main_color()

    RUN['lastRenderedAngles'] = None
    update_vtk(RUN['render_window'], root_widget)





def set_vtk_topmost_delayed():
    window_title = "AR4 Virtual Robot Viewer"

    def set_topmost():
        time.sleep(0.5)
        os_type = platform.system()

        for _attempt in range(20):
            if not RUN['vtk_running']:
                return
            try:
                if os_type == 'Windows':
                    import win32con
                    import win32gui

                    hwnd = win32gui.FindWindow(None, window_title)
                    if hwnd:
                        win32gui.SetWindowPos(
                            hwnd,
                            win32con.HWND_TOPMOST,
                            0,
                            0,
                            0,
                            0,
                            win32con.SWP_NOMOVE
                            | win32con.SWP_NOSIZE
                            | win32con.SWP_NOACTIVATE,
                        )
                        return

                elif os_type == 'Linux':
                    try:
                        result = subprocess.run(
                            ['wmctrl', '-l'],
                            capture_output=True,
                            text=True,
                            check=False,
                            timeout=2,
                        )
                    except FileNotFoundError:
                        result = None

                    if result is not None and result.returncode == 0:
                        for line in result.stdout.splitlines():
                            fields = line.split(maxsplit=3)
                            if len(fields) == 4 and window_title in fields[3]:
                                mutation = subprocess.run(
                                    [
                                        'wmctrl',
                                        '-i',
                                        '-r',
                                        fields[0],
                                        '-b',
                                        'add,above',
                                    ],
                                    capture_output=True,
                                    check=False,
                                    timeout=2,
                                )
                                if mutation.returncode == 0:
                                    return
                                logger.debug(
                                    "wmctrl could not mark the VTK viewer topmost: exit %s",
                                    mutation.returncode,
                                )

                    try:
                        result = subprocess.run(
                            ['xdotool', 'search', '--name', window_title],
                            capture_output=True,
                            text=True,
                            check=False,
                            timeout=2,
                        )
                    except FileNotFoundError:
                        result = None

                    if result is not None and result.returncode == 0:
                        window_ids = result.stdout.split()
                        if window_ids:
                            mutation = subprocess.run(
                                ['xdotool', 'windowraise', window_ids[0]],
                                capture_output=True,
                                check=False,
                                timeout=2,
                            )
                            if mutation.returncode == 0:
                                return
                            logger.debug(
                                "xdotool could not raise the VTK viewer: exit %s",
                                mutation.returncode,
                            )
                else:
                    return
            except (OSError, subprocess.SubprocessError) as exc:
                logger.debug("VTK topmost attempt failed: %s", exc)

            time.sleep(0.25)

        logger.debug("VTK viewer did not become available for topmost configuration")

    threading.Thread(target=set_topmost, daemon=True).start()



def update_stl_transform():
    name = stl_name_var.get()
    if name not in imported_actors:
        logger.error("File not found in imported actors.")
        return

    actor = imported_actors[name]
    try:
        x = float(x_var.get())
        y = float(y_var.get())
        z = float(z_var.get())
        rot = float(rot_var.get())
    except ValueError:
        logger.error("Invalid number entered.")
        return

    transform = vtk.vtkTransform()
    transform.Translate(x, y, z)
    transform.RotateZ(rot)

    actor.SetUserTransform(transform)
    RUN['render_window'].Render()





imported_actors = {}  # filename -> actor mapping

stl_name_var = tk.StringVar()
x_var = tk.StringVar(value="0")
y_var = tk.StringVar(value="0")
z_var = tk.StringVar(value="0")
rot_var = tk.StringVar(value="0")

def import_stl_file():
    file_path = fd.askopenfilename(filetypes=[("STL files", "*.stl")])
    if not file_path:
        return

    filename = os.path.basename(file_path)
    stl_name_var.set(filename)

    reader = vtk.vtkSTLReader()
    reader.SetFileName(file_path)
    reader.Update()

    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(reader.GetOutputPort())

    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    actor.GetProperty().SetColor(0.254, 0.41, 0.882)
    actor.SetPosition(0, 0, 0)

    RUN['renderer'].AddActor(actor)
    RUN['render_window'].Render()

    imported_actors[filename] = actor


def load_stl_into_scene(stl_path, renderer, render_window):
    reader = vtk.vtkSTLReader()
    reader.SetFileName(stl_path)
    reader.Update()

    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(reader.GetOutputPort())

    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    actor.GetProperty().SetColor(0.254, 0.41, 0.882)  

    actor.SetPosition(0, 0, 0)  # Adjust as needed

    renderer.AddActor(actor)
    RUN['render_window'].Render()

   
    




###############################################################################################################################################################
### STARTUP DEFS ################################################################################################################# 
###############################################################################################################################################################




def startup_spinner(root, message="Please wait…"):
    win = tk.Toplevel(root)
    win.title("")
    win.transient(root)
    win.resizable(False, False)
    win.grab_set()  # modal

    # Use same icon as main window
    #win.iconbitmap(r'AR.png')
    win.iconphoto(True, tk.PhotoImage(file="AR.png"))
    ttk.Label(win, text=message, padding=12).pack()
    pb = ttk.Progressbar(win, mode="indeterminate", length=220)
    pb.pack(padx=12, pady=(0, 12))
    pb.start(12)

    # Center on parent
    root.update_idletasks()
    x = root.winfo_rootx() + (root.winfo_width() - win.winfo_reqwidth()) // 2
    y = root.winfo_rooty() + (root.winfo_height() - win.winfo_reqheight()) // 2
    win.geometry(f"+{x}+{y}")

    win.update_idletasks()
    return win, pb


def _validated_startup_command(command, expected_prefix):
    if not isinstance(command, str) or not command.startswith(expected_prefix):
        raise MotionInputError(
            f"controller startup command must begin with {expected_prefix!r}"
        )
    if not command.endswith("\n") or "\n" in command[:-1] or "\r" in command:
        raise MotionInputError(
            "controller startup command must contain one trailing line delimiter"
        )
    if len(command) > MAX_COMMAND_LENGTH:
        raise MotionInputError("controller startup command exceeds the size limit")
    try:
        command.encode("ascii")
    except UnicodeEncodeError as exc:
        raise MotionInputError(
            "controller startup command must contain ASCII characters only"
        ) from exc
    return command


def _build_startup_numeric_command(prefix, fields):
    if not isinstance(prefix, str) or not prefix or not prefix.isascii():
        raise MotionInputError("controller startup prefix must be ASCII text")
    if isinstance(fields, (str, bytes)):
        raise MotionInputError("controller startup fields must be a sequence")
    try:
        fields = iter(fields)
    except TypeError as exc:
        raise MotionInputError(
            "controller startup fields must be a sequence"
        ) from exc
    parts = [prefix]
    markers = set()
    for index, field in enumerate(fields):
        if (
            not isinstance(field, (tuple, list))
            or len(field) != 2
        ):
            raise MotionInputError(
                f"controller startup field {index} must contain a marker and value"
            )
        marker, value = field
        if (
            not isinstance(marker, str)
            or not marker
            or not marker.isascii()
            or "\n" in marker
            or "\r" in marker
        ):
            raise MotionInputError("controller startup marker must be ASCII text")
        if marker in markers:
            raise MotionInputError(
                f"controller startup marker {marker!r} is duplicated"
            )
        markers.add(marker)
        parts.append(marker)
        parts.append(
            controller_protocol_decimal(
                value,
                f"{prefix} field {marker!r}",
            )
        )
    return _validated_startup_command("".join(parts) + "\n", prefix)


@dataclass(frozen=True)
class ControllerStartupRequest:
    auxiliary_port: Optional[str]
    update_parameters_command: str
    external_axis_command: str
    position_command: str
    auxiliary_board: Optional[str] = None

    def __post_init__(self):
        if self.auxiliary_port is None:
            auxiliary_port = None
        elif not isinstance(self.auxiliary_port, str):
            raise MotionInputError("auxiliary controller port must be text")
        else:
            auxiliary_port = self.auxiliary_port.strip()
            if auxiliary_port in ("", "None"):
                auxiliary_port = None
        auxiliary_board = normalize_auxiliary_board_profile(
            self.auxiliary_board,
            allow_none=True,
        )
        object.__setattr__(self, "auxiliary_port", auxiliary_port)
        object.__setattr__(self, "auxiliary_board", auxiliary_board)
        object.__setattr__(
            self,
            "update_parameters_command",
            _validated_startup_command(self.update_parameters_command, "UP"),
        )
        object.__setattr__(
            self,
            "external_axis_command",
            _validated_startup_command(self.external_axis_command, "CE"),
        )
        object.__setattr__(
            self,
            "position_command",
            _validated_startup_command(self.position_command, "SP"),
        )


@dataclass(frozen=True)
class ControllerStartupResult:
    position: PositionResponse
    visual_options: tuple
    auxiliary_serial: object
    auxiliary_error: Optional[str] = None

    def __post_init__(self):
        if not isinstance(self.position, PositionResponse):
            raise ProtocolResponseError(
                "controller startup position has an invalid type"
            )
        if isinstance(self.visual_options, (str, bytes)):
            raise ProtocolResponseError("visual options must be a sequence")
        try:
            visual_options = tuple(self.visual_options)
        except TypeError as exc:
            raise ProtocolResponseError("visual options must be a sequence") from exc
        if not all(
            isinstance(option, str)
            and option.endswith('.jpg')
            and os.path.basename(option) == option
            for option in visual_options
        ):
            raise ProtocolResponseError("visual options contain an invalid filename")
        auxiliary_error = self.auxiliary_error
        if auxiliary_error is not None:
            if not isinstance(auxiliary_error, str) or not auxiliary_error.strip():
                raise ProtocolResponseError(
                    "auxiliary startup error must be non-empty text or None"
                )
            auxiliary_error = auxiliary_error.strip()
        if self.auxiliary_serial is not None and auxiliary_error is not None:
            raise ProtocolResponseError(
                "auxiliary startup cannot contain both a connection and an error"
            )
        object.__setattr__(self, "visual_options", visual_options)
        object.__setattr__(self, "auxiliary_error", auxiliary_error)


def _startup_visual_options():
    if getattr(sys, 'frozen', False):
        folder = os.path.dirname(sys.executable)
    else:
        folder = os.path.dirname(os.path.realpath(__file__))
    return tuple(sorted(
        filename
        for filename in os.listdir(folder)
        if isinstance(filename, str) and filename.endswith('.jpg')
    ))


def _startup_exchange_response(
    command,
    cancel_event,
    expected_response=None,
):
    if not isinstance(command, str):
        raise MotionInputError("controller startup command must be text")
    command = _validated_startup_command(command, command[:2])
    if not callable(getattr(cancel_event, "is_set", None)):
        raise TypeError("controller startup cancellation must satisfy the event contract")
    if expected_response is not None and (
        not isinstance(expected_response, bytes) or not expected_response
    ):
        raise MotionInputError("expected startup response must be non-empty bytes")
    framed_expected_response = (
        expected_response is not None and expected_response.endswith(b"\n")
    )
    expected_payload = expected_response
    if framed_expected_response:
        try:
            expected_payload = decode_serial_response_line(
                expected_response
            ).encode("ascii")
        except ProtocolResponseError as exc:
            raise MotionInputError(
                "expected framed startup response is invalid"
            ) from exc
    elif expected_response is not None and (
        len(expected_response) > MAX_RESPONSE_PAYLOAD_LENGTH
        or expected_response != expected_response.strip()
        or b"\r" in expected_response
        or b"\n" in expected_response
        or not expected_response.isascii()
    ):
        raise MotionInputError(
            "expected startup response must contain normalized unframed ASCII bytes"
        )
    response_limit = (
        MAX_RESPONSE_FRAME_LENGTH
        if expected_response is None or framed_expected_response
        else MAX_RESPONSE_PAYLOAD_LENGTH
    )

    serial_port = RUN.get('ser')
    if serial_port is None or not getattr(serial_port, "is_open", False):
        raise ConnectionError("controller serial connection is not open")
    if serial_transport_quarantined(serial_port):
        raise SerialTransportQuarantinedError(
            "controller serial connection is quarantined; reconnect required"
        )
    try:
        original_timeout = serial_port.timeout
    except Exception as exc:
        raise TypeError("controller serial connection has no read timeout") from exc

    reset_input = getattr(serial_port, "reset_input_buffer", None)
    if not callable(reset_input):
        reset_input = getattr(serial_port, "flushInput", None)
    write = getattr(serial_port, "write", None)
    flush = getattr(serial_port, "flush", None)
    read = getattr(serial_port, "read", None)
    read_until = getattr(serial_port, "read_until", None)
    if not all(callable(method) for method in (reset_input, write, flush, read)):
        raise TypeError("controller serial connection lacks the startup I/O contract")

    def cancellation_requested():
        requested = cancel_event.is_set()
        if not isinstance(requested, bool):
            raise ProtocolResponseError(
                "controller startup cancellation state must be boolean"
            )
        return requested

    command_bytes = command.encode("ascii")
    acquired = serial_write_lock.acquire()
    if acquired is False:
        raise RuntimeError("controller startup write lock acquisition failed")
    try:
        if cancellation_requested():
            raise TimeoutError("controller startup cancelled")
        reset_input()
        if cancellation_requested():
            raise TimeoutError("controller startup cancelled")
        written = write(command_bytes)
        if written != len(command_bytes):
            raise OSError(
                "controller startup write was incomplete: "
                f"expected {len(command_bytes)} bytes, wrote {written!r}"
            )
        flush()
    finally:
        serial_write_lock.release()

    response = bytearray()
    deadline = time.monotonic() + SERIAL_STARTUP_READ_TIMEOUT_SECONDS
    operation_error = None
    try:
        while True:
            if cancellation_requested():
                raise TimeoutError("controller startup cancelled")
            remaining_time = deadline - time.monotonic()
            if remaining_time <= 0:
                raise SerialTransportTimeout(
                    "controller startup response deadline expired"
                )
            serial_port.timeout = min(
                CONTROL_POLL_INTERVAL_SECONDS,
                remaining_time,
            )
            remaining_size = response_limit + 1 - len(response)
            if expected_response is not None and not framed_expected_response:
                remaining_size = len(expected_response) - len(response)
                chunk = read(remaining_size)
            elif callable(read_until):
                chunk = read_until(b"\n", remaining_size)
            else:
                chunk = read(remaining_size)
            if not isinstance(chunk, (bytes, bytearray)):
                raise ProtocolResponseError(
                    "controller startup reader returned a non-bytes response"
                )
            if not chunk:
                continue
            response.extend(chunk)
            if len(response) > response_limit:
                raise ProtocolResponseError(
                    "controller startup response exceeds the size limit"
                )
            if expected_response is not None and not framed_expected_response:
                if len(response) < len(expected_response):
                    continue
                if bytes(response) != expected_response:
                    raise ProtocolResponseError(
                        "controller startup returned an unexpected acknowledgement"
                    )
                if cancellation_requested():
                    raise TimeoutError("controller startup cancelled")
                remaining_time = deadline - time.monotonic()
                if remaining_time < CONTROL_POLL_INTERVAL_SECONDS:
                    raise SerialTransportTimeout(
                      "controller startup quiet-boundary deadline expired"
                    )
                serial_port.timeout = CONTROL_POLL_INTERVAL_SECONDS
                trailing = read(1)
                if not isinstance(trailing, (bytes, bytearray)):
                    raise ProtocolResponseError(
                        "controller startup quiet-boundary reader returned non-bytes"
                    )
                if trailing:
                    raise ProtocolResponseError(
                        "controller startup returned trailing unframed data"
                    )
                if cancellation_requested():
                    raise TimeoutError("controller startup cancelled")
                if not getattr(serial_port, "is_open", False):
                    raise ConnectionError(
                        "controller startup connection closed before the quiet boundary"
                    )
                return bytes(response).decode("ascii")
            if b"\n" not in response:
                continue
            if response.find(b"\n") != len(response) - 1:
                raise ProtocolResponseError(
                    "controller startup returned trailing framed data"
                )
            framed_payload = bytes(response[:-1])
            if framed_payload.endswith(b"\r"):
                framed_payload = framed_payload[:-1]
            if (
                framed_expected_response
                and framed_payload != expected_payload
            ):
                raise ProtocolResponseError(
                    "controller startup returned an unexpected acknowledgement"
                )
            decoded = decode_serial_response_line(response)
            if cancellation_requested():
                raise TimeoutError("controller startup cancelled")
            remaining_time = deadline - time.monotonic()
            if remaining_time < CONTROL_POLL_INTERVAL_SECONDS:
                raise SerialTransportTimeout(
                  "controller startup quiet-boundary deadline expired"
                )
            serial_port.timeout = CONTROL_POLL_INTERVAL_SECONDS
            trailing = read(1)
            if not isinstance(trailing, (bytes, bytearray)):
                raise ProtocolResponseError(
                    "controller startup quiet-boundary reader returned non-bytes"
                )
            if trailing:
                raise ProtocolResponseError(
                    "controller startup returned queued trailing framed data"
                )
            if not getattr(serial_port, "is_open", False):
                raise ConnectionError(
                    "controller startup connection closed before the quiet boundary"
                )
            return decoded
    except Exception as exc:
        operation_error = exc
        raise
    finally:
        try:
            serial_port.timeout = original_timeout
        except Exception as exc:
            if operation_error is None:
                raise TypeError(
                    "unable to restore the controller startup read timeout"
                ) from exc
            raise TypeError(
                f"{operation_error}; unable to restore the controller startup read timeout"
            ) from exc


def _close_startup_auxiliary(serial_port):
    if serial_port is None:
        return True
    if not auxiliary_serial_lock.acquire(blocking=False):
        logger.error("Auxiliary startup cleanup could not reserve the transport")
        return False
    try:
        if getattr(serial_port, "is_open", None) is False:
            if RUN.get('ser2') is serial_port:
                RUN['ser2'] = None
                _clear_auxiliary_board_profile(serial_port)
            return True
        try:
            serial_port.close()
        except Exception:
            logger.exception("Unable to close the auxiliary startup connection")
            return False
        if getattr(serial_port, "is_open", False):
            logger.error("Auxiliary startup connection remained open during cleanup")
            return False
        if RUN.get('ser2') is serial_port:
            RUN['ser2'] = None
            _clear_auxiliary_board_profile(serial_port)
        return True
    finally:
        auxiliary_serial_lock.release()


def _run_startup_auxiliary_cleanup():
    global startup_auxiliary_cleanup_worker

    while True:
        with startup_auxiliary_cleanup_lock:
            serial_ports = tuple(startup_auxiliary_cleanup_pending.values())
            if not serial_ports:
                startup_auxiliary_cleanup_worker = None
                return

        closed_any = False
        for serial_port in serial_ports:
            if not _close_startup_auxiliary(serial_port):
                continue
            with startup_auxiliary_cleanup_lock:
                key = id(serial_port)
                if startup_auxiliary_cleanup_pending.get(key) is serial_port:
                    del startup_auxiliary_cleanup_pending[key]
            closed_any = True

        if not closed_any:
            time.sleep(SERIAL_SHUTDOWN_RETRY_MS / 1000.0)


def _ensure_startup_auxiliary_cleanup():
    global startup_auxiliary_cleanup_worker

    with startup_auxiliary_cleanup_lock:
        if not startup_auxiliary_cleanup_pending:
            return True
        worker = startup_auxiliary_cleanup_worker
        if worker is not None and worker.is_alive():
            return True
        try:
            worker = threading.Thread(
                target=_run_startup_auxiliary_cleanup,
                name="ar4-startup-auxiliary-cleanup",
                daemon=True,
            )
            startup_auxiliary_cleanup_worker = worker
            worker.start()
        except Exception:
            startup_auxiliary_cleanup_worker = None
            logger.exception("Unable to start auxiliary startup cleanup retry")
            return False
    return True


def _request_startup_auxiliary_cleanup(serial_port):
    if serial_port is None:
        return True
    if _close_startup_auxiliary(serial_port):
        with startup_auxiliary_cleanup_lock:
            key = id(serial_port)
            if startup_auxiliary_cleanup_pending.get(key) is serial_port:
                del startup_auxiliary_cleanup_pending[key]
        return True

    # Retain the failed handle until a retry closes it; dropping the last
    # reference can leave an open OS handle outside shutdown ownership.
    with startup_auxiliary_cleanup_lock:
        startup_auxiliary_cleanup_pending[id(serial_port)] = serial_port
    _ensure_startup_auxiliary_cleanup()
    return False


def _connect_startup_auxiliary(port, board_profile):
  if not auxiliary_serial_lock.acquire(blocking=False):
    raise MotionTransportBusy("auxiliary controller transport is busy")
  try:
    return _replace_auxiliary_serial(port, board_profile)
  finally:
    auxiliary_serial_lock.release()


def _clear_unavailable_startup_auxiliary():
  serial_port = RUN.get('ser2')
  if serial_port is None:
    _clear_auxiliary_board_profile()
    return True
  if _request_startup_auxiliary_cleanup(serial_port):
    if RUN.get('ser2') is None:
      _clear_auxiliary_board_profile()
      return True
    raise MotionTransportBusy(
      "auxiliary controller ownership changed during startup cleanup"
    )
  raise MotionTransportBusy(
    "existing auxiliary controller could not be closed before startup commit"
  )


def startup_with_spinner(
    root,
    startup_request,
    on_finished,
    on_timeout,
    on_abandoned,
    timeout=10.0,
):
    timeout = finite_number(timeout, "controller startup timeout")
    if timeout <= 0:
        raise MotionInputError("controller startup timeout must be positive")
    if not isinstance(startup_request, ControllerStartupRequest):
        raise MotionInputError(
            "startup_request must be a ControllerStartupRequest"
        )
    if not all(callable(callback) for callback in (
        on_finished,
        on_timeout,
        on_abandoned,
    )):
        raise TypeError("controller startup callbacks must be callable")

    spinner, pb = startup_spinner(root, "Please Wait.. System Starting")
    q = Queue()
    cancel_event = threading.Event()
    missing = object()
    pending_result = missing
    timed_out = False
    spinner_closed = False
    cancelled = False
    work_complete = threading.Event()
    finalization_complete = threading.Event()
    scheduler_failed = threading.Event()

    def worker():
        try:
            result = startup(startup_request, cancel_event)
        except BaseException as exc:
            result = exc
        q.put((time.monotonic(), result))
        work_complete.set()
        while not finalization_complete.wait(CONTROL_POLL_INTERVAL_SECONDS):
            if scheduler_failed.is_set():
                try:
                    on_abandoned(result)
                except Exception:
                    logger.exception("Unable to abandon controller startup safely")
                return

    def close_spinner():
        nonlocal spinner_closed
        if spinner_closed:
            return
        spinner_closed = True
        try:
            pb.stop()
        except Exception:
            logger.exception("Unable to stop the controller startup progress bar")
        try:
            spinner.grab_release()
        except Exception:
            logger.exception("Unable to release the controller startup dialog")
        try:
            spinner.destroy()
        except Exception:
            logger.exception("Unable to destroy the controller startup dialog")

    try:
        worker_thread = Thread(target=worker, daemon=True)
    except Exception:
        close_spinner()
        raise
    deadline = time.monotonic() + timeout

    def poll_worker():
        nonlocal pending_result, timed_out
        if cancelled:
            return

        if pending_result is missing:
            try:
                pending_result = q.get_nowait()
            except Empty:
                pass

        now = time.monotonic()
        completion_time = (
            pending_result[0]
            if pending_result is not missing
            else None
        )
        deadline_exceeded = (
            completion_time > deadline
            if completion_time is not None
            else now >= deadline
        )
        if not timed_out and deadline_exceeded:
            timed_out = True
            cancel_event.set()
            close_spinner()
            try:
                on_timeout()
            except Exception:
                logger.exception("Unable to report the controller startup timeout")

        if pending_result is not missing and work_complete.is_set():
            close_spinner()
            try:
                on_finished(pending_result[1], timed_out)
            except Exception:
                logger.exception("Unable to finalize controller startup")
            finally:
                finalization_complete.set()
            return

        try:
            root.after(10, poll_worker)
        except Exception:
            logger.exception("Unable to continue controller startup polling")
            cancel_event.set()
            if not timed_out:
                timed_out = True
                close_spinner()
                try:
                    on_timeout()
                except Exception:
                    logger.exception("Unable to report the controller startup timeout")
            scheduler_failed.set()

    try:
        poll_job = root.after(10, poll_worker)
    except Exception:
        close_spinner()
        raise
    try:
        worker_thread.start()
    except Exception:
        cancelled = True
        finalization_complete.set()
        try:
            root.after_cancel(poll_job)
        except Exception:
            logger.exception("Unable to cancel controller startup polling")
        close_spinner()
        raise
    return worker_thread


def startup(startup_request, cancel_event=None):
  if not isinstance(startup_request, ControllerStartupRequest):
    raise MotionInputError("startup_request must be a ControllerStartupRequest")
  if cancel_event is None:
    cancel_event = threading.Event()
  if not callable(getattr(cancel_event, "is_set", None)):
    raise TypeError("controller startup cancellation must satisfy the event contract")
  auxiliary_serial = None
  auxiliary_error = None
  try:
    controller_identity = parse_controller_identity_response(
      _startup_exchange_response("HO\n", cancel_event)
    )
    if (
      CONTROLLER_CAPABILITY_JT_WRIST_CONFIG_V1
      not in controller_identity.protocol_capabilities
    ):
      raise ProtocolResponseError(
        "controller firmware lacks command-local JT wrist configuration"
      )
    if startup_request.auxiliary_port is None:
      _clear_unavailable_startup_auxiliary()
    elif startup_request.auxiliary_board is None:
      _clear_unavailable_startup_auxiliary()
      auxiliary_error = "auxiliary-board profile must be selected"
    else:
      try:
        auxiliary_serial = _connect_startup_auxiliary(
          startup_request.auxiliary_port,
          startup_request.auxiliary_board,
        )
      except Exception as exc:
        _clear_unavailable_startup_auxiliary()
        auxiliary_error = str(exc).strip() or type(exc).__name__
    _startup_exchange_response(
      startup_request.update_parameters_command,
      cancel_event,
      expected_response=b"Done",
    )
    _startup_exchange_response(
      startup_request.external_axis_command,
      cancel_event,
      expected_response=b"Done",
    )
    _startup_exchange_response(
      startup_request.position_command,
      cancel_event,
      expected_response=b"Done\n",
    )
    position_text = _startup_exchange_response("RP\n", cancel_event)
    position = parse_position_response(position_text)
    if position.flag:
      raise ProtocolResponseError(
        f"controller reported a startup fault: {position.flag}"
      )
    return ControllerStartupResult(
      position=position,
      visual_options=_startup_visual_options(),
      auxiliary_serial=auxiliary_serial,
      auxiliary_error=auxiliary_error,
    )
  except BaseException:
    if auxiliary_serial is not None:
      _request_startup_auxiliary_cleanup(auxiliary_serial)
    raise




###############################################################################################################################################################
### COMMUNICATION DEFS ################################################################################################################# COMMUNICATION DEFS ###
###############################################################################################################################################################

###############################################################################################################################################################
def _prepare_controller_startup():
  auxiliary_port = com2SelectedValue.get()
  auxiliary_board = auxiliaryBoardSelectedValue.get()
  (
    update_values,
    update_command,
    external_values,
    external_command,
  ) = _prepare_controller_calibration()
  merged_values = dict(CAL)
  merged_values.update(update_values)
  merged_values.update(external_values)
  _prepare_cpp_kinematics_configuration(merged_values)
  request = ControllerStartupRequest(
    auxiliary_port=auxiliary_port,
    auxiliary_board=auxiliary_board,
    update_parameters_command=update_command,
    external_axis_command=external_command,
    position_command=_prepare_position_command(merged_values),
  )
  return request, update_values, external_values


def _apply_controller_startup_result(
  startup_request,
  result,
  update_values,
  external_values,
  startup_serial,
  original_startup_timeout,
):
  staged_values = dict(CAL)
  staged_values.update(update_values)
  staged_values.update(external_values)
  staged_calibration = _controller_joint_calibration_from_values(staged_values)
  staged_calibration.validate_positions(
    result.position.joints + result.position.external
  )

  calibration_applied = False
  try:
    _apply_controller_calibration(update_values, external_values)
    calibration_applied = True
    applied_position = displayPosition(
      result.position.raw,
      parsed=result.position,
    )
    if applied_position is None:
      raise ProtocolResponseError(
        "controller startup position could not be applied"
      )
    updateVisOp(result.visual_options)
    startup_serial.timeout = original_startup_timeout
    value = tab8.ElogView.get(0, END)
    with open("ErrorLog", "wb") as error_log:
      pickle.dump(value, error_log)
    CAL['com2Port'] = startup_request.auxiliary_port
    CAL['auxiliaryBoard'] = (
      startup_request.auxiliary_board or AUXILIARY_BOARD_NONE
    )
  except Exception:
    if calibration_applied:
      try:
        _invalidate_joint_motion_state(
          "controller startup finalization failed after staged calibration "
          "was applied"
        )
      except Exception:
        logger.exception(
          "Unable to invalidate motion after controller startup finalization"
        )
    raise
  return applied_position


def setCom(misc=None):
  request_lease = _acquire_motion_request(
    "Controller connection change",
    allow_position_recovery=True,
    requires_kinematics=False,
  )
  if request_lease is None:
    message = _motion_request_rejection_message(
      "Controller connection change not started"
    )
    almStatusLab.config(text=message, style="Warn.TLabel")
    almStatusLab2.config(text=message, style="Warn.TLabel")
    return False
  request_state = {"transferred": False}
  try:
    return _set_com_admitted(request_lease, request_state, misc)
  finally:
    if (
      not request_state["transferred"]
      and motion_request_registry.owns(request_lease)
    ):
      _finish_motion_request(request_lease)


def _set_com_admitted(request_lease, request_state, misc=None):
  if application_closing.is_set():
    logger.warning("Controller connection change rejected during application shutdown")
    return False
  if not serial_lock.acquire(blocking=False):
    message = "Controller connection change rejected while the transport is busy"
    logger.warning(message)
    almStatusLab.config(text=message, style="Warn.TLabel")
    almStatusLab2.config(text=message, style="Warn.TLabel")
    return False
  try:
    activity_lease = serial_activity_registry.lease("ser")
  except SerialActivityRejected as exc:
    serial_lock.release()
    message = f"Controller connection change rejected: {exc}"
    logger.warning(message)
    almStatusLab.config(text=message, style="Warn.TLabel")
    almStatusLab2.config(text=message, style="Warn.TLabel")
    return False
  except Exception:
    serial_lock.release()
    raise
  release_transport = True
  try:
    selected_main_port = com1SelectedValue.get()
    if not isinstance(selected_main_port, str):
      raise MotionInputError("main controller port must be text")
    selected_main_port = selected_main_port.strip()
    baud = 9600

    existing_serial = RUN.get('ser')
    if existing_serial is not None:
      if getattr(existing_serial, "is_open", False):
        existing_serial.close()
        if getattr(existing_serial, "is_open", False):
          raise OSError("Existing Teensy serial connection remained open")
        time.sleep(0.2)  # Windows can retain a just-closed COM handle briefly.
      if RUN.get('ser') is existing_serial:
        RUN['ser'] = None

    if selected_main_port in ("", "None"):
      raise ValueError("No COM port selected")
    # Command-specific read deadlines are applied by the owned transport path.
    RUN['ser'] = serial.Serial(
      port=selected_main_port,
      baudrate=baud,
      timeout=None,
      write_timeout=SERIAL_WRITE_TIMEOUT_SECONDS,
    )
    logger.info(
      "COMMUNICATIONS STARTED WITH TEENSY 4.1 CONTROLLER on Port %s",
      selected_main_port,
    )

    almStatusLab.config(text="CONTROLLER STARTING", style="Warn.TLabel")
    almStatusLab2.config(text="CONTROLLER STARTING", style="Warn.TLabel")

    time.sleep(.1)
    # Prefer reset_input_buffer over deprecated flushInput
    try:
      RUN['ser'].reset_input_buffer()
      RUN['ser'].reset_output_buffer()
    except Exception as exc:
      raise OSError("Unable to reset controller serial buffers") from exc

    (
      startup_request,
      startup_update_values,
      startup_external_values,
    ) = _prepare_controller_startup()
    startup_serial = RUN['ser']
    original_startup_timeout = startup_serial.timeout
    startup_serial.timeout = SERIAL_STARTUP_READ_TIMEOUT_SECONDS

    def set_startup_status(message, style):
      try:
        almStatusLab.config(text=message, style=style)
        almStatusLab2.config(text=message, style=style)
      except Exception:
        logger.exception("Unable to update controller startup status")

    def report_startup_timeout():
      message = "CONTROLLER STARTUP TIMED OUT; WAITING FOR CLEANUP"
      logger.error(message)
      set_startup_status(message, "Alarm.TLabel")

    def finish_startup(result, timed_out):
      if RUN.get('ser') is not startup_serial:
        logger.error("Main serial reference changed during controller startup")
        if isinstance(result, ControllerStartupResult):
          _request_startup_auxiliary_cleanup(result.auxiliary_serial)
        _poll_failed_controller_close(
          startup_serial,
          activity_lease,
          request_lease,
        )
        return

      if (
        timed_out
        or isinstance(result, BaseException)
        or not isinstance(result, ControllerStartupResult)
      ):
        if isinstance(result, BaseException) and not (
          timed_out and isinstance(result, TimeoutError)
        ):
          logger.error(
            "Startup failed while initializing Teensy 4.1 controller",
            exc_info=(type(result), result, result.__traceback__),
          )
        elif not isinstance(result, (BaseException, ControllerStartupResult)):
          logger.error("Controller startup worker returned an invalid result")
        if isinstance(result, ControllerStartupResult):
          _request_startup_auxiliary_cleanup(result.auxiliary_serial)
        message = (
          "CONTROLLER STARTUP TIMED OUT; CONNECTION CLOSING"
          if timed_out
          else "UNABLE TO ESTABLISH COMMUNICATIONS WITH TEENSY 4.1 CONTROLLER"
        )
        set_startup_status(message, "Alarm.TLabel")
        _poll_failed_controller_close(
          startup_serial,
          activity_lease,
          request_lease,
        )
        return

      try:
        if not getattr(startup_serial, "is_open", False):
          raise OSError("Teensy serial connection closed during startup")
        startup_position = _apply_controller_startup_result(
          startup_request,
          result,
          startup_update_values,
          startup_external_values,
          startup_serial,
          original_startup_timeout,
        )
      except Exception:
        logger.exception("Unable to finalize the Teensy 4.1 controller connection")
        _request_startup_auxiliary_cleanup(result.auxiliary_serial)
        message = "UNABLE TO ESTABLISH COMMUNICATIONS WITH TEENSY 4.1 CONTROLLER"
        set_startup_status(message, "Alarm.TLabel")
        _poll_failed_controller_close(
          startup_serial,
          activity_lease,
          request_lease,
        )
        return

      logger.info("COMMUNICATIONS STARTED WITH TEENSY 4.1 CONTROLLER")
      if result.auxiliary_error is not None:
        logger.warning(
          "Auxiliary controller unavailable during main startup: %s",
          result.auxiliary_error,
        )
      if not startup_position.speed_violation:
        if result.auxiliary_error is not None:
          set_startup_status(
            "SYSTEM READY; AUXILIARY CONTROLLER UNAVAILABLE",
            "Warn.TLabel",
          )
        elif startup_request.auxiliary_port is None:
          set_startup_status(
            "SYSTEM READY; AUXILIARY CONTROLLER NOT CONFIGURED",
            "Warn.TLabel",
          )
        else:
          set_startup_status("SYSTEM READY", "OK.TLabel")
      CAL['comPort'] = selected_main_port
      _release_async_main_serial_transport(activity_lease, request_lease)

    def abandon_startup(result):
      if isinstance(result, ControllerStartupResult):
        _request_startup_auxiliary_cleanup(result.auxiliary_serial)
      _abandon_failed_controller_startup(
        startup_serial,
        activity_lease,
        request_lease,
      )

    release_transport = False
    try:
      startup_with_spinner(
        root,
        startup_request,
        finish_startup,
        report_startup_timeout,
        abandon_startup,
        timeout=SERIAL_STARTUP_READ_TIMEOUT_SECONDS,
      )
      request_state["transferred"] = True
    except Exception:
      release_transport = True
      raise
    return True

  except Exception:
    failed_serial = RUN.get('ser')
    if failed_serial is not None:
      release_transport = False
      _poll_failed_controller_close(
        failed_serial,
        activity_lease,
        request_lease,
      )
      request_state["transferred"] = True

    logger.exception(
      "UNABLE TO ESTABLISH COMMUNICATIONS WITH TEENSY 4.1 CONTROLLER"
    )

    almStatusLab.config(text="UNABLE TO ESTABLISH COMMUNICATIONS WITH TEENSY 4.1 CONTROLLER", style="Alarm.TLabel")
    almStatusLab2.config(text="UNABLE TO ESTABLISH COMMUNICATIONS WITH TEENSY 4.1 CONTROLLER", style="Alarm.TLabel")

    try:
      value = tab8.ElogView.get(0, END)
      pickle.dump(value, open("ErrorLog", "wb"))
    except Exception:
      logger.exception("Unable to persist the controller startup error log")
    return False
  finally:
    if release_transport:
      _release_async_main_serial_transport(activity_lease, request_lease)


def _replace_auxiliary_serial(port, board_profile):
  if not isinstance(port, str):
    raise MotionInputError("auxiliary controller port must be text")
  port = port.strip()
  if port in ("", "None"):
    raise MotionInputError("no auxiliary controller port selected")
  board_profile = normalize_auxiliary_board_profile(board_profile)

  existing_serial = RUN.get('ser2')
  if existing_serial is not None:
    if getattr(existing_serial, "is_open", False):
      existing_serial.close()
      if getattr(existing_serial, "is_open", False):
        raise OSError("Existing auxiliary serial connection remained open")
    if RUN.get('ser2') is existing_serial:
      RUN['ser2'] = None
      _clear_auxiliary_board_profile(existing_serial)

  replacement = serial.Serial(
    port=port,
    baudrate=9600,
    timeout=SERIAL_AUXILIARY_RESPONSE_TIMEOUT_SECONDS,
    write_timeout=SERIAL_WRITE_TIMEOUT_SECONDS,
  )
  if not getattr(replacement, "is_open", False):
    try:
      replacement.close()
    finally:
      raise OSError("Auxiliary serial connection did not open")
  RUN['ser2'] = replacement
  try:
    _bind_auxiliary_board_profile(replacement, board_profile)
  except Exception as bind_error:
    close_error = None
    try:
      replacement.close()
    except Exception as exc:
      close_error = exc
      logger.exception(
        "Unable to close auxiliary replacement after profile binding failed"
      )
    closed = not getattr(replacement, "is_open", False)
    _clear_auxiliary_board_profile(replacement)
    if closed:
      if RUN.get('ser2') is replacement:
        RUN['ser2'] = None
    if close_error is not None:
      raise OSError(
        "Auxiliary profile binding failed and the replacement remained open"
      ) from bind_error
    if not closed:
      raise OSError(
        "Auxiliary profile binding failed and the replacement remained open"
      ) from bind_error
    raise
  return replacement


@_synchronous_motion_request(
  "Auxiliary connection change",
  requires_kinematics=False,
)
def setCom2(misc=None):
  if application_closing.is_set():
    logger.warning(
      "Auxiliary connection change rejected during application shutdown"
    )
    return False
  if not auxiliary_serial_lock.acquire(blocking=False):
    logger.warning("Auxiliary connection change rejected while the transport is busy")
    return False
  previous_port = CAL.get('com2Port', "None")
  previous_board = CAL.get('auxiliaryBoard', AUXILIARY_BOARD_NONE)
  try:
    selected_port = com2SelectedValue.get()
    if not isinstance(selected_port, str):
      raise MotionInputError("auxiliary controller port must be text")
    selected_port = selected_port.strip()
    if selected_port in ("", "None"):
      selected_port = None
    selected_board = normalize_auxiliary_board_profile(
      auxiliaryBoardSelectedValue.get(),
      allow_none=True,
    )
    if selected_port is None or selected_board is None:
      if RUN.get('ser2') is not None and not _close_serial_port(
        'ser2',
        "auxiliary configuration change",
      ):
        raise OSError("Unable to close the prior auxiliary connection")
      _clear_auxiliary_board_profile()
      logger.warning(
        "Auxiliary controller disabled until both a COM port and board profile "
        "are selected"
      )
    else:
      _replace_auxiliary_serial(selected_port, selected_board)
      logger.info(
        "COMMUNICATIONS STARTED WITH %s ARDUINO IO BOARD on port: %s",
        selected_board,
        selected_port,
      )

    committed_port = selected_port or "None"
    committed_board = selected_board or AUXILIARY_BOARD_NONE
    CAL['com2Port'] = committed_port
    CAL['auxiliaryBoard'] = committed_board
    _retain_calibration_persistence_retry()
    try:
      value = tab8.ElogView.get(0, END)
      pickle.dump(value, open("ErrorLog", "wb"))
    except Exception:
      logger.exception("Unable to persist the auxiliary connection log")
    return True
  except Exception as e:
    CAL['com2Port'] = previous_port
    CAL['auxiliaryBoard'] = previous_board
    try:
      com2SelectedValue.set(previous_port)
      auxiliaryBoardSelectedValue.set(previous_board)
    except Exception:
      logger.exception("Unable to restore the auxiliary configuration selection")
    logger.error(
      "UNABLE TO ESTABLISH COMMUNICATIONS WITH ARDUINO IO BOARD: %s",
      e,
    )
    try:
      value = tab8.ElogView.get(0, END)
      pickle.dump(value, open("ErrorLog", "wb"))
    except Exception:
      logger.exception("Unable to persist the auxiliary connection failure log")
    return False
  finally:
    auxiliary_serial_lock.release()
    
def darkTheme():
  CAL['curTheme'] = 0
  # Use the existing style instance and switch theme
  if hasattr(root, 'style'):
    style = root.style
    style.theme_use("darkly")
  else:
    style = BootstrapStyle(theme="darkly")
    root.style = style
  
  # Configure custom styles for the darkly theme
  style.configure("Alarm.TLabel", foreground="#dc3545", font = ('Arial','10','bold'))  # Bootstrap danger color
  style.configure("Warn.TLabel", foreground="#fd7e14", font = ('Arial','10','bold'))   # Bootstrap warning color
  style.configure("OK.TLabel", foreground="#198754", font = ('Arial','10','bold'))     # Bootstrap success color
  style.configure("Jointlim.TLabel", foreground="#0dcaf0", font = ('Arial','8'))      # Bootstrap info color
  style.configure('AlarmBut.TButton', foreground ='#dc3545')                          # Bootstrap danger color
  style.configure('Frame1.TFrame', background='#ffffff')
  style.configure("Offline.TButton", foreground="#198754", font = ('Arial','8','bold'))  # Bootstrap success color
  style.configure("Online.TButton")
  # Configure Entry widgets for better alignment with buttons
  style.configure("TEntry", 
                  fieldbackground="#495057",  # Dark background for darkly theme
                  borderwidth=1,
                  insertcolor="#ffffff",      # White cursor
                  padding=(1, 0, 1, 0))      # More aggressive padding reduction: left, top, right, bottom
  
  # Configure Button widgets for subtle size reduction
  style.configure("TButton", 
                  padding=(5, 3, 5, 3))     # Very subtle padding reduction: just slightly smaller than default
  
  # Configure OptionMenu widgets to match button proportions
  style.configure("TMenubutton", 
                  padding=(5, 3, 5, 3))     # Match button padding for proportional scaling


def lightTheme():
  CAL['curTheme'] = 1
  # Use the existing style instance and switch theme
  if hasattr(root, 'style'):
    style = root.style
    style.theme_use("flatly")  # Changed to sandstone theme
  else:
    style = BootstrapStyle(theme="flatly")  # Changed to sandstone theme
    root.style = style
  
  # Configure custom styles for the light theme
  style.configure("Alarm.TLabel", foreground="#dc3545", font = ('Arial','10','bold'))  # Bootstrap danger color
  style.configure("Warn.TLabel", foreground="#fd7e14", font = ('Arial','10','bold'))   # Bootstrap warning color
  style.configure("OK.TLabel", foreground="#198754", font = ('Arial','10','bold'))     # Bootstrap success color
  style.configure("Jointlim.TLabel", foreground="#0d6efd", font = ('Arial','8'))      # Bootstrap primary color
  style.configure('AlarmBut.TButton', foreground ='#dc3545')                          # Bootstrap danger color
  style.configure('Frame1.TFrame', background='#000000')
  style.configure("Offline.TButton", foreground="#fd7e14")                            # Bootstrap warning color
  style.configure("Online.TButton", foreground="#000000")
  style.configure("Offline.TButton", foreground="#198754", font = ('Arial','8','bold'))  # Bootstrap success color
  style.configure("Online.TButton")
  # Configure Entry widgets for better alignment with buttons
  style.configure("TEntry", 
                  fieldbackground="#ffffff",  # White background for light theme
                  borderwidth=1,
                  insertcolor="#000000",      # Black cursor
                  padding=(1, 0, 1, 0))      # More aggressive padding reduction: left, top, right, bottom
  
  # Configure Button widgets for subtle size reduction
  style.configure("TButton", 
                  padding=(5, 3, 5, 3))     # Very subtle padding reduction: just slightly smaller than default
  
  # Configure OptionMenu widgets to match button proportions
  style.configure("TMenubutton", 
                  padding=(5, 3, 5, 3))     # Match button padding for proportional scaling



###############################################################################################################################################################  
### EXECUTION DEFS ######################################################################################################################### EXECUTION DEFS ###  
############################################################################################################################################################### 

ROW_EXECUTION_COMPLETE = "complete"
ROW_EXECUTION_PENDING = "pending"
ROW_EXECUTION_REJECTED = "rejected"
VIRTUAL_COMPLETION_POLL_MS = 10
VIRTUAL_COMPLETION_BASE_TIMEOUT_SECONDS = 120
VIRTUAL_COMPLETION_SAFETY_SCALE = 1.25
VIRTUAL_JOINT_SECONDS_SCALE = 4.5
VIRTUAL_CARTESIAN_SECONDS_SCALE = 4.7
VIRTUAL_TOOL_SECONDS_SCALE = 5.1
LIVE_TOOL_JOG_INCREMENT = 0.25


@dataclass(frozen=True)
class ProgramMotionPoseSnapshot:
  controller_positions: Optional[tuple]
  virtual_pose: tuple


def _apply_valid_position_response(response):
  try:
    parsed = parse_position_response(response)
  except ProtocolResponseError:
    displayPosition(response)
    return None
  applied_position = displayPosition(response, parsed=parsed)
  if applied_position is None or applied_position.flag:
    return None
  return applied_position


def _exchange_legacy_main_command(
  command,
  *,
  read_line=True,
  response_timeout=SERIAL_BASE_RESPONSE_TIMEOUT_SECONDS,
  accepted_responses=None,
  expected_response=None,
):
  if not isinstance(read_line, bool):
    raise TypeError("read_line must be boolean")
  response_timeout = finite_number(
    response_timeout,
    "legacy controller response timeout",
  )
  if response_timeout <= 0:
    raise MotionInputError("legacy controller response timeout must be positive")
  if read_line == (expected_response is not None):
    raise MotionInputError(
      "legacy controller commands require exactly one framed or exact response contract"
    )
  if not read_line and accepted_responses is not None:
    raise MotionInputError(
      "accepted legacy responses require line-response ownership"
    )

  serial_port = RUN.get('ser')
  try:
    write_serial_control(
      serial_port,
      command,
      write_lock=serial_write_lock,
      reset_input=True,
    )
    if read_line:
      return read_serial_line_response(
        serial_port,
        response_timeout,
        accepted_responses=accepted_responses,
      )
    return read_serial_exact_response(
      serial_port,
      expected_response,
      response_timeout,
    )
  finally:
    if (
      RUN.get('ser') is serial_port
      and not getattr(serial_port, "is_open", False)
    ):
      RUN['ser'] = None


@_synchronous_motion_request(
  "Program controller command",
  rejection_result=(ROW_EXECUTION_REJECTED, None),
  requires_kinematics=False,
)
@_tracked_serial_operation(
  "ser",
  rejection_result=(ROW_EXECUTION_REJECTED, None),
)
def _execute_row_main_command(command, **response_contract):
  try:
    response = _exchange_legacy_main_command(command, **response_contract)
  except Exception as exc:
    message = f"Program controller command failed: {exc}"
    logger.exception(message)
    almStatusLab.config(text=message, style="Alarm.TLabel")
    almStatusLab2.config(text=message, style="Alarm.TLabel")
    return ROW_EXECUTION_REJECTED, None
  return ROW_EXECUTION_COMPLETE, response


def _execute_row_main_response(
  command,
  *,
  response_parser=None,
  **response_contract,
):
  if response_parser is not None and not callable(response_parser):
    raise TypeError("controller response parser must be callable or None")
  execution_state, response = _execute_row_main_command(
    command,
    **response_contract,
  )
  if execution_state != ROW_EXECUTION_COMPLETE:
    _finish_execute_row()
    return execution_state, None
  if response_parser is not None:
    try:
      response = response_parser(command, response)
    except ProtocolResponseError as exc:
      message = f"Program controller response rejected: {exc}"
      logger.error(message)
      almStatusLab.config(text=message, style="Alarm.TLabel")
      almStatusLab2.config(text=message, style="Alarm.TLabel")
      _finish_execute_row()
      return ROW_EXECUTION_REJECTED, None
  return execution_state, response


def _apply_controller_position_response(response):
  if isinstance(response, str) and response.startswith("E"):
    _invalidate_joint_motion_state(f"controller rejected request: {response}")
    ErrorHandler(response)
    return False
  return _apply_valid_position_response(response) is not None


def _finish_execute_row():
  RUN['VR_angles'] = [
    float(CAL['J1AngCur']),
    float(CAL['J2AngCur']),
    float(CAL['J3AngCur']),
    float(CAL['J4AngCur']),
    float(CAL['J5AngCur']),
    float(CAL['J6AngCur']),
  ]
  setStepMonitorsVR()
  RUN['progRunning'] = False
  RUN['rowinproc'] = 0


def _execute_row_auxiliary_command(
  command,
  *,
  read_line=False,
  control_injectable=False,
  response_delay=0.1,
  response_timeout=SERIAL_AUXILIARY_RESPONSE_TIMEOUT_SECONDS,
  accepted_responses=None,
  expected_response=None,
):
  if not isinstance(read_line, bool):
    raise TypeError("read_line must be boolean")
  if not isinstance(control_injectable, bool):
    raise TypeError("control_injectable must be boolean")
  response_delay = finite_number(response_delay, "auxiliary response delay")
  if response_delay < 0:
    raise MotionInputError("auxiliary response delay must not be negative")
  response_timeout = finite_number(
    response_timeout,
    "auxiliary response timeout",
  )
  if response_timeout <= 0:
    raise MotionInputError("auxiliary response timeout must be positive")
  if control_injectable and (
    not read_line or accepted_responses is None
  ):
    raise MotionInputError(
      "injectable auxiliary operations require validated line responses"
    )
  if read_line == (expected_response is not None):
    raise MotionInputError(
      "auxiliary commands require exactly one framed or exact response contract"
    )
  if not read_line and accepted_responses is not None:
    raise MotionInputError(
      "accepted auxiliary responses require line-response ownership"
    )
  if expected_response is not None and (
    not isinstance(expected_response, bytes)
    or not expected_response
    or len(expected_response) > MAX_RESPONSE_PAYLOAD_LENGTH
    or expected_response != expected_response.strip()
    or b"\r" in expected_response
    or b"\n" in expected_response
    or not expected_response.isascii()
  ):
    raise MotionInputError(
      "expected auxiliary response must contain normalized unframed ASCII bytes"
    )
  try:
    with _tracked_auxiliary_operation(
      control_injectable=control_injectable,
    ):
      _write_legacy_auxiliary_command(command)
      time.sleep(response_delay)
      serial_port = RUN.get('ser2')
      try:
        if read_line:
          if control_injectable:
            _begin_auxiliary_stop_owner_wait()
          try:
            if control_injectable:
              response, stop_response = (
                read_serial_line_response_with_optional_followup(
                  serial_port,
                  response_timeout,
                  accepted_responses=accepted_responses,
                  followup_after_responses=AUXILIARY_WAIT_NATURAL_RESPONSES,
                  accepted_followup_responses=(
                    AUXILIARY_INACTIVE_STOP_RESPONSE,
                  ),
                  control_event=auxiliary_stop_injected_event,
                  control_response_timeout_seconds=(
                    SERIAL_AUXILIARY_RESPONSE_TIMEOUT_SECONDS
                  ),
                  control_response_deadline_provider=(
                    _auxiliary_stop_acknowledgement_deadline_value
                  ),
                )
              )
            else:
              response = read_serial_line_response(
                serial_port,
                response_timeout,
                accepted_responses=accepted_responses,
              )
              stop_response = None
          except Exception as exc:
            if control_injectable:
              _publish_auxiliary_stop_owner_result(False, str(exc))
            raise
          else:
            if control_injectable:
              stop_result_published = _publish_auxiliary_stop_owner_result(
                True,
                response if stop_response is None else stop_response,
              )
              if stop_response is not None and not stop_result_published:
                quarantine_serial_transport(
                  serial_port,
                  "auxiliary stop acknowledgement had no active stop request",
                )
                raise ProtocolResponseError(
                  "auxiliary stop acknowledgement had no active stop request"
                )
        else:
          response = read_serial_exact_response(
            serial_port,
            expected_response,
            response_timeout,
          )
      finally:
        if (
          RUN.get('ser2') is serial_port
          and not getattr(serial_port, "is_open", False)
        ):
          RUN['ser2'] = None
          _clear_auxiliary_board_profile(serial_port)
  except SerialActivityRejected as exc:
    logger.warning("Program auxiliary command rejected: %s", exc)
    _finish_execute_row()
    return ROW_EXECUTION_REJECTED, None
  except Exception as exc:
    message = f"Program auxiliary command failed: {exc}"
    logger.exception(message)
    almStatusLab.config(text=message, style="Alarm.TLabel")
    almStatusLab2.config(text=message, style="Alarm.TLabel")
    _finish_execute_row()
    return ROW_EXECUTION_REJECTED, None
  return ROW_EXECUTION_COMPLETE, response


def _auxiliary_wait_timeout_seconds(value):
  if isinstance(value, bool):
    raise MotionInputError("auxiliary wait timeout must be an integer")
  text = str(value).strip()
  if not re.fullmatch(r"[0-9]+", text):
    raise MotionInputError(
      "auxiliary wait timeout must be a non-negative integer"
    )
  timeout = int(text)
  if timeout > AUXILIARY_FIRMWARE_SIGNED_INT_MAX:
    raise MotionInputError("auxiliary wait timeout exceeds the firmware range")
  return timeout


def _main_modbus_wait_timeout_seconds(value):
  if isinstance(value, bool):
    raise MotionInputError("controller Modbus wait timeout must be an integer")
  text = str(value).strip()
  if not re.fullmatch(r"[0-9]+", text):
    raise MotionInputError(
      "controller Modbus wait timeout must be a positive integer"
    )
  timeout = int(text)
  if timeout == 0:
    raise MotionInputError(
      "controller Modbus wait timeout must be a positive integer"
    )
  if timeout > MAIN_FIRMWARE_WAIT_MAX_SECONDS:
    raise MotionInputError(
      "controller Modbus wait timeout exceeds the firmware range"
    )
  return timeout


def _main_timed_wait_seconds(value):
  wait_seconds = finite_number(value, "controller timed wait")
  if wait_seconds < 0:
    raise MotionInputError("controller timed wait must be non-negative")
  encoded = controller_protocol_decimal(wait_seconds, "controller timed wait")
  encoded_seconds = float(encoded)
  if encoded_seconds > MAIN_FIRMWARE_WAIT_MAX_SECONDS:
    raise MotionInputError("controller timed wait exceeds the firmware range")
  return encoded_seconds, encoded


def _finish_step_reverse_selection(sel_row):
  last = tab1.progView.index('end')
  for row in range(0, sel_row):
    tab1.progView.itemconfig(row, {'fg': "#9E9E9E"})
  tab1.progView.itemconfig(sel_row, {'fg': "#FF0000"})
  for row in range(sel_row + 1, last):
    tab1.progView.itemconfig(row, {'fg': "#EE5C42"})
  tab1.progView.selection_clear(0, END)
  sel_row -= 1
  tab1.progView.select_set(sel_row)
  try:
    selected_row = tab1.progView.curselection()[0]
    curRowEntryField.delete(0, 'end')
    curRowEntryField.insert(0, selected_row)
  except Exception:
    curRowEntryField.delete(0, 'end')
    curRowEntryField.insert(0, "---")


def _virtual_motion_active(ignored_request_lease=None):
  if (
    ignored_request_lease is not None
    and not motion_request_registry.owns(ignored_request_lease)
  ):
    raise RuntimeError("ignored virtual-motion request lease is not active")
  with offline_live_jog_state_lock:
    if offline_live_jog_operation is not None:
      return True
  return (
    (
      motion_request_registry.active
      and ignored_request_lease is None
    )
    or offline_live_jog_lock.locked()
    or live_jog_lock.locked()
    or live_cartesian_lock.locked()
    or live_tool_lock.locked()
    or drive_lock.locked()
  )


def _motion_request_rejection_message(fallback):
  if not isinstance(fallback, str) or not fallback:
    raise TypeError("motion request rejection fallback must be non-empty text")
  reason = getattr(motion_request_admission_state, "rejection_reason", None)
  if isinstance(reason, str) and reason:
    return reason
  return fallback


def _reject_motion_request(name, reason):
  message = f"{name} not started; {reason}"
  motion_request_admission_state.rejection_reason = message
  logger.warning(message)
  return None


def _acquire_motion_request(
  name,
  allow_position_recovery=False,
  requires_kinematics=True,
):
  if not isinstance(allow_position_recovery, bool):
    raise TypeError("position-recovery admission flag must be boolean")
  if not isinstance(requires_kinematics, bool):
    raise TypeError("kinematics admission flag must be boolean")
  motion_request_admission_state.rejection_reason = None
  if application_closing.is_set():
    return _reject_motion_request(name, "application shutdown is active")
  if requires_kinematics and not kinematics_configuration_ready.is_set():
    return _reject_motion_request(
      name,
      "native kinematics configuration is unavailable",
    )
  if not allow_position_recovery and (
    manual_motion_pose_pending.is_set()
    or controller_position_resynchronization_required.is_set()
  ):
    return _reject_motion_request(
      name,
      "controller position resynchronization is required",
    )
  lease = motion_request_registry.acquire(name)
  if lease is None:
    owner = motion_request_registry.active_name
    return _reject_motion_request(
      name,
      f"{owner or 'another request'} owns motion",
    )
  return lease


def _finish_motion_request(
  request_lease,
  completion_callback=None,
  succeeded=None,
):
  if not isinstance(request_lease, MotionRequestLease):
    raise TypeError("motion request completion requires a valid lease")
  if completion_callback is not None and not callable(completion_callback):
    raise TypeError("motion request completion callback must be callable")
  if completion_callback is not None and not isinstance(succeeded, bool):
    raise TypeError("motion request callback requires a boolean result")
  if request_lease.close() is not True:
    raise RuntimeError("motion request result reused a released lease")
  try:
    if completion_callback is not None:
      completion_callback(succeeded)
  finally:
    virtual_motion_event_queue.put(("motion-released", None))
  return True


def _manual_motion_request(name):
  def decorate(callback):
    @wraps(callback)
    def guarded(*args, **kwargs):
      # Admission precedes coordinate reads; only an admitted virtual request
      # transfers this lease beyond the callback's synchronous lifetime.
      request_lease = _acquire_motion_request(name)
      if request_lease is None:
        message = _motion_request_rejection_message(
          f"{name} not started; another motion request is active"
        )
        almStatusLab.config(text=message, style="Warn.TLabel")
        almStatusLab2.config(text=message, style="Warn.TLabel")
        return False
      if getattr(manual_motion_request_state, "lease", None) is not None:
        _finish_motion_request(request_lease)
        raise RuntimeError("nested manual motion request is not supported")
      manual_motion_request_state.lease = request_lease
      manual_motion_request_state.transferred = False
      try:
        return callback(*args, **kwargs)
      finally:
        transferred = manual_motion_request_state.transferred
        manual_motion_request_state.lease = None
        manual_motion_request_state.transferred = False
        if not transferred and motion_request_registry.owns(request_lease):
          _finish_motion_request(request_lease)
    return guarded
  return decorate


def _start_manual_motion(
  physical_command,
  motion_name,
  virtual_dispatch,
  virtual_command,
):
  request_lease = getattr(manual_motion_request_state, "lease", None)
  if not motion_request_registry.owns(request_lease):
    raise RuntimeError("manual motion requires matching request ownership")
  if not callable(virtual_dispatch):
    raise TypeError("manual virtual dispatch must be callable")

  try:
    virtual_wrist_config = parse_motion_wrist_config(
      virtual_command,
      virtual=True,
    )
    if physical_command is not None:
      physical_wrist_config = parse_motion_wrist_config(
        physical_command,
        virtual=False,
      )
      if physical_wrist_config != virtual_wrist_config:
        raise MotionInputError(
          "physical and virtual wrist configurations must match"
        )
  except (TypeError, ValueError, MotionInputError) as exc:
    message = f"{motion_name} rejected because the wrist mode is invalid: {exc}"
    logger.error(message)
    almStatusLab.config(text=message, style="Alarm.TLabel")
    almStatusLab2.config(text=message, style="Alarm.TLabel")
    return False

  try:
    if RUN['offlineMode']:
      saved_controller_positions = None
      saved_virtual_pose = tuple(
        finite_number(value, f"saved virtual joint {axis}")
        for axis, value in enumerate(RUN['VR_angles'], start=1)
      )
      if len(saved_virtual_pose) != 6:
        raise MotionInputError("saved virtual pose must contain six joints")
    else:
      saved_controller_positions = _current_joint_positions()
      saved_virtual_pose = saved_controller_positions[:6]
  except (KeyError, TypeError, ValueError, MotionInputError) as exc:
    message = f"{motion_name} rejected because the confirmed pose is invalid: {exc}"
    logger.error(message)
    almStatusLab.config(text=message, style="Alarm.TLabel")
    almStatusLab2.config(text=message, style="Alarm.TLabel")
    return False

  def apply_confirmed_pose(controller_positions, virtual_pose):
    try:
      if controller_positions is not None and (
        joint_motion_dispatcher.synchronize(controller_positions) is not True
      ):
        raise MotionQueueFault(
          "joint dispatcher rejected confirmed-position synchronization"
        )
      if controller_positions is None:
        if refresh_gui_from_joint_angles(virtual_pose) is not True:
          raise MotionQueueFault("virtual GUI rejected confirmed position")
      elif _try_set_virtual_joint_target(virtual_pose) is not True:
        raise MotionQueueFault("virtual model rejected confirmed position")
    except (KeyError, TypeError, ValueError, MotionInputError, MotionQueueFault) as exc:
      logger.error("Unable to restore confirmed manual-motion pose: %s", exc)
      return False
    return True

  def require_controller_resynchronization():
    controller_position_resynchronization_required.set()

  def restore_untransmitted_preview():
    restored = apply_confirmed_pose(
      saved_controller_positions,
      saved_virtual_pose,
    )
    if not restored and not RUN['offlineMode']:
      require_controller_resynchronization()
    return restored

  try:
    virtual_completion_timeout = _virtual_completion_timeout(virtual_command)
    virtual_operation = virtual_dispatch(virtual_command)
  except Exception as exc:
    restore_untransmitted_preview()
    message = f"{motion_name} virtual preview did not start: {exc}"
    logger.exception(message)
    almStatusLab.config(text=message, style="Alarm.TLabel")
    almStatusLab2.config(text=message, style="Alarm.TLabel")
    return False
  if not isinstance(virtual_operation, VirtualMotionOperation):
    restore_untransmitted_preview()
    message = f"{motion_name} virtual preview did not publish an operation"
    logger.error(message)
    almStatusLab.config(text=message, style="Alarm.TLabel")
    almStatusLab2.config(text=message, style="Alarm.TLabel")
    return False
  if virtual_operation.completed:
    virtual_succeeded, virtual_error = virtual_operation.result()
    if not virtual_succeeded:
      restore_untransmitted_preview()
      message = f"{motion_name} virtual preview failed: {virtual_error}"
      logger.error(message)
      almStatusLab.config(text=message, style="Alarm.TLabel")
      almStatusLab2.config(text=message, style="Alarm.TLabel")
      return False

  virtual_completion_deadline = time.monotonic() + virtual_completion_timeout
  controller_write_started = (
    threading.Event()
    if physical_command is not None
    else None
  )
  controller_state = {
    "succeeded": physical_command is None,
    "speed_violation": False,
  }
  if physical_command is not None:
    manual_motion_pose_pending.set()

  def reconcile_manual_motion(succeeded):
    pose_confirmed = True
    try:
      if physical_command is None:
        if succeeded:
          pose_confirmed = apply_confirmed_pose(
            None,
            tuple(RUN['VR_angles']),
          )
        else:
          pose_confirmed = restore_untransmitted_preview()
      elif controller_state["succeeded"]:
        try:
          controller_positions = _current_joint_positions()
        except (KeyError, TypeError, ValueError, MotionInputError) as exc:
          logger.error("Unable to read confirmed controller pose: %s", exc)
          pose_confirmed = False
        else:
          pose_confirmed = apply_confirmed_pose(
            controller_positions,
            controller_positions[:6],
          )
      elif not controller_write_started.is_set():
        pose_confirmed = restore_untransmitted_preview()
      else:
        pose_confirmed = False

      if physical_command is not None:
        if pose_confirmed:
          controller_position_resynchronization_required.clear()
        else:
          require_controller_resynchronization()
    finally:
      manual_motion_pose_pending.clear()

    return succeeded and pose_confirmed

  def complete_manual_motion(succeeded):
    if succeeded:
      if not controller_state["speed_violation"]:
        almStatusLab.config(text="SYSTEM READY", style="OK.TLabel")
        almStatusLab2.config(text="SYSTEM READY", style="OK.TLabel")
      return
    if controller_position_resynchronization_required.is_set():
      message = (
        f"{motion_name} failed; controller position resynchronization is required"
      )
    else:
      message = f"{motion_name} failed during controller or virtual settlement"
    logger.error(message)
    almStatusLab.config(text=message, style="Alarm.TLabel")
    almStatusLab2.config(text=message, style="Alarm.TLabel")

  def settle(controller_position):
    if physical_command is None:
      if controller_position is not None:
        raise RuntimeError(
          "offline manual motion received a physical controller result"
        )
      controller_state["succeeded"] = True
      controller_state["speed_violation"] = False
    else:
      if controller_position is not None and not isinstance(
        controller_position,
        PositionResponse,
      ):
        raise RuntimeError(
          "manual motion completion returned an invalid controller result"
        )
      controller_state["succeeded"] = controller_position is not None
      controller_state["speed_violation"] = (
        controller_position is not None
        and controller_position.speed_violation
      )
    _complete_program_motion_when_virtual_idle(
      complete_manual_motion,
      request_lease,
      virtual_operation,
      virtual_completion_timeout,
      controller_succeeded=controller_state["succeeded"],
      deadline=virtual_completion_deadline,
      settlement_callback=reconcile_manual_motion,
    )

  manual_motion_request_state.transferred = True
  if physical_command is None:
    settle(None)
    return True

  try:
    controller_started = _start_legacy_motion(
      physical_command,
      motion_name,
      completion_callback=settle,
      request_lease=request_lease,
      write_started_event=controller_write_started,
    )
  except Exception:
    logger.exception("%s controller worker failed during startup", motion_name)
    controller_started = False
  if not controller_started:
    settle(None)
  return controller_started


def _virtual_completion_timeout(command):
  timing = parse_virtual_command_timing(command)
  minimum_ramp_full_scale_seconds, millimeter_motion_distance_bound = (
    _configured_motion_timeout_bounds()
  )
  timeout = motion_timing_response_timeout(
    timing,
    VIRTUAL_COMPLETION_BASE_TIMEOUT_SECONDS,
    minimum_ramp_full_scale_seconds,
    millimeter_motion_distance_bound,
    margin_seconds=SERIAL_RESPONSE_MARGIN_SECONDS,
  )
  if timing.mode == "s":
    simulated_duration = timing.speed * max(
      VIRTUAL_JOINT_SECONDS_SCALE,
      VIRTUAL_CARTESIAN_SECONDS_SCALE,
      VIRTUAL_TOOL_SECONDS_SCALE,
    )
    timeout = max(
      timeout,
      simulated_duration * VIRTUAL_COMPLETION_SAFETY_SCALE
      + SERIAL_RESPONSE_MARGIN_SECONDS,
    )
  return timeout


def _finish_settled_motion_request(
  completion_callback,
  request_lease,
  succeeded,
  settlement_callback=None,
):
  if not isinstance(succeeded, bool):
    raise TypeError("motion settlement result must be boolean")
  if settlement_callback is not None:
    if not callable(settlement_callback):
      raise TypeError("motion settlement callback must be callable")
    try:
      succeeded = settlement_callback(succeeded)
    except Exception:
      logger.exception("Unable to reconcile motion state before ownership release")
      succeeded = False
    if not isinstance(succeeded, bool):
      logger.error("Motion settlement callback returned a non-boolean result")
      succeeded = False
  return _finish_motion_request(
    request_lease,
    completion_callback,
    succeeded,
  )


def _complete_program_motion_when_virtual_idle(
  completion_callback,
  request_lease,
  operation,
  timeout,
  controller_succeeded=True,
  deadline=None,
  timed_out=False,
  settlement_callback=None,
):
  if not isinstance(operation, VirtualMotionOperation):
    logger.error("Virtual program motion did not publish an operation result")
    _finish_settled_motion_request(
      completion_callback,
      request_lease,
      False,
      settlement_callback,
    )
    return
  if deadline is None:
    deadline = time.monotonic() + timeout
  if not timed_out and time.monotonic() >= deadline:
    logger.error("Virtual program motion did not finish before timeout")
    timed_out = True
  if not operation.completed:
    try:
      root.after(
        VIRTUAL_COMPLETION_POLL_MS,
        lambda: _complete_program_motion_when_virtual_idle(
          completion_callback,
          request_lease,
          operation,
          timeout,
          controller_succeeded,
          deadline,
          timed_out,
          settlement_callback,
        ),
      )
    except Exception:
      logger.exception("Unable to schedule virtual program completion")
      fallback_started = _start_program_completion_fallback(
        completion_callback,
        request_lease,
        operation,
        controller_succeeded,
        deadline,
        timed_out,
        settlement_callback,
      )
      if not fallback_started:
        _finish_settled_motion_request(
          completion_callback,
          request_lease,
          _program_completion_fallback_result(
            operation,
            controller_succeeded,
            deadline,
            timed_out,
          ),
          settlement_callback,
        )
    return
  succeeded, error = operation.result()
  if not succeeded:
    logger.error("Virtual program motion failed: %s", error)
  _finish_settled_motion_request(
    completion_callback,
    request_lease,
    controller_succeeded is True and succeeded and not timed_out,
    settlement_callback,
  )


def _program_completion_fallback_result(
  operation,
  controller_succeeded,
  deadline,
  timed_out,
):
  while not operation.completed:
    if not timed_out and time.monotonic() >= deadline:
      logger.error("Virtual program motion did not finish before timeout")
      timed_out = True
    operation.wait(CONTROL_POLL_INTERVAL_SECONDS)
  if not timed_out and time.monotonic() >= deadline:
    logger.error("Virtual program motion did not finish before timeout")
    timed_out = True
  succeeded, error = operation.result()
  if not succeeded:
    logger.error("Virtual program motion failed: %s", error)
  return controller_succeeded is True and succeeded and not timed_out


def _run_program_completion_fallback(
  completion_callback,
  request_lease,
  operation,
  controller_succeeded,
  deadline,
  timed_out,
  settlement_callback,
):
  succeeded = _program_completion_fallback_result(
    operation,
    controller_succeeded,
    deadline,
    timed_out,
  )
  virtual_motion_event_queue.put((
    "program-completion",
    (
      completion_callback,
      request_lease,
      succeeded,
      settlement_callback,
    ),
  ))


def _start_program_completion_fallback(
  completion_callback,
  request_lease,
  operation,
  controller_succeeded,
  deadline,
  timed_out,
  settlement_callback,
):
  try:
    thread = threading.Thread(
      target=_run_program_completion_fallback,
      args=(
        completion_callback,
        request_lease,
        operation,
        controller_succeeded,
        deadline,
        timed_out,
        settlement_callback,
      ),
      daemon=True,
    )
    thread.start()
  except Exception:
    logger.exception("Unable to retain virtual program completion ownership")
    return False
  return True


def _wait_for_virtual_motion_operation(operation, timeout, deadline=None):
  if not isinstance(operation, VirtualMotionOperation):
    logger.error("Virtual program motion did not publish an operation result")
    return False
  if deadline is None:
    deadline = time.monotonic() + timeout
  timed_out = False
  while not operation.completed:
    remaining = deadline - time.monotonic()
    if not timed_out and remaining <= 0:
      logger.error("Virtual program motion did not finish before timeout")
      timed_out = True
    operation.wait(CONTROL_POLL_INTERVAL_SECONDS)
  if not timed_out and time.monotonic() >= deadline:
    logger.error("Virtual program motion did not finish before timeout")
    timed_out = True
  succeeded, error = operation.result()
  if not succeeded:
    logger.error("Virtual program motion failed: %s", error)
  return succeeded and not timed_out


def _complete_step_reverse_motion(sel_row, succeeded):
  _finish_execute_row()
  if succeeded:
    _finish_step_reverse_selection(sel_row)


def _start_legacy_motion(
  command,
  motion_name,
  completion_callback=None,
  request_lease=None,
  write_started_event=None,
):
  if not motion_request_registry.owns(request_lease):
    raise RuntimeError("legacy motion requires matching request ownership")
  if start_send_serial_thread(
    command,
    completion_callback=completion_callback,
    write_started_event=write_started_event,
  ):
    return True
  message = f"{motion_name} not started; controller transport is unavailable"
  logger.warning(message)
  almStatusLab.config(text=message, style="Warn.TLabel")
  almStatusLab2.config(text=message, style="Warn.TLabel")
  return False


def _capture_program_motion_pose():
  if RUN['offlineMode']:
    controller_positions = None
    virtual_pose = tuple(
      finite_number(value, f"saved virtual joint {axis}")
      for axis, value in enumerate(RUN['VR_angles'], start=1)
    )
    if len(virtual_pose) != 6:
      raise MotionInputError("saved virtual pose must contain six joints")
  else:
    controller_positions = tuple(_current_joint_positions())
    if len(controller_positions) != 9:
      raise MotionInputError("saved controller pose must contain nine axes")
    virtual_pose = controller_positions[:6]
  return ProgramMotionPoseSnapshot(controller_positions, virtual_pose)


def _apply_program_motion_pose(controller_positions, virtual_pose):
  try:
    if controller_positions is None:
      normalized_controller_positions = None
    else:
      normalized_controller_positions = tuple(
        finite_number(value, f"confirmed controller axis {axis}")
        for axis, value in enumerate(controller_positions, start=1)
      )
      if len(normalized_controller_positions) != 9:
        raise MotionInputError("confirmed controller pose must contain nine axes")
    normalized_virtual_pose = tuple(
      finite_number(value, f"confirmed virtual joint {axis}")
      for axis, value in enumerate(virtual_pose, start=1)
    )
    if len(normalized_virtual_pose) != 6:
      raise MotionInputError("confirmed virtual pose must contain six joints")

    if normalized_controller_positions is not None and (
      joint_motion_dispatcher.synchronize(normalized_controller_positions)
      is not True
    ):
      raise MotionQueueFault(
        "joint dispatcher rejected program-position synchronization"
      )
    if normalized_controller_positions is None:
      if refresh_gui_from_joint_angles(normalized_virtual_pose) is not True:
        raise MotionQueueFault("virtual GUI rejected the program position")
    elif _try_set_virtual_joint_target(normalized_virtual_pose) is not True:
      raise MotionQueueFault("virtual model rejected the program position")
  except (KeyError, TypeError, ValueError, MotionInputError, MotionQueueFault) as exc:
    logger.error("Unable to apply a confirmed program-motion pose: %s", exc)
    return False
  return True


def _reconcile_program_motion_pose(
  snapshot,
  controller_position,
  controller_write_started,
  motion_succeeded,
):
  if not isinstance(snapshot, ProgramMotionPoseSnapshot):
    raise TypeError("program motion pose snapshot has an invalid type")
  if controller_position is not None and not isinstance(
    controller_position,
    PositionResponse,
  ):
    raise TypeError("program motion controller position has an invalid type")
  if not isinstance(motion_succeeded, bool):
    raise TypeError("program motion settlement result must be boolean")

  online_motion = snapshot.controller_positions is not None
  if online_motion:
    if not all(
      callable(getattr(controller_write_started, method, None))
      for method in ("set", "is_set")
    ):
      raise TypeError("program motion write boundary has an invalid event")
  elif controller_write_started is not None or controller_position is not None:
    raise RuntimeError("offline program motion received physical controller state")

  if not online_motion:
    virtual_pose = (
      tuple(RUN['VR_angles'])
      if motion_succeeded
      else snapshot.virtual_pose
    )
    pose_confirmed = _apply_program_motion_pose(None, virtual_pose)
  elif controller_position is not None:
    pose_confirmed = _apply_program_motion_pose(
      controller_position.joints + controller_position.external,
      controller_position.joints,
    )
  elif not controller_write_started.is_set():
    pose_confirmed = _apply_program_motion_pose(
      snapshot.controller_positions,
      snapshot.virtual_pose,
    )
  else:
    pose_confirmed = False

  if online_motion:
    if (
      pose_confirmed
      and controller_position_resynchronization_required.is_set()
    ):
      pose_confirmed = False
    if not pose_confirmed:
      message = (
        "Program motion pose is uncertain; controller position "
        "resynchronization is required"
      )
      _invalidate_joint_motion_state(message)
      logger.error(message)
      almStatusLab.config(text=message, style="Alarm.TLabel")
      almStatusLab2.config(text=message, style="Alarm.TLabel")
  elif not pose_confirmed:
    message = "Offline program motion could not restore the confirmed virtual pose"
    logger.error(message)
    almStatusLab.config(text=message, style="Alarm.TLabel")
    almStatusLab2.config(text=message, style="Alarm.TLabel")

  return motion_succeeded and pose_confirmed


def _dispatch_program_controller_sequence(commands, completion_callback):
  if completion_callback is not None and not callable(completion_callback):
    raise TypeError("program motion completion callback must be callable")
  if RUN['offlineMode']:
    message = "Program motion requires virtual playback while offline"
    logger.error(message)
    almStatusLab.config(text=message, style="Alarm.TLabel")
    almStatusLab2.config(text=message, style="Alarm.TLabel")
    return ROW_EXECUTION_REJECTED
  if isinstance(commands, (str, bytes)):
    commands = (commands,)
  else:
    try:
      commands = tuple(commands)
    except TypeError as exc:
      raise MotionInputError("program motion commands must be a sequence") from exc
  if not commands or any(not isinstance(command, str) for command in commands):
    raise MotionInputError(
      "program motion commands must contain controller command text"
    )

  try:
    commands = tuple(
      _canonicalize_main_serial_command(command)
      for command in commands
    )
    completion_timeout = sum(
      _controller_response_timeout(command)
      + SERIAL_EVENT_APPLICATION_MARGIN_SECONDS
      for command in commands
    )
  except Exception as exc:
    message = f"Program motion sequence rejected: {exc}"
    logger.error(message)
    almStatusLab.config(text=message, style="Alarm.TLabel")
    almStatusLab2.config(text=message, style="Alarm.TLabel")
    return ROW_EXECUTION_REJECTED

  request_lease = _acquire_motion_request("Program motion")
  if request_lease is None:
    message = _motion_request_rejection_message(
      "Program motion not started; another motion request is active"
    )
    logger.warning(message)
    almStatusLab.config(text=message, style="Warn.TLabel")
    almStatusLab2.config(text=message, style="Warn.TLabel")
    return ROW_EXECUTION_REJECTED

  try:
    pose_snapshot = _capture_program_motion_pose()
  except (KeyError, TypeError, ValueError, MotionInputError) as exc:
    message = f"Program motion rejected because the confirmed pose is invalid: {exc}"
    logger.error(message)
    almStatusLab.config(text=message, style="Alarm.TLabel")
    almStatusLab2.config(text=message, style="Alarm.TLabel")
    _finish_motion_request(request_lease)
    return ROW_EXECUTION_REJECTED

  controller_write_started = threading.Event()
  controller_result = {"position": None}
  completion_event = threading.Event()
  completion_result = {"succeeded": False}
  completion_lock = threading.Lock()
  completion_state = {"finished": False}

  def publish_completion(succeeded):
    completion_result["succeeded"] = succeeded is True
    completion_event.set()
    if completion_callback is not None:
      completion_callback(succeeded is True)

  def finalize_sequence(succeeded):
    with completion_lock:
      if completion_state["finished"]:
        logger.error("Program motion sequence emitted duplicate completion")
        return False
      completion_state["finished"] = True
    return _finish_settled_motion_request(
      publish_completion,
      request_lease,
      succeeded is True,
      lambda motion_succeeded: _reconcile_program_motion_pose(
        pose_snapshot,
        controller_result["position"],
        controller_write_started,
        motion_succeeded,
      ),
    )

  def start_command(index):
    controller_write_started.clear()

    def complete_command(controller_position):
      if controller_position is not None and not isinstance(
        controller_position,
        PositionResponse,
      ):
        logger.error("Program motion sequence returned an invalid controller result")
        if controller_write_started.is_set():
          controller_result["position"] = None
        finalize_sequence(False)
        return
      if controller_position is None:
        if controller_write_started.is_set():
          controller_result["position"] = None
        finalize_sequence(False)
        return
      controller_result["position"] = controller_position
      next_index = index + 1
      if next_index >= len(commands):
        finalize_sequence(True)
        return
      if not start_command(next_index):
        finalize_sequence(False)

    try:
      started = _start_legacy_motion(
        commands[index],
        "Program motion",
        completion_callback=complete_command,
        request_lease=request_lease,
        write_started_event=controller_write_started,
      )
    except Exception:
      logger.exception("Program motion controller worker failed during startup")
      started = False
    if not started and controller_write_started.is_set():
      controller_result["position"] = None
    return started

  if not start_command(0):
    finalize_sequence(False)
    return ROW_EXECUTION_REJECTED
  if completion_callback is not None:
    return ROW_EXECUTION_PENDING

  deadline_missed = not completion_event.wait(completion_timeout)
  if deadline_missed:
    logger.error("Controller program sequence missed the application deadline")
    completion_event.wait()
  return (
    ROW_EXECUTION_COMPLETE
    if completion_result["succeeded"] and not deadline_missed
    else ROW_EXECUTION_REJECTED
  )


def _dispatch_program_motion(
  command,
  virtual_dispatch,
  virtual_command,
  completion_callback,
):
  if virtual_dispatch is None:
    if virtual_command is not None:
      raise MotionInputError(
        "controller-only program motion cannot receive a virtual command"
      )
    return _dispatch_program_controller_sequence(
      (command,),
      completion_callback,
    )
  if not callable(virtual_dispatch) or not isinstance(virtual_command, str):
    raise MotionInputError("program virtual motion contract is invalid")
  try:
    command = _canonicalize_main_serial_command(command)
    physical_wrist_config = parse_motion_wrist_config(command, virtual=False)
    virtual_wrist_config = parse_motion_wrist_config(
      virtual_command,
      virtual=True,
    )
    if physical_wrist_config != virtual_wrist_config:
      raise MotionInputError(
        "physical and virtual program commands require the same wrist configuration"
      )
    virtual_completion_timeout = _virtual_completion_timeout(virtual_command)
    controller_completion_timeout = (
      None
      if RUN['offlineMode']
      else _controller_response_timeout(command)
      + SERIAL_EVENT_APPLICATION_MARGIN_SECONDS
    )
  except Exception as exc:
    message = f"Program motion timing rejected: {exc}"
    logger.error(message)
    almStatusLab.config(text=message, style="Alarm.TLabel")
    almStatusLab2.config(text=message, style="Alarm.TLabel")
    return ROW_EXECUTION_REJECTED

  request_lease = _acquire_motion_request("Program motion")
  if request_lease is None:
    message = _motion_request_rejection_message(
      "Program motion not started; another motion request is active"
    )
    logger.warning(message)
    almStatusLab.config(text=message, style="Warn.TLabel")
    almStatusLab2.config(text=message, style="Warn.TLabel")
    return ROW_EXECUTION_REJECTED

  try:
    pose_snapshot = _capture_program_motion_pose()
  except (KeyError, TypeError, ValueError, MotionInputError) as exc:
    message = f"Program motion rejected because the confirmed pose is invalid: {exc}"
    logger.error(message)
    almStatusLab.config(text=message, style="Alarm.TLabel")
    almStatusLab2.config(text=message, style="Alarm.TLabel")
    _finish_motion_request(request_lease)
    return ROW_EXECUTION_REJECTED

  controller_write_started = (
    None
    if RUN['offlineMode']
    else threading.Event()
  )
  controller_result = {"position": None}

  def settle_program_pose(succeeded):
    return _reconcile_program_motion_pose(
      pose_snapshot,
      controller_result["position"],
      controller_write_started,
      succeeded,
    )

  try:
    virtual_operation = virtual_dispatch(virtual_command)
  except Exception:
    logger.exception("Virtual program preview failed during startup")
    virtual_operation = False
  if not isinstance(virtual_operation, VirtualMotionOperation):
    virtual_operation = None
  if virtual_operation is None:
    message = "Program motion stopped because virtual preview did not start"
    logger.error(message)
    almStatusLab.config(text=message, style="Alarm.TLabel")
    almStatusLab2.config(text=message, style="Alarm.TLabel")
    _finish_settled_motion_request(
      None,
      request_lease,
      False,
      settle_program_pose,
    )
    return ROW_EXECUTION_REJECTED
  virtual_completion_deadline = time.monotonic() + virtual_completion_timeout

  if virtual_operation.completed:
    virtual_succeeded, virtual_error = virtual_operation.result()
    if not virtual_succeeded:
      message = f"Program motion stopped after virtual preview failed: {virtual_error}"
      logger.error(message)
      almStatusLab.config(text=message, style="Alarm.TLabel")
      almStatusLab2.config(text=message, style="Alarm.TLabel")
      _finish_settled_motion_request(
        None,
        request_lease,
        False,
        settle_program_pose,
      )
      return ROW_EXECUTION_REJECTED

  def start_controller_motion(callback):
    try:
      return _start_legacy_motion(
        command,
        "Program motion",
        completion_callback=callback,
        request_lease=request_lease,
        write_started_event=controller_write_started,
      )
    except Exception as exc:
      message = f"Program motion controller worker failed during startup: {exc}"
      logger.exception(message)
      almStatusLab.config(text=message, style="Alarm.TLabel")
      almStatusLab2.config(text=message, style="Alarm.TLabel")
      return False

  if completion_callback is not None:
    if RUN['offlineMode']:
      _complete_program_motion_when_virtual_idle(
        completion_callback,
        request_lease,
        virtual_operation,
        virtual_completion_timeout,
        deadline=virtual_completion_deadline,
        settlement_callback=settle_program_pose,
      )
      return ROW_EXECUTION_PENDING

    def complete_controller_and_virtual(controller_position):
      if controller_position is not None and not isinstance(
        controller_position,
        PositionResponse,
      ):
        raise RuntimeError(
          "program motion completion returned an invalid controller result"
        )
      controller_result["position"] = controller_position
      _complete_program_motion_when_virtual_idle(
        completion_callback,
        request_lease,
        virtual_operation,
        virtual_completion_timeout,
        controller_succeeded=controller_position is not None,
        deadline=virtual_completion_deadline,
        settlement_callback=settle_program_pose,
      )

    if not start_controller_motion(complete_controller_and_virtual):
      _complete_program_motion_when_virtual_idle(
        completion_callback,
        request_lease,
        virtual_operation,
        virtual_completion_timeout,
        controller_succeeded=False,
        deadline=virtual_completion_deadline,
        settlement_callback=settle_program_pose,
      )
    return ROW_EXECUTION_PENDING

  try:
    controller_completion = None
    if not RUN['offlineMode']:
      controller_completion = threading.Event()

      def record_controller_completion(controller_position):
        if controller_position is not None and not isinstance(
          controller_position,
          PositionResponse,
        ):
          raise RuntimeError(
            "program motion completion returned an invalid controller result"
          )
        controller_result["position"] = controller_position
        controller_completion.set()

      if not start_controller_motion(record_controller_completion):
        _wait_for_virtual_motion_operation(
          virtual_operation,
          virtual_completion_timeout,
          deadline=virtual_completion_deadline,
        )
        settle_program_pose(False)
        return ROW_EXECUTION_REJECTED

    controller_succeeded = True
    if controller_completion is not None:
      controller_timed_out = not controller_completion.wait(
        controller_completion_timeout,
      )
      if controller_timed_out:
        logger.error("Controller program result missed the application deadline")
        controller_completion.wait()
      controller_succeeded = (
        not controller_timed_out
        and controller_result["position"] is not None
      )

    virtual_finished = _wait_for_virtual_motion_operation(
      virtual_operation,
      virtual_completion_timeout,
      deadline=virtual_completion_deadline,
    )

    motion_succeeded = controller_succeeded and virtual_finished
    pose_succeeded = settle_program_pose(motion_succeeded)

    if not pose_succeeded:
      if not controller_succeeded:
        message = "Program motion stopped after controller completion failed"
      elif not virtual_finished:
        message = "Program motion stopped after virtual preview failed or timed out"
      else:
        message = "Program motion stopped because pose reconciliation failed"
      logger.error(message)
      almStatusLab.config(text=message, style="Alarm.TLabel")
      almStatusLab2.config(text=message, style="Alarm.TLabel")
      return ROW_EXECUTION_REJECTED
    return ROW_EXECUTION_COMPLETE
  finally:
    _finish_motion_request(request_lease)


def _dispatch_program_command(
  command,
  virtual_dispatch,
  virtual_command,
  completion_callback,
):
  if not RUN['offlineMode']:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
  if RUN['vtk_running']:
    return _dispatch_program_motion(
      command,
      virtual_dispatch,
      virtual_command,
      completion_callback,
    )
  return _dispatch_program_motion(
    command,
    None,
    None,
    completion_callback,
  )


def _gcode_storage_filename(filename):
  if not isinstance(filename, str):
    raise MotionInputError("G-code filename must be text")
  filename = filename.strip()
  validate_controller_filename(filename, "G-code filename")
  storage_filename = f"{filename}.txt"
  return validate_controller_filename(storage_filename, "G-code filename")


def _gcode_playback_command(filename):
  return f"PGFn{_gcode_storage_filename(filename)}\n"


def _gcode_feed_rate_mm_per_second(value, imperial):
  if not isinstance(imperial, bool):
    raise MotionInputError("G-code unit mode must be boolean")
  feed_rate = finite_number(value, "G-code feed rate")
  if feed_rate <= 0:
    raise MotionInputError("G-code feed rate must be positive")
  millimeters_per_minute = feed_rate * (25.4 if imperial else 1.0)
  return controller_protocol_decimal(
    millimeters_per_minute / 60.0,
    "G-code feed rate in millimeters per second",
  )


def _dispatch_program_gcode(filename, completion_callback):
  controller_completion = None
  controller_result = None
  controller_callback = completion_callback

  try:
    _gcode_playback_command(filename)
    if completion_callback is None:
      controller_completion = threading.Event()
      controller_result = {"succeeded": False}

      def record_controller_completion(succeeded):
        controller_result["succeeded"] = succeeded is True
        controller_completion.set()

      controller_callback = record_controller_completion
  except Exception as exc:
    message = f"G-code playback rejected: {exc}"
    logger.error(message)
    almStatusLab.config(text=message, style="Alarm.TLabel")
    almStatusLab2.config(text=message, style="Alarm.TLabel")
    return ROW_EXECUTION_REJECTED

  if not GCplayProg(filename, completion_callback=controller_callback):
    message = "G-code playback not started; controller transport is unavailable"
    logger.warning(message)
    almStatusLab.config(text=message, style="Alarm.TLabel")
    almStatusLab2.config(text=message, style="Alarm.TLabel")
    return ROW_EXECUTION_REJECTED

  if completion_callback is not None:
    return ROW_EXECUTION_PENDING

  controller_succeeded = (
    controller_completion.wait()
    and controller_result["succeeded"]
  )
  if not controller_succeeded:
    message = "G-code playback stopped after controller completion failed"
    logger.error(message)
    almStatusLab.config(text=message, style="Alarm.TLabel")
    almStatusLab2.config(text=message, style="Alarm.TLabel")
    return ROW_EXECUTION_REJECTED
  return ROW_EXECUTION_COMPLETE

def runProg():
  with program_stop_state_lock:
    if RUN.get('programStopRequestId') is not None:
      _set_program_stop_status("pending")
      return False
    RUN['programStopStatusLatched'] = False
    RUN['programStopState'] = "completed"
    RUN['estopActive'] = False
    RUN['posOutreach'] = False

  def threadProg():
    last = tab1.progView.index('end')
    for row in range (0,last):
      tab1.progView.itemconfig(row, {'fg': "#FFFFFF"})

    try:
      curRow = tab1.progView.curselection()[0]
      if (curRow == 0):
        curRow=1
    except:
      curRow=1
      tab1.progView.selection_clear(0, END)
      tab1.progView.select_set(curRow)
    tab1.runTrue = 1
    while tab1.runTrue == 1:
      if (tab1.runTrue == 0):
        _set_program_stop_status(RUN['programStopState'])
      else:
        almStatusLab.config(text="PROGRAM RUNNING",  style="OK.TLabel")
        almStatusLab2.config(text="PROGRAM RUNNING",  style="OK.TLabel")
      RUN['rowinproc'] = 1
      try:
        selRow = tab1.progView.curselection()[0]
      except:
        if(tab1.lastProg == ""):
          selRow = 1
          progLoop = ("## START PROGRAM LOOP ##\r\n").encode('utf-8')
          try:
            index = tab1.progView.get(0, "end").index(progLoop)
            tab1.progView.selection_clear(0, END)
            tab1.progView.select_set(index)
          except:
            stopProg()
        else:
          lastRow = tab1.lastRow + 1
          lastProg = tab1.lastProg
          ProgEntryField.delete(0, 'end')
          ProgEntryField.insert(0,lastProg)
          callProg(lastProg)
          time.sleep(.4)
          tab1.progView.selection_clear(0, END)
          tab1.progView.select_set(lastRow)
          curRowEntryField.delete(0, 'end')
          curRowEntryField.insert(0,lastRow)
          tab1.lastProg = ""
      if(tab1.runTrue == 1):
        execution_state = executeRow()
        if execution_state == ROW_EXECUTION_REJECTED:
          tab1.runTrue = 0
          RUN['rowinproc'] = 0
          break

        while RUN['rowinproc'] == 1:
          time.sleep(.1)

        try:
          selRow = tab1.progView.curselection()[0]
          selRow += 1
          tab1.progView.selection_clear(0, END)
          tab1.progView.select_set(selRow)
          curRowEntryField.delete(0, 'end')
          curRowEntryField.insert(0,selRow)
        except:
          curRow=1
          tab1.progView.selection_clear(0, END)
          tab1.progView.select_set(curRow)
        time.sleep(.1)

    _set_program_stop_status(RUN['programStopState'])
  t = threading.Thread(target=threadProg)
  t.start()
  return True

def stepFwd():
    with program_stop_state_lock:
      RUN['programStopStatusLatched'] = False
      RUN['estopActive'] = False
      RUN['posOutreach'] = False
    def threadProg():
      almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
      almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel")
      try:
        selRow = tab1.progView.curselection()[0]
      except:
        if(tab1.lastProg == ""):
          selRow = 1
          progLoop = ("## START PROGRAM LOOP ##\r\n").encode('utf-8')
          try:
            index = tab1.progView.get(0, "end").index(progLoop)
            tab1.progView.selection_clear(0, END)
            tab1.progView.select_set(index)
          except:
            stopProg()
        else:
          lastRow = tab1.lastRow + 1
          lastProg = tab1.lastProg
          ProgEntryField.delete(0, 'end')
          ProgEntryField.insert(0,lastProg)
          callProg(lastProg)
          time.sleep(.4)
          tab1.progView.selection_clear(0, END)
          tab1.progView.select_set(lastRow)
          curRowEntryField.delete(0, 'end')
          curRowEntryField.insert(0,lastRow)
          tab1.lastProg = ""
      if executeRow() == ROW_EXECUTION_REJECTED:
        return
      try:
        last = tab1.progView.index('end')
        selRow = tab1.progView.curselection()[0]
        for row in range (0,selRow):
          tab1.progView.itemconfig(row, {'fg': "#1E90FF"})
        tab1.progView.itemconfig(selRow, {'fg': "#0561BD"})
        for row in range (selRow+1,last):
          tab1.progView.itemconfig(row, {'fg': "#9E9E9E"})
        try:
          selRow = tab1.progView.curselection()[0]
          selRow += 1
          tab1.progView.selection_clear(0, END)
          tab1.progView.select_set(selRow)
          curRowEntryField.delete(0, 'end')
          curRowEntryField.insert(0,selRow)
        except:
          curRow=1
          tab1.progView.selection_clear(0, END)
          tab1.progView.select_set(curRow)
        time.sleep(.1)
      except Exception:
        pass
    t = threading.Thread(target=threadProg)
    t.start()

def stepRev():
    with program_stop_state_lock:
      RUN['programStopStatusLatched'] = False
      RUN['estopActive'] = False
      RUN['posOutreach'] = False
    almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel") 
    try:
      selRow = tab1.progView.curselection()[0]
    except:
      selRow = 1
      tab1.progView.selection_clear(0, END)
      tab1.progView.select_set(selRow) 
    execution_state = executeRow(
      motion_complete=lambda succeeded: _complete_step_reverse_motion(
        selRow,
        succeeded,
      )
    )
    if execution_state == ROW_EXECUTION_COMPLETE:
      _finish_step_reverse_selection(selRow)


def _set_program_stop_status(auxiliary_state):
  if auxiliary_state not in ("pending", "completed", "failed"):
    raise ValueError("unknown auxiliary program-stop state")
  with program_stop_state_lock:
    RUN['programStopState'] = auxiliary_state
    RUN['programStopStatusLatched'] = True
    if RUN['estopActive']:
      message = "Estop Button was Pressed"
      style = "Alarm.TLabel"
    elif RUN['posOutreach']:
      message = "Position Out of Reach"
      style = "Alarm.TLabel"
    elif auxiliary_state == "pending":
      message = (
        "PROGRAM HALT REQUESTED; AUXILIARY STOP PENDING; "
        "ACTIVE MAIN MOTION NOT PREEMPTED"
      )
      style = "Warn.TLabel"
    elif auxiliary_state == "failed":
      message = (
        "PROGRAM SCHEDULING HALTED; AUXILIARY STOP FAILED; "
        "ACTIVE MAIN MOTION NOT PREEMPTED"
      )
      style = "Alarm.TLabel"
    else:
      message = (
        "PROGRAM SCHEDULING HALTED; ACTIVE MAIN MOTION NOT PREEMPTED"
      )
      style = "Alarm.TLabel"
    program_stop_status_event_queue.put((message, style))
  return True


def _apply_program_stop_status_events():
  applied = False
  while True:
    try:
      event = program_stop_status_event_queue.get_nowait()
    except Empty:
      break
    if (
      not isinstance(event, tuple)
      or len(event) != 2
      or not isinstance(event[0], str)
      or not event[0].strip()
      or event[0] != event[0].strip()
      or not isinstance(event[1], str)
      or not event[1]
    ):
      raise RuntimeError("program stop status queue emitted an invalid event")
    message, style = event
    almStatusLab.config(text=message, style=style)
    almStatusLab2.config(text=message, style=style)
    applied = True
  return applied


def stopProg():
  try:
    _begin_manual_auxiliary_stop("program stop requested")
  except Exception:
    logger.exception(
      "Unable to cancel queued manual auxiliary commands during program stop"
    )
  tab1.runTrue = 0
  with program_stop_state_lock:
    if RUN.get('programStopRequestId') is not None:
      _set_program_stop_status("pending")
      return

  def register_stop_request(request_id):
    if (
      isinstance(request_id, bool)
      or not isinstance(request_id, int)
      or request_id <= 0
    ):
      raise RuntimeError("auxiliary stop registration received an invalid ID")
    with program_stop_state_lock:
      RUN['programStopRequestId'] = request_id
      _set_program_stop_status("pending")

  try:
    auxiliary_state, request_id = _request_auxiliary_stop(
      register_stop_request
    )
  except Exception:
    logger.exception("Unable to dispatch the auxiliary program stop")
    with program_stop_state_lock:
      RUN['programStopRequestId'] = None
      manual_auxiliary_stop_barrier.clear()
      _set_program_stop_status("failed")
    return
  if auxiliary_state == AUXILIARY_STOP_NOT_REQUIRED:
    with program_stop_state_lock:
      RUN['programStopRequestId'] = None
      manual_auxiliary_stop_barrier.clear()
      _set_program_stop_status("completed")



def executeRow(motion_complete=None):
  RUN['progRunning'] = True
  try:
    selRow = tab1.progView.curselection()[0]
    tab1.progView.see(selRow+2)
  except Exception:
    pass

  try: 
    data = list(map(int, tab1.progView.curselection()))
    command=tab1.progView.get(data[0]).decode().strip()
    RUN['cmdType'] =command[:6]
    RUN['cmdTypeLong']=command[:11]
  except:
    RUN['cmdType'] = "Stop P"
    RUN['cmdTypeLong'] = "Stop P"

  if (RUN['cmdType'] == "Call P"):
    if (RUN['moveInProc'] == 1):
      RUN['moveInProc'] = 2
    tab1.lastRow = tab1.progView.curselection()[0]
    tab1.lastProg = ProgEntryField.get()
    programIndex = command.find("Program -")
    progName = str(command[programIndex+10:])
    ProgEntryField.delete(0, 'end')
    ProgEntryField.insert(0,progName)
    callProg(progName)
    time.sleep(.4) 
    index = 0
    tab1.progView.selection_clear(0, END)
    tab1.progView.select_set(index)
    


  if (RUN['cmdType'] == "Run Gc"):
    if RUN['offlineMode']:
      almStatusLab.config(text="Gcode not supported in offline programming mode", style="Alarm.TLabel")
      _finish_execute_row()
      return ROW_EXECUTION_REJECTED
    if (RUN['moveInProc'] == 1):
      RUN['moveInProc'] = 2
    programIndex = command.find("Program -")
    filename = str(command[programIndex+10:])
    manEntryField.delete(0, 'end')
    manEntryField.insert(0,filename)
    motion_state = _dispatch_program_gcode(filename, motion_complete)
    if motion_state == ROW_EXECUTION_REJECTED:
      _finish_execute_row()
      return motion_state
    if motion_state == ROW_EXECUTION_PENDING:
      return motion_state

  if (RUN['cmdType'] == "Stop P"):
    if (RUN['moveInProc'] == 1):
      RUN['moveInProc'] = 2
    stopProg()


  if (RUN['cmdType'] == "Test L"):
    if RUN['offlineMode']:
      almStatusLab.config(text="Test limit switches not supported in offline programming mode", style="Alarm.TLabel")
      _finish_execute_row()
      return ROW_EXECUTION_REJECTED
    if (RUN['moveInProc'] == 1):
      RUN['moveInProc'] = 2
    command = "TL\n" 
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0,command)
    execution_state, response = _execute_row_main_response(command)
    if execution_state != ROW_EXECUTION_COMPLETE:
      return execution_state
    manEntryField.delete(0, 'end')
    manEntryField.insert(0,response)

  if (RUN['cmdType'] == "Test G"):
    if RUN['offlineMode']:
      almStatusLab.config(text="Test limit switches not supported in offline programming mode", style="Alarm.TLabel")
      _finish_execute_row()
      return ROW_EXECUTION_REJECTED
    if (RUN['moveInProc'] == 1):
      RUN['moveInProc'] = 2
    command = "TG\n" 
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0,command)
    execution_state, response = _execute_row_auxiliary_command(
      command,
      read_line=True,
      response_delay=.05,
    )
    if execution_state != ROW_EXECUTION_COMPLETE:
      return execution_state
    manEntryField.delete(0, 'end')
    manEntryField.insert(0,response)

  if (RUN['cmdType'] == "Set En"):
    if RUN['offlineMode']:
      almStatusLab.config(text="Encoder testing not supported in offline programming mode", style="Alarm.TLabel")
      _finish_execute_row()
      return ROW_EXECUTION_REJECTED
    if (RUN['moveInProc'] == 1):
      RUN['moveInProc'] = 2
    command = "SE\n" 
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0,command)
    execution_state, _ = _execute_row_main_response(
      command,
      read_line=False,
      expected_response=b"Done",
    )
    if execution_state != ROW_EXECUTION_COMPLETE:
      return execution_state

  if (RUN['cmdType'] == "Read E"):
    if RUN['offlineMode']:
      almStatusLab.config(text="Read Encoders not supported in offline programming mode", style="Alarm.TLabel")
      _finish_execute_row()
      return ROW_EXECUTION_REJECTED
    if (RUN['moveInProc'] == 1):
      RUN['moveInProc'] = 2
    command = "RE\n" 
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0,command)
    execution_state, response = _execute_row_main_response(command)
    if execution_state != ROW_EXECUTION_COMPLETE:
      return execution_state
    manEntryField.delete(0, 'end')
    manEntryField.insert(0,response)

  if (RUN['cmdType'] == "Servo "):
    if RUN['offlineMode']:
      almStatusLab.config(text="Servo control not supported in offline programming mode", style="Alarm.TLabel")
      _finish_execute_row()
      return ROW_EXECUTION_REJECTED
    if (RUN['moveInProc'] == 1):
      RUN['moveInProc'] = 2
    servoIndex = command.find("number ")
    posIndex = command.find("position: ")
    servoNum = str(command[servoIndex+7:posIndex-4])
    servoPos = str(command[posIndex+10:])
    command = "SV"+servoNum+"P"+servoPos+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0,command)
    execution_state, _ = _execute_row_auxiliary_command(
      command,
      expected_response=b"Servo Done",
    )
    if execution_state != ROW_EXECUTION_COMPLETE:
      return execution_state

  if (RUN['cmdType'] == "If Inp"):
    if RUN['offlineMode']:
      almStatusLab.config(text="IO not supported in offline programming mode", style="Alarm.TLabel")
      _finish_execute_row()
      return ROW_EXECUTION_REJECTED
    if (RUN['moveInProc'] == 1):
      RUN['moveInProc'] = 2
    inputIndex = command.find("# ")
    valIndex = command.find("= ")
    actionIndex = command.find(": ")
    inputNum = str(command[inputIndex+2:valIndex])
    valNum = int(command[valIndex+2:actionIndex-1])
    action = str(command[actionIndex+2:actionIndex+6])
    cmd = "JFX"+inputNum+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0,cmd)   
    execution_state, response = _execute_row_auxiliary_command(
      cmd,
      read_line=True,
      accepted_responses=("T", "F"),
    )
    if execution_state != ROW_EXECUTION_COMPLETE:
      return execution_state
    if (response == "T"):
      querry = 1
    elif(response == "F"):
      querry = 0
    if(querry == valNum):
      if(action == "Call"):
        tab1.lastRow = tab1.progView.curselection()[0]
        tab1.lastProg = ProgEntryField.get()
        progIndex = command.find("Prog")
        progName = str(command[progIndex+5:]) + ".ar4" 
        callProg(progName)
        time.sleep(.4) 
        index = 0  
        tab1.progView.selection_clear(0, END)
        tab1.progView.select_set(index) 
      elif(action == "Jump"):
        tabIndex = command.find("Tab")
        tabNum = str(command[tabIndex+4:])
        tabNum = ("Tab Number " + tabNum + "\r\n").encode('utf-8')
        index = tab1.progView.get(0, "end").index(tabNum)
        index = index-1
        tab1.progView.selection_clear(0, END)
        tab1.progView.select_set(index)
      elif(action == "Stop"):
        stopProg()
         


  if (RUN['cmdType'] == "Read C"):
    if RUN['offlineMode']:
      almStatusLab.config(text="IO not supported in offline programming mode", style="Alarm.TLabel")
      _finish_execute_row()
      return ROW_EXECUTION_REJECTED
    comIndex = command.find("# ")
    charIndex = command.find("Char: ")
    actionIndex = command.find(": ")
    comNum = str(command[comIndex+2:charIndex-1])
    charNum = int(command[charIndex+6:])
    try:
      # global RUN['ser3']    
      port = "COM" + comNum   
      baud = 9600    
      RUN['ser3'] = serial.Serial(port,baud,timeout=10)
    except:
      #Curtime = datetime.now().strftime("%B %d %Y - %I:%M%p")
      #tab8.ElogView.insert(END, Curtime+" - UNABLE TO ESTABLISH COMMUNICATIONS WITH SERIAL DEVICE")
      logger.error("UNABLE TO ESTABLISH COMMUNICATIONS WITH SERIAL DEVICE")
      value=tab8.ElogView.get(0,END)
      pickle.dump(value,open("ErrorLog","wb"))
    RUN['ser3'].flushInput()
    response = str(RUN['ser3'].read(charNum).strip(),'utf-8')    
    com3outPortEntryField.delete(0, 'end')
    com3outPortEntryField.insert(0,response)
    manEntryField.delete(0, 'end')
    manEntryField.insert(0,response)


  
  if (RUN['cmdType'] == "If Reg"):
    if (RUN['moveInProc'] == 1):
      RUN['moveInProc'] = 2
    inputIndex = command.find("# ")
    valIndex = command.find("= ")
    actionIndex = command.find(": ")
    inputNum = str(command[inputIndex+2:valIndex-1])
    valNum = int(command[valIndex+2:actionIndex-1])
    action = str(command[actionIndex+2:actionIndex+6])
    regEntry = "R"+inputNum+"EntryField"
    curRegVal = eval(regEntry).get()
    if (int(curRegVal) == valNum):
      if(action == "Call"):
        tab1.lastRow = tab1.progView.curselection()[0]
        tab1.lastProg = ProgEntryField.get()
        progIndex = command.find("Prog")
        progName = str(command[progIndex+5:]) + ".ar4" 
        callProg(progName)
        time.sleep(.4) 
        index = 0  
        tab1.progView.selection_clear(0, END)
        tab1.progView.select_set(index) 
      elif(action == "Jump"):
        tabIndex = command.find("Tab")
        tabNum = str(command[tabIndex+4:])
        tabNum = ("Tab Number " + tabNum + "\r\n").encode('utf-8')
        index = tab1.progView.get(0, "end").index(tabNum)
        index = index-1
        tab1.progView.selection_clear(0, END)
        tab1.progView.select_set(index)
      elif(action == "Stop"):
        stopProg()  

  if (RUN['cmdType'] == "If COM"):
    if RUN['offlineMode']:
      almStatusLab.config(text="IO not supported in offline programming mode", style="Alarm.TLabel")
      _finish_execute_row()
      return ROW_EXECUTION_REJECTED
    if (RUN['moveInProc'] == 1):
      RUN['moveInProc'] = 2
    inputIndex = command.find("# ")
    valIndex = command.find("= ")
    actionIndex = command.find(": ")
    inputNum = str(command[inputIndex+2:valIndex-1])
    valNum = str(command[valIndex+2:actionIndex-1])
    action = str(command[actionIndex+2:actionIndex+6])
    curCOMVal = com3outPortEntryField.get()
    if (curCOMVal == valNum):
      if(action == "Call"):
        tab1.lastRow = tab1.progView.curselection()[0]
        tab1.lastProg = ProgEntryField.get()
        progIndex = command.find("Prog")
        progName = str(command[actionIndex+12:]) + ".ar4" 
        callProg(progName)
        time.sleep(.4) 
        index = 0  
        tab1.progView.selection_clear(0, END)
        tab1.progView.select_set(index) 
      elif(action == "Jump"):
        tabIndex = command.find("Tab")
        tabNum = str(command[tabIndex+4:])
        tabNum = ("Tab Number " + tabNum + "\r\n").encode('utf-8')
        index = tab1.progView.get(0, "end").index(tabNum)
        index = index-1
        tab1.progView.selection_clear(0, END)
        tab1.progView.select_set(index)
      elif(action == "Stop"):
        stopProg()           

  if (RUN['cmdType'] == "If MBc"):
    if RUN['offlineMode']:
      almStatusLab.config(text="IO not supported in offline programming mode", style="Alarm.TLabel")
      _finish_execute_row()
      return ROW_EXECUTION_REJECTED
    if (RUN['moveInProc'] == 1):
      RUN['moveInProc'] = 2
    inputIndex = command.find("# ")
    valIndex = command.find("= ")
    actionIndex = command.find(": ")
    slavestartIndex = command.find("(")
    slaveendIndex = command.find(")")
    inputNum = str(command[inputIndex+2:valIndex-1])
    valNum = str(command[valIndex+2:actionIndex-1])
    action = str(command[actionIndex+2:actionIndex+6])
    slaveID = str(command[slavestartIndex+1:slaveendIndex])
    opVal = "1"
    subcommand = "BB"+"A"+slaveID+"B"+inputNum+"C"+opVal+"\n"
    execution_state, response = _execute_row_main_response(
      subcommand,
      response_parser=parse_controller_modbus_response,
    )
    if execution_state != ROW_EXECUTION_COMPLETE:
      return execution_state
    if (response == valNum):
      if(action == "Call"):
        tab1.lastRow = tab1.progView.curselection()[0]
        tab1.lastProg = ProgEntryField.get()
        progIndex = command.find("Prog")
        progName = str(command[actionIndex+12:]) + ".ar4" 
        callProg(progName)
        time.sleep(.4) 
        index = 0  
        tab1.progView.selection_clear(0, END)
        tab1.progView.select_set(index) 
      elif(action == "Jump"):
        tabIndex = command.find("Tab")
        tabNum = str(command[tabIndex+4:])
        tabNum = ("Tab Number " + tabNum + "\r\n").encode('utf-8')
        index = tab1.progView.get(0, "end").index(tabNum)
        index = index-1
        tab1.progView.selection_clear(0, END)
        tab1.progView.select_set(index)
      elif(action == "Stop"):
        stopProg()   

  if (RUN['cmdType'] == "If MBi"):
    if RUN['offlineMode']:
      almStatusLab.config(text="IO not supported in offline programming mode", style="Alarm.TLabel")
      _finish_execute_row()
      return ROW_EXECUTION_REJECTED
    if (RUN['moveInProc'] == 1):
      RUN['moveInProc'] = 2
    inputIndex = command.find("# ")
    valIndex = command.find("= ")
    actionIndex = command.find(": ")
    slavestartIndex = command.find("(")
    slaveendIndex = command.find(")")
    inputNum = str(command[inputIndex+2:valIndex-1])
    valNum = str(command[valIndex+2:actionIndex-1])
    action = str(command[actionIndex+2:actionIndex+6])
    slaveID = str(command[slavestartIndex+1:slaveendIndex])
    opVal = "1"
    subcommand = "BC"+"A"+slaveID+"B"+inputNum+"C"+opVal+"\n"
    execution_state, response = _execute_row_main_response(
      subcommand,
      response_parser=parse_controller_modbus_response,
    )
    if execution_state != ROW_EXECUTION_COMPLETE:
      return execution_state
    if (response == valNum):
      if(action == "Call"):
        tab1.lastRow = tab1.progView.curselection()[0]
        tab1.lastProg = ProgEntryField.get()
        progIndex = command.find("Prog")
        progName = str(command[actionIndex+12:]) + ".ar4" 
        callProg(progName)
        time.sleep(.4) 
        index = 0  
        tab1.progView.selection_clear(0, END)
        tab1.progView.select_set(index) 
      elif(action == "Jump"):
        tabIndex = command.find("Tab")
        tabNum = str(command[tabIndex+4:])
        tabNum = ("Tab Number " + tabNum + "\r\n").encode('utf-8')
        index = tab1.progView.get(0, "end").index(tabNum)
        index = index-1
        tab1.progView.selection_clear(0, END)
        tab1.progView.select_set(index)
      elif(action == "Stop"):
        stopProg()

  if (RUN['cmdType'] == "If MBh"):
    if RUN['offlineMode']:
      almStatusLab.config(text="IO not supported in offline programming mode", style="Alarm.TLabel")
      _finish_execute_row()
      return ROW_EXECUTION_REJECTED
    if (RUN['moveInProc'] == 1):
      RUN['moveInProc'] = 2
    inputIndex = command.find("# ")
    valIndex = command.find("= ")
    actionIndex = command.find(": ")
    slavestartIndex = command.find("SlaveID (")
    regNumstartIndex = command.find("Num Reg's (")
    inputNum = str(command[inputIndex+2:valIndex-1])
    valNum = str(command[valIndex+2:actionIndex-1])
    action = str(command[actionIndex+2:actionIndex+6])
    slaveID = str(command[slavestartIndex+9:regNumstartIndex-2])
    opVal = str(command[regNumstartIndex+11:inputIndex-8])
    subcommand = "BH"+"A"+slaveID+"B"+inputNum+"C"+opVal+"\n"
    execution_state, response = _execute_row_main_response(
      subcommand,
      response_parser=parse_controller_modbus_response,
    )
    if execution_state != ROW_EXECUTION_COMPLETE:
      return execution_state
    if (response == valNum):
      if(action == "Call"):
        tab1.lastRow = tab1.progView.curselection()[0]
        tab1.lastProg = ProgEntryField.get()
        progIndex = command.find("Prog")
        progName = str(command[actionIndex+12:]) + ".ar4" 
        callProg(progName)
        time.sleep(.4) 
        index = 0  
        tab1.progView.selection_clear(0, END)
        tab1.progView.select_set(index) 
      elif(action == "Jump"):
        tabIndex = command.find("Tab")
        tabNum = str(command[tabIndex+4:])
        tabNum = ("Tab Number " + tabNum + "\r\n").encode('utf-8')
        index = tab1.progView.get(0, "end").index(tabNum)
        index = index-1
        tab1.progView.selection_clear(0, END)
        tab1.progView.select_set(index)
      elif(action == "Stop"):
        stopProg()  

  if (RUN['cmdType'] == "If MBI"):
    if RUN['offlineMode']:
      almStatusLab.config(text="IO not supported in offline programming mode", style="Alarm.TLabel")
      _finish_execute_row()
      return ROW_EXECUTION_REJECTED
    if (RUN['moveInProc'] == 1):
      RUN['moveInProc'] = 2
    inputIndex = command.find("# ")
    valIndex = command.find("= ")
    actionIndex = command.find(": ")
    slavestartIndex = command.find("SlaveID (")
    regNumstartIndex = command.find("Num Reg's (")
    inputNum = str(command[inputIndex+2:valIndex-1])
    valNum = str(command[valIndex+2:actionIndex-1])
    action = str(command[actionIndex+2:actionIndex+6])
    slaveID = str(command[slavestartIndex+9:regNumstartIndex-2])
    opVal = str(command[regNumstartIndex+11:inputIndex-14])
    subcommand = "BD"+"A"+slaveID+"B"+inputNum+"C"+opVal+"\n"
    execution_state, response = _execute_row_main_response(
      subcommand,
      response_parser=parse_controller_modbus_response,
    )
    if execution_state != ROW_EXECUTION_COMPLETE:
      return execution_state
    if (response == valNum):
      if(action == "Call"):
        tab1.lastRow = tab1.progView.curselection()[0]
        tab1.lastProg = ProgEntryField.get()
        progIndex = command.find("Prog")
        progName = str(command[actionIndex+12:]) + ".ar4" 
        callProg(progName)
        time.sleep(.4) 
        index = 0  
        tab1.progView.selection_clear(0, END)
        tab1.progView.select_set(index) 
      elif(action == "Jump"):
        tabIndex = command.find("Tab")
        tabNum = str(command[tabIndex+4:])
        tabNum = ("Tab Number " + tabNum + "\r\n").encode('utf-8')
        index = tab1.progView.get(0, "end").index(tabNum)
        index = index-1
        tab1.progView.selection_clear(0, END)
        tab1.progView.select_set(index)
      elif(action == "Stop"):
        stopProg()

  if (RUN['cmdTypeLong'] == "Wait 5v Inp"):
    if RUN['offlineMode']:
      almStatusLab.config(text="IO not supported in offline programming mode", style="Alarm.TLabel")
      _finish_execute_row()
      return ROW_EXECUTION_REJECTED
    if (RUN['moveInProc'] == 1):
      RUN['moveInProc'] = 2
    inputIndex = command.find("# ")
    valIndex = command.find("= ")
    timeoutIndex = command.find("Timeout =")
    inputNum = str(command[inputIndex+2:valIndex-1])
    valNum = str(command[valIndex+2:timeoutIndex-3])
    try:
      timeout = _auxiliary_wait_timeout_seconds(command[timeoutIndex+10:])
    except MotionInputError as exc:
      message = f"Wait command rejected: {exc}"
      logger.error(message)
      almStatusLab.config(text=message, style="Alarm.TLabel")
      almStatusLab2.config(text=message, style="Alarm.TLabel")
      _finish_execute_row()
      return ROW_EXECUTION_REJECTED
    command = "WI"+"A"+inputNum+"B"+valNum+"C"+str(timeout)+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0,command)
    execution_state, _ = _execute_row_auxiliary_command(
      command,
      read_line=True,
      control_injectable=True,
      response_timeout=timeout + SERIAL_AUXILIARY_WAIT_MARGIN_SECONDS,
      accepted_responses=AUXILIARY_WAIT_TERMINAL_RESPONSES,
    )
    if execution_state != ROW_EXECUTION_COMPLETE:
      return execution_state
 

  if (RUN['cmdTypeLong'] == "Wait MBcoil"):
    if RUN['offlineMode']:
      almStatusLab.config(text="IO not supported in offline programming mode", style="Alarm.TLabel")
      _finish_execute_row()
      return ROW_EXECUTION_REJECTED
    if (RUN['moveInProc'] == 1):
      RUN['moveInProc'] = 2
    slavestartIndex = command.find("SlaveID (")  
    inputIndex = command.find("# ")
    valIndex = command.find("= ")
    timeoutIndex = command.find("Timeout =")
    slaveID = str(command[slavestartIndex+9:inputIndex-9])
    inputNum = str(command[inputIndex+2:valIndex-1])
    valNum = str(command[valIndex+2:timeoutIndex-3])
    try:
      timeout = _main_modbus_wait_timeout_seconds(command[timeoutIndex+10:])
    except MotionInputError as exc:
      message = f"Controller Modbus wait rejected: {exc}"
      logger.error(message)
      almStatusLab.config(text=message, style="Alarm.TLabel")
      almStatusLab2.config(text=message, style="Alarm.TLabel")
      _finish_execute_row()
      return ROW_EXECUTION_REJECTED
    command = "WJ"+"A"+slaveID+"B"+inputNum+"C"+valNum+"D"+str(timeout)+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0,command)
    execution_state, _ = _execute_row_main_response(
      command,
      response_parser=parse_controller_modbus_response,
      response_timeout=timeout + SERIAL_RESPONSE_MARGIN_SECONDS,
    )
    if execution_state != ROW_EXECUTION_COMPLETE:
      return execution_state

  if (RUN['cmdTypeLong'] == "Wait MBinpu"):
    if RUN['offlineMode']:
      almStatusLab.config(text="IO not supported in offline programming mode", style="Alarm.TLabel")
      _finish_execute_row()
      return ROW_EXECUTION_REJECTED
    if (RUN['moveInProc'] == 1):
      RUN['moveInProc'] = 2
    slavestartIndex = command.find("SlaveID (")  
    inputIndex = command.find("# ")
    valIndex = command.find("= ")
    timeoutIndex = command.find("Timeout =")
    slaveID = str(command[slavestartIndex+9:inputIndex-9])
    inputNum = str(command[inputIndex+2:valIndex-1])
    valNum = str(command[valIndex+2:timeoutIndex-3])
    try:
      timeout = _main_modbus_wait_timeout_seconds(command[timeoutIndex+10:])
    except MotionInputError as exc:
      message = f"Controller Modbus wait rejected: {exc}"
      logger.error(message)
      almStatusLab.config(text=message, style="Alarm.TLabel")
      almStatusLab2.config(text=message, style="Alarm.TLabel")
      _finish_execute_row()
      return ROW_EXECUTION_REJECTED
    command = "WK"+"A"+slaveID+"B"+inputNum+"C"+valNum+"D"+str(timeout)+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0,command)
    execution_state, _ = _execute_row_main_response(
      command,
      response_parser=parse_controller_modbus_response,
      response_timeout=timeout + SERIAL_RESPONSE_MARGIN_SECONDS,
    )
    if execution_state != ROW_EXECUTION_COMPLETE:
      return execution_state
   
              

  ''' 
    if (RUN['moveInProc'] == 1):
        RUN['moveInProc'] = 2
    tabIndex = command.find("Tab-")
    tabNum = ("Tab Number " + str(command[tabIndex+4:]) + "\r\n").encode('utf-8')
    index = tab1.progView.get(0, "end").index(tabNum)
    tab1.progView.selection_clear(0, END)
    tab1.progView.select_set(index) 
  ''' 
  
  if RUN['cmdType'] == "Jump T":
      if (RUN['moveInProc'] == 1):
        RUN['moveInProc'] = 2

      tabIndex = command.find("Tab-")
      if tabIndex == -1:
          print("[Jump T] Malformed command, missing 'Tab-':", repr(command))
          _finish_execute_row()
          return ROW_EXECUTION_REJECTED

      # keep your original tabNum (bytes with CRLF)
      tabNum = ("Tab Number " + str(command[tabIndex+4:]) + "\r\n").encode('utf-8')

      def _norm(x):
          # bytes -> str; strip CR/LF and outer spaces; lower for case-insensitive match
          if isinstance(x, bytes):
              try:
                  x = x.decode("utf-8", "replace")
              except Exception:
                  x = str(x)
          return str(x).replace("\r", "").replace("\n", "").strip().lower()

      target_norm = _norm(tabNum)

      # Always read current items in the widget
      items = list(tab1.progView.get(0, tk.END))

      # 1) Try exact normalized match (works whether items are bytes or str)
      idx = next((i for i, it in enumerate(items) if _norm(it) == target_norm), None)

      if idx is None:
          # 2) Optional fallback: if your rows are like "Jump Tab-3", match by number
          #    Extract the number from tabNum and look for common forms
          m = re.search(r'\d+', _norm(tabNum))
          if m:
              n = m.group(0)
              forms = {f"tab number {n}", f"tab-{n}", f"tab {n}", f"tab: {n}", f"jump tab-{n}"}
              idx = next((i for i, it in enumerate(items) if _norm(it) in forms), None)

      if idx is None:
          print(f"[Jump T] Not found: {repr(tabNum)}")
      else:
          tab1.progView.selection_clear(0, END)
          tab1.progView.select_set(idx)
          tab1.progView.see(idx)



  if (RUN['cmdTypeLong'] == "Set 5v Outp"):
    if RUN['offlineMode']:
      almStatusLab.config(text="IO not supported in offline programming mode", style="Alarm.TLabel")
      _finish_execute_row()
      return ROW_EXECUTION_REJECTED
    if (RUN['moveInProc'] == 1):
      RUN['moveInProc'] = 2
    outputIndex = command.find("# ")
    valueIndex = command.find("= ")
    output = str(command[outputIndex+2:valueIndex-1])
    value = str(command[valueIndex+2:])
    if (value == "1"):
      command = "ONX"+output+"\n"
    elif (value == "0"):
      command = "OFX"+output+"\n"  
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0,command)
    execution_state, _ = _execute_row_auxiliary_command(
      command,
      expected_response=b"Done",
    )
    if execution_state != ROW_EXECUTION_COMPLETE:
      return execution_state

  if (RUN['cmdTypeLong'] == "Set MBcoil "):
    if RUN['offlineMode']:
      almStatusLab.config(text="IO not supported in offline programming mode", style="Alarm.TLabel")
      _finish_execute_row()
      return ROW_EXECUTION_REJECTED
    if (RUN['moveInProc'] == 1):
      RUN['moveInProc'] = 2
    slavestartIndex = command.find("SlaveID (")  
    inputIndex = command.find("# ")
    valIndex = command.find("= ")
    slaveID = str(command[slavestartIndex+9:inputIndex-9])
    inputNum = str(command[inputIndex+2:valIndex-1])
    valNum = str(command[valIndex+2:])
    command = "SC"+"A"+slaveID+"B"+inputNum+"C"+valNum+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0,command)
    execution_state, _ = _execute_row_main_response(
      command,
      response_parser=parse_controller_modbus_response,
    )
    if execution_state != ROW_EXECUTION_COMPLETE:
      return execution_state

  if (RUN['cmdTypeLong'] == "Set MBoutpu"):
    if RUN['offlineMode']:
      almStatusLab.config(text="IO not supported in offline programming mode", style="Alarm.TLabel")
      _finish_execute_row()
      return ROW_EXECUTION_REJECTED
    if (RUN['moveInProc'] == 1):
      RUN['moveInProc'] = 2
    slavestartIndex = command.find("SlaveID (")  
    inputIndex = command.find("# ")
    valIndex = command.find("= ")
    slaveID = str(command[slavestartIndex+9:inputIndex-9])
    inputNum = str(command[inputIndex+2:valIndex-1])
    valNum = str(command[valIndex+2:])
    command = "SO"+"A"+slaveID+"B"+inputNum+"C"+valNum+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0,command)
    execution_state, _ = _execute_row_main_response(
      command,
      response_parser=parse_controller_modbus_response,
    )
    if execution_state != ROW_EXECUTION_COMPLETE:
      return execution_state
 


  if (RUN['cmdType'] == "Wait T"):
    if RUN['offlineMode']:
      almStatusLab.config(text="Wait time not supported in offline programming mode", style="Alarm.TLabel")
      _finish_execute_row()
      return ROW_EXECUTION_REJECTED
    if (RUN['moveInProc'] == 1):
      RUN['moveInProc'] = 2
    timeIndex = command.find("Wait Time = ")
    try:
      timeSeconds, encoded_wait = _main_timed_wait_seconds(
        command[timeIndex+12:]
      )
    except MotionInputError as exc:
      message = f"Controller timed wait rejected: {exc}"
      logger.error(message)
      almStatusLab.config(text=message, style="Alarm.TLabel")
      almStatusLab2.config(text=message, style="Alarm.TLabel")
      _finish_execute_row()
      return ROW_EXECUTION_REJECTED
    command = "WTS"+encoded_wait+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0,command)
    execution_state, _ = _execute_row_main_response(
      command,
      accepted_responses=("WTdone",),
      response_timeout=timeSeconds + SERIAL_RESPONSE_MARGIN_SECONDS,
    )
    if execution_state != ROW_EXECUTION_COMPLETE:
      return execution_state

  if (RUN['cmdType'] == "Regist"):
    if (RUN['moveInProc'] == 1):
      RUN['moveInProc'] = 2
    regNumIndex = command.find("Register ")
    regEqIndex = command.find(" = ")
    regNumVal = str(command[regNumIndex+9:regEqIndex])
    regEntry = "R"+regNumVal+"EntryField"
    testOper = str(command[regEqIndex+3:regEqIndex+5])
    if (testOper == "++"):
      regCEqVal = str(command[regEqIndex+5:])
      curRegVal = eval(regEntry).get()
      regEqVal = str(int(regCEqVal)+int(curRegVal))      
    elif (testOper == "--"):
      regCEqVal = str(command[regEqIndex+5:])
      curRegVal = eval(regEntry).get()
      regEqVal = str(int(curRegVal)-int(regCEqVal))
    else:
      regEqVal = str(command[regEqIndex+3:])    
    eval(regEntry).delete(0, 'end')
    eval(regEntry).insert(0,regEqVal)

  if (RUN['cmdType'] == "Positi"):
    if (RUN['moveInProc'] == 1):
      RUN['moveInProc'] = 2
    regNumIndex = command.find("Position Register ")
    regElIndex = command.find("Element")
    regEqIndex = command.find(" = ")
    regNumVal = str(command[regNumIndex+18:regElIndex-1])
    regNumEl = str(command[regElIndex+8:regEqIndex])
    regEntry = "SP_"+regNumVal+"_E"+regNumEl+"_EntryField"
    testOper = str(command[regEqIndex+3:regEqIndex+5])
    if (testOper == "++"):
      regCEqVal = str(command[regEqIndex+4:])
      curRegVal = eval(regEntry).get()
      regEqVal = str(float(regCEqVal)+float(curRegVal))      
    elif (testOper == "--"):
      regCEqVal = str(command[regEqIndex+5:])
      curRegVal = eval(regEntry).get()
      regEqVal = str(float(curRegVal)-float(regCEqVal))
    else:
      regEqVal = str(command[regEqIndex+3:])    
    eval(regEntry).delete(0, 'end')
    eval(regEntry).insert(0,regEqVal)
    

  if RUN['cmdType'] in {
    "Calibr", "Cal_J1", "Cal_J2", "Cal_J3", "Cal_J4",
    "Cal_J5", "Cal_J6", "Cal_J7", "Cal_J8", "Cal_J9",
  }:
    calibration_action = {
      "Calibr": _run_program_calibration_all,
      "Cal_J1": _run_program_calibration_j1,
      "Cal_J2": _run_program_calibration_j2,
      "Cal_J3": _run_program_calibration_j3,
      "Cal_J4": _run_program_calibration_j4,
      "Cal_J5": _run_program_calibration_j5,
      "Cal_J6": _run_program_calibration_j6,
      "Cal_J7": _run_program_calibration_j7,
      "Cal_J8": _run_program_calibration_j8,
      "Cal_J9": _run_program_calibration_j9,
    }[RUN['cmdType']]
    if RUN['moveInProc'] == 1:
      RUN['moveInProc'] = 2
    if calibration_action() is not True:
      _finish_execute_row()
      return ROW_EXECUTION_REJECTED

  if (RUN['cmdType'] == "Tool S"):
    if RUN['offlineMode']:
      almStatusLab.config(text="Set tool not supported in offline programming mode", style="Alarm.TLabel")
      _finish_execute_row()
      return ROW_EXECUTION_REJECTED
    message = "Set tool program rows are unsupported by the Teensy 6.7.1 protocol"
    logger.error(message)
    almStatusLab.config(text=message, style="Alarm.TLabel")
    almStatusLab2.config(text=message, style="Alarm.TLabel")
    _finish_execute_row()
    return ROW_EXECUTION_REJECTED
     
  
  if (RUN['cmdType'] == "Move J"): 
    if (RUN['moveInProc'] == 0):
      RUN['moveInProc'] == 1
    xIndex = command.find(" X ")
    yIndex = command.find(" Y ")
    zIndex = command.find(" Z ")
    rzIndex = command.find(" Rz ")
    ryIndex = command.find(" Ry ")
    rxIndex = command.find(" Rx ")
    J7Index = command.find(" J7 ")
    J8Index = command.find(" J8 ")
    J9Index = command.find(" J9 ")	
    SpeedIndex = command.find(" S")
    ACCspdIndex = command.find(" Ac ")
    DECspdIndex = command.find(" Dc ")
    ACCrampIndex = command.find(" Rm ")
    WristConfIndex = command.find(" $")
    RUN['xVal'] = command[xIndex+3:yIndex]
    RUN['yVal'] = command[yIndex+3:zIndex]
    RUN['zVal'] = command[zIndex+3:rzIndex]
    rzVal = command[rzIndex+4:ryIndex]
    ryVal = command[ryIndex+4:rxIndex]
    rxVal = command[rxIndex+4:J7Index]
    J7Val = command[J7Index+4:J8Index]
    J8Val = command[J8Index+4:J9Index]
    J9Val = command[J9Index+4:SpeedIndex]
    speedPrefix = command[SpeedIndex+1:SpeedIndex+3]
    Speed = command[SpeedIndex+4:ACCspdIndex]
    ACCspd = command[ACCspdIndex+4:DECspdIndex]
    DECspd = command[DECspdIndex+4:ACCrampIndex]
    ACCramp = command[ACCrampIndex+4:WristConfIndex]
    RUN['WC'] = command[WristConfIndex+3:]
    LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
    command = "MJ"+"X"+RUN['xVal']+"Y"+RUN['yVal']+"Z"+RUN['zVal']+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"J7"+J7Val+"J8"+J8Val+"J9"+J9Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
    commandVR = "MJ"+"X"+RUN['xVal']+"Y"+RUN['yVal']+"Z"+RUN['zVal']+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
    motion_state = _dispatch_program_command(
      command,
      mj_command,
      commandVR,
      motion_complete,
    )
    if motion_state != ROW_EXECUTION_COMPLETE:
      if motion_state == ROW_EXECUTION_REJECTED:
        _finish_execute_row()
      return motion_state



  if (RUN['cmdType'] == "OFF J "): 
    if (RUN['moveInProc'] == 0):
      RUN['moveInProc'] == 1
    SPnewInex = command.find("[ PR: ")  
    SPendInex = command.find(" ] [")
    xIndex = command.find(" X ")
    yIndex = command.find(" Y ")
    zIndex = command.find(" Z ")
    rzIndex = command.find(" Rz ")
    ryIndex = command.find(" Ry ")
    rxIndex = command.find(" Rx ")
    J7Index = command.find(" J7 ")
    J8Index = command.find(" J8 ")
    J9Index = command.find(" J9 ")	
    SpeedIndex = command.find(" S")
    ACCspdIndex = command.find(" Ac ")
    DECspdIndex = command.find(" Dc ")
    ACCrampIndex = command.find(" Rm ")
    WristConfIndex = command.find(" $")
    SP = str(command[SPnewInex+6:SPendInex])
    cx = eval("SP_"+SP+"_E1_EntryField").get()
    cy = eval("SP_"+SP+"_E2_EntryField").get()
    cz = eval("SP_"+SP+"_E3_EntryField").get()
    crz = eval("SP_"+SP+"_E4_EntryField").get()
    cry = eval("SP_"+SP+"_E5_EntryField").get()
    crx = eval("SP_"+SP+"_E6_EntryField").get()
    RUN['xVal'] = str(float(cx) + float(command[xIndex+3:yIndex]))
    RUN['yVal'] = str(float(cy) + float(command[yIndex+3:zIndex]))
    RUN['zVal'] = str(float(cz) + float(command[zIndex+3:rzIndex]))
    rzVal = str(float(crz) + float(command[rzIndex+4:ryIndex]))
    ryVal = str(float(cry) + float(command[ryIndex+4:rxIndex]))
    rxVal = str(float(crx) + float(command[rxIndex+4:J7Index]))
    J7Val = command[J7Index+4:J8Index]
    J8Val = command[J8Index+4:J9Index]
    J9Val = command[J9Index+4:SpeedIndex]
    speedPrefix = command[SpeedIndex+1:SpeedIndex+3]
    Speed = command[SpeedIndex+4:ACCspdIndex]
    ACCspd = command[ACCspdIndex+4:DECspdIndex]
    DECspd = command[DECspdIndex+4:ACCrampIndex]
    ACCramp = command[ACCrampIndex+4:WristConfIndex]
    RUN['WC'] = command[WristConfIndex+3:]
    LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
    command = "MJ"+"X"+RUN['xVal']+"Y"+RUN['yVal']+"Z"+RUN['zVal']+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"J7"+J7Val+"J8"+J8Val+"J9"+J9Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
    commandVR = "MJ"+"X"+RUN['xVal']+"Y"+RUN['yVal']+"Z"+RUN['zVal']+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
    motion_state = _dispatch_program_command(
      command,
      mj_command,
      commandVR,
      motion_complete,
    )
    if motion_state != ROW_EXECUTION_COMPLETE:
      if motion_state == ROW_EXECUTION_REJECTED:
        _finish_execute_row()
      return motion_state

  if (RUN['cmdType'] == "Move V"): 
    if (RUN['moveInProc'] == 0):
      RUN['moveInProc'] == 1
    SPnewInex = command.find("[ PR: ")  
    SPendInex = command.find(" ] [")
    xIndex = command.find(" X ")
    yIndex = command.find(" Y ")
    zIndex = command.find(" Z ")
    rzIndex = command.find(" Rz ")
    ryIndex = command.find(" Ry ")
    rxIndex = command.find(" Rx ")
    J7Index = command.find(" J7 ")
    J8Index = command.find(" J8 ")
    J9Index = command.find(" J9 ")	
    SpeedIndex = command.find(" S")
    ACCspdIndex = command.find(" Ac ")
    DECspdIndex = command.find(" Dc ")
    ACCrampIndex = command.find(" Rm ")
    WristConfIndex = command.find(" $")
    SP = str(command[SPnewInex+6:SPendInex])
    cx = eval("SP_"+SP+"_E1_EntryField").get()
    cy = eval("SP_"+SP+"_E2_EntryField").get()
    cz = eval("SP_"+SP+"_E3_EntryField").get()
    crz = eval("SP_"+SP+"_E4_EntryField").get()
    cry = eval("SP_"+SP+"_E5_EntryField").get()
    crx = eval("SP_"+SP+"_E6_EntryField").get()
    RUN['xVal'] = str(float(cx) + float(VisRetXrobEntryField.get()))
    RUN['yVal'] = str(float(cy) + float(VisRetYrobEntryField.get()))
    RUN['zVal'] = str(float(cz) + float(command[zIndex+3:rzIndex]))
    rzVal = str(float(crz) + float(command[rzIndex+4:ryIndex]))
    ryVal = str(float(cry) + float(command[ryIndex+4:rxIndex]))
    rxVal = str(float(crx) + float(command[rxIndex+4:J7Index]))
    J7Val = command[J7Index+4:J8Index]
    J8Val = command[J8Index+4:J9Index]
    J9Val = command[J9Index+4:SpeedIndex]
    speedPrefix = command[SpeedIndex+1:SpeedIndex+3]
    Speed = command[SpeedIndex+4:ACCspdIndex]
    ACCspd = command[ACCspdIndex+4:DECspdIndex]
    DECspd = command[DECspdIndex+4:ACCrampIndex]
    ACCramp = command[ACCrampIndex+4:WristConfIndex]
    RUN['WC'] = command[WristConfIndex+3:]
    visRot = VisRetAngleEntryField.get()
    LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
    command = "MV"+"X"+RUN['xVal']+"Y"+RUN['yVal']+"Z"+RUN['zVal']+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"J7"+J7Val+"J8"+J8Val+"J9"+J9Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Vr"+visRot+"Lm"+LoopMode+"\n"
    commandVR = "MV"+"X"+RUN['xVal']+"Y"+RUN['yVal']+"Z"+RUN['zVal']+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Vr"+visRot+"Lm"+LoopMode+"\n"
    motion_state = _dispatch_program_command(
      command,
      mv_command,
      commandVR,
      motion_complete,
    )
    if motion_state != ROW_EXECUTION_COMPLETE:
      if motion_state == ROW_EXECUTION_REJECTED:
        _finish_execute_row()
      return motion_state

  if (RUN['cmdType'] == "Move P"): 
    if (RUN['moveInProc'] == 0):
      RUN['moveInProc'] == 1
    SPnewInex = command.find("[ PR: ")  
    SPendInex = command.find(" ] [")
    J7Index = command.find(" J7 ")
    J8Index = command.find(" J8 ")
    J9Index = command.find(" J9 ")		
    SpeedIndex = command.find(" S")
    ACCspdIndex = command.find(" Ac ")
    DECspdIndex = command.find(" Dc ")
    ACCrampIndex = command.find(" Rm ")
    WristConfIndex = command.find(" $")
    SP = str(command[SPnewInex+6:SPendInex])
    cx = eval("SP_"+SP+"_E1_EntryField").get()
    cy = eval("SP_"+SP+"_E2_EntryField").get()
    cz = eval("SP_"+SP+"_E3_EntryField").get()
    crz = eval("SP_"+SP+"_E4_EntryField").get()
    cry = eval("SP_"+SP+"_E5_EntryField").get()
    crx = eval("SP_"+SP+"_E6_EntryField").get()
    RUN['xVal'] = str(float(cx))
    RUN['yVal'] = str(float(cy))
    RUN['zVal'] = str(float(cz))
    rzVal = str(float(crz))
    ryVal = str(float(cry))
    rxVal = str(float(crx))
    J7Val = command[J7Index+4:J8Index]
    J8Val = command[J8Index+4:J9Index]
    J9Val = command[J9Index+4:SpeedIndex]
    speedPrefix = command[SpeedIndex+1:SpeedIndex+3]
    Speed = command[SpeedIndex+4:ACCspdIndex]
    ACCspd = command[ACCspdIndex+4:DECspdIndex]
    DECspd = command[DECspdIndex+4:ACCrampIndex]
    ACCramp = command[ACCrampIndex+4:WristConfIndex]
    RUN['WC'] = command[WristConfIndex+3:]
    LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
    command = "MJ"+"X"+RUN['xVal']+"Y"+RUN['yVal']+"Z"+RUN['zVal']+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"J7"+J7Val+"J8"+J8Val+"J9"+J9Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
    commandVR = "MJ"+"X"+RUN['xVal']+"Y"+RUN['yVal']+"Z"+RUN['zVal']+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
    motion_state = _dispatch_program_command(
      command,
      mj_command,
      commandVR,
      motion_complete,
    )
    if motion_state != ROW_EXECUTION_COMPLETE:
      if motion_state == ROW_EXECUTION_REJECTED:
        _finish_execute_row()
      return motion_state

  if (RUN['cmdType'] == "OFF PR"): 
    if (RUN['moveInProc'] == 0):
      RUN['moveInProc'] == 1
    SPnewInex = command.find("[ PR: ")  
    SPendInex = command.find(" ] offs")
    SP2newInex = command.find("[ *PR: ")  
    SP2endInex = command.find(" ]  [")
    J7Index = command.find(" J7 ")
    J8Index = command.find(" J8 ")
    J9Index = command.find(" J9 ")
    SpeedIndex = command.find(" S")
    ACCspdIndex = command.find(" Ac ")
    DECspdIndex = command.find(" Dc ")
    ACCrampIndex = command.find(" Rm ")
    WristConfIndex = command.find(" $")
    SP = str(command[SPnewInex+6:SPendInex])
    SP2 = str(command[SP2newInex+7:SP2endInex])
    RUN['xVal'] = str(float(eval("SP_"+SP+"_E1_EntryField").get()) + float(eval("SP_"+SP2+"_E1_EntryField").get()))
    RUN['yVal'] = str(float(eval("SP_"+SP+"_E2_EntryField").get()) + float(eval("SP_"+SP2+"_E2_EntryField").get()))
    RUN['zVal'] = str(float(eval("SP_"+SP+"_E3_EntryField").get()) + float(eval("SP_"+SP2+"_E3_EntryField").get()))
    rzVal = str(float(eval("SP_"+SP+"_E4_EntryField").get()) + float(eval("SP_"+SP2+"_E4_EntryField").get()))
    ryVal = str(float(eval("SP_"+SP+"_E5_EntryField").get()) + float(eval("SP_"+SP2+"_E5_EntryField").get()))
    rxVal = str(float(eval("SP_"+SP+"_E6_EntryField").get()) + float(eval("SP_"+SP2+"_E6_EntryField").get()))	
    J7Val = command[J7Index+4:J8Index]
    J8Val = command[J8Index+4:J9Index]
    J9Val = command[J9Index+4:SpeedIndex]
    speedPrefix = command[SpeedIndex+1:SpeedIndex+3]
    Speed = command[SpeedIndex+4:ACCspdIndex]
    ACCspd = command[ACCspdIndex+4:DECspdIndex]
    DECspd = command[DECspdIndex+4:ACCrampIndex]
    ACCramp = command[ACCrampIndex+4:WristConfIndex]
    RUN['WC'] = command[WristConfIndex+3:]
    LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
    command = "MJ"+"X"+RUN['xVal']+"Y"+RUN['yVal']+"Z"+RUN['zVal']+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"J7"+J7Val+"J8"+J8Val+"J9"+J9Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
    commandVR = "MJ"+"X"+RUN['xVal']+"Y"+RUN['yVal']+"Z"+RUN['zVal']+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
    motion_state = _dispatch_program_command(
      command,
      mj_command,
      commandVR,
      motion_complete,
    )
    if motion_state != ROW_EXECUTION_COMPLETE:
      if motion_state == ROW_EXECUTION_REJECTED:
        _finish_execute_row()
      return motion_state

  if (RUN['cmdType'] == "Move L"): 
    if (RUN['moveInProc'] == 0):
      RUN['moveInProc'] == 1
    xIndex = command.find(" X ")
    yIndex = command.find(" Y ")
    zIndex = command.find(" Z ")
    rzIndex = command.find(" Rz ")
    ryIndex = command.find(" Ry ")
    rxIndex = command.find(" Rx ")
    J7Index = command.find(" J7 ")
    J8Index = command.find(" J8 ")
    J9Index = command.find(" J9 ")
    SpeedIndex = command.find(" S")
    ACCspdIndex = command.find(" Ac ")
    DECspdIndex = command.find(" Dc ")
    ACCrampIndex = command.find(" Rm ")
    RoundingIndex = command.find(" Rnd ")
    WristConfIndex = command.find(" $")
    RUN['xVal'] = command[xIndex+3:yIndex]
    RUN['yVal'] = command[yIndex+3:zIndex]
    RUN['zVal'] = command[zIndex+3:rzIndex]
    rzVal = command[rzIndex+4:ryIndex]
    #if (np.sign(float(rzVal)) != np.sign(float(CAL['RzcurPos']))):
    #  rzVal=str(float(rzVal)*-1)
    ryVal = command[ryIndex+4:rxIndex]
    rxVal = command[rxIndex+4:J7Index]
    J7Val = command[J7Index+4:J8Index]
    J8Val = command[J8Index+4:J9Index]
    J9Val = command[J9Index+4:SpeedIndex]
    speedPrefix = command[SpeedIndex+1:SpeedIndex+3]
    Speed = command[SpeedIndex+4:ACCspdIndex]
    ACCspd = command[ACCspdIndex+4:DECspdIndex]
    DECspd = command[DECspdIndex+4:ACCrampIndex]
    ACCramp = command[ACCrampIndex+4:RoundingIndex]
    Rounding = command[RoundingIndex+5:WristConfIndex]
    RUN['WC'] = command[WristConfIndex+3:]
    LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
    DisWrist = str(CAL['DisableWristRotVal'].get())
    command = "ML"+"X"+RUN['xVal']+"Y"+RUN['yVal']+"Z"+RUN['zVal']+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"J7"+J7Val+"J8"+J8Val+"J9"+J9Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"Rnd"+Rounding+"W"+RUN['WC']+"Lm"+LoopMode+"Q"+DisWrist+"\n"
    commandVR = "MJ"+"X"+RUN['xVal']+"Y"+RUN['yVal']+"Z"+RUN['zVal']+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
    motion_state = _dispatch_program_command(
      command,
      mj_command,
      commandVR,
      motion_complete,
    )
    if motion_state != ROW_EXECUTION_COMPLETE:
      if motion_state == ROW_EXECUTION_REJECTED:
        _finish_execute_row()
      return motion_state




  if (RUN['cmdType'] == "Move R"):
    if (RUN['moveInProc'] == 0):
      RUN['moveInProc'] == 1 
    J1Index = command.find(" J1 ")
    J2Index = command.find(" J2 ")
    J3Index = command.find(" J3 ")
    J4Index = command.find(" J4 ")
    J5Index = command.find(" J5 ")
    J6Index = command.find(" J6 ")
    J7Index = command.find(" J7 ")
    J8Index = command.find(" J8 ")
    J9Index = command.find(" J9 ")
    SpeedIndex = command.find(" S")
    ACCspdIndex = command.find(" Ac ")
    DECspdIndex = command.find(" Dc ")
    ACCrampIndex = command.find(" Rm ")
    WristConfIndex = command.find(" $")
    J1Val = command[J1Index+4:J2Index]
    J2Val = command[J2Index+4:J3Index]
    J3Val = command[J3Index+4:J4Index]
    J4Val = command[J4Index+4:J5Index]
    J5Val = command[J5Index+4:J6Index]
    J6Val = command[J6Index+4:J7Index]
    J7Val = command[J7Index+4:J8Index]
    J8Val = command[J8Index+4:J9Index]
    J9Val = command[J9Index+4:SpeedIndex]
    speedPrefix = command[SpeedIndex+1:SpeedIndex+3]
    Speed = command[SpeedIndex+4:ACCspdIndex]
    ACCspd = command[ACCspdIndex+4:DECspdIndex]
    DECspd = command[DECspdIndex+4:ACCrampIndex]
    ACCramp = command[ACCrampIndex+4:WristConfIndex]
    RUN['WC'] = command[WristConfIndex+3:]
    LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
    command = "RJ"+"A"+J1Val+"B"+J2Val+"C"+J3Val+"D"+J4Val+"E"+J5Val+"F"+J6Val+"J7"+J7Val+"J8"+J8Val+"J9"+J9Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
    commandVR = "RJ"+"A"+J1Val+"B"+J2Val+"C"+J3Val+"D"+J4Val+"E"+J5Val+"F"+J6Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
    motion_state = _dispatch_program_command(
      command,
      rj_command,
      commandVR,
      motion_complete,
    )
    if motion_state != ROW_EXECUTION_COMPLETE:
      if motion_state == ROW_EXECUTION_REJECTED:
        _finish_execute_row()
      return motion_state
      
  if (RUN['cmdType'] == "Move A"):
    message = (
      "Arc program motion is disabled pending a safe Teensy MA protocol"
    )
    logger.error(message)
    almStatusLab.config(text=message, style="Alarm.TLabel")
    almStatusLab2.config(text=message, style="Alarm.TLabel")
    _finish_execute_row()
    return ROW_EXECUTION_REJECTED

  if (RUN['cmdType'] == "Move C"):
    message = (
      "Circle program motion is disabled pending a safe Teensy MC protocol"
    )
    logger.error(message)
    almStatusLab.config(text=message, style="Alarm.TLabel")
    almStatusLab2.config(text=message, style="Alarm.TLabel")
    _finish_execute_row()
    return ROW_EXECUTION_REJECTED

  if (RUN['cmdType'] == "Start "):
    message = "Spline program motion is disabled pending an owned response protocol"
    logger.error(message)
    almStatusLab.config(text=message, style="Alarm.TLabel")
    almStatusLab2.config(text=message, style="Alarm.TLabel")
    _finish_execute_row()
    return ROW_EXECUTION_REJECTED

  if (RUN['cmdType'] == "End Sp"):
    message = "Spline program motion is disabled pending an owned response protocol"
    logger.error(message)
    almStatusLab.config(text=message, style="Alarm.TLabel")
    almStatusLab2.config(text=message, style="Alarm.TLabel")
    _finish_execute_row()
    return ROW_EXECUTION_REJECTED

  if(RUN['cmdType'] == "Cam On"):
    if (RUN['moveInProc'] == 1):
      RUN['moveInProc'] = 2
    start_vid()

  if(RUN['cmdType'] == "Cam Of"):
    if (RUN['moveInProc'] == 1):
      RUN['moveInProc'] = 2
    stop_vid()  

  if(RUN['cmdType'] == "Vis Fi"):
    #if (RUN['moveInProc'] == 1):
      #RUN['moveInProc'] = 2
    templateIndex = command.find("Vis Find - ")
    bgColorIndex = command.find(" - BGcolor ")
    scoreIndex = command.find(" Score ")
    passIndex = command.find(" Pass ")
    failIndex = command.find(" Fail ")
    template = command[templateIndex+11:bgColorIndex]
    checkBG = command[bgColorIndex+11:scoreIndex]
    if(checkBG == "(Auto)"):
      background = "Auto"
    else:  
      background = eval(command[bgColorIndex+11:scoreIndex])
    min_score = float(command[scoreIndex+7:passIndex])*.01
    take_pic()
    status = visFind(template,min_score,background)
    if (status == "pass"):
      tabNum = ("Tab Number " + str(command[passIndex+6:failIndex]) + "\r\n").encode('utf-8')
      index = tab1.progView.get(0, "end").index(tabNum)
      tab1.progView.selection_clear(0, END)
      tab1.progView.select_set(index)  
    elif (status == "fail"): 
      tabNum = ("Tab Number " + str(command[failIndex+6:]) + "\r\n").encode('utf-8')
      index = tab1.progView.get(0, "end").index(tabNum)
      tab1.progView.selection_clear(0, END)
      tab1.progView.select_set(index) 
  


  _finish_execute_row()
  return ROW_EXECUTION_COMPLETE
  





  
##############################################################################################################################################################
### BUTTON JOGGING DEFS ############################################################################################################## BUTTON JOGGING DEFS ###
##############################################################################################################################################################  

##xbox  ######################################################################################################################################################

################################################################
# New XBOX updates require windows DLL files
# use old method when not on windows


if CE['Platform']['IS_WINDOWS']:
  # ---------- XINPUT (Xbox 360 / Xbox One) - Windows ----------
  for _dll in ("XInput1_4.dll", "XInput9_1_0.dll", "XInput1_3.dll"):
      try:
          _xinput = ctypes.WinDLL(_dll); break
      except OSError:
          _xinput = None
  if _xinput is None:
      raise OSError("XInput DLL not found")

  class XINPUT_GAMEPAD(ctypes.Structure):
      _fields_ = [
          ("wButtons", ctypes.c_ushort),
          ("bLeftTrigger", ctypes.c_ubyte),
          ("bRightTrigger", ctypes.c_ubyte),
          ("sThumbLX", ctypes.c_short),
          ("sThumbLY", ctypes.c_short),
          ("sThumbRX", ctypes.c_short),
          ("sThumbRY", ctypes.c_short),
      ]
  class XINPUT_STATE(ctypes.Structure):
      _fields_ = [("dwPacketNumber", ctypes.c_uint), ("Gamepad", XINPUT_GAMEPAD)]
  XInputGetState = _xinput.XInputGetState
  XInputGetState.argtypes = [ctypes.c_uint, ctypes.POINTER(XINPUT_STATE)]
  XInputGetState.restype  = ctypes.c_uint

  # ---------- Buttons / DPAD ----------
  BTN_A = 0x1000
  BTN_B = 0x2000
  BTN_X = 0x4000
  BTN_Y = 0x8000
  BTN_START = 0x0010
  BTN_LB = 0x0100
  BTN_RB = 0x0200
  DPAD_UP    = 0x0001
  DPAD_DOWN  = 0x0002
  DPAD_LEFT  = 0x0004
  DPAD_RIGHT = 0x0008

  # ---------- Stick robustness ----------
  DZ_LX = 7849; DZ_LY = 7849
  DZ_RX = 8689 + 1500; DZ_RY = 8689 + 1500
  START_THR_L = 0.18; STOP_THR_L = 0.12
  START_THR_R = 0.22; STOP_THR_R = 0.10
  LPF_ALPHA_L = 0.30; LPF_ALPHA_R = 0.35

  def _norm_axis(v, dz):
      if abs(v) < dz: return 0.0
      n = v / 32767.0
      return -1.0 if n < -1.0 else (1.0 if n > 1.0 else n)

  def _lbl(text, style="Warn.TLabel"):
      try:
          root.after(0, lambda: almStatusLab.config(text=text, style=style))
          root.after(0, lambda: almStatusLab2.config(text=text, style=style))
      except Exception:
          pass

  # ---------- Mode state (A=Joint, B=Cartesian) ----------
  RUN['_mainMode'] = 1
  def _show_mode_banner():
      try:
          _lbl("JOINT MODE" if RUN['_mainMode'] == 1 else "CARTESIAN MODE", style="Warn.TLabel")
      except Exception:
          pass

  # ---------- Tk-thread GUI calls ----------
  def _tk_call(fn, *args):
      if not callable(fn): return False
      try:
          root.after(0, (lambda f=fn, a=args: f(*a)))
          return True
      except Exception:
          return False

  def _gui_stop():
      live_pending = globals().get('live_serial_result_pending')
      stop_requested = globals().get('live_jog_stop_requested')
      if (
          live_pending is not None
          and stop_requested is not None
          and live_pending.is_set()
      ):
          stop_requested.set()
      scheduled = _tk_call(globals().get("StopJog"), None)
      if not scheduled:
          logger.warning("Unable to schedule the live-jog stop callback")
      return scheduled

  def _gui_start_joint(active):
      callback = globals().get("LiveJointJog")
      return callable(callback) and callback(_lj_code(*active)) is True

  def _gui_start_cart(active):
      callback = globals().get("LiveCarJog")
      axis, direction = active
      code = _cart_code(axis, direction)
      return callable(callback) and code is not None and callback(code) is True

  def _gui_start_tool(active):
      callback = globals().get("LiveToolJog")
      axis, direction = active
      code = _tool_code(axis, direction)
      return callable(callback) and code is not None and callback(code) is True

  def _schedule_xbox_motion(delay_ms, callback, reject_callback):
      if application_closing.is_set():
          return LiveMotionScheduleResult.CANCELLED

      def schedule_on_tk():
          if application_closing.is_set():
              reject_callback(LiveMotionScheduleResult.CANCELLED)
              return
          try:
              if delay_ms > 0:
                  root.after(delay_ms, callback)
              else:
                  callback()
          except Exception as exc:
              reject_callback(exc)

      return _tk_call(schedule_on_tk)

  # ----- Teach (X button) -----
  def _teach_position():
      if not _tk_call(globals().get("teachInsertBelSelected")):
          logger.warning("Unable to schedule the Xbox teach-position callback")

  # ----- Servo gripper toggle (Y button) over ser2 -----
  RUN['_grip_closed'] = False  # False = open; first press closes (SV0P0)
  RUN['_grip_pending_request_id'] = None

  def _toggle_servo_gripper():
      if not _tk_call(_request_xbox_auxiliary_toggle, "_grip_closed"):
          logger.warning("Unable to schedule the Xbox servo-gripper toggle")

  # ----- Pneumatic gripper toggle (START) over ser2 -----
  RUN['_pneu_open'] = False  # False = closed
  RUN['_pneu_pending_request_id'] = None

  def _toggle_pneu_gripper():
      if not _tk_call(_request_xbox_auxiliary_toggle, "_pneu_open"):
          logger.warning("Unable to schedule the Xbox pneumatic-gripper toggle")

  # ----- Triggers adjust speedEntryField (smart stepping) -----
  def _bump_speed_smart(delta_hint):
      def do():
          try:
              val = int(speedEntryField.get())
          except Exception:
              val = 25
          if delta_hint < 0:
              # Decrease: above 5 → -5; at/under 5 → -1 (to a floor of 1)
              step = -5 if val > 5 else -1
          else:
              # Increase: below 5 → +1 up to 5; above 5 → +5
              step = +1 if val < 5 else +5
          newv = max(1, min(100, val + step))
          speedEntryField.delete(0, 'end')
          speedEntryField.insert(0, str(newv))
      try:
          root.after(0, do)
      except Exception:
          do()

  RUN['_last_input_time'] = 0.0
  RUN['_xbox_watchdog_failed'] = False
  SWITCH_DELAY_MS = 60
  WATCHDOG_MS     = 200

  def _lj_code(j, direction):  # J1- = 10, J1+ = 11; J2- = 20, J2+ = 21; ...
      return j*10 + (1 if direction > 0 else 0)

  def _cart_code(axis, d):
      if axis == 'X':  return 10 if d < 0 else 11
      if axis == 'Y':  return 20 if d < 0 else 21
      if axis == 'Z':  return 30 if d < 0 else 31
      if axis == 'Rz': return 40 if d < 0 else 41
      if axis == 'Ry': return 50 if d < 0 else 51
      if axis == 'Rx': return 60 if d < 0 else 61
      return None

  def _tool_code(axis, d):
      if axis == 'Tz': return 30 if d < 0 else 31
      return None

  def _report_xbox_motion_error(message):
      logger.error("Xbox live-motion arbitration failed: %s", message)
      _lbl("XBOX LIVE MOTION FAILED", style="Alarm.TLabel")

  joint_xbox_arbiter = DeferredLiveMotionArbiter(
      _schedule_xbox_motion,
      _gui_start_joint,
      _gui_stop,
      SWITCH_DELAY_MS,
      _report_xbox_motion_error,
  )
  cartesian_xbox_arbiter = DeferredLiveMotionArbiter(
      _schedule_xbox_motion,
      _gui_start_cart,
      _gui_stop,
      SWITCH_DELAY_MS,
      _report_xbox_motion_error,
  )
  tool_xbox_arbiter = DeferredLiveMotionArbiter(
      _schedule_xbox_motion,
      _gui_start_tool,
      _gui_stop,
      SWITCH_DELAY_MS,
      _report_xbox_motion_error,
  )

  def _request_switch(new_active):
      return joint_xbox_arbiter.request(new_active)

  def _request_switch_cart(new_active):
      return cartesian_xbox_arbiter.request(new_active)

  def _request_switch_tool(new_active):
      return tool_xbox_arbiter.request(new_active)

  def _fail_xbox_watchdog(error):
      RUN['_xbox_watchdog_failed'] = True
      live_stop = globals().get('live_jog_stop_requested')
      if live_stop is not None:
          live_stop.set()
      offline_lock = globals().get('offline_live_jog_state_lock')
      offline_stop = globals().get('offline_live_jog_stop_event')
      if offline_lock is not None and offline_stop is not None:
          with offline_lock:
              offline_stop.set()
      for requester in (
          _request_switch,
          _request_switch_cart,
          _request_switch_tool,
      ):
          try:
              requester(None)
          except Exception:
              logger.exception("Unable to stop Xbox motion after watchdog failure")
      logger.error("Xbox watchdog scheduling failed: %s", error)
      _lbl("XBOX WATCHDOG FAILED", style="Alarm.TLabel")
      return False

  def _schedule_watchdog():
      try:
          root.after(WATCHDOG_MS, _watchdog_tick)
      except Exception as exc:
          return _fail_xbox_watchdog(exc)
      return True

  def _watchdog_tick():
      try:
          if application_closing.is_set():
              return
          now = time.monotonic()
          if (now - RUN['_last_input_time']) * 1000.0 > WATCHDOG_MS:
              _request_switch(None)
              _request_switch_cart(None)
              _request_switch_tool(None)
      finally:
          if (
              not application_closing.is_set()
              and not RUN['_xbox_watchdog_failed']
          ):
              _schedule_watchdog()

  # ---------- Axis selection (one axis per stick) ----------
  RUN['_smooth'] = {'LX': 0, 'LY': 0, 'RX': 0, 'RY': 0}
  def _lp(prev, new, alpha): return int(prev + alpha * (new - prev))

  def _stick_to_axis(raw_x, raw_y, dz_x, dz_y, alpha, start_thr, stop_thr, tag):
      """
      Returns (axis, dir) with axis in {'X','Y',None}, dir in {-1,0,+1}
      (One axis per stick; picks stronger if diagonal.)
      """
      if tag == 'L':
          RUN['_smooth']['LX'] = _lp(RUN['_smooth']['LX'], raw_x, alpha)
          RUN['_smooth']['LY'] = _lp(RUN['_smooth']['LY'], raw_y, alpha)
          nx = _norm_axis(RUN['_smooth']['LX'], dz_x); ny = _norm_axis(RUN['_smooth']['LY'], dz_y)
      else:
          RUN['_smooth']['RX'] = _lp(RUN['_smooth']['RX'], raw_x, alpha)
          RUN['_smooth']['RY'] = _lp(RUN['_smooth']['RY'], raw_y, alpha)
          nx = _norm_axis(RUN['_smooth']['RX'], dz_x); ny = _norm_axis(RUN['_smooth']['RY'], dz_y)

      ix = 1 if nx >= start_thr else (-1 if nx <= -start_thr else 0)
      iy = 1 if ny >= start_thr else (-1 if ny <= -start_thr else 0)

      if ix == 0 and iy == 0:
          return None, 0
      if ix != 0 and iy != 0:
          return ('X', 1 if nx>0 else -1) if abs(nx) >= abs(ny) else ('Y', 1 if ny>0 else -1)
      return ('X', ix) if ix != 0 else ('Y', iy)

  # --- Dominant-axis lock for CARTESIAN left stick (prevents X<->Y flip mid-hold)
  _cartL_lock = {'which': None, 'dir': 0}
  def _cart_left_locked(raw_lx, raw_ly):
      # global RUN['_smooth']
      RUN['_smooth']['LX'] = int(RUN['_smooth']['LX'] + LPF_ALPHA_L * (raw_lx - RUN['_smooth']['LX']))
      RUN['_smooth']['LY'] = int(RUN['_smooth']['LY'] + LPF_ALPHA_L * (raw_ly - RUN['_smooth']['LY']))
      nx = _norm_axis(RUN['_smooth']['LX'], DZ_LX)
      ny = _norm_axis(RUN['_smooth']['LY'], DZ_LY)
      ix =  1 if nx >= START_THR_L else (-1 if nx <= -START_THR_L else 0)
      iy =  1 if ny >= START_THR_L else (-1 if ny <= -START_THR_L else 0)
      lock = _cartL_lock
      if lock['which'] == 'X':
          if abs(nx) > STOP_THR_L:
              lock['dir'] = 1 if nx > 0 else -1
              return 'X', lock['dir']
          else:
              lock['which'] = None; lock['dir'] = 0
      elif lock['which'] == 'Y':
          if abs(ny) > STOP_THR_L:
              lock['dir'] = 1 if ny > 0 else -1
              return 'Y', lock['dir']
          else:
              lock['which'] = None; lock['dir'] = 0
      if ix == 0 and iy == 0:
          return None, 0
      if ix != 0 and iy != 0:
          if abs(nx) >= abs(ny):
              lock['which'] = 'X'; lock['dir'] = 1 if nx > 0 else -1
          else:
              lock['which'] = 'Y'; lock['dir'] = 1 if ny > 0 else -1
      elif ix != 0:
          lock['which'] = 'X'; lock['dir'] = ix
      else:
          lock['which'] = 'Y'; lock['dir'] = iy
      return lock['which'], lock['dir']

  # ---------- Controller plumbing ----------
  def _find_controller():
      st = XINPUT_STATE()
      for i in range(4):
          if XInputGetState(i, ctypes.byref(st)) == 0:
              return i
      return None

  def _poll_loop():
      # global RUN['_mainMode'], RUN['_last_input_time']
      idx = _find_controller()
      if idx is None:
          _lbl("No XInput controller detected"); return
      _lbl(f"Xbox connected (slot {idx})")
      if not _tk_call(_schedule_watchdog):
          _fail_xbox_watchdog("initial Tk scheduling failed")
          return

      last_buttons = 0  # for edges X/Y/START/LB/RB
      lt_down = False
      rt_down = False
      TRIG_THR = 30  # analog threshold for a 'press'

      while (
          not application_closing.is_set()
          and not RUN['_xbox_watchdog_failed']
      ):
          st = XINPUT_STATE()
          if XInputGetState(idx, ctypes.byref(st)) != 0:
              _request_switch(None); _request_switch_cart(None); _request_switch_tool(None)
              _lbl("XBOX CONTROLLER NOT RESPONDING", style="Alarm.TLabel")
              time.sleep(0.2)
              idx = _find_controller()
              if idx is not None: _lbl(f"Xbox reconnected (slot {idx})")
              continue

          gp = st.Gamepad
          buttons = gp.wButtons

          # --- Button edges: X (teach), Y (servo gripper), START (pneumatic gripper) ---
          pressed = buttons & ~last_buttons
          if pressed & BTN_X:
              _teach_position()
          if pressed & BTN_Y:
              _toggle_servo_gripper()
          if pressed & BTN_START:
              _toggle_pneu_gripper()

          # --- Triggers: smart speed (edge) ---
          if gp.bLeftTrigger >= TRIG_THR and not lt_down:
              lt_down = True
              _bump_speed_smart(-1)
          elif gp.bLeftTrigger < TRIG_THR and lt_down:
              lt_down = False

          if gp.bRightTrigger >= TRIG_THR and not rt_down:
              rt_down = True
              _bump_speed_smart(+1)
          elif gp.bRightTrigger < TRIG_THR and rt_down:
              rt_down = False

          # --- Mode switching (A=Joint, B=Cartesian) ---
          if buttons & BTN_A:
              if RUN['_mainMode'] != 1:
                  _request_switch(None); _request_switch_cart(None); _request_switch_tool(None)
                  RUN['_mainMode'] = 1; _show_mode_banner()
          elif buttons & BTN_B:
              if RUN['_mainMode'] != 2:
                  _request_switch(None); _request_switch_cart(None); _request_switch_tool(None)
                  RUN['_mainMode'] = 2; _show_mode_banner()

          # --- Tool bumpers (priority over sticks/dpad) ---
          # LB => Tz−, RB => Tz+
          intended_tool = None
          if (buttons & BTN_LB) and not (buttons & BTN_RB):
              intended_tool = ('Tz', -1)
          elif (buttons & BTN_RB) and not (buttons & BTN_LB):
              intended_tool = ('Tz', +1)
          else:
              intended_tool = None

          if intended_tool is not None:
              # tool jog takes priority: stop other modes first
              _request_switch(None)
              _request_switch_cart(None)
              _request_switch_tool(intended_tool)
          else:
              _request_switch_tool(None)

              # --- Movement based on mode (only if no tool jog active) ---
              if RUN['_mainMode'] == 1:
                  # JOINT MODE (custom mapping)
                  axL, dirL = _stick_to_axis(gp.sThumbLX, gp.sThumbLY, DZ_LX, DZ_LY,
                                            LPF_ALPHA_L, START_THR_L, STOP_THR_L, 'L')
                  axR, dirR = _stick_to_axis(gp.sThumbRX, gp.sThumbRY, DZ_RX, DZ_RY,
                                            LPF_ALPHA_R, START_THR_R, STOP_THR_R, 'R')

                  # D-pad: J5 (Down=+1, Up=-1), J6 (Right=+1, Left=-1)
                  dJ5 = (+1 if (buttons & DPAD_DOWN) else -1 if (buttons & DPAD_UP) else 0)
                  dJ6 = (+1 if (buttons & DPAD_RIGHT) else -1 if (buttons & DPAD_LEFT) else 0)

                  intended = None
                  if dJ5 != 0:
                      intended = (5, dJ5)
                  elif dJ6 != 0:
                      intended = (6, dJ6)
                  elif axL is not None:
                      intended = (1, -dirL) if axL == 'X' else (2, -dirL)
                  elif axR is not None:
                      intended = (3, dirR) if axR == 'X' else (4, dirR)

                  _request_switch(intended)

              else:
                  # CARTESIAN MODE (left-stick axis lock)
                  axL, dirL = _cart_left_locked(gp.sThumbLX, gp.sThumbLY)
                  axR, dirR = _stick_to_axis(gp.sThumbRX, gp.sThumbRY, DZ_RX, DZ_RY,
                                            LPF_ALPHA_R, START_THR_R, STOP_THR_R, 'R')

                  # D-pad: Rx / Ry
                  dRx = (+1 if (buttons & DPAD_UP)    else -1 if (buttons & DPAD_DOWN) else 0)
                  dRy = (+1 if (buttons & DPAD_RIGHT) else -1 if (buttons & DPAD_LEFT) else 0)

                  intended_cart = None
                  if dRx != 0:
                      intended_cart = ('Rx', dRx)
                  elif dRy != 0:
                      intended_cart = ('Ry', dRy)
                  elif axL is not None:
                      intended_cart = ('X', dirL) if axL == 'Y' else ('Y', -dirL)
                  elif axR is not None:
                      intended_cart = ('Rz', dirR) if axR == 'X' else ('Z', dirR)

                  _request_switch_cart(intended_cart)

          RUN['_last_input_time'] = time.monotonic()
          last_buttons = buttons
          time.sleep(0.008)  # ~120 Hz

      _request_switch(None)
      _request_switch_cart(None)
      _request_switch_tool(None)

  # ---------- Public entry ----------
  def start_xbox():
      RUN['_xbox_watchdog_failed'] = False
      threading.Thread(target=_poll_loop, daemon=True).start()
      _lbl("Xbox ON / polling…", style="Warn.TLabel")

else:
  from inputs import get_gamepad
  def xbox():
    def send_xbox_auxiliary(command):
      serial_port = RUN.get('ser2')
      return _exchange_xbox_auxiliary_command(
        command,
        _xbox_auxiliary_expected_response(command, serial_port),
        serial_port,
      )

    def threadxbox():
      # global RUN['xboxUse']
      jogMode = 1
      if RUN['xboxUse'] == 0:
        RUN['xboxUse'] = 1
        mainMode = 1
        jogMode = 1
        grip = 0
        almStatusLab.config(text='JOGGING JOINTS 1 & 2', style="Warn.TLabel")
        almStatusLab2.config(text='JOGGING JOINTS 1 & 2', style="Warn.TLabel")
        #xbcStatusLab.config(text='Xbox ON', )
        ChgDis(2)
      else:
        RUN['xboxUse'] = 0
        almStatusLab.config(text='XBOX CONTROLLER OFF', style="Warn.TLabel")
        almStatusLab2.config(text='XBOX CONTROLLER OFF', style="Warn.TLabel")
        #xbcStatusLab.config(text='Xbox OFF', )
      while RUN['xboxUse'] == 1 and not application_closing.is_set():
        try:
        #if (TRUE):
          events = get_gamepad()
          if application_closing.is_set() or RUN['xboxUse'] != 1:
            break
          for event in events:
            ##DISTANCE
            if (event.code == 'ABS_RZ' and event.state >= 100):
              ChgDis(0)
            elif (event.code == 'ABS_Z' and event.state >= 100): 
              ChgDis(1)
            ##SPEED
            elif (event.code == 'BTN_TR' and event.state == 1): 
              ChgSpd(0)
            elif (event.code == 'BTN_TL' and event.state == 1): 
              ChgSpd(1)
            ##JOINT MODE
            elif (event.code == 'BTN_WEST' and event.state == 1): 
              if mainMode != 1:
                mainMode = 1
                jogMode = 1
                almStatusLab.config(text='JOGGING JOINTS 1 & 2', style="Warn.TLabel")
                almStatusLab2.config(text='JOGGING JOINTS 1 & 2', style="Warn.TLabel")
              else:                
                jogMode +=1        
              if jogMode == 2:
                almStatusLab.config(text='JOGGING JOINTS 3 & 4', style="Warn.TLabel")
                almStatusLab2.config(text='JOGGING JOINTS 3 & 4', style="Warn.TLabel")
              elif jogMode == 3:
                almStatusLab.config(text='JOGGING JOINTS 5 & 6', style="Warn.TLabel")
                almStatusLab2.config(text='JOGGING JOINTS 5 & 6', style="Warn.TLabel")
              elif jogMode == 4:
                jogMode = 1
                almStatusLab.config(text='JOGGING JOINTS 1 & 2', style="Warn.TLabel")
                almStatusLab2.config(text='JOGGING JOINTS 1 & 2', style="Warn.TLabel")
            ##JOINT JOG
            elif (mainMode == 1 and event.code == 'ABS_HAT0X' and event.state == 1 and jogMode == 1): 
              J1jogNeg(float(incrementEntryField.get()))    
            elif (mainMode == 1 and event.code == 'ABS_HAT0X' and event.state == -1 and jogMode == 1): 
              J1jogPos(float(incrementEntryField.get()))
            elif (mainMode == 1 and event.code == 'ABS_HAT0Y' and event.state == -1 and jogMode == 1): 
              J2jogNeg(float(incrementEntryField.get()))    
            elif (mainMode == 1 and event.code == 'ABS_HAT0Y' and event.state == 1 and jogMode == 1): 
              J2jogPos(float(incrementEntryField.get()))           
            elif (mainMode == 1 and event.code == 'ABS_HAT0Y' and event.state == -1 and jogMode == 2): 
              J3jogNeg(float(incrementEntryField.get()))    
            elif (mainMode == 1 and event.code == 'ABS_HAT0Y' and event.state == 1 and jogMode == 2): 
              J3jogPos(float(incrementEntryField.get()))
            elif (mainMode == 1 and event.code == 'ABS_HAT0X' and event.state == 1 and jogMode == 2): 
              J4jogNeg(float(incrementEntryField.get()))    
            elif (mainMode == 1 and event.code == 'ABS_HAT0X' and event.state == -1 and jogMode == 2): 
              J4jogPos(float(incrementEntryField.get()))           
            elif (mainMode == 1 and event.code == 'ABS_HAT0Y' and event.state == -1 and jogMode == 3): 
              J5jogNeg(float(incrementEntryField.get()))    
            elif (mainMode == 1 and event.code == 'ABS_HAT0Y' and event.state == 1 and jogMode == 3): 
              J5jogPos(float(incrementEntryField.get()))
            elif (mainMode == 1 and event.code == 'ABS_HAT0X' and event.state == 1 and jogMode == 3): 
              J6jogNeg(float(incrementEntryField.get()))    
            elif (mainMode == 1 and event.code == 'ABS_HAT0X' and event.state == -1 and jogMode == 3): 
              J6jogPos(float(incrementEntryField.get()))                      
          ##CARTESIAN DIR MODE
            elif (event.code == 'BTN_SOUTH' and event.state == 1): 
              if mainMode != 2:
                mainMode = 2
                jogMode = 1
                almStatusLab.config(text='JOGGING X & Y AXIS', style="Warn.TLabel")
                almStatusLab2.config(text='JOGGING X & Y AXIS', style="Warn.TLabel")
              else:                
                jogMode +=1        
              if jogMode == 2:
                almStatusLab.config(text='JOGGING Z AXIS', style="Warn.TLabel")
                almStatusLab2.config(text='JOGGING Z AXIS', style="Warn.TLabel")
              elif jogMode == 3:
                jogMode = 1
                almStatusLab.config(text='JOGGING X & Y AXIS', style="Warn.TLabel")
                almStatusLab2.config(text='JOGGING X & Y AXIS', style="Warn.TLabel")
            ##CARTESIAN DIR JOG
            elif (mainMode == 2 and event.code == 'ABS_HAT0Y' and event.state == -1 and jogMode == 1): 
              XjogNeg(float(incrementEntryField.get()))    
            elif (mainMode == 2 and event.code == 'ABS_HAT0Y' and event.state == 1 and jogMode == 1): 
              XjogPos(float(incrementEntryField.get()))
            elif (mainMode == 2 and event.code == 'ABS_HAT0X' and event.state == 1 and jogMode == 1): 
              YjogNeg(float(incrementEntryField.get()))    
            elif (mainMode == 2 and event.code == 'ABS_HAT0X' and event.state == -1 and jogMode == 1): 
              YjogPos(float(incrementEntryField.get()))           
            elif (mainMode == 2 and event.code == 'ABS_HAT0Y' and event.state == 1 and jogMode == 2): 
              ZjogNeg(float(incrementEntryField.get()))    
            elif (mainMode == 2 and event.code == 'ABS_HAT0Y' and event.state == -1 and jogMode == 2): 
              ZjogPos(float(incrementEntryField.get()))                          
          ##CARTESIAN ORIENTATION MODE
            elif (event.code == 'BTN_EAST' and event.state == 1): 
              if mainMode != 3:
                mainMode = 3
                jogMode = 1
                almStatusLab.config(text='JOGGING Rx & Ry AXIS', style="Warn.TLabel")
                almStatusLab2.config(text='JOGGING Rx & Ry AXIS', style="Warn.TLabel")
              else:                
                jogMode +=1        
              if jogMode == 2:
                almStatusLab.config(text='JOGGING Rz AXIS', style="Warn.TLabel")
                almStatusLab2.config(text='JOGGING Rz AXIS', style="Warn.TLabel")
              elif jogMode == 3:
                jogMode = 1
                almStatusLab.config(text='JOGGING Rx & Ry AXIS', style="Warn.TLabel")
                almStatusLab2.config(text='JOGGING Rx & Ry AXIS', style="Warn.TLabel")
            ##CARTESIAN ORIENTATION JOG
            elif (mainMode == 3 and event.code == 'ABS_HAT0X' and event.state == -1 and jogMode == 1): 
              RxjogNeg(float(incrementEntryField.get()))    
            elif (mainMode == 3 and event.code == 'ABS_HAT0X' and event.state == 1 and jogMode == 1): 
              RxjogPos(float(incrementEntryField.get()))
            elif (mainMode == 3 and event.code == 'ABS_HAT0Y' and event.state == 1 and jogMode == 1): 
              RyjogNeg(float(incrementEntryField.get()))    
            elif (mainMode == 3 and event.code == 'ABS_HAT0Y' and event.state == -1 and jogMode == 1): 
              RyjogPos(float(incrementEntryField.get()))           
            elif (mainMode == 3 and event.code == 'ABS_HAT0X' and event.state == 1 and jogMode == 2): 
              RzjogNeg(float(incrementEntryField.get()))    
            elif (mainMode == 3 and event.code == 'ABS_HAT0X' and event.state == -1 and jogMode == 2): 
              RzjogPos(float(incrementEntryField.get()))
            ##J7 MODE
            elif (event.code == 'BTN_START' and event.state == 1): 
              mainMode = 4
              almStatusLab.config(text='JOGGING TRACK', style="Warn.TLabel")
              almStatusLab2.config(text='JOGGING TRACK', style="Warn.TLabel")
            ##TRACK JOG
            elif (mainMode == 4 and event.code == 'ABS_HAT0X' and event.state == 1): 
              J7jogPos(float(incrementEntryField.get()))    
            elif (mainMode == 4 and event.code == 'ABS_HAT0X' and event.state == -1): 
              J7jogNeg(float(incrementEntryField.get()))                   
            ##TEACH POS          
            elif (event.code == 'BTN_NORTH' and event.state == 1): 
              teachInsertBelSelected()
            ##GRIPPER         
            elif (event.code == 'BTN_SELECT' and event.state == 1): 
              if grip == 0:
                outputNum = DO1offEntryField.get()
                command = "OFX"+outputNum+"\n"
                if send_xbox_auxiliary(command):
                  grip = 1
              else:
                outputNum = DO1onEntryField.get()
                command = "ONX"+outputNum+"\n"
                if send_xbox_auxiliary(command):
                  grip = 0
                  time.sleep(.1)
            else:
              pass   
        except:
        #else:
          almStatusLab.config(text='XBOX CONTROLLER NOT RESPONDING', style="Alarm.TLabel")
          almStatusLab2.config(text='XBOX CONTROLLER NOT RESPONDING', style="Alarm.TLabel")        
    t = threading.Thread(target=threadxbox)
    t.start()

  def ChgDis(val):
    curSpd = int(incrementEntryField.get())
    if curSpd >=100 and val == 0:
      curSpd = 100 
    elif curSpd < 5 and val == 0:  
      curSpd += 1
    elif val == 0:
      curSpd += 5   
    if curSpd <=1 and val == 1:
      curSpd = 1 
    elif curSpd <= 5 and val == 1:  
      curSpd -= 1
    elif val == 1:
      curSpd -= 5
    elif val == 2:
      curSpd = 5  
    incrementEntryField.delete(0, 'end')
    incrementEntryField.insert(0,str(curSpd))

    time.sleep(.3)  

  def ChgSpd(val):
    curSpd = int(speedEntryField.get())
    if curSpd >=100 and val == 0:
      curSpd = 100 
    elif curSpd < 5 and val == 0:  
      curSpd += 1
    elif val == 0:
      curSpd += 5   
    if curSpd <=1 and val == 1:
      curSpd = 1 
    elif curSpd <= 5 and val == 1:  
      curSpd -= 1
    elif val == 1:
      curSpd -= 5
    elif val == 2:
      curSpd = 5  
    speedEntryField.delete(0, 'end')    
    speedEntryField.insert(0,str(curSpd))  


##end xbox ###################################################################################################################################################

def _configuration_number(key):
  try:
    raw_value = CAL[key]
  except (KeyError, TypeError) as exc:
    raise MotionInputError(f"invalid controller timing configuration {key!r}") from exc
  return finite_number(raw_value, f"controller timing configuration {key!r}")


def _runtime_number(key):
  try:
    raw_value = RUN[key]
  except (KeyError, TypeError) as exc:
    raise MotionInputError(f"invalid controller runtime value {key!r}") from exc
  return finite_number(raw_value, f"controller runtime value {key!r}")


def _configured_motion_timeout_bounds():
  axis_step_ranges = []
  for axis in range(1, 7):
    positive_limit = _configuration_number(f'J{axis}PosLim')
    negative_limit = _configuration_number(f'J{axis}NegLim')
    steps_per_degree = _configuration_number(f'J{axis}StepDeg')
    if positive_limit < 0 or negative_limit < 0 or steps_per_degree <= 0:
      raise MotionInputError(f"J{axis} motion timing configuration is out of range")
    axis_step_ranges.append(
      (positive_limit + negative_limit) * steps_per_degree
    )

  external_lengths = []
  for axis, length_key in ((7, 'J7PosLim'), (8, 'J8length'), (9, 'J9length')):
    length = _configuration_number(length_key)
    rotation = _configuration_number(f'J{axis}rotation')
    steps = _configuration_number(f'J{axis}steps')
    if length < 0 or rotation <= 0 or steps <= 0:
      raise MotionInputError(f"J{axis} motion timing configuration is out of range")
    external_lengths.append(length)
    axis_step_ranges.append(length * (steps / rotation))

  maximum_steps = max(axis_step_ranges)
  minimum_step_delay = _runtime_number('minSpeedDelay')
  if minimum_step_delay <= 0:
    raise MotionInputError("controller minimum step delay must be positive and finite")
  distribution_delay = (
    minimum_step_delay
    + FIRMWARE_DISTRIBUTION_DELAY_MICROSECONDS * FIRMWARE_AXIS_COUNT
  )
  minimum_ramp_full_scale_seconds = (
    maximum_steps * distribution_delay / 1_000_000.0
  )
  if (
    not math.isfinite(minimum_ramp_full_scale_seconds)
    or minimum_ramp_full_scale_seconds <= 0
  ):
    raise MotionInputError("configured full-scale motion duration is invalid")

  link_extent = sum(
    abs(_configuration_number(f'J{axis}{parameter}DHpar'))
    for axis in range(1, 7)
    for parameter in ('a', 'd')
  )
  tool_extent = math.sqrt(sum(
    _configuration_number(key) ** 2
    for key in ('TFx', 'TFy', 'TFz')
  ))
  maximum_radius = link_extent + tool_extent + sum(external_lengths)
  millimeter_motion_distance_bound = max(
    2.0 * math.pi * maximum_radius,
    minimum_ramp_full_scale_seconds * FIRMWARE_MAX_MILLIMETERS_PER_SECOND,
  )
  if (
    not math.isfinite(millimeter_motion_distance_bound)
    or millimeter_motion_distance_bound <= 0
  ):
    raise MotionInputError("configured Cartesian path bound is invalid")

  return minimum_ramp_full_scale_seconds, millimeter_motion_distance_bound


def _controller_response_timeout(command):
  timing = parse_command_timing(command)
  if timing is None:
    if command[:2] == "PG":
      raise MotionInputError(
        "G-code playback has no fixed response deadline"
      )
    return SERIAL_BASE_RESPONSE_TIMEOUT_SECONDS

  minimum_ramp_full_scale_seconds, millimeter_motion_distance_bound = (
    _configured_motion_timeout_bounds()
  )
  return command_response_timeout(
    command,
    SERIAL_BASE_RESPONSE_TIMEOUT_SECONDS,
    minimum_ramp_full_scale_seconds,
    millimeter_motion_distance_bound,
    margin_seconds=SERIAL_RESPONSE_MARGIN_SECONDS,
  )


def _canonicalize_main_serial_command(command):
  calibration = None
  if isinstance(command, str) and command[:2] in (
    "MG", "MJ", "ML", "MV", "RJ", "WC", "WG",
  ):
    calibration = _current_controller_joint_calibration()
  return canonicalize_serial_command(command, calibration)


def _exchange_serial_line(
  command,
  control_event=None,
  write_started_event=None,
):
  command = _canonicalize_main_serial_command(command)
  serial_port = RUN.get('ser')
  try:
    if command[:2] == "PG":
      if control_event is not None:
        raise MotionInputError(
          "G-code playback does not support live-jog control injection"
        )
      parse_command_timing(command)
      return exchange_serial_line_until_cancelled(
        serial_port,
        command,
        application_closing,
        write_lock=serial_write_lock,
        write_started_event=write_started_event,
      )
    response_timeout = _controller_response_timeout(command)
    return exchange_serial_line(
      serial_port,
      command,
      response_timeout,
      write_lock=serial_write_lock,
      control_event=control_event,
      control_command="S\n" if control_event is not None else None,
      control_ack_timeout_seconds=(
        SERIAL_LIVE_ACK_TIMEOUT_SECONDS
        if control_event is not None
        else None
      ),
      control_response_timeout_seconds=(
        response_timeout
        if control_event is not None
        else None
      ),
      write_started_event=write_started_event,
    )
  finally:
    if (
      RUN.get('ser') is serial_port
      and not getattr(serial_port, "is_open", False)
    ):
      RUN['ser'] = None


def _exchange_auxiliary_line(command):
  serial_port = RUN.get('ser2')
  try:
    return exchange_serial_line(
      serial_port,
      command,
      SERIAL_AUXILIARY_RESPONSE_TIMEOUT_SECONDS,
      write_lock=auxiliary_serial_write_lock,
    )
  finally:
    if (
      RUN.get('ser2') is serial_port
      and not getattr(serial_port, "is_open", False)
    ):
      RUN['ser2'] = None
      _clear_auxiliary_board_profile(serial_port)


def _raise_auxiliary_stop_acknowledgement_timeout():
  serial_port = RUN.get('ser2')
  if serial_port is not None:
    try:
      quarantine_serial_transport(
        serial_port,
        "auxiliary stop acknowledgement deadline expired",
      )
    except Exception:
      logger.exception("Unable to close auxiliary transport after stop timeout")
  raise SerialTransportTimeout(
    "auxiliary stop acknowledgement deadline expired"
  )


def _remaining_auxiliary_stop_acknowledgement_time(deadline):
  deadline = finite_number(deadline, "auxiliary stop acknowledgement deadline")
  remaining = deadline - time.monotonic()
  if remaining <= 0:
    _raise_auxiliary_stop_acknowledgement_timeout()
  return remaining


def _read_auxiliary_inactive_stop_response(deadline):
  serial_port = RUN.get('ser2')
  try:
    remaining = _remaining_auxiliary_stop_acknowledgement_time(deadline)
    return read_serial_line_response(
      serial_port,
      remaining,
      accepted_responses=(AUXILIARY_INACTIVE_STOP_RESPONSE,),
      response_deadline=deadline,
    )
  finally:
    if (
      RUN.get('ser2') is serial_port
      and not getattr(serial_port, "is_open", False)
    ):
      RUN['ser2'] = None
      _clear_auxiliary_board_profile(serial_port)


def _begin_auxiliary_stop_owner_wait():
  global auxiliary_stop_acknowledgement_deadline
  global auxiliary_stop_owner_waiting

  with auxiliary_stop_state_lock:
    if auxiliary_stop_owner_waiting:
      raise RuntimeError("auxiliary stop owner wait was already active")
    auxiliary_stop_injected_event.clear()
    auxiliary_stop_acknowledgement_deadline = None
    auxiliary_stop_owner_waiting = True


def _auxiliary_stop_acknowledgement_deadline_value():
  with auxiliary_stop_state_lock:
    deadline = auxiliary_stop_acknowledgement_deadline
  if deadline is None:
    raise ProtocolResponseError(
      "auxiliary stop acknowledgement deadline is unavailable"
    )
  return deadline


def _publish_auxiliary_stop_owner_result(succeeded, value):
  global auxiliary_stop_acknowledgement_deadline
  global auxiliary_stop_owner_result
  global auxiliary_stop_owner_waiting

  validation_error = None
  if not isinstance(succeeded, bool):
    validation_error = "auxiliary stop owner result must use a boolean status"
  elif not isinstance(value, str):
    validation_error = "auxiliary stop owner result must contain text"
  else:
    value = value.strip()
    if succeeded and value not in AUXILIARY_STOP_OWNER_RESPONSES:
      validation_error = "unexpected auxiliary wait terminal response"
    elif not value:
      value = "auxiliary wait owner ended without a terminal response"

  with auxiliary_stop_state_lock:
    if not auxiliary_stop_owner_waiting:
      raise RuntimeError("auxiliary stop owner wait was not active")
    auxiliary_stop_owner_waiting = False
    request_id = auxiliary_stop_active_request_id
    if request_id is not None:
      if auxiliary_stop_owner_result is not None:
        raise RuntimeError("auxiliary stop owner result was already published")
      if validation_error is not None:
        auxiliary_stop_owner_result = (request_id, False, validation_error)
      else:
        auxiliary_stop_owner_result = (request_id, succeeded, value)
      auxiliary_stop_owner_result_event.set()
    auxiliary_stop_injected_event.clear()
    auxiliary_stop_acknowledgement_deadline = None
  if validation_error is not None:
    raise ProtocolResponseError(validation_error)
  if request_id is None:
    return False
  return True


def _run_auxiliary_stop_safe(request_id, control_mode):
  global auxiliary_stop_acknowledgement_deadline
  global auxiliary_stop_active_request_id
  global auxiliary_stop_owner_result

  terminal_type = "completed"
  terminal_value = ""
  auxiliary_serial_event_queue.put(("started", request_id, "STOP\n"))
  try:
    if control_mode == SerialActivityRegistry.CONTROL_INJECT:
      write_serial_control(
        RUN.get('ser2'),
        "STOP\n",
        write_lock=auxiliary_serial_write_lock,
      )
      acknowledgement_deadline = (
        time.monotonic() + SERIAL_AUXILIARY_RESPONSE_TIMEOUT_SECONDS
      )
      with auxiliary_stop_state_lock:
        if auxiliary_stop_active_request_id != request_id:
          raise RuntimeError("auxiliary stop request ownership changed before wait")
        auxiliary_stop_acknowledgement_deadline = acknowledgement_deadline
      auxiliary_stop_injected_event.set()
      while True:
        remaining = acknowledgement_deadline - time.monotonic()
        if remaining <= 0:
          _raise_auxiliary_stop_acknowledgement_timeout()
        if auxiliary_stop_owner_result_event.wait(
          min(CONTROL_POLL_INTERVAL_SECONDS, remaining)
        ):
          break
        if application_closing.is_set():
          raise SerialActivityRejected(
            "application shutdown interrupted auxiliary stop acknowledgement"
          )
      with auxiliary_stop_state_lock:
        owner_result = auxiliary_stop_owner_result
        if (
          not isinstance(owner_result, tuple)
          or len(owner_result) != 3
          or owner_result[0] != request_id
          or not isinstance(owner_result[1], bool)
          or not isinstance(owner_result[2], str)
        ):
          raise RuntimeError("auxiliary stop owner result is invalid")
        _, owner_succeeded, terminal_value = owner_result
        auxiliary_stop_owner_result = None
        auxiliary_stop_owner_result_event.clear()
      if not owner_succeeded:
        raise RuntimeError(terminal_value)
      if terminal_value in AUXILIARY_WAIT_NATURAL_RESPONSES:
        remaining = _remaining_auxiliary_stop_acknowledgement_time(
          acknowledgement_deadline
        )
        acquired = auxiliary_serial_lock.acquire(
          timeout=remaining
        )
        if acquired is False:
          _raise_auxiliary_stop_acknowledgement_timeout()
        try:
          terminal_value = _read_auxiliary_inactive_stop_response(
            acknowledgement_deadline
          )
        finally:
          auxiliary_serial_lock.release()
      elif terminal_value not in (
        "Nano Stopped",
        AUXILIARY_INACTIVE_STOP_RESPONSE,
      ):
        raise ProtocolResponseError("unexpected injected auxiliary stop response")
    elif control_mode == SerialActivityRegistry.CONTROL_EXCLUSIVE:
      acquired = auxiliary_serial_lock.acquire()
      if acquired is False:
        raise RuntimeError("auxiliary stop transport acquisition failed")
      try:
        terminal_value = _exchange_auxiliary_line("STOP\n")
      finally:
        auxiliary_serial_lock.release()
      if terminal_value != AUXILIARY_INACTIVE_STOP_RESPONSE:
        raise ProtocolResponseError("unexpected inactive auxiliary stop response")
    else:
      raise RuntimeError("auxiliary stop received an invalid control mode")
  except Exception as exc:
    terminal_type = "failed"
    terminal_value = str(exc).strip() or "auxiliary stop failed without details"

  cleanup_errors = []
  try:
    serial_activity_registry.finish_control("ser2", control_mode)
  except Exception as exc:
    cleanup_errors.append(f"control ownership cleanup failed: {exc}")
  with auxiliary_stop_state_lock:
    if auxiliary_stop_active_request_id != request_id:
      cleanup_errors.append("auxiliary stop request ownership changed unexpectedly")
    else:
      auxiliary_stop_active_request_id = None
    if (
      isinstance(auxiliary_stop_owner_result, tuple)
      and auxiliary_stop_owner_result[0] == request_id
    ):
      auxiliary_stop_owner_result = None
      auxiliary_stop_owner_result_event.clear()
    if (
      not auxiliary_stop_owner_waiting
      or auxiliary_stop_acknowledgement_deadline is None
    ):
      auxiliary_stop_injected_event.clear()
      auxiliary_stop_acknowledgement_deadline = None
  if cleanup_errors:
    terminal_type = "failed"
    cleanup_message = "; ".join(cleanup_errors)
    terminal_value = (
      f"{terminal_value}; {cleanup_message}"
      if terminal_value
      else cleanup_message
    )
  auxiliary_serial_event_queue.put(
    (terminal_type, request_id, terminal_value)
  )


def _auxiliary_stop_not_required():
  if application_closing.is_set() or RUN['offlineMode']:
    return True
  configured_port = CAL.get('com2Port')
  configured_board = CAL.get('auxiliaryBoard')
  return (
    configured_port is None
    or configured_port == ""
    or configured_port == "None"
    or configured_board is None
    or configured_board == ""
    or configured_board == AUXILIARY_BOARD_NONE
  )


def _try_dispatch_auxiliary_stop():
  global auxiliary_stop_acknowledgement_deadline
  global auxiliary_stop_active_request_id
  global auxiliary_stop_owner_result
  global auxiliary_stop_pending_request_id

  with auxiliary_stop_state_lock:
    request_id = auxiliary_stop_pending_request_id
    if (
      request_id is None
      or auxiliary_stop_active_request_id is not None
      or auxiliary_stop_acknowledgement_deadline is not None
    ):
      return False
    if _auxiliary_stop_not_required():
      auxiliary_stop_pending_request_id = None
      auxiliary_stop_requested.clear()
      auxiliary_serial_event_queue.put(
        ("completed", request_id, "AUXILIARY STOP NOT REQUIRED")
      )
      return False
    control_mode = serial_activity_registry.reserve_control("ser2")
    if control_mode is None:
      return False
    if (
      control_mode == SerialActivityRegistry.CONTROL_INJECT
      and not auxiliary_stop_owner_waiting
    ):
      serial_activity_registry.finish_control("ser2", control_mode)
      return False
    auxiliary_stop_pending_request_id = None
    auxiliary_stop_active_request_id = request_id
    auxiliary_stop_owner_result = None
    auxiliary_stop_owner_result_event.clear()
    auxiliary_stop_injected_event.clear()
    auxiliary_stop_acknowledgement_deadline = None
    auxiliary_stop_requested.clear()

  try:
    thread = threading.Thread(
      target=_run_auxiliary_stop_safe,
      args=(request_id, control_mode),
      daemon=True,
    )
    thread.start()
  except Exception as exc:
    failure = f"unable to start auxiliary stop worker: {exc}"
    try:
      serial_activity_registry.finish_control("ser2", control_mode)
    except Exception as cleanup_exc:
      failure = f"{failure}; control ownership cleanup failed: {cleanup_exc}"
    with auxiliary_stop_state_lock:
      if auxiliary_stop_active_request_id == request_id:
        auxiliary_stop_active_request_id = None
      auxiliary_stop_owner_result = None
      auxiliary_stop_owner_result_event.clear()
      auxiliary_stop_injected_event.clear()
      auxiliary_stop_acknowledgement_deadline = None
    auxiliary_serial_event_queue.put(("failed", request_id, failure))
  return True


def _request_auxiliary_stop(on_reserved=None):
  global auxiliary_stop_next_request_id
  global auxiliary_stop_pending_request_id

  if on_reserved is not None and not callable(on_reserved):
    raise TypeError("auxiliary stop reservation callback must be callable")
  if _auxiliary_stop_not_required():
    return AUXILIARY_STOP_NOT_REQUIRED, None
  with auxiliary_stop_state_lock:
    if auxiliary_stop_active_request_id is not None:
      request_id = auxiliary_stop_active_request_id
      if on_reserved is not None:
        on_reserved(request_id)
      return AUXILIARY_STOP_DISPATCHED, request_id
    if auxiliary_stop_pending_request_id is None:
      request_id = auxiliary_stop_next_request_id + 1
      if on_reserved is not None:
        on_reserved(request_id)
      auxiliary_stop_next_request_id = request_id
      auxiliary_stop_pending_request_id = request_id
      auxiliary_stop_requested.set()
    else:
      request_id = auxiliary_stop_pending_request_id
      if on_reserved is not None:
        on_reserved(request_id)

  if _try_dispatch_auxiliary_stop():
    return AUXILIARY_STOP_DISPATCHED, request_id
  with auxiliary_stop_state_lock:
    if auxiliary_stop_active_request_id == request_id:
      return AUXILIARY_STOP_DISPATCHED, request_id
    if auxiliary_stop_pending_request_id == request_id:
      return AUXILIARY_STOP_PENDING, request_id
  return AUXILIARY_STOP_NOT_REQUIRED, None


def start_send_serial_thread(
  command,
  live_jog=False,
  completion_callback=None,
  controller_recovery=False,
  write_started_event=None,
):
  if completion_callback is not None and not callable(completion_callback):
    raise TypeError("completion_callback must be callable")
  if not isinstance(controller_recovery, bool):
    raise TypeError("controller_recovery must be boolean")
  if write_started_event is not None and not all(
    callable(getattr(write_started_event, method, None))
    for method in ("set", "is_set")
  ):
    raise TypeError("write_started_event must satisfy the event contract")
  if application_closing.is_set():
    logger.warning("Serial command rejected during application shutdown")
    return False
  if controller_correction_requested.is_set() and not controller_recovery:
    logger.warning("Serial command rejected until controller correction completes")
    return False
  inherited_transport = _transfer_main_serial_reservation()
  if not inherited_transport and not serial_lock.acquire(blocking=False):
    logger.warning("Serial command already in progress; command rejected")
    return False
  try:
    activity_lease = serial_activity_registry.lease("ser")
  except SerialActivityRejected as exc:
    if inherited_transport:
      _restore_main_serial_reservation()
    else:
      serial_lock.release()
    logger.warning("Serial command rejected: %s", exc)
    return False
  except Exception:
    if inherited_transport:
      _restore_main_serial_reservation()
    else:
      serial_lock.release()
    raise
  legacy_serial_result_pending.set()
  if live_jog:
    live_jog_stop_requested.clear()
    live_serial_result_pending.set()

  try:
    thread = threading.Thread(
      target=run_send_serial_safe,
      args=(
        command,
        live_jog,
        completion_callback,
        activity_lease,
        write_started_event,
      ),
      daemon=True,
    )
    thread.start()
  except Exception:
    legacy_serial_result_pending.clear()
    if live_jog:
      live_serial_result_pending.clear()
      live_jog_stop_requested.clear()
    if inherited_transport:
      _restore_main_serial_reservation()
    else:
      serial_lock.release()
    activity_lease.close()
    raise
  return True


def _try_dispatch_controller_correction():
  with controller_correction_state_lock:
    if not controller_correction_requested.is_set():
      return False
    if application_closing.is_set() or RUN['offlineMode']:
      return False
    serial_port = RUN.get('ser')
    if (
      serial_port is None
      or not getattr(serial_port, "is_open", False)
      or serial_transport_quarantined(serial_port)
    ):
      return False
    if serial_lock.locked() or motion_request_registry.active:
      return False
    request_lease = _acquire_motion_request(
      "Controller correction",
      allow_position_recovery=True,
      requires_kinematics=False,
    )
    if request_lease is None:
      return False
    try:
      started = start_send_serial_thread(
        "CP\n",
        completion_callback=lambda controller_position: _complete_controller_correction(
          request_lease,
          controller_position,
        ),
        controller_recovery=True,
      )
    except Exception:
      _finish_motion_request(request_lease)
      logger.exception("Unable to start controller correction")
      return False
    if not started:
      _finish_motion_request(request_lease)
      return False
    return True


def _complete_controller_correction(request_lease, controller_position):
  if controller_position is not None and not isinstance(
    controller_position,
    PositionResponse,
  ):
    raise RuntimeError(
      "controller correction returned an invalid position result"
    )
  try:
    with controller_correction_state_lock:
      if (
        controller_position is not None
        and not controller_position_resynchronization_required.is_set()
      ):
        controller_correction_requested.clear()
      else:
        controller_correction_requested.set()
  finally:
    _finish_motion_request(request_lease)


def _request_controller_correction():
  if application_closing.is_set() or RUN['offlineMode']:
    return False
  _clear_deferred_joint_adjustments()
  with controller_correction_state_lock:
    controller_correction_requested.set()
  return _try_dispatch_controller_correction()


def run_send_serial_safe(
  command,
  live_jog=False,
  completion_callback=None,
  activity_lease=None,
  write_started_event=None,
):
  serial_event_queue.put(
    (
      "started",
      command,
      None,
      None,
      live_jog,
      completion_callback,
      activity_lease,
    )
  )
  try:
    response = _exchange_serial_line(
      command,
      control_event=live_jog_stop_requested if live_jog else None,
      write_started_event=write_started_event,
    )
    serial_event_queue.put(
      (
        "completed",
        command,
        response,
        None,
        live_jog,
        completion_callback,
        activity_lease,
      )
    )
  except Exception as exc:
    serial_event_queue.put(
      (
        "failed",
        command,
        None,
        str(exc),
        live_jog,
        completion_callback,
        activity_lease,
      )
    )


def _invalidate_joint_motion_state(reason):
  if not isinstance(reason, str) or not reason.strip():
    reason = "controller state became uncertain"
  else:
    reason = reason.strip()
  controller_position_resynchronization_required.set()
  pending_discarded = joint_motion_dispatcher.invalidate(reason)
  deferred_discarded = deferred_joint_adjustments.pending
  _clear_deferred_joint_adjustments()
  if pending_discarded or deferred_discarded:
    logger.warning("Pending joint target discarded because controller state is unknown")


def _apply_legacy_serial_response(response):
  if not isinstance(response, str):
    reason = "legacy serial exchange returned a non-text response"
    _invalidate_joint_motion_state(reason)
    raise ProtocolResponseError(reason)
  if response.startswith('E'):
    ErrorHandler(response)
    _invalidate_joint_motion_state(f"controller rejected motion: {response}")
    return None

  try:
    parsed = parse_position_response(response)
  except ProtocolResponseError:
    displayPosition(response)
    return None

  applied_position = displayPosition(response, parsed=parsed)
  if applied_position is None:
    return None
  if parsed.flag:
    return None
  return applied_position


def _poll_serial_events():
  try:
    while True:
      try:
        event = serial_event_queue.get_nowait()
      except Empty:
        break
      if not isinstance(event, tuple) or len(event) != 7:
        raise RuntimeError("serial worker emitted an invalid event")
      (
        event_type,
        command,
        response,
        error,
        live_jog,
        completion_callback,
        activity_lease,
      ) = event
      if completion_callback is not None and not callable(completion_callback):
        raise RuntimeError("serial worker emitted an invalid completion callback")
      if activity_lease is not None and not callable(
        getattr(activity_lease, "close", None)
      ):
        raise RuntimeError("serial worker emitted an invalid activity lease")

      if event_type == "started":
        cmdSentEntryField.delete(0, 'end')
        cmdSentEntryField.insert(0, command)
        if live_jog:
          almStatusLab.config(text="LIVE JOG IN PROGRESS", style="OK.TLabel")
          almStatusLab2.config(text="LIVE JOG IN PROGRESS", style="OK.TLabel")
        continue

      if event_type not in ("completed", "failed"):
        raise RuntimeError(f"serial worker emitted an unknown event: {event_type!r}")

      applied_position = None
      try:
        if event_type == "failed":
          message = f"Serial command failed: {error}"
          logger.error(message)
          almStatusLab.config(text=message, style="Alarm.TLabel")
          almStatusLab2.config(text=message, style="Alarm.TLabel")
          _invalidate_joint_motion_state(message)
        else:
          applied_position = _apply_legacy_serial_response(response)
          if applied_position is not None and not isinstance(
            applied_position,
            PositionResponse,
          ):
            raise RuntimeError(
              "serial response application returned an invalid position result"
            )
          if applied_position is not None and live_jog:
            RUN['VR_angles'] = [
              finite_number(CAL[f'J{axis}AngCur'], f"J{axis} angle")
              for axis in range(1, 7)
            ]
            setStepMonitorsVR()
            if not applied_position.speed_violation:
              almStatusLab.config(text="SYSTEM READY", style="OK.TLabel")
              almStatusLab2.config(text="SYSTEM READY", style="OK.TLabel")
      except Exception as exc:
        message = f"Unable to apply serial worker result: {exc}"
        logger.exception(message)
        _invalidate_joint_motion_state(message)
      finally:
        if live_jog:
          RUN['liveJog'] = False
        legacy_serial_result_pending.clear()
        live_serial_result_pending.clear()
        live_jog_stop_requested.clear()
        try:
          if serial_lock.locked():
            serial_lock.release()
          else:
            logger.error("Serial result arrived without transport ownership")
        finally:
          if activity_lease is not None and activity_lease.close() is not True:
            logger.error("Serial result reused a released activity lease")

        if completion_callback is not None:
          try:
            completion_callback(applied_position)
          except Exception:
            logger.exception("Unable to apply a serial completion callback")
        if (
          applied_position is not None
          and applied_position.speed_violation
        ):
          if deferred_joint_adjustments.pending:
            logger.warning(
              "Deferred joint target discarded after a controller speed violation"
            )
          _clear_deferred_joint_adjustments()
          deferred_dispatched = False
        else:
          deferred_dispatched = _try_dispatch_deferred_joint_adjustments(
            allow_current_generation=True,
          )
        if (
          applied_position is not None
          and not applied_position.speed_violation
          and not deferred_dispatched
          and not joint_motion_dispatcher.active
          and not motion_request_registry.active
          and not application_closing.is_set()
        ):
          almStatusLab.config(text="SYSTEM READY", style="OK.TLabel")
          almStatusLab2.config(text="SYSTEM READY", style="OK.TLabel")
  except Exception:
    logger.exception("Unable to apply a serial worker result on the Tk event thread")
  finally:
    if not application_closing.is_set():
      root.after(25, _poll_serial_events)


def _poll_auxiliary_serial_events():
  try:
    while True:
      try:
        event = auxiliary_serial_event_queue.get_nowait()
      except Empty:
        break
      if (
        not isinstance(event, tuple)
        or len(event) != 3
        or event[0] not in ("started", "completed", "failed")
        or isinstance(event[1], bool)
        or not isinstance(event[1], int)
        or event[1] <= 0
        or not isinstance(event[2], str)
        or not event[2]
      ):
        raise RuntimeError("auxiliary serial worker emitted an invalid event")

      event_type, request_id, value = event
      with program_stop_state_lock:
        current_request_id = RUN.get('programStopRequestId')
        if event_type == "started":
          if current_request_id == request_id:
            _set_program_stop_status("pending")
        elif event_type == "completed":
          if current_request_id == request_id:
            RUN['programStopRequestId'] = None
            manual_auxiliary_stop_barrier.clear()
            _set_program_stop_status("completed")
        else:
          message = f"Auxiliary stop failed: {value}"
          logger.error(message)
          if current_request_id == request_id:
            RUN['programStopRequestId'] = None
            manual_auxiliary_stop_barrier.clear()
            _set_program_stop_status("failed")
      if event_type == "started":
        cmdSentEntryField.delete(0, 'end')
        cmdSentEntryField.insert(0, value)
      elif event_type == "completed":
        cmdRecEntryField.delete(0, 'end')
        cmdRecEntryField.insert(0, value)
  except Exception:
    logger.exception("Unable to apply an auxiliary serial result on the Tk event thread")
  finally:
    try:
      _apply_program_stop_status_events()
    except Exception:
      logger.exception(
        "Unable to apply a program-stop status on the Tk event thread"
      )
    _try_dispatch_controller_correction()
    _try_dispatch_auxiliary_stop()
    _ensure_startup_auxiliary_cleanup()
    if not application_closing.is_set():
      try:
        root.after(25, _poll_auxiliary_serial_events)
      except (RuntimeError, tk.TclError):
        logger.exception(
          "Unable to schedule auxiliary serial result polling"
        )


def _poll_manual_auxiliary_events():
  global manual_auxiliary_active_request

  try:
    while True:
      try:
        result = manual_auxiliary_event_queue.get_nowait()
      except Empty:
        break
      if not isinstance(result, ManualAuxiliaryResult):
        raise RuntimeError("manual auxiliary worker emitted an invalid result")
      with manual_auxiliary_state_lock:
        request = manual_auxiliary_active_request
        if (
          not isinstance(request, ManualAuxiliaryRequest)
          or request.request_id != result.request_id
        ):
          raise RuntimeError(
            "manual auxiliary worker result ownership is invalid"
          )
        manual_auxiliary_active_request = None

      if result.outcome == "completed":
        expected_response = request.expected_response.decode("ascii")
        if result.value != expected_response:
          raise RuntimeError(
            "manual auxiliary worker result response is invalid"
          )
        cmdRecEntryField.delete(0, 'end')
        cmdRecEntryField.insert(0, result.value)
        CAL[request.calibration_key] = request.calibration_value
        if _retain_calibration_persistence_retry():
          _set_manual_auxiliary_status(
            "AUXILIARY COMMAND COMPLETE",
            "OK.TLabel",
          )
        else:
          _set_manual_auxiliary_status(
            "AUXILIARY COMMAND COMPLETE; SETTINGS SAVE DEFERRED",
            "Warn.TLabel",
          )
      else:
        with manual_auxiliary_state_lock:
          discarded = len(manual_auxiliary_request_queue)
          manual_auxiliary_request_queue.clear()
        message = f"Manual auxiliary command failed: {result.value}"
        if discarded:
          message = (
            f"{message}; {discarded} queued command(s) discarded"
          )
        logger.error(message)
        _set_manual_auxiliary_feedback(message)
        _set_manual_auxiliary_status(message, "Alarm.TLabel")
  except Exception as exc:
    logger.exception(
      "Unable to apply a manual auxiliary result on the Tk event thread"
    )
    with manual_auxiliary_state_lock:
      manual_auxiliary_active_request = None
      discarded = len(manual_auxiliary_request_queue)
      manual_auxiliary_request_queue.clear()
    _close_serial_port('ser2', "invalid manual auxiliary worker result")
    detail = _manual_auxiliary_error_detail(
      exc,
      "manual auxiliary result handling failed",
    )
    message = f"Manual auxiliary command failed: {detail}"
    if discarded:
      message = f"{message}; {discarded} queued command(s) discarded"
    logger.error(message)
    _set_manual_auxiliary_feedback(message)
    _set_manual_auxiliary_status(
      message,
      "Alarm.TLabel",
    )
  finally:
    if application_closing.is_set():
      with manual_auxiliary_state_lock:
        discarded = len(manual_auxiliary_request_queue)
        manual_auxiliary_request_queue.clear()
      if discarded:
        logger.warning(
          "Discarded queued manual auxiliary commands during shutdown"
        )
    else:
      try:
        if _manual_auxiliary_program_active():
          _reject_queued_manual_auxiliary_requests(
            "AUXILIARY COMMANDS REJECTED WHILE PROGRAM IS RUNNING"
          )
        elif (
          serial_activity_registry.active("ser2")
          or auxiliary_serial_lock.locked()
        ):
          _reject_queued_manual_auxiliary_requests(
            "AUXILIARY COMMANDS REJECTED WHILE TRANSPORT IS BUSY"
          )
        else:
          _try_dispatch_manual_auxiliary_request()
      except Exception:
        logger.exception(
          "Unable to dispatch a queued manual auxiliary command"
        )
      try:
        root.after(25, _poll_manual_auxiliary_events)
      except (RuntimeError, tk.TclError):
        logger.exception(
          "Unable to schedule manual auxiliary result polling"
        )


def _poll_xbox_auxiliary_events():
  try:
    while True:
      try:
        event = xbox_auxiliary_event_queue.get_nowait()
      except Empty:
        break
      if (
        not isinstance(event, tuple)
        or len(event) != 5
        or event[0] not in ("completed", "rejected", "failed")
        or isinstance(event[1], bool)
        or not isinstance(event[1], int)
        or event[1] <= 0
        or event[2] not in XBOX_AUXILIARY_PENDING_KEYS
        or not isinstance(event[3], bool)
        or not isinstance(event[4], str)
        or not event[4].strip()
      ):
        raise RuntimeError("Xbox auxiliary worker emitted an invalid event")

      event_type, request_id, state_name, target_state, value = event
      pending_name = XBOX_AUXILIARY_PENDING_KEYS[state_name]
      if RUN.get(pending_name) != request_id:
        logger.warning(
          "Ignoring a stale Xbox auxiliary result for %s",
          state_name,
        )
        continue
      current_state = RUN.get(state_name)
      if not isinstance(current_state, bool) or target_state is current_state:
        raise RuntimeError("Xbox auxiliary result conflicts with confirmed state")
      value = value.strip()
      if event_type == "completed":
        expected_response = XBOX_AUXILIARY_TOGGLE_RESPONSES[state_name]
        if value != expected_response.decode("ascii"):
          raise RuntimeError("Xbox auxiliary acknowledgement contract changed")

      RUN[pending_name] = None
      if event_type == "completed":
        RUN[state_name] = target_state
        continue

      if event_type == "failed":
        RUN[state_name] = None
        message = f"Xbox auxiliary command failed: {value}"
        style = "Alarm.TLabel"
        logger.error(message)
      else:
        message = f"Xbox auxiliary command rejected: {value}"
        style = "Warn.TLabel"
        logger.warning(message)
      almStatusLab.config(text=message, style=style)
      almStatusLab2.config(text=message, style=style)
  except Exception:
    logger.exception("Unable to apply an Xbox auxiliary result on the Tk event thread")
  finally:
    if not application_closing.is_set():
      root.after(25, _poll_xbox_auxiliary_events)


def _poll_virtual_motion_events():
  try:
    while True:
      try:
        event = virtual_motion_event_queue.get_nowait()
      except Empty:
        break
      if not isinstance(event, tuple) or len(event) != 2:
        raise RuntimeError("virtual motion worker emitted an invalid event")
      event_type, value = event
      if event_type == "error" and isinstance(value, str):
        ErrorHandler(value)
      elif (
        event_type == "program-completion"
        and isinstance(value, tuple)
        and len(value) == 4
        and callable(value[0])
        and isinstance(value[1], MotionRequestLease)
        and isinstance(value[2], bool)
        and (value[3] is None or callable(value[3]))
      ):
        (
          completion_callback,
          request_lease,
          succeeded,
          settlement_callback,
        ) = value
        _finish_settled_motion_request(
          completion_callback,
          request_lease,
          succeeded,
          settlement_callback,
        )
      elif (
        event_type == "offline-live-terminal"
        and isinstance(value, VirtualMotionOperation)
      ):
        _settle_offline_live_jog(value)
      elif event_type == "motion-released" and value is None:
        _try_dispatch_controller_correction()
        _try_dispatch_deferred_joint_adjustments(
          allow_current_generation=True,
        )
      else:
        raise RuntimeError("virtual motion worker emitted an invalid event")
  except Exception:
    logger.exception("Unable to apply a virtual-motion result on the Tk event thread")
  finally:
    if not application_closing.is_set():
      root.after(25, _poll_virtual_motion_events)


def _current_joint_positions():
  return _controller_joint_positions_from_values(CAL)


def _controller_joint_positions_from_values(values):
  return tuple(
    finite_number(values[key], label)
    for key, label in (
      ('J1AngCur', 'J1 current angle'),
      ('J2AngCur', 'J2 current angle'),
      ('J3AngCur', 'J3 current angle'),
      ('J4AngCur', 'J4 current angle'),
      ('J5AngCur', 'J5 current angle'),
      ('J6AngCur', 'J6 current angle'),
      ('J7PosCur', 'J7 current position'),
      ('J8PosCur', 'J8 current position'),
      ('J9PosCur', 'J9 current position'),
    )
  )


def _controller_joint_calibration_from_values(values):
  negative_limits = tuple(
    finite_number(values[f'J{axis}NegLim'], f'J{axis} negative limit')
    for axis in range(1, 7)
  ) + (0.0, 0.0, 0.0)
  positive_limits = tuple(
    finite_number(values[f'J{axis}PosLim'], f'J{axis} positive limit')
    for axis in range(1, 7)
  ) + tuple(
    finite_number(values[key], key)
    for key in ('J7PosLim', 'J8length', 'J9length')
  )
  steps_per_unit = [
    finite_number(values[f'J{axis}StepDeg'], f'J{axis} steps per degree')
    for axis in range(1, 7)
  ]
  for axis in range(7, 10):
    rotation = finite_number(values[f'J{axis}rotation'], f'J{axis} rotation')
    steps = finite_number(values[f'J{axis}steps'], f'J{axis} steps')
    if rotation <= 0:
      raise MotionInputError(f"J{axis} rotation must be positive")
    steps_per_unit.append(
      controller_ratio(steps, rotation, f"J{axis} steps per unit")
    )
  return ControllerJointCalibration(
    negative_limits=negative_limits,
    positive_limits=positive_limits,
    steps_per_unit=tuple(steps_per_unit),
  )


def _validate_controller_pose(values):
  calibration = _controller_joint_calibration_from_values(values)
  return calibration.validate_positions(
    _controller_joint_positions_from_values(values)
  )


def _current_controller_joint_calibration():
  return _controller_joint_calibration_from_values(CAL)


def _current_joint_motion_profile():
  checkSpeedVals()
  speed_type = speedOption.get()
  if speed_type == "mm per Sec":
    speedOption.set("Percent")
    speedEntryField.delete(0, 'end')
    speedEntryField.insert(0, "50")
    speed_prefix = "Sp"
  elif speed_type == "Seconds":
    speed_prefix = "Ss"
  elif speed_type == "Percent":
    speed_prefix = "Sp"
  else:
    raise MotionInputError(f"Unsupported joint speed type: {speed_type!r}")

  wrist_config = RUN.get('WC')
  if wrist_config not in ("N", "F"):
    wrist_config = (
      "F" if finite_number(CAL['J5AngCur'], "J5 angle") > 0 else "N"
    )
    RUN['WC'] = wrist_config

  loop_mode = ''.join(
    str(CAL[f'J{axis}OpenLoopVal'].get())
    for axis in range(1, 7)
  )
  return MotionProfile(
    speed_prefix=speed_prefix,
    speed=speedEntryField.get(),
    acceleration=ACCspeedField.get(),
    deceleration=DECspeedField.get(),
    ramp=ACCrampField.get(),
    wrist_config=wrist_config,
    loop_mode=loop_mode,
  )


def _exchange_joint_motion(command):
  return _exchange_serial_line(command)


joint_motion_dispatcher = CoalescingJointDispatcher(
  _exchange_joint_motion,
  _current_controller_joint_calibration,
  transport_lock=serial_lock,
  activity_factory=lambda: serial_activity_registry.lease("ser"),
)
confirmed_position_generation = 0
deferred_joint_adjustments = DeferredJointAdjustments()


def _reserve_joint_motion_request():
  global joint_motion_request_lease

  with joint_motion_request_lock:
    if joint_motion_request_lease is not None:
      if not motion_request_registry.owns(joint_motion_request_lease):
        raise RuntimeError("joint dispatcher lost motion request ownership")
      return joint_motion_request_lease, False
    request_lease = _acquire_motion_request("Joint target dispatcher")
    if request_lease is None:
      return None, False
    joint_motion_request_lease = request_lease
    return request_lease, True


def _abandon_joint_motion_request(request_lease):
  global joint_motion_request_lease

  with joint_motion_request_lock:
    if joint_motion_request_lease is not request_lease:
      raise RuntimeError("joint dispatcher request cleanup is stale")
    if joint_motion_dispatcher.active:
      raise RuntimeError("active joint dispatcher request cannot be abandoned")
    joint_motion_request_lease = None
  if request_lease.close() is not True:
    raise RuntimeError("joint dispatcher request was already released")
  return True


def _finish_joint_motion_request_if_idle():
  global joint_motion_request_lease

  with joint_motion_request_lock:
    request_lease = joint_motion_request_lease
    if request_lease is None or joint_motion_dispatcher.active:
      return False
    joint_motion_request_lease = None
  _finish_motion_request(request_lease)
  return True


def _clear_deferred_joint_adjustments():
  deferred_joint_adjustments.clear()


def _defer_joint_adjustment(axis, delta, profile):
  return deferred_joint_adjustments.add(
    axis,
    delta,
    profile,
    confirmed_position_generation,
  )


def _defer_joint_target(axis, target, profile):
  return deferred_joint_adjustments.set_target(
    axis,
    target,
    profile,
    confirmed_position_generation,
  )


def _try_dispatch_deferred_joint_adjustments(allow_current_generation=False):
  if application_closing.is_set():
    return False
  if not deferred_joint_adjustments.pending:
    return False
  if (
    controller_correction_requested.is_set()
    or legacy_serial_result_pending.is_set()
    or serial_lock.locked()
    or joint_motion_dispatcher.active
    or motion_request_registry.active
  ):
    return False
  if not deferred_joint_adjustments.ready(
    confirmed_position_generation,
    allow_current_generation=allow_current_generation,
  ):
    return False

  request_lease = None
  lease_created = False
  try:
    actual_positions = _current_joint_positions()

    def submit_deferred(target_positions, profile):
      nonlocal request_lease, lease_created
      request_lease, lease_created = _reserve_joint_motion_request()
      if request_lease is None:
        raise MotionTransportBusy(_motion_request_rejection_message(
          "another motion request is active"
        ))
      return joint_motion_dispatcher.submit_positions(
        target_positions,
        actual_positions,
        profile,
      )

    submission = deferred_joint_adjustments.consume(
      actual_positions,
      confirmed_position_generation,
      submit_deferred,
      allow_current_generation=allow_current_generation,
    )
  except MotionTransportBusy:
    if lease_created and not joint_motion_dispatcher.active:
      _abandon_joint_motion_request(request_lease)
    return False
  except (KeyError, TypeError, ValueError, MotionInputError, MotionQueueFault) as exc:
    if (
      lease_created
      and request_lease is not None
      and not joint_motion_dispatcher.active
    ):
      _abandon_joint_motion_request(request_lease)
    _clear_deferred_joint_adjustments()
    message = f"Deferred joint jog rejected: {exc}"
    logger.error(message)
    almStatusLab.config(text=message, style="Alarm.TLabel")
    almStatusLab2.config(text=message, style="Alarm.TLabel")
    return False

  status = "JOINT TARGET QUEUED" if submission.coalesced else "JOINT MOVE IN PROGRESS"
  almStatusLab.config(text=status, style="OK.TLabel")
  almStatusLab2.config(text=status, style="OK.TLabel")
  _try_set_virtual_joint_target(submission.target)
  return True


def _set_virtual_joint_target(target_positions):
  try:
    values = tuple(target_positions)
    if len(values) < 6:
      raise ValueError("joint target contains fewer than six robot axes")
    joints = [
      finite_number(value, f"virtual joint {axis}")
      for axis, value in enumerate(values[:6], start=1)
    ]
  except (TypeError, ValueError) as exc:
    raise MotionInputError(f"invalid virtual joint target: {exc}") from exc
  if not all(math.isfinite(value) for value in joints):
    raise MotionInputError("virtual joint target must contain finite values")
  RUN['VR_angles'] = joints
  setStepMonitorsVR()


def _try_set_virtual_joint_target(target_positions):
  try:
    _set_virtual_joint_target(target_positions)
  except (KeyError, TypeError, ValueError, MotionInputError) as exc:
    message = f"Virtual model update failed; controller state remains authoritative: {exc}"
    logger.exception(message)
    almStatusLab.config(text=message, style="Warn.TLabel")
    almStatusLab2.config(text=message, style="Warn.TLabel")
    return False
  return True


def _set_virtual_from_joint_result(position):
  target = joint_motion_dispatcher.desired_target
  if target is None:
    target = position.joints
  return _try_set_virtual_joint_target(target)


def _start_offline_joint_motion(command):
  request_lease = _acquire_motion_request("Offline joint motion")
  if request_lease is None:
    return False

  try:
    saved_virtual_pose = _validated_virtual_six_vector(
      RUN['VR_angles'],
      "offline joint starting pose",
    )
  except (KeyError, TypeError, ValueError, MotionInputError) as exc:
    _finish_motion_request(request_lease)
    message = f"Offline virtual joint motion rejected: {exc}"
    logger.error(message)
    almStatusLab.config(text=message, style="Alarm.TLabel")
    almStatusLab2.config(text=message, style="Alarm.TLabel")
    return False

  def reconcile_offline_joint(succeeded):
    target_pose = (
      tuple(RUN['VR_angles'])
      if succeeded
      else saved_virtual_pose
    )
    try:
      if refresh_gui_from_joint_angles(target_pose) is not True:
        raise MotionQueueFault("virtual GUI rejected offline joint settlement")
    except (KeyError, TypeError, ValueError, MotionInputError, MotionQueueFault) as exc:
      logger.error("Unable to reconcile offline joint motion: %s", exc)
      return False
    return succeeded

  try:
    virtual_completion_timeout = _virtual_completion_timeout(command)
    operation = rj_command(command)
  except Exception:
    _finish_settled_motion_request(
      None,
      request_lease,
      False,
      reconcile_offline_joint,
    )
    raise
  if not isinstance(operation, VirtualMotionOperation):
    _finish_settled_motion_request(
      None,
      request_lease,
      False,
      reconcile_offline_joint,
    )
    return False

  def complete_offline_joint(succeeded):
    if succeeded:
      almStatusLab.config(text="SYSTEM READY", style="OK.TLabel")
      almStatusLab2.config(text="SYSTEM READY", style="OK.TLabel")
      return
    message = "Offline virtual joint motion failed"
    logger.error(message)
    almStatusLab.config(text=message, style="Alarm.TLabel")
    almStatusLab2.config(text=message, style="Alarm.TLabel")

  _complete_program_motion_when_virtual_idle(
    complete_offline_joint,
    request_lease,
    operation,
    virtual_completion_timeout,
    settlement_callback=reconcile_offline_joint,
  )
  return True


def _poll_joint_motion_events():
  try:
    for event in joint_motion_dispatcher.drain_events():
      try:
        try:
          if event.kind == "started":
            cmdSentEntryField.delete(0, 'end')
            cmdSentEntryField.insert(0, event.move.command)
            almStatusLab.config(text="JOINT MOVE IN PROGRESS", style="OK.TLabel")
            almStatusLab2.config(text="JOINT MOVE IN PROGRESS", style="OK.TLabel")
            continue

          if event.kind == "completed":
            applied_position = displayPosition(
              event.response,
              parsed=event.position,
              synchronize_dispatcher=False,
            )
            if applied_position is not None and not isinstance(
              applied_position,
              PositionResponse,
            ):
              raise RuntimeError(
                "joint response application returned an invalid position result"
            )
            if applied_position is not None:
              if applied_position.speed_violation:
                pending_discarded = (
                  joint_motion_dispatcher.discard_pending_after_completion(
                    applied_position.joints + applied_position.external
                  )
                )
                deferred_discarded = deferred_joint_adjustments.pending
                _clear_deferred_joint_adjustments()
                if pending_discarded or deferred_discarded:
                  logger.warning(
                    "Pending joint target discarded after a controller speed violation"
                  )
              if _set_virtual_from_joint_result(event.position) is not True:
                _invalidate_joint_motion_state(
                  "completed joint motion could not update the virtual model"
                )
                continue
              if applied_position.speed_violation:
                continue
              status = (
                "JOINT TARGET QUEUED"
                if joint_motion_dispatcher.pending
                else "SYSTEM READY"
              )
              almStatusLab.config(text=status, style="OK.TLabel")
              almStatusLab2.config(text=status, style="OK.TLabel")
            continue

          if event.position is not None:
            applied_position = displayPosition(
              event.response,
              parsed=event.position,
              synchronize_dispatcher=False,
            )
            if applied_position is not None and not isinstance(
              applied_position,
              PositionResponse,
            ):
              raise RuntimeError(
                "joint response application returned an invalid position result"
              )
            if applied_position is not None:
              _set_virtual_from_joint_result(event.position)
          elif event.response is not None and event.response.startswith('E'):
            _try_set_virtual_joint_target(_current_joint_positions())
            _invalidate_joint_motion_state(
              f"controller rejected joint motion: {event.response}"
            )
            ErrorHandler(event.response)
          else:
            _try_set_virtual_joint_target(_current_joint_positions())
            message = f"Joint motion failed: {event.error}"
            _invalidate_joint_motion_state(message)
            logger.error(message)
            almStatusLab.config(text=message, style="Alarm.TLabel")
            almStatusLab2.config(text=message, style="Alarm.TLabel")
          if event.pending_discarded:
            logger.warning("Pending joint target discarded because controller state is unknown")
        except Exception as exc:
          message = f"Unable to apply joint-motion result: {exc}"
          logger.exception(message)
          _invalidate_joint_motion_state(message)
          almStatusLab.config(text=message, style="Alarm.TLabel")
          almStatusLab2.config(text=message, style="Alarm.TLabel")
      finally:
        event.acknowledge()

    if not _finish_joint_motion_request_if_idle():
      _try_dispatch_deferred_joint_adjustments()
  finally:
    if not application_closing.is_set():
      root.after(25, _poll_joint_motion_events)


def _queue_joint_motion(axis, value, absolute):
  try:
    if application_closing.is_set():
      raise MotionQueueFault("joint motion is unavailable during application shutdown")
    if controller_correction_requested.is_set():
      raise MotionQueueFault("joint motion is unavailable during controller correction")
    if RUN['xboxUse'] != 1:
      almStatusLab.config(text="SYSTEM READY", style="OK.TLabel")
      almStatusLab2.config(text="SYSTEM READY", style="OK.TLabel")

    if isinstance(axis, bool) or not isinstance(axis, int) or not 0 <= axis < 9:
      raise MotionInputError("joint axis must be an integer in [0, 8]")
    normalized_value = finite_number(value, "joint input")
    if not absolute and normalized_value == 0:
      raise MotionInputError("joint adjustment must be non-zero")

    profile = _current_joint_motion_profile()
    deferred = False
    submission = None

    if RUN['offlineMode']:
      if axis >= 6:
        raise MotionInputError(
          "offline virtual motion supports J1-J6 only; J7-J9 require a controller"
        )
      virtual_positions = [
        finite_number(position, f"virtual joint {virtual_axis}")
        for virtual_axis, position in enumerate(RUN['VR_angles'], start=1)
      ]
      if absolute:
        virtual_positions[axis] = normalized_value
      else:
        virtual_positions[axis] += normalized_value
      if not _start_offline_joint_motion(
        build_virtual_joint_command(virtual_positions, profile)
      ):
        raise MotionQueueFault(_motion_request_rejection_message(
          "offline virtual joint motion did not start"
        ))
      coalesced = False
    elif (
      deferred_joint_adjustments.pending
      or (
        motion_request_registry.active
        and not joint_motion_dispatcher.active
      )
    ):
      deferred = True
      if absolute:
        coalesced = _defer_joint_target(axis, normalized_value, profile)
      else:
        coalesced = _defer_joint_adjustment(axis, normalized_value, profile)
    else:
      actual_positions = _current_joint_positions()
      request_lease = None
      lease_created = False
      try:
        request_lease, lease_created = _reserve_joint_motion_request()
        if request_lease is None:
          raise MotionTransportBusy(_motion_request_rejection_message(
            "another motion request is active"
          ))
        if absolute:
          submission = joint_motion_dispatcher.submit_target(
            axis,
            normalized_value,
            actual_positions,
            profile,
          )
        else:
          submission = joint_motion_dispatcher.submit_delta(
            axis,
            normalized_value,
            actual_positions,
            profile,
          )
        coalesced = submission.coalesced
      except MotionTransportBusy:
        if lease_created and not joint_motion_dispatcher.active:
          _abandon_joint_motion_request(request_lease)
        if (
          not legacy_serial_result_pending.is_set()
          and not motion_request_registry.active
        ):
          raise MotionQueueFault(
            "controller transport is busy outside the legacy motion queue"
          )
        deferred = True
        if absolute:
          coalesced = _defer_joint_target(axis, normalized_value, profile)
        else:
          coalesced = _defer_joint_adjustment(axis, normalized_value, profile)
      except Exception:
        if lease_created and not joint_motion_dispatcher.active:
          _abandon_joint_motion_request(request_lease)
        raise

    if not RUN['offlineMode']:
      if deferred and not coalesced:
        if live_serial_result_pending.is_set():
          status = "LIVE JOG IN PROGRESS"
        elif (
          legacy_serial_result_pending.is_set()
          or serial_lock.locked()
          or joint_motion_dispatcher.active
        ):
          status = "CONTROLLER MOVE IN PROGRESS"
        else:
          status = "SYSTEM READY"
      else:
        status = "JOINT TARGET QUEUED" if coalesced else "JOINT MOVE IN PROGRESS"
      almStatusLab.config(text=status, style="OK.TLabel")
      almStatusLab2.config(text=status, style="OK.TLabel")
      if submission is not None:
        _try_set_virtual_joint_target(submission.target)
    return True
  except (KeyError, TypeError, ValueError, MotionInputError, MotionQueueFault) as exc:
    message = f"Joint motion rejected: {exc}"
    logger.error(message)
    almStatusLab.config(text=message, style="Alarm.TLabel")
    almStatusLab2.config(text=message, style="Alarm.TLabel")
    return False


def _queue_joint_jog(axis, delta):
  return _queue_joint_motion(axis, delta, absolute=False)


def _queue_joint_target(axis, target):
  return _queue_joint_motion(axis, target, absolute=True)


PRIMARY_JOINT_COUNT = 6


def _primary_joint_position_key(axis):
  if (
    isinstance(axis, bool)
    or not isinstance(axis, int)
    or not 0 <= axis < PRIMARY_JOINT_COUNT
  ):
    raise ValueError(
      "primary joint axis must be an integer in "
      f"[0, {PRIMARY_JOINT_COUNT - 1}]"
    )
  return f"J{axis + 1}AngCur"


def _restore_joint_entry_value(axis, entry):
  position_key = _primary_joint_position_key(axis)
  return _reset_joint_position_entry(entry, CAL[position_key])


def _submit_joint_entry_target(axis, entry):
  _primary_joint_position_key(axis)
  accepted = _queue_joint_target(axis, entry.get())
  entry._joint_target_editing = not accepted
  entry._joint_target_replace_on_key = accepted
  entry._joint_target_pointer_focus = False
  return accepted


def _bind_joint_target_entry(axis, entry):
  _primary_joint_position_key(axis)
  entry._joint_target_editing = False
  entry._joint_target_replace_on_key = True
  entry._joint_target_pointer_focus = False
  non_editing_keys = frozenset((
    "Tab", "ISO_Left_Tab",
    "Shift_L", "Shift_R", "Control_L", "Control_R",
    "Alt_L", "Alt_R", "Caps_Lock", "Num_Lock",
  ))

  def begin_focus_edit(_event):
    pointer_focus = entry._joint_target_pointer_focus
    entry._joint_target_pointer_focus = False
    entry._joint_target_editing = True
    entry._joint_target_replace_on_key = not pointer_focus
    if not pointer_focus:
      entry.selection_range(0, 'end')

  def begin_pointer_edit(_event):
    entry._joint_target_pointer_focus = True
    entry._joint_target_editing = True
    entry._joint_target_replace_on_key = False

  def begin_key_edit(event):
    if getattr(event, "keysym", "") in non_editing_keys:
      return
    entry._joint_target_pointer_focus = False
    if entry._joint_target_replace_on_key:
      entry.selection_range(0, 'end')
    entry._joint_target_editing = True
    entry._joint_target_replace_on_key = False

  def submit_target(_event):
    _submit_joint_entry_target(axis, entry)
    entry.selection_range(0, 'end')
    return "break"

  def cancel_edit(_event):
    _restore_joint_entry_value(axis, entry)
    entry.selection_range(0, 'end')
    return "break"

  def restore_on_blur(_event):
    _restore_joint_entry_value(axis, entry)

  entry.bind("<FocusIn>", begin_focus_edit)
  entry.bind("<Button-1>", begin_pointer_edit)
  entry.bind("<KeyPress>", begin_key_edit)
  entry.bind("<Return>", submit_target)
  entry.bind("<KP_Enter>", submit_target)
  entry.bind("<Escape>", cancel_edit)
  entry.bind("<FocusOut>", restore_on_blur)


def J1jogNeg(value):
  _queue_joint_jog(0, -value)

def J1jogPos(value):
  _queue_joint_jog(0, value)

def J2jogNeg(value):
  _queue_joint_jog(1, -value)

def J2jogPos(value):
  _queue_joint_jog(1, value)

def J3jogNeg(value):
  _queue_joint_jog(2, -value)

def J3jogPos(value):
  _queue_joint_jog(2, value)

def J4jogNeg(value):
  _queue_joint_jog(3, -value)

def J4jogPos(value):
  _queue_joint_jog(3, value)

def J5jogNeg(value):
  _queue_joint_jog(4, -value)

def J5jogPos(value):
  _queue_joint_jog(4, value)

def J6jogNeg(value):
  _queue_joint_jog(5, -value)

def J6jogPos(value):
  _queue_joint_jog(5, value)




def J7jogNeg(value):
  _queue_joint_jog(6, -value)

def J7jogPos(value):
  _queue_joint_jog(6, value)



def J8jogNeg(value):
  _queue_joint_jog(7, -value)



def J8jogPos(value):
  _queue_joint_jog(7, value)


def J9jogNeg(value):
  _queue_joint_jog(8, -value)



def J9jogPos(value):
  _queue_joint_jog(8, value)


def _live_jog_start_is_blocked():
  offline_operation = None
  if application_closing.is_set():
    message = "Live jog rejected during application shutdown"
  else:
    with offline_live_jog_state_lock:
      offline_operation = offline_live_jog_operation
    if offline_operation is not None:
      message = "An offline live jog is already in progress"
    elif live_serial_result_pending.is_set():
      message = "A controller live jog is already in progress"
    elif motion_request_registry.active:
      message = (
        "Live jog rejected while "
        f"{motion_request_registry.active_name} owns motion"
      )
    else:
      return False
  logger.warning(message)
  almStatusLab.config(text=message, style="Warn.TLabel")
  almStatusLab2.config(text=message, style="Warn.TLabel")
  return True


def _live_jog_parameters(default_percent):
  checkSpeedVals()
  speed_type = speedOption.get()
  if speed_type in ("mm per Sec", "Seconds"):
    speedOption.set("Percent")
    speedEntryField.delete(0, 'end')
    speedEntryField.insert(0, str(default_percent))
    speed_type = "Percent"

  if speed_type == "Percent":
    speed_prefix = "Sp"
  else:
    raise MotionInputError(f"Unsupported live-jog speed type: {speed_type!r}")

  loop_mode = ''.join(
    str(CAL[f'J{axis}OpenLoopVal'].get())
    for axis in range(1, 7)
  )
  profile = MotionProfile(
    speed_prefix=speed_prefix,
    speed=speedEntryField.get(),
    acceleration=ACCspeedField.get(),
    deceleration=DECspeedField.get(),
    ramp=ACCrampField.get(),
    wrist_config="N",
    loop_mode=loop_mode,
  )
  return (
    profile.speed_prefix,
    controller_protocol_decimal(profile.speed, "live-jog speed"),
    controller_protocol_decimal(
      profile.acceleration,
      "live-jog acceleration",
    ),
    controller_protocol_decimal(
      profile.deceleration,
      "live-jog deceleration",
    ),
    controller_protocol_decimal(profile.ramp, "live-jog ramp"),
    profile.loop_mode,
  )


LIVE_CARTESIAN_VECTORS = frozenset((10, 11, 20, 21, 30, 31, 40, 41, 50, 51, 60, 61))
LIVE_JOINT_VECTORS = LIVE_CARTESIAN_VECTORS | frozenset((70, 71, 80, 81, 90, 91))


def _prepare_live_jog(value, allowed_vectors, default_percent):
  try:
    if isinstance(value, bool):
      raise MotionInputError("live-jog vector must be numeric")
    vector = finite_number(value, "live-jog vector")
    if vector not in allowed_vectors:
      raise MotionInputError(f"unsupported live-jog vector: {value!r}")
    parameters = _live_jog_parameters(default_percent)
  except (KeyError, TypeError, ValueError, tk.TclError) as exc:
    message = f"Live jog rejected: {exc}"
    logger.error(message)
    almStatusLab.config(text=message, style="Alarm.TLabel")
    almStatusLab2.config(text=message, style="Alarm.TLabel")
    return None
  return (
    controller_protocol_decimal(vector, "live-jog vector"),
  ) + parameters


def _dispatch_live_jog(command, motion_name):
  if RUN['offlineMode']:
    return True
  request_lease = _acquire_motion_request(motion_name)
  if request_lease is None:
    message = _motion_request_rejection_message(
      f"Controller busy; {motion_name} not started"
    )
    almStatusLab.config(text=message, style="Warn.TLabel")
    almStatusLab2.config(text=message, style="Warn.TLabel")
    return False
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0, command)
  try:
    started = start_send_serial_thread(
      command,
      live_jog=True,
      completion_callback=lambda controller_position: _finish_motion_request(
        request_lease,
      ),
    )
  except Exception:
    if motion_request_registry.owns(request_lease):
      _finish_motion_request(request_lease)
    raise
  if started:
    RUN['liveJog'] = True
    return True

  _finish_motion_request(request_lease)
  if not live_serial_result_pending.is_set():
    RUN['liveJog'] = False
  message = f"Controller busy; {motion_name} not started"
  almStatusLab.config(text=message, style="Warn.TLabel")
  almStatusLab2.config(text=message, style="Warn.TLabel")
  return False



def _settle_offline_live_jog(operation):
  global offline_live_jog_operation, offline_live_jog_motion_lease
  global offline_live_jog_pose_snapshot

  if not operation.completed:
    raise RuntimeError("offline live-jog terminal event arrived before completion")
  with offline_live_jog_state_lock:
    if offline_live_jog_operation is not operation:
      raise RuntimeError("offline live-jog terminal event is stale")
    request_lease = offline_live_jog_motion_lease
    if not motion_request_registry.owns(request_lease):
      raise RuntimeError("offline live-jog motion ownership is missing")
    pose_snapshot = offline_live_jog_pose_snapshot
    if not isinstance(pose_snapshot, tuple) or len(pose_snapshot) != 6:
      raise RuntimeError("offline live-jog starting pose is missing")
  succeeded, error = operation.result()
  refresh_succeeded = True
  try:
    target_pose = tuple(RUN['VR_angles']) if succeeded else pose_snapshot
    refresh_gui_from_joint_angles(target_pose)
  except Exception:
    refresh_succeeded = False
    logger.exception("Unable to settle offline live-jog GUI state")
  finally:
    with offline_live_jog_state_lock:
      if offline_live_jog_operation is operation:
        offline_live_jog_operation = None
        offline_live_jog_motion_lease = None
        offline_live_jog_pose_snapshot = None
    RUN['liveJog'] = False
    _finish_motion_request(request_lease)
  if not succeeded or not refresh_succeeded:
    message = error or "offline live-jog GUI settlement failed"
    message = f"Offline live jog failed: {message}"
    logger.error(message)
    almStatusLab.config(text=message, style="Alarm.TLabel")
    almStatusLab2.config(text=message, style="Alarm.TLabel")
    return False
  return True


def _start_offline_live_jog(mode_lock, target, args):
  global offline_live_jog_operation, offline_live_jog_stop_event
  global offline_live_jog_motion_lease, offline_live_jog_pose_snapshot

  request_lease = _acquire_motion_request("Offline live jog")
  if request_lease is None:
    return False
  try:
    pose_snapshot = _validated_virtual_six_vector(
      RUN['VR_angles'],
      "offline live-jog starting pose",
    )
  except (KeyError, TypeError, ValueError, MotionInputError) as exc:
    _finish_motion_request(request_lease)
    logger.error("Offline live jog rejected: %s", exc)
    return False
  if not offline_live_jog_lock.acquire(blocking=False):
    _finish_motion_request(request_lease)
    logger.warning("Offline live jog already in progress; request rejected")
    return False
  if not mode_lock.acquire(blocking=False):
    offline_live_jog_lock.release()
    _finish_motion_request(request_lease)
    logger.warning("Offline live-jog mode already in progress; request rejected")
    return False

  stop_event = threading.Event()
  operation = VirtualMotionOperation()
  with offline_live_jog_state_lock:
    if application_closing.is_set() or offline_live_jog_operation is not None:
      mode_lock.release()
      offline_live_jog_lock.release()
      _finish_motion_request(request_lease)
      if application_closing.is_set():
        logger.warning("Offline live jog rejected during application shutdown")
      else:
        logger.warning("Offline live-jog settlement remains pending; request rejected")
      return False
    offline_live_jog_stop_event.set()
    offline_live_jog_stop_event = stop_event
    offline_live_jog_operation = operation
    offline_live_jog_motion_lease = request_lease
    offline_live_jog_pose_snapshot = pose_snapshot
  RUN['liveJog'] = True

  def thread_wrapper():
    succeeded = False
    error = None
    try:
      succeeded = target(*args, stop_event) is True
      if not succeeded:
        error = "offline live-jog worker reported failure"
    except BaseException as exc:
      detail = str(exc).strip() or type(exc).__name__
      error = f"{type(exc).__name__}: {detail}"
      logger.exception("Offline live-jog worker failed")
    finally:
      stop_event.set()
      mode_lock.release()
      offline_live_jog_lock.release()
    if succeeded:
      operation.complete(True)
    else:
      operation.complete(False, error)
    virtual_motion_event_queue.put(("offline-live-terminal", operation))

  try:
    thread = threading.Thread(target=thread_wrapper, daemon=True)
    thread.start()
  except Exception:
    stop_event.set()
    RUN['liveJog'] = False
    with offline_live_jog_state_lock:
      if offline_live_jog_operation is operation:
        offline_live_jog_operation = None
        offline_live_jog_motion_lease = None
        offline_live_jog_pose_snapshot = None
    mode_lock.release()
    offline_live_jog_lock.release()
    _finish_motion_request(request_lease)
    raise
  return operation


def start_live_joint_jog_thread(command):
  return _start_offline_live_jog(
    live_jog_lock,
    live_joint_jog,
    (command,),
  )


def LiveJointJog(value):
  if _live_jog_start_is_blocked():
    return False
  almStatusLab.config(text="SYSTEM READY", style="OK.TLabel")
  almStatusLab2.config(text="SYSTEM READY", style="OK.TLabel")
  prepared = _prepare_live_jog(
    value,
    LIVE_JOINT_VECTORS,
    25,
  )
  if prepared is None:
    return False
  vector, speed_prefix, speed, acceleration, deceleration, ramp, loop_mode = prepared
  if RUN['offlineMode'] and finite_number(vector, "live-jog vector") >= 70:
    message = "Offline live motion supports J1-J6 only; J7-J9 require a controller"
    logger.error(message)
    almStatusLab.config(text=message, style="Alarm.TLabel")
    almStatusLab2.config(text=message, style="Alarm.TLabel")
    return False
  command = (
    f"LJV{vector}{speed_prefix}{speed}Ac{acceleration}"
    f"Dc{deceleration}Rm{ramp}WALm{loop_mode}\n"
  )
  if RUN['offlineMode'] and not start_live_joint_jog_thread(command):
    message = _motion_request_rejection_message(
      "Offline live joint jog not started; another live jog is active"
    )
    logger.warning(message)
    almStatusLab.config(text=message, style="Warn.TLabel")
    almStatusLab2.config(text=message, style="Warn.TLabel")
    return False
  return _dispatch_live_jog(command, "live joint jog")


def start_live_cartesian_jog_thread(command):
  return _start_offline_live_jog(
    live_cartesian_lock,
    live_cartesian_jog,
    (command,),
  )


def LiveCarJog(value):
  if _live_jog_start_is_blocked():
    return False
  try:
    wrist_config = _validated_wrist_config(RUN.get('WC', 'A'))
  except MotionInputError as exc:
    message = f"Live Cartesian jog rejected: {exc}"
    logger.error(message)
    almStatusLab.config(text=message, style="Alarm.TLabel")
    almStatusLab2.config(text=message, style="Alarm.TLabel")
    return False
  almStatusLab.config(text="SYSTEM READY", style="OK.TLabel")
  almStatusLab2.config(text="SYSTEM READY", style="OK.TLabel")
  prepared = _prepare_live_jog(
    value,
    LIVE_CARTESIAN_VECTORS,
    25,
  )
  if prepared is None:
    return False
  vector, speed_prefix, speed, acceleration, deceleration, ramp, loop_mode = prepared
  command = (
    f"LCV{vector}{speed_prefix}{speed}Ac{acceleration}"
    f"Dc{deceleration}Rm{ramp}W{wrist_config}Lm{loop_mode}\n"
  )
  if RUN['offlineMode'] and not start_live_cartesian_jog_thread(command):
    message = _motion_request_rejection_message(
      "Offline live Cartesian jog not started; another live jog is active"
    )
    logger.warning(message)
    almStatusLab.config(text=message, style="Warn.TLabel")
    almStatusLab2.config(text=message, style="Warn.TLabel")
    return False
  return _dispatch_live_jog(command, "live Cartesian jog")


def start_live_tool_jog_thread(command, original_tool_frame):
  return _start_offline_live_jog(
    live_tool_lock,
    live_tool_jog,
    (command, original_tool_frame),
  )


def LiveToolJog(value):
  if _live_jog_start_is_blocked():
    return False
  try:
    wrist_config = _validated_wrist_config(RUN.get('WC', 'A'))
  except MotionInputError as exc:
    message = f"Live tool jog rejected: {exc}"
    logger.error(message)
    almStatusLab.config(text=message, style="Alarm.TLabel")
    almStatusLab2.config(text=message, style="Alarm.TLabel")
    return False
  almStatusLab.config(text="SYSTEM READY", style="OK.TLabel")
  almStatusLab2.config(text="SYSTEM READY", style="OK.TLabel")
  prepared = _prepare_live_jog(
    value,
    LIVE_CARTESIAN_VECTORS,
    50,
  )
  if prepared is None:
    return False
  vector, speed_prefix, speed, acceleration, deceleration, ramp, loop_mode = prepared
  try:
    original_tool_frame = _active_tool_frame()
  except (KeyError, TypeError, ValueError, MotionInputError) as exc:
    message = f"Live tool jog rejected: {exc}"
    logger.error(message)
    almStatusLab.config(text=message, style="Alarm.TLabel")
    almStatusLab2.config(text=message, style="Alarm.TLabel")
    return False
  command = (
    f"LTV{vector}{speed_prefix}{speed}Ac{acceleration}"
    f"Dc{deceleration}Rm{ramp}W{wrist_config}Lm{loop_mode}\n"
  )
  if RUN['offlineMode'] and not start_live_tool_jog_thread(
    command,
    original_tool_frame,
  ):
    message = _motion_request_rejection_message(
      "Offline live tool jog not started; another live jog is active"
    )
    logger.warning(message)
    almStatusLab.config(text=message, style="Warn.TLabel")
    almStatusLab2.config(text=message, style="Warn.TLabel")
    return False
  return _dispatch_live_jog(command, "live tool jog")

def StopJog(self):
  with offline_live_jog_state_lock:
    offline_operation = offline_live_jog_operation
    if offline_operation is not None:
      offline_live_jog_stop_event.set()
  if offline_operation is not None:
    almStatusLab.config(text="OFFLINE LIVE JOG STOP REQUESTED", style="Warn.TLabel")
    almStatusLab2.config(text="OFFLINE LIVE JOG STOP REQUESTED", style="Warn.TLabel")
    return
  if not RUN['liveJog'] and not live_serial_result_pending.is_set():
    return
  RUN['liveJog'] = False
  if live_serial_result_pending.is_set():
    live_jog_stop_requested.set()
    almStatusLab.config(text="LIVE JOG STOP REQUESTED", style="Warn.TLabel")
    almStatusLab2.config(text="LIVE JOG STOP REQUESTED", style="Warn.TLabel")
  else:
    message = "Live jog has no active controller request"
    logger.warning(message)
    almStatusLab.config(text=message, style="Warn.TLabel")
    almStatusLab2.config(text=message, style="Warn.TLabel")




















@_manual_motion_request("Cartesian jog")
def XjogNeg(value):
  # global RUN['xboxUse']
  # global WC, RUN['VR_angles']
  checkSpeedVals()
  if RUN['xboxUse'] != 1:
    almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel")
  speedtype = speedOption.get()
  #mm/sec
  if(speedtype == "mm per Sec"):
    speedPrefix = "Sm" 
  #seconds
  if(speedtype == "Seconds"):
    speedPrefix = "Ss"
  #percent
  if(speedtype == "Percent"):
    speedPrefix = "Sp"   
  Speed = speedEntryField.get() 
  ACCspd = ACCspeedField.get()
  DECspd = DECspeedField.get()
  ACCramp = ACCrampField.get()
  RUN['xVal'] = str(float(CAL['XcurPos']) - value)
  RUN['yVal'] = CAL['YcurPos']
  RUN['zVal'] = CAL['ZcurPos']
  rzVal = CAL['RzcurPos']
  ryVal = CAL['RycurPos']
  rxVal = CAL['RxcurPos']
  j7Val = str(CAL['J7PosCur'])
  j8Val = str(CAL['J8PosCur'])
  j9Val = str(CAL['J9PosCur'])
  LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
  if not RUN['offlineMode']:
    command = "MJ"+"X"+RUN['xVal']+"Y"+RUN['yVal']+"Z"+RUN['zVal']+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"J7"+j7Val+"J8"+j8Val+"J9"+j9Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
    commandVR = "MJ"+"X"+RUN['xVal']+"Y"+RUN['yVal']+"Z"+RUN['zVal']+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    return _start_manual_motion(
      command,
      "Cartesian jog",
      mj_command,
      commandVR,
    )
  else:
    xyzuvw = _forward_kinematics_display_pose(
      RUN['VR_angles'],
      "offline Cartesian jog pose",
    )
    CAL['XcurPos'], CAL['YcurPos'], CAL['ZcurPos'], CAL['RzcurPos'], CAL['RycurPos'], CAL['RxcurPos'] = [round(v, 3) for v in xyzuvw]
    CAL['XcurPos'] = CAL['XcurPos'] - value
    commandVR = (
        f"MJX{CAL['XcurPos']:.3f}Y{CAL['YcurPos']:.3f}Z{CAL['ZcurPos']:.3f}"
        f"Rz{CAL['RzcurPos']:.3f}Ry{CAL['RycurPos']:.3f}Rx{CAL['RxcurPos']:.3f}"
        f"{speedPrefix}{Speed}Ac{ACCspd}Dc{DECspd}Rm{ACCramp}"
        f"W{RUN['WC']}Lm{LoopMode}\n"
    )
    return _start_manual_motion(
      None,
      "Cartesian jog",
      mj_command,
      commandVR,
    )



  

@_manual_motion_request("Cartesian jog")
def YjogNeg(value):
  # global RUN['xboxUse']
  # global WC, RUN['VR_angles']
  checkSpeedVals()
  if RUN['xboxUse'] != 1:
    almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel")
  speedtype = speedOption.get()
  #mm/sec
  if(speedtype == "mm per Sec"):
    speedPrefix = "Sm" 
  #seconds
  if(speedtype == "Seconds"):
    speedPrefix = "Ss"
  #percent
  if(speedtype == "Percent"):
    speedPrefix = "Sp"   
  Speed = speedEntryField.get() 
  ACCspd = ACCspeedField.get()
  DECspd = DECspeedField.get()
  ACCramp = ACCrampField.get()
  RUN['xVal'] = CAL['XcurPos']
  RUN['yVal'] = str(float(CAL['YcurPos']) - value)
  RUN['zVal'] = CAL['ZcurPos']
  rzVal = CAL['RzcurPos']
  ryVal = CAL['RycurPos']
  rxVal = CAL['RxcurPos']
  j7Val = str(CAL['J7PosCur'])
  j8Val = str(CAL['J8PosCur'])
  j9Val = str(CAL['J9PosCur'])
  LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
  if not RUN['offlineMode']:
    command = "MJ"+"X"+RUN['xVal']+"Y"+RUN['yVal']+"Z"+RUN['zVal']+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"J7"+j7Val+"J8"+j8Val+"J9"+j9Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
    commandVR = "MJ"+"X"+RUN['xVal']+"Y"+RUN['yVal']+"Z"+RUN['zVal']+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    return _start_manual_motion(
      command,
      "Cartesian jog",
      mj_command,
      commandVR,
    )
  else:
    xyzuvw = _forward_kinematics_display_pose(
      RUN['VR_angles'],
      "offline Cartesian jog pose",
    )
    CAL['XcurPos'], CAL['YcurPos'], CAL['ZcurPos'], CAL['RzcurPos'], CAL['RycurPos'], CAL['RxcurPos'] = [round(v, 3) for v in xyzuvw]
    CAL['YcurPos'] = CAL['YcurPos'] - value
    commandVR = (
        f"MJX{CAL['XcurPos']:.3f}Y{CAL['YcurPos']:.3f}Z{CAL['ZcurPos']:.3f}"
        f"Rz{CAL['RzcurPos']:.3f}Ry{CAL['RycurPos']:.3f}Rx{CAL['RxcurPos']:.3f}"
        f"{speedPrefix}{Speed}Ac{ACCspd}Dc{DECspd}Rm{ACCramp}"
        f"W{RUN['WC']}Lm{LoopMode}\n"
    )
    return _start_manual_motion(
      None,
      "Cartesian jog",
      mj_command,
      commandVR,
    )





@_manual_motion_request("Cartesian jog")
def ZjogNeg(value):
  # global RUN['xboxUse']
  # global WC, RUN['VR_angles']
  checkSpeedVals()
  if RUN['xboxUse'] != 1:
    almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel")
  speedtype = speedOption.get()
  #mm/sec
  if(speedtype == "mm per Sec"):
    speedPrefix = "Sm" 
  #seconds
  if(speedtype == "Seconds"):
    speedPrefix = "Ss"
  #percent
  if(speedtype == "Percent"):
    speedPrefix = "Sp"   
  Speed = speedEntryField.get() 
  ACCspd = ACCspeedField.get()
  DECspd = DECspeedField.get()
  ACCramp = ACCrampField.get()
  RUN['xVal'] = CAL['XcurPos']
  RUN['yVal'] = CAL['YcurPos']
  RUN['zVal'] = str(float(CAL['ZcurPos']) - value)
  rzVal = CAL['RzcurPos']
  ryVal = CAL['RycurPos']
  rxVal = CAL['RxcurPos']
  j7Val = str(CAL['J7PosCur'])
  j8Val = str(CAL['J8PosCur'])
  j9Val = str(CAL['J9PosCur'])
  LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
  if not RUN['offlineMode']:
    command = "MJ"+"X"+RUN['xVal']+"Y"+RUN['yVal']+"Z"+RUN['zVal']+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"J7"+j7Val+"J8"+j8Val+"J9"+j9Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
    commandVR = "MJ"+"X"+RUN['xVal']+"Y"+RUN['yVal']+"Z"+RUN['zVal']+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    return _start_manual_motion(
      command,
      "Cartesian jog",
      mj_command,
      commandVR,
    )
  else:
    xyzuvw = _forward_kinematics_display_pose(
      RUN['VR_angles'],
      "offline Cartesian jog pose",
    )
    CAL['XcurPos'], CAL['YcurPos'], CAL['ZcurPos'], CAL['RzcurPos'], CAL['RycurPos'], CAL['RxcurPos'] = [round(v, 3) for v in xyzuvw]
    CAL['ZcurPos'] = CAL['ZcurPos'] - value
    commandVR = (
        f"MJX{CAL['XcurPos']:.3f}Y{CAL['YcurPos']:.3f}Z{CAL['ZcurPos']:.3f}"
        f"Rz{CAL['RzcurPos']:.3f}Ry{CAL['RycurPos']:.3f}Rx{CAL['RxcurPos']:.3f}"
        f"{speedPrefix}{Speed}Ac{ACCspd}Dc{DECspd}Rm{ACCramp}"
        f"W{RUN['WC']}Lm{LoopMode}\n"
    )
    return _start_manual_motion(
      None,
      "Cartesian jog",
      mj_command,
      commandVR,
    )

@_manual_motion_request("Cartesian jog")
def RxjogNeg(value):
  # global RUN['xboxUse']
  # global WC, RUN['VR_angles']
  checkSpeedVals()
  if RUN['xboxUse'] != 1:
    almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel")
  speedtype = speedOption.get()
  #mm/sec
  if(speedtype == "mm per Sec"):
    speedPrefix = "Sm" 
  #seconds
  if(speedtype == "Seconds"):
    speedPrefix = "Ss"
  #percent
  if(speedtype == "Percent"):
    speedPrefix = "Sp"   
  Speed = speedEntryField.get() 
  ACCspd = ACCspeedField.get()
  DECspd = DECspeedField.get()
  ACCramp = ACCrampField.get()
  RUN['xVal'] = CAL['XcurPos']
  RUN['yVal'] = CAL['YcurPos']
  RUN['zVal'] = CAL['ZcurPos']
  rzVal = CAL['RzcurPos']
  ryVal = CAL['RycurPos']
  rxVal =  str(float(CAL['RxcurPos']) - value)
  j7Val = str(CAL['J7PosCur'])
  j8Val = str(CAL['J8PosCur'])
  j9Val = str(CAL['J9PosCur'])
  LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
  if not RUN['offlineMode']:
    command = "MJ"+"X"+RUN['xVal']+"Y"+RUN['yVal']+"Z"+RUN['zVal']+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"J7"+j7Val+"J8"+j8Val+"J9"+j9Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
    commandVR = "MJ"+"X"+RUN['xVal']+"Y"+RUN['yVal']+"Z"+RUN['zVal']+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    return _start_manual_motion(
      command,
      "Cartesian jog",
      mj_command,
      commandVR,
    )
  else:
    xyzuvw = _forward_kinematics_display_pose(
      RUN['VR_angles'],
      "offline Cartesian jog pose",
    )
    CAL['XcurPos'], CAL['YcurPos'], CAL['ZcurPos'], CAL['RzcurPos'], CAL['RycurPos'], CAL['RxcurPos'] = [round(v, 3) for v in xyzuvw]
    CAL['RxcurPos'] = CAL['RxcurPos'] - value
    commandVR = (
        f"MJX{CAL['XcurPos']:.3f}Y{CAL['YcurPos']:.3f}Z{CAL['ZcurPos']:.3f}"
        f"Rz{CAL['RzcurPos']:.3f}Ry{CAL['RycurPos']:.3f}Rx{CAL['RxcurPos']:.3f}"
        f"{speedPrefix}{Speed}Ac{ACCspd}Dc{DECspd}Rm{ACCramp}"
        f"W{RUN['WC']}Lm{LoopMode}\n"
    )
    return _start_manual_motion(
      None,
      "Cartesian jog",
      mj_command,
      commandVR,
    )

@_manual_motion_request("Cartesian jog")
def RyjogNeg(value):
  # global RUN['xboxUse']
  # global XcurPos, YcurPos, ZcurPos, RzcurPos, RycurPos, RxcurPos
  # global WC, RUN['VR_angles']
  checkSpeedVals()
  if RUN['xboxUse'] != 1:
    almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel")
  speedtype = speedOption.get()
  #mm/sec
  if(speedtype == "mm per Sec"):
    speedPrefix = "Sm" 
  #seconds
  if(speedtype == "Seconds"):
    speedPrefix = "Ss"
  #percent
  if(speedtype == "Percent"):
    speedPrefix = "Sp"   
  Speed = speedEntryField.get() 
  ACCspd = ACCspeedField.get()
  DECspd = DECspeedField.get()
  ACCramp = ACCrampField.get()
  RUN['xVal'] = CAL['XcurPos']
  RUN['yVal'] = CAL['YcurPos']
  RUN['zVal'] = CAL['ZcurPos']
  rzVal = CAL['RzcurPos']
  ryVal = str(float(CAL['RycurPos']) - value)
  rxVal =  CAL['RxcurPos']
  j7Val = str(CAL['J7PosCur'])
  j8Val = str(CAL['J8PosCur'])
  j9Val = str(CAL['J9PosCur'])
  LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
  if not RUN['offlineMode']:
    command = "MJ"+"X"+RUN['xVal']+"Y"+RUN['yVal']+"Z"+RUN['zVal']+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"J7"+j7Val+"J8"+j8Val+"J9"+j9Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
    commandVR = "MJ"+"X"+RUN['xVal']+"Y"+RUN['yVal']+"Z"+RUN['zVal']+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    return _start_manual_motion(
      command,
      "Cartesian jog",
      mj_command,
      commandVR,
    )
  else:
    xyzuvw = _forward_kinematics_display_pose(
      RUN['VR_angles'],
      "offline Cartesian jog pose",
    )
    CAL['XcurPos'], CAL['YcurPos'], CAL['ZcurPos'], CAL['RzcurPos'], CAL['RycurPos'], CAL['RxcurPos'] = [round(v, 3) for v in xyzuvw]
    CAL['RycurPos'] = CAL['RycurPos'] - value
    commandVR = (
        f"MJX{CAL['XcurPos']:.3f}Y{CAL['YcurPos']:.3f}Z{CAL['ZcurPos']:.3f}"
        f"Rz{CAL['RzcurPos']:.3f}Ry{CAL['RycurPos']:.3f}Rx{CAL['RxcurPos']:.3f}"
        f"{speedPrefix}{Speed}Ac{ACCspd}Dc{DECspd}Rm{ACCramp}"
        f"W{RUN['WC']}Lm{LoopMode}\n"
    )
    return _start_manual_motion(
      None,
      "Cartesian jog",
      mj_command,
      commandVR,
    )

@_manual_motion_request("Cartesian jog")
def RzjogNeg(value):
  # global RUN['xboxUse']
  # global WC, RUN['VR_angles']
  checkSpeedVals()
  if RUN['xboxUse'] != 1:
    almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel")
  speedtype = speedOption.get()
  #mm/sec
  if(speedtype == "mm per Sec"):
    speedPrefix = "Sm" 
  #seconds
  if(speedtype == "Seconds"):
    speedPrefix = "Ss"
  #percent
  if(speedtype == "Percent"):
    speedPrefix = "Sp"   
  Speed = speedEntryField.get() 
  ACCspd = ACCspeedField.get()
  DECspd = DECspeedField.get()
  ACCramp = ACCrampField.get()
  RUN['xVal'] = CAL['XcurPos']
  RUN['yVal'] = CAL['YcurPos']
  RUN['zVal'] = CAL['ZcurPos']
  rzVal =  str(float(CAL['RzcurPos']) - value)
  ryVal = CAL['RycurPos']
  rxVal = CAL['RxcurPos']
  j7Val = str(CAL['J7PosCur'])
  j8Val = str(CAL['J8PosCur'])
  j9Val = str(CAL['J9PosCur'])
  LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
  if not RUN['offlineMode']:
    command = "MJ"+"X"+RUN['xVal']+"Y"+RUN['yVal']+"Z"+RUN['zVal']+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"J7"+j7Val+"J8"+j8Val+"J9"+j9Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
    commandVR = "MJ"+"X"+RUN['xVal']+"Y"+RUN['yVal']+"Z"+RUN['zVal']+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    return _start_manual_motion(
      command,
      "Cartesian jog",
      mj_command,
      commandVR,
    )
  else:
    xyzuvw = _forward_kinematics_display_pose(
      RUN['VR_angles'],
      "offline Cartesian jog pose",
    )
    CAL['XcurPos'], CAL['YcurPos'], CAL['ZcurPos'], CAL['RzcurPos'], CAL['RycurPos'], CAL['RxcurPos'] = [round(v, 3) for v in xyzuvw]
    CAL['RzcurPos'] = CAL['RzcurPos'] - value
    commandVR = (
        f"MJX{CAL['XcurPos']:.3f}Y{CAL['YcurPos']:.3f}Z{CAL['ZcurPos']:.3f}"
        f"Rz{CAL['RzcurPos']:.3f}Ry{CAL['RycurPos']:.3f}Rx{CAL['RxcurPos']:.3f}"
        f"{speedPrefix}{Speed}Ac{ACCspd}Dc{DECspd}Rm{ACCramp}"
        f"W{RUN['WC']}Lm{LoopMode}\n"
    )
    return _start_manual_motion(
      None,
      "Cartesian jog",
      mj_command,
      commandVR,
    )

@_manual_motion_request("Cartesian jog")
def XjogPos(value):
  # global RUN['xboxUse']
  # global WC, RUN['VR_angles']
  checkSpeedVals()
  if RUN['xboxUse'] != 1:
    almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel")
  speedtype = speedOption.get()
  #mm/sec
  if(speedtype == "mm per Sec"):
    speedPrefix = "Sm" 
  #seconds
  if(speedtype == "Seconds"):
    speedPrefix = "Ss"
  #percent
  if(speedtype == "Percent"):
    speedPrefix = "Sp"   
  Speed = speedEntryField.get() 
  ACCspd = ACCspeedField.get()
  DECspd = DECspeedField.get()
  ACCramp = ACCrampField.get()
  RUN['xVal'] = str(float(CAL['XcurPos']) + value)
  RUN['yVal'] = CAL['YcurPos']
  RUN['zVal'] = CAL['ZcurPos']
  rzVal = CAL['RzcurPos']
  ryVal = CAL['RycurPos']
  rxVal = CAL['RxcurPos']
  j7Val = str(CAL['J7PosCur'])
  j8Val = str(CAL['J8PosCur'])
  j9Val = str(CAL['J9PosCur'])
  LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
  if not RUN['offlineMode']:
    command = "MJ"+"X"+RUN['xVal']+"Y"+RUN['yVal']+"Z"+RUN['zVal']+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"J7"+j7Val+"J8"+j8Val+"J9"+j9Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
    commandVR = "MJ"+"X"+RUN['xVal']+"Y"+RUN['yVal']+"Z"+RUN['zVal']+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    return _start_manual_motion(
      command,
      "Cartesian jog",
      mj_command,
      commandVR,
    )
  else:
    xyzuvw = _forward_kinematics_display_pose(
      RUN['VR_angles'],
      "offline Cartesian jog pose",
    )
    CAL['XcurPos'], CAL['YcurPos'], CAL['ZcurPos'], CAL['RzcurPos'], CAL['RycurPos'], CAL['RxcurPos'] = [round(v, 3) for v in xyzuvw]
    CAL['XcurPos'] = CAL['XcurPos'] + value
    commandVR = (
        f"MJX{CAL['XcurPos']:.3f}Y{CAL['YcurPos']:.3f}Z{CAL['ZcurPos']:.3f}"
        f"Rz{CAL['RzcurPos']:.3f}Ry{CAL['RycurPos']:.3f}Rx{CAL['RxcurPos']:.3f}"
        f"{speedPrefix}{Speed}Ac{ACCspd}Dc{DECspd}Rm{ACCramp}"
        f"W{RUN['WC']}Lm{LoopMode}\n"
    )
    return _start_manual_motion(
      None,
      "Cartesian jog",
      mj_command,
      commandVR,
    )

@_manual_motion_request("Cartesian jog")
def YjogPos(value):
  # global RUN['xboxUse']
  # global WC, RUN['VR_angles']
  checkSpeedVals()
  if RUN['xboxUse'] != 1:
    almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel")
  speedtype = speedOption.get()
  #mm/sec
  if(speedtype == "mm per Sec"):
    speedPrefix = "Sm" 
  #seconds
  if(speedtype == "Seconds"):
    speedPrefix = "Ss"
  #percent
  if(speedtype == "Percent"):
    speedPrefix = "Sp"   
  Speed = speedEntryField.get() 
  ACCspd = ACCspeedField.get()
  DECspd = DECspeedField.get()
  ACCramp = ACCrampField.get()
  RUN['xVal'] = CAL['XcurPos']
  RUN['yVal'] = str(float(CAL['YcurPos']) + value)
  RUN['zVal'] = CAL['ZcurPos']
  rzVal = CAL['RzcurPos']
  ryVal = CAL['RycurPos']
  rxVal = CAL['RxcurPos']
  j7Val = str(CAL['J7PosCur'])
  j8Val = str(CAL['J8PosCur'])
  j9Val = str(CAL['J9PosCur'])
  LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
  if not RUN['offlineMode']:
    command = "MJ"+"X"+RUN['xVal']+"Y"+RUN['yVal']+"Z"+RUN['zVal']+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"J7"+j7Val+"J8"+j8Val+"J9"+j9Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
    commandVR = "MJ"+"X"+RUN['xVal']+"Y"+RUN['yVal']+"Z"+RUN['zVal']+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    return _start_manual_motion(
      command,
      "Cartesian jog",
      mj_command,
      commandVR,
    )
  else:
    xyzuvw = _forward_kinematics_display_pose(
      RUN['VR_angles'],
      "offline Cartesian jog pose",
    )
    CAL['XcurPos'], CAL['YcurPos'], CAL['ZcurPos'], CAL['RzcurPos'], CAL['RycurPos'], CAL['RxcurPos'] = [round(v, 3) for v in xyzuvw]
    CAL['YcurPos'] = CAL['YcurPos'] + value
    commandVR = (
        f"MJX{CAL['XcurPos']:.3f}Y{CAL['YcurPos']:.3f}Z{CAL['ZcurPos']:.3f}"
        f"Rz{CAL['RzcurPos']:.3f}Ry{CAL['RycurPos']:.3f}Rx{CAL['RxcurPos']:.3f}"
        f"{speedPrefix}{Speed}Ac{ACCspd}Dc{DECspd}Rm{ACCramp}"
        f"W{RUN['WC']}Lm{LoopMode}\n"
    )
    return _start_manual_motion(
      None,
      "Cartesian jog",
      mj_command,
      commandVR,
    )


@_manual_motion_request("Cartesian jog")
def ZjogPos(value):
  # global RUN['xboxUse']
  # global WC, RUN['VR_angles']
  checkSpeedVals()
  if RUN['xboxUse'] != 1:
    almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel")
  speedtype = speedOption.get()
  #mm/sec
  if(speedtype == "mm per Sec"):
    speedPrefix = "Sm" 
  #seconds
  if(speedtype == "Seconds"):
    speedPrefix = "Ss"
  #percent
  if(speedtype == "Percent"):
    speedPrefix = "Sp"   
  Speed = speedEntryField.get() 
  ACCspd = ACCspeedField.get()
  DECspd = DECspeedField.get()
  ACCramp = ACCrampField.get()
  RUN['xVal'] = CAL['XcurPos']
  RUN['yVal'] = CAL['YcurPos']
  RUN['zVal'] = str(float(CAL['ZcurPos']) + value)
  rzVal = CAL['RzcurPos']
  ryVal = CAL['RycurPos']
  rxVal = CAL['RxcurPos']
  j7Val = str(CAL['J7PosCur'])
  j8Val = str(CAL['J8PosCur'])
  j9Val = str(CAL['J9PosCur'])
  LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
  if not RUN['offlineMode']:
    command = "MJ"+"X"+RUN['xVal']+"Y"+RUN['yVal']+"Z"+RUN['zVal']+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"J7"+j7Val+"J8"+j8Val+"J9"+j9Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
    commandVR = "MJ"+"X"+RUN['xVal']+"Y"+RUN['yVal']+"Z"+RUN['zVal']+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    return _start_manual_motion(
      command,
      "Cartesian jog",
      mj_command,
      commandVR,
    )
  else:
    xyzuvw = _forward_kinematics_display_pose(
      RUN['VR_angles'],
      "offline Cartesian jog pose",
    )
    CAL['XcurPos'], CAL['YcurPos'], CAL['ZcurPos'], CAL['RzcurPos'], CAL['RycurPos'], CAL['RxcurPos'] = [round(v, 3) for v in xyzuvw]
    CAL['ZcurPos'] = CAL['ZcurPos'] + value
    commandVR = (
        f"MJX{CAL['XcurPos']:.3f}Y{CAL['YcurPos']:.3f}Z{CAL['ZcurPos']:.3f}"
        f"Rz{CAL['RzcurPos']:.3f}Ry{CAL['RycurPos']:.3f}Rx{CAL['RxcurPos']:.3f}"
        f"{speedPrefix}{Speed}Ac{ACCspd}Dc{DECspd}Rm{ACCramp}"
        f"W{RUN['WC']}Lm{LoopMode}\n"
    )
    return _start_manual_motion(
      None,
      "Cartesian jog",
      mj_command,
      commandVR,
    )

@_manual_motion_request("Cartesian jog")
def RxjogPos(value):
  # global RUN['xboxUse']
  # global XcurPos, YcurPos, ZcurPos, RzcurPos, RycurPos, RxcurPos
  # global WC, RUN['VR_angles']
  checkSpeedVals()
  if RUN['xboxUse'] != 1:
    almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel")
  speedtype = speedOption.get()
  #mm/sec
  if(speedtype == "mm per Sec"):
    speedPrefix = "Sm" 
  #seconds
  if(speedtype == "Seconds"):
    speedPrefix = "Ss"
  #percent
  if(speedtype == "Percent"):
    speedPrefix = "Sp"   
  Speed = speedEntryField.get() 
  ACCspd = ACCspeedField.get()
  DECspd = DECspeedField.get()
  ACCramp = ACCrampField.get()
  RUN['xVal'] = CAL['XcurPos']
  RUN['yVal'] = CAL['YcurPos']
  RUN['zVal'] = CAL['ZcurPos']
  rzVal = CAL['RzcurPos']
  ryVal = CAL['RycurPos']
  rxVal =  str(float(CAL['RxcurPos']) + value)
  j7Val = str(CAL['J7PosCur'])
  j8Val = str(CAL['J8PosCur'])
  j9Val = str(CAL['J9PosCur'])
  LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
  if not RUN['offlineMode']:
    command = "MJ"+"X"+RUN['xVal']+"Y"+RUN['yVal']+"Z"+RUN['zVal']+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"J7"+j7Val+"J8"+j8Val+"J9"+j9Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
    commandVR = "MJ"+"X"+RUN['xVal']+"Y"+RUN['yVal']+"Z"+RUN['zVal']+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    return _start_manual_motion(
      command,
      "Cartesian jog",
      mj_command,
      commandVR,
    )
  else:
    xyzuvw = _forward_kinematics_display_pose(
      RUN['VR_angles'],
      "offline Cartesian jog pose",
    )
    CAL['XcurPos'], CAL['YcurPos'], CAL['ZcurPos'], CAL['RzcurPos'], CAL['RycurPos'], CAL['RxcurPos'] = [round(v, 3) for v in xyzuvw]
    CAL['RxcurPos'] = CAL['RxcurPos'] + value
    commandVR = (
        f"MJX{CAL['XcurPos']:.3f}Y{CAL['YcurPos']:.3f}Z{CAL['ZcurPos']:.3f}"
        f"Rz{CAL['RzcurPos']:.3f}Ry{CAL['RycurPos']:.3f}Rx{CAL['RxcurPos']:.3f}"
        f"{speedPrefix}{Speed}Ac{ACCspd}Dc{DECspd}Rm{ACCramp}"
        f"W{RUN['WC']}Lm{LoopMode}\n"
    )
    return _start_manual_motion(
      None,
      "Cartesian jog",
      mj_command,
      commandVR,
    )

@_manual_motion_request("Cartesian jog")
def RyjogPos(value):
  # global RUN['xboxUse']
  # global WC, RUN['VR_angles']
  checkSpeedVals()
  if RUN['xboxUse'] != 1:
    almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel")
  speedtype = speedOption.get()
  #mm/sec
  if(speedtype == "mm per Sec"):
    speedPrefix = "Sm" 
  #seconds
  if(speedtype == "Seconds"):
    speedPrefix = "Ss"
  #percent
  if(speedtype == "Percent"):
    speedPrefix = "Sp"   
  Speed = speedEntryField.get() 
  ACCspd = ACCspeedField.get()
  DECspd = DECspeedField.get()
  ACCramp = ACCrampField.get()
  RUN['xVal'] = CAL['XcurPos']
  RUN['yVal'] = CAL['YcurPos']
  RUN['zVal'] = CAL['ZcurPos']
  rzVal = CAL['RzcurPos']
  ryVal = str(float(CAL['RycurPos']) + value)
  rxVal =  CAL['RxcurPos']
  j7Val = str(CAL['J7PosCur'])
  j8Val = str(CAL['J8PosCur'])
  j9Val = str(CAL['J9PosCur'])
  LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
  if not RUN['offlineMode']:
    command = "MJ"+"X"+RUN['xVal']+"Y"+RUN['yVal']+"Z"+RUN['zVal']+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"J7"+j7Val+"J8"+j8Val+"J9"+j9Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
    commandVR = "MJ"+"X"+RUN['xVal']+"Y"+RUN['yVal']+"Z"+RUN['zVal']+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    return _start_manual_motion(
      command,
      "Cartesian jog",
      mj_command,
      commandVR,
    )
  else:
    xyzuvw = _forward_kinematics_display_pose(
      RUN['VR_angles'],
      "offline Cartesian jog pose",
    )
    CAL['XcurPos'], CAL['YcurPos'], CAL['ZcurPos'], CAL['RzcurPos'], CAL['RycurPos'], CAL['RxcurPos'] = [round(v, 3) for v in xyzuvw]
    CAL['RycurPos'] = CAL['RycurPos'] + value
    commandVR = (
        f"MJX{CAL['XcurPos']:.3f}Y{CAL['YcurPos']:.3f}Z{CAL['ZcurPos']:.3f}"
        f"Rz{CAL['RzcurPos']:.3f}Ry{CAL['RycurPos']:.3f}Rx{CAL['RxcurPos']:.3f}"
        f"{speedPrefix}{Speed}Ac{ACCspd}Dc{DECspd}Rm{ACCramp}"
        f"W{RUN['WC']}Lm{LoopMode}\n"
    )
    return _start_manual_motion(
      None,
      "Cartesian jog",
      mj_command,
      commandVR,
    )

@_manual_motion_request("Cartesian jog")
def RzjogPos(value):
  # global RUN['xboxUse']
  # global WC, RUN['VR_angles']
  checkSpeedVals()
  if RUN['xboxUse'] != 1:
    almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel")
  speedtype = speedOption.get()
  #mm/sec
  if(speedtype == "mm per Sec"):
    speedPrefix = "Sm" 
  #seconds
  if(speedtype == "Seconds"):
    speedPrefix = "Ss"
  #percent
  if(speedtype == "Percent"):
    speedPrefix = "Sp"   
  Speed = speedEntryField.get() 
  ACCspd = ACCspeedField.get()
  DECspd = DECspeedField.get()
  ACCramp = ACCrampField.get()
  RUN['xVal'] = CAL['XcurPos']
  RUN['yVal'] = CAL['YcurPos']
  RUN['zVal'] = CAL['ZcurPos']
  rzVal =  str(float(CAL['RzcurPos']) + value)
  ryVal = CAL['RycurPos']
  rxVal = CAL['RxcurPos']
  j7Val = str(CAL['J7PosCur'])
  j8Val = str(CAL['J8PosCur'])
  j9Val = str(CAL['J9PosCur'])
  LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
  if not RUN['offlineMode']:
    command = "MJ"+"X"+RUN['xVal']+"Y"+RUN['yVal']+"Z"+RUN['zVal']+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"J7"+j7Val+"J8"+j8Val+"J9"+j9Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
    commandVR = "MJ"+"X"+RUN['xVal']+"Y"+RUN['yVal']+"Z"+RUN['zVal']+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    return _start_manual_motion(
      command,
      "Cartesian jog",
      mj_command,
      commandVR,
    )
  else:
    xyzuvw = _forward_kinematics_display_pose(
      RUN['VR_angles'],
      "offline Cartesian jog pose",
    )
    CAL['XcurPos'], CAL['YcurPos'], CAL['ZcurPos'], CAL['RzcurPos'], CAL['RycurPos'], CAL['RxcurPos'] = [round(v, 3) for v in xyzuvw]
    CAL['RzcurPos'] = CAL['RzcurPos'] + value
    commandVR = (
        f"MJX{CAL['XcurPos']:.3f}Y{CAL['YcurPos']:.3f}Z{CAL['ZcurPos']:.3f}"
        f"Rz{CAL['RzcurPos']:.3f}Ry{CAL['RycurPos']:.3f}Rx{CAL['RxcurPos']:.3f}"
        f"{speedPrefix}{Speed}Ac{ACCspd}Dc{DECspd}Rm{ACCramp}"
        f"W{RUN['WC']}Lm{LoopMode}\n"
    )
    return _start_manual_motion(
      None,
      "Cartesian jog",
      mj_command,
      commandVR,
    )

   
  
@_manual_motion_request("Tool-frame jog")
def TXjogNeg(value):
  # global RUN['xboxUse']
  checkSpeedVals()
  if RUN['xboxUse'] != 1:
    almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel")
  speedtype = speedOption.get()
  #dont allow mm/sec - switch to sec
  if(speedtype == "mm per Sec"):
    speedMenu=OptionMenu(tab1, speedOption, "Percent", "Percent", "Seconds", "mm per Sec")
    speedPrefix = "Ss" 
    speedEntryField.delete(0, 'end')
    speedEntryField.insert(0,"50")
  #seconds
  if(speedtype == "Seconds"):
    speedPrefix = "Ss"
  #percent
  if(speedtype == "Percent"):
    speedPrefix = "Sp"   
  Speed = speedEntryField.get() 
  ACCspd = ACCspeedField.get()
  DECspd = DECspeedField.get()
  ACCramp = ACCrampField.get()
  LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
  command = "JTX1"+str(value)+speedPrefix+Speed+"G"+ACCspd+"H"+DECspd+"I"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
  if not RUN['offlineMode']:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    return _start_manual_motion(
      command,
      "Tool-frame jog",
      mt_command,
      command,
    )
  return _start_manual_motion(
    None,
    "Tool-frame jog",
    mt_command,
    command,
  )


@_manual_motion_request("Tool-frame jog")
def TYjogNeg(value):
  # global RUN['xboxUse']
  checkSpeedVals()
  if RUN['xboxUse'] != 1:
    almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel")
  speedtype = speedOption.get()
  #dont allow mm/sec - switch to sec
  if(speedtype == "mm per Sec"):
    speedMenu=OptionMenu(tab1, speedOption, "Percent", "Percent", "Seconds", "mm per Sec")
    speedPrefix = "Ss" 
    speedEntryField.delete(0, 'end')
    speedEntryField.insert(0,"50")
  #seconds
  if(speedtype == "Seconds"):
    speedPrefix = "Ss"
  #percent
  if(speedtype == "Percent"):
    speedPrefix = "Sp"   
  Speed = speedEntryField.get() 
  ACCspd = ACCspeedField.get()
  DECspd = DECspeedField.get()
  ACCramp = ACCrampField.get()
  LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
  command = "JTY1"+str(value)+speedPrefix+Speed+"G"+ACCspd+"H"+DECspd+"I"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
  if not RUN['offlineMode']:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    return _start_manual_motion(
      command,
      "Tool-frame jog",
      mt_command,
      command,
    )
  return _start_manual_motion(
    None,
    "Tool-frame jog",
    mt_command,
    command,
  )

@_manual_motion_request("Tool-frame jog")
def TZjogNeg(value):
  # global RUN['xboxUse']
  checkSpeedVals()
  if RUN['xboxUse'] != 1:
    almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel")
  speedtype = speedOption.get()
  #dont allow mm/sec - switch to sec
  if(speedtype == "mm per Sec"):
    speedMenu=OptionMenu(tab1, speedOption, "Percent", "Percent", "Seconds", "mm per Sec")
    speedPrefix = "Ss" 
    speedEntryField.delete(0, 'end')
    speedEntryField.insert(0,"50")
  #seconds
  if(speedtype == "Seconds"):
    speedPrefix = "Ss"
  #percent
  if(speedtype == "Percent"):
    speedPrefix = "Sp"   
  Speed = speedEntryField.get() 
  ACCspd = ACCspeedField.get()
  DECspd = DECspeedField.get()
  ACCramp = ACCrampField.get()
  LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
  command = "JTZ1"+str(value)+speedPrefix+Speed+"G"+ACCspd+"H"+DECspd+"I"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
  if not RUN['offlineMode']:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    return _start_manual_motion(
      command,
      "Tool-frame jog",
      mt_command,
      command,
    )
  return _start_manual_motion(
    None,
    "Tool-frame jog",
    mt_command,
    command,
  )




@_manual_motion_request("Tool-frame jog")
def TRxjogNeg(value):
  # global RUN['xboxUse']
  checkSpeedVals()
  if RUN['xboxUse'] != 1:
    almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel")
  speedtype = speedOption.get()
  #dont allow mm/sec - switch to sec
  if(speedtype == "mm per Sec"):
    speedMenu=OptionMenu(tab1, speedOption, "Percent", "Percent", "Seconds", "mm per Sec")
    speedPrefix = "Ss" 
    speedEntryField.delete(0, 'end')
    speedEntryField.insert(0,"50")
  #seconds
  if(speedtype == "Seconds"):
    speedPrefix = "Ss"
  #percent
  if(speedtype == "Percent"):
    speedPrefix = "Sp"   
  Speed = speedEntryField.get() 
  ACCspd = ACCspeedField.get()
  DECspd = DECspeedField.get()
  ACCramp = ACCrampField.get()
  LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
  command = "JTW1"+str(value)+speedPrefix+Speed+"G"+ACCspd+"H"+DECspd+"I"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
  if not RUN['offlineMode']:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    return _start_manual_motion(
      command,
      "Tool-frame jog",
      mt_command,
      command,
    )
  return _start_manual_motion(
    None,
    "Tool-frame jog",
    mt_command,
    command,
  )

@_manual_motion_request("Tool-frame jog")
def TRyjogNeg(value):
  # global RUN['xboxUse']
  checkSpeedVals()
  if RUN['xboxUse'] != 1:
    almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel")
  speedtype = speedOption.get()
  #dont allow mm/sec - switch to sec
  if(speedtype == "mm per Sec"):
    speedMenu=OptionMenu(tab1, speedOption, "Percent", "Percent", "Seconds", "mm per Sec")
    speedPrefix = "Ss" 
    speedEntryField.delete(0, 'end')
    speedEntryField.insert(0,"50")
  #seconds
  if(speedtype == "Seconds"):
    speedPrefix = "Ss"
  #percent
  if(speedtype == "Percent"):
    speedPrefix = "Sp"   
  Speed = speedEntryField.get() 
  ACCspd = ACCspeedField.get()
  DECspd = DECspeedField.get()
  ACCramp = ACCrampField.get()
  LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
  command = "JTP1"+str(value)+speedPrefix+Speed+"G"+ACCspd+"H"+DECspd+"I"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
  if not RUN['offlineMode']:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    return _start_manual_motion(
      command,
      "Tool-frame jog",
      mt_command,
      command,
    )
  return _start_manual_motion(
    None,
    "Tool-frame jog",
    mt_command,
    command,
  )

@_manual_motion_request("Tool-frame jog")
def TRzjogNeg(value):
  # global RUN['xboxUse']
  checkSpeedVals()
  if RUN['xboxUse'] != 1:
    almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel")
  speedtype = speedOption.get()
  #dont allow mm/sec - switch to sec
  if(speedtype == "mm per Sec"):
    speedMenu=OptionMenu(tab1, speedOption, "Percent", "Percent", "Seconds", "mm per Sec")
    speedPrefix = "Ss" 
    speedEntryField.delete(0, 'end')
    speedEntryField.insert(0,"50")
  #seconds
  if(speedtype == "Seconds"):
    speedPrefix = "Ss"
  #percent
  if(speedtype == "Percent"):
    speedPrefix = "Sp"   
  Speed = speedEntryField.get() 
  ACCspd = ACCspeedField.get()
  DECspd = DECspeedField.get()
  ACCramp = ACCrampField.get()
  LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
  command = "JTR1"+str(value)+speedPrefix+Speed+"G"+ACCspd+"H"+DECspd+"I"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
  if not RUN['offlineMode']:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    return _start_manual_motion(
      command,
      "Tool-frame jog",
      mt_command,
      command,
    )
  return _start_manual_motion(
    None,
    "Tool-frame jog",
    mt_command,
    command,
  )

@_manual_motion_request("Tool-frame jog")
def TXjogPos(value):
  # global RUN['xboxUse']
  checkSpeedVals()
  if RUN['xboxUse'] != 1:
    almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel")
  speedtype = speedOption.get()
  #dont allow mm/sec - switch to sec
  if(speedtype == "mm per Sec"):
    speedMenu=OptionMenu(tab1, speedOption, "Percent", "Percent", "Seconds", "mm per Sec")
    speedPrefix = "Ss" 
    speedEntryField.delete(0, 'end')
    speedEntryField.insert(0,"50")
  #seconds
  if(speedtype == "Seconds"):
    speedPrefix = "Ss"
  #percent
  if(speedtype == "Percent"):
    speedPrefix = "Sp"   
  Speed = speedEntryField.get() 
  ACCspd = ACCspeedField.get()
  DECspd = DECspeedField.get()
  ACCramp = ACCrampField.get()
  LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
  command = "JTX0"+str(value)+speedPrefix+Speed+"G"+ACCspd+"H"+DECspd+"I"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
  if not RUN['offlineMode']:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    return _start_manual_motion(
      command,
      "Tool-frame jog",
      mt_command,
      command,
    )
  return _start_manual_motion(
    None,
    "Tool-frame jog",
    mt_command,
    command,
  )

@_manual_motion_request("Tool-frame jog")
def TYjogPos(value):
  # global RUN['xboxUse']
  checkSpeedVals()
  if RUN['xboxUse'] != 1:
    almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel")
  speedtype = speedOption.get()
  #dont allow mm/sec - switch to sec
  if(speedtype == "mm per Sec"):
    speedMenu=OptionMenu(tab1, speedOption, "Percent", "Percent", "Seconds", "mm per Sec")
    speedPrefix = "Ss" 
    speedEntryField.delete(0, 'end')
    speedEntryField.insert(0,"50")
  #seconds
  if(speedtype == "Seconds"):
    speedPrefix = "Ss"
  #percent
  if(speedtype == "Percent"):
    speedPrefix = "Sp"   
  Speed = speedEntryField.get() 
  ACCspd = ACCspeedField.get()
  DECspd = DECspeedField.get()
  ACCramp = ACCrampField.get()
  LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
  command = "JTY0"+str(value)+speedPrefix+Speed+"G"+ACCspd+"H"+DECspd+"I"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
  if not RUN['offlineMode']:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    return _start_manual_motion(
      command,
      "Tool-frame jog",
      mt_command,
      command,
    )
  return _start_manual_motion(
    None,
    "Tool-frame jog",
    mt_command,
    command,
  )

@_manual_motion_request("Tool-frame jog")
def TZjogPos(value):
  # global RUN['xboxUse']
  checkSpeedVals()
  if RUN['xboxUse'] != 1:
    almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel")
  speedtype = speedOption.get()
  #dont allow mm/sec - switch to sec
  if(speedtype == "mm per Sec"):
    speedMenu=OptionMenu(tab1, speedOption, "Percent", "Percent", "Seconds", "mm per Sec")
    speedPrefix = "Ss" 
    speedEntryField.delete(0, 'end')
    speedEntryField.insert(0,"50")
  #seconds
  if(speedtype == "Seconds"):
    speedPrefix = "Ss"
  #percent
  if(speedtype == "Percent"):
    speedPrefix = "Sp"   
  Speed = speedEntryField.get() 
  ACCspd = ACCspeedField.get()
  DECspd = DECspeedField.get()
  ACCramp = ACCrampField.get()
  LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
  command = "JTZ0"+str(value)+speedPrefix+Speed+"G"+ACCspd+"H"+DECspd+"I"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
  if not RUN['offlineMode']:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    return _start_manual_motion(
      command,
      "Tool-frame jog",
      mt_command,
      command,
    )
  return _start_manual_motion(
    None,
    "Tool-frame jog",
    mt_command,
    command,
  )

@_manual_motion_request("Tool-frame jog")
def TRxjogPos(value):
  # global RUN['xboxUse']
  checkSpeedVals()
  if RUN['xboxUse'] != 1:
    almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel")
  speedtype = speedOption.get()
  #dont allow mm/sec - switch to sec
  if(speedtype == "mm per Sec"):
    speedMenu=OptionMenu(tab1, speedOption, "Percent", "Percent", "Seconds", "mm per Sec")
    speedPrefix = "Ss" 
    speedEntryField.delete(0, 'end')
    speedEntryField.insert(0,"50")
  #seconds
  if(speedtype == "Seconds"):
    speedPrefix = "Ss"
  #percent
  if(speedtype == "Percent"):
    speedPrefix = "Sp"   
  Speed = speedEntryField.get() 
  ACCspd = ACCspeedField.get()
  DECspd = DECspeedField.get()
  ACCramp = ACCrampField.get()
  LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
  command = "JTW0"+str(value)+speedPrefix+Speed+"G"+ACCspd+"H"+DECspd+"I"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
  if not RUN['offlineMode']:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    return _start_manual_motion(
      command,
      "Tool-frame jog",
      mt_command,
      command,
    )
  return _start_manual_motion(
    None,
    "Tool-frame jog",
    mt_command,
    command,
  )

@_manual_motion_request("Tool-frame jog")
def TRyjogPos(value):
  # global RUN['xboxUse']
  checkSpeedVals()
  if RUN['xboxUse'] != 1:
    almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel")
  speedtype = speedOption.get()
  #dont allow mm/sec - switch to sec
  if(speedtype == "mm per Sec"):
    speedMenu=OptionMenu(tab1, speedOption, "Percent", "Percent", "Seconds", "mm per Sec")
    speedPrefix = "Ss" 
    speedEntryField.delete(0, 'end')
    speedEntryField.insert(0,"50")
  #seconds
  if(speedtype == "Seconds"):
    speedPrefix = "Ss"
  #percent
  if(speedtype == "Percent"):
    speedPrefix = "Sp"   
  Speed = speedEntryField.get() 
  ACCspd = ACCspeedField.get()
  DECspd = DECspeedField.get()
  ACCramp = ACCrampField.get()
  LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
  command = "JTP0"+str(value)+speedPrefix+Speed+"G"+ACCspd+"H"+DECspd+"I"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
  if not RUN['offlineMode']:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    return _start_manual_motion(
      command,
      "Tool-frame jog",
      mt_command,
      command,
    )
  return _start_manual_motion(
    None,
    "Tool-frame jog",
    mt_command,
    command,
  )

@_manual_motion_request("Tool-frame jog")
def TRzjogPos(value):
  # global RUN['xboxUse']
  checkSpeedVals()
  if RUN['xboxUse'] != 1:
    almStatusLab.config(text="SYSTEM READY",  style="OK.TLabel")
    almStatusLab2.config(text="SYSTEM READY",  style="OK.TLabel")
  speedtype = speedOption.get()
  #dont allow mm/sec - switch to sec
  if(speedtype == "mm per Sec"):
    speedMenu=OptionMenu(tab1, speedOption, "Percent", "Percent", "Seconds", "mm per Sec")
    speedPrefix = "Ss" 
    speedEntryField.delete(0, 'end')
    speedEntryField.insert(0,"50")
  #seconds
  if(speedtype == "Seconds"):
    speedPrefix = "Ss"
  #percent
  if(speedtype == "Percent"):
    speedPrefix = "Sp"   
  Speed = speedEntryField.get() 
  ACCspd = ACCspeedField.get()
  DECspd = DECspeedField.get()
  ACCramp = ACCrampField.get()
  LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
  command = "JTR0"+str(value)+speedPrefix+Speed+"G"+ACCspd+"H"+DECspd+"I"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
  if not RUN['offlineMode']:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    return _start_manual_motion(
      command,
      "Tool-frame jog",
      mt_command,
      command,
    )
  return _start_manual_motion(
    None,
    "Tool-frame jog",
    mt_command,
    command,
  )


  
  
##############################################################################################################################################################  
### TEACH DEFS ################################################################################################################################ TEACH DEFS ###
##############################################################################################################################################################  

def teachInsertBelSelected():
  # global WC
  checkSpeedVals()
  try:
    selRow = tab1.progView.curselection()[0]
    selRow += 1
  except:
    last = tab1.progView.index('end')
    selRow = last
    tab1.progView.select_set(selRow)
  Speed = speedEntryField.get()
  speedtype = speedOption.get()
  if(speedtype == "Seconds"):
    speedPrefix = "Ss"
  if(speedtype == "mm per Sec"):
    speedPrefix = "Sm" 
  if(speedtype == "Percent"):
    speedPrefix = "Sp"    
  ACCspd = ACCspeedField.get()
  DECspd = DECspeedField.get()
  ACCramp = ACCrampField.get()
  Rounding = roundEntryField.get()
  movetype = options.get()
  if(movetype == "OFF J"):
    movetype = movetype+" [ PR: "+str(SavePosEntryField.get())+" ]"
    newPos = movetype + " [*] X "+CAL['XcurPos']+" Y "+CAL['YcurPos']+" Z "+CAL['ZcurPos']+" Rz "+CAL['RzcurPos']+" Ry "+CAL['RycurPos']+" Rx "+CAL['RxcurPos']+" J7 "+str(CAL['J7PosCur'])+" J8 "+str(CAL['J8PosCur'])+" J9 "+str(CAL['J9PosCur'])+" "+speedPrefix+" "+Speed+" Ac "+ACCspd+ " Dc "+DECspd+" Rm "+ACCramp+" $ "+RUN['WC']              
    tab1.progView.insert(selRow, bytes(newPos + '\n', 'utf-8')) 
    tab1.progView.selection_clear(0, END)
    tab1.progView.select_set(selRow)
    items = tab1.progView.get(0,END)
    file_path = path.relpath(ProgEntryField.get())
    with open(file_path,'w', encoding='utf-8') as f:
      for item in items:
        f.write(str(item.strip(), encoding='utf-8'))
        f.write('\n')
      f.close()
  if(movetype == "Move Vis"):
    movetype = movetype+" [ PR: "+str(SavePosEntryField.get())+" ]"
    newPos = movetype + " [*] X "+CAL['XcurPos']+" Y "+CAL['YcurPos']+" Z "+CAL['ZcurPos']+" Rz "+CAL['RzcurPos']+" Ry "+CAL['RycurPos']+" Rx "+CAL['RxcurPos']+" J7 "+str(CAL['J7PosCur'])+" J8 "+str(CAL['J8PosCur'])+" J9 "+str(CAL['J9PosCur'])+" "+speedPrefix+" "+Speed+" Ac "+ACCspd+ " Dc "+DECspd+" Rm "+ACCramp+" $ "+RUN['WC']              
    tab1.progView.insert(selRow, bytes(newPos + '\n', 'utf-8')) 
    tab1.progView.selection_clear(0, END)
    tab1.progView.select_set(selRow)
    items = tab1.progView.get(0,END)
    file_path = path.relpath(ProgEntryField.get())
    with open(file_path,'w', encoding='utf-8') as f:
      for item in items:
        f.write(str(item.strip(), encoding='utf-8'))
        f.write('\n')
      f.close() 
  elif(movetype == "Move PR"):
    movetype = movetype+" [ PR: "+str(SavePosEntryField.get())+" ]"
    newPos = movetype + " [*]"+" J7 "+str(CAL['J7PosCur'])+" J8 "+str(CAL['J8PosCur'])+" J9 "+str(CAL['J9PosCur'])+" "+speedPrefix+" "+Speed+" Ac "+ACCspd+ " Dc "+DECspd+" Rm "+ACCramp+" $ "+RUN['WC']          
    tab1.progView.insert(selRow, bytes(newPos + '\n', 'utf-8')) 
    tab1.progView.selection_clear(0, END)
    tab1.progView.select_set(selRow)
    items = tab1.progView.get(0,END)
    file_path = path.relpath(ProgEntryField.get())
    try:
      with open(file_path,'w', encoding='utf-8') as f:
        for item in items:
          f.write(str(item.strip(), encoding='utf-8'))
          f.write('\n')
        f.close()
    except:
      logger.error("No file specified")
  elif(movetype == "OFF PR "):
    movetype = movetype+" [ PR: "+str(SavePosEntryField.get())+" ] offs [ *PR: "+str(int(SavePosEntryField.get())+1)+" ] "
    newPos = movetype + " [*]"+" J7 "+str(CAL['J7PosCur'])+" J8 "+str(CAL['J8PosCur'])+" J9 "+str(CAL['J9PosCur'])+" "+speedPrefix+" "+Speed+" Ac "+ACCspd+ " Dc "+DECspd+" Rm "+ACCramp+" $ "+RUN['WC']
    tab1.progView.insert(selRow, bytes(newPos + '\n', 'utf-8')) 
    tab1.progView.selection_clear(0, END)
    tab1.progView.select_set(selRow)
    items = tab1.progView.get(0,END)
    file_path = path.relpath(ProgEntryField.get())
    with open(file_path,'w', encoding='utf-8') as f:
      for item in items:
        f.write(str(item.strip(), encoding='utf-8'))
        f.write('\n')
      f.close()
  elif(movetype == "Move J"):
    for name, val in [
        ("XcurPos", CAL['XcurPos']),
        ("YcurPos", CAL['YcurPos']),
        ("ZcurPos", CAL['ZcurPos']),
        ("RzcurPos", CAL['RzcurPos']),
        ("RycurPos", CAL['RycurPos']),
        ("RxcurPos", CAL['RxcurPos']),
        ("J7PosCur", CAL['J7PosCur']),
        ("J8PosCur", CAL['J8PosCur']),
        ("J9PosCur", CAL['J9PosCur']),
        ("speedPrefix", speedPrefix),
        ("Speed", Speed),
        ("ACCspd", ACCspd),
        ("DECspd", DECspd),
        ("ACCramp", ACCramp),
        ("WC", RUN['WC']),
    ]:
        if not isinstance(val, str):
            logger.warning(f"{name} is not a string — it is {type(val)}: {val}")
    newPos = movetype + " [*] X "+CAL['XcurPos']+" Y "+CAL['YcurPos']+" Z "+CAL['ZcurPos']+" Rz "+CAL['RzcurPos']+" Ry "+CAL['RycurPos']+" Rx "+CAL['RxcurPos']+" J7 "+str(CAL['J7PosCur'])+" J8 "+str(CAL['J8PosCur'])+" J9 "+str(CAL['J9PosCur'])+" "+speedPrefix+" "+Speed+" Ac "+ACCspd+ " Dc "+DECspd+" Rm "+ACCramp+" $ "+RUN['WC']              
    tab1.progView.insert(selRow, bytes(newPos + '\n', 'utf-8')) 
    tab1.progView.selection_clear(0, END)
    tab1.progView.select_set(selRow)
    items = tab1.progView.get(0,END)
    file_path = path.relpath(ProgEntryField.get())
    with open(file_path,'w', encoding='utf-8') as f:
      for item in items:
        f.write(str(item.strip(), encoding='utf-8'))
        f.write('\n')
      f.close()
  elif(movetype == "Move L"):
    #newPos = movetype + " [*] X "+CAL['XcurPos']+" Y "+CAL['YcurPos']+" Z "+CAL['ZcurPos']+" Rz "+CAL['RzcurPos']+" Ry "+CAL['RycurPos']+" Rx "+CAL['RxcurPos']+" J7 "+str(CAL['J7PosCur'])+" J8 "+str(CAL['J8PosCur'])+" J9 "+str(CAL['J9PosCur'])+" "+speedPrefix+" "+Speed+" Ac "+ACCspd+ " Dc "+DECspd+" Rm "+ACCramp+" Rnd "+Rounding+" $ "+RUN['WC'] 
    newPos = movetype + " [*] X "+CAL['XcurPos']+" Y "+CAL['YcurPos']+" Z "+CAL['ZcurPos']+" Rz "+CAL['RzcurPos']+" Ry "+CAL['RycurPos']+" Rx "+CAL['RxcurPos']+" J7 "+str(CAL['J7PosCur'])+" J8 "+str(CAL['J8PosCur'])+" J9 "+str(CAL['J9PosCur'])+" "+speedPrefix+" "+Speed+" Ac "+ACCspd+ " Dc "+DECspd+" Rm "+ACCramp+" Rnd "+Rounding+" $ A" 
    tab1.progView.insert(selRow, bytes(newPos + '\n', 'utf-8')) 
    tab1.progView.selection_clear(0, END)
    tab1.progView.select_set(selRow)
    items = tab1.progView.get(0,END)
    file_path = path.relpath(ProgEntryField.get())
    with open(file_path,'w', encoding='utf-8') as f:
      for item in items:
        f.write(str(item.strip(), encoding='utf-8'))
        f.write('\n')
      f.close()
  elif(movetype == "Move R"):
    newPos = movetype + " [*] J1 "+CAL['J1AngCur']+" J2 "+CAL['J2AngCur']+" J3 "+CAL['J3AngCur']+" J4 "+CAL['J4AngCur']+" J5 "+CAL['J5AngCur']+" J6 "+CAL['J6AngCur']+" J7 "+str(CAL['J7PosCur'])+" J8 "+str(CAL['J8PosCur'])+" J9 "+str(CAL['J9PosCur'])+" "+speedPrefix+" "+Speed+" Ac "+ACCspd+ " Dc "+DECspd+" Rm "+ACCramp+" $ "+RUN['WC']            
    tab1.progView.insert(selRow, bytes(newPos + '\n', 'utf-8')) 
    tab1.progView.selection_clear(0, END)
    tab1.progView.select_set(selRow)
    items = tab1.progView.get(0,END)
    file_path = path.relpath(ProgEntryField.get())
    with open(file_path,'w', encoding='utf-8') as f:
      for item in items:
        f.write(str(item.strip(), encoding='utf-8'))
        f.write('\n')
      f.close()
  elif(movetype == "Move A Mid"):
    newPos = movetype + " [*] X "+CAL['XcurPos']+" Y "+CAL['YcurPos']+" Z "+CAL['ZcurPos']+" Rz "+CAL['RzcurPos']+" Ry "+CAL['RycurPos']+" Rx "+CAL['RxcurPos']+" J7 "+str(CAL['J7PosCur'])+" J8 "+str(CAL['J8PosCur'])+" J9 "+str(CAL['J9PosCur'])+" "+speedPrefix+" "+Speed+" Ac "+ACCspd+ " Dc "+DECspd+" Rm "+ACCramp+" $ "+RUN['WC']             
    tab1.progView.insert(selRow, bytes(newPos + '\n', 'utf-8')) 
    tab1.progView.selection_clear(0, END)
    tab1.progView.select_set(selRow)
    items = tab1.progView.get(0,END)
    file_path = path.relpath(ProgEntryField.get())
    with open(file_path,'w', encoding='utf-8') as f:
      for item in items:
        f.write(str(item.strip(), encoding='utf-8'))
        f.write('\n')
      f.close()	
  elif(movetype == "Move A End"):
    newPos = movetype + " [*] X "+CAL['XcurPos']+" Y "+CAL['YcurPos']+" Z "+CAL['ZcurPos']+" Rz "+CAL['RzcurPos']+" Ry "+CAL['RycurPos']+" Rx "+CAL['RxcurPos']+" J7 "+str(CAL['J7PosCur'])+" J8 "+str(CAL['J8PosCur'])+" J9 "+str(CAL['J9PosCur'])+" "+speedPrefix+" "+Speed+" Ac "+ACCspd+ " Dc "+DECspd+" Rm "+ACCramp+" $ "+RUN['WC']             
    tab1.progView.insert(selRow, bytes(newPos + '\n', 'utf-8')) 
    tab1.progView.selection_clear(0, END)
    tab1.progView.select_set(selRow)
    items = tab1.progView.get(0,END)
    file_path = path.relpath(ProgEntryField.get())
    with open(file_path,'w', encoding='utf-8') as f:
      for item in items:
        f.write(str(item.strip(), encoding='utf-8'))
        f.write('\n')
      f.close()	
  elif(movetype == "Move C Center"):
    newPos = movetype + " [*] X "+CAL['XcurPos']+" Y "+CAL['YcurPos']+" Z "+CAL['ZcurPos']+" Rz "+CAL['RzcurPos']+" Ry "+CAL['RycurPos']+" Rx "+CAL['RxcurPos']+" J7 "+str(CAL['J7PosCur'])+" J8 "+str(CAL['J8PosCur'])+" J9 "+str(CAL['J9PosCur'])+" "+speedPrefix+" "+Speed+" Ac "+ACCspd+ " Dc "+DECspd+" Rm "+ACCramp+" $ "+RUN['WC']              
    tab1.progView.insert(selRow, bytes(newPos + '\n', 'utf-8')) 
    tab1.progView.selection_clear(0, END)
    tab1.progView.select_set(selRow)
    items = tab1.progView.get(0,END)
    file_path = path.relpath(ProgEntryField.get())
    with open(file_path,'w', encoding='utf-8') as f:
      for item in items:
        f.write(str(item.strip(), encoding='utf-8'))
        f.write('\n')
      f.close()
  elif(movetype == "Move C Start"):
    newPos = movetype + " [*] X "+CAL['XcurPos']+" Y "+CAL['YcurPos']+" Z "+CAL['ZcurPos']                 
    tab1.progView.insert(selRow, bytes(newPos + '\n', 'utf-8')) 
    tab1.progView.selection_clear(0, END)
    tab1.progView.select_set(selRow)
    items = tab1.progView.get(0,END)
    file_path = path.relpath(ProgEntryField.get())
    with open(file_path,'w', encoding='utf-8') as f:
      for item in items:
        f.write(str(item.strip(), encoding='utf-8'))
        f.write('\n')
      f.close()	
  elif(movetype == "Move C Plane"):
    newPos = movetype + " [*] X "+CAL['XcurPos']+" Y "+CAL['YcurPos']+" Z "+CAL['ZcurPos']
    tab1.progView.insert(selRow, bytes(newPos + '\n', 'utf-8')) 
    tab1.progView.selection_clear(0, END)
    tab1.progView.select_set(selRow)
    items = tab1.progView.get(0,END)
    file_path = path.relpath(ProgEntryField.get())
    with open(file_path,'w', encoding='utf-8') as f:
      for item in items:
        f.write(str(item.strip(), encoding='utf-8'))
        f.write('\n')
      f.close()
  elif(movetype == "Start Spline" or movetype == "End Spline"):
    newPos = movetype              
    tab1.progView.insert(selRow, bytes(newPos + '\n', 'utf-8')) 
    tab1.progView.selection_clear(0, END)
    tab1.progView.select_set(selRow)
    items = tab1.progView.get(0,END)
    file_path = path.relpath(ProgEntryField.get())
    with open(file_path,'w', encoding='utf-8') as f:
      for item in items:
        f.write(str(item.strip(), encoding='utf-8'))
        f.write('\n')
      f.close()
  elif(movetype == "Teach PR"):
    PR = str(SavePosEntryField.get())
    SPE6 = "Position Register "+PR+" Element 6 = "+CAL['RxcurPos']         
    tab1.progView.insert(selRow, bytes(SPE6 + '\n', 'utf-8')) 
    SPE5 = "Position Register "+PR+" Element 5 = "+CAL['RycurPos']            
    tab1.progView.insert(selRow, bytes(SPE5 + '\n', 'utf-8')) 
    SPE4 = "Position Register "+PR+" Element 4 = "+CAL['RzcurPos']           
    tab1.progView.insert(selRow, bytes(SPE4 + '\n', 'utf-8')) 	
    SPE3 = "Position Register "+PR+" Element 3 = "+CAL['ZcurPos']       
    tab1.progView.insert(selRow, bytes(SPE3 + '\n', 'utf-8')) 	
    SPE2 = "Position Register "+PR+" Element 2 = "+CAL['YcurPos']            
    tab1.progView.insert(selRow, bytes(SPE2 + '\n', 'utf-8')) 
    SPE1 = "Position Register "+PR+" Element 1 = "+CAL['XcurPos']         
    tab1.progView.insert(selRow, bytes(SPE1 + '\n', 'utf-8'))    	
    tab1.progView.selection_clear(0, END)
    tab1.progView.select_set(selRow)
    items = tab1.progView.get(0,END)
    file_path = path.relpath(ProgEntryField.get())
    with open(file_path,'w', encoding='utf-8') as f:
      for item in items:
        f.write(str(item.strip(), encoding='utf-8'))
        f.write('\n')
      f.close()

def teachReplaceSelected():
  try:
    deleteitem()
    selRow = tab1.progView.curselection()[0]
    tab1.progView.select_set(selRow-1)
  except:
    last = tab1.progView.index('end')
    selRow = last
    tab1.progView.select_set(selRow)
  teachInsertBelSelected()



 

############################################################################################################################################################## 
### PROGRAM FUNCTION DEFS ########################################################################################################## PROGRAM FUNCTION DEFS ###
############################################################################################################################################################## 


@_tracked_serial_operation("ser")
def MBreadHoldReg():
  slaveID = MBslaveEntryField.get()
  address = MBaddressEntryField.get()
  opVal = MBoperValEntryField.get()
  command = "BA"+"A"+slaveID+"B"+address+"C"+opVal+"\n"
  response = _exchange_legacy_main_command(command)
  MBoutputEntryField.delete(0, 'end')
  MBoutputEntryField.insert(0,response)

@_tracked_serial_operation("ser")
def MBreadCoil():
  slaveID = MBslaveEntryField.get()
  address = MBaddressEntryField.get()
  opVal = MBoperValEntryField.get()
  command = "BB"+"A"+slaveID+"B"+address+"C"+opVal+"\n"
  response = _exchange_legacy_main_command(command)
  MBoutputEntryField.delete(0, 'end')
  MBoutputEntryField.insert(0,response)

@_tracked_serial_operation("ser")
def MBreadInput():
  slaveID = MBslaveEntryField.get()
  address = MBaddressEntryField.get()
  opVal = MBoperValEntryField.get()
  command = "BC"+"A"+slaveID+"B"+address+"C"+opVal+"\n"
  response = _exchange_legacy_main_command(command)
  MBoutputEntryField.delete(0, 'end')
  MBoutputEntryField.insert(0,response)

@_tracked_serial_operation("ser")
def MBreadInputReg():
  slaveID = MBslaveEntryField.get()
  address = MBaddressEntryField.get()
  opVal = MBoperValEntryField.get()
  command = "BD"+"A"+slaveID+"B"+address+"C"+opVal+"\n"
  response = _exchange_legacy_main_command(command)
  MBoutputEntryField.delete(0, 'end')
  MBoutputEntryField.insert(0,response) 

@_tracked_serial_operation("ser")
def MBwriteCoil():
  slaveID = MBslaveEntryField.get()
  address = MBaddressEntryField.get()
  opVal = MBoperValEntryField.get()
  command = "BE"+"A"+slaveID+"B"+address+"C"+opVal+"\n"
  response = _exchange_legacy_main_command(command)
  MBoutputEntryField.delete(0, 'end')
  MBoutputEntryField.insert(0,response) 

   
@_tracked_serial_operation("ser")
def MBwriteReg():
  slaveID = MBslaveEntryField.get()
  address = MBaddressEntryField.get()
  opVal = MBoperValEntryField.get()
  command = "BF"+"A"+slaveID+"B"+address+"C"+opVal+"\n"
  response = _exchange_legacy_main_command(command)
  MBoutputEntryField.delete(0, 'end')
  MBoutputEntryField.insert(0,response)          

@_tracked_serial_operation("ser")
def QueryModbus():
  #command = "HD"+"\n"
  command = "MQ"+"\n"
  response = _exchange_legacy_main_command(command)
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,response)

@_tracked_serial_operation("ser")
def FaultReset():
  command = "FR"+"\n"
  response = _exchange_legacy_main_command(command)
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,response)    

  
def deleteitem():
  selRow = tab1.progView.curselection()[0]
  selection = tab1.progView.curselection()  
  tab1.progView.delete(selection[0])
  tab1.progView.selection_clear(0, END)
  tab1.progView.select_set(selRow)
  items = tab1.progView.get(0,END)
  file_path = path.relpath(ProgEntryField.get())
  with open(file_path,'w', encoding='utf-8') as f:
    for item in items:
      f.write(str(item.strip(), encoding='utf-8'))
      f.write('\n')
    f.close()  
  
def manInsItem():
  try:
    selRow = tab1.progView.curselection()[0]
    selRow += 1
  except:
    last = tab1.progView.index('end')
    selRow = last
    tab1.progView.select_set(selRow) 
  tab1.progView.insert(selRow, bytes(manEntryField.get() + '\n', 'utf-8')) 
  tab1.progView.selection_clear(0, END)
  tab1.progView.select_set(selRow) 
  selRow = tab1.progView.curselection()[0]
  curRowEntryField.delete(0, 'end')
  curRowEntryField.insert(0,selRow)
  tab1.progView.itemconfig(selRow, {'fg': "#8264B8"})
  items = tab1.progView.get(0,END)
  file_path = path.relpath(ProgEntryField.get())
  with open(file_path,'w', encoding='utf-8') as f:
    for item in items:
      f.write(str(item.strip(), encoding='utf-8'))
      f.write('\n')
    f.close()
  
def manReplItem():
  #selRow = curRowEntryField.get()
  selRow = tab1.progView.curselection()[0]
  tab1.progView.delete(selRow) 
  tab1.progView.insert(selRow, bytes(manEntryField.get() + '\n', 'utf-8')) 
  tab1.progView.selection_clear(0, END)
  tab1.progView.select_set(selRow)
  tab1.progView.itemconfig(selRow, {'fg': "#8264B8"})  
  items = tab1.progView.get(0,END)
  file_path = path.relpath(ProgEntryField.get())
  with open(file_path,'w', encoding='utf-8') as f:
    for item in items:
      f.write(str(item.strip(), encoding='utf-8'))
      f.write('\n')
    f.close()
  
def waitTime():
  try:
    selRow = tab1.progView.curselection()[0]
    selRow += 1
  except:
    last = tab1.progView.index('end')
    selRow = last
    tab1.progView.select_set(selRow)
  seconds = waitSecField.get()
  newTime = "Wait Time = "+seconds               
  tab1.progView.insert(selRow, bytes(newTime + '\n', 'utf-8')) 
  tab1.progView.selection_clear(0, END)
  tab1.progView.select_set(selRow)
  items = tab1.progView.get(0,END)
  file_path = path.relpath(ProgEntryField.get())
  with open(file_path,'w', encoding='utf-8') as f:
    for item in items:
      f.write(str(item.strip(), encoding='utf-8'))
      f.write('\n')
    f.close()



#!! Appears Not to be used
'''
def setOutputOn(): #!! Is this used anywhere?
  try:
    selRow = tab1.progView.curselection()[0]
    selRow += 1
  except:
    last = tab1.progView.index('end')
    selRow = last
    tab1.progView.select_set(selRow)
  output = outputOnEntryField.get()
  newOutput = "Out On = "+output              
  tab1.progView.insert(selRow, bytes(newOutput + '\n', 'utf-8')) 
  tab1.progView.selection_clear(0, END)
  tab1.progView.select_set(selRow)
  items = tab1.progView.get(0,END)
  file_path = path.relpath(ProgEntryField.get())
  with open(file_path,'w', encoding='utf-8') as f:
    for item in items:
      f.write(str(item.strip(), encoding='utf-8'))
      f.write('\n')
    f.close()
'''

#!! Appears Not to be used
'''
def setOutputOff():
  try:
    selRow = tab1.progView.curselection()[0]
    selRow += 1
  except:
    last = tab1.progView.index('end')
    selRow = last
    tab1.progView.select_set(selRow)
  output = outputOffEntryField.get()
  newOutput = "Out Off = "+output              
  tab1.progView.insert(selRow, bytes(newOutput + '\n', 'utf-8')) 
  tab1.progView.selection_clear(0, END)
  tab1.progView.select_set(selRow)
  items = tab1.progView.get(0,END)
  file_path = path.relpath(ProgEntryField.get())
  with open(file_path,'w', encoding='utf-8') as f:
    for item in items:
      f.write(str(item.strip(), encoding='utf-8'))
      f.write('\n')
    f.close()
'''

def tabNumber():
  try:
    selRow = tab1.progView.curselection()[0]
    selRow += 1
  except:
    last = tab1.progView.index('end')
    selRow = last
    tab1.progView.select_set(selRow)
  tabNum = tabNumEntryField.get()
  tabins = "Tab Number "+tabNum              
  tab1.progView.insert(selRow, bytes(tabins + '\n', 'utf-8')) 
  tab1.progView.selection_clear(0, END)
  tab1.progView.select_set(selRow)
  items = tab1.progView.get(0,END)
  file_path = path.relpath(ProgEntryField.get())
  with open(file_path,'w', encoding='utf-8') as f:
    for item in items:
      f.write(str(item.strip(), encoding='utf-8'))
      f.write('\n')
    f.close()






def jumpTab():
  try:
    selRow = tab1.progView.curselection()[0]
    selRow += 1
  except:
    last = tab1.progView.index('end')
    selRow = last
    tab1.progView.select_set(selRow)
  tabNum = jumpTabEntryField.get()
  tabjmp = "Jump Tab-"+tabNum              
  tab1.progView.insert(selRow, bytes(tabjmp + '\n', 'utf-8')) 
  tab1.progView.selection_clear(0, END)
  tab1.progView.select_set(selRow)
  items = tab1.progView.get(0,END)
  file_path = path.relpath(ProgEntryField.get())
  with open(file_path,'w', encoding='utf-8') as f:
    for item in items:
      f.write(str(item.strip(), encoding='utf-8'))
      f.write('\n')
    f.close()
 
def cameraOn():
  try:
    selRow = tab1.progView.curselection()[0]
    selRow += 1
  except:
    last = tab1.progView.index('end')
    selRow = last
    tab1.progView.select_set(selRow)
  value = "Cam On"
  tab1.progView.insert(selRow, bytes(value + '\n', 'utf-8'))
  tab1.progView.selection_clear(0, END)
  tab1.progView.select_set(selRow)
  items = tab1.progView.get(0,END)
  file_path = path.relpath(ProgEntryField.get())
  with open(file_path,'w', encoding='utf-8') as f:
    for item in items:
      f.write(str(item.strip(), encoding='utf-8'))
      f.write('\n')
    f.close()

def cameraOff():
  try:
    selRow = tab1.progView.curselection()[0]
    selRow += 1
  except:
    last = tab1.progView.index('end')
    selRow = last
    tab1.progView.select_set(selRow)
  value = "Cam Off"
  tab1.progView.insert(selRow, bytes(value + '\n', 'utf-8')) 
  tab1.progView.selection_clear(0, END)
  tab1.progView.select_set(selRow)
  items = tab1.progView.get(0,END)
  file_path = path.relpath(ProgEntryField.get())
  with open(file_path,'w', encoding='utf-8') as f:
    for item in items:
      f.write(str(item.strip(), encoding='utf-8'))
      f.write('\n')
    f.close()


def IfCMDInsert():
  localErrorFlag = False
  try:
    selRow = tab1.progView.curselection()[0]
    selRow += 1
  except:
    last = tab1.progView.index('end')
    selRow = last
    tab1.progView.select_set(selRow)

  option = iFoption.get()
  selection = iFselection.get()
  variable = IfVarEntryField.get()
  if (variable == ""):
    localErrorFlag = True
    message = "Please enter an input, register number or COM Port" 
    almStatusLab.config(text=message, style="Alarm.TLabel")
  inputVal = IfInputEntryField.get()
  destVal = IfDestEntryField.get()
  if(option == "5v Input"):
    if(inputVal == "1" or inputVal == "0"):
      prefix = "If Input # " + variable + " = " + inputVal + " :"
    else:
      localErrorFlag = True
      message = "Please enter a 1 or 0 for the = value" 
      almStatusLab.config(text=message, style="Alarm.TLabel")

  elif (option == "Register"):
    if(inputVal == ""):
      localErrorFlag = True
      message = "Please enter a register number" 
      almStatusLab.config(text=message, style="Alarm.TLabel")
    prefix = "If Register # " + variable + " = " + inputVal + " :"

  elif (option == "COM Device"):
    if(inputVal == ""):
      localErrorFlag = True
      message = "Please enter expected COM device input" 
      almStatusLab.config(text=message, style="Alarm.TLabel")
    prefix = "If COM Device # " + variable + " = " + inputVal + " :"

  elif (option == "MB Coil"):
    if(inputVal == ""):
      localErrorFlag = True
      message = "Please enter expected Modbus Coil" 
      almStatusLab.config(text=message, style="Alarm.TLabel")
    prefix = "If MBcoil - SlaveID (1) - Coil # " + variable + " = " + inputVal + " :"

  elif (option == "MB Input"):
    if(inputVal == ""):
      localErrorFlag = True
      message = "Please enter expected Modbus Input" 
      almStatusLab.config(text=message, style="Alarm.TLabel")
    prefix = "If MBinput - SlaveID (1) - Input # " + variable + " = " + inputVal + " :"

  elif (option == "MB Hold Reg"):
    if(inputVal == ""):
      localErrorFlag = True
      message = "Please enter expected Modbus Holding Register" 
      almStatusLab.config(text=message, style="Alarm.TLabel")
    prefix = "If MBhold reg - SlaveID (1) Num Reg's (1) - Reg # " + variable + " = " + inputVal + " :"

  elif (option == "MB Input Reg"):
    if(inputVal == ""):
      localErrorFlag = True
      message = "Please enter expected Modbus Holding Register" 
      almStatusLab.config(text=message, style="Alarm.TLabel")
    prefix = "If MBInput Reg - SlaveID (1) Num Reg's (1) - Input Reg # " + variable + " = " + inputVal + " :"          

  if(selection == "Call Prog"):
    if (destVal == ""):
      localErrorFlag = True
      message = "Please enter a program name" 
      almStatusLab.config(text=message, style="Alarm.TLabel")
    value = prefix  + " Call Prog " + destVal
  elif(selection == "Jump Tab"):
    if (destVal == ""):
      localErrorFlag = True
      message = "Please enter a destination tab" 
      almStatusLab.config(text=message, style="Alarm.TLabel")
    value = prefix + " Jump to Tab " + destVal
  elif(selection == "Stop"):
    value = prefix + " Stop " 

  if(not localErrorFlag):        
    tab1.progView.insert(selRow, bytes(value + '\n', 'utf-8')) 
    tab1.progView.selection_clear(0, END)
    tab1.progView.select_set(selRow)
    items = tab1.progView.get(0,END)
    file_path = path.relpath(ProgEntryField.get())
    with open(file_path,'w', encoding='utf-8') as f:
      for item in items:
        f.write(str(item.strip(), encoding='utf-8'))
        f.write('\n')
      f.close()



def WaitCMDInsert():
  localErrorFlag = False
  try:
    selRow = tab1.progView.curselection()[0]
    selRow += 1
  except:
    last = tab1.progView.index('end')
    selRow = last
    tab1.progView.select_set(selRow)

  option = waitoption.get()
  variable = waitVarEntryField.get()
  if (variable == ""):
    localErrorFlag = True
    message = "Please enter an input or Modbus address" 
    almStatusLab.config(text=message, style="Alarm.TLabel")
  inputVal = waitInputEntryField.get()
  timoutVal = waitTimeoutEntryField.get()
  if(option == "5v Input"):
    if(inputVal == "1" or inputVal == "0"):
      value = "Wait 5v Input # " + variable + " = " + inputVal + " : Timeout = " + timoutVal 
    else:
      localErrorFlag = True
      message = "Please enter a 1 or 0 for the = value" 
      almStatusLab.config(text=message, style="Alarm.TLabel")

  elif (option == "MB Coil"):
    if(inputVal == ""):
      localErrorFlag = True
      message = "Please enter expected Modbus Coil" 
      almStatusLab.config(text=message, style="Alarm.TLabel")
    value = "Wait MBcoil - SlaveID (1) - Coil # " + variable + " = " + inputVal + " : Timeout = " + timoutVal 

  elif (option == "MB Input"):
    if(inputVal == ""):
      localErrorFlag = True
      message = "Please enter expected Modbus Input" 
      almStatusLab.config(text=message, style="Alarm.TLabel")
    value = "Wait MBinput - SlaveID (1) - Input # " + variable + " = " + inputVal + " : Timeout = " + timoutVal  

  if(not localErrorFlag):        
    tab1.progView.insert(selRow, bytes(value + '\n', 'utf-8')) 
    tab1.progView.selection_clear(0, END)
    tab1.progView.select_set(selRow)
    items = tab1.progView.get(0,END)
    file_path = path.relpath(ProgEntryField.get())
    with open(file_path,'w', encoding='utf-8') as f:
      for item in items:
        f.write(str(item.strip(), encoding='utf-8'))
        f.write('\n')
      f.close()  



def SetCMDInsert():
  localErrorFlag = False
  try:
    selRow = tab1.progView.curselection()[0]
    selRow += 1
  except:
    last = tab1.progView.index('end')
    selRow = last
    tab1.progView.select_set(selRow)

  option = setoption.get()
  variable = setVarEntryField.get()
  if (variable == ""):
    localErrorFlag = True
    message = "Please enter an input or Modbus address" 
    almStatusLab.config(text=message, style="Alarm.TLabel")
  inputVal = setInputEntryField.get()
  if(option == "5v Output"):
    if(inputVal == "1" or inputVal == "0"):
      value = "Set 5v Output # " + variable + " = " + inputVal 
    else:
      localErrorFlag = True
      message = "Please enter a 1 or 0 for the = value" 
      almStatusLab.config(text=message, style="Alarm.TLabel")

  elif (option == "MB Coil"):
    if(inputVal == ""):
      localErrorFlag = True
      message = "Please enter expected Modbus Coil" 
      almStatusLab.config(text=message, style="Alarm.TLabel")
    value = "Set MBcoil - SlaveID (1) - Coil # " + variable + " = " + inputVal

  elif (option == "MB Register"):
    if(inputVal == ""):
      localErrorFlag = True
      message = "Please enter expected Modbus Register" 
      almStatusLab.config(text=message, style="Alarm.TLabel")
    value = "Set MBoutput - SlaveID (1) - Input # " + variable + " = " + inputVal

  if(not localErrorFlag):        
    tab1.progView.insert(selRow, bytes(value + '\n', 'utf-8')) 
    tab1.progView.selection_clear(0, END)
    tab1.progView.select_set(selRow)
    items = tab1.progView.get(0,END)
    file_path = path.relpath(ProgEntryField.get())
    with open(file_path,'w', encoding='utf-8') as f:
      for item in items:
        f.write(str(item.strip(), encoding='utf-8'))
        f.write('\n')
      f.close()          


def ReadAuxCom():
  try:
    selRow = tab1.progView.curselection()[0]
    selRow += 1
  except:
    last = tab1.progView.index('end')
    selRow = last
    tab1.progView.select_set(selRow)
  comNum = auxPortEntryField .get()
  comChar = auxCharEntryField .get()
  servoins = "Read COM # "+comNum+" Char: "+comChar              
  tab1.progView.insert(selRow, bytes(servoins + '\n', 'utf-8')) 
  tab1.progView.selection_clear(0, END)
  tab1.progView.select_set(selRow)
  items = tab1.progView.get(0,END)
  file_path = path.relpath(ProgEntryField.get())
  with open(file_path,'w', encoding='utf-8') as f:
    for item in items:
      f.write(str(item.strip(), encoding='utf-8'))
      f.write('\n')
    f.close()


def TestAuxCom():
  try:
    # global RUN['ser3']    
    port = "COM" + com3PortEntryField.get()     
    baud = 9600    
    RUN['ser3'] = serial.Serial(port,baud,timeout=5)
  except:
    #Curtime = datetime.now().strftime("%B %d %Y - %I:%M%p")
    #tab8.ElogView.insert(END, Curtime+" - UNABLE TO ESTABLISH COMMUNICATIONS WITH SERIAL DEVICE")
    logger.error("UNABLE TO ESTABLISH COMMUNICATIONS WITH SERIAL DEVICE")
    value=tab8.ElogView.get(0,END)
    pickle.dump(value,open("ErrorLog","wb"))
  RUN['ser3'].flushInput()
  numChar = int(com3charPortEntryField.get())
  response = str(RUN['ser3'].read(numChar).strip(),'utf-8')    
  com3outPortEntryField .delete(0, 'end')
  com3outPortEntryField .insert(0,response)



def Servo():
  try:
    selRow = tab1.progView.curselection()[0]
    selRow += 1
  except:
    last = tab1.progView.index('end')
    selRow = last
    tab1.progView.select_set(selRow)
  servoNum = servoNumEntryField.get()
  servoPos = servoPosEntryField.get()
  servoins = "Servo number "+servoNum+" to position: "+servoPos              
  tab1.progView.insert(selRow, bytes(servoins + '\n', 'utf-8')) 
  tab1.progView.selection_clear(0, END)
  tab1.progView.select_set(selRow)
  items = tab1.progView.get(0,END)
  file_path = path.relpath(ProgEntryField.get())
  with open(file_path,'w', encoding='utf-8') as f:
    for item in items:
      f.write(str(item.strip(), encoding='utf-8'))
      f.write('\n')
    f.close()

def loadProg():
  if getattr(sys, 'frozen', False):
    folder = os.path.dirname(sys.executable)
  elif __file__:
    folder = os.path.dirname(os.path.realpath(__file__))
  #folder = os.path.dirname(os.path.realpath(__file__))
  filetypes = (('robot program', '*.ar4'),("all files", "*.*"))
  filename = fd.askopenfilename(title='Open files',initialdir=folder,filetypes=filetypes)
  name = os.path.basename(filename)
  ProgEntryField.delete(0, 'end')
  ProgEntryField.insert(0,name)
  tab1.progView.delete(0,END)
  if filename == "":
    return
  try:
    Prog = open(filename,"rb")
    time.sleep(.1)
    for item in Prog:
      tab1.progView.insert(END,item)
    tab1.progView.pack()
    scrollbar.config(command=tab1.progView.yview)
    save_calibration(CAL)
  except FileNotFoundError:
    logger.warning("File not found. Please check the file path and try again.")
  except Exception as e:
    logger.error(f"An error occurred: {e}")

def callProg(name):  
  ProgEntryField.delete(0, 'end')
  ProgEntryField.insert(0,name)
  tab1.progView.delete(0,END)
  Prog = open(name,"rb")
  time.sleep(.1)
  for item in Prog:
    tab1.progView.insert(END,item)
  tab1.progView.pack()
  scrollbar.config(command=tab1.progView.yview)

def CreateProg():
  user_input = simpledialog.askstring(title="New Program", prompt="New Program Name:")
  file_path = user_input + ".ar4"
  with open(file_path,'w', encoding='utf-8') as f:
    f.write("## RUN ONCE ##")
    f.write('\n')
    f.write('\n')
    f.write("## START PROGRAM LOOP ##")
    f.write('\n')
  f.close()
  ProgEntryField.delete(0, 'end')
  ProgEntryField.insert(0,file_path)
  tab1.progView.delete(0,END)
  Prog = open(file_path,"rb")
  time.sleep(.1)
  for item in Prog:
    tab1.progView.insert(END,item)
  tab1.progView.pack()
  scrollbar.config(command=tab1.progView.yview)
  save_calibration(CAL) 



def insertCallProg():  
  try:
    selRow = tab1.progView.curselection()[0]
    selRow += 1
  except:
    last = tab1.progView.index('end')
    selRow = last
    tab1.progView.select_set(selRow)
  newProg = changeProgEntryField.get()
  changeProg = "Call Program - "+newProg
  if  str(changeProg[-4:]) != ".ar4":
    changeProg = changeProg + ".ar4"             
  tab1.progView.insert(selRow, bytes(changeProg + '\n', 'utf-8')) 
  tab1.progView.selection_clear(0, END)
  tab1.progView.select_set(selRow)
  items = tab1.progView.get(0,END)
  file_path = path.relpath(ProgEntryField.get())
  with open(file_path,'w', encoding='utf-8') as f:
    for item in items:
      f.write(str(item.strip(), encoding='utf-8'))
      f.write('\n')
    f.close()

def insertGCprog():  
  try:
    selRow = tab1.progView.curselection()[0]
    selRow += 1
  except:
    last = tab1.progView.index('end')
    selRow = last
    tab1.progView.select_set(selRow)
  newProg = PlayGCEntryField.get()
  GCProg = "Run Gcode Program - "+newProg            
  tab1.progView.insert(selRow, bytes(GCProg + '\n', 'utf-8')) 
  tab1.progView.selection_clear(0, END)
  tab1.progView.select_set(selRow)
  items = tab1.progView.get(0,END)
  file_path = path.relpath(ProgEntryField.get())
  with open(file_path,'w', encoding='utf-8') as f:
    for item in items:
      f.write(str(item.strip(), encoding='utf-8'))
      f.write('\n')
    f.close()    

    

def insertStop():  
  try:
    selRow = tab1.progView.curselection()[0]
    selRow += 1
  except:
    last = tab1.progView.index('end')
    selRow = last
    tab1.progView.select_set(selRow)
  value = "Stop Program"           
  tab1.progView.insert(selRow, bytes(value + '\n', 'utf-8')) 
  tab1.progView.selection_clear(0, END)
  tab1.progView.select_set(selRow)
  items = tab1.progView.get(0,END)
  file_path = path.relpath(ProgEntryField.get())
  with open(file_path,'w', encoding='utf-8') as f:
    for item in items:
      f.write(str(item.strip(), encoding='utf-8'))
      f.write('\n')
    f.close()


def openText():
  file_path = path.relpath(ProgEntryField.get())
  
  match CE['Platform']['OS']:
    case 'Windows':
      os.startfile(file_path)
    case 'Linux':
      try:
        subprocess.run(["xdg-open", file_path], check=False)
      except FileNotFoundError:
          logger.error("xdg-open not found. Please install xdg-utils package.")
    case _:
      logger.error("Unsupported OS on File Open")

def reloadProg():
  file_path = path.relpath(ProgEntryField.get())
  ProgEntryField.delete(0, 'end')
  ProgEntryField.insert(0,file_path)
  tab1.progView.delete(0,END)
  Prog = open(file_path,"rb")
  time.sleep(.1)
  for item in Prog:
    tab1.progView.insert(END,item)
  tab1.progView.pack()
  scrollbar.config(command=tab1.progView.yview)
  save_calibration(CAL)      


def insertvisFind():
  try:
    selRow = tab1.progView.curselection()[0]
    selRow += 1
  except:
    last = tab1.progView.index('end')
    selRow = last
    tab1.progView.select_set(selRow)
  template = RUN['selectedTemplate'].get()
  if (template == ""):
    template = "None_Selected.jpg"
  CAL['autoBGVal'] = int(RUN['autoBG'].get())
  if (CAL['autoBGVal'] == 1):
    BGcolor = "(Auto)"
  else:
    BGcolor = VisBacColorEntryField.get()
  score = VisScoreEntryField.get()
  passTab = visPassEntryField.get()
  failTab = visFailEntryField.get()
  value = "Vis Find - "+template+" - BGcolor "+BGcolor+" Score "+score+" Pass "+passTab+" Fail "+failTab
  tab1.progView.insert(selRow, bytes(value + '\n', 'utf-8'))
  tab1.progView.selection_clear(0, END)
  tab1.progView.select_set(selRow)
  items = tab1.progView.get(0,END)
  file_path = path.relpath(ProgEntryField.get())
  with open(file_path,'w', encoding='utf-8') as f:
    for item in items:
      f.write(str(item.strip(), encoding='utf-8'))
      f.write('\n')
    f.close()

def insertRegister():  
  try:
    selRow = tab1.progView.curselection()[0]
    selRow += 1
  except:
    last = tab1.progView.index('end')
    selRow = last
    tab1.progView.select_set(selRow)
  regNum = regNumEntryField.get()
  regCmd = regEqEntryField.get()
  regIns = "Register "+regNum+" = "+regCmd             
  tab1.progView.insert(selRow, bytes(regIns + '\n', 'utf-8')) 
  tab1.progView.selection_clear(0, END)
  tab1.progView.select_set(selRow)
  items = tab1.progView.get(0,END)
  file_path = path.relpath(ProgEntryField.get())
  with open(file_path,'w', encoding='utf-8') as f:
    for item in items:
      f.write(str(item.strip(), encoding='utf-8'))
      f.write('\n')
    f.close()
  
def storPos():
  try:
    selRow = tab1.progView.curselection()[0]
    selRow += 1
  except:
    last = tab1.progView.index('end')
    selRow = last
    tab1.progView.select_set(selRow)
  regNum = storPosNumEntryField.get()
  regElmnt = storPosElEntryField.get()
  regCmd = storPosValEntryField.get()
  regIns = "Position Register "+regNum+" Element "+regElmnt+" = "+regCmd             
  tab1.progView.insert(selRow, bytes(regIns + '\n', 'utf-8')) 
  tab1.progView.selection_clear(0, END)
  tab1.progView.select_set(selRow)
  items = tab1.progView.get(0,END)
  file_path = path.relpath(ProgEntryField.get())
  with open(file_path,'w', encoding='utf-8') as f:
    for item in items:
      f.write(str(item.strip(), encoding='utf-8'))
      f.write('\n')
    f.close()
  
def insCalibrate():  
  try:
    selRow = tab1.progView.curselection()[0]
    selRow += 1
  except:
    last = tab1.progView.index('end')
    selRow = last
    tab1.progView.select_set(selRow)
  insCal = "Calibrate Robot"          
  tab1.progView.insert(selRow, bytes(insCal + '\n', 'utf-8')) 
  tab1.progView.selection_clear(0, END)
  tab1.progView.select_set(selRow)
  items = tab1.progView.get(0,END)
  file_path = path.relpath(ProgEntryField.get())
  with open(file_path,'w', encoding='utf-8') as f:
    for item in items:
      f.write(str(item.strip(), encoding='utf-8'))
      f.write('\n')
    f.close()

def progViewselect(e):
  try:
    selRow = tab1.progView.curselection()[0]
    curRowEntryField.delete(0, 'end')
    curRowEntryField.insert(0,selRow)
  except Exception as e:
    logger.debug(f"No selection available: {e}")
 
def getSel():
  selRow = tab1.progView.curselection()[0]
  tab1.progView.see(selRow+2)
  data = list(map(int, tab1.progView.curselection()))
  command=tab1.progView.get(data[0]).decode()
  manEntryField.delete(0, 'end')
  manEntryField.insert(0, command)  
  
def Servo0on():
  return _request_manual_servo(0, True, servo0onEntryField)


def Servo0off():
  return _request_manual_servo(0, False, servo0offEntryField)


def Servo1on():
  return _request_manual_servo(1, True, servo1onEntryField)


def Servo1off():
  return _request_manual_servo(1, False, servo1offEntryField)


def Servo2on():
  return _request_manual_servo(2, True, servo2onEntryField)


def Servo2off():
  return _request_manual_servo(2, False, servo2offEntryField)


def Servo3on():
  return _request_manual_servo(3, True, servo3onEntryField)


def Servo3off():
  return _request_manual_servo(3, False, servo3offEntryField)


def DO1on():
  return _request_manual_output(1, True, DO1onEntryField)


def DO1off():
  return _request_manual_output(1, False, DO1offEntryField)


def DO2on():
  return _request_manual_output(2, True, DO2onEntryField)


def DO2off():
  return _request_manual_output(2, False, DO2offEntryField)


def DO3on():
  return _request_manual_output(3, True, DO3onEntryField)


def DO3off():
  return _request_manual_output(3, False, DO3offEntryField)


def DO4on():
  return _request_manual_output(4, True, DO4onEntryField)


def DO4off():
  return _request_manual_output(4, False, DO4offEntryField)


def DO5on():
  return _request_manual_output(5, True, DO5onEntryField)


def DO5off():
  return _request_manual_output(5, False, DO5offEntryField)


def DO6on():
  return _request_manual_output(6, True, DO6onEntryField)


def DO6off():
  return _request_manual_output(6, False, DO6offEntryField)

def CalcLinDist(X2,Y2,Z2):
  # global RUN['LineDist']
  X1 = CAL['XcurPos']
  Y1 = CAL['YcurPos']
  Z1 = CAL['ZcurPos']
  RUN['LineDist'] = (((X2-X1)**2)+((Y2-Y1)**2)+((Z2-Z1)**2))**.5
  return (RUN['LineDist'])

def CalcLinVect(X2,Y2,Z2):
  # global RUN['Xv']
  # global RUN['Yv']
  # global RUN['Zv']
  X1 = CAL['XcurPos']
  Y1 = CAL['YcurPos']
  Z1 = CAL['ZcurPos']
  RUN['Xv'] = X2-X1
  RUN['Yv'] = Y2-Y1
  RUN['Zv'] = Z2-Z1
  return (RUN['Xv'],RUN['Yv'],RUN['Zv'])

##############################################################################################################################################################
### CALIBRATION & SAVE DEFS ###################################################################################################### CALIBRATION & SAVE DEFS ###
##############################################################################################################################################################

def _calibration_available():
  if not RUN['offlineMode']:
    return True
  message = "Calibration not supported in offline mode"
  almStatusLab.config(text=message, style="Alarm.TLabel")
  almStatusLab2.config(text=message, style="Alarm.TLabel")
  return False


def _binary_controller_flag(value, field_name):
  number = finite_number(value, field_name)
  if number not in (0, 1):
    raise MotionInputError(f"{field_name} must be 0 or 1")
  return int(number)


def _prepare_calibration_command(selections):
  if isinstance(selections, (str, bytes)):
    raise MotionInputError("calibration selections must be a numeric sequence")
  try:
    selections = tuple(selections)
  except TypeError as exc:
    raise MotionInputError(
      "calibration selections must be a numeric sequence"
    ) from exc
  if len(selections) != 9:
    raise MotionInputError("calibration selections must contain 9 values")
  normalized_selections = tuple(
    _binary_controller_flag(value, f"J{axis} calibration selection")
    for axis, value in enumerate(selections, start=1)
  )
  offsets = tuple(
    finite_number(CAL[f'J{axis}calOff'], f"J{axis} calibration offset")
    for axis in range(1, 10)
  )
  return _build_startup_numeric_command(
    "LL",
    zip(tuple("ABCDEFGHIJKLMNOPQR"), normalized_selections + offsets),
  )


def _record_calibration_response(
  response,
  success_message,
  failure_message,
  *,
  update_virtual,
  controller_write_started=True,
  uncertainty_reason=None,
):
  if not isinstance(controller_write_started, bool):
    raise TypeError("calibration write-start state must be boolean")
  if uncertainty_reason is not None and (
    not isinstance(uncertainty_reason, str)
    or not uncertainty_reason.strip()
    or uncertainty_reason != uncertainty_reason.strip()
  ):
    raise TypeError("calibration uncertainty reason must be normalized text")

  parsed_position = None
  applied_position = None
  position_validation_reason = None
  if isinstance(response, str):
    try:
      parsed_position = parse_position_response(response)
    except ProtocolResponseError:
      pass
  if parsed_position is not None:
    try:
      _current_controller_joint_calibration().validate_positions(
        parsed_position.joints + parsed_position.external
      )
    except MotionInputError:
      parsed_position = None
      position_validation_reason = (
        "calibration command returned a position outside calibrated limits"
      )
  if parsed_position is not None and parsed_position.flag:
    ErrorHandler(parsed_position.flag)
    succeeded = False
    if controller_write_started:
      _invalidate_uncertain_controller_calibration(
        "calibration command ended with a controller motion fault"
      )
  else:
    applied_position = (
      _apply_valid_position_response(response)
      if parsed_position is not None
      else None
    )
    succeeded = applied_position is not None
  if parsed_position is None and controller_write_started:
    _invalidate_uncertain_controller_calibration(
      position_validation_reason
      or uncertainty_reason
      or "calibration command returned no valid controller position"
    )
  if (
    not succeeded
    and isinstance(response, str)
    and response
    and not response.startswith("A")
  ):
    ErrorHandler(response)

  message = success_message if succeeded else failure_message
  style = "OK.TLabel" if succeeded else "Alarm.TLabel"
  if not (
    succeeded
    and applied_position.speed_violation
  ):
    almStatusLab.config(text=message, style=style)
    almStatusLab2.config(text=message, style=style)
  if succeeded and update_virtual:
    RUN['VR_angles'] = [
      float(CAL['J1AngCur']),
      float(CAL['J2AngCur']),
      float(CAL['J3AngCur']),
      float(CAL['J4AngCur']),
      float(CAL['J5AngCur']),
      float(CAL['J6AngCur']),
    ]
    setStepMonitorsVR()

  if succeeded:
    logger.info(message)
  else:
    logger.error(message)
  value = tab8.ElogView.get(0, END)
  pickle.dump(value, open("ErrorLog", "wb"))
  return succeeded


@dataclass(frozen=True)
class CalibrationStage:
  command: str
  success_message: str
  failure_message: str
  update_virtual: bool = True

  def __post_init__(self):
    object.__setattr__(
      self,
      "command",
      _validated_startup_command(self.command, "LL"),
    )
    for field_name in ("success_message", "failure_message"):
      value = getattr(self, field_name)
      if (
        not isinstance(value, str)
        or not value.strip()
        or value != value.strip()
      ):
        raise MotionInputError(
          f"calibration {field_name.replace('_', ' ')} must be normalized text"
        )
    if not isinstance(self.update_virtual, bool):
      raise MotionInputError("calibration virtual-update state must be boolean")


@dataclass(frozen=True)
class CalibrationWorkerResult:
  worker_token: object
  request_id: object
  stage_index: object
  event_type: object
  response: object
  error: object
  write_started: object


CALIBRATION_POSITION_KEYS = (
  'J1AngCur', 'J2AngCur', 'J3AngCur',
  'J4AngCur', 'J5AngCur', 'J6AngCur',
  'XcurPos', 'YcurPos', 'ZcurPos',
  'RzcurPos', 'RycurPos', 'RxcurPos',
  'J7PosCur', 'J8PosCur', 'J9PosCur',
)


@dataclass(frozen=True)
class CalibrationPoseSnapshot:
  calibration_values: tuple
  wrist_config: str
  virtual_angles: tuple
  step_monitors: tuple
  step_values: tuple
  non_primary_entry_values: tuple
  slider_values: tuple
  manual_debug: object
  confirmed_generation: int
  acknowledged_forced_target: object
  resynchronization_required: bool
  persistence_dirty: bool
  persistence_job: object


@dataclass
class CalibrationOperation:
  request_id: int
  name: str
  stages: tuple
  request_lease: object
  activity_lease: object
  serial_port: object
  completion_callback: object = None
  stage_index: int = 0
  worker_token: object = None
  worker_active: bool = False
  stage_snapshot: object = None
  settled: bool = False

  def __post_init__(self):
    if (
      isinstance(self.request_id, bool)
      or not isinstance(self.request_id, int)
      or self.request_id <= 0
    ):
      raise MotionInputError("calibration request ID must be positive")
    if (
      not isinstance(self.name, str)
      or not self.name.strip()
      or self.name != self.name.strip()
    ):
      raise MotionInputError("calibration request name must be normalized text")
    if (
      not isinstance(self.stages, tuple)
      or not self.stages
      or not all(isinstance(stage, CalibrationStage) for stage in self.stages)
    ):
      raise MotionInputError("calibration operation requires validated stages")
    if not isinstance(self.request_lease, MotionRequestLease):
      raise MotionInputError("calibration operation requires a motion lease")
    if not callable(getattr(self.activity_lease, "close", None)):
      raise MotionInputError("calibration operation requires a serial activity lease")
    if self.serial_port is None or not getattr(self.serial_port, "is_open", False):
      raise ConnectionError("calibration requires an open controller connection")
    if self.completion_callback is not None and not callable(
      self.completion_callback
    ):
      raise MotionInputError("calibration completion callback must be callable")
    if (
      self.stage_index != 0
      or self.worker_token is not None
      or self.worker_active
      or self.stage_snapshot is not None
      or self.settled
    ):
      raise MotionInputError("calibration operation initial state is invalid")


def _set_calibration_status(message, style):
  if (
    not isinstance(message, str)
    or not message.strip()
    or message != message.strip()
  ):
    raise TypeError("calibration status must be normalized text")
  if not isinstance(style, str) or not style:
    raise TypeError("calibration status style must be non-empty text")
  almStatusLab.config(text=message, style=style)
  almStatusLab2.config(text=message, style=style)


def _calibration_pose_widget_groups():
  return (
    (
      J1curAngEntryField, J2curAngEntryField, J3curAngEntryField,
      J4curAngEntryField, J5curAngEntryField, J6curAngEntryField,
      XcurEntryField, YcurEntryField, ZcurEntryField,
      RzcurEntryField, RycurEntryField, RxcurEntryField,
      J7curAngEntryField, J8curAngEntryField, J9curAngEntryField,
    ),
    (
      J1jogslide, J2jogslide, J3jogslide,
      J4jogslide, J5jogslide, J6jogslide,
      J7jogslide, J8jogslide, J9jogslide,
    ),
  )


def _capture_calibration_pose_snapshot():
  calibration_values = tuple(
    (key, CAL[key])
    for key in CALIBRATION_POSITION_KEYS
  )
  wrist_config = RUN['WC']
  if wrist_config not in ("F", "N"):
    raise RuntimeError("calibration pose wrist state is invalid")
  virtual_angles = tuple(
    finite_number(value, f"saved virtual joint {axis}")
    for axis, value in enumerate(RUN['VR_angles'], start=1)
  )
  step_monitors = tuple(
    finite_number(value, f"saved step monitor {axis}")
    for axis, value in enumerate(RUN['StepMonitors'], start=1)
  )
  step_values = tuple(
    finite_number(RUN[f'J{axis}StepM'], f"saved J{axis} step monitor")
    for axis in range(1, PRIMARY_JOINT_COUNT + 1)
  )
  if (
    len(virtual_angles) != PRIMARY_JOINT_COUNT
    or len(step_monitors) != PRIMARY_JOINT_COUNT
  ):
    raise RuntimeError("calibration pose runtime vectors must contain six values")

  entry_fields, jog_sliders = _calibration_pose_widget_groups()
  primary_joint_count = PRIMARY_JOINT_COUNT
  non_primary_entry_values = tuple(
    entry_field.get()
    for entry_field in entry_fields[primary_joint_count:]
  )
  slider_values = tuple(jog_slider.get() for jog_slider in jog_sliders)
  manual_debug = manEntryField.get()
  generation = confirmed_position_generation
  if (
    isinstance(generation, bool)
    or not isinstance(generation, int)
    or generation < 0
  ):
    raise RuntimeError("confirmed position generation is invalid")
  resynchronization_required = (
    controller_position_resynchronization_required.is_set()
  )
  if not isinstance(resynchronization_required, bool):
    raise RuntimeError("controller resynchronization state is invalid")
  persistence_dirty = _calibration_dirty
  if not isinstance(persistence_dirty, bool):
    raise RuntimeError("calibration persistence state is invalid")
  if _calibration_save_job is not None and not persistence_dirty:
    raise RuntimeError("calibration persistence job has no dirty state")

  return CalibrationPoseSnapshot(
    calibration_values,
    wrist_config,
    virtual_angles,
    step_monitors,
    step_values,
    non_primary_entry_values,
    slider_values,
    manual_debug,
    generation,
    _acknowledged_forced_position_target_value(),
    resynchronization_required,
    persistence_dirty,
    _calibration_save_job,
  )


def _restore_calibration_pose_snapshot(snapshot):
  global _calibration_dirty
  global _calibration_save_job
  global acknowledged_forced_position_target
  global confirmed_position_generation

  if not isinstance(snapshot, CalibrationPoseSnapshot):
    raise TypeError("calibration pose snapshot has an invalid type")
  snapshot_keys = tuple(key for key, _ in snapshot.calibration_values)
  if snapshot_keys != CALIBRATION_POSITION_KEYS:
    raise RuntimeError("calibration pose snapshot keys are invalid")
  if (
    len(snapshot.virtual_angles) != PRIMARY_JOINT_COUNT
    or len(snapshot.step_monitors) != PRIMARY_JOINT_COUNT
    or len(snapshot.step_values) != PRIMARY_JOINT_COUNT
    or len(snapshot.non_primary_entry_values) != (
      len(CALIBRATION_POSITION_KEYS) - PRIMARY_JOINT_COUNT
    )
    or len(snapshot.slider_values) != 9
  ):
    raise RuntimeError("calibration pose snapshot dimensions are invalid")
  if not isinstance(snapshot.resynchronization_required, bool):
    raise RuntimeError("calibration resynchronization snapshot is invalid")
  if not isinstance(snapshot.persistence_dirty, bool):
    raise RuntimeError("calibration persistence snapshot is invalid")

  for key, value in snapshot.calibration_values:
    CAL[key] = value
  RUN['WC'] = snapshot.wrist_config
  RUN['VR_angles'] = list(snapshot.virtual_angles)
  RUN['StepMonitors'] = list(snapshot.step_monitors)
  for axis, value in enumerate(snapshot.step_values, start=1):
    RUN[f'J{axis}StepM'] = value
  confirmed_position_generation = snapshot.confirmed_generation
  with acknowledged_forced_position_lock:
    acknowledged_forced_position_target = snapshot.acknowledged_forced_target
  if snapshot.resynchronization_required:
    controller_position_resynchronization_required.set()
  else:
    controller_position_resynchronization_required.clear()

  restoration_errors = []
  entry_fields, jog_sliders = _calibration_pose_widget_groups()
  primary_joint_count = PRIMARY_JOINT_COUNT
  for entry_field, (_, value) in zip(
    entry_fields[:primary_joint_count],
    snapshot.calibration_values[:primary_joint_count],
  ):
    try:
      _reset_joint_position_entry(entry_field, value)
    except Exception as exc:
      restoration_errors.append(f"pose entry restoration failed: {exc}")
  for entry_field, value in zip(
    entry_fields[primary_joint_count:],
    snapshot.non_primary_entry_values,
  ):
    try:
      entry_field.delete(0, 'end')
      entry_field.insert(0, value)
    except Exception as exc:
      restoration_errors.append(f"pose entry restoration failed: {exc}")
  for jog_slider, value in zip(jog_sliders, snapshot.slider_values):
    try:
      jog_slider.set(value)
    except Exception as exc:
      restoration_errors.append(f"pose slider restoration failed: {exc}")
  try:
    manEntryField.delete(0, 'end')
    manEntryField.insert(0, snapshot.manual_debug)
  except Exception as exc:
    restoration_errors.append(f"manual debug restoration failed: {exc}")

  persistence_changed = (
    _calibration_dirty != snapshot.persistence_dirty
    or _calibration_save_job is not snapshot.persistence_job
  )
  if persistence_changed:
    pending_job = _calibration_save_job
    if pending_job is not None:
      try:
        root.after_cancel(pending_job)
      except Exception as exc:
        restoration_errors.append(
          f"calibration persistence cancellation failed: {exc}"
        )
    _calibration_save_job = None
    _calibration_dirty = True
    try:
      shutdown_started = application_closing.is_set()
      if not isinstance(shutdown_started, bool):
        raise TypeError("application shutdown state must be boolean")
      if not shutdown_started:
        repair_job = root.after(
          CALIBRATION_SAVE_DEBOUNCE_MS,
          _write_pending_calibration,
        )
        if repair_job is None:
          raise RuntimeError(
            "calibration persistence repair scheduler returned no job"
          )
        _calibration_save_job = repair_job
    except Exception as exc:
      restoration_errors.append(
        f"calibration persistence repair scheduling failed: {exc}"
      )

  if restoration_errors:
    raise RuntimeError("; ".join(restoration_errors))
  return True


def _calibration_result_failure_details(failure):
  if not isinstance(failure, BaseException):
    raise TypeError("calibration result failure requires an exception")
  details = " ".join(str(failure).split())
  return details or failure.__class__.__name__


def _handle_calibration_result_application_failure(snapshot, failure):
  details = _calibration_result_failure_details(failure)
  reason = (
    "calibration result application failed after controller "
    f"transmission: {details}"
  )
  try:
    _restore_calibration_pose_snapshot(snapshot)
  except Exception as exc:
    rollback_details = " ".join(str(exc).split()) or exc.__class__.__name__
    reason = f"{reason}; local pose rollback failed: {rollback_details}"
    logger.exception("Unable to restore the pre-command calibration pose")
  try:
    _invalidate_uncertain_controller_calibration(reason)
  except Exception:
    logger.exception(
      "Unable to quarantine a failed calibration result application"
    )
  return details


def _calibration_shutdown_pending():
  if (
    calibration_terminal_response_pending.is_set()
    or calibration_serial_write_committed.is_set()
  ):
    return True
  with calibration_operation_lock:
    return calibration_operation is not None


@contextmanager
def _require_calibration_terminal_response(write_commitment):
  if not isinstance(write_commitment, CalibrationWriteCommitment):
    raise TypeError("calibration exchange requires a write commitment")
  if write_commitment._shared_event is not calibration_serial_write_committed:
    raise TypeError("calibration write commitment is bound to the wrong state")
  response_boundary = CalibrationCancellationBoundary(
    application_closing,
    write_commitment,
  )
  if response_boundary.is_set():
    raise SerialActivityRejected(
      "calibration exchange rejected during application shutdown"
    )
  if not calibration_terminal_owner_lock.acquire(blocking=False):
    raise RuntimeError("another calibration exchange requires a terminal response")
  calibration_terminal_response_pending.set()
  try:
    yield response_boundary
  finally:
    calibration_terminal_response_pending.clear()
    calibration_terminal_owner_lock.release()


def _finish_calibration_operation(operation, succeeded):
  global calibration_operation

  if not isinstance(operation, CalibrationOperation):
    raise TypeError("calibration completion requires a valid operation")
  if not isinstance(succeeded, bool):
    raise TypeError("calibration completion state must be boolean")

  with calibration_operation_lock:
    if calibration_operation is not operation or operation.settled:
      raise RuntimeError("calibration operation ownership changed before completion")
    if (
      operation.worker_active
      or operation.worker_token is not None
      or operation.stage_snapshot is not None
    ):
      raise RuntimeError("calibration operation cannot finish with active stage state")
    operation.settled = True
    calibration_operation = None
    calibration_serial_write_committed.clear()

  cleanup_errors = []
  try:
    if operation.activity_lease.close() is not True:
      cleanup_errors.append("serial activity lease was already released")
  except Exception as exc:
    cleanup_errors.append(f"serial activity cleanup failed: {exc}")
  try:
    if serial_lock.locked():
      serial_lock.release()
    else:
      cleanup_errors.append("controller transport ownership was already released")
  except Exception as exc:
    cleanup_errors.append(f"controller transport cleanup failed: {exc}")

  final_result = succeeded and not cleanup_errors
  try:
    _finish_motion_request(
      operation.request_lease,
      completion_callback=operation.completion_callback,
      succeeded=final_result,
    )
  except Exception as exc:
    cleanup_errors.append(f"motion request cleanup failed: {exc}")
    final_result = False

  if cleanup_errors:
    message = "Calibration cleanup failed: " + "; ".join(cleanup_errors)
    logger.error(message)
    try:
      _set_calibration_status(message, "Alarm.TLabel")
    except Exception:
      logger.exception("Unable to display calibration cleanup failure")
  return final_result


def _run_calibration_stage_safe(
  request_id,
  stage_index,
  worker_token,
  serial_port,
  command,
):
  write_commitment = CalibrationWriteCommitment(
    calibration_serial_write_committed
  )
  try:
    with _require_calibration_terminal_response(
      write_commitment
    ) as response_requirement:
      response = exchange_serial_line_until_cancelled(
        serial_port,
        command,
        response_requirement,
        write_lock=serial_write_lock,
        write_boundary_lock=application_lifecycle_lock,
        write_started_event=write_commitment,
      )
    event = CalibrationWorkerResult(
      worker_token,
      request_id,
      stage_index,
      "completed",
      response,
      None,
      write_commitment.is_set(),
    )
  except Exception as exc:
    message = str(exc).strip() or "calibration exchange failed without details"
    event = CalibrationWorkerResult(
      worker_token,
      request_id,
      stage_index,
      "failed",
      None,
      message,
      write_commitment.is_set(),
    )
  calibration_serial_event_queue.put(event)


def _start_calibration_stage_worker(operation):
  if not isinstance(operation, CalibrationOperation):
    raise TypeError("calibration worker requires a valid operation")
  with application_lifecycle_lock:
    if application_closing.is_set():
      raise SerialActivityRejected(
        "calibration stage rejected during application shutdown"
      )
    with calibration_operation_lock:
      if calibration_operation is not operation or operation.settled:
        raise RuntimeError("calibration worker operation is no longer active")
      if operation.worker_active:
        raise RuntimeError("calibration stage already has an active worker")
      if not 0 <= operation.stage_index < len(operation.stages):
        raise RuntimeError("calibration stage index is out of range")
      stage_index = operation.stage_index
      stage = operation.stages[stage_index]
      stage_snapshot = _capture_calibration_pose_snapshot()
      calibration_serial_write_committed.clear()
      worker_token = object()
      operation.stage_snapshot = stage_snapshot
      operation.worker_token = worker_token
      operation.worker_active = True

  try:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, stage.command)
    _set_calibration_status(
      f"{operation.name.upper()} IN PROGRESS",
      "OK.TLabel",
    )
    thread = threading.Thread(
      target=_run_calibration_stage_safe,
      args=(
        operation.request_id,
        stage_index,
        worker_token,
        operation.serial_port,
        stage.command,
      ),
      daemon=True,
    )
    thread.start()
  except Exception:
    with calibration_operation_lock:
      if calibration_operation is operation and not operation.settled:
        operation.stage_snapshot = None
        operation.worker_token = None
        operation.worker_active = False
    raise
  return True


def _start_calibration_sequence(name, stages, completion_callback=None):
  global calibration_next_request_id
  global calibration_operation

  if not isinstance(name, str) or not name.strip() or name != name.strip():
    raise MotionInputError("calibration request name must be normalized text")
  if isinstance(stages, (str, bytes)):
    raise MotionInputError("calibration stages must be a sequence")
  try:
    stages = tuple(stages)
  except TypeError as exc:
    raise MotionInputError("calibration stages must be a sequence") from exc
  if not stages or not all(isinstance(stage, CalibrationStage) for stage in stages):
    raise MotionInputError("calibration stages must contain validated stages")
  if completion_callback is not None and not callable(completion_callback):
    raise MotionInputError("calibration completion callback must be callable")

  request_lease = _acquire_motion_request(name)
  if request_lease is None:
    message = _motion_request_rejection_message(
      f"{name} not started; another motion request is active"
    )
    _set_calibration_status(message, "Warn.TLabel")
    return False
  if not serial_lock.acquire(blocking=False):
    _finish_motion_request(request_lease)
    message = f"{name} not started; controller transport is busy"
    logger.warning(message)
    _set_calibration_status(message, "Warn.TLabel")
    return False

  activity_lease = None
  operation = None
  try:
    activity_lease = serial_activity_registry.lease("ser")
    serial_port = RUN.get('ser')
    if serial_port is None or not getattr(serial_port, "is_open", False):
      raise ConnectionError("calibration requires an open controller connection")
    if serial_transport_quarantined(serial_port):
      raise SerialTransportQuarantinedError(
        "calibration requires a reconnected controller transport"
      )
    with calibration_operation_lock:
      if calibration_operation is not None:
        raise RuntimeError("another calibration operation is already active")
      if (
        isinstance(calibration_next_request_id, bool)
        or not isinstance(calibration_next_request_id, int)
        or calibration_next_request_id < 0
      ):
        raise RuntimeError("calibration request counter is invalid")
      calibration_next_request_id += 1
      operation = CalibrationOperation(
        calibration_next_request_id,
        name,
        stages,
        request_lease,
        activity_lease,
        serial_port,
        completion_callback,
      )
      calibration_operation = operation
    _start_calibration_stage_worker(operation)
  except Exception as exc:
    if operation is not None:
      try:
        _finish_calibration_operation(operation, False)
      except Exception:
        logger.exception("Unable to clean up rejected calibration operation")
    else:
      if activity_lease is not None:
        try:
          activity_lease.close()
        except Exception:
          logger.exception("Unable to release rejected calibration activity")
      if serial_lock.locked():
        serial_lock.release()
      _finish_motion_request(request_lease)
    message = str(exc).strip() or "calibration request failed without details"
    logger.exception("Unable to start calibration")
    _set_calibration_status(f"Calibration not started: {message}", "Alarm.TLabel")
    return False
  return True


def _claim_calibration_worker_result(event):
  if not isinstance(event, CalibrationWorkerResult):
    raise RuntimeError("calibration worker emitted an invalid event")
  event_type = event.event_type
  request_id = event.request_id
  stage_index = event.stage_index
  response = event.response
  error = event.error
  write_started = event.write_started
  if event_type not in ("completed", "failed"):
    raise RuntimeError("calibration worker emitted an unknown event type")
  if (
    isinstance(request_id, bool)
    or not isinstance(request_id, int)
    or request_id <= 0
  ):
    raise RuntimeError("calibration worker emitted an invalid request ID")
  if (
    isinstance(stage_index, bool)
    or not isinstance(stage_index, int)
    or stage_index < 0
  ):
    raise RuntimeError("calibration worker emitted an invalid stage index")
  if not isinstance(write_started, bool):
    raise RuntimeError("calibration worker emitted an invalid write-start state")
  if event_type == "completed":
    if (
      not write_started
      or not isinstance(response, str)
      or error is not None
    ):
      raise RuntimeError("calibration worker emitted an invalid success result")
  elif (
    response is not None
    or not isinstance(error, str)
    or not error.strip()
    or error != error.strip()
  ):
    raise RuntimeError("calibration worker emitted an invalid failure result")

  with calibration_operation_lock:
    operation = calibration_operation
    if operation is None or operation.settled:
      raise RuntimeError("calibration result has no active operation")
    if (
      operation.request_id != request_id
      or operation.stage_index != stage_index
      or operation.worker_token is not event.worker_token
      or not operation.worker_active
      or operation.stage_snapshot is None
    ):
      raise RuntimeError("calibration result does not own the active stage")
    stage_snapshot = operation.stage_snapshot
    operation.stage_snapshot = None
    operation.worker_token = None
    operation.worker_active = False
  return (
    operation,
    operation.stages[stage_index],
    stage_snapshot,
    event_type,
    response,
    error,
    write_started,
  )


def _settle_rejected_calibration_worker_result(event, failure):
  if not isinstance(failure, BaseException):
    raise TypeError("calibration result rejection requires an exception")
  if not isinstance(event, CalibrationWorkerResult):
    return False

  with calibration_operation_lock:
    operation = calibration_operation
    if operation is None or operation.settled or not operation.worker_active:
      return False
    if event.worker_token is not operation.worker_token:
      return False
    operation.stage_snapshot = None
    operation.worker_token = None
    operation.worker_active = False

  details = str(failure).strip() or failure.__class__.__name__
  reason = f"calibration worker result rejected: {details}"
  try:
    _invalidate_uncertain_controller_calibration(reason)
  except Exception:
    logger.exception("Unable to quarantine a rejected calibration result")
  try:
    return _finish_calibration_operation(operation, False)
  except Exception:
    logger.exception("Unable to settle a rejected calibration result")
    return False


def _apply_calibration_worker_result(event):
  (
    operation,
    stage,
    stage_snapshot,
    event_type,
    response,
    error,
    write_started,
  ) = _claim_calibration_worker_result(event)
  try:
    cmdRecEntryField.delete(0, 'end')
    cmdRecEntryField.insert(0, response if event_type == "completed" else error)
    if event_type == "failed":
      logger.error("Calibration controller exchange failed: %s", error)
    succeeded = _record_calibration_response(
      response,
      stage.success_message,
      stage.failure_message,
      update_virtual=stage.update_virtual,
      controller_write_started=write_started,
      uncertainty_reason=(
        "calibration response failed after controller transmission"
        if event_type == "failed" and write_started
        else None
      ),
    )
    if not succeeded:
      return _finish_calibration_operation(operation, False)

    with calibration_operation_lock:
      if calibration_operation is not operation or operation.settled:
        raise RuntimeError("calibration operation changed while applying a result")
      operation.stage_index += 1
      complete = operation.stage_index == len(operation.stages)
    if complete:
      return _finish_calibration_operation(operation, True)
    return _start_calibration_stage_worker(operation)
  except Exception as exc:
    details = _calibration_result_failure_details(exc)
    if write_started:
      details = _handle_calibration_result_application_failure(
        stage_snapshot,
        exc,
      )
    with calibration_operation_lock:
      can_finish = (
        calibration_operation is operation
        and not operation.settled
        and not operation.worker_active
      )
    if can_finish:
      try:
        _finish_calibration_operation(operation, False)
      except Exception:
        logger.exception("Unable to clean up a failed calibration result")
    message = details
    try:
      _set_calibration_status(
        f"Calibration result failed to settle: {message}",
        "Alarm.TLabel",
      )
    except Exception:
      logger.exception("Unable to display calibration settlement failure")
    raise


def _poll_calibration_events():
  try:
    while True:
      try:
        event = calibration_serial_event_queue.get_nowait()
      except Empty:
        break
      try:
        _apply_calibration_worker_result(event)
      except Exception as exc:
        logger.exception(
          "Unable to apply a calibration result on the Tk event thread"
        )
        _settle_rejected_calibration_worker_result(event, exc)
  finally:
    if not application_closing.is_set():
      root.after(25, _poll_calibration_events)


def _execute_calibration_command(
  command,
  success_message,
  failure_message,
  *,
  update_virtual=True,
):
  try:
    pose_snapshot = _capture_calibration_pose_snapshot()
  except Exception:
    logger.exception("Unable to capture the pre-command calibration pose")
    return False
  write_commitment = CalibrationWriteCommitment(
    calibration_serial_write_committed
  )
  calibration_serial_write_committed.clear()
  response = None
  uncertainty_reason = None
  try:
    try:
      with _require_calibration_terminal_response(
        write_commitment
      ) as response_requirement:
        cmdSentEntryField.delete(0, 'end')
        cmdSentEntryField.insert(0, command)
        response = exchange_serial_line_until_cancelled(
          RUN.get('ser'),
          command,
          response_requirement,
          write_lock=serial_write_lock,
          write_boundary_lock=application_lifecycle_lock,
          write_started_event=write_commitment,
        )
    except Exception:
      if write_commitment.is_set():
        uncertainty_reason = (
          "calibration response failed after controller transmission"
        )
      logger.exception("Calibration controller exchange failed")

    try:
      cmdRecEntryField.delete(0, 'end')
      cmdRecEntryField.insert(0, "" if response is None else response)
      return _record_calibration_response(
        response,
        success_message,
        failure_message,
        update_virtual=update_virtual,
        controller_write_started=write_commitment.is_set(),
        uncertainty_reason=uncertainty_reason,
      )
    except Exception as exc:
      if write_commitment.is_set():
        _handle_calibration_result_application_failure(
          pose_snapshot,
          exc,
        )
      logger.exception("Unable to apply calibration controller result")
      return False
  finally:
    calibration_serial_write_committed.clear()


@_synchronous_motion_request("Automatic calibration")
@_tracked_serial_operation("ser")
def _run_program_calibration_all():
  if not _calibration_available():
    return False
  first_stage = tuple(
    CAL[f'J{joint}CalStatVal'].get()
    for joint in range(1, 10)
  )
  second_stage = tuple(
    _binary_controller_flag(
      CAL[f'J{joint}CalStatVal2'].get(),
      f"J{joint} second-stage calibration selection",
    )
    for joint in range(1, 10)
  )
  first_command = _prepare_calibration_command(first_stage)
  second_command = (
    _prepare_calibration_command(second_stage)
    if sum(second_stage) > 0
    else None
  )
  if not _execute_calibration_command(
    first_command,
    "Auto Calibration Stage 1 Successful",
    "Auto Calibration Stage 1 Failed - See Log",
  ):
    return False

  if second_command is None:
    return True
  return _execute_calibration_command(
    second_command,
    "Auto Calibration Stage 2 Successful",
    "Auto Calibration Stage 2 Failed - See Log",
  )


@_synchronous_motion_request("J1 calibration")
@_tracked_serial_operation("ser")
def _run_program_calibration_j1():
  if not _calibration_available():
    return False
  command = _prepare_calibration_command((1, 0, 0, 0, 0, 0, 0, 0, 0))
  return _execute_calibration_command(
    command,
    "J1 Calibrated Successfully",
    "J1 Calibration Failed",
  )


@_synchronous_motion_request("J2 calibration")
@_tracked_serial_operation("ser")
def _run_program_calibration_j2():
  if not _calibration_available():
    return False
  command = _prepare_calibration_command((0, 1, 0, 0, 0, 0, 0, 0, 0))
  return _execute_calibration_command(
    command,
    "J2 Calibrated Successfully",
    "J2 Calibration Failed",
  )


@_synchronous_motion_request("J3 calibration")
@_tracked_serial_operation("ser")
def _run_program_calibration_j3():
  if not _calibration_available():
    return False
  command = _prepare_calibration_command((0, 0, 1, 0, 0, 0, 0, 0, 0))
  return _execute_calibration_command(
    command,
    "J3 Calibrated Successfully",
    "J3 Calibration Failed",
  )


@_synchronous_motion_request("J4 calibration")
@_tracked_serial_operation("ser")
def _run_program_calibration_j4():
  if not _calibration_available():
    return False
  command = _prepare_calibration_command((0, 0, 0, 1, 0, 0, 0, 0, 0))
  return _execute_calibration_command(
    command,
    "J4 Calibrated Successfully",
    "J4 Calibration Failed",
  )


@_synchronous_motion_request("J5 calibration")
@_tracked_serial_operation("ser")
def _run_program_calibration_j5():
  if not _calibration_available():
    return False
  command = _prepare_calibration_command((0, 0, 0, 0, 1, 0, 0, 0, 0))
  return _execute_calibration_command(
    command,
    "J5 Calibrated Successfully",
    "J5 Calibration Failed",
  )


@_synchronous_motion_request("J6 calibration")
@_tracked_serial_operation("ser")
def _run_program_calibration_j6():
  if not _calibration_available():
    return False
  command = _prepare_calibration_command((0, 0, 0, 0, 0, 1, 0, 0, 0))
  return _execute_calibration_command(
    command,
    "J6 Calibrated Successfully",
    "J6 Calibration Failed",
  )


@_synchronous_motion_request("J7 calibration")
@_tracked_serial_operation("ser")
def _run_program_calibration_j7():
  if not _calibration_available():
    return False
  command = _prepare_calibration_command((0, 0, 0, 0, 0, 0, 1, 0, 0))
  return _execute_calibration_command(
    command,
    "J7 Calibrated Successfully",
    "J7 Calibration Failed",
    update_virtual=False,
  )


@_synchronous_motion_request("J8 calibration")
@_tracked_serial_operation("ser")
def _run_program_calibration_j8():
  if not _calibration_available():
    return False
  command = _prepare_calibration_command((0, 0, 0, 0, 0, 0, 0, 1, 0))
  return _execute_calibration_command(
    command,
    "J8 Calibrated Successfully",
    "J8 Calibration Failed",
    update_virtual=False,
  )


@_synchronous_motion_request("J9 calibration")
@_tracked_serial_operation("ser")
def _run_program_calibration_j9():
  if not _calibration_available():
    return False
  command = _prepare_calibration_command((0, 0, 0, 0, 0, 0, 0, 0, 1))
  return _execute_calibration_command(
    command,
    "J9 Calibrated Successfully",
    "J9 Calibration Failed",
    update_virtual=False,
  )


def startCalRobotAll():
  if not _calibration_available():
    return False
  try:
    first_stage = tuple(
      CAL[f'J{joint}CalStatVal'].get()
      for joint in range(1, 10)
    )
    second_stage = tuple(
      _binary_controller_flag(
        CAL[f'J{joint}CalStatVal2'].get(),
        f"J{joint} second-stage calibration selection",
      )
      for joint in range(1, 10)
    )
    stages = [
      CalibrationStage(
        _prepare_calibration_command(first_stage),
        "Auto Calibration Stage 1 Successful",
        "Auto Calibration Stage 1 Failed - See Log",
      )
    ]
    if sum(second_stage) > 0:
      stages.append(
        CalibrationStage(
          _prepare_calibration_command(second_stage),
          "Auto Calibration Stage 2 Successful",
          "Auto Calibration Stage 2 Failed - See Log",
        )
      )
    return _start_calibration_sequence("Automatic calibration", stages)
  except Exception as exc:
    message = str(exc).strip() or "automatic calibration input is invalid"
    logger.exception("Unable to prepare automatic calibration")
    _set_calibration_status(f"Calibration not started: {message}", "Alarm.TLabel")
    return False


def _start_single_joint_calibration(joint):
  if not _calibration_available():
    return False
  if isinstance(joint, bool) or not isinstance(joint, int) or not 1 <= joint <= 9:
    raise MotionInputError("calibration joint must be between 1 and 9")
  try:
    selections = tuple(1 if axis == joint else 0 for axis in range(1, 10))
    stage = CalibrationStage(
      _prepare_calibration_command(selections),
      f"J{joint} Calibrated Successfully",
      f"J{joint} Calibration Failed",
      update_virtual=joint <= 6,
    )
    return _start_calibration_sequence(f"J{joint} calibration", (stage,))
  except Exception as exc:
    message = str(exc).strip() or f"J{joint} calibration input is invalid"
    logger.exception("Unable to prepare J%s calibration", joint)
    _set_calibration_status(f"Calibration not started: {message}", "Alarm.TLabel")
    return False


def startCalRobotJ1():
  return _start_single_joint_calibration(1)


def startCalRobotJ2():
  return _start_single_joint_calibration(2)


def startCalRobotJ3():
  return _start_single_joint_calibration(3)


def startCalRobotJ4():
  return _start_single_joint_calibration(4)


def startCalRobotJ5():
  return _start_single_joint_calibration(5)


def startCalRobotJ6():
  return _start_single_joint_calibration(6)


def startCalRobotJ7():
  return _start_single_joint_calibration(7)


def startCalRobotJ8():
  return _start_single_joint_calibration(8)


def startCalRobotJ9():
  return _start_single_joint_calibration(9)
	



def correctPos():
  return _request_controller_correction()

@_tracked_serial_operation("ser")
def requestPos():
  response = _exchange_serial_line("RP\n")
  return _apply_controller_position_response(response)


def _collect_update_parameter_values():
  values = {
    'TFx': TFxEntryField.get(),
    'TFy': TFyEntryField.get(),
    'TFz': TFzEntryField.get(),
    'TFrz': TFrzEntryField.get(),
    'TFry': TFryEntryField.get(),
    'TFrx': TFrxEntryField.get(),
  }
  field_groups = (
    ('MotDir', (J1MotDirEntryField, J2MotDirEntryField, J3MotDirEntryField, J4MotDirEntryField, J5MotDirEntryField, J6MotDirEntryField, J7MotDirEntryField, J8MotDirEntryField, J9MotDirEntryField)),
    ('CalDir', (J1CalDirEntryField, J2CalDirEntryField, J3CalDirEntryField, J4CalDirEntryField, J5CalDirEntryField, J6CalDirEntryField, J7CalDirEntryField, J8CalDirEntryField, J9CalDirEntryField)),
    ('PosLim', (J1PosLimEntryField, J2PosLimEntryField, J3PosLimEntryField, J4PosLimEntryField, J5PosLimEntryField, J6PosLimEntryField)),
    ('NegLim', (J1NegLimEntryField, J2NegLimEntryField, J3NegLimEntryField, J4NegLimEntryField, J5NegLimEntryField, J6NegLimEntryField)),
    ('StepDeg', (J1StepDegEntryField, J2StepDegEntryField, J3StepDegEntryField, J4StepDegEntryField, J5StepDegEntryField, J6StepDegEntryField)),
    ('ΘDHpar', (J1ΘEntryField, J2ΘEntryField, J3ΘEntryField, J4ΘEntryField, J5ΘEntryField, J6ΘEntryField)),
    ('αDHpar', (J1αEntryField, J2αEntryField, J3αEntryField, J4αEntryField, J5αEntryField, J6αEntryField)),
    ('dDHpar', (J1dEntryField, J2dEntryField, J3dEntryField, J4dEntryField, J5dEntryField, J6dEntryField)),
    ('aDHpar', (J1aEntryField, J2aEntryField, J3aEntryField, J4aEntryField, J5aEntryField, J6aEntryField)),
    ('DriveMS', (J1DriveMSEntryField, J2DriveMSEntryField, J3DriveMSEntryField, J4DriveMSEntryField, J5DriveMSEntryField, J6DriveMSEntryField)),
    ('EncCPR', (J1EncCPREntryField, J2EncCPREntryField, J3EncCPREntryField, J4EncCPREntryField, J5EncCPREntryField, J6EncCPREntryField)),
  )
  for suffix, fields in field_groups:
    for axis, field in enumerate(fields, start=1):
      value = field.get()
      if suffix in ('MotDir', 'CalDir'):
        value = _binary_controller_flag(
          value,
          f"J{axis} {suffix}",
        )
      elif suffix in ('DriveMS', 'EncCPR'):
        value = int(value)
      values[f'J{axis}{suffix}'] = value
  return values


def _robot_joint_calibration_from_values(values):
  return ControllerJointCalibration(
    negative_limits=tuple(
      finite_number(values[f'J{axis}NegLim'], f'J{axis} negative limit')
      for axis in range(1, 7)
    ) + (0.0, 0.0, 0.0),
    positive_limits=tuple(
      finite_number(values[f'J{axis}PosLim'], f'J{axis} positive limit')
      for axis in range(1, 7)
    ) + (0.0, 0.0, 0.0),
    steps_per_unit=tuple(
      finite_number(values[f'J{axis}StepDeg'], f'J{axis} steps per degree')
      for axis in range(1, 7)
    ) + (1.0, 1.0, 1.0),
  )


def _prepare_update_parameters_from_values(source_values):
  if not isinstance(source_values, dict):
    raise MotionInputError("update-parameter values must be a dictionary")
  try:
    values = {
      key: source_values[key]
      for key in ('TFx', 'TFy', 'TFz', 'TFrz', 'TFry', 'TFrx')
    }
    for suffix, axis_count in (
      ('MotDir', 9),
      ('CalDir', 9),
      ('PosLim', 6),
      ('NegLim', 6),
      ('StepDeg', 6),
      ('DriveMS', 6),
      ('EncCPR', 6),
      ('ΘDHpar', 6),
      ('αDHpar', 6),
      ('dDHpar', 6),
      ('aDHpar', 6),
    ):
      for axis in range(1, axis_count + 1):
        key = f'J{axis}{suffix}'
        values[key] = source_values[key]
  except KeyError as exc:
    raise MotionInputError(
      f"update-parameter values are missing {exc.args[0]}"
    ) from exc

  _validated_native_kinematics_rotations(values)
  for axis in range(1, 10):
    for suffix in ('MotDir', 'CalDir'):
      values[f'J{axis}{suffix}'] = _binary_controller_flag(
        values[f'J{axis}{suffix}'],
        f"J{axis} {suffix}",
      )
  _robot_joint_calibration_from_values(values)
  encoder_multipliers = []
  for axis in range(1, 7):
    drive_key = f'J{axis}DriveMS'
    encoder_key = f'J{axis}EncCPR'
    drive_microsteps = finite_number(
      values[drive_key],
      f"J{axis} drive microsteps",
    )
    encoder_counts = finite_number(
      values[encoder_key],
      f"J{axis} encoder counts",
    )
    if (
      drive_microsteps <= 0
      or encoder_counts <= 0
      or not drive_microsteps.is_integer()
      or not encoder_counts.is_integer()
    ):
      raise MotionInputError(
        f"J{axis} encoder counts and drive microsteps must be positive integers"
      )
    drive_microsteps = int(drive_microsteps)
    encoder_counts = int(encoder_counts)
    values[drive_key] = drive_microsteps
    values[encoder_key] = encoder_counts
    encoder_multipliers.append(controller_ratio(
      encoder_counts,
      drive_microsteps,
      f"J{axis} encoder multiplier",
    ))

  command_fields = list(zip(
    tuple("ABCDEF"),
    tuple(values[key] for key in ('TFx', 'TFy', 'TFz', 'TFrz', 'TFry', 'TFrx')),
  ))
  command_fields.extend(zip(
    tuple("GHIJKLMNO"),
    tuple(values[f'J{axis}MotDir'] for axis in range(1, 10)),
  ))
  command_fields.extend(zip(
    tuple("PQRSTUVWX"),
    tuple(values[f'J{axis}CalDir'] for axis in range(1, 10)),
  ))
  command_fields.extend(zip(
    ("Y", "Z", "a", "b", "c", "d", "e", "f", "g", "h", "i", "j"),
    tuple(
      values[f'J{axis}{limit}Lim']
      for axis in range(1, 7)
      for limit in ("Pos", "Neg")
    ),
  ))
  command_fields.extend(zip(
    tuple("klmnop"),
    tuple(values[f'J{axis}StepDeg'] for axis in range(1, 7)),
  ))
  command_fields.extend(zip(tuple("qrstuv"), encoder_multipliers))
  command_fields.extend(zip(
    ("w", "x", "y", "z", "!", "@"),
    tuple(values[f'J{axis}ΘDHpar'] for axis in range(1, 7)),
  ))
  command_fields.extend(zip(
    ("#", "$", "%", "^", "&", "*"),
    tuple(values[f'J{axis}αDHpar'] for axis in range(1, 7)),
  ))
  command_fields.extend(zip(
    ("(", ")", "+", "=", ",", "_"),
    tuple(values[f'J{axis}dDHpar'] for axis in range(1, 7)),
  ))
  command_fields.extend(zip(
    ("<", ">", "?", "{", "}", "~"),
    tuple(values[f'J{axis}aDHpar'] for axis in range(1, 7)),
  ))
  return values, _build_startup_numeric_command("UP", command_fields)


def _prepare_update_parameters():
  return _prepare_update_parameters_from_values(
    _collect_update_parameter_values()
  )


def _apply_joint_limit_widgets(values):
  negative_labels = (J1negLimLab, J2negLimLab, J3negLimLab, J4negLimLab, J5negLimLab, J6negLimLab)
  positive_labels = (J1posLimLab, J2posLimLab, J3posLimLab, J4posLimLab, J5posLimLab, J6posLimLab)
  sliders = (J1jogslide, J2jogslide, J3jogslide, J4jogslide, J5jogslide, J6jogslide)
  slider_callbacks = (J1sliderUpdate, J2sliderUpdate, J3sliderUpdate, J4sliderUpdate, J5sliderUpdate, J6sliderUpdate)
  for axis, widgets in enumerate(
    zip(negative_labels, positive_labels, sliders, slider_callbacks),
    start=1,
  ):
    negative_label, positive_label, slider, callback = widgets
    negative = finite_number(values[f'J{axis}NegLim'], f'J{axis} negative limit')
    positive = finite_number(values[f'J{axis}PosLim'], f'J{axis} positive limit')
    negative_label.config(text=f"-{values[f'J{axis}NegLim']}", style="Jointlim.TLabel")
    positive_label.config(text=values[f'J{axis}PosLim'], style="Jointlim.TLabel")
    slider.config(
      from_=-negative,
      to=positive,
      length=180,
      orient=HORIZONTAL,
      command=callback,
    )


def _apply_update_parameter_values(values):
  _set_cpp_kinematics_from_values(values)
  CAL.update(values)
  _apply_joint_limit_widgets(values)
  return True


@_tracked_serial_operation("ser")
def _exchange_controller_calibration_acknowledgement(
  command,
  write_started_event=None,
):
  serial_port = RUN.get('ser')
  try:
    write_serial_control(
      serial_port,
      command,
      write_lock=serial_write_lock,
      reset_input=True,
      write_started_event=write_started_event,
    )
    return read_serial_exact_response(
      serial_port,
      b"Done",
      SERIAL_STARTUP_READ_TIMEOUT_SECONDS,
    ) == "Done"
  finally:
    if (
      RUN.get('ser') is serial_port
      and not getattr(serial_port, "is_open", False)
    ):
      RUN['ser'] = None


def _transmit_update_parameters(command, write_started_event=None):
  return _exchange_controller_calibration_acknowledgement(
    command,
    write_started_event,
  )


def _preflight_controller_calibration_transport():
  serial_port = RUN.get('ser')
  if serial_port is None or not getattr(serial_port, "is_open", False):
    raise ConnectionError("controller serial connection is not open")
  if serial_transport_quarantined(serial_port):
    raise SerialTransportQuarantinedError(
      "controller serial connection is quarantined; reconnect required"
    )
  return serial_port


def _restore_prewrite_calibration(snapshot, context):
  try:
    _restore_controller_calibration(snapshot)
  except Exception:
    logger.exception("Unable to restore calibration after %s", context)
    return _invalidate_uncertain_controller_calibration(
      f"local calibration rollback failed after {context}"
    )
  return False


def _apply_single_calibration_transaction(
  values,
  apply_values,
  command,
  transmit_command,
  context,
):
  try:
    _preflight_controller_calibration_transport()
  except Exception:
    logger.exception("Calibration transport preflight failed during %s", context)
    return False

  snapshot = dict(CAL)
  try:
    apply_values(values)
  except Exception:
    logger.exception("Local calibration application failed during %s", context)
    return _restore_prewrite_calibration(snapshot, context)

  write_started = threading.Event()
  try:
    acknowledged = transmit_command(command, write_started)
  except Exception:
    logger.exception("Controller calibration transmission failed during %s", context)
    if write_started.is_set():
      return _invalidate_uncertain_controller_calibration(
        f"controller calibration became uncertain during {context}"
      )
    return _restore_prewrite_calibration(snapshot, context)
  if acknowledged is not True:
    logger.error(
      "Controller calibration returned a non-true acknowledgement during %s",
      context,
    )
    if write_started.is_set():
      return _invalidate_uncertain_controller_calibration(
        f"controller calibration acknowledgement was invalid during {context}"
      )
    return _restore_prewrite_calibration(snapshot, context)
  return True


@_synchronous_motion_request(
  "Update controller parameters",
  requires_kinematics=False,
)
@_tracked_serial_operation(
  "ser",
  operation_required=_main_serial_transmit_required,
)
def updateParams(transmit=True):
  if not isinstance(transmit, bool):
    raise TypeError("update-parameters transmit flag must be boolean")
  values, command = _prepare_update_parameters()
  if not transmit:
    snapshot = dict(CAL)
    try:
      _apply_update_parameter_values(values)
    except Exception:
      _restore_controller_calibration(snapshot)
      raise
    return command
  return _apply_single_calibration_transaction(
    values,
    _apply_update_parameter_values,
    command,
    _transmit_update_parameters,
    "update-parameters application",
  )


def _collect_external_axis_values():
  fields = (
    (7, axis7lengthEntryField, axis7rotEntryField, axis7stepsEntryField),
    (8, axis8lengthEntryField, axis8rotEntryField, axis8stepsEntryField),
    (9, axis9lengthEntryField, axis9rotEntryField, axis9stepsEntryField),
  )
  values = {}
  for axis, length_field, rotation_field, steps_field in fields:
    length_key = 'J7PosLim' if axis == 7 else f'J{axis}length'
    values[length_key] = finite_number(length_field.get(), f'J{axis} length')
    values[f'J{axis}rotation'] = finite_number(
      rotation_field.get(),
      f'J{axis} rotation',
    )
    values[f'J{axis}steps'] = finite_number(steps_field.get(), f'J{axis} steps')
  return values


def _prepare_external_axis_parameters_from_values(
  source_values,
  base_values=None,
):
  if not isinstance(source_values, dict):
    raise MotionInputError("external-axis values must be a dictionary")
  try:
    values = {
      key: finite_number(source_values[key], label)
      for key, label in (
        ('J7PosLim', 'J7 length'),
        ('J7rotation', 'J7 rotation'),
        ('J7steps', 'J7 steps'),
        ('J8length', 'J8 length'),
        ('J8rotation', 'J8 rotation'),
        ('J8steps', 'J8 steps'),
        ('J9length', 'J9 length'),
        ('J9rotation', 'J9 rotation'),
        ('J9steps', 'J9 steps'),
      )
    }
  except KeyError as exc:
    raise MotionInputError(
      f"external-axis values are missing {exc.args[0]}"
    ) from exc
  calibration_values = dict(CAL if base_values is None else base_values)
  calibration_values.update(values)
  _controller_joint_calibration_from_values(calibration_values)
  command = _build_startup_numeric_command(
    "CE",
    zip(
      tuple("ABCDEFGHI"),
      (
        values['J7PosLim'], values['J7rotation'], values['J7steps'],
        values['J8length'], values['J8rotation'], values['J8steps'],
        values['J9length'], values['J9rotation'], values['J9steps'],
      ),
    ),
  )
  return values, command


def _prepare_external_axis_parameters(base_values=None):
  return _prepare_external_axis_parameters_from_values(
    _collect_external_axis_values(),
    base_values,
  )


def _apply_external_axis_values(values):
  CAL.update(values)
  negative_labels = (J7negLimLab, J8negLimLab, J9negLimLab)
  positive_labels = (J7posLimLab, J8posLimLab, J9posLimLab)
  sliders = (J7jogslide, J8jogslide, J9jogslide)
  slider_callbacks = (J7sliderUpdate, J8sliderUpdate, J9sliderUpdate)
  lengths = (values['J7PosLim'], values['J8length'], values['J9length'])
  for negative_label, positive_label, slider, callback, length in zip(
    negative_labels,
    positive_labels,
    sliders,
    slider_callbacks,
    lengths,
  ):
    negative_label.config(text="0", style="Jointlim.TLabel")
    positive_label.config(text=str(length), style="Jointlim.TLabel")
    slider.config(
      from_=0,
      to=length,
      length=125,
      orient=HORIZONTAL,
      command=callback,
    )
  return True


def _transmit_external_axis_parameters(command, write_started_event=None):
  return _exchange_controller_calibration_acknowledgement(
    command,
    write_started_event,
  )


def _prepare_controller_calibration():
  update_values, update_command = _prepare_update_parameters()
  merged_values = dict(CAL)
  merged_values.update(update_values)
  external_values, external_command = _prepare_external_axis_parameters(
    merged_values,
  )
  return update_values, update_command, external_values, external_command


def _apply_controller_calibration(update_values, external_values):
  snapshot = dict(CAL)
  try:
    _apply_update_parameter_values(update_values)
    _apply_external_axis_values(external_values)
  except Exception:
    _restore_controller_calibration(snapshot)
    raise
  return True

@_synchronous_motion_request(
  "Apply external-axis calibration",
  requires_kinematics=False,
)
@_tracked_serial_operation(
  "ser",
  operation_required=_main_serial_transmit_required,
)
def calExtAxis(transmit=True):
  if not isinstance(transmit, bool):
    raise TypeError("external-axis transmit flag must be boolean")
  values, command = _prepare_external_axis_parameters()
  if not transmit:
    snapshot = dict(CAL)
    try:
      _apply_external_axis_values(values)
    except Exception:
      _restore_controller_calibration(snapshot)
      raise
    return command
  return _apply_single_calibration_transaction(
    values,
    _apply_external_axis_values,
    command,
    _transmit_external_axis_parameters,
    "external-axis application",
  )

@_synchronous_motion_request("Zero external axis 7")
@_tracked_serial_operation("ser")
def zeroAxis7():
  command = "Z7"+"\n"
  response = _exchange_legacy_main_command(command)
  if not _apply_controller_position_response(response):
    return False
  almStatusLab.config(text="J7 Calibration Forced to Zero", style="Warn.TLabel")
  almStatusLab2.config(text="J7 Calibration Forced to Zero", style="Warn.TLabel")
  message = "J7 Calibration Forced to Zero - this is for commissioning and testing - be careful!"
  #Curtime = datetime.now().strftime("%B %d %Y - %I:%M%p")
  #tab8.ElogView.insert(END, Curtime+" - "+message)
  logger.warning(message)
  value=tab8.ElogView.get(0,END)
  pickle.dump(value,open("ErrorLog","wb"))  
  return True

@_synchronous_motion_request("Zero external axis 8")
@_tracked_serial_operation("ser")
def zeroAxis8():
  command = "Z8"+"\n"
  response = _exchange_legacy_main_command(command)
  if not _apply_controller_position_response(response):
    return False
  almStatusLab.config(text="J8 Calibration Forced to Zero", style="Warn.TLabel")
  almStatusLab2.config(text="J8 Calibration Forced to Zero", style="Warn.TLabel")
  message = "J8 Calibration Forced to Zero - this is for commissioning and testing - be careful!"
  #Curtime = datetime.now().strftime("%B %d %Y - %I:%M%p")
  #tab8.ElogView.insert(END, Curtime+" - "+message)
  logger.warning(message)
  value=tab8.ElogView.get(0,END)
  pickle.dump(value,open("ErrorLog","wb"))  
  return True

@_synchronous_motion_request("Zero external axis 9")
@_tracked_serial_operation("ser")
def zeroAxis9():
  command = "Z9"+"\n"
  response = _exchange_legacy_main_command(command)
  if not _apply_controller_position_response(response):
    return False
  almStatusLab.config(text="J9 Calibration Forced to Zero", style="Warn.TLabel")
  almStatusLab2.config(text="J9 Calibration Forced to Zero", style="Warn.TLabel")
  message = "J9 Calibration Forced to Zero - this is for commissioning and testing - be careful!"
  #Curtime = datetime.now().strftime("%B %d %Y - %I:%M%p")
  #tab8.ElogView.insert(END, Curtime+" - "+message)
  logger.warning(message)
  value=tab8.ElogView.get(0,END)
  pickle.dump(value,open("ErrorLog","wb"))  
  return True


def _acknowledged_forced_position_target_value():
  with acknowledged_forced_position_lock:
    if acknowledged_forced_position_target is None:
      return None
    return tuple(acknowledged_forced_position_target)


def _record_acknowledged_forced_position_target(target):
  global acknowledged_forced_position_target

  if isinstance(target, (str, bytes)):
    raise MotionInputError("acknowledged forced position must be a numeric sequence")
  try:
    target = tuple(target)
  except TypeError as exc:
    raise MotionInputError(
      "acknowledged forced position must be a numeric sequence"
    ) from exc
  if len(target) != 9:
    raise MotionInputError("acknowledged forced position must contain 9 values")
  target = tuple(
    finite_number(value, f"acknowledged J{axis} forced position")
    for axis, value in enumerate(target, start=1)
  )
  with acknowledged_forced_position_lock:
    acknowledged_forced_position_target = target
  controller_position_resynchronization_required.set()
  _invalidate_joint_motion_state(
    "controller acknowledged a forced position; position recovery is required"
  )
  return target


def _clear_acknowledged_forced_position_target():
  global acknowledged_forced_position_target

  with acknowledged_forced_position_lock:
    acknowledged_forced_position_target = None


def _prepare_position_command(calibration_values=None):
  positions = _acknowledged_forced_position_target_value()
  if positions is None:
    positions = _current_joint_positions()
  calibration = (
    _current_controller_joint_calibration()
    if calibration_values is None
    else _controller_joint_calibration_from_values(calibration_values)
  )
  calibration.validate_positions(positions)
  return _build_startup_numeric_command(
    "SP",
    zip(tuple("ABCDEFGHI"), positions),
  )


def _prepare_forced_position_request(primary_positions):
  if isinstance(primary_positions, (str, bytes)):
    raise MotionInputError("forced primary positions must be a numeric sequence")
  try:
    primary_positions = tuple(primary_positions)
  except TypeError as exc:
    raise MotionInputError(
      "forced primary positions must be a numeric sequence"
    ) from exc
  if len(primary_positions) != 6:
    raise MotionInputError("forced primary positions must contain 6 values")
  positions = primary_positions + _current_joint_positions()[6:]
  calibration = _current_controller_joint_calibration()
  normalized = calibration.validate_positions(positions)
  encoded_target = tuple(
    float(controller_protocol_decimal(value, f"J{axis} forced position"))
    for axis, value in enumerate(normalized, start=1)
  )
  calibration.validate_positions(encoded_target)
  command = _build_startup_numeric_command(
    "SP",
    zip(tuple("ABCDEFGHI"), encoded_target),
  )
  return command, encoded_target


def _prepare_forced_position_command(primary_positions):
  command, _ = _prepare_forced_position_request(primary_positions)
  return command


def _force_controller_position(primary_positions):
  command, target = _prepare_forced_position_request(primary_positions)
  if _exchange_position_acknowledgement(command) is not True:
    return False
  try:
    _record_acknowledged_forced_position_target(target)
    return requestPos() is True
  except Exception as exc:
    message = f"Forced controller position requires recovery: {exc}"
    logger.exception(message)
    almStatusLab.config(text=message, style="Alarm.TLabel")
    almStatusLab2.config(text=message, style="Alarm.TLabel")
    return False


@_tracked_serial_operation("ser")
def _exchange_position_acknowledgement(command):
  command = _validated_startup_command(command, "SP")
  serial_port = RUN.get('ser')
  write_started = threading.Event()
  try:
    write_serial_control(
      serial_port,
      command,
      write_lock=serial_write_lock,
      reset_input=True,
      write_started_event=write_started,
    )
    response = read_serial_line_response(
      serial_port,
      SERIAL_STARTUP_READ_TIMEOUT_SECONDS,
      accepted_responses=("Done",),
    )
    if response != "Done":
      raise ProtocolResponseError(
        "controller returned an invalid set-position acknowledgement"
      )
    return True
  except Exception:
    if write_started.is_set():
      try:
        _invalidate_joint_motion_state(
          "set-position acknowledgement failed after controller transmission"
        )
      except Exception:
        logger.exception(
          "Unable to invalidate motion after set-position acknowledgement failure"
        )
    raise
  finally:
    if (
      RUN.get('ser') is serial_port
      and not getattr(serial_port, "is_open", False)
    ):
      RUN['ser'] = None


@_synchronous_motion_request("Set controller position")
@_tracked_serial_operation(
  "ser",
  operation_required=_main_serial_transmit_required,
)
def sendPos(transmit=True):
  if not isinstance(transmit, bool):
    raise TypeError("send-position transmit flag must be boolean")
  command = _prepare_position_command()
  if not transmit:
    return command
  return _exchange_position_acknowledgement(command)

@_synchronous_motion_request("Force calibration home position")
@_tracked_serial_operation("ser")
def CalZeroPos():
  # global RUN['VR_angles']
  #Curtime = datetime.now().strftime("%B %d %Y - %I:%M%p")
  if _force_controller_position((0, 0, 0, 0, 45, 0)) is not True:
    return False
  almStatusLab.config(text="Calibration Forced to Home", style="Warn.TLabel")
  almStatusLab2.config(text="Calibration Forced to Home", style="Warn.TLabel")
  message = "Calibration Forced to Home - this is for commissioning and testing - be careful!"
  #tab8.ElogView.insert(END, Curtime+" - "+message)
  logger.warning(message)
  value=tab8.ElogView.get(0,END)
  pickle.dump(value,open("ErrorLog","wb"))
  RUN['VR_angles'] = [float(CAL['J1AngCur']), float(CAL['J2AngCur']), float(CAL['J3AngCur']), float(CAL['J4AngCur']), float(CAL['J5AngCur']), float(CAL['J6AngCur'])]
  setStepMonitorsVR()
  return True

@_synchronous_motion_request("Force calibration rest position")
@_tracked_serial_operation("ser")
def CalRestPos():
  # global RUN['VR_angles']
  #Curtime = datetime.now().strftime("%B %d %Y - %I:%M%p")
  if _force_controller_position((0, 0, -89, 0, 0, 0)) is not True:
    return False
  almStatusLab.config(text="Calibration Forced to Vertical Rest Pos", style="Warn.TLabel")
  almStatusLab2.config(text="Calibration Forced to Vertical Rest Pos", style="Warn.TLabel")
  message = "Calibration Forced to Vertical - this is for commissioning and testing - be careful!"
  #tab8.ElogView.insert(END, Curtime+" - "+message)
  logger.warning(message)
  value=tab8.ElogView.get(0,END)
  pickle.dump(value,open("ErrorLog","wb")) 
  RUN['VR_angles'] = [float(CAL['J1AngCur']), float(CAL['J2AngCur']), float(CAL['J3AngCur']), float(CAL['J4AngCur']), float(CAL['J5AngCur']), float(CAL['J6AngCur'])]
  setStepMonitorsVR()
  return True




CALIBRATION_SAVE_DEBOUNCE_MS = 250
_calibration_save_job = None
_calibration_dirty = False


def _write_pending_calibration():
  global _calibration_save_job, _calibration_dirty
  _calibration_save_job = None
  if not _calibration_dirty:
    return True

  failure_logged = False
  try:
    persisted = save_calibration(CAL)
  except Exception:
    persisted = False
    failure_logged = True
    logger.exception("Unable to persist the latest calibration state")
  if persisted is True:
    _calibration_dirty = False
    return True

  if persisted is not False:
    logger.error("Calibration persistence returned an invalid result")
  elif not failure_logged:
    logger.error("Unable to persist the latest calibration state")
  if not application_closing.is_set():
    try:
      _calibration_save_job = root.after(
        CALIBRATION_SAVE_DEBOUNCE_MS,
        _write_pending_calibration,
      )
    except (RuntimeError, tk.TclError):
      logger.exception("Unable to schedule a calibration persistence retry")
  return False


def _schedule_calibration_save():
  global _calibration_save_job, _calibration_dirty
  _calibration_dirty = True
  if _calibration_save_job is not None:
    root.after_cancel(_calibration_save_job)
    _calibration_save_job = None
  scheduled_job = root.after(
    CALIBRATION_SAVE_DEBOUNCE_MS,
    _write_pending_calibration,
  )
  if scheduled_job is None:
    raise RuntimeError("calibration persistence scheduler returned no job")
  _calibration_save_job = scheduled_job
  return scheduled_job


def _flush_calibration_save():
  global _calibration_save_job
  if _calibration_save_job is not None:
    root.after_cancel(_calibration_save_job)
    _calibration_save_job = None
  return _write_pending_calibration()


def _retain_calibration_persistence_retry():
  global _calibration_dirty

  try:
    _schedule_calibration_save()
  except Exception:
    _calibration_dirty = True
    logger.exception("Unable to schedule calibration persistence retry")
    return False
  return True


def displayPosition(response, parsed=None, synchronize_dispatcher=True):
  global confirmed_position_generation
  try:
    if parsed is None:
      parsed = parse_position_response(response)
    elif not isinstance(parsed, PositionResponse):
      raise ProtocolResponseError("parsed response has an invalid type")
    elif not isinstance(response, str) or parsed.raw != response:
      raise ProtocolResponseError("parsed response does not match the raw response")
    _current_controller_joint_calibration().validate_positions(
      parsed.joints + parsed.external
    )
  except (MotionInputError, ProtocolResponseError) as exc:
    message = f"Invalid controller position response: {exc}"
    _invalidate_joint_motion_state(message)
    logger.error(message)
    almStatusLab.config(text=message, style="Alarm.TLabel")
    almStatusLab2.config(text=message, style="Alarm.TLabel")
    return None

  if parsed.flag:
    _invalidate_joint_motion_state(
      f"controller reported motion fault: {parsed.flag}"
    )
  elif synchronize_dispatcher:
    if joint_motion_dispatcher.synchronize(
      parsed.joints + parsed.external
    ) is not True:
      message = "Controller position response rejected while joint motion is active"
      _invalidate_joint_motion_state(message)
      logger.error(message)
      almStatusLab.config(text=message, style="Alarm.TLabel")
      almStatusLab2.config(text=message, style="Alarm.TLabel")
      return None

  resynchronizing_virtual_pose = (
    not parsed.flag
    and controller_position_resynchronization_required.is_set()
  )
  if resynchronizing_virtual_pose and (
    _try_set_virtual_joint_target(parsed.joints) is not True
  ):
    message = "Controller position could not resynchronize the virtual model"
    _invalidate_joint_motion_state(message)
    logger.error(message)
    almStatusLab.config(text=message, style="Alarm.TLabel")
    almStatusLab2.config(text=message, style="Alarm.TLabel")
    return None

  cmdRecEntryField.delete(0, 'end')
  cmdRecEntryField.insert(0, parsed.raw)

  position_values = (
    parsed.joint_text + parsed.cartesian_text + parsed.external_text
  )
  for key, value in zip(CALIBRATION_POSITION_KEYS, position_values):
    CAL[key] = value

  RUN['WC'] = "F" if parsed.joints[4] > 0 else "N"

  entry_fields, jog_sliders = _calibration_pose_widget_groups()
  for entry_field, value in zip(entry_fields, position_values):
    _write_joint_position_entry(entry_field, value)

  for jog_slider, value in zip(
    jog_sliders,
    parsed.joint_text + parsed.external_text,
  ):
    jog_slider.set(value)

  manEntryField.delete(0, 'end')
  manEntryField.insert(0, parsed.debug)

  _schedule_calibration_save()
  if not parsed.flag:
    confirmed_position_generation += 1
    _clear_acknowledged_forced_position_target()
    if resynchronizing_virtual_pose:
      controller_position_resynchronization_required.clear()

  if parsed.flag:
    ErrorHandler(parsed.flag)

  if parsed.speed_violation:
    message = "Max Speed Violation - Reduce Speed Setpoint or Travel Distance"
    logger.warning(message)
    value = tab8.ElogView.get(0, END)
    pickle.dump(value, open("ErrorLog", "wb"))
    almStatusLab.config(text=message, style="Warn.TLabel")
    almStatusLab2.config(text=message, style="Warn.TLabel")

  return parsed




def ClearKinTabFields():
  J1MotDirEntryField.delete(0, 'end')
  J2MotDirEntryField.delete(0, 'end')
  J3MotDirEntryField.delete(0, 'end')
  J4MotDirEntryField.delete(0, 'end')
  J5MotDirEntryField.delete(0, 'end')
  J6MotDirEntryField.delete(0, 'end')
  J7MotDirEntryField.delete(0, 'end')
  J8MotDirEntryField.delete(0, 'end')
  J9MotDirEntryField.delete(0, 'end')
  J1CalDirEntryField.delete(0, 'end')
  J2CalDirEntryField.delete(0, 'end')
  J3CalDirEntryField.delete(0, 'end')
  J4CalDirEntryField.delete(0, 'end')
  J5CalDirEntryField.delete(0, 'end')
  J6CalDirEntryField.delete(0, 'end')
  J7CalDirEntryField.delete(0, 'end')
  J8CalDirEntryField.delete(0, 'end')
  J9CalDirEntryField.delete(0, 'end')
  J1PosLimEntryField.delete(0, 'end')
  J1NegLimEntryField.delete(0, 'end')
  J2PosLimEntryField.delete(0, 'end')
  J2NegLimEntryField.delete(0, 'end')
  J3PosLimEntryField.delete(0, 'end')
  J3NegLimEntryField.delete(0, 'end')
  J4PosLimEntryField.delete(0, 'end')
  J4NegLimEntryField.delete(0, 'end')
  J5PosLimEntryField.delete(0, 'end')
  J5NegLimEntryField.delete(0, 'end')
  J6PosLimEntryField.delete(0, 'end')
  J6NegLimEntryField.delete(0, 'end')  
  J1StepDegEntryField.delete(0, 'end')
  J2StepDegEntryField.delete(0, 'end') 
  J3StepDegEntryField.delete(0, 'end') 
  J4StepDegEntryField.delete(0, 'end') 
  J5StepDegEntryField.delete(0, 'end') 
  J6StepDegEntryField.delete(0, 'end')
  J1DriveMSEntryField.delete(0, 'end')
  J2DriveMSEntryField.delete(0, 'end')  
  J3DriveMSEntryField.delete(0, 'end')  
  J4DriveMSEntryField.delete(0, 'end')  
  J5DriveMSEntryField.delete(0, 'end')  
  J6DriveMSEntryField.delete(0, 'end')
  J1EncCPREntryField.delete(0, 'end')
  J2EncCPREntryField.delete(0, 'end')
  J3EncCPREntryField.delete(0, 'end')
  J4EncCPREntryField.delete(0, 'end')
  J5EncCPREntryField.delete(0, 'end')
  J6EncCPREntryField.delete(0, 'end')
  J1ΘEntryField.delete(0, 'end')
  J2ΘEntryField.delete(0, 'end')
  J3ΘEntryField.delete(0, 'end')
  J4ΘEntryField.delete(0, 'end')
  J5ΘEntryField.delete(0, 'end')
  J6ΘEntryField.delete(0, 'end')
  J1αEntryField.delete(0, 'end')
  J2αEntryField.delete(0, 'end')
  J3αEntryField.delete(0, 'end')
  J4αEntryField.delete(0, 'end')
  J5αEntryField.delete(0, 'end')
  J6αEntryField.delete(0, 'end')
  J1dEntryField.delete(0, 'end')
  J2dEntryField.delete(0, 'end')
  J3dEntryField.delete(0, 'end')
  J4dEntryField.delete(0, 'end')
  J5dEntryField.delete(0, 'end')
  J6dEntryField.delete(0, 'end')
  J1aEntryField.delete(0, 'end')
  J2aEntryField.delete(0, 'end')
  J3aEntryField.delete(0, 'end')
  J4aEntryField.delete(0, 'end')
  J5aEntryField.delete(0, 'end')
  J6aEntryField.delete(0, 'end')


def LoadAR4Mk3default():
  ClearKinTabFields()
  J1MotDirEntryField.insert(0,str(0))
  J2MotDirEntryField.insert(0,str(1))
  J3MotDirEntryField.insert(0,str(1))
  J4MotDirEntryField.insert(0,str(1))
  J5MotDirEntryField.insert(0,str(1))
  J6MotDirEntryField.insert(0,str(1))
  J7MotDirEntryField.insert(0,str(1))
  J8MotDirEntryField.insert(0,str(1))
  J9MotDirEntryField.insert(0,str(1))
  J1CalDirEntryField.insert(0,str(1))
  J2CalDirEntryField.insert(0,str(0))
  J3CalDirEntryField.insert(0,str(1))
  J4CalDirEntryField.insert(0,str(0))
  J5CalDirEntryField.insert(0,str(0))
  J6CalDirEntryField.insert(0,str(1))
  J7CalDirEntryField.insert(0,str(0))
  J8CalDirEntryField.insert(0,str(0))
  J9CalDirEntryField.insert(0,str(0))
  J1PosLimEntryField.insert(0,str(170))
  J1NegLimEntryField.insert(0,str(170))
  J2PosLimEntryField.insert(0,str(90))
  J2NegLimEntryField.insert(0,str(42))
  J3PosLimEntryField.insert(0,str(52))
  J3NegLimEntryField.insert(0,str(89))
  J4PosLimEntryField.insert(0,str(180))
  J4NegLimEntryField.insert(0,str(180))
  J5PosLimEntryField.insert(0,str(105))
  J5NegLimEntryField.insert(0,str(105))
  J6PosLimEntryField.insert(0,str(180))
  J6NegLimEntryField.insert(0,str(180))  
  J1StepDegEntryField.insert(0,str(88.888))
  J2StepDegEntryField.insert(0,str(111.111)) 
  J3StepDegEntryField.insert(0,str(111.111)) 
  J4StepDegEntryField.insert(0,str(99.555)) 
  J5StepDegEntryField.insert(0,str(43.720)) 
  J6StepDegEntryField.insert(0,str(44.444))
  J1DriveMSEntryField.insert(0,str(800))
  J2DriveMSEntryField.insert(0,str(800))  
  J3DriveMSEntryField.insert(0,str(800))  
  J4DriveMSEntryField.insert(0,str(800))  
  J5DriveMSEntryField.insert(0,str(1600))  
  J6DriveMSEntryField.insert(0,str(800))
  J1EncCPREntryField.insert(0,str(4000))
  J2EncCPREntryField.insert(0,str(4000))
  J3EncCPREntryField.insert(0,str(4000))
  J4EncCPREntryField.insert(0,str(4000))
  J5EncCPREntryField.insert(0,str(4000))
  J6EncCPREntryField.insert(0,str(4000))
  J1ΘEntryField.insert(0,str(0))
  J2ΘEntryField.insert(0,str(-90))
  J3ΘEntryField.insert(0,str(0))
  J4ΘEntryField.insert(0,str(0))
  J5ΘEntryField.insert(0,str(0))
  J6ΘEntryField.insert(0,str(180))
  J1αEntryField.insert(0,str(0))
  J2αEntryField.insert(0,str(-90))
  J3αEntryField.insert(0,str(0))
  J4αEntryField.insert(0,str(-90))
  J5αEntryField.insert(0,str(90))
  J6αEntryField.insert(0,str(-90))
  J1dEntryField.insert(0,str(169.77))
  J2dEntryField.insert(0,str(0))
  J3dEntryField.insert(0,str(0))
  J4dEntryField.insert(0,str(222.63))
  J5dEntryField.insert(0,str(0))
  J6dEntryField.insert(0,str(41))
  J1aEntryField.insert(0,str(0))
  J2aEntryField.insert(0,str(64.2))
  J3aEntryField.insert(0,str(305))
  J4aEntryField.insert(0,str(0))
  J5aEntryField.insert(0,str(0))
  J6aEntryField.insert(0,str(0)) 

def LoadAR4Mk2default():
  ClearKinTabFields()
  J1MotDirEntryField.insert(0,str(0))
  J2MotDirEntryField.insert(0,str(1))
  J3MotDirEntryField.insert(0,str(1))
  J4MotDirEntryField.insert(0,str(1))
  J5MotDirEntryField.insert(0,str(1))
  J6MotDirEntryField.insert(0,str(1))
  J7MotDirEntryField.insert(0,str(1))
  J8MotDirEntryField.insert(0,str(1))
  J9MotDirEntryField.insert(0,str(1))
  J1CalDirEntryField.insert(0,str(1))
  J2CalDirEntryField.insert(0,str(0))
  J3CalDirEntryField.insert(0,str(1))
  J4CalDirEntryField.insert(0,str(0))
  J5CalDirEntryField.insert(0,str(0))
  J6CalDirEntryField.insert(0,str(1))
  J7CalDirEntryField.insert(0,str(0))
  J8CalDirEntryField.insert(0,str(0))
  J9CalDirEntryField.insert(0,str(0))
  J1PosLimEntryField.insert(0,str(170))
  J1NegLimEntryField.insert(0,str(170))
  J2PosLimEntryField.insert(0,str(90))
  J2NegLimEntryField.insert(0,str(42))
  J3PosLimEntryField.insert(0,str(52))
  J3NegLimEntryField.insert(0,str(89))
  J4PosLimEntryField.insert(0,str(165))
  J4NegLimEntryField.insert(0,str(165))
  J5PosLimEntryField.insert(0,str(105))
  J5NegLimEntryField.insert(0,str(105))
  J6PosLimEntryField.insert(0,str(155))
  J6NegLimEntryField.insert(0,str(155))  
  J1StepDegEntryField.insert(0,str(44.4444))
  J2StepDegEntryField.insert(0,str(55.5555)) 
  J3StepDegEntryField.insert(0,str(55.5555)) 
  J4StepDegEntryField.insert(0,str(42.7266)) 
  J5StepDegEntryField.insert(0,str(21.8602)) 
  J6StepDegEntryField.insert(0,str(22.2222))
  J1DriveMSEntryField.insert(0,str(400))
  J2DriveMSEntryField.insert(0,str(400))  
  J3DriveMSEntryField.insert(0,str(400))  
  J4DriveMSEntryField.insert(0,str(400))  
  J5DriveMSEntryField.insert(0,str(800))  
  J6DriveMSEntryField.insert(0,str(400))
  J1EncCPREntryField.insert(0,str(4000))
  J2EncCPREntryField.insert(0,str(4000))
  J3EncCPREntryField.insert(0,str(4000))
  J4EncCPREntryField.insert(0,str(4000))
  J5EncCPREntryField.insert(0,str(4000))
  J6EncCPREntryField.insert(0,str(4000))
  J1ΘEntryField.insert(0,str(0))
  J2ΘEntryField.insert(0,str(-90))
  J3ΘEntryField.insert(0,str(0))
  J4ΘEntryField.insert(0,str(0))
  J5ΘEntryField.insert(0,str(0))
  J6ΘEntryField.insert(0,str(180))
  J1αEntryField.insert(0,str(0))
  J2αEntryField.insert(0,str(-90))
  J3αEntryField.insert(0,str(0))
  J4αEntryField.insert(0,str(-90))
  J5αEntryField.insert(0,str(90))
  J6αEntryField.insert(0,str(-90))
  J1dEntryField.insert(0,str(169.77))
  J2dEntryField.insert(0,str(0))
  J3dEntryField.insert(0,str(0))
  J4dEntryField.insert(0,str(222.63))
  J5dEntryField.insert(0,str(0))
  J6dEntryField.insert(0,str(36.25))
  J1aEntryField.insert(0,str(0))
  J2aEntryField.insert(0,str(64.2))
  J3aEntryField.insert(0,str(305))
  J4aEntryField.insert(0,str(0))
  J5aEntryField.insert(0,str(0))
  J6aEntryField.insert(0,str(0)) 


def LoadAR4default():
  ClearKinTabFields()
  J1MotDirEntryField.insert(0,str(1))
  J2MotDirEntryField.insert(0,str(0))
  J3MotDirEntryField.insert(0,str(0))
  J4MotDirEntryField.insert(0,str(1))
  J5MotDirEntryField.insert(0,str(0))
  J6MotDirEntryField.insert(0,str(0))
  J7MotDirEntryField.insert(0,str(1))
  J8MotDirEntryField.insert(0,str(1))
  J9MotDirEntryField.insert(0,str(1))
  J1CalDirEntryField.insert(0,str(1))
  J2CalDirEntryField.insert(0,str(0))
  J3CalDirEntryField.insert(0,str(1))
  J4CalDirEntryField.insert(0,str(0))
  J5CalDirEntryField.insert(0,str(0))
  J6CalDirEntryField.insert(0,str(1))
  J7CalDirEntryField.insert(0,str(0))
  J8CalDirEntryField.insert(0,str(0))
  J9CalDirEntryField.insert(0,str(0))
  J1PosLimEntryField.insert(0,str(170))
  J1NegLimEntryField.insert(0,str(170))
  J2PosLimEntryField.insert(0,str(90))
  J2NegLimEntryField.insert(0,str(42))
  J3PosLimEntryField.insert(0,str(52))
  J3NegLimEntryField.insert(0,str(89))
  J4PosLimEntryField.insert(0,str(165))
  J4NegLimEntryField.insert(0,str(165))
  J5PosLimEntryField.insert(0,str(105))
  J5NegLimEntryField.insert(0,str(105))
  J6PosLimEntryField.insert(0,str(155))
  J6NegLimEntryField.insert(0,str(155))  
  J1StepDegEntryField.insert(0,str(44.4444))
  J2StepDegEntryField.insert(0,str(55.5555)) 
  J3StepDegEntryField.insert(0,str(55.5555)) 
  J4StepDegEntryField.insert(0,str(42.7266)) 
  J5StepDegEntryField.insert(0,str(21.8602)) 
  J6StepDegEntryField.insert(0,str(22.2222))
  J1DriveMSEntryField.insert(0,str(400))
  J2DriveMSEntryField.insert(0,str(400))  
  J3DriveMSEntryField.insert(0,str(400))  
  J4DriveMSEntryField.insert(0,str(400))  
  J5DriveMSEntryField.insert(0,str(800))  
  J6DriveMSEntryField.insert(0,str(400))
  J1EncCPREntryField.insert(0,str(4000))
  J2EncCPREntryField.insert(0,str(4000))
  J3EncCPREntryField.insert(0,str(4000))
  J4EncCPREntryField.insert(0,str(4000))
  J5EncCPREntryField.insert(0,str(4000))
  J6EncCPREntryField.insert(0,str(4000))
  J1ΘEntryField.insert(0,str(0))
  J2ΘEntryField.insert(0,str(-90))
  J3ΘEntryField.insert(0,str(0))
  J4ΘEntryField.insert(0,str(0))
  J5ΘEntryField.insert(0,str(0))
  J6ΘEntryField.insert(0,str(180))
  J1αEntryField.insert(0,str(0))
  J2αEntryField.insert(0,str(-90))
  J3αEntryField.insert(0,str(0))
  J4αEntryField.insert(0,str(-90))
  J5αEntryField.insert(0,str(90))
  J6αEntryField.insert(0,str(-90))
  J1dEntryField.insert(0,str(169.77))
  J2dEntryField.insert(0,str(0))
  J3dEntryField.insert(0,str(0))
  J4dEntryField.insert(0,str(222.63))
  J5dEntryField.insert(0,str(0))
  J6dEntryField.insert(0,str(36.25))
  J1aEntryField.insert(0,str(0))
  J2aEntryField.insert(0,str(64.2))
  J3aEntryField.insert(0,str(305))
  J4aEntryField.insert(0,str(0))
  J5aEntryField.insert(0,str(0))
  J6aEntryField.insert(0,str(0)) 

def LoadAR3default():
  ClearKinTabFields()
  J1MotDirEntryField.insert(0,str(1))
  J2MotDirEntryField.insert(0,str(0))
  J3MotDirEntryField.insert(0,str(0))
  J4MotDirEntryField.insert(0,str(1))
  J5MotDirEntryField.insert(0,str(0))
  J6MotDirEntryField.insert(0,str(0))
  J7MotDirEntryField.insert(0,str(1))
  J8MotDirEntryField.insert(0,str(1))
  J9MotDirEntryField.insert(0,str(1))
  J1CalDirEntryField.insert(0,str(1))
  J2CalDirEntryField.insert(0,str(0))
  J3CalDirEntryField.insert(0,str(1))
  J4CalDirEntryField.insert(0,str(0))
  J5CalDirEntryField.insert(0,str(0))
  J6CalDirEntryField.insert(0,str(1))
  J7CalDirEntryField.insert(0,str(0))
  J8CalDirEntryField.insert(0,str(0))
  J9CalDirEntryField.insert(0,str(0))
  J1PosLimEntryField.insert(0,str(170))
  J1NegLimEntryField.insert(0,str(170))
  J2PosLimEntryField.insert(0,str(90))
  J2NegLimEntryField.insert(0,str(42))
  J3PosLimEntryField.insert(0,str(52))
  J3NegLimEntryField.insert(0,str(89))
  J4PosLimEntryField.insert(0,str(165))
  J4NegLimEntryField.insert(0,str(165))
  J5PosLimEntryField.insert(0,str(105))
  J5NegLimEntryField.insert(0,str(105))
  J6PosLimEntryField.insert(0,str(155))
  J6NegLimEntryField.insert(0,str(155))  
  J1StepDegEntryField.insert(0,str(44.4444))
  J2StepDegEntryField.insert(0,str(55.5555)) 
  J3StepDegEntryField.insert(0,str(55.5555)) 
  J4StepDegEntryField.insert(0,str(42.7266)) 
  J5StepDegEntryField.insert(0,str(21.8602)) 
  J6StepDegEntryField.insert(0,str(22.2222))
  J1DriveMSEntryField.insert(0,str(400))
  J2DriveMSEntryField.insert(0,str(400))  
  J3DriveMSEntryField.insert(0,str(400))  
  J4DriveMSEntryField.insert(0,str(400))  
  J5DriveMSEntryField.insert(0,str(800))  
  J6DriveMSEntryField.insert(0,str(400))
  J1EncCPREntryField.insert(0,str(2048))
  J2EncCPREntryField.insert(0,str(2048))
  J3EncCPREntryField.insert(0,str(2048))
  J4EncCPREntryField.insert(0,str(2048))
  J5EncCPREntryField.insert(0,str(2048))
  J6EncCPREntryField.insert(0,str(2048))
  J1ΘEntryField.insert(0,str(0))
  J2ΘEntryField.insert(0,str(-90))
  J3ΘEntryField.insert(0,str(0))
  J4ΘEntryField.insert(0,str(0))
  J5ΘEntryField.insert(0,str(0))
  J6ΘEntryField.insert(0,str(180))
  J1αEntryField.insert(0,str(0))
  J2αEntryField.insert(0,str(-90))
  J3αEntryField.insert(0,str(0))
  J4αEntryField.insert(0,str(-90))
  J5αEntryField.insert(0,str(90))
  J6αEntryField.insert(0,str(-90))
  J1dEntryField.insert(0,str(169.77))
  J2dEntryField.insert(0,str(0))
  J3dEntryField.insert(0,str(0))
  J4dEntryField.insert(0,str(222.63))
  J5dEntryField.insert(0,str(0))
  J6dEntryField.insert(0,str(36.25))
  J1aEntryField.insert(0,str(0))
  J2aEntryField.insert(0,str(64.2))
  J3aEntryField.insert(0,str(305))
  J4aEntryField.insert(0,str(0))
  J5aEntryField.insert(0,str(0))
  J6aEntryField.insert(0,str(0)) 

def LoadMaxdefault():
  ClearKinTabFields()
  J1MotDirEntryField.insert(0,str(0))
  J2MotDirEntryField.insert(0,str(1))
  J3MotDirEntryField.insert(0,str(1))
  J4MotDirEntryField.insert(0,str(1))
  J5MotDirEntryField.insert(0,str(1))
  J6MotDirEntryField.insert(0,str(1))
  J7MotDirEntryField.insert(0,str(1))
  J8MotDirEntryField.insert(0,str(1))
  J9MotDirEntryField.insert(0,str(1))
  J1CalDirEntryField.insert(0,str(1))
  J2CalDirEntryField.insert(0,str(0))
  J3CalDirEntryField.insert(0,str(1))
  J4CalDirEntryField.insert(0,str(0))
  J5CalDirEntryField.insert(0,str(0))
  J6CalDirEntryField.insert(0,str(1))
  J7CalDirEntryField.insert(0,str(0))
  J8CalDirEntryField.insert(0,str(0))
  J9CalDirEntryField.insert(0,str(0))
  J1PosLimEntryField.insert(0,str(170))
  J1NegLimEntryField.insert(0,str(170))
  J2PosLimEntryField.insert(0,str(90))
  J2NegLimEntryField.insert(0,str(42))
  J3PosLimEntryField.insert(0,str(52))
  J3NegLimEntryField.insert(0,str(89))
  J4PosLimEntryField.insert(0,str(180))
  J4NegLimEntryField.insert(0,str(180))
  J5PosLimEntryField.insert(0,str(105))
  J5NegLimEntryField.insert(0,str(105))
  J6PosLimEntryField.insert(0,str(180))
  J6NegLimEntryField.insert(0,str(180))  
  J1StepDegEntryField.insert(0,str(1422.222))
  J2StepDegEntryField.insert(0,str(1777.777)) 
  J3StepDegEntryField.insert(0,str(1777.777)) 
  J4StepDegEntryField.insert(0,str(1592.888)) 
  J5StepDegEntryField.insert(0,str(349.763)) 
  J6StepDegEntryField.insert(0,str(711.111))
  J1DriveMSEntryField.insert(0,str(12800))
  J2DriveMSEntryField.insert(0,str(12800))  
  J3DriveMSEntryField.insert(0,str(12800))  
  J4DriveMSEntryField.insert(0,str(12800))  
  J5DriveMSEntryField.insert(0,str(12800))  
  J6DriveMSEntryField.insert(0,str(12800))
  J1EncCPREntryField.insert(0,str(4000))
  J2EncCPREntryField.insert(0,str(4000))
  J3EncCPREntryField.insert(0,str(4000))
  J4EncCPREntryField.insert(0,str(4000))
  J5EncCPREntryField.insert(0,str(4000))
  J6EncCPREntryField.insert(0,str(4000))
  J1ΘEntryField.insert(0,str(0))
  J2ΘEntryField.insert(0,str(-90))
  J3ΘEntryField.insert(0,str(0))
  J4ΘEntryField.insert(0,str(0))
  J5ΘEntryField.insert(0,str(0))
  J6ΘEntryField.insert(0,str(180))
  J1αEntryField.insert(0,str(0))
  J2αEntryField.insert(0,str(-90))
  J3αEntryField.insert(0,str(0))
  J4αEntryField.insert(0,str(-90))
  J5αEntryField.insert(0,str(90))
  J6αEntryField.insert(0,str(-90))
  J1dEntryField.insert(0,str(169.77))
  J2dEntryField.insert(0,str(0))
  J3dEntryField.insert(0,str(0))
  J4dEntryField.insert(0,str(222.63))
  J5dEntryField.insert(0,str(0))
  J6dEntryField.insert(0,str(41))
  J1aEntryField.insert(0,str(0))
  J2aEntryField.insert(0,str(64.2))
  J3aEntryField.insert(0,str(305))
  J4aEntryField.insert(0,str(0))
  J5aEntryField.insert(0,str(0))
  J6aEntryField.insert(0,str(0))   
  
def _custom_calibration_profile_keys():
  keys = ['TFx', 'TFy', 'TFz', 'TFrz', 'TFry', 'TFrx']
  for suffix, axis_count in (
    ('MotDir', 9),
    ('CalDir', 9),
    ('PosLim', 6),
    ('NegLim', 6),
    ('StepDeg', 6),
    ('DriveMS', 6),
    ('EncCPR', 6),
    ('ΘDHpar', 6),
    ('αDHpar', 6),
    ('dDHpar', 6),
    ('aDHpar', 6),
  ):
    keys.extend(f'J{axis}{suffix}' for axis in range(1, axis_count + 1))
  keys.extend((
    'J7PosLim', 'J7rotation', 'J7steps',
    'J8length', 'J8rotation', 'J8steps',
    'J9length', 'J9rotation', 'J9steps',
  ))
  keys.extend(f'J{axis}calOff' for axis in range(1, 10))
  return tuple(keys)


def _custom_calibration_field_values(calibration_values):
  if not isinstance(calibration_values, dict):
    raise MotionInputError("custom calibration profile must be a dictionary")
  try:
    return {
      key: calibration_values[key]
      for key in _custom_calibration_profile_keys()
    }
  except KeyError as exc:
    raise MotionInputError(
      f"custom calibration profile is missing {exc.args[0]}"
    ) from exc


def _prepare_custom_calibration_profile(loaded_calibration):
  profile_values = _custom_calibration_field_values(loaded_calibration)
  update_values, _ = _prepare_update_parameters_from_values(profile_values)
  staged_values = dict(CAL)
  staged_values.update(update_values)
  external_values, _ = _prepare_external_axis_parameters_from_values(
    profile_values,
    staged_values,
  )
  staged_values.update(external_values)
  for axis in range(1, 10):
    key = f'J{axis}calOff'
    profile_values[key] = finite_number(
      profile_values[key],
      f"J{axis} calibration offset",
    )
  _validate_controller_pose(staged_values)
  profile_values.update(update_values)
  profile_values.update(external_values)
  return profile_values


def _prepare_custom_calibration_snapshot():
  field_values = _collect_fields_to_calibration()
  (
    update_values,
    _,
    external_values,
    _,
  ) = _prepare_controller_calibration()
  staged_values = dict(CAL)
  staged_values.update(field_values)
  staged_values.update(update_values)
  staged_values.update(external_values)
  _validate_controller_pose(staged_values)
  return staged_values


def save_custom_calibration():
  try:
    profile_values = _prepare_custom_calibration_snapshot()
    persisted = save_calibration(
      calibration_file='custom.json',
      calibration_data=profile_values,
    )
  except Exception:
    logger.exception("Custom calibration validation or persistence failed")
    return False
  if persisted is not True:
    logger.error("Custom calibration persistence returned a non-true result")
    return False
  return True


def load_custom_calibration():
  try:
    loaded_calibration = load_calibration(
      calibration_file='custom.json',
      allow_fallback=False,
    )
    profile_values = _prepare_custom_calibration_profile(loaded_calibration)
    sync_calibration_to_fields(profile_values)
  except Exception:
    logger.exception("Custom calibration loading or validation failed")
    return False
  logger.debug(
    "Loaded custom J1 drive microsteps into editable fields: %s",
    profile_values['J1DriveMS'],
  )
  return True


def _custom_calibration_field_bindings(calibration_values):
  values = _custom_calibration_field_values(calibration_values)
  fields = [
    TFxEntryField, TFyEntryField, TFzEntryField,
    TFrzEntryField, TFryEntryField, TFrxEntryField,
  ]
  fields.extend((
    J1MotDirEntryField, J2MotDirEntryField, J3MotDirEntryField,
    J4MotDirEntryField, J5MotDirEntryField, J6MotDirEntryField,
    J7MotDirEntryField, J8MotDirEntryField, J9MotDirEntryField,
  ))
  fields.extend((
    J1CalDirEntryField, J2CalDirEntryField, J3CalDirEntryField,
    J4CalDirEntryField, J5CalDirEntryField, J6CalDirEntryField,
    J7CalDirEntryField, J8CalDirEntryField, J9CalDirEntryField,
  ))
  for field_group in (
    (J1PosLimEntryField, J2PosLimEntryField, J3PosLimEntryField,
     J4PosLimEntryField, J5PosLimEntryField, J6PosLimEntryField),
    (J1NegLimEntryField, J2NegLimEntryField, J3NegLimEntryField,
     J4NegLimEntryField, J5NegLimEntryField, J6NegLimEntryField),
    (J1StepDegEntryField, J2StepDegEntryField, J3StepDegEntryField,
     J4StepDegEntryField, J5StepDegEntryField, J6StepDegEntryField),
    (J1DriveMSEntryField, J2DriveMSEntryField, J3DriveMSEntryField,
     J4DriveMSEntryField, J5DriveMSEntryField, J6DriveMSEntryField),
    (J1EncCPREntryField, J2EncCPREntryField, J3EncCPREntryField,
     J4EncCPREntryField, J5EncCPREntryField, J6EncCPREntryField),
    (J1ΘEntryField, J2ΘEntryField, J3ΘEntryField,
     J4ΘEntryField, J5ΘEntryField, J6ΘEntryField),
    (J1αEntryField, J2αEntryField, J3αEntryField,
     J4αEntryField, J5αEntryField, J6αEntryField),
    (J1dEntryField, J2dEntryField, J3dEntryField,
     J4dEntryField, J5dEntryField, J6dEntryField),
    (J1aEntryField, J2aEntryField, J3aEntryField,
     J4aEntryField, J5aEntryField, J6aEntryField),
  ):
    fields.extend(field_group)
  fields.extend((
    axis7lengthEntryField, axis7rotEntryField, axis7stepsEntryField,
    axis8lengthEntryField, axis8rotEntryField, axis8stepsEntryField,
    axis9lengthEntryField, axis9rotEntryField, axis9stepsEntryField,
  ))
  fields.extend((
    J1calOffEntryField, J2calOffEntryField, J3calOffEntryField,
    J4calOffEntryField, J5calOffEntryField, J6calOffEntryField,
    J7calOffEntryField, J8calOffEntryField, J9calOffEntryField,
  ))
  keys = _custom_calibration_profile_keys()
  if len(fields) != len(keys):
    raise RuntimeError("custom calibration field mapping is inconsistent")
  return tuple(
    (field, values[key])
    for field, key in zip(fields, keys)
  )


def sync_calibration_to_fields(calibration_values=None):
  source_values = CAL if calibration_values is None else calibration_values
  bindings = _custom_calibration_field_bindings(source_values)
  for field, value in bindings:
    field.delete(0, 'end')
    field.insert(0, str(value))
  return True

def _collect_fields_to_calibration():
  values = {
    'comPort': com1SelectedValue.get(),
    'com2Port': com2SelectedValue.get(),
    'auxiliaryBoard': (
      normalize_auxiliary_board_profile(
        auxiliaryBoardSelectedValue.get(),
        allow_none=True,
      ) or AUXILIARY_BOARD_NONE
    ),
    'J7PosCur': finite_number(
      J7curAngEntryField.get(),
      "J7 current position",
    ),
    'J8PosCur': finite_number(
      J8curAngEntryField.get(),
      "J8 current position",
    ),
    'J9PosCur': finite_number(
      J9curAngEntryField.get(),
      "J9 current position",
    ),
    'VisProg': visoptions.get(),
    'VisBrightVal': finite_number(VisBrightSlide.get(), "vision brightness"),
    'VisContVal': finite_number(VisContrastSlide.get(), "vision contrast"),
    'VisBacColor': str(VisBacColorEntryField.get()),
    'VisScore': finite_number(VisScoreEntryField.get(), "vision score"),
    'VisX1Val': int(VisX1PixEntryField.get()),
    'VisY1Val': int(VisY1PixEntryField.get()),
    'VisX2Val': int(VisX2PixEntryField.get()),
    'VisY2Val': int(VisY2PixEntryField.get()),
    'VisRobX1Val': finite_number(VisX1RobEntryField.get(), "vision robot X1"),
    'VisRobY1Val': finite_number(VisY1RobEntryField.get(), "vision robot Y1"),
    'VisRobX2Val': finite_number(VisX2RobEntryField.get(), "vision robot X2"),
    'VisRobY2Val': finite_number(VisY2RobEntryField.get(), "vision robot Y2"),
    'zoom': finite_number(VisZoomSlide.get(), "vision zoom"),
    'pick180Val': int(RUN['pick180'].get()),
    'pickClosestVal': int(RUN['pickClosest'].get()),
    'curCam': str(visoptions.get()),
    'fullRotVal': int(RUN['fullRot'].get()),
    'autoBGVal': int(RUN['autoBG'].get()),
  }
  calibration_offset_fields = (
    J1calOffEntryField,
    J2calOffEntryField,
    J3calOffEntryField,
    J4calOffEntryField,
    J5calOffEntryField,
    J6calOffEntryField,
    J7calOffEntryField,
    J8calOffEntryField,
    J9calOffEntryField,
  )
  for axis, field in enumerate(calibration_offset_fields, start=1):
    values[f'J{axis}calOff'] = finite_number(
      field.get(),
      f"J{axis} calibration offset",
    )

  external_values = _collect_external_axis_values()
  values.update(external_values)
  drive_fields = (
    J1DriveMSEntryField,
    J2DriveMSEntryField,
    J3DriveMSEntryField,
    J4DriveMSEntryField,
    J5DriveMSEntryField,
    J6DriveMSEntryField,
  )
  encoder_fields = (
    J1EncCPREntryField,
    J2EncCPREntryField,
    J3EncCPREntryField,
    J4EncCPREntryField,
    J5EncCPREntryField,
    J6EncCPREntryField,
  )
  for axis, (drive_field, encoder_field) in enumerate(
    zip(drive_fields, encoder_fields),
    start=1,
  ):
    values[f'J{axis}DriveMS'] = int(drive_field.get())
    values[f'J{axis}EncCPR'] = int(encoder_field.get())
  return values


def _restore_controller_calibration(snapshot):
  CAL.clear()
  CAL.update(snapshot)
  _set_cpp_kinematics_from_values(snapshot)
  _apply_joint_limit_widgets(snapshot)
  _apply_external_axis_values({
    key: snapshot[key]
    for key in (
      'J7PosLim', 'J7rotation', 'J7steps',
      'J8length', 'J8rotation', 'J8steps',
      'J9length', 'J9rotation', 'J9steps',
    )
  })
  return True


def _invalidate_uncertain_controller_calibration(reason):
  serial_port = RUN.get('ser')
  if serial_port is not None:
    try:
      quarantine_serial_transport(serial_port, reason)
    except Exception:
      logger.exception("Unable to quarantine uncertain controller calibration")
    finally:
      if (
        RUN.get('ser') is serial_port
        and not getattr(serial_port, "is_open", False)
      ):
        RUN['ser'] = None
  try:
    _invalidate_joint_motion_state(reason)
  except Exception:
    logger.exception("Unable to invalidate joint motion after calibration failure")
  message = (
    "Controller calibration state is uncertain; "
    "controller quarantined and reconnection is required"
  )
  logger.error("%s: %s", message, reason)
  almStatusLab.config(text=message, style="Alarm.TLabel")
  almStatusLab2.config(text=message, style="Alarm.TLabel")
  return False


@_synchronous_motion_request(
  "Save and apply controller calibration",
  requires_kinematics=False,
)
@_tracked_serial_operation("ser")
def SaveAndApplyCalibration():
  snapshot = dict(CAL)
  try:
    field_values = _collect_fields_to_calibration()
    (
      update_values,
      update_command,
      external_values,
      external_command,
    ) = _prepare_controller_calibration()
    staged_values = dict(CAL)
    staged_values.update(field_values)
    staged_values.update(update_values)
    staged_values.update(external_values)
    _validate_controller_pose(staged_values)
  except Exception:
    logger.exception("Calibration validation failed")
    CAL.clear()
    CAL.update(snapshot)
    return False

  try:
    _preflight_controller_calibration_transport()
  except Exception:
    logger.exception("Calibration transport preflight failed")
    CAL.clear()
    CAL.update(snapshot)
    return False

  try:
    CAL.update(field_values)
    _apply_controller_calibration(update_values, external_values)
  except Exception:
    logger.exception("Calibration application failed")
    try:
      _restore_controller_calibration(snapshot)
    except Exception:
      logger.exception("Unable to restore calibration after application failure")
      return _invalidate_uncertain_controller_calibration(
        "local calibration rollback failed after application failure"
      )
    return False

  update_write_started = threading.Event()
  try:
    update_acknowledged = _transmit_update_parameters(
      update_command,
      update_write_started,
    )
  except Exception:
    logger.exception("Update-parameters acknowledgement failed")
    if not update_write_started.is_set():
      return _restore_prewrite_calibration(
        snapshot,
        "update-parameters pre-write failure",
      )
    return _invalidate_uncertain_controller_calibration(
      "update-parameters acknowledgement failed after transmission started"
    )
  if update_acknowledged is not True:
    logger.error("Update-parameters transmission returned a non-true result")
    if not update_write_started.is_set():
      return _restore_prewrite_calibration(
        snapshot,
        "update-parameters pre-write rejection",
      )
    return _invalidate_uncertain_controller_calibration(
      "update-parameters acknowledgement was invalid after transmission started"
    )

  external_write_started = threading.Event()
  try:
    external_acknowledged = _transmit_external_axis_parameters(
      external_command,
      external_write_started,
    )
  except Exception:
    logger.exception("External-axis acknowledgement failed")
    phase = (
      "after transmission started"
      if external_write_started.is_set()
      else "before transmission started"
    )
    return _invalidate_uncertain_controller_calibration(
      "external-axis acknowledgement failed "
      f"{phase} after primary calibration applied"
    )
  if external_acknowledged is not True:
    logger.error(
      "External-axis transmission was rejected after primary calibration applied"
    )
    return _invalidate_uncertain_controller_calibration(
      "external-axis calibration was not applied after primary calibration"
    )

  try:
    persisted = save_calibration(CAL)
  except Exception:
    logger.exception("Calibration applied but persistence failed")
    _retain_calibration_persistence_retry()
    return False
  if persisted is not True:
    logger.error("Calibration applied but persistence returned a non-true result")
    _retain_calibration_persistence_retry()
    return False
  return True


def checkSpeedVals():
  speedtype = speedOption.get()
  Speed = float(speedEntryField.get())
  if(speedtype == "mm per Sec"):
    if(Speed <= .01):
      speedEntryField.delete(0, 'end')
      speedEntryField.insert(0,"5")
  if(speedtype == "Seconds"):
    if(Speed <= .001):
      speedEntryField.delete(0, 'end')
      speedEntryField.insert(0,"1")
  if(speedtype == "Percent"):
    if(Speed <= .01 or Speed > 100):
      speedEntryField.delete(0, 'end')
      speedEntryField.insert(0,"10")
  ACCspd = float(ACCspeedField.get())
  if(ACCspd <= .01 or ACCspd > 100):
    ACCspeedField.delete(0, 'end')
    ACCspeedField.insert(0,"10")
  DECspd = float(DECspeedField.get())
  if(DECspd <= .01 or DECspd >=100):
    DECspeedField.delete(0, 'end')
    DECspeedField.insert(0,"10")
  if(ACCspd + DECspd > 100):
    ACCspeedField.delete(0, 'end')
    ACCspeedField.insert(0,"10")
    DECspeedField.delete(0, 'end')
    DECspeedField.insert(0,"10")
  ACCramp = float(ACCrampField.get())
  if(ACCramp <= .01 or ACCramp > 100):
    ACCrampField.delete(0, 'end')
    ACCrampField.insert(0,"50")



def ErrorHandler(response):
  #global estopActive
  #global posOutreach
  #Curtime = datetime.now().strftime("%B %d %Y - %I:%M%p")
  cmdRecEntryField.delete(0, 'end')
  cmdRecEntryField.insert(0,response)
  messages = [] #list to hold possible multiple error messages
  ##AXIS LIMIT ERROR
  if (response[1:2] == 'L'):
    if (response[2:3] == '1'):
      messages.append("J1 Axis Limit")
    if (response[3:4] == '1'):
      messages.append("J2 Axis Limit")
    if (response[4:5] == '1'):
      messages.append("J3 Axis Limit")
    if (response[5:6] == '1'):
      messages.append("J4 Axis Limit")
    if (response[6:7] == '1'):
      messages.append("J5 Axis Limit")
    if (response[7:8] == '1'):
      messages.append("J6 Axis Limit")
    if (response[8:9] == '1'):
      messages.append("J7 Axis Limit")
    if (response[9:10] == '1'):
      messages.append("J8 Axis Limit")
    if (response[10:11] == '1'):
      messages.append("J9 Axis Limit")

    # Actions to take on axis limit error
    cmdRecEntryField.delete(0, 'end')
    cmdRecEntryField.insert(0,response)            
    alarm_message = "Axis Limit Error - See Log"
    #Progstop()

  ##COLLISION ERROR   
  elif (response[1:2] == 'C'):
    if (response[2:3] == '1'):
      messages.append("J1 Collision or Motor Error")
    if (response[3:4] == '1'):
      messages.append("J2 Collision or Motor Error")
    if (response[4:5] == '1'):
      messages.append("J3 Collision or Motor Error")
    if (response[5:6] == '1'):
      messages.append("J4 Collision or Motor Error")
    if (response[6:7] == '1'):
      messages.append("J5 Collision or Motor Error")
    if (response[7:8] == '1'):
      messages.append("J6 Collision or Motor Error")

    # Actions to take on collision all errors
    correctPos()
    stopProg()        
    alarm_message = "Collision or Motor Error - See Log"

  ##REACH ERROR   
  elif (response[1:2] == 'R'):
    with program_stop_state_lock:
      RUN['posOutreach'] = TRUE
    stopProg()
    message = "Position Out of Reach"
    messages.append(message)
    alarm_message = message

  ##SPLINE ERROR   
  elif (response[1:2] == 'S'):  
    stopProg()
    message = "Spline Can Only Have Move L Types"
    messages.append(message)
    alarm_message = message

  ##GCODE ERROR   
  elif (response[1:2] == 'G'):
    stopProg()
    message = "Gcode file not found"
    messages.append(message)
    alarm_message = message

  ##ESTOP BUTTON   
  elif (response[1:2] == 'B'):
    with program_stop_state_lock:
      RUN['estopActive'] = TRUE
    stopProg()
    message = "Estop Button was Pressed"
    messages.append(message)
    alarm_message = message    

  ##CALIBRATION ERROR 
  elif (response[1:2] == 'A'):  
    if (response[2:3] == '1'):
      messages.append("J1 CALIBRATION ERROR")
    if (response[2:3] == '2'):
      messages.append("J2 CALIBRATION ERROR")
    if (response[2:3] == '3'):
      messages.append("J3 CALIBRATION ERROR")
    if (response[2:3] == '4'):
      messages.append("J4 CALIBRATION ERROR")
    if (response[2:3] == '5'):
      messages.append("J5 CALIBRATION ERROR")
    if (response[2:3] == '6'):
      messages.append("J6 CALIBRATION ERROR") 
    if (response[2:3] == '7'):
      messages.append("J7 CALIBRATION ERROR")
    if (response[2:3] == '8'):
      messages.append("J8 CALIBRATION ERROR")
    if (response[2:3] == '9'):
      messages.append("J9 CALIBRATION ERROR")

    alarm_message = "Calibration Error - See Log"             
     
  ##MODBUS ERROR   
  elif (response == 'Modbus Error'):
    stopProg()
    message = "Modbus Error"
    messages.append(message)
    alarm_message = message
  
  else:
    stopProg() 
    message = "Unknown Error"
    messages.append(message)
    alarm_message = message

  # After taking actions for each error type, log all messages
  for msg in messages:
    logger.error(msg)

  # After logging all message, save the error log
  value=tab8.ElogView.get(0,END)
  pickle.dump(value,open("ErrorLog","wb"))

  # Update the alarm status label once
  almStatusLab.config(text=alarm_message, style="Alarm.TLabel")
  almStatusLab2.config(text=alarm_message, style="Alarm.TLabel")
  GCalmStatusLab.config(text=alarm_message, style="Alarm.TLabel")
      
	
	

###VISION DEFS###################################################################
#################################################################################	
 
def viscalc():
  # global RUN['xMMpos']
  # global RUN['yMMpos']
  #origin x1 y1
  CAL['VisOrigXpix'] = float(VisX1PixEntryField.get())
  CAL['VisOrigXmm'] = float(VisX1RobEntryField.get()) 
  CAL['VisOrigYpix'] = float(VisY1PixEntryField.get()) 
  CAL['VisOrigYmm'] = float(VisY1RobEntryField.get()) 
  # x2 y2
  CAL['VisEndXpix'] = float(VisX2PixEntryField.get())
  CAL['VisEndXmm'] = float(VisX2RobEntryField.get()) 
  CAL['VisEndYpix'] = float(VisY2PixEntryField.get()) 
  CAL['VisEndYmm'] = float(VisY2RobEntryField.get())

  x = float(VisRetXpixEntryField.get()) 
  y = float(VisRetYpixEntryField.get()) 

  XPrange = float(CAL['VisEndXpix']) - float(CAL['VisOrigXpix'])
  XPratio = (x-float(CAL['VisOrigXpix'])) / XPrange
  XMrange = float(CAL['VisEndXmm']) - float(CAL['VisOrigXmm'])
  XMpos = float(XMrange) * float(XPratio)
  RUN['xMMpos'] = float(CAL['VisOrigXmm']) + XMpos
  ##
  YPrange = float(CAL['VisEndYpix']) - float(CAL['VisOrigYpix'])
  YPratio = (y-float(CAL['VisOrigYpix'])) / YPrange
  YMrange = float(CAL['VisEndYmm']) - float(CAL['VisOrigYmm'])
  YMpos = float(YMrange) * float(YPratio)
  RUN['yMMpos'] = float(CAL['VisOrigYmm']) + YMpos
  return (RUN['xMMpos'],RUN['yMMpos'])





# Define function to show frame
def show_frame():

    if RUN['cam_on']:

        ret, frame = RUN['cap'].read()    

        if ret:
            cv2image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)    
            img = Image.fromarray(cv2image).resize((480,320))
            imgtk = ImageTk.PhotoImage(image=img)        
            live_lbl.imgtk = imgtk    
            live_lbl.configure(image=imgtk)    
        
        live_lbl.after(10, show_frame)

def start_vid():
    #global cam_on, cap
    #global cap
    stop_vid()
    RUN['cam_on'] = True
    curVisStringSel = visoptions.get()
    for i in range(len(camList)):
      if curVisStringSel == camList[i]:
          RUN['selectedCam'] = i
          break
    #RUN['cap'] = cv2.VideoCapture(RUN['selectedCam']) 
    RUN['cap'] = cv2.VideoCapture(RUN['selectedCam'], cv2.CAP_DSHOW)
    for _ in range(5):
      RUN['cap'].read()
    show_frame()




def stop_vid():
    #global cam_on
    RUN['cam_on'] = False
    
    if RUN['cap']:
        RUN['cap'].release()

#vismenu.size

def take_pic():
  # global RUN['selectedCam']
  #global cap
  # global RUN['BGavg']
  # global RUN['mX1']
  # global RUN['mY1']
  # global RUN['mX2']
  # global RUN['mY2']

  try:
    if(RUN['cam_on']):
      ret, frame = RUN['cap'].read()
    else:
      curVisStingSel = visoptions.get()
      l = len(camList)
      for i in range(l):
        if (visoptions.get() == camList[i]):
          RUN['selectedCam'] = i
      RUN['cap'] = cv2.VideoCapture(RUN['selectedCam']) 
      ret, frame = RUN['cap'].read()

    brightness = int(VisBrightSlide.get())
    contrast = int(VisContrastSlide.get())
    CAL['zoom'] = int(VisZoomSlide.get())

    frame = np.int16(frame)
    frame = frame * (contrast/127+1) - contrast + brightness
    frame = np.clip(frame, 0, 255)
    frame = np.uint8(frame) 
    cv2image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) 
    

    #get the webcam size
    height, width = cv2image.shape

    #prepare the crop
    centerX,centerY=int(height/2),int(width/2)
    radiusX,radiusY= int(CAL['zoom']*height/100),int(CAL['zoom']*width/100)

    minX,maxX=centerX-radiusX,centerX+radiusX
    minY,maxY=centerY-radiusY,centerY+radiusY

    cropped = cv2image[minX:maxX, minY:maxY]
    cv2image = cv2.resize(cropped, (width, height))

    CAL['autoBGVal'] = int(RUN['autoBG'].get())
    if(CAL['autoBGVal']==1):
      x1 = int(float(VisX1PixEntryField.get()))
      y1 = int(float(VisY1PixEntryField.get()))
      x2 = int(float(VisX2PixEntryField.get()))
      y2 = int(float(VisY1PixEntryField.get()))
      x3 = int(float(VisX1PixEntryField.get()))
      y3 = int(float(VisY2PixEntryField.get()))
      BG1 = cv2image[x1][y1]
      BG2 = cv2image[x2][y2]
      BG3 = cv2image[x3][y3]
      avg = int(mean([BG1,BG2,BG3]))
      RUN['BGavg'] = (avg,avg,avg) 
      background = avg
      VisBacColorEntryField.configure(state='enabled')  
      VisBacColorEntryField.delete(0, 'end')
      VisBacColorEntryField.insert(0,str(RUN['BGavg']))
      VisBacColorEntryField.configure(state='disabled')  
    else:
      temp = VisBacColorEntryField.get()  
      startIndex = temp.find("(")
      endIndex = temp.find(",")
      background = int(temp[startIndex+1:endIndex])
      #background = eval(VisBacColorEntryField.get())

    h = cv2image.shape[0]
    w = cv2image.shape[1]
    # loop over the image
    for y in range(0, h):
      for x in range(0, w):
        # change the pixel
        cv2image[y, x] = background if x >= RUN['mX2'] or x <= RUN['mX1'] or y <= RUN['mY1'] or y >= RUN['mY2'] else cv2image[y, x]  

    img = Image.fromarray(cv2image).resize((640,480))

    

    imgtk = ImageTk.PhotoImage(image=img) 
    vid_lbl.imgtk = imgtk    
    vid_lbl.configure(image=imgtk) 
    filename = 'curImage.jpg'
    cv2.imwrite(filename, cv2image)
  except:
    print("camera failed")

def mask_pic():
  # global RUN['selectedCam']
  #global cap
  # global RUN['BGavg']
  # global RUN['mX1']
  # global RUN['mY1']
  # global RUN['mX2']
  # global RUN['mY2']

  if(RUN['cam_on']):
    ret, frame = RUN['cap'].read()
  else:
    curVisStingSel = visoptions.get()
    l = len(camList)
    for i in range(l):
      if (visoptions.get() == camList[i]):
        RUN['selectedCam'] = i
    RUN['cap'] = cv2.VideoCapture(RUN['selectedCam']) 
    ret, frame = RUN['cap'].read()
  brightness = int(VisBrightSlide.get())
  contrast = int(VisContrastSlide.get())
  CAL['zoom'] = int(VisZoomSlide.get())
  frame = np.int16(frame)
  frame = frame * (contrast/127+1) - contrast + brightness
  frame = np.clip(frame, 0, 255)
  frame = np.uint8(frame) 
  cv2image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) 
  #get the webcam size
  height, width = cv2image.shape
  #prepare the crop
  centerX,centerY=int(height/2),int(width/2)
  radiusX,radiusY= int(CAL['zoom']*height/100),int(CAL['zoom']*width/100)
  minX,maxX=centerX-radiusX,centerX+radiusX
  minY,maxY=centerY-radiusY,centerY+radiusY
  cropped = cv2image[minX:maxX, minY:maxY]
  cv2image = cv2.resize(cropped, (width, height))
  #img = Image.fromarray(cv2image).resize((640,480))
  #imgtk = ImageTk.PhotoImage(image=img) 
  #vid_lbl.imgtk = imgtk    
  #vid_lbl.configure(image=imgtk) 
  filename = 'curImage.jpg'
  cv2.imwrite(filename, cv2image)

  



def mask_crop(event, x, y, flags, param):
    # global RUN['x_start'], RUN['y_start'], RUN['x_end'], RUN['y_end'], RUN['cropping']
    #global oriImage
    # global RUN['box_points']
    # global RUN['button_down']
    # global RUN['mX1']
    # global RUN['mY1']
    # global RUN['mX2']
    # global RUN['mY2']


    cropDone = False
    

    if (not RUN['button_down']) and (event == cv2.EVENT_LBUTTONDOWN):
        RUN['x_start'], RUN['y_start'], RUN['x_end'], RUN['y_end'] = x, y, x, y
        RUN['cropping'] = True
        RUN['button_down'] = True
        RUN['box_points'] = [(x, y)]
        
    # Mouse is Moving
    elif (RUN['button_down']) and (event == cv2.EVENT_MOUSEMOVE):
        if RUN['cropping']:
            image_copy = RUN['oriImage'].copy()
            RUN['x_end'], RUN['y_end'] = x, y
            point = (x, y)
            cv2.rectangle(image_copy, RUN['box_points'][0], point, (0, 255, 0), 2)
            cv2.imshow("image", image_copy)

    # if the left mouse button was released
    elif event == cv2.EVENT_LBUTTONUP:
        RUN['button_down'] = False
        RUN['box_points'].append((x, y))
        cv2.rectangle(RUN['oriImage'], RUN['box_points'][0], RUN['box_points'][1], (0, 255, 0), 2)
        cv2.imshow("image", RUN['oriImage'])
        # record the ending (x, y) coordinates
        RUN['x_end'], RUN['y_end'] = x, y
        RUN['cropping'] = False # cropping is finished

        RUN['mX1'] = RUN['x_start']+3
        RUN['mY1'] = RUN['y_start']+3
        RUN['mX2'] = RUN['x_end']-3
        RUN['mY2'] = RUN['y_end']-3

        CAL['autoBGVal'] = int(RUN['autoBG'].get())
        if(CAL['autoBGVal']==1):
          BG1 = RUN['oriImage'][int(VisX1PixEntryField.get())][int(VisY1PixEntryField.get())]
          BG2 = RUN['oriImage'][int(VisX1PixEntryField.get())][int(VisY2PixEntryField.get())]
          BG3 = RUN['oriImage'][int(VisX2PixEntryField.get())][int(VisY2PixEntryField.get())]
          avg = int(mean([BG1,BG2,BG3]))
          RUN['BGavg'] = (avg,avg,avg) 
          background = avg
          VisBacColorEntryField.configure(state='enabled')  
          VisBacColorEntryField.delete(0, 'end')
          VisBacColorEntryField.insert(0,str(RUN['BGavg']))
          VisBacColorEntryField.configure(state='disabled')   
        else:  
          background = eval(VisBacColorEntryField.get())

        h = RUN['oriImage'].shape[0]
        w = RUN['oriImage'].shape[1]
        # loop over the image
        for y in range(0, h):
            for x in range(0, w):
                # change the pixel
                RUN['oriImage'][y, x] = background if x >= RUN['mX2'] or x <= RUN['mX1'] or y <= RUN['mY1'] or y >= RUN['mY2'] else RUN['oriImage'][y, x]

        img = Image.fromarray(RUN['oriImage'])
        imgtk = ImageTk.PhotoImage(image=img) 
        vid_lbl.imgtk = imgtk    
        vid_lbl.configure(image=imgtk) 
        filename = 'curImage.jpg'
        cv2.imwrite(filename, RUN['oriImage'])
        cv2.destroyAllWindows()



def selectMask():
  #global oriImage
  # global RUN['button_down']
  RUN['button_down'] = False
  RUN['x_start'], RUN['y_start'], RUN['x_end'], RUN['y_end'] = 0, 0, 0, 0
  mask_pic()

  image = cv2.imread('curImage.jpg')
  RUN['oriImage'] = image.copy()
  
  cv2.namedWindow("image")
  cv2.setMouseCallback("image", mask_crop)
  cv2.imshow("image", image)



def mouse_crop(event, x, y, flags, param):
    # global RUN['x_start'], RUN['y_start'], RUN['x_end'], RUN['y_end'], RUN['cropping']
    #global oriImage
    # global RUN['box_points']
    # global RUN['button_down']

    cropDone = False
    

    if (not RUN['button_down']) and (event == cv2.EVENT_LBUTTONDOWN):
        RUN['x_start'], RUN['y_start'], RUN['x_end'], RUN['y_end'] = x, y, x, y
        RUN['cropping'] = True
        RUN['button_down'] = True
        RUN['box_points'] = [(x, y)]
        
    # Mouse is Moving
    elif (RUN['button_down']) and (event == cv2.EVENT_MOUSEMOVE):
        if RUN['cropping']:
            image_copy = RUN['oriImage'].copy()
            RUN['x_end'], RUN['y_end'] = x, y
            point = (x, y)
            cv2.rectangle(image_copy, RUN['box_points'][0], point, (0, 255, 0), 2)
            cv2.imshow("image", image_copy)

    # if the left mouse button was released
    elif event == cv2.EVENT_LBUTTONUP:
        RUN['button_down'] = False
        RUN['box_points'].append((x, y))
        cv2.rectangle(RUN['oriImage'], RUN['box_points'][0], RUN['box_points'][1], (0, 255, 0), 2)
        cv2.imshow("image", RUN['oriImage'])
        # record the ending (x, y) coordinates
        RUN['x_end'], RUN['y_end'] = x, y
        RUN['cropping'] = False # cropping is finished

        refPoint = [(RUN['x_start']+3, RUN['y_start']+3), (RUN['x_end']-3, RUN['y_end']-3)]

        if len(refPoint) == 2: #when two points were found
            roi = RUN['oriImage'][refPoint[0][1]:refPoint[1][1], refPoint[0][0]:refPoint[1][0]]
            
            cv2.imshow("Cropped", roi)
            USER_INP = simpledialog.askstring(title="Teach Vision Object",
                                  prompt="Save Object As:")
            templateName = USER_INP+".jpg"                      
            cv2.imwrite(templateName, roi)
            cv2.destroyAllWindows()
            updateVisOp()  



def selectTemplate():
  #global oriImage
  # global RUN['button_down']
  RUN['button_down'] = False
  RUN['x_start'], RUN['y_start'], RUN['x_end'], RUN['y_end'] = 0, 0, 0, 0
  image = cv2.imread('curImage.jpg')
  RUN['oriImage'] = image.copy()
  
  cv2.namedWindow("image")
  cv2.setMouseCallback("image", mouse_crop)
  cv2.imshow("image", image)




def snapFind():
  # global RUN['selectedTemplate']
  # global RUN['BGavg']
  take_pic()
  template = RUN['selectedTemplate'].get()
  min_score = float(VisScoreEntryField.get())*.01
  CAL['autoBGVal'] = int(RUN['autoBG'].get())
  if(CAL['autoBGVal']==1):
    background = RUN['BGavg']
    VisBacColorEntryField.configure(state='enabled')  
    VisBacColorEntryField.delete(0, 'end')
    VisBacColorEntryField.insert(0,str(RUN['BGavg']))
    VisBacColorEntryField.configure(state='disabled')  
  else:  
    background = eval(VisBacColorEntryField.get())
  visFind(template,min_score,background)




def rotate_image(img,angle,background):
    image_center = tuple(np.array(img.shape[1::-1]) / 2)
    rot_mat = cv2.getRotationMatrix2D(image_center, -angle, 1.0)
    result = cv2.warpAffine(img, rot_mat, img.shape[1::-1],borderMode=cv2.BORDER_CONSTANT, borderValue=background, flags=cv2.INTER_LINEAR)
    return result

def visFind(template,min_score,background):
    # global RUN['xMMpos']
    # global RUN['yMMpos']
    #global autoBG

    if(background == "Auto"):
      background = RUN['BGavg']
      VisBacColorEntryField.configure(state='enabled')  
      VisBacColorEntryField.delete(0, 'end')
      VisBacColorEntryField.insert(0,str(RUN['BGavg']))
      VisBacColorEntryField.configure(state='disabled')  
      

    green = (0,255,0)
    red = (255,0,0)
    blue = (0,0,255)
    dkgreen = (0,128,0)
    status = "fail"
    highscore = 0
    img1 = cv2.imread('curImage.jpg')  # target Image
    img2 = cv2.imread(template)  # target Image
    
    #method = cv2.TM_CCOEFF_NORMED
    #method = cv2.TM_CCORR_NORMED

    img = img1.copy()

    CAL['fullRotVal'] = int(RUN['fullRot'].get())

    for i in range (1):
      if(i==0):
        method = cv2.TM_CCOEFF_NORMED
      else:
        #method = cv2.TM_CCOEFF_NORMED
        method = cv2.TM_CCORR_NORMED  

      #USE 1/3 - EACH SIDE SEARCH
      if (CAL['fullRotVal'] == 0): 
        ## fist pass 1/3rds
        curangle = 0
        highangle = 0
        highscore = 0
        highmax_loc = 0
        for x in range(3):
          template = img2
          template = rotate_image(img2,curangle,background)
          w, h = template.shape[1::-1]
          res = cv2.matchTemplate(img,template,method)
          min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
          if(max_val>highscore):
            highscore=max_val
            highangle=curangle
            highmax_loc=max_loc
            highw,highh = w,h
          curangle += 120
        
        #check each side and narrow in
        while True:
          curangle=curangle/2
          if(curangle<.9):
            break
          nextangle1 = highangle+curangle
          nextangle2 = highangle-curangle
          template = img2
          template = rotate_image(img2,nextangle1,background)
          w, h = template.shape[1::-1]
          res = cv2.matchTemplate(img,template,method)
          min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
          if(max_val>highscore):
            highscore=max_val
            highangle=nextangle1
            highmax_loc=max_loc
            highw,highh = w,h
          template = img2
          template = rotate_image(img2,nextangle2,background)
          w, h = template.shape[1::-1]
          res = cv2.matchTemplate(img,template,method)
          min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
          if(max_val>highscore):
            highscore=max_val
            highangle=nextangle2
            highmax_loc=max_loc
            highw,highh = w,h     
    
      #USE FULL 360 SEARCh
      else:
        for i in range (720):
          template = rotate_image(img2,i,background)
          w, h = template.shape[1::-1]

          img = img1.copy()
          # Apply template Matching
          res = cv2.matchTemplate(img,template,method)
          min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)
          highscore=max_val
          highangle=i
          highmax_loc=max_loc
          highw,highh = w,h
          if highscore >= min_score:
            break
      if(i==1):
        highscore = highscore*.5    
      if highscore >= min_score:
        break         

    if highscore >= min_score:
      status = "pass"
      #normalize angle to increment of +180 to -180
      if(highangle>180):
        highangle = -360 + highangle
      #pick closest 180   
      CAL['pick180Val'] = int(RUN['pick180'].get())  
      if (CAL['pick180Val'] == 1):
        if (highangle>90):
          highangle = -180 + highangle
        elif (highangle<-90):
          highangle = 180 + highangle
      #try closest
      CAL['pickClosestVal'] = int(RUN['pickClosest'].get())
      if (CAL['pickClosestVal'] == highangle and highangle>int(CAL['J6PosLim'])):
        highangle=CAL['J6PosLim']
      elif (CAL['pickClosestVal'] == 0 and highangle>int(CAL['J6PosLim'])):    
        status = "fail"
      if (CAL['pickClosestVal'] == 1 and highangle<(int(CAL['J6NegLim'])*-1)):
        highangle=CAL['J6NegLim']*-1
      elif (CAL['pickClosestVal'] == 0 and highangle<(int(CAL['J6NegLim'])*-1)):  
        status = "fail"

      top_left = highmax_loc
      bottom_right = (top_left[0] + highw, top_left[1] + highh)
      #find center
      center = (top_left[0] + highw/2, top_left[1] + highh/2)
      xPos = int(center[1])
      yPos = int(center[0])

      imgxPos = int(center[0])
      imgyPos = int(center[1])

      #find line 1 end
      line1x = int(imgxPos + 60*math.cos(math.radians(highangle-90)))
      line1y = int(imgyPos + 60*math.sin(math.radians(highangle-90)))
      cv2.line(img, (imgxPos,imgyPos), (line1x,line1y), green, 3) 

      #find line 2 end
      line2x = int(imgxPos + 60*math.cos(math.radians(highangle+90)))
      line2y = int(imgyPos + 60*math.sin(math.radians(highangle+90)))
      cv2.line(img, (imgxPos,imgyPos), (line2x,line2y), green, 3)  

      #find line 3 end
      line3x = int(imgxPos + 30*math.cos(math.radians(highangle)))
      line3y = int(imgyPos + 30*math.sin(math.radians(highangle)))
      cv2.line(img, (imgxPos,imgyPos), (line3x,line3y), green, 3)

      #find line 4 end
      line4x = int(imgxPos + 30*math.cos(math.radians(highangle+180)))
      line4y = int(imgyPos + 30*math.sin(math.radians(highangle+180)))
      cv2.line(img, (imgxPos,imgyPos), (line4x,line4y), green, 3) 

      #find tip start
      lineTx = int(imgxPos + 56*math.cos(math.radians(highangle-90)))
      lineTy = int(imgyPos + 56*math.sin(math.radians(highangle-90)))
      cv2.line(img, (lineTx,lineTy), (line1x,line1y), dkgreen, 2) 



      cv2.circle(img, (imgxPos,imgyPos), 20, green, 1)
      #cv2.rectangle(img,top_left, bottom_right, green, 2)
      cv2.imwrite('temp.jpg', img)
      img = Image.fromarray(img).resize((640,480))
      imgtk = ImageTk.PhotoImage(image=img)        
      vid_lbl.imgtk = imgtk    
      vid_lbl.configure(image=imgtk)
      VisRetScoreEntryField.delete(0, 'end')
      VisRetScoreEntryField.insert(0,str(round((highscore*100),2))) 
      VisRetAngleEntryField.delete(0, 'end')
      VisRetAngleEntryField.insert(0,str(highangle)) 
      VisRetXpixEntryField.delete(0, 'end')
      VisRetXpixEntryField.insert(0,str(xPos))
      VisRetYpixEntryField.delete(0, 'end')
      VisRetYpixEntryField.insert(0,str(yPos))           
      viscalc()
      VisRetXrobEntryField .delete(0, 'end')
      VisRetXrobEntryField .insert(0,str(round(RUN['xMMpos'],2)))  
      VisRetYrobEntryField .delete(0, 'end')
      VisRetYrobEntryField .insert(0,str(round(RUN['yMMpos'],2)))  

      


          #break
        #if (score > highscore):
          #highscore=score


    if status == "fail":
      cv2.rectangle(img,(5,5), (635,475), red, 5)
      cv2.imwrite('temp.jpg', img)
      img = Image.fromarray(img).resize((640,480))
      imgtk = ImageTk.PhotoImage(image=img)        
      vid_lbl.imgtk = imgtk    
      vid_lbl.configure(image=imgtk)
      VisRetScoreEntryField.delete(0, 'end')
      VisRetScoreEntryField.insert(0,str(round((highscore*100),2)))
      VisRetAngleEntryField.delete(0, 'end')
      VisRetAngleEntryField.insert(0,"NA")
      VisRetXpixEntryField.delete(0, 'end')
      VisRetXpixEntryField.insert(0,"NA")
      VisRetYpixEntryField.delete(0, 'end')
      VisRetYpixEntryField.insert(0,"NA") 

    return (status)    
    





# initial vis attempt using sift with flann pattern match
#def visFind(template):
#  take_pic()
#  MIN_MATCH_COUNT = 10
#  img1 = cv2.imread(template)  # query Image
#  img2 = cv2.imread('curImage.jpg')  # target Image
#  # Initiate SIFT detector
#  sift = cv2.SIFT_create()
#  # find the keypoints and descriptors with SIFT
#  kp1, des1 = sift.detectAndCompute(img1,None)
#  kp2, des2 = sift.detectAndCompute(img2,None)
#  FLANN_INDEX_KDTREE = 1
#  index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
#  search_params = dict(checks = 50)
#  flann = cv2.FlannBasedMatcher(index_params, search_params)
#  matches = flann.knnMatch(des1,des2,k=2)
#  # store all the good matches as per Lowe's ratio test.
#  good = []
#  for m,n in matches:
#      if m.distance < 1.1*n.distance:
#          good.append(m)

#  if len(good)>MIN_MATCH_COUNT:
#      src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2)
#      dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2)
#      M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
#      matchesMask = mask.ravel().tolist()
#      h,w,c = img1.shape
#      pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
#      dst = cv2.perspectiveTransform(pts,M)
#      #img2 = cv.polylines(img2,[np.int32(dst)],True,255,3, cv.LINE_AA)
#
#      pts = np.float32([ [0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)
#      dst = cv2.perspectiveTransform(pts,M)
#
#      crosspts = np.float32([ [w/2,0],[w/2,h-1],[0,h/2],[w-1,h/2] ]).reshape(-1,1,2)
#      crossCoord = cv2.perspectiveTransform(crosspts,M)
#
#      cenPt = np.float32([w/2,h/2]).reshape(-1,1,2)
#      cenCoord = cv2.perspectiveTransform(cenPt,M)
#
#      cenResult = cenCoord[0].reshape(1,-1).flatten().tolist()
#      theta = - math.atan2(M[0,1], M[0,0]) * 180 / math.pi
#
#      xPos = cenResult[0]
#      yPos = cenResult[1]
#
#      cross1Result = crossCoord[0].reshape(2,-1).flatten().tolist()
#      cross2Result = crossCoord[1].reshape(2,-1).flatten().tolist()
#      cross3Result = crossCoord[2].reshape(2,-1).flatten().tolist()
#      cross4Result = crossCoord[3].reshape(2,-1).flatten().tolist()
#
#      x1Pos = int(cross1Result[0])
#      y1Pos = int(cross1Result[1])
#      x2Pos = int(cross2Result[0])
#      y2Pos = int(cross2Result[1])
#      x3Pos = int(cross3Result[0])
#      y3Pos = int(cross3Result[1])
#      x4Pos = int(cross4Result[0])
#      y4Pos = int(cross4Result[1])
#
#
#      print(xPos)
#      print(yPos)
#      print(theta)
#
#
#      #draw bounding box
#      #img2 = cv2.polylines(img2, [np.int32(dst)], True, (0,255,0),3, cv2.LINE_AA)
#
#      #draw circle
#      img2 = cv2.circle(img2, (int(xPos),int(yPos)), radius=30, color=(0, 255, 0), thickness=3)
#
#      #draw line 1
#      cv2.line(img2, (x1Pos,y1Pos), (x2Pos,y2Pos), (0,255,0), 3) 
#      #draw line 2
#      cv2.line(img2, (x3Pos,y3Pos), (x4Pos,y4Pos), (0,255,0), 3)
#
#      #save image
#      cv2.imwrite('curImage.jpg', img2)
#      img = Image.fromarray(img2)
#      imgtk = ImageTk.PhotoImage(image=img)        
#      vid_lbl.imgtk = imgtk    
#      vid_lbl.configure(image=imgtk) 
#
#
#
#
#  else:
#      print( "Not enough matches are found - {}/{}".format(len(good), MIN_MATCH_COUNT) )
#      matchesMask = None 




def updateVisOp(filelist=None):
  # global RUN['selectedTemplate']
  RUN['selectedTemplate'] = StringVar()
  if filelist is None:
    filelist = _startup_visual_options()
  elif isinstance(filelist, (str, bytes)):
    raise TypeError("visual options must be a filename sequence")
  else:
    filelist = tuple(filelist)
    if not all(
      isinstance(filename, str)
      and filename.endswith('.jpg')
      and os.path.basename(filename) == filename
      for filename in filelist
    ):
      raise ValueError("visual options contain an invalid filename")
  Visoptmenu = ttk.Combobox(tab6, textvariable=RUN['selectedTemplate'], values=filelist, state='readonly')
  Visoptmenu.place(x=390, y=52)
  Visoptmenu.bind("<<ComboboxSelected>>", VisOpUpdate)




def VisOpUpdate(foo):
    file = RUN['selectedTemplate'].get()
    logger.info(file)

    # Load image
    img = cv2.imread(file, cv2.IMREAD_COLOR)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    # --- Square preview settings ---
    TARGET_SIZE = 150   # final image will be 150x150

    h, w = img.shape[:2]

    # Scale so the longest side fits TARGET_SIZE
    scale = TARGET_SIZE / max(w, h)
    new_w = int(w * scale)
    new_h = int(h * scale)

    img = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_AREA)

    # Create square canvas (background color can be changed)
    square = np.zeros((TARGET_SIZE, TARGET_SIZE, 3), dtype=np.uint8)
    # Example gray background instead of black:
    # square[:] = (40, 40, 40)

    # Center the resized image
    x = (TARGET_SIZE - new_w) // 2
    y = (TARGET_SIZE - new_h) // 2
    square[y:y+new_h, x:x+new_w] = img

    # Convert to Tk image
    img = Image.fromarray(square)
    imgtk = ImageTk.PhotoImage(image=img)

    template_lbl.imgtk = imgtk
    template_lbl.configure(image=imgtk, anchor='center')



def zeroBrCn():
  # global RUN['mX1']
  # global RUN['mY1']
  # global RUN['mX2']
  # global RUN['mY2']
  RUN['mX1'] = 0
  RUN['mY1'] = 0
  RUN['mX2'] = 640
  RUN['mY2'] = 480
  VisBrightSlide.set(0)
  VisContrastSlide.set(0)
  #VisZoomSlide.set(50)
  take_pic()

def VisUpdateBriCon(foo):
  take_pic()  

  
  
       
def motion(event):
    y = event.x
    x = event.y

    if (x<=240 and y<=320):
      VisX1PixEntryField.delete(0, 'end')
      VisX1PixEntryField.insert(0,x)
      VisY1PixEntryField.delete(0, 'end')
      VisY1PixEntryField.insert(0,y)
    elif (x>240):
      VisX2PixEntryField.delete(0, 'end')
      VisX2PixEntryField.insert(0,x)
    elif (y>320):   
      VisY2PixEntryField.delete(0, 'end')
      VisY2PixEntryField.insert(0,y)

    

def checkAutoBG():
  CAL['autoBGVal'] = int(RUN['autoBG'].get())
  if(CAL['autoBGVal']==1):
    VisBacColorEntryField.configure(state='disabled')
  else:
    VisBacColorEntryField.configure(state='enabled')  



### GCODE DEFS ###################################################################
##################################################################################




def gcodeFrame():
  gcodeframe=Frame(tab7)
  gcodeframe.place(x=300,y=10)
  #progframe.pack(side=RIGHT, fill=Y)
  scrollbar = Scrollbar(gcodeframe) 
  scrollbar.pack(side=RIGHT, fill=Y)
  tab7.gcodeView = Listbox(gcodeframe,width=105,height=46, yscrollcommand=scrollbar.set)
  tab7.gcodeView.bind('<<ListboxSelect>>', gcodeViewselect)
  time.sleep(.1)
  tab7.gcodeView.pack()
  scrollbar.config(command=tab7.gcodeView.yview)



def gcodeViewselect(e):
  gcodeRow = tab7.gcodeView.curselection()[0]
  GcodCurRowEntryField.delete(0, 'end')
  GcodCurRowEntryField.insert(0,gcodeRow)  


def loadGcodeProg():
  filetypes = (('gcode files', '*.gcode *.nc *.ngc *.cnc *.tap'),('text files', '*.txt'))
  filename = fd.askopenfilename(title='Open files',initialdir='/',filetypes=filetypes)
  GcodeProgEntryField.delete(0, 'end')
  GcodeProgEntryField.insert(0,filename)
  gcodeProg = open(GcodeProgEntryField.get(),"rb")
  tab7.gcodeView.delete(0,END)
  previtem = ""
  for item in gcodeProg:
    try:
      commentIndex=item.find(b";")
      item = item[:commentIndex]
    except:
      pass
    item=item + b" " 
    if(item != previtem ):
      tab7.gcodeView.insert(END,item)
    previtem = item 
  tab7.gcodeView.pack()
  gcodescrollbar.config(command=tab7.gcodeView.yview)

def SetGcodeStartPos():
  GC_ST_E1_EntryField.delete(0, 'end')
  GC_ST_E1_EntryField.insert(0,str(CAL['XcurPos']))
  GC_ST_E2_EntryField.delete(0, 'end')
  GC_ST_E2_EntryField.insert(0,str(CAL['YcurPos']))  
  GC_ST_E3_EntryField.delete(0, 'end')
  GC_ST_E3_EntryField.insert(0,str(CAL['ZcurPos']))  
  GC_ST_E4_EntryField.delete(0, 'end')
  GC_ST_E4_EntryField.insert(0,str(CAL['RzcurPos']))  
  GC_ST_E5_EntryField.delete(0, 'end')
  GC_ST_E5_EntryField.insert(0,str(CAL['RycurPos']))  
  GC_ST_E6_EntryField.delete(0, 'end')
  GC_ST_E6_EntryField.insert(0,str(CAL['RxcurPos']))
  GC_ST_WC_EntryField.delete(0, 'end')
  GC_ST_WC_EntryField.insert(0,str(RUN['WC']))  

@_manual_motion_request("G-code start-position motion")
def MoveGcodeStartPos():
  RUN['xVal'] = str(float(GC_ST_E1_EntryField.get())+float(GC_SToff_E1_EntryField.get()))
  RUN['yVal'] = str(float(GC_ST_E2_EntryField.get())+float(GC_SToff_E2_EntryField.get()))
  RUN['zVal'] = str(float(GC_ST_E3_EntryField.get())+float(GC_SToff_E3_EntryField.get()))
  rzVal = str(float(GC_ST_E4_EntryField.get())+float(GC_SToff_E4_EntryField.get()))
  ryVal = str(float(GC_ST_E5_EntryField.get())+float(GC_SToff_E5_EntryField.get()))
  rxVal = str(float(GC_ST_E6_EntryField.get())+float(GC_SToff_E6_EntryField.get()))
  J7Val = str(CAL['J7PosCur'])
  J8Val = str(CAL['J8PosCur'])
  J9Val = str(CAL['J9PosCur'])
  speedPrefix = "Sm"
  Speed = "25"
  ACCspd = "10"
  DECspd = "10"
  ACCramp = "100"
  RUN['WC'] = GC_ST_WC_EntryField.get()
  LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
  command = "MJ"+"X"+RUN['xVal']+"Y"+RUN['yVal']+"Z"+RUN['zVal']+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"J7"+J7Val+"J8"+J8Val+"J9"+J9Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
  commandVR = "MJ"+"X"+RUN['xVal']+"Y"+RUN['yVal']+"Z"+RUN['zVal']+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"\n"
  physical_command = None
  if not RUN['offlineMode']:
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0, command)
    physical_command = command
  return _start_manual_motion(
    physical_command,
    "G-code start-position motion",
    mj_command,
    commandVR,
  )
  



def GCstepFwd():
    GCalmStatusLab.config(text="GCODE READY",  style="OK.TLabel")
    if GCexecuteRow() != ROW_EXECUTION_COMPLETE:
      return False
    GCselRow = tab7.gcodeView.curselection()[0]
    last = tab7.gcodeView.index('end')
    for row in range (0,GCselRow):
      tab7.gcodeView.itemconfig(row, {'fg': "#1E90FF"})
    tab7.gcodeView.itemconfig(GCselRow, {'fg': "#0057A6"})
    for row in range (GCselRow+1,last):
      tab7.gcodeView.itemconfig(row, {'fg': "#959697"})
    tab7.gcodeView.selection_clear(0, END)
    GCselRow += 1
    tab7.gcodeView.select_set(GCselRow)
    try:
      GCselRow = tab7.gcodeView.curselection()[0]
      GcodCurRowEntryField.delete(0, 'end')
      GcodCurRowEntryField.insert(0,GCselRow)
    except:
      GcodCurRowEntryField.delete(0, 'end')
      GcodCurRowEntryField.insert(0,"---")
    return True

@_tracked_serial_operation("ser")
def GCdelete():
  if(GcodeFilenameField.get() != ""):
    try:
      Filename = _gcode_storage_filename(GcodeFilenameField.get())
    except MotionInputError as exc:
      message = f"G-code deletion rejected: {exc}"
      logger.error(message)
      GCalmStatusLab.config(text=message, style="Alarm.TLabel")
      return False
    command = "DG"+"Fn"+Filename+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0,command)
    response = _exchange_legacy_main_command(command)
    if (response[:1] == 'E'):
      ErrorHandler(response)   
    else:
      if(response == "P"):
        text = Filename + " has been deleted"
        GCalmStatusLab.config(text= text,  style="OK.TLabel")
        status = "no"
        GCread(status)
      elif(response == "F"):
        text = Filename + " was not found"
        GCalmStatusLab.config(text= text,  style="Alarm.TLabel")
    return True
  else:
    messagebox.showwarning("warning","Please Enter a Filename")
    return False

@_tracked_serial_operation("ser")
def GCread(status):
  command = "RG"+"\n"
  cmdSentEntryField.delete(0, 'end')
  cmdSentEntryField.insert(0,command)
  response = _exchange_legacy_main_command(command)
  if (response[:1] == 'E'):
    ErrorHandler(response)   
  else:
    if(status == "yes"):
      GCalmStatusLab.config(text= "FILES FOUND ON SD CARD:",  style="OK.TLabel")
    GcodeProgEntryField.delete(0, 'end')
    tab7.gcodeView.delete(0,END)
    for value in response.split(","):
      tab7.gcodeView.insert(END,value)
    tab7.gcodeView.pack()
    gcodescrollbar.config(command=tab7.gcodeView.yview)


def GCplay():
  Filename = GcodeFilenameField.get()
  GCplayProg(Filename)

  

def GCplayProg(Filename, completion_callback=None):
  if completion_callback is not None and not callable(completion_callback):
    raise TypeError("G-code completion callback must be callable")
  if RUN['offlineMode']:
    message = "G-code playback is unavailable while offline"
    logger.error(message)
    GCalmStatusLab.config(text=message, style="Alarm.TLabel")
    return False
  try:
    command = _gcode_playback_command(Filename)
  except MotionInputError as exc:
    message = f"G-code playback rejected: {exc}"
    logger.error(message)
    GCalmStatusLab.config(text=message, style="Alarm.TLabel")
    return False

  request_lease = _acquire_motion_request("G-code playback")
  if request_lease is None:
    GCalmStatusLab.config(
      text=_motion_request_rejection_message(
        "GCODE FILE NOT STARTED; ANOTHER MOTION REQUEST IS ACTIVE"
      ),
      style="Warn.TLabel",
    )
    return False

  GCalmStatusLab.config(text="GCODE FILE RUNNING", style="OK.TLabel")

  def complete_playback(controller_position):
    if controller_position is not None and not isinstance(
      controller_position,
      PositionResponse,
    ):
      raise RuntimeError(
        "G-code playback returned an invalid controller position"
      )
    succeeded = controller_position is not None
    try:
      if succeeded:
        GCalmStatusLab.config(text="GCODE FILE COMPLETE", style="Warn.TLabel")
      elif RUN['estopActive']:
        GCalmStatusLab.config(
          text="Estop Button was Pressed",
          style="Alarm.TLabel",
        )
      else:
        GCalmStatusLab.config(text="GCODE FILE FAILED", style="Alarm.TLabel")
    finally:
      _finish_motion_request(
        request_lease,
        completion_callback,
        succeeded is True,
      )

  try:
    started = start_send_serial_thread(
      command,
      completion_callback=complete_playback,
    )
  except Exception:
    logger.exception("Unable to start G-code playback worker")
    started = False
  if not started:
    if motion_request_registry.owns(request_lease):
      _finish_motion_request(request_lease)
    GCalmStatusLab.config(
      text="GCODE FILE NOT STARTED",
      style="Alarm.TLabel",
    )
    return False
  return True


@_tracked_serial_operation("ser")
def GCconvertProg():
  if(GcodeProgEntryField.get() == ""):
    messagebox.showwarning("warning","Please Load a Gcode Program") 
  elif (GcodeFilenameField.get() == ""):  
    messagebox.showwarning("warning","Please Enter a Filename") 
  else:
    try:
      Filename = _gcode_storage_filename(GcodeFilenameField.get())
    except MotionInputError as exc:
      message = f"G-code conversion rejected: {exc}"
      logger.error(message)
      GCalmStatusLab.config(text=message, style="Alarm.TLabel")
      return False
    command = "DG"+"Fn"+Filename+"\n"
    cmdSentEntryField.delete(0, 'end')
    cmdSentEntryField.insert(0,command)
    response = _exchange_legacy_main_command(command)
    last = tab7.gcodeView.index('end')
    for row in range (0,last):
      tab7.gcodeView.itemconfig(row, {'fg': "#000000"})
    def GCthreadProg():
      # global RUN['GCrowinproc']
      # global RUN['prevxVal']
      # global RUN['prevyVal']
      # global RUN['prevzVal']
      RUN['prevxVal'] = 0
      RUN['prevyVal'] = 0
      RUN['prevzVal'] = 0
      try:
        GCselRow = tab7.gcodeView.curselection()[0]
        if (GCselRow == 0):
          GCselRow=1
      except:
        GCselRow=1
        tab7.gcodeView.selection_clear(0, END)
        tab7.gcodeView.select_set(GCselRow)
      tab7.GCrunTrue = 1
      while tab7.GCrunTrue == 1:
        if (tab7.GCrunTrue == 0):
          GCalmStatusLab.config(text="GCODE CONVERSION STOPPED",  style="Alarm.TLabel")
        else:
          GCalmStatusLab.config(text="GCODE CONVERSION RUNNING",  style="OK.TLabel")
        RUN['GCrowinproc'] = 1
        execution_state = ROW_EXECUTION_PENDING
        while tab7.GCrunTrue == 1:
          while serial_lock.locked() and tab7.GCrunTrue == 1:
            time.sleep(.01)
          if tab7.GCrunTrue == 0:
            RUN['GCrowinproc'] = 0
            return
          execution_state = GCexecuteRow()
          if execution_state == ROW_EXECUTION_COMPLETE:
            break
          if execution_state == ROW_EXECUTION_REJECTED:
            tab7.GCrunTrue = 0
            RUN['GCrowinproc'] = 0
            return
          if execution_state != ROW_EXECUTION_PENDING:
            tab7.GCrunTrue = 0
            RUN['GCrowinproc'] = 0
            GCalmStatusLab.config(
              text="GCODE CONVERSION RETURNED AN INVALID ROW STATE",
              style="Alarm.TLabel",
            )
            return
          time.sleep(.01)
        if tab7.GCrunTrue != 1:
          RUN['GCrowinproc'] = 0
          return
        while RUN['GCrowinproc'] == 1:
          time.sleep(.1)
        if (
          execution_state != ROW_EXECUTION_COMPLETE
          or tab7.GCrunTrue != 1
        ):
          return
        GCselRow = tab7.gcodeView.curselection()[0]
        tab7.gcodeView.itemconfig(GCselRow, {'fg': "#0057A6"})
        tab7.gcodeView.selection_clear(0, END)
        GCselRow += 1
        tab7.gcodeView.select_set(GCselRow)
        try:
          GCselRow = tab7.gcodeView.curselection()[0]
          GcodCurRowEntryField.delete(0, 'end')
          GcodCurRowEntryField.insert(0,GCselRow)
        except:
          GcodCurRowEntryField.delete(0, 'end')
          GcodCurRowEntryField.insert(0,"---") 
          tab7.GCrunTrue = 0
          GCalmStatusLab.config(text="GCODE CONVERSION STOPPED",  style="Alarm.TLabel")
    GCt = threading.Thread(target=GCthreadProg)
    GCt.start()    

     


@_tracked_serial_operation(
  "ser",
  rejection_result=None,
)
def _exchange_gcode_row(command):
    command = _canonicalize_main_serial_command(command)
    response_timeout = _controller_response_timeout(command)
    serial_port = RUN.get('ser')
    try:
      acquired = serial_write_lock.acquire()
      if acquired is False:
        raise RuntimeError("G-code serial write lock acquisition failed")
      try:
        if tab7.GCrunTrue != 1:
          return None
        write_serial_control(
          serial_port,
          command,
          reset_input=True,
        )
      finally:
        serial_write_lock.release()
      return read_serial_line_response(serial_port, response_timeout)
    finally:
      if (
        RUN.get('ser') is serial_port
        and not getattr(serial_port, "is_open", False)
      ):
        RUN['ser'] = None


def GCstopProg():
    tab7.GCrunTrue = 0
    message = "GCODE SCHEDULING HALTED; ACTIVE CONTROLLER MOTION IS NOT PREEMPTED"
    GCalmStatusLab.config(text=message, style="Alarm.TLabel")
    return True


@_synchronous_motion_request(
  "G-code conversion row",
  rejection_result=ROW_EXECUTION_PENDING,
)
@_tracked_serial_operation(
  "ser",
  rejection_result=ROW_EXECUTION_PENDING,
)
def GCexecuteRow():
  # global RUN['GCrowinproc']
  # global RUN['LineDist']
  # global RUN['Xv']
  # global RUN['Yv']
  # global RUN['Zv']
  #global moveInProc
  #global gcodeSpeed
  #global inchTrue
  # global RUN['prevxVal']
  # global RUN['prevyVal']
  # global RUN['prevzVal']
  # global RUN['xVal']
  # global RUN['yVal']
  # global RUN['zVal']
  if tab7.GCrunTrue != 1:
    RUN['GCrowinproc'] = 0
    return ROW_EXECUTION_REJECTED
  GCstartTime = time.time()
  GCselRow = tab7.gcodeView.curselection()[0]
  tab7.gcodeView.see(GCselRow+2)
  data = list(map(int, tab7.gcodeView.curselection()))
  command=tab7.gcodeView.get(data[0]).decode()
  RUN['cmdType'] =command[:1]
  subCmd=command[1:command.find(" ")].rstrip()


  ## F ##
  if (RUN['cmdType'] == "F"):
    RUN['gcodeSpeed'] = _gcode_feed_rate_mm_per_second(
      command[command.find("F")+1:],
      RUN['inchTrue'],
    )


  ## G ##
  if (RUN['cmdType'] == "G"):

    #IMPERIAL
    if (subCmd == "20"):
      RUN['inchTrue'] = True; 
    
    #METRIC
    if (subCmd == "21"):
      RUN['inchTrue'] = False;
    
    #ABSOLUTE / INCREMENTAL - HOME (absolute is forced and moves to start position offset)
    if (subCmd == "90" or subCmd == "91" or subCmd == "28"):
      
      RUN['xVal'] = str(float(GC_ST_E1_EntryField.get())+float(GC_SToff_E1_EntryField.get()))
      RUN['yVal'] = str(float(GC_ST_E2_EntryField.get())+float(GC_SToff_E2_EntryField.get()))
      RUN['zVal'] = str(float(GC_ST_E3_EntryField.get())+float(GC_SToff_E3_EntryField.get()))
      rzVal = str(float(GC_ST_E4_EntryField.get())+float(GC_SToff_E4_EntryField.get()))
      ryVal = str(float(GC_ST_E5_EntryField.get())+float(GC_SToff_E5_EntryField.get()))
      rxVal = str(float(GC_ST_E6_EntryField.get())+float(GC_SToff_E6_EntryField.get()))
      J7Val = str(CAL['J7PosCur'])
      J8Val = str(CAL['J8PosCur'])
      J9Val = str(CAL['J9PosCur'])
      speedPrefix = "Sm"
      Speed = "25"
      ACCspd = "10"
      DECspd = "10"
      ACCramp = "100"
      RUN['WC'] = GC_ST_WC_EntryField.get()
      LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
      Filename = _gcode_storage_filename(GcodeFilenameField.get())
      command = "WC"+"X"+RUN['xVal']+"Y"+RUN['yVal']+"Z"+RUN['zVal']+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"J7"+J7Val+"J8"+J8Val+"J9"+J9Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"Fn"+Filename+"\n"
      cmdSentEntryField.delete(0, 'end') 
      
      print(str(command))

      if tab7.GCrunTrue != 1:
        RUN['GCrowinproc'] = 0
        return ROW_EXECUTION_REJECTED
      cmdSentEntryField.insert(0,command)
      try:
        response = _exchange_gcode_row(command)
      except Exception as exc:
        tab7.GCrunTrue = 0
        RUN['GCrowinproc'] = 0
        message = f"Unable to write G-code row: {exc}"
        logger.exception(message)
        GCalmStatusLab.config(text=message, style="Alarm.TLabel")
        return ROW_EXECUTION_REJECTED
      if response is None:
        RUN['GCrowinproc'] = 0
        return ROW_EXECUTION_REJECTED
      print(str(response))
      if (response[:1] == 'E'):
        ErrorHandler(response)
        GCstopProg()
        tab7.GCrunTrue = 0
        RUN['GCrowinproc'] = 0
        GCalmStatusLab.config(text="UNABLE TO WRITE TO SD CARD",  style="Alarm.TLabel")
        return ROW_EXECUTION_REJECTED
      else:
        if _apply_valid_position_response(response) is None:
          tab7.GCrunTrue = 0
          RUN['GCrowinproc'] = 0
          GCalmStatusLab.config(
            text="INVALID G-CODE CONTROLLER RESPONSE",
            style="Alarm.TLabel",
          )
          return ROW_EXECUTION_REJECTED


    #LINEAR MOVE
    if (subCmd == "0" or subCmd == "1"):

      if("X" in command):
        xtemp=command[command.find("X")+1:]     
        RUN['xVal'] =xtemp[:xtemp.find(" ")]
        RUN['xVal'] =str(round(float(RUN['xVal']),3))
      else:
        RUN['xVal'] =""  
      if("Y" in command):
        ytemp=command[command.find("Y")+1:]     
        RUN['yVal'] =ytemp[:ytemp.find(" ")]
        RUN['yVal'] =str(round(float(RUN['yVal']),3))
      else:
        RUN['yVal'] =""
      if("Z" in command):
        ztemp=command[command.find("Z")+1:]     
        RUN['zVal'] =ztemp[:ztemp.find(" ")]
        RUN['zVal'] =str(round(float(RUN['zVal']),3))
      else:
        RUN['zVal'] =""
      if("A" in command):
        atemp=command[command.find("A")+1:]     
        aVal=atemp[:atemp.find(" ")]
        aVal=str(round(float(aVal),3))
      else:
        aVal=""
      if("B" in command):
        btemp=command[command.find("B")+1:]     
        bVal=btemp[:btemp.find(" ")]
        bVal=str(round(float(bVal),3))
      else:
        bVal=""
      if("C" in command):
        ctemp=command[command.find("C")+1:]     
        cVal=ctemp[:ctemp.find(" ")]
        cVal=str(round(float(cVal),3))
      else:
        cVal=""
      if("E" in command):
        etemp=command[command.find("E")+1:]     
        eVal=etemp[:etemp.find(" ")]
        eVal=str(round(float(eVal),3))
      else:
        eVal=""
      if("F" in command):
        ftemp=command[command.find("F")+1:]     
        fVal=ftemp[:ftemp.find(" ")]
        fVal=str(round(float(fVal),3))
      else:
        fVal=""
       


      if(RUN['xVal'] != ""):
        if(RUN['inchTrue']):
          RUN['xVal'] =str(float(RUN['xVal'])*25.4)
        RUN['xVal'] = str(round((float(GC_ST_E1_EntryField.get())+float(RUN['xVal'])),3))
      else:
        try:
          if(RUN['prevxVal'] != 0):
            RUN['xVal'] = RUN['prevxVal']
          else:  
            RUN['xVal'] = str(CAL['XcurPos'])
        except:
          RUN['xVal'] = str(CAL['XcurPos'])   


      if(RUN['yVal'] != ""):
        if(RUN['inchTrue']):
          RUN['yVal'] =str(float(RUN['yVal'])*25.4)
        RUN['yVal'] = str(round((float(GC_ST_E2_EntryField.get())+float(RUN['yVal'])),3))
      else:
        try:
          if(RUN['prevyVal'] != 0):
            RUN['yVal'] = RUN['prevyVal']
          else: 
            RUN['yVal'] = str(CAL['YcurPos'])
        except:
          RUN['yVal'] = str(CAL['YcurPos'])  
        
      if(RUN['zVal'] != ""):
        if(RUN['inchTrue']):
          RUN['zVal'] =str(float(RUN['zVal'])*25.4)
        RUN['zVal'] = str(round((float(GC_ST_E3_EntryField.get())+float(RUN['zVal'])),3))
      else:
        try:
          if(RUN['prevzVal'] != 0):
            RUN['zVal'] = RUN['prevzVal']
          else: 
            RUN['zVal'] = str(CAL['ZcurPos'])
        except:
          RUN['zVal'] = str(CAL['ZcurPos'])          

      if(aVal != ""):
        rzVal = str(float(GC_ST_E4_EntryField.get())+float(aVal))
        if (np.sign(float(rzVal)) != np.sign(float(CAL['RzcurPos']))):
          rzVal=str(round((float(rzVal)*-1),3))
      else:
        rzVal = str(CAL['RzcurPos'])
      
      if(bVal != ""):
        ryVal = str(round((float(GC_ST_E5_EntryField.get())+float(bVal)),3))
      else:
        ryVal = str(CAL['RycurPos'])

      if(cVal != ""):
        rxVal = str(round((float(GC_ST_E6_EntryField.get())+float(cVal)),3))
      else:
        rxVal = str(CAL['RxcurPos'])

      if(eVal != ""):
        J7Val = eVal
      else:
        J7Val = str(CAL['J7PosCur'])
      
      J8Val = str(CAL['J8PosCur'])
      J9Val = str(CAL['J9PosCur'])
      
      if(fVal != ""):
        RUN['gcodeSpeed'] = _gcode_feed_rate_mm_per_second(
          fVal,
          RUN['inchTrue'],
        )
      speedPrefix = "Sm"
      Speed = RUN['gcodeSpeed']



      if (subCmd == "0"):
        Speed = speedEntryField.get()

      #FORCE ROTATIONS TO BASE VALUE FOR NOW
      rzVal = GC_ST_E4_EntryField.get()
      ryVal = GC_ST_E5_EntryField.get()
      rxVal = GC_ST_E6_EntryField.get()

      #ACCspd = ACCspeedField.get()
      #DECspd = DECspeedField.get()
      #ACCramp = ACCrampField.get()

      ACCspd = ".1"
      DECspd = ".1"
      ACCramp = "100"


      RUN['WC'] = GC_ST_WC_EntryField.get()
      #LoopMode = str(CAL['J1OpenLoopVal'].get())+str(CAL['J2OpenLoopVal'].get())+str(CAL['J3OpenLoopVal'].get())+str(CAL['J4OpenLoopVal'].get())+str(CAL['J5OpenLoopVal'].get())+str(CAL['J6OpenLoopVal'].get())
      LoopMode ="111111"
      #DisWrist = str(CAL['DisableWristRotVal'].get())
      Filename = _gcode_storage_filename(GcodeFilenameField.get())

      command = "WC"+"X"+RUN['xVal']+"Y"+RUN['yVal']+"Z"+RUN['zVal']+"Rz"+rzVal+"Ry"+ryVal+"Rx"+rxVal+"J7"+J7Val+"J8"+J8Val+"J9"+J9Val+speedPrefix+Speed+"Ac"+ACCspd+"Dc"+DECspd+"Rm"+ACCramp+"W"+RUN['WC']+"Lm"+LoopMode+"Fn"+Filename+"\n"
      RUN['prevxVal'] = RUN['xVal']
      RUN['prevyVal'] = RUN['yVal']
      RUN['prevzVal'] = RUN['zVal']
      cmdSentEntryField.delete(0, 'end')
      cmdSentEntryField.insert(0,command)

      #tab8.ElogView.insert(END, command)
      #value=tab8.ElogView.get(0,END)
      #pickle.dump(value,open("ErrorLog","wb"))

      if tab7.GCrunTrue != 1:
        RUN['GCrowinproc'] = 0
        return ROW_EXECUTION_REJECTED
      try:
        response = _exchange_gcode_row(command)
      except Exception as exc:
        tab7.GCrunTrue = 0
        RUN['GCrowinproc'] = 0
        message = f"Unable to write G-code row: {exc}"
        logger.exception(message)
        GCalmStatusLab.config(text=message, style="Alarm.TLabel")
        return ROW_EXECUTION_REJECTED
      if response is None:
        RUN['GCrowinproc'] = 0
        return ROW_EXECUTION_REJECTED
      if (response[:1] == 'E'):
        tab7.GCrunTrue = 0
        RUN['GCrowinproc'] = 0
        GCalmStatusLab.config(text="UNABLE TO WRITE TO SD CARD",  style="Alarm.TLabel")
        ErrorHandler(response)
        return ROW_EXECUTION_REJECTED
      else:
        if _apply_valid_position_response(response) is None:
          tab7.GCrunTrue = 0
          RUN['GCrowinproc'] = 0
          GCalmStatusLab.config(
            text="INVALID G-CODE CONTROLLER RESPONSE",
            style="Alarm.TLabel",
          )
          return ROW_EXECUTION_REJECTED

  RUN['GCrowinproc'] = 0
  return ROW_EXECUTION_COMPLETE

  

   


        



  
####################################################################################################################################################
####################################################################################################################################################
####################################################################################################################################################
"""
COMPLETE TAB 1 REFACTORING - FROM .place() TO .grid()
This replaces lines 11249-12534 in the original AR4.py file
"""

#####TAB 1
##########################################################################


def posRegFieldVisible(self):
  """Show/hide position register field based on move type selection"""
  curCmdtype = options.get()
  if (curCmdtype=="Move PR" or curCmdtype=="OFF PR " or curCmdtype=="Teach PR"):
    SavePosEntryField.grid()  # Show the field
  else:
    SavePosEntryField.grid_remove()  # Hide the field

# Tkinter variables for Tab 1
speedOption = StringVar(tab1)
options = StringVar(tab1)

# Entry fields for command builders (created here, placed in frames later)
waitTimeEntryField = Entry(tab1, width=4, justify="center")
tabNumEntryField = Entry(tab1, width=4, justify="center")
jumpTabEntryField = Entry(tab1, width=4, justify="center")
servoNumEntryField = Entry(tab1, width=4, justify="center")
servoPosEntryField = Entry(tab1, width=4, justify="center")
regNumEntryField = Entry(tab1, width=4, justify="center")
regEqEntryField = Entry(tab1, width=4, justify="center")
visPassEntryField = Entry(tab1, width=4, justify="center")
visFailEntryField = Entry(tab1, width=4, justify="center")
changeProgEntryField = Entry(tab1, width=14, justify="center")
PlayGCEntryField = Entry(tab1, width=14, justify="center")
auxPortEntryField = Entry(tab1, width=4, justify="center")
auxCharEntryField = Entry(tab1, width=4, justify="center")
storPosNumEntryField = Entry(tab1, width=4, justify="center")
storPosElEntryField = Entry(tab1, width=4, justify="center")
storPosValEntryField = Entry(tab1, width=4, justify="center")
waitTimeoutEntryField = Entry(tab1, width=4, justify="center")
waitInputEntryField = Entry(tab1, width=4, justify="center")
waitInputOffEntryField = Entry(tab1, width=4, justify="center")
waitVarEntryField = Entry(tab1, width=4, justify="center")
outputOnEntryField = Entry(tab1, width=4, justify="center")
outputOffEntryField = Entry(tab1, width=4, justify="center")
setInputEntryField = Entry(tab1, width=4, justify="center")
setVarEntryField = Entry(tab1, width=4, justify="center")
IfInputEntryField = Entry(tab1, width=4, justify="center")
IfVarEntryField = Entry(tab1, width=4, justify="center")
IfDestEntryField = Entry(tab1, width=4, justify="center")

IfVarEntryField = Entry(tab1, width=4, justify="center")
IfInputEntryField = Entry(tab1, width=4, justify="center")
waitVarEntryField = Entry(tab1, width=4, justify="center")
waitInputEntryField = Entry(tab1, width=4, justify="center")
waitTimeoutEntryField = Entry(tab1, width=5, justify="center")
setVarEntryField = Entry(tab1, width=4, justify="center")
setInputEntryField = Entry(tab1, width=4, justify="center")
auxPortEntryField = Entry(tab1, width=4, justify="center")
auxCharEntryField = Entry(tab1, width=4, justify="center")
storPosNumEntryField = Entry(tab1, width=4, justify="center")
storPosElEntryField = Entry(tab1, width=4, justify="center")
storPosValEntryField = Entry(tab1, width=4, justify="center")



# All Entry Fields for Tab 1


### TAB 1 - MAIN CONTROLS - REFACTORED TO USE GRID LAYOUT
##########################################################################

# Configure tab1 main grid layout (3-column design)
tab1.grid_rowconfigure(0, weight=1)  # Main content area expands
tab1.grid_columnconfigure(0, weight=0, minsize=200)  # Left panel - fixed minimum width (narrower)
tab1.grid_columnconfigure(1, weight=1, minsize=550)  # Center panel - expands
tab1.grid_columnconfigure(2, weight=0, minsize=800)  # Right panel - fixed minimum width

# ============================================================================
# LEFT PANEL - Program Controls
# ============================================================================
leftPanel = Frame(tab1, relief="raised", borderwidth=1)
leftPanel.grid(row=0, column=0, sticky="nsew", padx=2, pady=2)

# Configure left panel grid
leftPanel.grid_rowconfigure(20, weight=1)  # Spacer row at bottom to push controls to top
leftPanel.grid_columnconfigure(0, weight=1)
leftPanel.grid_columnconfigure(1, weight=1)

# Row 0: Program label and entry (with more top padding)
ProgLab = Label(leftPanel, text="Program:")
ProgLab.grid(row=0, column=0, sticky="w", padx=5, pady=(8, 2))

ProgEntryField = Entry(leftPanel, width=15, justify="center")
ProgEntryField.grid(row=0, column=1, sticky="ew", padx=5, pady=(8, 2))

# Row 1: Load button
loadBut = ttk.Button(leftPanel, text="Load", command=loadProg)
loadBut.grid(row=1, column=0, columnspan=2, sticky="ew", padx=5, pady=2)

# Row 2: New Prog button
savProg = ttk.Button(leftPanel, text="New Prog", command=CreateProg)
savProg.grid(row=2, column=0, columnspan=2, sticky="ew", padx=5, pady=2)

# Row 3: Incremental Jog checkbox and entry field
IncJogCbut = Checkbutton(leftPanel, text="Inc Jog", variable=RUN['IncJogStat'])
IncJogCbut.grid(row=3, column=0, sticky="w", padx=5, pady=2)

incrementEntryField = Entry(leftPanel, width=6, justify="center")
incrementEntryField.grid(row=3, column=1, sticky="ew", padx=5, pady=2)

# Row 4: Current Row
curRowLab = Label(leftPanel, text="Current Row:")
curRowLab.grid(row=4, column=0, sticky="w", padx=5, pady=2)

curRowEntryField = Entry(leftPanel, width=6, justify="center")
curRowEntryField.grid(row=4, column=1, sticky="ew", padx=5, pady=2)

# Row 5: Motion controls frame (with Xbox button at bottom)
speedFrame = LabelFrame(leftPanel, text="Motion", padding=5)
speedFrame.grid(row=5, column=0, columnspan=2, sticky="ew", padx=5, pady=5)

speedFrame.grid_columnconfigure(1, weight=1)

speedLab = Label(speedFrame, text="Speed")
speedLab.grid(row=0, column=0, sticky="w", padx=2, pady=1)

speedEntryField = Entry(speedFrame, width=4, justify="center")
speedEntryField.grid(row=0, column=1, sticky="ew", padx=2, pady=1)

speedOptionMenu = OptionMenu(speedFrame, speedOption, "Percent", "Percent", "Seconds", "mm per Sec")
speedOptionMenu.grid(row=0, column=2, sticky="ew", padx=2, pady=1)

ACCLab = Label(speedFrame, text="Acceleration")
ACCLab.grid(row=1, column=0, sticky="w", padx=2, pady=1)

ACCspeedField = Entry(speedFrame, width=4, justify="center")
ACCspeedField.grid(row=1, column=1, sticky="ew", padx=2, pady=1)

Label(speedFrame, text="%").grid(row=1, column=2, sticky="w", padx=2)

DECLab = Label(speedFrame, text="Deceleration")
DECLab.grid(row=2, column=0, sticky="w", padx=2, pady=1)

DECspeedField = Entry(speedFrame, width=4, justify="center")
DECspeedField.grid(row=2, column=1, sticky="ew", padx=2, pady=1)

Label(speedFrame, text="%").grid(row=2, column=2, sticky="w", padx=2)

RampLab = Label(speedFrame, text="Ramp")
RampLab.grid(row=3, column=0, sticky="w", padx=2, pady=1)

ACCrampField = Entry(speedFrame, width=4, justify="center")
ACCrampField.grid(row=3, column=1, sticky="ew", padx=2, pady=1)

Label(speedFrame, text="%").grid(row=3, column=2, sticky="w", padx=2)

RoundLab = Label(speedFrame, text="Rounding")
RoundLab.grid(row=4, column=0, sticky="w", padx=2, pady=1)

roundEntryField = Entry(speedFrame, width=4, justify="center")
roundEntryField.grid(row=4, column=1, sticky="ew", padx=2, pady=1)

Label(speedFrame, text="mm").grid(row=4, column=2, sticky="w", padx=2)

# Xbox button at bottom of Motion frame (centered)
if CE['Platform']['IS_WINDOWS']:
    xboxBut = Button(speedFrame, command=start_xbox)
else:
    xboxBut = Button(speedFrame, command=xbox)
xboxPhoto = PhotoImage(file="xbox.png")
xboxBut.config(image=xboxPhoto)
xboxBut.grid(row=5, column=0, columnspan=3, pady=(5, 0))

# Row 6: Virtual controls frame
virtualFrame = LabelFrame(leftPanel, text="Virtual", padding=5)
virtualFrame.grid(row=6, column=0, columnspan=2, sticky="ew", padx=5, pady=5)

virtualFrame.grid_columnconfigure(0, weight=1)

virtRobBut = ttk.Button(virtualFrame, text="Virtual Robot", command=lambda: launch_vtk_nonblocking(tab1))
virtRobBut.grid(row=0, column=0, sticky="ew", padx=2, pady=2)

offline_button = ttk.Button(virtualFrame, text="Run Offline", width=20, command=toggle_offline_mode, style="Online.TButton")
offline_button.grid(row=1, column=0, sticky="ew", padx=2, pady=2)
# Row 7: Position Commands frame
posFrame = LabelFrame(leftPanel, text="Position Commands", padding=5)
posFrame.grid(row=7, column=0, columnspan=2, sticky="new", padx=5, pady=(2, 5))

posFrame.grid_columnconfigure(0, weight=1)

moveSelMenu = OptionMenu(posFrame, options, "Move J", "Move J", "Move L", "Move R", "OFF J", "Move PR", "OFF PR ", "Teach PR", "Move A Mid", "Move A End", "Move C Center", "Move C Start", "Move C Plane", "Start Spline", "End Spline", "Move Vis", command=posRegFieldVisible)
moveSelMenu.grid(row=0, column=0, sticky="ew", padx=2, pady=2)

# Position Register Entry Field (hidden by default, shown when PR moves selected)
SavePosEntryField = Entry(posFrame, width=4, justify="center")
SavePosEntryField.grid(row=1, column=0, sticky="ew", padx=2, pady=2)
SavePosEntryField.grid_remove()  # Hidden by default, shown by posRegFieldVisible()

teachPosBut = ttk.Button(posFrame, text="Teach New Position", command=teachInsertBelSelected)
teachPosBut.grid(row=2, column=0, sticky="ew", padx=2, pady=2)

modPosBut = ttk.Button(posFrame, text="Modify Position", command=teachReplaceSelected)
modPosBut.grid(row=3, column=0, sticky="ew", padx=2, pady=2)

deleteBut = ttk.Button(posFrame, text="Delete", command=deleteitem)
deleteBut.grid(row=4, column=0, sticky="ew", padx=2, pady=2)

autoCalBut = ttk.Button(posFrame, text="Auto Calibrate CMD", command=insCalibrate)
CalibrateBut = autoCalBut  # Alias for compatibility
autoCalBut.grid(row=5, column=0, sticky="ew", padx=2, pady=2)

# Row 8: Vision container
visionFrame = LabelFrame(leftPanel, text="Vision", padding=5)
visionFrame.grid(row=8, column=0, columnspan=2, sticky="ew", padx=5, pady=(2, 5))

visionFrame.grid_columnconfigure(0, weight=1)
visionFrame.grid_columnconfigure(1, weight=1)
visionFrame.grid_columnconfigure(2, weight=1)
visionFrame.grid_columnconfigure(3, weight=1)

# Row 0: Camera On and Camera Off buttons
camOnBut = ttk.Button(visionFrame, text="Camera On", command=cameraOn)
camOnBut.grid(row=0, column=0, columnspan=2, sticky="ew", padx=2, pady=2)

camOffBut = ttk.Button(visionFrame, text="Camera Off", command=cameraOff)
camOffBut.grid(row=0, column=2, columnspan=2, sticky="ew", padx=2, pady=2)

# Row 1: Vision Find button
visFindBut = ttk.Button(visionFrame, text="Vision Find", command=insertvisFind)
visFindBut.grid(row=1, column=0, columnspan=2, sticky="ew", padx=2, pady=(5, 2))

# Row 2: Pass Tab and Fail Tab labels
Label(visionFrame, text="Pass Tab", font=("Arial", 8)).grid(row=2, column=0, sticky="e", padx=(2, 1))
Label(visionFrame, text="Fail Tab", font=("Arial", 8)).grid(row=2, column=1, sticky="w", padx=(1, 2))

# Row 3: Pass Tab and Fail Tab entry fields
visPassEntryField = Entry(visionFrame, width=6, justify="center")
visPassEntryField.grid(row=3, column=0, sticky="ew", padx=(2, 1), pady=(0, 2))

visFailEntryField = Entry(visionFrame, width=6, justify="center")
visFailEntryField.grid(row=3, column=1, sticky="ew", padx=(1, 2), pady=(0, 2))

# Vision Find additional fields (placeholders - full UI on Tab 6)
VisBacColorEntryField = Entry(tab1, width=6)  # Hidden, used by insertvisFind
VisScoreEntryField = Entry(tab1, width=6)  # Hidden, used by insertvisFind
VisBacColorEntryField.insert(0, "116, 116, 116")  # Default background color
VisScoreEntryField.insert(0, "85")  # Default score threshold

# Row 9: Wait container
waitContainer = LabelFrame(leftPanel, text="Wait - Stop", padding=5)
waitContainer.grid(row=9, column=0, columnspan=2, sticky="ew", padx=5, pady=(2, 5))

waitContainer.grid_columnconfigure(0, weight=1)
waitContainer.grid_columnconfigure(1, weight=0)

waitSecBut = ttk.Button(waitContainer, text="Wait Time (seconds)", command=waitTime)
waitSecBut.grid(row=0, column=0, sticky="ew", padx=2, pady=2)

waitSecField = Entry(waitContainer, width=8, justify="center")
waitSecField.grid(row=0, column=1, sticky="w", padx=2, pady=2)

stopBut = ttk.Button(waitContainer, text="Stop", command=insertStop)
stopBut.grid(row=1, column=0, sticky="ew", padx=2, pady=2)

# ============================================================================
# CENTER PANEL - Program Display and Controls
# ============================================================================
centerPanel = Frame(tab1)
centerPanel.grid(row=0, column=1, sticky="nsew", padx=2, pady=2)

# Configure center panel grid
centerPanel.grid_rowconfigure(0, weight=0)  # Status message
centerPanel.grid_rowconfigure(1, weight=0)  # Play controls
centerPanel.grid_rowconfigure(2, weight=3)  # Program view (more weight) (expands)
centerPanel.grid_rowconfigure(3, weight=0)  # Manual entry (at bottom)
centerPanel.grid_rowconfigure(4, weight=0)  # Position controls
centerPanel.grid_rowconfigure(5, weight=0)  # Command builders
centerPanel.grid_rowconfigure(6, weight=0)  # Bottom buttons
centerPanel.grid_columnconfigure(0, weight=1)

# Row 0: Status message
runStatusLab = Label(centerPanel, text="SYSTEM STARTING - PLEASE WAIT", font=("Arial", 10, "bold"), style="OK.TLabel")
runStatusLab.grid(row=0, column=0, sticky="ew", padx=5, pady=5)

# Create alias for compatibility with existing code
almStatusLab = runStatusLab

# Row 0.5: Play controls
playFrame = Frame(centerPanel)
playFrame.grid(row=1, column=0, sticky="ew", padx=5, pady=5)

for i in range(4):
    playFrame.grid_columnconfigure(i, weight=1)

# Load play/stop button images
try:
    playPhoto = PhotoImage(file="play-icon.png")
    stopPhoto = PhotoImage(file="stop-icon.png")
    use_images = True
except:
    use_images = False

# Import tk to get regular Button (not ttk.Button)
import tkinter as tk_base

playBut = tk_base.Button(playFrame, command=runProg)
if use_images:
    playBut.config(image=playPhoto)
    playBut.image = playPhoto  # Keep reference
else:
    playBut.config(text="▶", width=8, height=2)
playBut.grid(row=0, column=0, sticky="nsew", padx=2, pady=0)

revBut = tk_base.Button(playFrame, text="REV", command=stepRev, height=2, font=("Arial", 10))
revBut.grid(row=0, column=1, sticky="nsew", padx=2, pady=0)

fwdBut = tk_base.Button(playFrame, text="FWD", command=stepFwd, height=2, font=("Arial", 10))
fwdBut.grid(row=0, column=2, sticky="nsew", padx=2, pady=0)

stopBut = tk_base.Button(playFrame, command=stopProg)
if use_images:
    stopBut.config(image=stopPhoto)
    stopBut.image = stopPhoto  # Keep reference
else:
    stopBut.config(text="⬛", width=8, height=2)
stopBut.grid(row=0, column=3, sticky="nsew", padx=2, pady=0)

# Row 2: Program view with scrollbar
progframe = Frame(centerPanel, relief="sunken", borderwidth=1)
progframe.grid(row=2, column=0, sticky="nsew", padx=5, pady=5)

scrollbar = Scrollbar(progframe)
scrollbar.pack(side=RIGHT, fill=Y)

tab1.progView = Listbox(progframe, exportselection=0, width=70, height=28, yscrollcommand=scrollbar.set)
tab1.progView.bind('<<ListboxSelect>>', progViewselect)
tab1.progView.pack(side=LEFT, fill=BOTH, expand=True)

scrollbar.config(command=tab1.progView.yview)

# Row 3: Manual Program Entry
manEntryFrame = LabelFrame(centerPanel, text="Manual Program Entry", padding=5)
manEntryFrame.grid(row=3, column=0, sticky="ew", padx=5, pady=5)

# Configure equal width columns for buttons
for i in range(5):
    manEntryFrame.grid_columnconfigure(i, weight=1)

# Entry field at top (full width)
manEntryField = Entry(manEntryFrame, width=60)
manEntryField.grid(row=0, column=0, columnspan=5, sticky="ew", padx=2, pady=(2, 5))

# Five buttons below entry field (equal width)
getSelBut = ttk.Button(manEntryFrame, text="Copy", command=getSel)
getSelBut.grid(row=1, column=0, sticky="ew", padx=2, pady=2)

insertBut = ttk.Button(manEntryFrame, text="Paste", command=manInsItem)
insertBut.grid(row=1, column=1, sticky="ew", padx=2, pady=2)

replaceBut = ttk.Button(manEntryFrame, text="Replace", command=manReplItem)
replaceBut.grid(row=1, column=2, sticky="ew", padx=2, pady=2)

openTextBut = ttk.Button(manEntryFrame, text="Open .txt File", command=openText)
openTextBut.grid(row=1, column=3, sticky="ew", padx=2, pady=2)

reloadBut = ttk.Button(manEntryFrame, text="Reload", command=reloadProg)
reloadBut.grid(row=1, column=4, sticky="ew", padx=2, pady=2)

# Row 3: Position controls
# Position Commands moved to left panel

# Wait Time moved to left panel Wait container

# Row 5: Tab and Servo controls
# Duplicate buttons removed - now in proper containers

# ============================================================================
# RIGHT PANEL - Joint and Cartesian Controls
# ============================================================================
rightPanel = Frame(tab1)
rightPanel.grid(row=0, column=2, sticky="nsew", padx=2, pady=2)

# Configure right panel grid
rightPanel.grid_rowconfigure(0, weight=0)  # Joint controls - fixed height
rightPanel.grid_rowconfigure(1, weight=0)  # Cartesian controls - fixed height
rightPanel.grid_rowconfigure(2, weight=0)  # Tool controls - fixed height
rightPanel.grid_rowconfigure(3, weight=0)  # Command builders - fixed height
rightPanel.grid_rowconfigure(4, weight=0)  # Navigation - fixed height
rightPanel.grid_rowconfigure(5, weight=0)  # Register commands - fixed height
rightPanel.grid_rowconfigure(6, weight=0)  # Device commands - fixed height
rightPanel.grid_rowconfigure(7, weight=1)  # Additional axes - compresses first when height reduced
rightPanel.grid_rowconfigure(8, weight=2)  # Spacer - expands most
rightPanel.grid_columnconfigure(0, weight=1)
rightPanel.grid_columnconfigure(1, weight=1)

# Joint controls container (J1-J6)
jointFrame = LabelFrame(
    rightPanel,
    text="Joint Control (J1-J6) - type target, press Enter",
    padding=5,
)
jointFrame.grid(row=0, column=0, columnspan=2, sticky="ew", padx=5, pady=5)

jointFrame.grid_columnconfigure(0, weight=1)
jointFrame.grid_columnconfigure(1, weight=1)

def create_joint_jog_frame(parent, row, col, joint_name, joint_num):
    frame = Frame(parent)
    frame.grid(row=row, column=col, sticky="ew", padx=2, pady=2)
    
    frame.grid_columnconfigure(0, weight=0)
    frame.grid_columnconfigure(1, weight=0)
    frame.grid_columnconfigure(2, weight=0)
    frame.grid_columnconfigure(3, weight=1)
    frame.grid_columnconfigure(4, weight=0)
    
    lab = Label(frame, font=("Arial", 14), text=joint_name)
    lab.grid(row=0, column=0, padx=2)
    
    entry = Entry(frame, width=8, justify="center")
    entry.grid(row=0, column=1, padx=2)
    _bind_joint_target_entry(joint_num - 1, entry)
    
    neg_but = Button(frame, text="-", width=3)
    neg_but.grid(row=0, column=2, padx=2)
    
    slider = Scale(frame, from_=-170, to=170, orient=HORIZONTAL, length=150)
    slider.grid(row=0, column=3, sticky="ew", padx=2)
    
    pos_but = Button(frame, text="+", width=3)
    pos_but.grid(row=0, column=4, padx=2)
    
    neg_lim_lab = Label(frame, font=("Arial", 8), text="-170", style="Jointlim.TLabel")
    neg_lim_lab.grid(row=1, column=2, sticky="w")
    
    pos_lim_lab = Label(frame, font=("Arial", 8), text="170", style="Jointlim.TLabel")
    pos_lim_lab.grid(row=1, column=4, sticky="e")
    
    slide_label = Label(frame, font=("Arial", 8))
    slide_label.grid(row=1, column=3)
    
    return frame, entry, neg_but, pos_but, slider, slide_label, neg_lim_lab, pos_lim_lab

# Create J1-J6 frames (2 columns x 3 rows)
##J1
J1jogFrame, J1curAngEntryField, J1jogNegBut, J1jogPosBut, J1jogslide, J1slidelabel, J1negLimLab, J1posLimLab = create_joint_jog_frame(jointFrame, 0, 0, "J1", 1)

# Bind J1 button events
def SelJ1jogNeg(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    J1jogNeg(float(incrementEntryField.get()))
  else:
    LiveJointJog(10)  
J1jogNegBut.bind("<ButtonPress>", SelJ1jogNeg)
J1jogNegBut.bind("<ButtonRelease>", StopJog)

def SelJ1jogPos(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    J1jogPos(float(incrementEntryField.get()))
  else:
    LiveJointJog(11)  
J1jogPosBut.bind("<ButtonPress>", SelJ1jogPos)
J1jogPosBut.bind("<ButtonRelease>", StopJog)

def J1sliderUpdate(foo):
  J1slidelabel.config(text=round(float(J1jogslide.get()),2))   
def J1sliderExecute(foo): 
  _queue_joint_target(0, J1jogslide.get())
J1jogslide.config(command=J1sliderUpdate)
J1jogslide.bind("<ButtonRelease-1>", J1sliderExecute)

##J2
J2jogFrame, J2curAngEntryField, J2jogNegBut, J2jogPosBut, J2jogslide, J2slidelabel, J2negLimLab, J2posLimLab = create_joint_jog_frame(jointFrame, 1, 0, "J2", 2)

def SelJ2jogNeg(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    J2jogNeg(float(incrementEntryField.get()))
  else:
    LiveJointJog(20)  
J2jogNegBut.bind("<ButtonPress>", SelJ2jogNeg)
J2jogNegBut.bind("<ButtonRelease>", StopJog)

def SelJ2jogPos(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    J2jogPos(float(incrementEntryField.get()))
  else:
    LiveJointJog(21)  
J2jogPosBut.bind("<ButtonPress>", SelJ2jogPos)
J2jogPosBut.bind("<ButtonRelease>", StopJog)

def J2sliderUpdate(foo):
  J2slidelabel.config(text=round(float(J2jogslide.get()),2))   
def J2sliderExecute(foo): 
  _queue_joint_target(1, J2jogslide.get())
J2jogslide.config(command=J2sliderUpdate)
J2jogslide.bind("<ButtonRelease-1>", J2sliderExecute)

##J3
J3jogFrame, J3curAngEntryField, J3jogNegBut, J3jogPosBut, J3jogslide, J3slidelabel, J3negLimLab, J3posLimLab = create_joint_jog_frame(jointFrame, 2, 0, "J3", 3)

def SelJ3jogNeg(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    J3jogNeg(float(incrementEntryField.get()))
  else:
    LiveJointJog(30)  
J3jogNegBut.bind("<ButtonPress>", SelJ3jogNeg)
J3jogNegBut.bind("<ButtonRelease>", StopJog)

def SelJ3jogPos(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    J3jogPos(float(incrementEntryField.get()))
  else:
    LiveJointJog(31)  
J3jogPosBut.bind("<ButtonPress>", SelJ3jogPos)
J3jogPosBut.bind("<ButtonRelease>", StopJog)

def J3sliderUpdate(foo):
  J3slidelabel.config(text=round(float(J3jogslide.get()),2))   
def J3sliderExecute(foo): 
  _queue_joint_target(2, J3jogslide.get())
J3jogslide.config(command=J3sliderUpdate)
J3jogslide.bind("<ButtonRelease-1>", J3sliderExecute)

##J4
J4jogFrame, J4curAngEntryField, J4jogNegBut, J4jogPosBut, J4jogslide, J4slidelabel, J4negLimLab, J4posLimLab = create_joint_jog_frame(jointFrame, 0, 1, "J4", 4)

def SelJ4jogNeg(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    J4jogNeg(float(incrementEntryField.get()))
  else:
    LiveJointJog(40)  
J4jogNegBut.bind("<ButtonPress>", SelJ4jogNeg)
J4jogNegBut.bind("<ButtonRelease>", StopJog)

def SelJ4jogPos(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    J4jogPos(float(incrementEntryField.get()))
  else:
    LiveJointJog(41)  
J4jogPosBut.bind("<ButtonPress>", SelJ4jogPos)
J4jogPosBut.bind("<ButtonRelease>", StopJog)

def J4sliderUpdate(foo):
  J4slidelabel.config(text=round(float(J4jogslide.get()),2))   
def J4sliderExecute(foo): 
  _queue_joint_target(3, J4jogslide.get())
J4jogslide.config(command=J4sliderUpdate)
J4jogslide.bind("<ButtonRelease-1>", J4sliderExecute)

##J5
J5jogFrame, J5curAngEntryField, J5jogNegBut, J5jogPosBut, J5jogslide, J5slidelabel, J5negLimLab, J5posLimLab = create_joint_jog_frame(jointFrame, 1, 1, "J5", 5)

def SelJ5jogNeg(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    J5jogNeg(float(incrementEntryField.get()))
  else:
    LiveJointJog(50)  
J5jogNegBut.bind("<ButtonPress>", SelJ5jogNeg)
J5jogNegBut.bind("<ButtonRelease>", StopJog)

def SelJ5jogPos(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    J5jogPos(float(incrementEntryField.get()))
  else:
    LiveJointJog(51)  
J5jogPosBut.bind("<ButtonPress>", SelJ5jogPos)
J5jogPosBut.bind("<ButtonRelease>", StopJog)

def J5sliderUpdate(foo):
  J5slidelabel.config(text=round(float(J5jogslide.get()),2))   
def J5sliderExecute(foo): 
  _queue_joint_target(4, J5jogslide.get())
J5jogslide.config(command=J5sliderUpdate)
J5jogslide.bind("<ButtonRelease-1>", J5sliderExecute)

##J6
J6jogFrame, J6curAngEntryField, J6jogNegBut, J6jogPosBut, J6jogslide, J6slidelabel, J6negLimLab, J6posLimLab = create_joint_jog_frame(jointFrame, 2, 1, "J6", 6)

def SelJ6jogNeg(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    J6jogNeg(float(incrementEntryField.get()))
  else:
    LiveJointJog(60)  
J6jogNegBut.bind("<ButtonPress>", SelJ6jogNeg)
J6jogNegBut.bind("<ButtonRelease>", StopJog)

def SelJ6jogPos(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    J6jogPos(float(incrementEntryField.get()))
  else:
    LiveJointJog(61)  
J6jogPosBut.bind("<ButtonPress>", SelJ6jogPos)
J6jogPosBut.bind("<ButtonRelease>", StopJog)

def J6sliderUpdate(foo):
  J6slidelabel.config(text=round(float(J6jogslide.get()),2))   
def J6sliderExecute(foo): 
  _queue_joint_target(5, J6jogslide.get())
J6jogslide.config(command=J6sliderUpdate)
J6jogslide.bind("<ButtonRelease-1>", J6sliderExecute)

# Cartesian jog controls
CartjogFrame = LabelFrame(rightPanel, text="Cartesian Control (X Y Z Rz Ry Rx)", padding=5)
CartjogFrame.grid(row=1, column=0, columnspan=2, sticky="ew", padx=5, pady=5)

CartjogFrame.grid_columnconfigure(0, weight=1)
CartjogFrame.grid_columnconfigure(1, weight=1)
CartjogFrame.grid_columnconfigure(2, weight=1)
CartjogFrame.grid_columnconfigure(3, weight=1)
CartjogFrame.grid_columnconfigure(4, weight=1)
CartjogFrame.grid_columnconfigure(5, weight=1)


def create_cart_control(parent, row, col, label_text):
    Label(parent, font=("Arial", 14), text=label_text).grid(row=row, column=col, pady=2)
    
    entry = Entry(parent, width=6, justify="center")
    entry.grid(row=row+1, column=col, pady=2)
    
    # Create frame for horizontal button layout
    button_frame = Frame(parent)
    button_frame.grid(row=row+2, column=col, pady=2)
    
    neg_but = Button(button_frame, text="-", width=3)
    neg_but.grid(row=0, column=0, padx=1)
    
    pos_but = Button(button_frame, text="+", width=3)
    pos_but.grid(row=0, column=1, padx=1)
    
    return entry, neg_but, pos_but

# Create cartesian controls
XcurEntryField, XjogNegBut, XjogPosBut = create_cart_control(CartjogFrame, 0, 0, "X")
YcurEntryField, YjogNegBut, YjogPosBut = create_cart_control(CartjogFrame, 0, 1, "Y")
ZcurEntryField, ZjogNegBut, ZjogPosBut = create_cart_control(CartjogFrame, 0, 2, "Z")
RzcurEntryField, RzjogNegBut, RzjogPosBut = create_cart_control(CartjogFrame, 0, 3, "Rz")
RycurEntryField, RyjogNegBut, RyjogPosBut = create_cart_control(CartjogFrame, 0, 4, "Ry")
RxcurEntryField, RxjogNegBut, RxjogPosBut = create_cart_control(CartjogFrame, 0, 5, "Rx")

# Bind cartesian button events
def SelXjogNeg(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    XjogNeg(float(incrementEntryField.get()))
  else:
    LiveCarJog(10)  
XjogNegBut.bind("<ButtonPress>", SelXjogNeg)
XjogNegBut.bind("<ButtonRelease>", StopJog)

def SelXjogPos(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    XjogPos(float(incrementEntryField.get()))
  else:
    LiveCarJog(11)  
XjogPosBut.bind("<ButtonPress>", SelXjogPos)
XjogPosBut.bind("<ButtonRelease>", StopJog)

def SelYjogNeg(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    YjogNeg(float(incrementEntryField.get()))
  else:
    LiveCarJog(20)  
YjogNegBut.bind("<ButtonPress>", SelYjogNeg)
YjogNegBut.bind("<ButtonRelease>", StopJog)

def SelYjogPos(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    YjogPos(float(incrementEntryField.get()))
  else:
    LiveCarJog(21)  
YjogPosBut.bind("<ButtonPress>", SelYjogPos)
YjogPosBut.bind("<ButtonRelease>", StopJog)

def SelZjogNeg(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    ZjogNeg(float(incrementEntryField.get()))
  else:
    LiveCarJog(30)  
ZjogNegBut.bind("<ButtonPress>", SelZjogNeg)
ZjogNegBut.bind("<ButtonRelease>", StopJog)

def SelZjogPos(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    ZjogPos(float(incrementEntryField.get()))
  else:
    LiveCarJog(31)  
ZjogPosBut.bind("<ButtonPress>", SelZjogPos)
ZjogPosBut.bind("<ButtonRelease>", StopJog)

def SelRzjogNeg(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    RzjogNeg(float(incrementEntryField.get()))
  else:
    LiveCarJog(40)  
RzjogNegBut.bind("<ButtonPress>", SelRzjogNeg)
RzjogNegBut.bind("<ButtonRelease>", StopJog)

def SelRzjogPos(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    RzjogPos(float(incrementEntryField.get()))
  else:
    LiveCarJog(41)  
RzjogPosBut.bind("<ButtonPress>", SelRzjogPos)
RzjogPosBut.bind("<ButtonRelease>", StopJog)

def SelRyjogNeg(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    RyjogNeg(float(incrementEntryField.get()))
  else:
    LiveCarJog(50)  
RyjogNegBut.bind("<ButtonPress>", SelRyjogNeg)
RyjogNegBut.bind("<ButtonRelease>", StopJog)

def SelRyjogPos(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    RyjogPos(float(incrementEntryField.get()))
  else:
    LiveCarJog(51)  
RyjogPosBut.bind("<ButtonPress>", SelRyjogPos)
RyjogPosBut.bind("<ButtonRelease>", StopJog)

def SelRxjogNeg(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    RxjogNeg(float(incrementEntryField.get()))
  else:
    LiveCarJog(60)  
RxjogNegBut.bind("<ButtonPress>", SelRxjogNeg)
RxjogNegBut.bind("<ButtonRelease>", StopJog)

def SelRxjogPos(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    RxjogPos(float(incrementEntryField.get()))
  else:
    LiveCarJog(61)  
RxjogPosBut.bind("<ButtonPress>", SelRxjogPos)
RxjogPosBut.bind("<ButtonRelease>", StopJog)

# Tool frame controls
TooljogFrame = LabelFrame(rightPanel, text="Tool Frame Control (Tx Ty Tz Trz Try Trx)", padding=5)
TooljogFrame.grid(row=2, column=0, columnspan=2, sticky="ew", padx=5, pady=5)

TooljogFrame.grid_columnconfigure(0, weight=1)
TooljogFrame.grid_columnconfigure(1, weight=1)
TooljogFrame.grid_columnconfigure(2, weight=1)
TooljogFrame.grid_columnconfigure(3, weight=1)
TooljogFrame.grid_columnconfigure(4, weight=1)
TooljogFrame.grid_columnconfigure(5, weight=1)

def create_tool_control(parent, col, label_text):
    Label(parent, font=("Arial", 14), text=label_text).grid(row=0, column=col, pady=2)
    
    # Create frame for horizontal button layout
    button_frame = Frame(parent)
    button_frame.grid(row=1, column=col, pady=2)
    
    neg_but = Button(button_frame, text="-", width=3)
    neg_but.grid(row=0, column=0, padx=1)
    
    pos_but = Button(button_frame, text="+", width=3)
    pos_but.grid(row=0, column=1, padx=1)
    
    return neg_but, pos_but

# Create tool frame controls (no entry fields)
TXjogNegBut, TXjogPosBut = create_tool_control(TooljogFrame, 0, "Tx")
TYjogNegBut, TYjogPosBut = create_tool_control(TooljogFrame, 1, "Ty")
TZjogNegBut, TZjogPosBut = create_tool_control(TooljogFrame, 2, "Tz")
TRzjogNegBut, TRzjogPosBut = create_tool_control(TooljogFrame, 3, "Trz")
TRyjogNegBut, TRyjogPosBut = create_tool_control(TooljogFrame, 4, "Try")
TRxjogNegBut, TRxjogPosBut = create_tool_control(TooljogFrame, 5, "Trx")

# Bind tool frame button events
def SelTXjogNeg(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    TXjogNeg(float(incrementEntryField.get()))
  else:
    LiveToolJog(10)
TXjogNegBut.bind("<ButtonPress>", SelTXjogNeg)
TXjogNegBut.bind("<ButtonRelease>", StopJog)

def SelTXjogPos(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    TXjogPos(float(incrementEntryField.get()))
  else:
    LiveToolJog(11)
TXjogPosBut.bind("<ButtonPress>", SelTXjogPos)
TXjogPosBut.bind("<ButtonRelease>", StopJog)

def SelTYjogNeg(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    TYjogNeg(float(incrementEntryField.get()))
  else:
    LiveToolJog(20)
TYjogNegBut.bind("<ButtonPress>", SelTYjogNeg)
TYjogNegBut.bind("<ButtonRelease>", StopJog)

def SelTYjogPos(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    TYjogPos(float(incrementEntryField.get()))
  else:
    LiveToolJog(21)
TYjogPosBut.bind("<ButtonPress>", SelTYjogPos)
TYjogPosBut.bind("<ButtonRelease>", StopJog)

def SelTZjogNeg(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    TZjogNeg(float(incrementEntryField.get()))
  else:
    LiveToolJog(30)
TZjogNegBut.bind("<ButtonPress>", SelTZjogNeg)
TZjogNegBut.bind("<ButtonRelease>", StopJog)

def SelTZjogPos(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    TZjogPos(float(incrementEntryField.get()))
  else:
    LiveToolJog(31)
TZjogPosBut.bind("<ButtonPress>", SelTZjogPos)
TZjogPosBut.bind("<ButtonRelease>", StopJog)

def SelTRzjogNeg(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    TRzjogNeg(float(incrementEntryField.get()))
  else:
    LiveToolJog(40)
TRzjogNegBut.bind("<ButtonPress>", SelTRzjogNeg)
TRzjogNegBut.bind("<ButtonRelease>", StopJog)

def SelTRzjogPos(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    TRzjogPos(float(incrementEntryField.get()))
  else:
    LiveToolJog(41)
TRzjogPosBut.bind("<ButtonPress>", SelTRzjogPos)
TRzjogPosBut.bind("<ButtonRelease>", StopJog)

def SelTRyjogNeg(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    TRyjogNeg(float(incrementEntryField.get()))
  else:
    LiveToolJog(50)
TRyjogNegBut.bind("<ButtonPress>", SelTRyjogNeg)
TRyjogNegBut.bind("<ButtonRelease>", StopJog)

def SelTRyjogPos(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    TRyjogPos(float(incrementEntryField.get()))
  else:
    LiveToolJog(51)
TRyjogPosBut.bind("<ButtonPress>", SelTRyjogPos)
TRyjogPosBut.bind("<ButtonRelease>", StopJog)

def SelTRxjogNeg(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    TRxjogNeg(float(incrementEntryField.get()))
  else:
    LiveToolJog(60)
TRxjogNegBut.bind("<ButtonPress>", SelTRxjogNeg)
TRxjogNegBut.bind("<ButtonRelease>", StopJog)

def SelTRxjogPos(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    TRxjogPos(float(incrementEntryField.get()))
  else:
    LiveToolJog(61)
TRxjogPosBut.bind("<ButtonPress>", SelTRxjogPos)
TRxjogPosBut.bind("<ButtonRelease>", StopJog)


# Extra axes (J7, J8, J9)
extraAxesFrame = LabelFrame(rightPanel, text="Additional Axes", padding=5)
extraAxesFrame.grid(row=7, column=0, columnspan=2, sticky="ew", padx=5, pady=5)

extraAxesFrame.grid_columnconfigure(0, weight=1)
extraAxesFrame.grid_columnconfigure(1, weight=1)
extraAxesFrame.grid_columnconfigure(2, weight=1)

# J7 Frame
J7jogFrame = Frame(extraAxesFrame, relief="raised", borderwidth=1, padding=5)
J7jogFrame.grid(row=0, column=0, sticky="nsew", padx=2, pady=2)

J7jogFrame.grid_columnconfigure(0, weight=1)
J7jogFrame.grid_columnconfigure(1, weight=0)
J7jogFrame.grid_columnconfigure(2, weight=1)

J7Lab = Label(J7jogFrame, font=("Arial", 12), text="7th Axis")
J7Lab.grid(row=0, column=0, columnspan=3, pady=2)

J7negLimLab = Label(J7jogFrame, font=("Arial", 8), text="0.00", style="Jointlim.TLabel")
J7negLimLab.grid(row=1, column=0, sticky="w", padx=2)
J7curAngEntryField = Entry(J7jogFrame, width=8, justify="center")
J7curAngEntryField.grid(row=1, column=1, padx=5)
J7posLimLab = Label(J7jogFrame, font=("Arial", 8), text="0", style="Jointlim.TLabel")
J7posLimLab.grid(row=1, column=2, sticky="e", padx=2)

J7jogslide = Scale(J7jogFrame, from_=0, to=0, orient=HORIZONTAL, length=120)
J7jogslide.grid(row=2, column=0, columnspan=3, sticky="ew", padx=2, pady=2)

# Create frame for button layout (- entry +)
J7buttonFrame = Frame(J7jogFrame)
J7buttonFrame.grid(row=3, column=0, columnspan=3, sticky="ew", padx=2, pady=2)

J7buttonFrame.grid_columnconfigure(0, weight=1)
J7buttonFrame.grid_columnconfigure(1, weight=0)
J7buttonFrame.grid_columnconfigure(2, weight=1)

J7jogNegBut = Button(J7buttonFrame, text="-", width=3)
J7jogNegBut.grid(row=0, column=0, sticky="w", padx=(0, 2))

J7slideLimLab = Entry(J7buttonFrame, width=8, justify="center", state="readonly")
J7slideLimLab.grid(row=0, column=1, padx=2)

J7jogPosBut = Button(J7buttonFrame, text="+", width=3)
J7jogPosBut.grid(row=0, column=2, sticky="e", padx=(2, 0))

# Bind J7 events
def SelJ7jogNeg(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    J7jogNeg(float(incrementEntryField.get()))
  else:
    LiveJointJog(70) 
J7jogNegBut.bind("<ButtonPress>", SelJ7jogNeg)
J7jogNegBut.bind("<ButtonRelease>", StopJog)

def SelJ7jogPos(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    J7jogPos(float(incrementEntryField.get()))
  else:
    LiveJointJog(71)  
J7jogPosBut.bind("<ButtonPress>", SelJ7jogPos)
J7jogPosBut.bind("<ButtonRelease>", StopJog)

def J7sliderUpdate(foo):
  J7slideLimLab.config(state="normal")
  J7slideLimLab.delete(0, END)
  J7slideLimLab.insert(0, round(float(J7jogslide.get()),2))
  J7slideLimLab.config(state="readonly")   
def J7sliderExecute(foo): 
  _queue_joint_target(6, J7jogslide.get())
J7jogslide.config(command=J7sliderUpdate)
J7jogslide.bind("<ButtonRelease-1>", J7sliderExecute)

# J8 Frame
J8jogFrame = Frame(extraAxesFrame, relief="raised", borderwidth=1, padding=5)
J8jogFrame.grid(row=0, column=1, sticky="nsew", padx=2, pady=2)

J8jogFrame.grid_columnconfigure(0, weight=1)
J8jogFrame.grid_columnconfigure(1, weight=0)
J8jogFrame.grid_columnconfigure(2, weight=1)

J8Lab = Label(J8jogFrame, font=("Arial", 12), text="8th Axis")
J8Lab.grid(row=0, column=0, columnspan=3, pady=2)

J8negLimLab = Label(J8jogFrame, font=("Arial", 8), text="0.00", style="Jointlim.TLabel")
J8negLimLab.grid(row=1, column=0, sticky="w", padx=2)
J8curAngEntryField = Entry(J8jogFrame, width=8, justify="center")
J8curAngEntryField.grid(row=1, column=1, padx=5)
J8posLimLab = Label(J8jogFrame, font=("Arial", 8), text="0", style="Jointlim.TLabel")
J8posLimLab.grid(row=1, column=2, sticky="e", padx=2)

J8jogslide = Scale(J8jogFrame, from_=0, to=0, orient=HORIZONTAL, length=120)
J8jogslide.grid(row=2, column=0, columnspan=3, sticky="ew", padx=2, pady=2)

# Create frame for button layout (- entry +)
J8buttonFrame = Frame(J8jogFrame)
J8buttonFrame.grid(row=3, column=0, columnspan=3, sticky="ew", padx=2, pady=2)

J8buttonFrame.grid_columnconfigure(0, weight=1)
J8buttonFrame.grid_columnconfigure(1, weight=0)
J8buttonFrame.grid_columnconfigure(2, weight=1)

J8jogNegBut = Button(J8buttonFrame, text="-", width=3)
J8jogNegBut.grid(row=0, column=0, sticky="w", padx=(0, 2))

J8slideLimLab = Entry(J8buttonFrame, width=8, justify="center", state="readonly")
J8slideLimLab.grid(row=0, column=1, padx=2)

J8jogPosBut = Button(J8buttonFrame, text="+", width=3)
J8jogPosBut.grid(row=0, column=2, sticky="e", padx=(2, 0))

# Bind J8 events
def SelJ8jogNeg(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    J8jogNeg(float(incrementEntryField.get()))
  else:
    LiveJointJog(80) 
J8jogNegBut.bind("<ButtonPress>", SelJ8jogNeg)
J8jogNegBut.bind("<ButtonRelease>", StopJog)

def SelJ8jogPos(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    J8jogPos(float(incrementEntryField.get()))
  else:
    LiveJointJog(81)  
J8jogPosBut.bind("<ButtonPress>", SelJ8jogPos)
J8jogPosBut.bind("<ButtonRelease>", StopJog)

def J8sliderUpdate(foo):
  J8slideLimLab.config(state="normal")
  J8slideLimLab.delete(0, END)
  J8slideLimLab.insert(0, round(float(J8jogslide.get()),2))
  J8slideLimLab.config(state="readonly")   
def J8sliderExecute(foo): 
  _queue_joint_target(7, J8jogslide.get())
J8jogslide.config(command=J8sliderUpdate)
J8jogslide.bind("<ButtonRelease-1>", J8sliderExecute)

# J9 Frame
J9jogFrame = Frame(extraAxesFrame, relief="raised", borderwidth=1, padding=5)
J9jogFrame.grid(row=0, column=2, sticky="nsew", padx=2, pady=2)

J9jogFrame.grid_columnconfigure(0, weight=1)
J9jogFrame.grid_columnconfigure(1, weight=0)
J9jogFrame.grid_columnconfigure(2, weight=1)

J9Lab = Label(J9jogFrame, font=("Arial", 12), text="9th Axis")
J9Lab.grid(row=0, column=0, columnspan=3, pady=2)

J9negLimLab = Label(J9jogFrame, font=("Arial", 8), text="0.00", style="Jointlim.TLabel")
J9negLimLab.grid(row=1, column=0, sticky="w", padx=2)
J9curAngEntryField = Entry(J9jogFrame, width=8, justify="center")
J9curAngEntryField.grid(row=1, column=1, padx=5)
J9posLimLab = Label(J9jogFrame, font=("Arial", 8), text="0", style="Jointlim.TLabel")
J9posLimLab.grid(row=1, column=2, sticky="e", padx=2)

J9jogslide = Scale(J9jogFrame, from_=0, to=0, orient=HORIZONTAL, length=120)
J9jogslide.grid(row=2, column=0, columnspan=3, sticky="ew", padx=2, pady=2)

# Create frame for button layout (- entry +)
J9buttonFrame = Frame(J9jogFrame)
J9buttonFrame.grid(row=3, column=0, columnspan=3, sticky="ew", padx=2, pady=2)

J9buttonFrame.grid_columnconfigure(0, weight=1)
J9buttonFrame.grid_columnconfigure(1, weight=0)
J9buttonFrame.grid_columnconfigure(2, weight=1)

J9jogNegBut = Button(J9buttonFrame, text="-", width=3)
J9jogNegBut.grid(row=0, column=0, sticky="w", padx=(0, 2))

J9slideLimLab = Entry(J9buttonFrame, width=8, justify="center", state="readonly")
J9slideLimLab.grid(row=0, column=1, padx=2)

J9jogPosBut = Button(J9buttonFrame, text="+", width=3)
J9jogPosBut.grid(row=0, column=2, sticky="e", padx=(2, 0))

# Bind J9 events
def SelJ9jogNeg(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    J9jogNeg(float(incrementEntryField.get()))
  else:
    LiveJointJog(90) 
J9jogNegBut.bind("<ButtonPress>", SelJ9jogNeg)
J9jogNegBut.bind("<ButtonRelease>", StopJog)

def SelJ9jogPos(self):
  IncJogStatVal = int(RUN['IncJogStat'].get())
  if (IncJogStatVal == 1):
    J9jogPos(float(incrementEntryField.get()))
  else:
    LiveJointJog(91)  
J9jogPosBut.bind("<ButtonPress>", SelJ9jogPos)
J9jogPosBut.bind("<ButtonRelease>", StopJog)

def J9sliderUpdate(foo):
  J9slideLimLab.config(state="normal")
  J9slideLimLab.delete(0, END)
  J9slideLimLab.insert(0, round(float(J9jogslide.get()),2))
  J9slideLimLab.config(state="readonly")   
def J9sliderExecute(foo): 
  _queue_joint_target(8, J9jogslide.get())
J9jogslide.config(command=J9sliderUpdate)
J9jogslide.bind("<ButtonRelease-1>", J9sliderExecute)

# Command builders (IF, SET, WAIT - reordered and aligned)
cmdFrame = LabelFrame(rightPanel, text="Command Builders", padding=5)
cmdFrame.grid(row=3, column=0, columnspan=2, sticky="ew", padx=5, pady=(5, 2))

# Configure columns for proper alignment
cmdFrame.grid_columnconfigure(0, weight=0, minsize=45)   # Label
cmdFrame.grid_columnconfigure(1, weight=0, minsize=130)  # Type dropdown
cmdFrame.grid_columnconfigure(2, weight=0, minsize=60)   # Var entry
cmdFrame.grid_columnconfigure(3, weight=0, minsize=25)   # "="
cmdFrame.grid_columnconfigure(4, weight=0, minsize=60)   # Value entry
cmdFrame.grid_columnconfigure(5, weight=0, minsize=100)  # Action dropdown (IF only)
cmdFrame.grid_columnconfigure(6, weight=0, minsize=80)   # Dest entry / Timeout entry
cmdFrame.grid_columnconfigure(7, weight=0, minsize=25)   # "•"
cmdFrame.grid_columnconfigure(8, weight=1, minsize=120)  # Insert button

# Create StringVars for OptionMenus
iFoption = StringVar(cmdFrame)
iFoption.set("5v Input")
iFselection = StringVar(cmdFrame)
iFselection.set("Call Prog")
waitoption = StringVar(cmdFrame)
waitoption.set("5v Input")
setoption = StringVar(cmdFrame)
setoption.set("5v Output")

# Row 0: IF command - IF [Type] [Var#] = [Value] [Action] [Dest] • [Insert]
Label(cmdFrame, text="IF", font=("Arial", 10, "bold")).grid(row=0, column=0, sticky="w", padx=(2, 5), pady=2)

iFmenu = OptionMenu(cmdFrame, iFoption, "5v Input", "5v Input", "Register", "COM Device", "MB Coil", "MB Input", "MB Hold Reg", "MB Input Reg")
iFmenu.config(width=12)
iFmenu.grid(row=0, column=1, sticky="ew", padx=2, pady=2)

IfVarEntryField = Entry(cmdFrame, width=6, justify="center")
IfVarEntryField.grid(row=0, column=2, sticky="ew", padx=2, pady=2)

Label(cmdFrame, text="=").grid(row=0, column=3, padx=2)

IfInputEntryField = Entry(cmdFrame, width=6, justify="center")
IfInputEntryField.grid(row=0, column=4, sticky="ew", padx=2, pady=2)

iFSelmenu = OptionMenu(cmdFrame, iFselection, "Call Prog", "Call Prog", "Jump Tab", "Stop")
iFSelmenu.config(width=9)
iFSelmenu.grid(row=0, column=5, sticky="ew", padx=2, pady=2)

IfDestEntryField = Entry(cmdFrame, width=8, justify="center")
IfDestEntryField.grid(row=0, column=6, sticky="ew", padx=2, pady=2)

Label(cmdFrame, text="•").grid(row=0, column=7, padx=2)

insertIFBut = ttk.Button(cmdFrame, text="Insert IF CMD", command=IfCMDInsert)
insertIFBut.grid(row=0, column=8, sticky="ew", padx=2, pady=2)

# Row 1: SET command - SET [Type] [Var#] = [Value] • [Insert]
Label(cmdFrame, text="SET", font=("Arial", 10, "bold")).grid(row=1, column=0, sticky="w", padx=(2, 5), pady=2)

setmenu = OptionMenu(cmdFrame, setoption, "5v Output", "5v Output", "MB Coil", "MB Register")
setmenu.config(width=12)
setmenu.grid(row=1, column=1, sticky="ew", padx=2, pady=2)

setVarEntryField = Entry(cmdFrame, width=6, justify="center")
setVarEntryField.grid(row=1, column=2, sticky="ew", padx=2, pady=2)

Label(cmdFrame, text="=").grid(row=1, column=3, padx=2)

setInputEntryField = Entry(cmdFrame, width=6, justify="center")
setInputEntryField.grid(row=1, column=4, sticky="ew", padx=2, pady=2)

Label(cmdFrame, text="•").grid(row=1, column=7, padx=2)

insertSetBut = ttk.Button(cmdFrame, text="Insert set CMD", command=SetCMDInsert)
insertSetBut.grid(row=1, column=8, sticky="ew", padx=2, pady=2)

# Row 2: WAIT command - WAIT [Type] [Var#] = [Value] Timeout = [Time] • [Insert]
Label(cmdFrame, text="WAIT", font=("Arial", 10, "bold")).grid(row=2, column=0, sticky="w", padx=(2, 5), pady=2)

waitmenu = OptionMenu(cmdFrame, waitoption, "5v Input", "5v Input", "MB Coil", "MB Input")
waitmenu.config(width=12)
waitmenu.grid(row=2, column=1, sticky="ew", padx=2, pady=2)

waitVarEntryField = Entry(cmdFrame, width=6, justify="center")
waitVarEntryField.grid(row=2, column=2, sticky="ew", padx=2, pady=2)

Label(cmdFrame, text="=").grid(row=2, column=3, padx=2)

waitInputEntryField = Entry(cmdFrame, width=6, justify="center")
waitInputEntryField.grid(row=2, column=4, sticky="ew", padx=2, pady=2)

Label(cmdFrame, text="Timeout =").grid(row=2, column=5, sticky="e", padx=2)

waitTimeoutEntryField = Entry(cmdFrame, width=6, justify="center")
waitTimeoutEntryField.grid(row=2, column=6, sticky="ew", padx=2, pady=2)

Label(cmdFrame, text="•").grid(row=2, column=7, padx=2)

insertWaitBut = ttk.Button(cmdFrame, text="Insert WAIT CMD", command=WaitCMDInsert)
insertWaitBut.grid(row=2, column=8, sticky="ew", padx=2, pady=2)

# Navigation container (2x2 grid layout)
navFrame = LabelFrame(rightPanel, text="Navigation", padding=5)
navFrame.grid(row=4, column=0, columnspan=2, sticky="ew", padx=5, pady=(2, 5))

# Configure 4 columns for 2x2 grid (button, entry, button, entry)
navFrame.grid_columnconfigure(0, weight=1, minsize=100)  # Button 1
navFrame.grid_columnconfigure(1, weight=1, minsize=80)   # Entry 1
navFrame.grid_columnconfigure(2, weight=1, minsize=100)  # Button 2
navFrame.grid_columnconfigure(3, weight=1, minsize=80)   # Entry 2

# Row 0: Create Tab | Call Program
createTabBut = ttk.Button(navFrame, text="Create Tab", command=tabNumber)
createTabBut.grid(row=0, column=0, sticky="ew", padx=2, pady=2)

tabNumEntryField = Entry(navFrame, width=8, justify="center")
tabNumEntryField.grid(row=0, column=1, sticky="ew", padx=2, pady=2)

callProgBut = ttk.Button(navFrame, text="Call Program", command=insertCallProg)
callProgBut.grid(row=0, column=2, sticky="ew", padx=2, pady=2)

changeProgEntryField = Entry(navFrame, width=8, justify="center")
changeProgEntryField.grid(row=0, column=3, sticky="ew", padx=2, pady=2)

# Row 1: Jump to Tab | Play Gcode
jumpTabBut = ttk.Button(navFrame, text="Jump to Tab", command=jumpTab)
jumpTabBut.grid(row=1, column=0, sticky="ew", padx=2, pady=2)

jumpTabEntryField = Entry(navFrame, width=8, justify="center")
jumpTabEntryField.grid(row=1, column=1, sticky="ew", padx=2, pady=2)

playGcodeBut = ttk.Button(navFrame, text="Play Gcode", command=insertGCprog)
playGcodeBut.grid(row=1, column=2, sticky="ew", padx=2, pady=2)

PlayGCEntryField = Entry(navFrame, width=8, justify="center")
PlayGCEntryField.grid(row=1, column=3, sticky="ew", padx=2, pady=2)

# Register Commands container
regFrame = LabelFrame(rightPanel, text="Register Commands", padding=5)
regFrame.grid(row=5, column=0, columnspan=2, sticky="ew", padx=5, pady=(2, 5))

# Configure columns for side-by-side layout
regFrame.grid_columnconfigure(0, weight=1, minsize=120)  # Register button
regFrame.grid_columnconfigure(1, weight=0, minsize=60)   # Register entry
regFrame.grid_columnconfigure(2, weight=0, minsize=60)   # (++/--) entry
regFrame.grid_columnconfigure(3, weight=1, minsize=140)  # Position Register button
regFrame.grid_columnconfigure(4, weight=0, minsize=60)   # Pos Reg entry
regFrame.grid_columnconfigure(5, weight=0, minsize=60)   # Element entry
regFrame.grid_columnconfigure(6, weight=0, minsize=60)   # (++/--) entry

# Row 0: Labels
Label(regFrame, text="Register", font=("Arial", 8)).grid(row=0, column=1, sticky="w", padx=2)
Label(regFrame, text="(++/--)", font=("Arial", 8)).grid(row=0, column=2, sticky="w", padx=2)
Label(regFrame, text="Pos Reg", font=("Arial", 8)).grid(row=0, column=4, sticky="w", padx=2)
Label(regFrame, text="Element", font=("Arial", 8)).grid(row=0, column=5, sticky="w", padx=2)
Label(regFrame, text="(++/--)", font=("Arial", 8)).grid(row=0, column=6, sticky="w", padx=2)

# Row 1: Buttons and entry fields
RegNumBut = ttk.Button(regFrame, text="Register", command=insertRegister)
RegNumBut.grid(row=1, column=0, sticky="ew", padx=2, pady=2)

regNumEntryField = Entry(regFrame, width=6, justify="center")
regNumEntryField.grid(row=1, column=1, sticky="ew", padx=2, pady=2)

regEqEntryField = Entry(regFrame, width=6, justify="center")
regEqEntryField.grid(row=1, column=2, sticky="ew", padx=2, pady=2)

StorPosBut = ttk.Button(regFrame, text="Position Register", command=storPos)
StorPosBut.grid(row=1, column=3, sticky="ew", padx=(10, 2), pady=2)

storPosNumEntryField = Entry(regFrame, width=6, justify="center")
storPosNumEntryField.grid(row=1, column=4, sticky="ew", padx=2, pady=2)

storPosElEntryField = Entry(regFrame, width=6, justify="center")
storPosElEntryField.grid(row=1, column=5, sticky="ew", padx=2, pady=2)

storPosValEntryField = Entry(regFrame, width=6, justify="center")
storPosValEntryField.grid(row=1, column=6, sticky="ew", padx=2, pady=2)

# Aliases for compatibility
posRegBut = StorPosBut
posRegEntryField = storPosNumEntryField

# Device Commands container
devFrame = LabelFrame(rightPanel, text="Device Commands", padding=5)
devFrame.grid(row=6, column=0, columnspan=2, sticky="ew", padx=5, pady=(2, 5))

# Configure columns
devFrame.grid_columnconfigure(0, weight=1, minsize=100)  # Servo button
devFrame.grid_columnconfigure(1, weight=0, minsize=60)   # Number entry
devFrame.grid_columnconfigure(2, weight=0, minsize=60)   # Position entry
devFrame.grid_columnconfigure(3, weight=1, minsize=140)  # Read COM button
devFrame.grid_columnconfigure(4, weight=0, minsize=60)   # Port entry
devFrame.grid_columnconfigure(5, weight=0, minsize=60)   # Char entry

# Row 0: Labels
Label(devFrame, text="Number", font=("Arial", 8)).grid(row=0, column=1, sticky="w", padx=2)
Label(devFrame, text="Position", font=("Arial", 8)).grid(row=0, column=2, sticky="w", padx=2)
Label(devFrame, text="Port", font=("Arial", 8)).grid(row=0, column=4, sticky="w", padx=2)
Label(devFrame, text="Char", font=("Arial", 8)).grid(row=0, column=5, sticky="w", padx=2)

# Row 1: Buttons and entry fields
servoBut = ttk.Button(devFrame, text="Servo", command=Servo)
servoBut.grid(row=1, column=0, sticky="ew", padx=2, pady=2)

servoNumEntryField = Entry(devFrame, width=6, justify="center")
servoNumEntryField.grid(row=1, column=1, sticky="ew", padx=2, pady=2)

servoPosEntryField = Entry(devFrame, width=6, justify="center")
servoPosEntryField.grid(row=1, column=2, sticky="ew", padx=2, pady=2)

readCOMBut = ttk.Button(devFrame, text="Read COM Device", command=ReadAuxCom)
readCOMBut.grid(row=1, column=3, sticky="ew", padx=(10, 2), pady=2)

auxPortEntryField = Entry(devFrame, width=6, justify="center")
auxPortEntryField.grid(row=1, column=4, sticky="ew", padx=2, pady=2)

auxCharEntryField = Entry(devFrame, width=6, justify="center")
auxCharEntryField.grid(row=1, column=5, sticky="ew", padx=2, pady=2)

##########################################################################
### END OF TAB 1 REFACTORING







####TAB 2

# Configure tab2 main grid
tab2.grid_rowconfigure(0, weight=0)  # Status bar
tab2.grid_rowconfigure(1, weight=1)  # Main content (expands)
tab2.grid_rowconfigure(2, weight=0)  # Commands and Save row

tab2.grid_columnconfigure(0, weight=0, minsize=180)  # Communication
tab2.grid_columnconfigure(1, weight=0, minsize=180)  # Robot Calibration
tab2.grid_columnconfigure(2, weight=0, minsize=150)  # Calibration Offsets
tab2.grid_columnconfigure(3, weight=0, minsize=150)  # Encoder Control
tab2.grid_columnconfigure(4, weight=0, minsize=180)  # External Axes
tab2.grid_columnconfigure(5, weight=0, minsize=150)  # Theme
tab2.grid_columnconfigure(6, weight=0, minsize=180)  # Virtual Import
tab2.grid_columnconfigure(7, weight=0, minsize=120)  # Save
tab2.grid_columnconfigure(8, weight=1)  # Spacer (expands)

# ============================================================================
# ROW 0: Status/Alarm Label (spans all columns)
# ============================================================================
almStatusLab2 = Label(tab2, text="SYSTEM STARTING - PLEASE WAIT", style="OK.TLabel")
almStatusLab2.grid(row=0, column=0, columnspan=9, sticky="w", padx=25, pady=20)

# ============================================================================
# ROW 1, COLUMN 0: Communication Frame
# ============================================================================
commFrame = LabelFrame(tab2, text="Communication", padding=10)
commFrame.grid(row=1, column=0, sticky="nsew", padx=5, pady=5)

commFrame.grid_columnconfigure(0, weight=1)

# COM port detection function (from original)
def detect_ports():
  from serial.tools import list_ports
  ports = list(list_ports.comports())
  choices = [p.device for p in ports]
  choices.insert(0, "None")
  if globals().get('CAL', {}).get('comPort') not in ("", None, "None"):
    port1_default = CAL['comPort']
  else:
    port1_default = "None"
  
  if globals().get('CAL', {}).get('com2Port') not in ("", None, "None"):
    port2_default = CAL['com2Port']
  else:
    port2_default = "None"
  
  return choices, port1_default, port2_default

port_choices, default_comport1, default_comport2 = detect_ports()
logger.debug(f"Available Comm Ports: {port_choices}")

# Teensy COM Port
ComPortLab = Label(commFrame, text="TEENSY COM PORT:")
ComPortLab.grid(row=0, column=0, sticky="w", padx=5, pady=(5, 2))

com1SelectedValue = tk.StringVar(value=default_comport1 or "None")
com1Select = tk.OptionMenu(commFrame, com1SelectedValue, *port_choices, command=setCom)
com1Select.grid(row=1, column=0, sticky="ew", padx=5, pady=2)

AuxiliaryBoardLab = Label(commFrame, text="5v IO BOARD PROFILE:")
AuxiliaryBoardLab.grid(row=2, column=0, sticky="w", padx=5, pady=(15, 2))

auxiliaryBoardSelectedValue = tk.StringVar(value=AUXILIARY_BOARD_NONE)
auxiliaryBoardSelect = tk.OptionMenu(
  commFrame,
  auxiliaryBoardSelectedValue,
  AUXILIARY_BOARD_NONE,
  AUXILIARY_BOARD_NANO,
  AUXILIARY_BOARD_MEGA,
  command=setCom2,
)
auxiliaryBoardSelect.grid(row=3, column=0, sticky="ew", padx=5, pady=2)

# 5v IO Board COM Port
ComPortLab2 = Label(commFrame, text="5v IO BOARD COM PORT:")
ComPortLab2.grid(row=4, column=0, sticky="w", padx=5, pady=(15, 2))

com2SelectedValue = tk.StringVar(value=default_comport2 or "None")
com2Select = tk.OptionMenu(commFrame, com2SelectedValue, *port_choices, command=setCom2)
com2Select.grid(row=5, column=0, sticky="ew", padx=5, pady=2)

# ============================================================================
# ROW 1, COLUMN 1: Robot Calibration Frame
# ============================================================================
calFrame = LabelFrame(tab2, text="Robot Calibration", padding=10)
calFrame.grid(row=1, column=1, sticky="nsew", padx=5, pady=5)

calFrame.grid_columnconfigure(0, weight=1)

# Auto Calibrate button
autoCalBut = Button(calFrame, text="  Auto Calibrate  ", command=startCalRobotAll)
autoCalBut.grid(row=0, column=0, sticky="ew", padx=5, pady=5)

# First set of checkboxes (J1-J9)
checkFrame1 = Frame(calFrame)
checkFrame1.grid(row=1, column=0, sticky="ew", padx=5, pady=5)

checkFrame1.grid_columnconfigure(0, weight=1)
checkFrame1.grid_columnconfigure(1, weight=1)
checkFrame1.grid_columnconfigure(2, weight=1)

J1calCbut = Checkbutton(checkFrame1, text="J1", variable=CAL['J1CalStatVal'])
J1calCbut.grid(row=0, column=0, sticky="w", padx=2)

J2calCbut = Checkbutton(checkFrame1, text="J2", variable=CAL['J2CalStatVal'])
J2calCbut.grid(row=0, column=1, sticky="w", padx=2)

J3calCbut = Checkbutton(checkFrame1, text="J3", variable=CAL['J3CalStatVal'])
J3calCbut.grid(row=0, column=2, sticky="w", padx=2)

J4calCbut = Checkbutton(checkFrame1, text="J4", variable=CAL['J4CalStatVal'])
J4calCbut.grid(row=1, column=0, sticky="w", padx=2)

J5calCbut = Checkbutton(checkFrame1, text="J5", variable=CAL['J5CalStatVal'])
J5calCbut.grid(row=1, column=1, sticky="w", padx=2)

J6calCbut = Checkbutton(checkFrame1, text="J6", variable=CAL['J6CalStatVal'])
J6calCbut.grid(row=1, column=2, sticky="w", padx=2)

J7calCbut = Checkbutton(checkFrame1, text="J7", variable=CAL['J7CalStatVal'])
J7calCbut.grid(row=2, column=0, sticky="w", padx=2)

J8calCbut = Checkbutton(checkFrame1, text="J8", variable=CAL['J8CalStatVal'])
J8calCbut.grid(row=2, column=1, sticky="w", padx=2)

J9calCbut = Checkbutton(checkFrame1, text="J9", variable=CAL['J9CalStatVal'])
J9calCbut.grid(row=2, column=2, sticky="w", padx=2)

# Second set of checkboxes (J1-J9)
checkFrame2 = Frame(calFrame)
checkFrame2.grid(row=2, column=0, sticky="ew", padx=5, pady=(10, 5))

checkFrame2.grid_columnconfigure(0, weight=1)
checkFrame2.grid_columnconfigure(1, weight=1)
checkFrame2.grid_columnconfigure(2, weight=1)

J1calCbut2 = Checkbutton(checkFrame2, text="J1", variable=CAL['J1CalStatVal2'])
J1calCbut2.grid(row=0, column=0, sticky="w", padx=2)

J2calCbut2 = Checkbutton(checkFrame2, text="J2", variable=CAL['J2CalStatVal2'])
J2calCbut2.grid(row=0, column=1, sticky="w", padx=2)

J3calCbut2 = Checkbutton(checkFrame2, text="J3", variable=CAL['J3CalStatVal2'])
J3calCbut2.grid(row=0, column=2, sticky="w", padx=2)

J4calCbut2 = Checkbutton(checkFrame2, text="J4", variable=CAL['J4CalStatVal2'])
J4calCbut2.grid(row=1, column=0, sticky="w", padx=2)

J5calCbut2 = Checkbutton(checkFrame2, text="J5", variable=CAL['J5CalStatVal2'])
J5calCbut2.grid(row=1, column=1, sticky="w", padx=2)

J6calCbut2 = Checkbutton(checkFrame2, text="J6", variable=CAL['J6CalStatVal2'])
J6calCbut2.grid(row=1, column=2, sticky="w", padx=2)

J7calCbut2 = Checkbutton(checkFrame2, text="J7", variable=CAL['J7CalStatVal2'])
J7calCbut2.grid(row=2, column=0, sticky="w", padx=2)

J8calCbut2 = Checkbutton(checkFrame2, text="J8", variable=CAL['J8CalStatVal2'])
J8calCbut2.grid(row=2, column=1, sticky="w", padx=2)

J9calCbut2 = Checkbutton(checkFrame2, text="J9", variable=CAL['J9CalStatVal2'])
J9calCbut2.grid(row=2, column=2, sticky="w", padx=2)

# Individual calibration buttons (EXACT names from original)
CalJ1But = Button(calFrame, text="Calibrate J1 Only", command=startCalRobotJ1)
CalJ1But.grid(row=3, column=0, sticky="ew", padx=5, pady=2)

CalJ2But = Button(calFrame, text="Calibrate J2 Only", command=startCalRobotJ2)
CalJ2But.grid(row=4, column=0, sticky="ew", padx=5, pady=2)

CalJ3But = Button(calFrame, text="Calibrate J3 Only", command=startCalRobotJ3)
CalJ3But.grid(row=5, column=0, sticky="ew", padx=5, pady=2)

CalJ4But = Button(calFrame, text="Calibrate J4 Only", command=startCalRobotJ4)
CalJ4But.grid(row=6, column=0, sticky="ew", padx=5, pady=2)

CalJ5But = Button(calFrame, text="Calibrate J5 Only", command=startCalRobotJ5)
CalJ5But.grid(row=7, column=0, sticky="ew", padx=5, pady=2)

CalJ6But = Button(calFrame, text="Calibrate J6 Only", command=startCalRobotJ6)
CalJ6But.grid(row=8, column=0, sticky="ew", padx=5, pady=(5,20))

ForceCalHomeBut = Button(calFrame, text="Force Cal Home", command=CalZeroPos)
ForceCalHomeBut.grid(row=9, column=0, sticky="ew", padx=5, pady=2)

ForceCalHomeBut = Button(calFrame, text="Force Cal Rest", command=CalRestPos)
ForceCalHomeBut.grid(row=10, column=0, sticky="ew", padx=5, pady=2)


# ============================================================================
# ROW 1, COLUMN 2: Calibration Offsets Frame
# ============================================================================
calOffsetFrame = LabelFrame(tab2, text="Calibration Offsets", padding=10)
calOffsetFrame.grid(row=1, column=2, sticky="nsew", padx=5, pady=5)

calOffsetFrame.grid_columnconfigure(0, weight=1)
calOffsetFrame.grid_columnconfigure(1, weight=1)

# J1 Offset
J1calLab = Label(calOffsetFrame, text="J1 Offset")
J1calLab.grid(row=0, column=0, sticky="e", padx=2, pady=2)
J1calOffEntryField = Entry(calOffsetFrame, width=5, justify="center")
J1calOffEntryField.grid(row=0, column=1, sticky="w", padx=2, pady=2)

# J2 Offset
J2calLab = Label(calOffsetFrame, text="J2 Offset")
J2calLab.grid(row=1, column=0, sticky="e", padx=2, pady=2)
J2calOffEntryField = Entry(calOffsetFrame, width=5, justify="center")
J2calOffEntryField.grid(row=1, column=1, sticky="w", padx=2, pady=2)

# J3 Offset
J3calLab = Label(calOffsetFrame, text="J3 Offset")
J3calLab.grid(row=2, column=0, sticky="e", padx=2, pady=2)
J3calOffEntryField = Entry(calOffsetFrame, width=5, justify="center")
J3calOffEntryField.grid(row=2, column=1, sticky="w", padx=2, pady=2)

# J4 Offset
J4calLab = Label(calOffsetFrame, text="J4 Offset")
J4calLab.grid(row=3, column=0, sticky="e", padx=2, pady=2)
J4calOffEntryField = Entry(calOffsetFrame, width=5, justify="center")
J4calOffEntryField.grid(row=3, column=1, sticky="w", padx=2, pady=2)

# J5 Offset
J5calLab = Label(calOffsetFrame, text="J5 Offset")
J5calLab.grid(row=4, column=0, sticky="e", padx=2, pady=2)
J5calOffEntryField = Entry(calOffsetFrame, width=5, justify="center")
J5calOffEntryField.grid(row=4, column=1, sticky="w", padx=2, pady=2)

# J6 Offset
J6calLab = Label(calOffsetFrame, text="J6 Offset")
J6calLab.grid(row=5, column=0, sticky="e", padx=2, pady=2)
J6calOffEntryField = Entry(calOffsetFrame, width=5, justify="center")
J6calOffEntryField.grid(row=5, column=1, sticky="w", padx=2, pady=2)

# J7 Offset
J7calLab = Label(calOffsetFrame, text="J7 Offset")
J7calLab.grid(row=6, column=0, sticky="e", padx=2, pady=2)
J7calOffEntryField = Entry(calOffsetFrame, width=5, justify="center")
J7calOffEntryField.grid(row=6, column=1, sticky="w", padx=2, pady=2)

# J8 Offset
J8calLab = Label(calOffsetFrame, text="J8 Offset")
J8calLab.grid(row=7, column=0, sticky="e", padx=2, pady=2)
J8calOffEntryField = Entry(calOffsetFrame, width=5, justify="center")
J8calOffEntryField.grid(row=7, column=1, sticky="w", padx=2, pady=2)

# J9 Offset
J9calLab = Label(calOffsetFrame, text="J9 Offset")
J9calLab.grid(row=8, column=0, sticky="e", padx=2, pady=2)
J9calOffEntryField = Entry(calOffsetFrame, width=5, justify="center")
J9calOffEntryField.grid(row=8, column=1, sticky="w", padx=2, pady=2)


# ============================================================================
# ROW 1, COLUMN 3: Encoder Control Frame
# ============================================================================
encoderFrame = LabelFrame(tab2, text="Encoder Control", padding=10)
encoderFrame.grid(row=1, column=3, sticky="nsew", padx=5, pady=5)

encoderFrame.grid_columnconfigure(0, weight=1)

# J1 Open Loop
J1OpenLoopCbut = Checkbutton(encoderFrame, text="J1 Open Loop (disable encoder)", variable=CAL['J1OpenLoopVal'])
J1OpenLoopCbut.grid(row=0, column=0, sticky="w", padx=5, pady=2)

# J2 Open Loop
J2OpenLoopCbut = Checkbutton(encoderFrame, text="J2 Open Loop (disable encoder)", variable=CAL['J2OpenLoopVal'])
J2OpenLoopCbut.grid(row=1, column=0, sticky="w", padx=5, pady=2)

# J3 Open Loop
J3OpenLoopCbut = Checkbutton(encoderFrame, text="J3 Open Loop (disable encoder)", variable=CAL['J3OpenLoopVal'])
J3OpenLoopCbut.grid(row=2, column=0, sticky="w", padx=5, pady=2)

# J4 Open Loop
J4OpenLoopCbut = Checkbutton(encoderFrame, text="J4 Open Loop (disable encoder)", variable=CAL['J4OpenLoopVal'])
J4OpenLoopCbut.grid(row=3, column=0, sticky="w", padx=5, pady=2)

# J5 Open Loop
J5OpenLoopCbut = Checkbutton(encoderFrame, text="J5 Open Loop (disable encoder)", variable=CAL['J5OpenLoopVal'])
J5OpenLoopCbut.grid(row=4, column=0, sticky="w", padx=5, pady=2)

# J6 Open Loop
J6OpenLoopCbut = Checkbutton(encoderFrame, text="J6 Open Loop (disable encoder)", variable=CAL['J6OpenLoopVal'])
J6OpenLoopCbut.grid(row=5, column=0, sticky="w", padx=5, pady=2)



# ============================================================================
# Color Configuration for Robot Visualization
# ============================================================================

main_color_parts = ["Link Base-2.STL", "Link 2-2.STL", "Link 4-2.STL"]
logo_color_parts = ["Link 2-3.STL", "Link 4-3.STL"]

def update_main_color(*args):
    selected = main_color_var.get()
    CAL['setColor'] = selected
    for part in main_color_parts:
        RUN['color_map'][part] = selected 
        RUN['actors'][part].GetProperty().SetColor(vtk.vtkNamedColors().GetColor3d(selected))
    RUN['render_window'].Render()

# Color options
color_options = [
    "Red", "IndianRed", "Crimson", "FireBrick", "DarkRed", "Maroon",
    "RosyBrown", "MediumVioletRed", "DeepPink", "HotPink", "Orchid", "Magenta",
    "Orange", "DarkOrange", "Tomato", "Gold", "Yellow", "Chartreuse", "YellowGreen",
    "Green", "LimeGreen", "MediumSpringGreen", "DarkOliveGreen", 
    "Teal", "DarkTurquoise", "Turquoise", "CadetBlue",
    "DodgerBlue", "Blue", "RoyalBlue", "SlateBlue", "MediumSlateBlue", 
    "Navy", "MidnightBlue", "SteelBlue",
    "Black", "DimGray", "DarkGray", "Gray", "Silver", 
    "LightSlateGray", "LightSteelBlue",
    "White", "Gainsboro", "AntiqueWhite", "Cornsilk"
]

# Initialize color variable
main_color_var = tk.StringVar(value="Royal Blue")


# ============================================================================
# ROW 1, COLUMN 5: Theme Frame
# ============================================================================
themeFrame = LabelFrame(tab2, text="Theme", padding=10)
themeFrame.grid(row=1, column=5, sticky="nsew", padx=5, pady=5)

themeFrame.grid_columnconfigure(0, weight=1)
themeFrame.grid_columnconfigure(1, weight=1)

# Theme buttons
lightBut = Button(themeFrame, text="  Light  ", command=lightTheme)
lightBut.grid(row=0, column=0, sticky="ew", padx=2, pady=2)

darkBut = Button(themeFrame, text="  Dark   ", command=darkTheme)
darkBut.grid(row=0, column=1, sticky="ew", padx=2, pady=2)

# Robot Color label and dropdown
robotColorLab = Label(themeFrame, text="Robot Color", font=("Arial", 10, "bold"))
robotColorLab.grid(row=1, column=0, columnspan=2, sticky="w", padx=2, pady=(10, 2))

main_color_dropdown = ttk.OptionMenu(themeFrame, main_color_var, main_color_var.get(), *color_options, command=update_main_color)
main_color_dropdown.grid(row=2, column=0, columnspan=2, sticky="ew", padx=2, pady=2)




# ============================================================================
# ROW 1, COLUMN 4: External Axes Frame
# ============================================================================
externalAxesFrame = LabelFrame(tab2, text="External Axes", padding=10)
externalAxesFrame.grid(row=1, column=4, sticky="nsew", padx=5, pady=5)

externalAxesFrame.grid_columnconfigure(0, weight=0)  # Labels
externalAxesFrame.grid_columnconfigure(1, weight=1)  # Entry fields

# --- 7th Axis Calibration ---
axis7Lab = Label(externalAxesFrame, font=("Arial 10 bold"), text="7th Axis Calibration")
axis7Lab.grid(row=0, column=0, columnspan=2, sticky="w", padx=5, pady=(5, 10))

axis7lengthLab = Label(externalAxesFrame, text="7th Axis Length:")
axis7lengthLab.grid(row=1, column=0, sticky="e", padx=5, pady=2)
axis7lengthEntryField = Entry(externalAxesFrame, width=5, justify="center")
axis7lengthEntryField.grid(row=1, column=1, sticky="w", padx=5, pady=2)

axis7rotLab = Label(externalAxesFrame, text="MM per Rotation:")
axis7rotLab.grid(row=2, column=0, sticky="e", padx=5, pady=2)
axis7rotEntryField = Entry(externalAxesFrame, width=5, justify="center")
axis7rotEntryField.grid(row=2, column=1, sticky="w", padx=5, pady=2)

axis7stepsLab = Label(externalAxesFrame, text="Drive Steps:")
axis7stepsLab.grid(row=3, column=0, sticky="e", padx=5, pady=2)
axis7stepsEntryField = Entry(externalAxesFrame, width=5, justify="center")
axis7stepsEntryField.grid(row=3, column=1, sticky="w", padx=5, pady=2)

J7zerobut = Button(externalAxesFrame, text="Set Axis 7 Calibration to Zero", width=28, command=zeroAxis7)
J7zerobut.grid(row=4, column=0, columnspan=2, sticky="ew", padx=5, pady=2)

J7calbut = Button(externalAxesFrame, text="Autocalibrate Axis 7", width=28, command=startCalRobotJ7)
J7calbut.grid(row=5, column=0, columnspan=2, sticky="ew", padx=5, pady=2)

axis7pinsetLab = Label(externalAxesFrame, font=("Arial", 8), text="StepPin = 12 / DirPin = 13 / CalPin = 36")
axis7pinsetLab.grid(row=6, column=0, columnspan=2, sticky="w", padx=5, pady=(2, 15))

# --- 8th Axis Calibration ---
axis8Lab = Label(externalAxesFrame, font=("Arial 10 bold"), text="8th Axis Calibration")
axis8Lab.grid(row=7, column=0, columnspan=2, sticky="w", padx=5, pady=(5, 10))

axis8lengthLab = Label(externalAxesFrame, text="8th Axis Length:")
axis8lengthLab.grid(row=8, column=0, sticky="e", padx=5, pady=2)
axis8lengthEntryField = Entry(externalAxesFrame, width=5, justify="center")
axis8lengthEntryField.grid(row=8, column=1, sticky="w", padx=5, pady=2)

axis8rotLab = Label(externalAxesFrame, text="MM per Rotation:")
axis8rotLab.grid(row=9, column=0, sticky="e", padx=5, pady=2)
axis8rotEntryField = Entry(externalAxesFrame, width=5, justify="center")
axis8rotEntryField.grid(row=9, column=1, sticky="w", padx=5, pady=2)

axis8stepsLab = Label(externalAxesFrame, text="Drive Steps:")
axis8stepsLab.grid(row=10, column=0, sticky="e", padx=5, pady=2)
axis8stepsEntryField = Entry(externalAxesFrame, width=5, justify="center")
axis8stepsEntryField.grid(row=10, column=1, sticky="w", padx=5, pady=2)

J8zerobut = Button(externalAxesFrame, text="Set Axis 8 Calibration to Zero", width=28, command=zeroAxis8)
J8zerobut.grid(row=11, column=0, columnspan=2, sticky="ew", padx=5, pady=2)

J8calbut = Button(externalAxesFrame, text="Autocalibrate Axis 8", width=28, command=startCalRobotJ8)
J8calbut.grid(row=12, column=0, columnspan=2, sticky="ew", padx=5, pady=2)

axis8pinsetLab = Label(externalAxesFrame, font=("Arial", 8), text="StepPin = 32 / DirPin = 33 / CalPin = 37")
axis8pinsetLab.grid(row=13, column=0, columnspan=2, sticky="w", padx=5, pady=(2, 15))

# --- 9th Axis Calibration ---
axis9Lab = Label(externalAxesFrame, font=("Arial 10 bold"), text="9th Axis Calibration")
axis9Lab.grid(row=14, column=0, columnspan=2, sticky="w", padx=5, pady=(5, 10))

axis9lengthLab = Label(externalAxesFrame, text="9th Axis Length:")
axis9lengthLab.grid(row=15, column=0, sticky="e", padx=5, pady=2)
axis9lengthEntryField = Entry(externalAxesFrame, width=5, justify="center")
axis9lengthEntryField.grid(row=15, column=1, sticky="w", padx=5, pady=2)

axis9rotLab = Label(externalAxesFrame, text="MM per Rotation:")
axis9rotLab.grid(row=16, column=0, sticky="e", padx=5, pady=2)
axis9rotEntryField = Entry(externalAxesFrame, width=5, justify="center")
axis9rotEntryField.grid(row=16, column=1, sticky="w", padx=5, pady=2)

axis9stepsLab = Label(externalAxesFrame, text="Drive Steps:")
axis9stepsLab.grid(row=17, column=0, sticky="e", padx=5, pady=2)
axis9stepsEntryField = Entry(externalAxesFrame, width=5, justify="center")
axis9stepsEntryField.grid(row=17, column=1, sticky="w", padx=5, pady=2)

J9zerobut = Button(externalAxesFrame, text="Set Axis 9 Calibration to Zero", width=28, command=zeroAxis9)
J9zerobut.grid(row=18, column=0, columnspan=2, sticky="ew", padx=5, pady=2)

J9calbut = Button(externalAxesFrame, text="Autocalibrate Axis 9", width=28, command=startCalRobotJ9)
J9calbut.grid(row=19, column=0, columnspan=2, sticky="ew", padx=5, pady=2)

axis9pinsetLab = Label(externalAxesFrame, font=("Arial", 8), text="StepPin = 34 / DirPin = 35 / CalPin = 38")
axis9pinsetLab.grid(row=20, column=0, columnspan=2, sticky="w", padx=5, pady=(2, 5))


# ============================================================================
# ROW 1, COLUMN 6: Virtual Import Frame
# ============================================================================
virtualImportFrame = LabelFrame(tab2, text="Virtual Import", padding=10)
virtualImportFrame.grid(row=1, column=6, sticky="nsew", padx=5, pady=5)

virtualImportFrame.grid_columnconfigure(0, weight=1)

# Import STL button
importSTLBut = ttk.Button(virtualImportFrame, text="Import STL", command=import_stl_file)
importSTLBut.grid(row=0, column=0, sticky="ew", padx=5, pady=5)

# File Name
fileNameLab = Label(virtualImportFrame, text="File Name")
fileNameLab.grid(row=1, column=0, sticky="w", padx=5, pady=(10, 2))
stl_name_entry = Entry(virtualImportFrame, textvariable=stl_name_var, width=20)
stl_name_entry.grid(row=2, column=0, sticky="ew", padx=5, pady=2)

# X Position
xPosLab = Label(virtualImportFrame, text="X Position")
xPosLab.grid(row=3, column=0, sticky="w", padx=5, pady=(10, 2))
x_entry = Entry(virtualImportFrame, textvariable=x_var, width=10)
x_entry.grid(row=4, column=0, sticky="w", padx=5, pady=2)

# Y Position
yPosLab = Label(virtualImportFrame, text="Y Position")
yPosLab.grid(row=5, column=0, sticky="w", padx=5, pady=(10, 2))
y_entry = Entry(virtualImportFrame, textvariable=y_var, width=10)
y_entry.grid(row=6, column=0, sticky="w", padx=5, pady=2)

# Z Position
zPosLab = Label(virtualImportFrame, text="Z Position")
zPosLab.grid(row=7, column=0, sticky="w", padx=5, pady=(10, 2))
z_entry = Entry(virtualImportFrame, textvariable=z_var, width=10)
z_entry.grid(row=8, column=0, sticky="w", padx=5, pady=2)

# Z Rotation
zRotLab = Label(virtualImportFrame, text="Z Rotation")
zRotLab.grid(row=9, column=0, sticky="w", padx=5, pady=(10, 2))
rot_entry = Entry(virtualImportFrame, textvariable=rot_var, width=10)
rot_entry.grid(row=10, column=0, sticky="w", padx=5, pady=2)

# Update Position button
updatePosBut = ttk.Button(virtualImportFrame, text="Update Position", command=update_stl_transform)
updatePosBut.grid(row=11, column=0, sticky="ew", padx=5, pady=(10, 5))

# ============================================================================
# ROW 2, COLUMN 5: Save Frame (below and right of Commands)
# ============================================================================
saveFrame = LabelFrame(tab2, text="Save", padding=10)
saveFrame.grid(row=2, column=5, columnspan=2, sticky="ew", padx=5, pady=5)

saveFrame.grid_columnconfigure(0, weight=1)
saveFrame.grid_rowconfigure(0, weight=1)  # Center vertically

# Save All button
saveCalBut = Button(saveFrame, text="SAVE ALL", width=15, command=SaveAndApplyCalibration)
saveCalBut.grid(row=0, column=0, sticky="", padx=5, pady=5)

# ============================================================================
# ROW 2: Commands Frame (spans all columns)
# ============================================================================
cmdFrame = LabelFrame(tab2, text="Commands", padding=10)
cmdFrame.grid(row=2, column=0, columnspan=5, sticky="ew", padx=5, pady=5)

cmdFrame.grid_columnconfigure(0, weight=1)

cmdSentLab = Label(cmdFrame, text="Last Command Sent to Controller")
cmdSentLab.grid(row=0, column=0, sticky="w", padx=5, pady=(0, 2))

cmdSentEntryField = Entry(cmdFrame, width=120, justify="center")
cmdSentEntryField.grid(row=1, column=0, sticky="ew", padx=5, pady=2)

cmdRecLab = Label(cmdFrame, text="Last Response From Controller")
cmdRecLab.grid(row=2, column=0, sticky="w", padx=5, pady=(10, 2))

cmdRecEntryField = Entry(cmdFrame, width=120, justify="center")
cmdRecEntryField.grid(row=3, column=0, sticky="ew", padx=5, pady=2)

####TAB 3

# ============================================================================
# Tab 3 Grid Layout Configuration
# ============================================================================
tab3.grid_rowconfigure(0, weight=1)
tab3.grid_rowconfigure(1, weight=1)
tab3.grid_columnconfigure(0, weight=0, minsize=180)  # Motor Dir, Cal Dir
tab3.grid_columnconfigure(1, weight=0, minsize=180)  # Pos Limits, Steps/Deg
tab3.grid_columnconfigure(2, weight=0, minsize=220)  # Drive MS, Encoder CPR
tab3.grid_columnconfigure(3, weight=0, minsize=280)  # DH Parameters, Tool Frame
tab3.grid_columnconfigure(4, weight=0, minsize=200)  # Defaults
tab3.grid_columnconfigure(5, weight=1)  # Remaining .place() widgets

# ============================================================================
# Motor Direction Frame (Row 0, Column 0)
# ============================================================================
motorDirFrame = LabelFrame(tab3, text="Motor Direction", padding=10)
motorDirFrame.grid(row=0, column=0, sticky="nsew", padx=5, pady=5)
motorDirFrame.grid_columnconfigure(0, weight=0)
motorDirFrame.grid_columnconfigure(1, weight=1)

J1MotDirLab_grid = Label(motorDirFrame, font=("Arial", 8), text="J1 Motor Direction")
J1MotDirLab_grid.grid(row=0, column=0, sticky="w", padx=5, pady=2)
J1MotDirEntryField = Entry(motorDirFrame, width=5, justify="center")
J1MotDirEntryField.grid(row=0, column=1, sticky="w", padx=5, pady=2)

J2MotDirLab_grid = Label(motorDirFrame, font=("Arial", 8), text="J2 Motor Direction")
J2MotDirLab_grid.grid(row=1, column=0, sticky="w", padx=5, pady=2)
J2MotDirEntryField = Entry(motorDirFrame, width=5, justify="center")
J2MotDirEntryField.grid(row=1, column=1, sticky="w", padx=5, pady=2)

J3MotDirLab_grid = Label(motorDirFrame, font=("Arial", 8), text="J3 Motor Direction")
J3MotDirLab_grid.grid(row=2, column=0, sticky="w", padx=5, pady=2)
J3MotDirEntryField = Entry(motorDirFrame, width=5, justify="center")
J3MotDirEntryField.grid(row=2, column=1, sticky="w", padx=5, pady=2)

J4MotDirLab_grid = Label(motorDirFrame, font=("Arial", 8), text="J4 Motor Direction")
J4MotDirLab_grid.grid(row=3, column=0, sticky="w", padx=5, pady=2)
J4MotDirEntryField = Entry(motorDirFrame, width=5, justify="center")
J4MotDirEntryField.grid(row=3, column=1, sticky="w", padx=5, pady=2)

J5MotDirLab_grid = Label(motorDirFrame, font=("Arial", 8), text="J5 Motor Direction")
J5MotDirLab_grid.grid(row=4, column=0, sticky="w", padx=5, pady=2)
J5MotDirEntryField = Entry(motorDirFrame, width=5, justify="center")
J5MotDirEntryField.grid(row=4, column=1, sticky="w", padx=5, pady=2)

J6MotDirLab_grid = Label(motorDirFrame, font=("Arial", 8), text="J6 Motor Direction")
J6MotDirLab_grid.grid(row=5, column=0, sticky="w", padx=5, pady=2)
J6MotDirEntryField = Entry(motorDirFrame, width=5, justify="center")
J6MotDirEntryField.grid(row=5, column=1, sticky="w", padx=5, pady=2)

J7MotDirLab_grid = Label(motorDirFrame, font=("Arial", 8), text="J7 Motor Direction")
J7MotDirLab_grid.grid(row=6, column=0, sticky="w", padx=5, pady=2)
J7MotDirEntryField = Entry(motorDirFrame, width=5, justify="center")
J7MotDirEntryField.grid(row=6, column=1, sticky="w", padx=5, pady=2)

J8MotDirLab_grid = Label(motorDirFrame, font=("Arial", 8), text="J8 Motor Direction")
J8MotDirLab_grid.grid(row=7, column=0, sticky="w", padx=5, pady=2)
J8MotDirEntryField = Entry(motorDirFrame, width=5, justify="center")
J8MotDirEntryField.grid(row=7, column=1, sticky="w", padx=5, pady=2)

J9MotDirLab_grid = Label(motorDirFrame, font=("Arial", 8), text="J9 Motor Direction")
J9MotDirLab_grid.grid(row=8, column=0, sticky="w", padx=5, pady=2)
J9MotDirEntryField = Entry(motorDirFrame, width=5, justify="center")
J9MotDirEntryField.grid(row=8, column=1, sticky="w", padx=5, pady=2)

# ============================================================================
# Calibration Direction Frame (Row 1, Column 0)
# ============================================================================
calDirFrame = LabelFrame(tab3, text="Calibration Direction", padding=10)
calDirFrame.grid(row=1, column=0, sticky="nsew", padx=5, pady=5)
calDirFrame.grid_columnconfigure(0, weight=0)
calDirFrame.grid_columnconfigure(1, weight=1)

J1CalDirLab_grid = Label(calDirFrame, font=("Arial", 8), text="J1 Calibration Dir.")
J1CalDirLab_grid.grid(row=0, column=0, sticky="w", padx=5, pady=2)
J1CalDirEntryField = Entry(calDirFrame, width=5, justify="center")
J1CalDirEntryField.grid(row=0, column=1, sticky="w", padx=5, pady=2)

J2CalDirLab_grid = Label(calDirFrame, font=("Arial", 8), text="J2 Calibration Dir.")
J2CalDirLab_grid.grid(row=1, column=0, sticky="w", padx=5, pady=2)
J2CalDirEntryField = Entry(calDirFrame, width=5, justify="center")
J2CalDirEntryField.grid(row=1, column=1, sticky="w", padx=5, pady=2)

J3CalDirLab_grid = Label(calDirFrame, font=("Arial", 8), text="J3 Calibration Dir.")
J3CalDirLab_grid.grid(row=2, column=0, sticky="w", padx=5, pady=2)
J3CalDirEntryField = Entry(calDirFrame, width=5, justify="center")
J3CalDirEntryField.grid(row=2, column=1, sticky="w", padx=5, pady=2)

J4CalDirLab_grid = Label(calDirFrame, font=("Arial", 8), text="J4 Calibration Dir.")
J4CalDirLab_grid.grid(row=3, column=0, sticky="w", padx=5, pady=2)
J4CalDirEntryField = Entry(calDirFrame, width=5, justify="center")
J4CalDirEntryField.grid(row=3, column=1, sticky="w", padx=5, pady=2)

J5CalDirLab_grid = Label(calDirFrame, font=("Arial", 8), text="J5 Calibration Dir.")
J5CalDirLab_grid.grid(row=4, column=0, sticky="w", padx=5, pady=2)
J5CalDirEntryField = Entry(calDirFrame, width=5, justify="center")
J5CalDirEntryField.grid(row=4, column=1, sticky="w", padx=5, pady=2)

J6CalDirLab_grid = Label(calDirFrame, font=("Arial", 8), text="J6 Calibration Dir.")
J6CalDirLab_grid.grid(row=5, column=0, sticky="w", padx=5, pady=2)
J6CalDirEntryField = Entry(calDirFrame, width=5, justify="center")
J6CalDirEntryField.grid(row=5, column=1, sticky="w", padx=5, pady=2)

J7CalDirLab_grid = Label(calDirFrame, font=("Arial", 8), text="J7 Calibration Dir.")
J7CalDirLab_grid.grid(row=6, column=0, sticky="w", padx=5, pady=2)
J7CalDirEntryField = Entry(calDirFrame, width=5, justify="center")
J7CalDirEntryField.grid(row=6, column=1, sticky="w", padx=5, pady=2)

J8CalDirLab_grid = Label(calDirFrame, font=("Arial", 8), text="J8 Calibration Dir.")
J8CalDirLab_grid.grid(row=7, column=0, sticky="w", padx=5, pady=2)
J8CalDirEntryField = Entry(calDirFrame, width=5, justify="center")
J8CalDirEntryField.grid(row=7, column=1, sticky="w", padx=5, pady=2)

J9CalDirLab_grid = Label(calDirFrame, font=("Arial", 8), text="J9 Calibration Dir.")
J9CalDirLab_grid.grid(row=8, column=0, sticky="w", padx=5, pady=2)
J9CalDirEntryField = Entry(calDirFrame, width=5, justify="center")
J9CalDirEntryField.grid(row=8, column=1, sticky="w", padx=5, pady=2)

# ============================================================================
# Position Limits Frame (Row 0, Column 1)
# ============================================================================
posLimFrame = LabelFrame(tab3, text="Position Limits", padding=10)
posLimFrame.grid(row=0, column=1, sticky="nsew", padx=5, pady=5)
posLimFrame.grid_columnconfigure(0, weight=0)
posLimFrame.grid_columnconfigure(1, weight=1)

J1PosLimLab_grid = Label(posLimFrame, font=("Arial", 8), text="J1 Pos Limit")
J1PosLimLab_grid.grid(row=0, column=0, sticky="w", padx=5, pady=2)
J1PosLimEntryField = Entry(posLimFrame, width=5, justify="center")
J1PosLimEntryField.grid(row=0, column=1, sticky="w", padx=5, pady=2)

J1NegLimLab_grid = Label(posLimFrame, font=("Arial", 8), text="J1 Neg Limit")
J1NegLimLab_grid.grid(row=1, column=0, sticky="w", padx=5, pady=2)
J1NegLimEntryField = Entry(posLimFrame, width=5, justify="center")
J1NegLimEntryField.grid(row=1, column=1, sticky="w", padx=5, pady=2)

J2PosLimLab_grid = Label(posLimFrame, font=("Arial", 8), text="J2 Pos Limit")
J2PosLimLab_grid.grid(row=2, column=0, sticky="w", padx=5, pady=2)
J2PosLimEntryField = Entry(posLimFrame, width=5, justify="center")
J2PosLimEntryField.grid(row=2, column=1, sticky="w", padx=5, pady=2)

J2NegLimLab_grid = Label(posLimFrame, font=("Arial", 8), text="J2 Neg Limit")
J2NegLimLab_grid.grid(row=3, column=0, sticky="w", padx=5, pady=2)
J2NegLimEntryField = Entry(posLimFrame, width=5, justify="center")
J2NegLimEntryField.grid(row=3, column=1, sticky="w", padx=5, pady=2)

J3PosLimLab_grid = Label(posLimFrame, font=("Arial", 8), text="J3 Pos Limit")
J3PosLimLab_grid.grid(row=4, column=0, sticky="w", padx=5, pady=2)
J3PosLimEntryField = Entry(posLimFrame, width=5, justify="center")
J3PosLimEntryField.grid(row=4, column=1, sticky="w", padx=5, pady=2)

J3NegLimLab_grid = Label(posLimFrame, font=("Arial", 8), text="J3 Neg Limit")
J3NegLimLab_grid.grid(row=5, column=0, sticky="w", padx=5, pady=2)
J3NegLimEntryField = Entry(posLimFrame, width=5, justify="center")
J3NegLimEntryField.grid(row=5, column=1, sticky="w", padx=5, pady=2)

J4PosLimLab_grid = Label(posLimFrame, font=("Arial", 8), text="J4 Pos Limit")
J4PosLimLab_grid.grid(row=6, column=0, sticky="w", padx=5, pady=2)
J4PosLimEntryField = Entry(posLimFrame, width=5, justify="center")
J4PosLimEntryField.grid(row=6, column=1, sticky="w", padx=5, pady=2)

J4NegLimLab_grid = Label(posLimFrame, font=("Arial", 8), text="J4 Neg Limit")
J4NegLimLab_grid.grid(row=7, column=0, sticky="w", padx=5, pady=2)
J4NegLimEntryField = Entry(posLimFrame, width=5, justify="center")
J4NegLimEntryField.grid(row=7, column=1, sticky="w", padx=5, pady=2)

J5PosLimLab_grid = Label(posLimFrame, font=("Arial", 8), text="J5 Pos Limit")
J5PosLimLab_grid.grid(row=8, column=0, sticky="w", padx=5, pady=2)
J5PosLimEntryField = Entry(posLimFrame, width=5, justify="center")
J5PosLimEntryField.grid(row=8, column=1, sticky="w", padx=5, pady=2)

J5NegLimLab_grid = Label(posLimFrame, font=("Arial", 8), text="J5 Neg Limit")
J5NegLimLab_grid.grid(row=9, column=0, sticky="w", padx=5, pady=2)
J5NegLimEntryField = Entry(posLimFrame, width=5, justify="center")
J5NegLimEntryField.grid(row=9, column=1, sticky="w", padx=5, pady=2)

J6PosLimLab_grid = Label(posLimFrame, font=("Arial", 8), text="J6 Pos Limit")
J6PosLimLab_grid.grid(row=10, column=0, sticky="w", padx=5, pady=2)
J6PosLimEntryField = Entry(posLimFrame, width=5, justify="center")
J6PosLimEntryField.grid(row=10, column=1, sticky="w", padx=5, pady=2)

J6NegLimLab_grid = Label(posLimFrame, font=("Arial", 8), text="J6 Neg Limit")
J6NegLimLab_grid.grid(row=11, column=0, sticky="w", padx=5, pady=2)
J6NegLimEntryField = Entry(posLimFrame, width=5, justify="center")
J6NegLimEntryField.grid(row=11, column=1, sticky="w", padx=5, pady=2)

# ============================================================================
# Steps per Degree Frame (Row 1, Column 1)
# ============================================================================
stepDegFrame = LabelFrame(tab3, text="Steps per Degree", padding=10)
stepDegFrame.grid(row=1, column=1, sticky="nsew", padx=5, pady=5)
stepDegFrame.grid_columnconfigure(0, weight=0)
stepDegFrame.grid_columnconfigure(1, weight=1)

J1StepDegLab_grid = Label(stepDegFrame, font=("Arial", 8), text="J1 Step/Deg")
J1StepDegLab_grid.grid(row=0, column=0, sticky="w", padx=5, pady=2)
J1StepDegEntryField = Entry(stepDegFrame, width=8, justify="center")
J1StepDegEntryField.grid(row=0, column=1, sticky="w", padx=5, pady=2)

J2StepDegLab_grid = Label(stepDegFrame, font=("Arial", 8), text="J2 Step/Deg")
J2StepDegLab_grid.grid(row=1, column=0, sticky="w", padx=5, pady=2)
J2StepDegEntryField = Entry(stepDegFrame, width=8, justify="center")
J2StepDegEntryField.grid(row=1, column=1, sticky="w", padx=5, pady=2)

J3StepDegLab_grid = Label(stepDegFrame, font=("Arial", 8), text="J3 Step/Deg")
J3StepDegLab_grid.grid(row=2, column=0, sticky="w", padx=5, pady=2)
J3StepDegEntryField = Entry(stepDegFrame, width=8, justify="center")
J3StepDegEntryField.grid(row=2, column=1, sticky="w", padx=5, pady=2)

J4StepDegLab_grid = Label(stepDegFrame, font=("Arial", 8), text="J4 Step/Deg")
J4StepDegLab_grid.grid(row=3, column=0, sticky="w", padx=5, pady=2)
J4StepDegEntryField = Entry(stepDegFrame, width=8, justify="center")
J4StepDegEntryField.grid(row=3, column=1, sticky="w", padx=5, pady=2)

J5StepDegLab_grid = Label(stepDegFrame, font=("Arial", 8), text="J5 Step/Deg")
J5StepDegLab_grid.grid(row=4, column=0, sticky="w", padx=5, pady=2)
J5StepDegEntryField = Entry(stepDegFrame, width=8, justify="center")
J5StepDegEntryField.grid(row=4, column=1, sticky="w", padx=5, pady=2)

J6StepDegLab_grid = Label(stepDegFrame, font=("Arial", 8), text="J6 Step/Deg")
J6StepDegLab_grid.grid(row=5, column=0, sticky="w", padx=5, pady=2)
J6StepDegEntryField = Entry(stepDegFrame, width=8, justify="center")
J6StepDegEntryField.grid(row=5, column=1, sticky="w", padx=5, pady=2)

# ============================================================================
# Drive Microsteps Frame (Row 0, Column 2)
# ============================================================================
driveMSFrame = LabelFrame(tab3, text="Drive Microsteps", padding=10)
driveMSFrame.grid(row=0, column=2, sticky="nsew", padx=5, pady=5)
driveMSFrame.grid_columnconfigure(0, weight=0)
driveMSFrame.grid_columnconfigure(1, weight=1)

J1DriveMSLab_grid = Label(driveMSFrame, font=("Arial", 8), text="J1 Drive Microstep")
J1DriveMSLab_grid.grid(row=0, column=0, sticky="w", padx=5, pady=2)
J1DriveMSEntryField = Entry(driveMSFrame, width=5, justify="center")
J1DriveMSEntryField.grid(row=0, column=1, sticky="w", padx=5, pady=2)

J2DriveMSLab_grid = Label(driveMSFrame, font=("Arial", 8), text="J2 Drive Microstep")
J2DriveMSLab_grid.grid(row=1, column=0, sticky="w", padx=5, pady=2)
J2DriveMSEntryField = Entry(driveMSFrame, width=5, justify="center")
J2DriveMSEntryField.grid(row=1, column=1, sticky="w", padx=5, pady=2)

J3DriveMSLab_grid = Label(driveMSFrame, font=("Arial", 8), text="J3 Drive Microstep")
J3DriveMSLab_grid.grid(row=2, column=0, sticky="w", padx=5, pady=2)
J3DriveMSEntryField = Entry(driveMSFrame, width=5, justify="center")
J3DriveMSEntryField.grid(row=2, column=1, sticky="w", padx=5, pady=2)

J4DriveMSLab_grid = Label(driveMSFrame, font=("Arial", 8), text="J4 Drive Microstep")
J4DriveMSLab_grid.grid(row=3, column=0, sticky="w", padx=5, pady=2)
J4DriveMSEntryField = Entry(driveMSFrame, width=5, justify="center")
J4DriveMSEntryField.grid(row=3, column=1, sticky="w", padx=5, pady=2)

J5DriveMSLab_grid = Label(driveMSFrame, font=("Arial", 8), text="J5 Drive Microstep")
J5DriveMSLab_grid.grid(row=4, column=0, sticky="w", padx=5, pady=2)
J5DriveMSEntryField = Entry(driveMSFrame, width=5, justify="center")
J5DriveMSEntryField.grid(row=4, column=1, sticky="w", padx=5, pady=2)

J6DriveMSLab_grid = Label(driveMSFrame, font=("Arial", 8), text="J6 Drive Microstep")
J6DriveMSLab_grid.grid(row=5, column=0, sticky="w", padx=5, pady=2)
J6DriveMSEntryField = Entry(driveMSFrame, width=5, justify="center")
J6DriveMSEntryField.grid(row=5, column=1, sticky="w", padx=5, pady=2)

# ============================================================================
# Encoder CPR Frame (Row 1, Column 2)
# ============================================================================
encCPRFrame = LabelFrame(tab3, text="Encoder CPR", padding=10)
encCPRFrame.grid(row=1, column=2, sticky="nsew", padx=5, pady=5)
encCPRFrame.grid_columnconfigure(0, weight=0)
encCPRFrame.grid_columnconfigure(1, weight=1)

J1EncCPRLab_grid = Label(encCPRFrame, font=("Arial", 8), text="J1 Encoder CPR")
J1EncCPRLab_grid.grid(row=0, column=0, sticky="w", padx=5, pady=2)
J1EncCPREntryField = Entry(encCPRFrame, width=5, justify="center")
J1EncCPREntryField.grid(row=0, column=1, sticky="w", padx=5, pady=2)

J2EncCPRLab_grid = Label(encCPRFrame, font=("Arial", 8), text="J2 Encoder CPR")
J2EncCPRLab_grid.grid(row=1, column=0, sticky="w", padx=5, pady=2)
J2EncCPREntryField = Entry(encCPRFrame, width=5, justify="center")
J2EncCPREntryField.grid(row=1, column=1, sticky="w", padx=5, pady=2)

J3EncCPRLab_grid = Label(encCPRFrame, font=("Arial", 8), text="J3 Encoder CPR")
J3EncCPRLab_grid.grid(row=2, column=0, sticky="w", padx=5, pady=2)
J3EncCPREntryField = Entry(encCPRFrame, width=5, justify="center")
J3EncCPREntryField.grid(row=2, column=1, sticky="w", padx=5, pady=2)

J4EncCPRLab_grid = Label(encCPRFrame, font=("Arial", 8), text="J4 Encoder CPR")
J4EncCPRLab_grid.grid(row=3, column=0, sticky="w", padx=5, pady=2)
J4EncCPREntryField = Entry(encCPRFrame, width=5, justify="center")
J4EncCPREntryField.grid(row=3, column=1, sticky="w", padx=5, pady=2)

J5EncCPRLab_grid = Label(encCPRFrame, font=("Arial", 8), text="J5 Encoder CPR")
J5EncCPRLab_grid.grid(row=4, column=0, sticky="w", padx=5, pady=2)
J5EncCPREntryField = Entry(encCPRFrame, width=5, justify="center")
J5EncCPREntryField.grid(row=4, column=1, sticky="w", padx=5, pady=2)

J6EncCPRLab_grid = Label(encCPRFrame, font=("Arial", 8), text="J6 Encoder CPR")
J6EncCPRLab_grid.grid(row=5, column=0, sticky="w", padx=5, pady=2)
J6EncCPREntryField = Entry(encCPRFrame, width=5, justify="center")
J6EncCPREntryField.grid(row=5, column=1, sticky="w", padx=5, pady=2)

# ============================================================================
# DH Parameters Frame (Row 0, Column 3)
# ============================================================================
dhParamsFrame = LabelFrame(tab3, text="DH Parameters", padding=10)
dhParamsFrame.grid(row=0, column=3, sticky="nsew", padx=5, pady=5)

# Column headers
dhParamsFrame.grid_columnconfigure(0, weight=0, minsize=30)   # J1-J6 labels
dhParamsFrame.grid_columnconfigure(1, weight=0, minsize=50)   # DH-Θ
dhParamsFrame.grid_columnconfigure(2, weight=0, minsize=50)   # DH-α
dhParamsFrame.grid_columnconfigure(3, weight=0, minsize=50)   # DH-d
dhParamsFrame.grid_columnconfigure(4, weight=0, minsize=50)   # DH-a

# Header row
Label(dhParamsFrame, font=("Arial", 8), text="").grid(row=0, column=0)
Label(dhParamsFrame, font=("Arial", 8), text="DH-Θ").grid(row=0, column=1)
Label(dhParamsFrame, font=("Arial", 8), text="DH-α").grid(row=0, column=2)
Label(dhParamsFrame, font=("Arial", 8), text="DH-d").grid(row=0, column=3)
Label(dhParamsFrame, font=("Arial", 8), text="DH-a").grid(row=0, column=4)

# J1 row
Label(dhParamsFrame, font=("Arial", 8), text="J1").grid(row=1, column=0, sticky="w", padx=5, pady=2)
J1ΘEntryField = Entry(dhParamsFrame, width=5, justify="center")
J1ΘEntryField.grid(row=1, column=1, padx=2, pady=2)
J1αEntryField = Entry(dhParamsFrame, width=5, justify="center")
J1αEntryField.grid(row=1, column=2, padx=2, pady=1)
J1dEntryField = Entry(dhParamsFrame, width=5, justify="center")
J1dEntryField.grid(row=1, column=3, padx=2, pady=2)
J1aEntryField = Entry(dhParamsFrame, width=5, justify="center")
J1aEntryField.grid(row=1, column=4, padx=2, pady=2)

# J2 row
Label(dhParamsFrame, font=("Arial", 8), text="J2").grid(row=2, column=0, sticky="w", padx=5, pady=2)
J2ΘEntryField = Entry(dhParamsFrame, width=5, justify="center")
J2ΘEntryField.grid(row=2, column=1, padx=2, pady=2)
J2αEntryField = Entry(dhParamsFrame, width=5, justify="center")
J2αEntryField.grid(row=2, column=2, padx=2, pady=1)
J2dEntryField = Entry(dhParamsFrame, width=5, justify="center")
J2dEntryField.grid(row=2, column=3, padx=2, pady=2)
J2aEntryField = Entry(dhParamsFrame, width=5, justify="center")
J2aEntryField.grid(row=2, column=4, padx=2, pady=2)

# J3 row
Label(dhParamsFrame, font=("Arial", 8), text="J3").grid(row=3, column=0, sticky="w", padx=5, pady=2)
J3ΘEntryField = Entry(dhParamsFrame, width=5, justify="center")
J3ΘEntryField.grid(row=3, column=1, padx=2, pady=2)
J3αEntryField = Entry(dhParamsFrame, width=5, justify="center")
J3αEntryField.grid(row=3, column=2, padx=2, pady=1)
J3dEntryField = Entry(dhParamsFrame, width=5, justify="center")
J3dEntryField.grid(row=3, column=3, padx=2, pady=2)
J3aEntryField = Entry(dhParamsFrame, width=5, justify="center")
J3aEntryField.grid(row=3, column=4, padx=2, pady=2)

# J4 row
Label(dhParamsFrame, font=("Arial", 8), text="J4").grid(row=4, column=0, sticky="w", padx=5, pady=2)
J4ΘEntryField = Entry(dhParamsFrame, width=5, justify="center")
J4ΘEntryField.grid(row=4, column=1, padx=2, pady=2)
J4αEntryField = Entry(dhParamsFrame, width=5, justify="center")
J4αEntryField.grid(row=4, column=2, padx=2, pady=1)
J4dEntryField = Entry(dhParamsFrame, width=5, justify="center")
J4dEntryField.grid(row=4, column=3, padx=2, pady=2)
J4aEntryField = Entry(dhParamsFrame, width=5, justify="center")
J4aEntryField.grid(row=4, column=4, padx=2, pady=2)

# J5 row
Label(dhParamsFrame, font=("Arial", 8), text="J5").grid(row=5, column=0, sticky="w", padx=5, pady=2)
J5ΘEntryField = Entry(dhParamsFrame, width=5, justify="center")
J5ΘEntryField.grid(row=5, column=1, padx=2, pady=2)
J5αEntryField = Entry(dhParamsFrame, width=5, justify="center")
J5αEntryField.grid(row=5, column=2, padx=2, pady=1)
J5dEntryField = Entry(dhParamsFrame, width=5, justify="center")
J5dEntryField.grid(row=5, column=3, padx=2, pady=2)
J5aEntryField = Entry(dhParamsFrame, width=5, justify="center")
J5aEntryField.grid(row=5, column=4, padx=2, pady=2)

# J6 row
Label(dhParamsFrame, font=("Arial", 8), text="J6").grid(row=6, column=0, sticky="w", padx=5, pady=2)
J6ΘEntryField = Entry(dhParamsFrame, width=5, justify="center")
J6ΘEntryField.grid(row=6, column=1, padx=2, pady=2)
J6αEntryField = Entry(dhParamsFrame, width=5, justify="center")
J6αEntryField.grid(row=6, column=2, padx=2, pady=1)
J6dEntryField = Entry(dhParamsFrame, width=5, justify="center")
J6dEntryField.grid(row=6, column=3, padx=2, pady=2)
J6aEntryField = Entry(dhParamsFrame, width=5, justify="center")
J6aEntryField.grid(row=6, column=4, padx=2, pady=2)

# ============================================================================
# Tool Frame Offset Frame (Row 1, Column 3)
# ============================================================================
toolFrameFrame = LabelFrame(tab3, text="Tool Frame Offset", padding=10)
toolFrameFrame.grid(row=1, column=3, sticky="nsew", padx=5, pady=5)

toolFrameFrame.grid_columnconfigure(0, weight=1)
toolFrameFrame.grid_columnconfigure(1, weight=1)
toolFrameFrame.grid_columnconfigure(2, weight=1)
toolFrameFrame.grid_columnconfigure(3, weight=1)
toolFrameFrame.grid_columnconfigure(4, weight=1)
toolFrameFrame.grid_columnconfigure(5, weight=1)

# Header row
Label(toolFrameFrame, font=("Arial", 11), text="X").grid(row=0, column=0, padx=2, pady=2)
Label(toolFrameFrame, font=("Arial", 11), text="Y").grid(row=0, column=1, padx=2, pady=2)
Label(toolFrameFrame, font=("Arial", 11), text="Z").grid(row=0, column=2, padx=2, pady=1)
Label(toolFrameFrame, font=("Arial", 11), text="Rz").grid(row=0, column=3, padx=2, pady=2)
Label(toolFrameFrame, font=("Arial", 11), text="Ry").grid(row=0, column=4, padx=2, pady=2)
Label(toolFrameFrame, font=("Arial", 11), text="Rx").grid(row=0, column=5, padx=2, pady=2)

# Entry fields row
TFxEntryField = Entry(toolFrameFrame, width=4, justify="center")
TFxEntryField.grid(row=1, column=0, padx=2, pady=2)
TFyEntryField = Entry(toolFrameFrame, width=4, justify="center")
TFyEntryField.grid(row=1, column=1, padx=2, pady=2)
TFzEntryField = Entry(toolFrameFrame, width=4, justify="center")
TFzEntryField.grid(row=1, column=2, padx=2, pady=1)
TFrzEntryField = Entry(toolFrameFrame, width=4, justify="center")
TFrzEntryField.grid(row=1, column=3, padx=2, pady=2)
TFryEntryField = Entry(toolFrameFrame, width=4, justify="center")
TFryEntryField.grid(row=1, column=4, padx=2, pady=2)
TFrxEntryField = Entry(toolFrameFrame, width=4, justify="center")
TFrxEntryField.grid(row=1, column=5, padx=2, pady=2)

# Checkbox row
DisableWristCbut = Checkbutton(toolFrameFrame, text="Disable Wrist Rotation - Linear Moves", variable=CAL['DisableWristRotVal'])
DisableWristCbut.grid(row=2, column=0, columnspan=6, sticky="w", padx=5, pady=5)

# ============================================================================
# Defaults Frame (Row 0-1, Column 4)
# ============================================================================
defaultsFrame = LabelFrame(tab3, text="Defaults", padding=10)
defaultsFrame.grid(row=0, column=4, rowspan=2, sticky="nsew", padx=5, pady=5)

defaultsFrame.grid_columnconfigure(0, weight=1)

loadAR4Mk4But = Button(defaultsFrame, text="Load AR4-MK4 Defaults", width=26, command=LoadAR4Mk3default)
loadAR4Mk4But.grid(row=0, column=0, padx=5, pady=5)

loadAR4Mk3But = Button(defaultsFrame, text="Load AR4-MK3 Defaults", width=26, command=LoadAR4Mk3default)
loadAR4Mk3But.grid(row=1, column=0, padx=5, pady=5)

loadAR4Mk2But = Button(defaultsFrame, text="Load AR4-MK2 Defaults", width=26, command=LoadAR4Mk2default)
loadAR4Mk2But.grid(row=2, column=0, padx=5, pady=5)

loadAR4But = Button(defaultsFrame, text="Load AR4-MK1 Defaults", width=26, command=LoadAR4default)
loadAR4But.grid(row=3, column=0, padx=5, pady=5)

loadAR3But = Button(defaultsFrame, text="Load AR3 Defaults", width=26, command=LoadAR3default)
loadAR3But.grid(row=4, column=0, padx=5, pady=5)

saveCalBut = Button(defaultsFrame, text="SAVE", width=26, command=SaveAndApplyCalibration)
saveCalBut.grid(row=5, column=0, padx=5, pady=(10, 30)) # 10 pixels above, 30 below


loadCustomBut = Button(defaultsFrame, text="Load Custom Calibration", width=26, command=load_custom_calibration)
loadCustomBut .grid(row=6, column=0, padx=5, pady=(30, 5))

saveCustomBut = Button(defaultsFrame, text="Save Custom Calibration", width=26, command=save_custom_calibration)
saveCustomBut.grid(row=7, column=0, padx=5, pady=5)

loadMaxStepBut = Button(defaultsFrame, text="Load Max Microsteps", width=26, command=LoadMaxdefault)
loadMaxStepBut.grid(row=8, column=0, padx=5, pady=5)



# #### TOOL FRAME ####
# ToolFrameLab = Label(tab3, text = "Tool Frame Offset")
# ToolFrameLab.place(x=970, y=60)
# 
# UFxLab = Label(tab3, font=("Arial", 11), text = "X")
# UFxLab.place(x=920, y=90)
# 
# UFyLab = Label(tab3, font=("Arial", 11), text = "Y")
# UFyLab.place(x=960, y=90)
# 
# UFzLab = Label(tab3, font=("Arial", 11), text = "Z")
# UFzLab.place(x=1000, y=90)
# 
# UFRxLab = Label(tab3, font=("Arial", 11), text = "Rz")
# UFRxLab.place(x=1040, y=90)
# 
# UFRyLab = Label(tab3, font=("Arial", 11), text = "Ry")
# UFRyLab.place(x=1080, y=90)
# 
# UFRzLab = Label(tab3, font=("Arial", 11), text = "Rx")
# UFRzLab.place(x=1120, y=90)
# 
# TFxEntryField = Entry(tab3,width=4,justify="center")
# TFxEntryField.place(x=910, y=115)
# TFyEntryField = Entry(tab3,width=4,justify="center")
# TFyEntryField.place(x=950, y=115)
# TFzEntryField = Entry(tab3,width=4,justify="center")
# TFzEntryField.place(x=990, y=115)
# TFrzEntryField = Entry(tab3,width=4,justify="center")
# TFrzEntryField.place(x=1030, y=115)
# TFryEntryField = Entry(tab3,width=4,justify="center")
# TFryEntryField.place(x=1070, y=115)
# TFrxEntryField = Entry(tab3,width=4,justify="center")
# TFrxEntryField.place(x=1110, y=115)
# 
# DisableWristCbut = Checkbutton(tab3, text="Disable Wrist Rotation - Linear Moves",variable = CAL['DisableWristRotVal'])
# DisableWristCbut.place(x=910, y=150)


# # ####  MOTOR DIRECTIONS ####

# # J1MotDirLab = Label(tab3, font=("Arial", 8), text = "J1 Motor Direction")
# # J1MotDirLab.place(x=10, y=20)
# # J2MotDirLab = Label(tab3, font=("Arial", 8), text = "J2 Motor Direction")
# # J2MotDirLab.place(x=10, y=45)
# # J3MotDirLab = Label(tab3, font=("Arial", 8), text = "J3 Motor Direction")
# # J3MotDirLab.place(x=10, y=70)
# # J4MotDirLab = Label(tab3, font=("Arial", 8), text = "J4 Motor Direction")
# # J4MotDirLab.place(x=10, y=95)
# # J5MotDirLab = Label(tab3, font=("Arial", 8), text = "J5 Motor Direction")
# # J5MotDirLab.place(x=10, y=120)
# # J6MotDirLab = Label(tab3, font=("Arial", 8), text = "J6 Motor Direction")
# # J6MotDirLab.place(x=10, y=145)
# # J7MotDirLab = Label(tab3, font=("Arial", 8), text = "J7 Motor Direction")
# # J7MotDirLab.place(x=10, y=170)
# # J8MotDirLab = Label(tab3, font=("Arial", 8), text = "J8 Motor Direction")
# # J8MotDirLab.place(x=10, y=195)
# # J9MotDirLab = Label(tab3, font=("Arial", 8), text = "J9 Motor Direction")
# # J9MotDirLab.place(x=10, y=220)

# # J1MotDirEntryField = Entry(tab3,width=5,justify="center")
# # J1MotDirEntryField.place(x=110, y=20)
# # J2MotDirEntryField = Entry(tab3,width=5,justify="center")
# # J2MotDirEntryField.place(x=110, y=45)
# # J3MotDirEntryField = Entry(tab3,width=5,justify="center")
# # J3MotDirEntryField.place(x=110, y=70)
# # J4MotDirEntryField = Entry(tab3,width=5,justify="center")
# # J4MotDirEntryField.place(x=110, y=95)
# # J5MotDirEntryField = Entry(tab3,width=5,justify="center")
# # J5MotDirEntryField.place(x=110, y=120)
# # J6MotDirEntryField = Entry(tab3,width=5,justify="center")
# # J6MotDirEntryField.place(x=110, y=145)
# # J7MotDirEntryField = Entry(tab3,width=5,justify="center")
# # J7MotDirEntryField.place(x=110, y=170)
# # J8MotDirEntryField = Entry(tab3,width=5,justify="center")
# # J8MotDirEntryField.place(x=110, y=195)
# # J9MotDirEntryField = Entry(tab3,width=5,justify="center")
# # J9MotDirEntryField.place(x=110, y=220)


# # ####  CALIBRATION DIRECTIONS ####

# # J1CalDirLab = Label(tab3, font=("Arial", 8), text = "J1 Calibration Dir.")
# # J1CalDirLab.place(x=10, y=280)
# # J2CalDirLab = Label(tab3, font=("Arial", 8), text = "J2 Calibration Dir.")
# # J2CalDirLab.place(x=10, y=305)
# # J3CalDirLab = Label(tab3, font=("Arial", 8), text = "J3 Calibration Dir.")
# # J3CalDirLab.place(x=10, y=330)
# # J4CalDirLab = Label(tab3, font=("Arial", 8), text = "J4 Calibration Dir.")
# # J4CalDirLab.place(x=10, y=355)
# # J5CalDirLab = Label(tab3, font=("Arial", 8), text = "J5 Calibration Dir.")
# # J5CalDirLab.place(x=10, y=380)
# # J6CalDirLab = Label(tab3, font=("Arial", 8), text = "J6 Calibration Dir.")
# # J6CalDirLab.place(x=10, y=405)
# # J7CalDirLab = Label(tab3, font=("Arial", 8), text = "J7 Calibration Dir.")
# # J7CalDirLab.place(x=10, y=430)
# # J8CalDirLab = Label(tab3, font=("Arial", 8), text = "J8 Calibration Dir.")
# # J8CalDirLab.place(x=10, y=455)
# # J9CalDirLab = Label(tab3, font=("Arial", 8), text = "J9 Calibration Dir.")
# # J9CalDirLab.place(x=10, y=480)

# # J1CalDirEntryField = Entry(tab3,width=5,justify="center")
# # J1CalDirEntryField.place(x=110, y=280)
# # J2CalDirEntryField = Entry(tab3,width=5,justify="center")
# # J2CalDirEntryField.place(x=110, y=305)
# # J3CalDirEntryField = Entry(tab3,width=5,justify="center")
# # J3CalDirEntryField.place(x=110, y=330)
# # J4CalDirEntryField = Entry(tab3,width=5,justify="center")
# # J4CalDirEntryField.place(x=110, y=355)
# # J5CalDirEntryField = Entry(tab3,width=5,justify="center")
# # J5CalDirEntryField.place(x=110, y=380)
# # J6CalDirEntryField = Entry(tab3,width=5,justify="center")
# # J6CalDirEntryField.place(x=110, y=405)
# # J7CalDirEntryField = Entry(tab3,width=5,justify="center")
# # J7CalDirEntryField.place(x=110, y=430)
# # J8CalDirEntryField = Entry(tab3,width=5,justify="center")
# # J8CalDirEntryField.place(x=110, y=455)
# # J9CalDirEntryField = Entry(tab3,width=5,justify="center")
# # J9CalDirEntryField.place(x=110, y=480)

# # ### axis limits
# # J1PosLimLab = Label(tab3, font=("Arial", 8), text = "J1 Pos Limit")
# # J1PosLimLab.place(x=200, y=20)
# # J1NegLimLab = Label(tab3, font=("Arial", 8), text = "J1 Neg Limit")
# # J1NegLimLab.place(x=200, y=45)
# # J2PosLimLab = Label(tab3, font=("Arial", 8), text = "J2 Pos Limit")
# # J2PosLimLab.place(x=200, y=70)
# # J2NegLimLab = Label(tab3, font=("Arial", 8), text = "J2 Neg Limit")
# # J2NegLimLab.place(x=200, y=95)
# # J3PosLimLab = Label(tab3, font=("Arial", 8), text = "J3 Pos Limit")
# # J3PosLimLab.place(x=200, y=120)
# # J3NegLimLab = Label(tab3, font=("Arial", 8), text = "J3 Neg Limit")
# # J3NegLimLab.place(x=200, y=145)
# # J4PosLimLab = Label(tab3, font=("Arial", 8), text = "J4 Pos Limit")
# # J4PosLimLab.place(x=200, y=170)
# # J4NegLimLab = Label(tab3, font=("Arial", 8), text = "J4 Neg Limit")
# # J4NegLimLab.place(x=200, y=195)
# # J5PosLimLab = Label(tab3, font=("Arial", 8), text = "J5 Pos Limit")
# # J5PosLimLab.place(x=200, y=220)
# # J5NegLimLab = Label(tab3, font=("Arial", 8), text = "J5 Neg Limit")
# # J5NegLimLab.place(x=200, y=245)
# # J6PosLimLab = Label(tab3, font=("Arial", 8), text = "J6 Pos Limit")
# # J6PosLimLab.place(x=200, y=270)
# # J6NegLimLab = Label(tab3, font=("Arial", 8), text = "J6 Neg Limit")
# # J6NegLimLab.place(x=200, y=295)

# # J1PosLimEntryField = Entry(tab3,width=5,justify="center")
# # J1PosLimEntryField.place(x=280, y=20)
# # J1NegLimEntryField = Entry(tab3,width=5,justify="center")
# # J1NegLimEntryField.place(x=280, y=45)
# # J2PosLimEntryField = Entry(tab3,width=5,justify="center")
# # J2PosLimEntryField.place(x=280, y=70)
# # J2NegLimEntryField = Entry(tab3,width=5,justify="center")
# # J2NegLimEntryField.place(x=280, y=95)
# # J3PosLimEntryField = Entry(tab3,width=5,justify="center")
# # J3PosLimEntryField.place(x=280, y=120)
# # J3NegLimEntryField = Entry(tab3,width=5,justify="center")
# # J3NegLimEntryField.place(x=280, y=145)
# # J4PosLimEntryField = Entry(tab3,width=5,justify="center")
# # J4PosLimEntryField.place(x=280, y=170)
# # J4NegLimEntryField = Entry(tab3,width=5,justify="center")
# # J4NegLimEntryField.place(x=280, y=195)
# # J5PosLimEntryField = Entry(tab3,width=5,justify="center")
# # J5PosLimEntryField.place(x=280, y=220)
# # J5NegLimEntryField = Entry(tab3,width=5,justify="center")
# # J5NegLimEntryField.place(x=280, y=245)
# # J6PosLimEntryField = Entry(tab3,width=5,justify="center")
# # J6PosLimEntryField.place(x=280, y=270)
# # J6NegLimEntryField = Entry(tab3,width=5,justify="center")
# # J6NegLimEntryField.place(x=280, y=295)


### steps per degress
# # J1StepDegLab = Label(tab3, font=("Arial", 8), text = "J1 Step/Deg")
# # J1StepDegLab.place(x=200, y=345)
# # J2StepDegLab = Label(tab3, font=("Arial", 8), text = "J2 Step/Deg")
# # J2StepDegLab.place(x=200, y=370)
# # J3StepDegLab = Label(tab3, font=("Arial", 8), text = "J3 Step/Deg")
# # J3StepDegLab.place(x=200, y=395)
# # J4StepDegLab = Label(tab3, font=("Arial", 8), text = "J4 Step/Deg")
# # J4StepDegLab.place(x=200, y=420)
# # J5StepDegLab = Label(tab3, font=("Arial", 8), text = "J5 Step/Deg")
# # J5StepDegLab.place(x=200, y=445)
# # J6StepDegLab = Label(tab3, font=("Arial", 8), text = "J6 Step/Deg")
# # J6StepDegLab.place(x=200, y=470)

# # J1StepDegEntryField = Entry(tab3,width=5,justify="center")
# # J1StepDegEntryField.place(x=280, y=345)
# # J2StepDegEntryField = Entry(tab3,width=5,justify="center")
# # J2StepDegEntryField.place(x=280, y=370)
# # J3StepDegEntryField = Entry(tab3,width=5,justify="center")
# # J3StepDegEntryField.place(x=280, y=395)
# # J4StepDegEntryField = Entry(tab3,width=5,justify="center")
# # J4StepDegEntryField.place(x=280, y=420)
# # J5StepDegEntryField = Entry(tab3,width=5,justify="center")
# # J5StepDegEntryField.place(x=280, y=445)
# # J6StepDegEntryField = Entry(tab3,width=5,justify="center")
# # J6StepDegEntryField.place(x=280, y=470)


### DRIVER STEPS
# # J1DriveMSLab = Label(tab3, font=("Arial", 8), text = "J1 Drive Microstep")
# # J1DriveMSLab.place(x=390, y=20)
# # J2DriveMSLab = Label(tab3, font=("Arial", 8), text = "J2 Drive Microstep")
# # J2DriveMSLab.place(x=390, y=45)
# # J3DriveMSLab = Label(tab3, font=("Arial", 8), text = "J3 Drive Microstep")
# # J3DriveMSLab.place(x=390, y=70)
# # J4DriveMSLab = Label(tab3, font=("Arial", 8), text = "J4 Drive Microstep")
# # J4DriveMSLab.place(x=390, y=95)
# # J5DriveMSLab = Label(tab3, font=("Arial", 8), text = "J5 Drive Microstep")
# # J5DriveMSLab.place(x=390, y=120)
# # J6DriveMSLab = Label(tab3, font=("Arial", 8), text = "J6 Drive Microstep")
# # J6DriveMSLab.place(x=390, y=145)

# # J1DriveMSEntryField = Entry(tab3,width=5,justify="center")
# # J1DriveMSEntryField.place(x=500, y=20)
# # J2DriveMSEntryField = Entry(tab3,width=5,justify="center")
# # J2DriveMSEntryField.place(x=500, y=45)
# # J3DriveMSEntryField = Entry(tab3,width=5,justify="center")
# # J3DriveMSEntryField.place(x=500, y=70)
# # J4DriveMSEntryField = Entry(tab3,width=5,justify="center")
# # J4DriveMSEntryField.place(x=500, y=95)
# # J5DriveMSEntryField = Entry(tab3,width=5,justify="center")
# # J5DriveMSEntryField.place(x=500, y=120)
# # J6DriveMSEntryField = Entry(tab3,width=5,justify="center")
# # J6DriveMSEntryField.place(x=500, y=145)


###ENCODER CPR
# # J1EncCPRLab = Label(tab3, font=("Arial", 8), text = "J1 Encoder CPR")
# # J1EncCPRLab.place(x=390, y=195)
# # J2EncCPRLab = Label(tab3, font=("Arial", 8), text = "J2 Encoder CPR")
# # J2EncCPRLab.place(x=390, y=220)
# # J3EncCPRLab = Label(tab3, font=("Arial", 8), text = "J3 Encoder CPR")
# # J3EncCPRLab.place(x=390, y=245)
# # J4EncCPRLab = Label(tab3, font=("Arial", 8), text = "J4 Encoder CPR")
# # J4EncCPRLab.place(x=390, y=270)
# # J5EncCPRLab = Label(tab3, font=("Arial", 8), text = "J5 Encoder CPR")
# # J5EncCPRLab.place(x=390, y=295)
# # J6EncCPRLab = Label(tab3, font=("Arial", 8), text = "J6 Encoder CPR")
# # J6EncCPRLab.place(x=390, y=320)

# # J1EncCPREntryField = Entry(tab3,width=5,justify="center")
# # J1EncCPREntryField.place(x=500, y=195)
# # J2EncCPREntryField = Entry(tab3,width=5,justify="center")
# # J2EncCPREntryField.place(x=500, y=220)
# # J3EncCPREntryField = Entry(tab3,width=5,justify="center")
# # J3EncCPREntryField.place(x=500, y=245)
# # J4EncCPREntryField = Entry(tab3,width=5,justify="center")
# # J4EncCPREntryField.place(x=500, y=270)
# # J5EncCPREntryField = Entry(tab3,width=5,justify="center")
# # J5EncCPREntryField.place(x=500, y=295)
# # J6EncCPREntryField = Entry(tab3,width=5,justify="center")
# # J6EncCPREntryField.place(x=500, y=320)


# ### DH PARAMS
# 
# ### DRIVER STEPS
# J1DHparamLab = Label(tab3, font=("Arial", 8), text = "J1")
# J1DHparamLab.place(x=600, y=45)
# J1DHparamLab = Label(tab3, font=("Arial", 8), text = "J2")
# J1DHparamLab.place(x=600, y=70)
# J1DHparamLab = Label(tab3, font=("Arial", 8), text = "J3")
# J1DHparamLab.place(x=600, y=95)
# J1DHparamLab = Label(tab3, font=("Arial", 8), text = "J4")
# J1DHparamLab.place(x=600, y=120)
# J1DHparamLab = Label(tab3, font=("Arial", 8), text = "J5")
# J1DHparamLab.place(x=600, y=145)
# J1DHparamLab = Label(tab3, font=("Arial", 8), text = "J6")
# J1DHparamLab.place(x=600, y=170)
# 
# ΘDHparamLab = Label(tab3, font=("Arial", 8), text = "DH-Θ")
# ΘDHparamLab.place(x=645, y=20)
# αDHparamLab = Label(tab3, font=("Arial", 8), text = "DH-α")
# αDHparamLab.place(x=700, y=20)
# dDHparamLab = Label(tab3, font=("Arial", 8), text = "DH-d")
# dDHparamLab.place(x=755, y=20)
# aDHparamLab = Label(tab3, font=("Arial", 8), text = "DH-a")
# aDHparamLab.place(x=810, y=20)
# 
# 
# J1ΘEntryField = Entry(tab3,width=5,justify="center")
# J1ΘEntryField.place(x=630, y=45)
# J2ΘEntryField = Entry(tab3,width=5,justify="center")
# J2ΘEntryField.place(x=630, y=70)
# J3ΘEntryField = Entry(tab3,width=5,justify="center")
# J3ΘEntryField.place(x=630, y=95)
# J4ΘEntryField = Entry(tab3,width=5,justify="center")
# J4ΘEntryField.place(x=630, y=120)
# J5ΘEntryField = Entry(tab3,width=5,justify="center")
# J5ΘEntryField.place(x=630, y=145)
# J6ΘEntryField = Entry(tab3,width=5,justify="center")
# J6ΘEntryField.place(x=630, y=170)
# 
# J1αEntryField = Entry(tab3,width=5,justify="center")
# J1αEntryField.place(x=685, y=45)
# J2αEntryField = Entry(tab3,width=5,justify="center")
# J2αEntryField.place(x=685, y=70)
# J3αEntryField = Entry(tab3,width=5,justify="center")
# J3αEntryField.place(x=685, y=95)
# J4αEntryField = Entry(tab3,width=5,justify="center")
# J4αEntryField.place(x=685, y=120)
# J5αEntryField = Entry(tab3,width=5,justify="center")
# J5αEntryField.place(x=685, y=145)
# J6αEntryField = Entry(tab3,width=5,justify="center")
# J6αEntryField.place(x=685, y=170)
# 
# J1dEntryField = Entry(tab3,width=5,justify="center")
# J1dEntryField.place(x=740, y=45)
# J2dEntryField = Entry(tab3,width=5,justify="center")
# J2dEntryField.place(x=740, y=70)
# J3dEntryField = Entry(tab3,width=5,justify="center")
# J3dEntryField.place(x=740, y=95)
# J4dEntryField = Entry(tab3,width=5,justify="center")
# J4dEntryField.place(x=740, y=120)
# J5dEntryField = Entry(tab3,width=5,justify="center")
# J5dEntryField.place(x=740, y=145)
# J6dEntryField = Entry(tab3,width=5,justify="center")
# J6dEntryField.place(x=740, y=170)
# 
# J1aEntryField = Entry(tab3,width=5,justify="center")
# J1aEntryField.place(x=795, y=45)
# J2aEntryField = Entry(tab3,width=5,justify="center")
# J2aEntryField.place(x=795, y=70)
# J3aEntryField = Entry(tab3,width=5,justify="center")
# J3aEntryField.place(x=795, y=95)
# J4aEntryField = Entry(tab3,width=5,justify="center")
# J4aEntryField.place(x=795, y=120)
# J5aEntryField = Entry(tab3,width=5,justify="center")
# J5aEntryField.place(x=795, y=145)
# J6aEntryField = Entry(tab3,width=5,justify="center")
# J6aEntryField.place(x=795, y=170)


# ### LOAD DEFAULT ###
# 
# loadAR4Mk2But = Button(tab3,  text="Load AR4-MK3 Defaults",  width=26, command = LoadAR4Mk3default)
# loadAR4Mk2But.place(x=1150, y=470)
# 
# loadAR4Mk2But = Button(tab3,  text="Load AR4-MK2 Defaults",  width=26, command = LoadAR4Mk2default)
# loadAR4Mk2But.place(x=1150, y=510)
# 
# loadAR4But = Button(tab3,  text="Load AR4 Defaults",  width=26, command = LoadAR4default)
# loadAR4But.place(x=1150, y=550)
# 
# loadAR3But = Button(tab3,  text="Load AR3 Defaults",  width=26, command = LoadAR3default)
# loadAR3But.place(x=1150, y=590)
# 
# 
# 
# 
# 
# 
# #### SAVE ####
# 
# saveCalBut = Button(tab3,  text="SAVE",  width=26, command = SaveAndApplyCalibration)
# saveCalBut.place(x=1150, y=630)



####################################################################################################################################################
####################################################################################################################################################
####################################################################################################################################################
####TAB 4

# ============================================================================
# Tab 4 Grid Layout Configuration
# ============================================================================
tab4.grid_rowconfigure(0, weight=1)
tab4.grid_rowconfigure(1, weight=0)
tab4.grid_columnconfigure(0, weight=0, minsize=300)  # 5v IO BOARD
tab4.grid_columnconfigure(1, weight=0, minsize=250)  # AUX COM DEVICE
tab4.grid_columnconfigure(2, weight=0, minsize=400)  # MODBUS DEVICE
tab4.grid_columnconfigure(3, weight=1)  # Remaining space

# ============================================================================
# 5v IO BOARD Frame (Row 0, Column 0)
# ============================================================================
ioBoardFrame = LabelFrame(tab4, text="5v IO BOARD", padding=10)
ioBoardFrame.grid(row=0, column=0, sticky="nsew", padx=5, pady=5)

ioBoardFrame.grid_columnconfigure(0, weight=0, minsize=60)   # Servo buttons
ioBoardFrame.grid_columnconfigure(1, weight=0, minsize=10)   # = labels
ioBoardFrame.grid_columnconfigure(2, weight=0, minsize=35)   # Entry fields
ioBoardFrame.grid_columnconfigure(3, weight=0, minsize=10)   # Spacer between columns
ioBoardFrame.grid_columnconfigure(4, weight=0, minsize=60)   # DO buttons
ioBoardFrame.grid_columnconfigure(5, weight=0, minsize=10)   # = labels
ioBoardFrame.grid_columnconfigure(6, weight=0, minsize=35)   # Entry fields (no expansion)

# Configure all rows to have uniform minimum height
for row in range(12):
    ioBoardFrame.grid_rowconfigure(row, minsize=25)

# Servo 0 on
servo0onBut = Button(ioBoardFrame, text="Servo 0", command=Servo0on)
servo0onBut.grid(row=0, column=0, sticky="ew", padx=2, pady=1)
Label(ioBoardFrame, text="=").grid(row=0, column=1, padx=2, pady=1)
servo0onEntryField = Entry(ioBoardFrame, width=4, justify="center")
servo0onEntryField.grid(row=0, column=2, padx=2, pady=1)

# DO on (row 0)
DO1onBut = Button(ioBoardFrame, text="DO on", command=DO1on)
DO1onBut.grid(row=0, column=4, sticky="ew", padx=2, pady=1)
Label(ioBoardFrame, text="=").grid(row=0, column=5, padx=2, pady=1)
DO1onEntryField = Entry(ioBoardFrame, width=4, justify="center")
DO1onEntryField.grid(row=0, column=6, padx=2, pady=1)

# Servo 0 off
servo0offBut = Button(ioBoardFrame, text="Servo 0", command=Servo0off)
servo0offBut.grid(row=1, column=0, sticky="ew", padx=2, pady=1)
Label(ioBoardFrame, text="=").grid(row=1, column=1, padx=2, pady=1)
servo0offEntryField = Entry(ioBoardFrame, width=4, justify="center")
servo0offEntryField.grid(row=1, column=2, padx=2, pady=1)

# DO off (row 1)
DO1offBut = Button(ioBoardFrame, text="DO off", command=DO1off)
DO1offBut.grid(row=1, column=4, sticky="ew", padx=2, pady=1)
Label(ioBoardFrame, text="=").grid(row=1, column=5, padx=2, pady=1)
DO1offEntryField = Entry(ioBoardFrame, width=4, justify="center")
DO1offEntryField.grid(row=1, column=6, padx=2, pady=1)

# Servo 1 on
servo1onBut = Button(ioBoardFrame, text="Servo 1", command=Servo1on)
servo1onBut.grid(row=2, column=0, sticky="ew", padx=2, pady=1)
Label(ioBoardFrame, text="=").grid(row=2, column=1, padx=2, pady=1)
servo1onEntryField = Entry(ioBoardFrame, width=4, justify="center")
servo1onEntryField.grid(row=2, column=2, padx=2, pady=1)

# DO on (row 2)
DO2onBut = Button(ioBoardFrame, text="DO on", command=DO2on)
DO2onBut.grid(row=2, column=4, sticky="ew", padx=2, pady=1)
Label(ioBoardFrame, text="=").grid(row=2, column=5, padx=2, pady=1)
DO2onEntryField = Entry(ioBoardFrame, width=4, justify="center")
DO2onEntryField.grid(row=2, column=6, padx=2, pady=1)

# Servo 1 off
servo1offBut = Button(ioBoardFrame, text="Servo 1", command=Servo1off)
servo1offBut.grid(row=3, column=0, sticky="ew", padx=2, pady=1)
Label(ioBoardFrame, text="=").grid(row=3, column=1, padx=2, pady=1)
servo1offEntryField = Entry(ioBoardFrame, width=4, justify="center")
servo1offEntryField.grid(row=3, column=2, padx=2, pady=1)

# DO off (row 3)
DO2offBut = Button(ioBoardFrame, text="DO off", command=DO2off)
DO2offBut.grid(row=3, column=4, sticky="ew", padx=2, pady=1)
Label(ioBoardFrame, text="=").grid(row=3, column=5, padx=2, pady=1)
DO2offEntryField = Entry(ioBoardFrame, width=4, justify="center")
DO2offEntryField.grid(row=3, column=6, padx=2, pady=1)

# Servo 2 on
servo2onBut = Button(ioBoardFrame, text="Servo 2", command=Servo2on)
servo2onBut.grid(row=4, column=0, sticky="ew", padx=2, pady=1)
Label(ioBoardFrame, text="=").grid(row=4, column=1, padx=2, pady=1)
servo2onEntryField = Entry(ioBoardFrame, width=4, justify="center")
servo2onEntryField.grid(row=4, column=2, padx=2, pady=1)

# DO on (row 4)
DO3onBut = Button(ioBoardFrame, text="DO on", command=DO3on)
DO3onBut.grid(row=4, column=4, sticky="ew", padx=2, pady=1)
Label(ioBoardFrame, text="=").grid(row=4, column=5, padx=2, pady=1)
DO3onEntryField = Entry(ioBoardFrame, width=4, justify="center")
DO3onEntryField.grid(row=4, column=6, padx=2, pady=1)

# Servo 2 off
servo2offBut = Button(ioBoardFrame, text="Servo 2", command=Servo2off)
servo2offBut.grid(row=5, column=0, sticky="ew", padx=2, pady=1)
Label(ioBoardFrame, text="=").grid(row=5, column=1, padx=2, pady=1)
servo2offEntryField = Entry(ioBoardFrame, width=4, justify="center")
servo2offEntryField.grid(row=5, column=2, padx=2, pady=1)

# DO off (row 5)
DO3offBut = Button(ioBoardFrame, text="DO off", command=DO3off)
DO3offBut.grid(row=5, column=4, sticky="ew", padx=2, pady=1)
Label(ioBoardFrame, text="=").grid(row=5, column=5, padx=2, pady=1)
DO3offEntryField = Entry(ioBoardFrame, width=4, justify="center")
DO3offEntryField.grid(row=5, column=6, padx=2, pady=1)

# Servo 3 on
servo3onBut = Button(ioBoardFrame, text="Servo 3", command=Servo3on)
servo3onBut.grid(row=6, column=0, sticky="ew", padx=2, pady=1)
Label(ioBoardFrame, text="=").grid(row=6, column=1, padx=2, pady=1)
servo3onEntryField = Entry(ioBoardFrame, width=4, justify="center")
servo3onEntryField.grid(row=6, column=2, padx=2, pady=1)

# DO on (row 6)
DO4onBut = Button(ioBoardFrame, text="DO on", command=DO4on)
DO4onBut.grid(row=6, column=4, sticky="ew", padx=2, pady=1)
Label(ioBoardFrame, text="=").grid(row=6, column=5, padx=2, pady=1)
DO4onEntryField = Entry(ioBoardFrame, width=4, justify="center")
DO4onEntryField.grid(row=6, column=6, padx=2, pady=1)

# Servo 3 off
servo3offBut = Button(ioBoardFrame, text="Servo 3", command=Servo3off)
servo3offBut.grid(row=7, column=0, sticky="ew", padx=2, pady=1)
Label(ioBoardFrame, text="=").grid(row=7, column=1, padx=2, pady=1)
servo3offEntryField = Entry(ioBoardFrame, width=4, justify="center")
servo3offEntryField.grid(row=7, column=2, padx=2, pady=1)

# DO off (row 7)
DO4offBut = Button(ioBoardFrame, text="DO off", command=DO4off)
DO4offBut.grid(row=7, column=4, sticky="ew", padx=2, pady=1)
Label(ioBoardFrame, text="=").grid(row=7, column=5, padx=2, pady=1)
DO4offEntryField = Entry(ioBoardFrame, width=4, justify="center")
DO4offEntryField.grid(row=7, column=6, padx=2, pady=1)



# DO on (row 8) - no servo
DO5onBut = Button(ioBoardFrame, text="DO on", command=DO5on)
DO5onBut.grid(row=8, column=4, sticky="ew", padx=2, pady=1)
Label(ioBoardFrame, text="=").grid(row=8, column=5, padx=2, pady=1)
DO5onEntryField = Entry(ioBoardFrame, width=4, justify="center")
DO5onEntryField.grid(row=8, column=6, padx=2, pady=1)



# DO off (row 9)
DO5offBut = Button(ioBoardFrame, text="DO off", command=DO5off)
DO5offBut.grid(row=9, column=4, sticky="ew", padx=2, pady=1)
Label(ioBoardFrame, text="=").grid(row=9, column=5, padx=2, pady=1)
DO5offEntryField = Entry(ioBoardFrame, width=4, justify="center")
DO5offEntryField.grid(row=9, column=6, padx=2, pady=1)



# DO on (row 10)
DO6onBut = Button(ioBoardFrame, text="DO on", command=DO6on)
DO6onBut.grid(row=10, column=4, sticky="ew", padx=2, pady=1)
Label(ioBoardFrame, text="=").grid(row=10, column=5, padx=2, pady=1)
DO6onEntryField = Entry(ioBoardFrame, width=4, justify="center")
DO6onEntryField.grid(row=10, column=6, padx=2, pady=1)



# DO off (row 11)
DO6offBut = Button(ioBoardFrame, text="DO off", command=DO6off)
DO6offBut.grid(row=11, column=4, sticky="ew", padx=2, pady=1)
Label(ioBoardFrame, text="=").grid(row=11, column=5, padx=2, pady=1)
DO6offEntryField = Entry(ioBoardFrame, width=4, justify="center")
DO6offEntryField.grid(row=11, column=6, padx=2, pady=1)

# ============================================================================
# AUX COM DEVICE Frame (Row 0, Column 1)
# ============================================================================
auxComFrame = LabelFrame(tab4, text="AUX COM DEVICE", padding=10)
auxComFrame.grid(row=0, column=1, sticky="nsew", padx=5, pady=5)

auxComFrame.grid_columnconfigure(0, weight=1)

# Aux Com Port
Label(auxComFrame, text="Aux Com Port").grid(row=0, column=0, sticky="w", padx=(5,2), pady=5)
com3PortEntryField = Entry(auxComFrame, width=10, justify="left")
com3PortEntryField.grid(row=0, column=1, sticky="w", padx=(2,5), pady=5)

# Char to Read
Label(auxComFrame, text="Char to Read").grid(row=1, column=0, sticky="w", padx=(5,2), pady=5)
com3charPortEntryField = Entry(auxComFrame, width=10, justify="left")
com3charPortEntryField.grid(row=1, column=1, sticky="w", padx=(2,5), pady=5)

# Test button
comPortBut3 = Button(auxComFrame, text="Test Aux COM Device", command=TestAuxCom)
comPortBut3.grid(row=2, column=0, columnspan=2, sticky="ew", padx=5, pady=5)

# Output
com3outPortEntryField = Entry(auxComFrame, width=25, justify="center")
com3outPortEntryField.grid(row=3, column=0, columnspan=2, sticky="ew", padx=5, pady=5)

# ============================================================================
# MODBUS DEVICE Frame (Row 0, Column 2)
# ============================================================================
modbusFrame = LabelFrame(tab4, text="MODBUS DEVICE", padding=10)
modbusFrame.grid(row=0, column=2, sticky="nsew", padx=5, pady=5)

modbusFrame.grid_columnconfigure(0, weight=1)

# Slave ID
Label(modbusFrame, text="Slave ID").grid(row=0, column=0, sticky="w", padx=(5,2), pady=5)
MBslaveEntryField = Entry(modbusFrame, width=10, justify="left")
MBslaveEntryField.grid(row=0, column=1, sticky="w", padx=(2,5), pady=5)

# Modbus Address
Label(modbusFrame, text="Modbus Address").grid(row=1, column=0, sticky="w", padx=(5,2), pady=5)
MBaddressEntryField = Entry(modbusFrame, width=10, justify="left")
MBaddressEntryField.grid(row=1, column=1, sticky="w", padx=(2,5), pady=5)

# Operation Value
Label(modbusFrame, text="Operation Value").grid(row=2, column=0, sticky="w", padx=(5,2), pady=5)
MBoperValEntryField = Entry(modbusFrame, width=10, justify="left")
MBoperValEntryField.grid(row=2, column=1, sticky="w", padx=(2,5), pady=5)

# Buttons
MBreadCoilBut = Button(modbusFrame, text="Read Coil", width=30, command=MBreadCoil)
MBreadCoilBut.grid(row=3, column=0, columnspan=2, sticky="ew", padx=5, pady=2)

MBreadDinputBut = Button(modbusFrame, text="Read Discrete Input", width=30, command=MBreadInput)
MBreadDinputBut.grid(row=4, column=0, columnspan=2, sticky="ew", padx=5, pady=2)

MBreadHoldRegBut = Button(modbusFrame, text="Read Holding Register", width=30, command=MBreadHoldReg)
MBreadHoldRegBut.grid(row=5, column=0, columnspan=2, sticky="ew", padx=5, pady=2)

MBreadInputRegBut = Button(modbusFrame, text="Read Input Register", width=30, command=MBreadInputReg)
MBreadInputRegBut.grid(row=6, column=0, columnspan=2, sticky="ew", padx=5, pady=2)

MBwriteCoilBut = Button(modbusFrame, text="Write Coil", width=30, command=MBwriteCoil)
MBwriteCoilBut.grid(row=7, column=0, columnspan=2, sticky="ew", padx=5, pady=2)

MBwriteRegBut = Button(modbusFrame, text="Write Register", width=30, command=MBwriteReg)
MBwriteRegBut.grid(row=8, column=0, columnspan=2, sticky="ew", padx=5, pady=2)

# Output Response
Label(modbusFrame, text="Output Response:").grid(row=9, column=0, columnspan=2, sticky="w", padx=5, pady=(10,2))
MBoutputEntryField = Entry(modbusFrame, width=33, justify="center")
MBoutputEntryField.grid(row=10, column=0, columnspan=2, sticky="ew", padx=5, pady=2)

# ============================================================================
# Information Frame (Row 1, Column 0-2, spans 3 columns)
# ============================================================================
infoFrame = LabelFrame(tab4, text="Information", padding=10)
infoFrame.grid(row=1, column=0, columnspan=3, sticky="ew", padx=5, pady=5)

infoFrame.grid_columnconfigure(0, weight=1)

Label(infoFrame, text="The following IO are available when using the default 5v Nano board for IO:   Inputs = 2-7  /  Outputs = 8-13  /  Servos = A0-A7").grid(row=0, column=0, sticky="w", padx=5, pady=2)

Label(infoFrame, text="The following IO are available when using the default 5v Mega board for IO:   Inputs = 0-27  /  Outputs = 28-53  /  Servos = A0-A7").grid(row=1, column=0, sticky="w", padx=5, pady=2)

Label(infoFrame, text="Please review this tutorial video on using 5v IO boards:").grid(row=2, column=0, sticky="w", padx=5, pady=2)

link2 = Label(infoFrame, font=("Arial", 8), text="https://youtu.be/76F6dS4ar8Y?si=Z6NstZy1zNeHgtCF", foreground="blue", cursor="hand2")
link2.bind("<Button-1>", lambda event: webbrowser.open(link2.cget("text")))
link2.grid(row=3, column=0, sticky="w", padx=5, pady=2)

Label(infoFrame, text="5v board inputs are high impedance and susceptable to floating voltage - inputs use a pullup resistor and will read high when nothing is connected - its best to connect your input signal to GND and if/wait for the input signal to = 0").grid(row=4, column=0, sticky="w", padx=5, pady=2)





# ### 4 LABELS#################################################################
# #############################################################################
# 
# servo0onequalsLab = Label(tab4, text = "=")
# servo0onequalsLab.place(x=70, y=42)
# 
# servo0offequalsLab = Label(tab4, text = "=")
# servo0offequalsLab.place(x=70, y=82)
# 
# servo1onequalsLab = Label(tab4, text = "=")
# servo1onequalsLab.place(x=70, y=122)
# 
# servo1offequalsLab = Label(tab4, text = "=")
# servo1offequalsLab.place(x=70, y=162)
# 
# servo2onequalsLab = Label(tab4, text = "=")
# servo2onequalsLab.place(x=70, y=202)
# 
# servo2offequalsLab = Label(tab4, text = "=")
# servo2offequalsLab.place(x=70, y=242)
# 
# servo3onequalsLab = Label(tab4, text = "=")
# servo3onequalsLab.place(x=70, y=282)
# 
# servo3offequalsLab = Label(tab4, text = "=")
# servo3offequalsLab.place(x=70, y=322)
# 
# 
# 
# Do1onequalsLab = Label(tab4, text = "=")
# Do1onequalsLab.place(x=210, y=42)
# 
# Do1offequalsLab = Label(tab4, text = "=")
# Do1offequalsLab.place(x=210, y=82)
# 
# Do2onequalsLab = Label(tab4, text = "=")
# Do2onequalsLab.place(x=210, y=122)
# 
# Do2offequalsLab = Label(tab4, text = "=")
# Do2offequalsLab.place(x=210, y=162)
# 
# Do3onequalsLab = Label(tab4, text = "=")
# Do3onequalsLab.place(x=210, y=202)
# 
# Do3offequalsLab = Label(tab4, text = "=")
# Do3offequalsLab.place(x=210, y=242)
# 
# Do4onequalsLab = Label(tab4, text = "=")
# Do4onequalsLab.place(x=210, y=282)
# 
# Do4offequalsLab = Label(tab4, text = "=")
# Do4offequalsLab.place(x=210, y=322)
# 
# Do5onequalsLab = Label(tab4, text = "=")
# Do5onequalsLab.place(x=210, y=362)
# 
# Do5offequalsLab = Label(tab4, text = "=")
# Do5offequalsLab.place(x=210, y=402)
# 
# Do6onequalsLab = Label(tab4, text = "=")
# Do6onequalsLab.place(x=210, y=442)
# 
# Do6offequalsLab = Label(tab4, text = "=")
# Do6offequalsLab.place(x=210, y=482)
# 
# IOboardLab = Label(tab4, font=("Arial 10 bold"), text = "5v IO BOARD")
# IOboardLab.place(x=95, y=10)
# 
# AuxComLab = Label(tab4, font=("Arial 10 bold"), text = "AUX COM DEVICE")
# AuxComLab.place(x=400, y=10)
# 
# ModbusLab = Label(tab4, font=("Arial 10 bold"), text = "MODBUS DEVICE")
# ModbusLab.place(x=700, y=10)
# 
# AuxPortNumLab= Label(tab4, text = "Aux Com Port")
# AuxPortNumLab.place(x=440, y=42)
# 
# AuxPortCharLab= Label(tab4, text = "Char to Read")
# AuxPortCharLab.place(x=440, y=82)
# 
# MBslaveLab= Label(tab4, text = "Slave ID")
# MBslaveLab.place(x=750, y=42)
# 
# MBaddressLab= Label(tab4, text = "Modbus Address")
# MBaddressLab.place(x=750, y=82)
# 
# MBwriteLab= Label(tab4, text = "Operation Value")
# MBwriteLab.place(x=750, y=122)
# 
# MBoutputLab= Label(tab4, text = "Output Response:")
# MBoutputLab.place(x=662, y=405)
# 
# 
# 
# inoutavailLab = Label(tab4, text = "The following IO are available when using the default 5v Nano board for IO:   Inputs = 2-7  /  Outputs = 8-13  /  Servos = A0-A7")
# inoutavailLab.place(x=10, y=640)
# 
# inoutavailLab = Label(tab4, text = "The following IO are available when using the default 5v Mega board for IO:   Inputs = 0-27  /  Outputs = 28-53  /  Servos = A0-A7")
# inoutavailLab.place(x=10, y=655)
# 
# inoutavailLab = Label(tab4, text = "Please review this tutorial video on using 5v IO boards:")
# inoutavailLab.place(x=10, y=670)
# 
# inoutavailLab = Label(tab4, text = "5v board inputs are high impedance and susceptable to floating voltage - inputs use a pullup resistor and will read high when nothing is connected - its best to connect your input signal to GND and if/wait for the input signal to = 0")
# inoutavailLab.place(x=10, y=685)
# 
# 
# 
# link2 = Label(tab4, font=("Arial", 8), text="https://youtu.be/76F6dS4ar8Y?si=Z6NstZy1zNeHgtCF", foreground="blue", cursor="hand2")
# link2.bind("<Button-1>", lambda event: webbrowser.open(link2.cget("text")))
# link2.place(x=300, y=671)
# 
# 
# 
# ### 4 BUTTONS################################################################
# #############################################################################
# 
# servo0onBut = Button(tab4,  text="Servo 0",  command = Servo0on)
# servo0onBut.place(x=10, y=40)
# 
# servo0offBut = Button(tab4,  text="Servo 0",  command = Servo0off)
# servo0offBut.place(x=10, y=80)
# 
# servo1onBut = Button(tab4,  text="Servo 1",  command = Servo1on)
# servo1onBut.place(x=10, y=120)
# 
# servo1offBut = Button(tab4,  text="Servo 1",  command = Servo1off)
# servo1offBut.place(x=10, y=160)
# 
# servo2onBut = Button(tab4,  text="Servo 2",  command = Servo2on)
# servo2onBut.place(x=10, y=200)
# 
# servo2offBut = Button(tab4,  text="Servo 2",  command = Servo2off)
# servo2offBut.place(x=10, y=240)
# 
# servo3onBut = Button(tab4,  text="Servo 3",  command = Servo3on)
# servo3onBut.place(x=10, y=280)
# 
# servo3offBut = Button(tab4,  text="Servo 3",  command = Servo3off)
# servo3offBut.place(x=10, y=320)
# 
# 
# 
# 
# 
# DO1onBut = Button(tab4,  text="DO on",  command = DO1on)
# DO1onBut.place(x=150, y=40)
# 
# DO1offBut = Button(tab4,  text="DO off",  command = DO1off)
# DO1offBut.place(x=150, y=80)
# 
# DO2onBut = Button(tab4,  text="DO on",  command = DO2on)
# DO2onBut.place(x=150, y=120)
# 
# DO2offBut = Button(tab4,  text="DO off",  command = DO2off)
# DO2offBut.place(x=150, y=160)
# 
# DO3onBut = Button(tab4,  text="DO on",  command = DO3on)
# DO3onBut.place(x=150, y=200)
# 
# DO3offBut = Button(tab4,  text="DO off",  command = DO3off)
# DO3offBut.place(x=150, y=240)
# 
# DO4onBut = Button(tab4,  text="DO on",  command = DO4on)
# DO4onBut.place(x=150, y=280)
# 
# DO4offBut = Button(tab4,  text="DO off",  command = DO4off)
# DO4offBut.place(x=150, y=320)
# 
# DO5onBut = Button(tab4,  text="DO on",  command = DO5on)
# DO5onBut.place(x=150, y=360)
# 
# DO5offBut = Button(tab4,  text="DO off",  command = DO5off)
# DO5offBut.place(x=150, y=400)
# 
# DO6onBut = Button(tab4,  text="DO on",  command = DO6on)
# DO6onBut.place(x=150, y=440)
# 
# DO6offBut = Button(tab4,  text="DO off",  command = DO6off)
# DO6offBut.place(x=150, y=480)
# 
# 
# comPortBut3 = Button(tab4,  text="Test Aux COM Device",   command = TestAuxCom)
# comPortBut3.place(x=395, y=120)
# 
# MBreadCoilBut = Button(tab4,  text="Read Coil", width=30, command = MBreadCoil)
# MBreadCoilBut.place(x=665, y=160)
# 
# MBreadDinputBut = Button(tab4,  text="Read Discrete Input", width=30, command = MBreadInput)
# MBreadDinputBut.place(x=665, y=200)
# 
# MBreadHoldRegBut = Button(tab4,  text="Read Holding Register", width=30, command = MBreadHoldReg)
# MBreadHoldRegBut.place(x=665, y=240)
# 
# MBreadInputRegBut = Button(tab4,  text="Read Input Register", width=30, command = MBreadInputReg)
# MBreadInputRegBut.place(x=665, y=280)
# 
# MBwriteCoilBut = Button(tab4,  text="Write Coil", width=30, command = MBwriteCoil)
# MBwriteCoilBut.place(x=665, y=320)
# 
# MBwriteRegBut = Button(tab4,  text="Write Register", width=30, command = MBwriteReg)
# MBwriteRegBut.place(x=665, y=360)
# 
# 
# 
# 
# 
# #### 4 ENTRY FIELDS##########################################################
# #############################################################################
# 
# 
# servo0onEntryField = Entry(tab4,width=4,justify="center")
# servo0onEntryField.place(x=90, y=45)
# 
# servo0offEntryField = Entry(tab4,width=4,justify="center")
# servo0offEntryField.place(x=90, y=85)
# 
# servo1onEntryField = Entry(tab4,width=4,justify="center")
# servo1onEntryField.place(x=90, y=125)
# 
# servo1offEntryField = Entry(tab4,width=4,justify="center")
# servo1offEntryField.place(x=90, y=165)
# 
# servo2onEntryField = Entry(tab4,width=4,justify="center")
# servo2onEntryField.place(x=90, y=205)
# 
# servo2offEntryField = Entry(tab4,width=4,justify="center")
# servo2offEntryField.place(x=90, y=245)
# 
# 
# servo3onEntryField = Entry(tab4,width=4,justify="center")
# servo3onEntryField.place(x=90, y=285)
# 
# servo3offEntryField = Entry(tab4,width=4,justify="center")
# servo3offEntryField.place(x=90, y=325)
# 
# 
# 
# 
# 
# DO1onEntryField = Entry(tab4,width=4,justify="center")
# DO1onEntryField.place(x=230, y=45)
# 
# DO1offEntryField = Entry(tab4,width=4,justify="center")
# DO1offEntryField.place(x=230, y=85)
# 
# DO2onEntryField = Entry(tab4,width=4,justify="center")
# DO2onEntryField.place(x=230, y=125)
# 
# DO2offEntryField = Entry(tab4,width=4,justify="center")
# DO2offEntryField.place(x=230, y=165)
# 
# DO3onEntryField = Entry(tab4,width=4,justify="center")
# DO3onEntryField.place(x=230, y=205)
# 
# DO3offEntryField = Entry(tab4,width=4,justify="center")
# DO3offEntryField.place(x=230, y=245)
# 
# DO4onEntryField = Entry(tab4,width=4,justify="center")
# DO4onEntryField.place(x=230, y=285)
# 
# DO4offEntryField = Entry(tab4,width=4,justify="center")
# DO4offEntryField.place(x=230, y=325)
# 
# DO5onEntryField = Entry(tab4,width=4,justify="center")
# DO5onEntryField.place(x=230, y=365)
# 
# DO5offEntryField = Entry(tab4,width=4,justify="center")
# DO5offEntryField.place(x=230, y=405)
# 
# DO6onEntryField = Entry(tab4,width=4,justify="center")
# DO6onEntryField.place(x=230, y=445)
# 
# DO6offEntryField = Entry(tab4,width=4,justify="center")
# DO6offEntryField.place(x=230, y=485)
# 
# 
# 
# com3PortEntryField = Entry(tab4,width=4,justify="center")
# com3PortEntryField.place(x=400, y=40)
# 
# com3charPortEntryField = Entry(tab4,width=4,justify="center")
# com3charPortEntryField.place(x=400, y=80)
# 
# com3outPortEntryField = Entry(tab4,width=25,justify="center")
# com3outPortEntryField.place(x=385, y=160)
# 
# 
# MBslaveEntryField = Entry(tab4,width=4,justify="center")
# MBslaveEntryField.place(x=710, y=40)
# 
# MBaddressEntryField = Entry(tab4,width=5,justify="center")
# MBaddressEntryField.place(x=690, y=80)
# 
# MBoperValEntryField = Entry(tab4,width=5,justify="center")
# MBoperValEntryField.place(x=690, y=120)
# 
# MBoutputEntryField = Entry(tab4,width=33,justify="center")
# MBoutputEntryField.place(x=662, y=425)
# 
# 
# 
# ####################################################################################################################################################
# ####################################################################################################################################################
# ####################################################################################################################################################
####TAB 5

# ============================================================================
# Tab 5 Grid Layout Configuration
# ============================================================================
tab5.grid_rowconfigure(0, weight=1)
tab5.grid_columnconfigure(0, weight=0, minsize=150)  # Registers
tab5.grid_columnconfigure(1, weight=0, minsize=300)  # Position Registers

# ============================================================================
# Registers Container (Column 0)
# ============================================================================
registersFrame = LabelFrame(tab5, text="Registers", padding=10)
registersFrame.grid(row=0, column=0, sticky="nsew", padx=5, pady=5)

registersFrame.grid_columnconfigure(0, weight=0)  # Entry field column
registersFrame.grid_columnconfigure(1, weight=1)  # Label column

# R1
R1EntryField = Entry(registersFrame, width=4, justify="center")
R1EntryField.grid(row=0, column=0, padx=2, pady=2)
R1Lab = Label(registersFrame, text="R1")
R1Lab.grid(row=0, column=1, sticky="w", padx=5, pady=2)

# R2
R2EntryField = Entry(registersFrame, width=4, justify="center")
R2EntryField.grid(row=1, column=0, padx=2, pady=2)
R2Lab = Label(registersFrame, text="R2")
R2Lab.grid(row=1, column=1, sticky="w", padx=5, pady=2)

# R3
R3EntryField = Entry(registersFrame, width=4, justify="center")
R3EntryField.grid(row=2, column=0, padx=2, pady=2)
R3Lab = Label(registersFrame, text="R3")
R3Lab.grid(row=2, column=1, sticky="w", padx=5, pady=2)

# R4
R4EntryField = Entry(registersFrame, width=4, justify="center")
R4EntryField.grid(row=3, column=0, padx=2, pady=2)
R4Lab = Label(registersFrame, text="R4")
R4Lab.grid(row=3, column=1, sticky="w", padx=5, pady=2)

# R5
R5EntryField = Entry(registersFrame, width=4, justify="center")
R5EntryField.grid(row=4, column=0, padx=2, pady=2)
R5Lab = Label(registersFrame, text="R5")
R5Lab.grid(row=4, column=1, sticky="w", padx=5, pady=2)

# R6
R6EntryField = Entry(registersFrame, width=4, justify="center")
R6EntryField.grid(row=5, column=0, padx=2, pady=2)
R6Lab = Label(registersFrame, text="R6")
R6Lab.grid(row=5, column=1, sticky="w", padx=5, pady=2)

# R7
R7EntryField = Entry(registersFrame, width=4, justify="center")
R7EntryField.grid(row=6, column=0, padx=2, pady=2)
R7Lab = Label(registersFrame, text="R7")
R7Lab.grid(row=6, column=1, sticky="w", padx=5, pady=2)

# R8
R8EntryField = Entry(registersFrame, width=4, justify="center")
R8EntryField.grid(row=7, column=0, padx=2, pady=2)
R8Lab = Label(registersFrame, text="R8")
R8Lab.grid(row=7, column=1, sticky="w", padx=5, pady=2)

# R9
R9EntryField = Entry(registersFrame, width=4, justify="center")
R9EntryField.grid(row=8, column=0, padx=2, pady=2)
R9Lab = Label(registersFrame, text="R9")
R9Lab.grid(row=8, column=1, sticky="w", padx=5, pady=2)

# R10
R10EntryField = Entry(registersFrame, width=4, justify="center")
R10EntryField.grid(row=9, column=0, padx=2, pady=2)
R10Lab = Label(registersFrame, text="R10")
R10Lab.grid(row=9, column=1, sticky="w", padx=5, pady=2)

# R11
R11EntryField = Entry(registersFrame, width=4, justify="center")
R11EntryField.grid(row=10, column=0, padx=2, pady=2)
R11Lab = Label(registersFrame, text="R11")
R11Lab.grid(row=10, column=1, sticky="w", padx=5, pady=2)

# R12
R12EntryField = Entry(registersFrame, width=4, justify="center")
R12EntryField.grid(row=11, column=0, padx=2, pady=2)
R12Lab = Label(registersFrame, text="R12")
R12Lab.grid(row=11, column=1, sticky="w", padx=5, pady=2)

# R13
R13EntryField = Entry(registersFrame, width=4, justify="center")
R13EntryField.grid(row=12, column=0, padx=2, pady=2)
R13Lab = Label(registersFrame, text="R13")
R13Lab.grid(row=12, column=1, sticky="w", padx=5, pady=2)

# R14
R14EntryField = Entry(registersFrame, width=4, justify="center")
R14EntryField.grid(row=13, column=0, padx=2, pady=2)
R14Lab = Label(registersFrame, text="R14")
R14Lab.grid(row=13, column=1, sticky="w", padx=5, pady=2)

# R15
R15EntryField = Entry(registersFrame, width=4, justify="center")
R15EntryField.grid(row=14, column=0, padx=2, pady=2)
R15Lab = Label(registersFrame, text="R15")
R15Lab.grid(row=14, column=1, sticky="w", padx=5, pady=2)

# R16
R16EntryField = Entry(registersFrame, width=4, justify="center")
R16EntryField.grid(row=15, column=0, padx=2, pady=2)
R16Lab = Label(registersFrame, text="R16")
R16Lab.grid(row=15, column=1, sticky="w", padx=5, pady=2)

# ============================================================================
# Position Registers Container (Column 1)
# ============================================================================
posRegistersFrame = LabelFrame(tab5, text="Position Registers", padding=10)
posRegistersFrame.grid(row=0, column=1, sticky="nsew", padx=5, pady=5)

posRegistersFrame.grid_columnconfigure(0, weight=0, minsize=35)  # X
posRegistersFrame.grid_columnconfigure(1, weight=0, minsize=35)  # Y
posRegistersFrame.grid_columnconfigure(2, weight=0, minsize=35)  # Z
posRegistersFrame.grid_columnconfigure(3, weight=0, minsize=35)  # Rz
posRegistersFrame.grid_columnconfigure(4, weight=0, minsize=35)  # Ry
posRegistersFrame.grid_columnconfigure(5, weight=0, minsize=35)  # Rx
posRegistersFrame.grid_columnconfigure(6, weight=0, minsize=40)  # PR label

# Header row
Label(posRegistersFrame, text="X").grid(row=0, column=0, padx=1, pady=2)
Label(posRegistersFrame, text="Y").grid(row=0, column=1, padx=1, pady=2)
Label(posRegistersFrame, text="Z").grid(row=0, column=2, padx=1, pady=2)
Label(posRegistersFrame, text="Rz").grid(row=0, column=3, padx=1, pady=2)
Label(posRegistersFrame, text="Ry").grid(row=0, column=4, padx=1, pady=2)
Label(posRegistersFrame, text="Rx").grid(row=0, column=5, padx=1, pady=2)

# PR1
SP_1_E1_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_1_E1_EntryField.grid(row=1, column=0, padx=1, pady=2)
SP_1_E2_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_1_E2_EntryField.grid(row=1, column=1, padx=1, pady=2)
SP_1_E3_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_1_E3_EntryField.grid(row=1, column=2, padx=1, pady=2)
SP_1_E4_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_1_E4_EntryField.grid(row=1, column=3, padx=1, pady=2)
SP_1_E5_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_1_E5_EntryField.grid(row=1, column=4, padx=1, pady=2)
SP_1_E6_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_1_E6_EntryField.grid(row=1, column=5, padx=1, pady=2)
SP1Lab = Label(posRegistersFrame, text="PR1")
SP1Lab.grid(row=1, column=6, sticky="w", padx=2, pady=2)

# PR2
SP_2_E1_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_2_E1_EntryField.grid(row=2, column=0, padx=1, pady=2)
SP_2_E2_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_2_E2_EntryField.grid(row=2, column=1, padx=1, pady=2)
SP_2_E3_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_2_E3_EntryField.grid(row=2, column=2, padx=1, pady=2)
SP_2_E4_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_2_E4_EntryField.grid(row=2, column=3, padx=1, pady=2)
SP_2_E5_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_2_E5_EntryField.grid(row=2, column=4, padx=1, pady=2)
SP_2_E6_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_2_E6_EntryField.grid(row=2, column=5, padx=1, pady=2)
SP2Lab = Label(posRegistersFrame, text="PR2")
SP2Lab.grid(row=2, column=6, sticky="w", padx=2, pady=2)

# PR3
SP_3_E1_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_3_E1_EntryField.grid(row=3, column=0, padx=1, pady=2)
SP_3_E2_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_3_E2_EntryField.grid(row=3, column=1, padx=1, pady=2)
SP_3_E3_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_3_E3_EntryField.grid(row=3, column=2, padx=1, pady=2)
SP_3_E4_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_3_E4_EntryField.grid(row=3, column=3, padx=1, pady=2)
SP_3_E5_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_3_E5_EntryField.grid(row=3, column=4, padx=1, pady=2)
SP_3_E6_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_3_E6_EntryField.grid(row=3, column=5, padx=1, pady=2)
SP3Lab = Label(posRegistersFrame, text="PR3")
SP3Lab.grid(row=3, column=6, sticky="w", padx=2, pady=2)

# PR4
SP_4_E1_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_4_E1_EntryField.grid(row=4, column=0, padx=1, pady=2)
SP_4_E2_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_4_E2_EntryField.grid(row=4, column=1, padx=1, pady=2)
SP_4_E3_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_4_E3_EntryField.grid(row=4, column=2, padx=1, pady=2)
SP_4_E4_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_4_E4_EntryField.grid(row=4, column=3, padx=1, pady=2)
SP_4_E5_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_4_E5_EntryField.grid(row=4, column=4, padx=1, pady=2)
SP_4_E6_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_4_E6_EntryField.grid(row=4, column=5, padx=1, pady=2)
SP4Lab = Label(posRegistersFrame, text="PR4")
SP4Lab.grid(row=4, column=6, sticky="w", padx=2, pady=2)

# PR5
SP_5_E1_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_5_E1_EntryField.grid(row=5, column=0, padx=1, pady=2)
SP_5_E2_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_5_E2_EntryField.grid(row=5, column=1, padx=1, pady=2)
SP_5_E3_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_5_E3_EntryField.grid(row=5, column=2, padx=1, pady=2)
SP_5_E4_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_5_E4_EntryField.grid(row=5, column=3, padx=1, pady=2)
SP_5_E5_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_5_E5_EntryField.grid(row=5, column=4, padx=1, pady=2)
SP_5_E6_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_5_E6_EntryField.grid(row=5, column=5, padx=1, pady=2)
SP5Lab = Label(posRegistersFrame, text="PR5")
SP5Lab.grid(row=5, column=6, sticky="w", padx=2, pady=2)

# PR6
SP_6_E1_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_6_E1_EntryField.grid(row=6, column=0, padx=1, pady=2)
SP_6_E2_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_6_E2_EntryField.grid(row=6, column=1, padx=1, pady=2)
SP_6_E3_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_6_E3_EntryField.grid(row=6, column=2, padx=1, pady=2)
SP_6_E4_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_6_E4_EntryField.grid(row=6, column=3, padx=1, pady=2)
SP_6_E5_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_6_E5_EntryField.grid(row=6, column=4, padx=1, pady=2)
SP_6_E6_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_6_E6_EntryField.grid(row=6, column=5, padx=1, pady=2)
SP6Lab = Label(posRegistersFrame, text="PR6")
SP6Lab.grid(row=6, column=6, sticky="w", padx=2, pady=2)

# PR7
SP_7_E1_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_7_E1_EntryField.grid(row=7, column=0, padx=1, pady=2)
SP_7_E2_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_7_E2_EntryField.grid(row=7, column=1, padx=1, pady=2)
SP_7_E3_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_7_E3_EntryField.grid(row=7, column=2, padx=1, pady=2)
SP_7_E4_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_7_E4_EntryField.grid(row=7, column=3, padx=1, pady=2)
SP_7_E5_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_7_E5_EntryField.grid(row=7, column=4, padx=1, pady=2)
SP_7_E6_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_7_E6_EntryField.grid(row=7, column=5, padx=1, pady=2)
SP7Lab = Label(posRegistersFrame, text="PR7")
SP7Lab.grid(row=7, column=6, sticky="w", padx=2, pady=2)

# PR8
SP_8_E1_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_8_E1_EntryField.grid(row=8, column=0, padx=1, pady=2)
SP_8_E2_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_8_E2_EntryField.grid(row=8, column=1, padx=1, pady=2)
SP_8_E3_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_8_E3_EntryField.grid(row=8, column=2, padx=1, pady=2)
SP_8_E4_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_8_E4_EntryField.grid(row=8, column=3, padx=1, pady=2)
SP_8_E5_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_8_E5_EntryField.grid(row=8, column=4, padx=1, pady=2)
SP_8_E6_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_8_E6_EntryField.grid(row=8, column=5, padx=1, pady=2)
SP8Lab = Label(posRegistersFrame, text="PR8")
SP8Lab.grid(row=8, column=6, sticky="w", padx=2, pady=2)

# PR9
SP_9_E1_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_9_E1_EntryField.grid(row=9, column=0, padx=1, pady=2)
SP_9_E2_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_9_E2_EntryField.grid(row=9, column=1, padx=1, pady=2)
SP_9_E3_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_9_E3_EntryField.grid(row=9, column=2, padx=1, pady=2)
SP_9_E4_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_9_E4_EntryField.grid(row=9, column=3, padx=1, pady=2)
SP_9_E5_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_9_E5_EntryField.grid(row=9, column=4, padx=1, pady=2)
SP_9_E6_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_9_E6_EntryField.grid(row=9, column=5, padx=1, pady=2)
SP9Lab = Label(posRegistersFrame, text="PR9")
SP9Lab.grid(row=9, column=6, sticky="w", padx=2, pady=2)

# PR10
SP_10_E1_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_10_E1_EntryField.grid(row=10, column=0, padx=1, pady=2)
SP_10_E2_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_10_E2_EntryField.grid(row=10, column=1, padx=1, pady=2)
SP_10_E3_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_10_E3_EntryField.grid(row=10, column=2, padx=1, pady=2)
SP_10_E4_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_10_E4_EntryField.grid(row=10, column=3, padx=1, pady=2)
SP_10_E5_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_10_E5_EntryField.grid(row=10, column=4, padx=1, pady=2)
SP_10_E6_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_10_E6_EntryField.grid(row=10, column=5, padx=1, pady=2)
SP10Lab = Label(posRegistersFrame, text="PR10")
SP10Lab.grid(row=10, column=6, sticky="w", padx=2, pady=2)

# PR11
SP_11_E1_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_11_E1_EntryField.grid(row=11, column=0, padx=1, pady=2)
SP_11_E2_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_11_E2_EntryField.grid(row=11, column=1, padx=1, pady=2)
SP_11_E3_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_11_E3_EntryField.grid(row=11, column=2, padx=1, pady=2)
SP_11_E4_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_11_E4_EntryField.grid(row=11, column=3, padx=1, pady=2)
SP_11_E5_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_11_E5_EntryField.grid(row=11, column=4, padx=1, pady=2)
SP_11_E6_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_11_E6_EntryField.grid(row=11, column=5, padx=1, pady=2)
SP11Lab = Label(posRegistersFrame, text="PR11")
SP11Lab.grid(row=11, column=6, sticky="w", padx=2, pady=2)

# PR12
SP_12_E1_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_12_E1_EntryField.grid(row=12, column=0, padx=1, pady=2)
SP_12_E2_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_12_E2_EntryField.grid(row=12, column=1, padx=1, pady=2)
SP_12_E3_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_12_E3_EntryField.grid(row=12, column=2, padx=1, pady=2)
SP_12_E4_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_12_E4_EntryField.grid(row=12, column=3, padx=1, pady=2)
SP_12_E5_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_12_E5_EntryField.grid(row=12, column=4, padx=1, pady=2)
SP_12_E6_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_12_E6_EntryField.grid(row=12, column=5, padx=1, pady=2)
SP12Lab = Label(posRegistersFrame, text="PR12")
SP12Lab.grid(row=12, column=6, sticky="w", padx=2, pady=2)

# PR13
SP_13_E1_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_13_E1_EntryField.grid(row=13, column=0, padx=1, pady=2)
SP_13_E2_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_13_E2_EntryField.grid(row=13, column=1, padx=1, pady=2)
SP_13_E3_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_13_E3_EntryField.grid(row=13, column=2, padx=1, pady=2)
SP_13_E4_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_13_E4_EntryField.grid(row=13, column=3, padx=1, pady=2)
SP_13_E5_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_13_E5_EntryField.grid(row=13, column=4, padx=1, pady=2)
SP_13_E6_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_13_E6_EntryField.grid(row=13, column=5, padx=1, pady=2)
SP13Lab = Label(posRegistersFrame, text="PR13")
SP13Lab.grid(row=13, column=6, sticky="w", padx=2, pady=2)

# PR14
SP_14_E1_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_14_E1_EntryField.grid(row=14, column=0, padx=1, pady=2)
SP_14_E2_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_14_E2_EntryField.grid(row=14, column=1, padx=1, pady=2)
SP_14_E3_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_14_E3_EntryField.grid(row=14, column=2, padx=1, pady=2)
SP_14_E4_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_14_E4_EntryField.grid(row=14, column=3, padx=1, pady=2)
SP_14_E5_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_14_E5_EntryField.grid(row=14, column=4, padx=1, pady=2)
SP_14_E6_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_14_E6_EntryField.grid(row=14, column=5, padx=1, pady=2)
SP14Lab = Label(posRegistersFrame, text="PR14")
SP14Lab.grid(row=14, column=6, sticky="w", padx=2, pady=2)

# PR15
SP_15_E1_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_15_E1_EntryField.grid(row=15, column=0, padx=1, pady=2)
SP_15_E2_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_15_E2_EntryField.grid(row=15, column=1, padx=1, pady=2)
SP_15_E3_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_15_E3_EntryField.grid(row=15, column=2, padx=1, pady=2)
SP_15_E4_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_15_E4_EntryField.grid(row=15, column=3, padx=1, pady=2)
SP_15_E5_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_15_E5_EntryField.grid(row=15, column=4, padx=1, pady=2)
SP_15_E6_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_15_E6_EntryField.grid(row=15, column=5, padx=1, pady=2)
SP15Lab = Label(posRegistersFrame, text="PR15")
SP15Lab.grid(row=15, column=6, sticky="w", padx=2, pady=2)

# PR16
SP_16_E1_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_16_E1_EntryField.grid(row=16, column=0, padx=1, pady=2)
SP_16_E2_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_16_E2_EntryField.grid(row=16, column=1, padx=1, pady=2)
SP_16_E3_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_16_E3_EntryField.grid(row=16, column=2, padx=1, pady=2)
SP_16_E4_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_16_E4_EntryField.grid(row=16, column=3, padx=1, pady=2)
SP_16_E5_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_16_E5_EntryField.grid(row=16, column=4, padx=1, pady=2)
SP_16_E6_EntryField = Entry(posRegistersFrame, width=4, justify="center")
SP_16_E6_EntryField.grid(row=16, column=5, padx=1, pady=2)
SP16Lab = Label(posRegistersFrame, text="PR16")
SP16Lab.grid(row=16, column=6, sticky="w", padx=2, pady=2)


####TAB 6




### 6 LABELS#################################################################
#############################################################################


#VisBackdropImg = ImageTk.PhotoImage(Image.open('VisBackdrop.png'))

VisBackdropImg = Image.open("VisBackdrop.png")
VisBackdropTk = ImageTk.PhotoImage(VisBackdropImg)
VisBackdromLbl = Label(tab6, image = VisBackdropTk)
VisBackdromLbl.place(x=15, y=215)


#RUN['cam_on']= cv2.VideoCapture(0)
video_frame = Frame(tab6,width=640,height=480)
video_frame.place(x=50, y=250)


vid_lbl = Label(video_frame)
vid_lbl.place(x=0, y=0)

vid_lbl.bind('<Button-1>', motion)


LiveLab = Label(tab6, text = "LIVE VIDEO FEED")
LiveLab.place(x=750, y=390)
liveCanvas = Canvas(tab6, width=490, height=330)
liveCanvas.place(x=750, y=410)
live_frame = Frame(tab6,width=480,height=320)
live_frame.place(x=757, y=417)
live_lbl = Label(live_frame)
live_lbl.place(x=0, y=0)


template_frame = Frame(tab6,width=150,height=150)
template_frame.place(x=565, y=50)

template_lbl = Label(template_frame)
template_lbl.place(x=0, y=0)

FoundValuesLab = Label(tab6, text = "FOUND VALUES")
FoundValuesLab.place(x=750, y=30)

CalValuesLab = Label(tab6, text = "CALIBRATION VALUES")
CalValuesLab.place(x=900, y=30)





### 6 BUTTONS################################################################
#############################################################################

match CE['Platform']['OS']:

  case "Windows":
    graph = FilterGraph()
    try:
      allcams = graph.get_input_devices()
    except Exception:
      allcams = ["Select a Camera"]

    # --- filter out virtual/fake cameras (case-insensitive) ---
    _ban = ("iriun", "droidcam", "obs", "virtual", "manycam", "snap camera", "ndi", "xsplit", "mmhmm")
    camList = [n for n in allcams if not any(b in n.lower() for b in _ban)]
    if not camList:
      camList = allcams[:]  # fallback if everything got filtered
    # ----------------------------------------------------------

    visoptions = StringVar(tab6)
    visoptions.set("Select a Camera")

    try:
      # If we have real cams, preselect the first real one; otherwise keep the placeholder
      if camList and camList[0] != "Select a Camera":
        vismenu = OptionMenu(tab6, visoptions, camList[0], *camList)
        visoptions.set(camList[0])  # ensures the real cam (e.g., Logi C270) is selected
      else:
        vismenu = OptionMenu(tab6, visoptions, "Select a Camera")
      vismenu.config(width=20)
      vismenu.place(x=10, y=10)
    except Exception:
      logger.error("no camera")

  case "Linux":
    # Build label->id map
    label_to_id = {c.label: c.id for c in CE['Cameras']['Enum']}
    camList = list(label_to_id.keys())

    selected_label = tk.StringVar(value=(camList[0] if camList else "Select a Camera"))

    visoptions = StringVar(tab6)
    visoptions.set("Select a Camera")

    def on_camera_select(chosen_label):
      cam_id = label_to_id.get(chosen_label, "None")
      logger.debug("Debug - User picked:", chosen_label, " -> using id:", cam_id)

    try:
      vismenu = OptionMenu(tab6, visoptions, selected_label.get(), *camList, command=on_camera_select)
      vismenu.config(width=20)
      vismenu.place(x=10, y=10)
    except Exception:
      logger.error("no camera")





StartCamBut = Button(tab6,  text="Start Camera",  width=12, command = start_vid)
StartCamBut.place(x=200, y=10)

StopCamBut = Button(tab6,  text="Stop Camera",  width=12, command = stop_vid)
StopCamBut.place(x=315, y=10)

CapImgBut = Button(tab6,  text="Snap Image",  width=12, command = take_pic)
CapImgBut.place(x=10, y=50)

TeachImgBut = Button(tab6,  text="Teach Object",  width=12, command = selectTemplate)
TeachImgBut.place(x=140, y=50)

FindVisBut = Button(tab6,  text="Snap & Find",  width=12, command = snapFind)
FindVisBut.place(x=270, y=50)


ZeroBrCnBut = Button(tab6, text="Zero",  width=5, command = zeroBrCn)
ZeroBrCnBut.place(x=10, y=110)

maskBut = Button(tab6, text="Mask",  width=5, command = selectMask)
maskBut.place(x=10, y=150)







VisZoomSlide = Scale(tab6, from_=50, to=1,  length=250, orient=HORIZONTAL)
VisZoomSlide.bind("<ButtonRelease-1>", VisUpdateBriCon)
VisZoomSlide.place(x=75, y=95)
VisZoomSlide.set(50)

VisZoomLab = Label(tab6, text = "Zoom")
VisZoomLab.place(x=75, y=110)

VisBrightSlide = Scale(tab6, from_=-127, to=127,  length=250, orient=HORIZONTAL)
VisBrightSlide.bind("<ButtonRelease-1>", VisUpdateBriCon)
VisBrightSlide.place(x=75, y=130)

VisBrightLab = Label(tab6, text = "Brightness")
VisBrightLab.place(x=75, y=145)

VisContrastSlide = Scale(tab6, from_=-127, to=127,  length=250, orient=HORIZONTAL)
VisContrastSlide.bind("<ButtonRelease-1>", VisUpdateBriCon)
VisContrastSlide.place(x=75, y=165)

VisContrastLab = Label(tab6, text = "Contrast")
VisContrastLab.place(x=75, y=180)


fullRotCbut = Checkbutton(tab6, text="Full Rotation Search",variable = RUN['fullRot'])
fullRotCbut.place(x=900, y=255)

pick180Cbut = Checkbutton(tab6, text="Pick Closest 180°",variable = RUN['pick180'])
pick180Cbut.place(x=900, y=275)

pickClosestCbut = Checkbutton(tab6, text="Try Closest When Out of Range",variable = RUN['pickClosest'])
pickClosestCbut.place(x=900, y=295)





saveCalBut = Button(tab6,  text="SAVE VISION DATA",  width=26, command = SaveAndApplyCalibration)
saveCalBut.place(x=915, y=340)





#### 6 ENTRY FIELDS##########################################################
#############################################################################

VisSelObjLab = Label(tab6, text = "Select Object")
VisSelObjLab.place(x=390, y=82)

VisBacColorEntryField = Entry(tab6,width=14,justify="center")
VisBacColorEntryField.place(x=390, y=115)
VisBacColorLab = Label(tab6, text = "Background Color")
VisBacColorLab.place(x=390, y=135)

bgAutoCbut = Checkbutton(tab6, command=checkAutoBG, text="Auto",variable = RUN['autoBG'])
bgAutoCbut.place(x=490, y=116)

VisScoreEntryField = Entry(tab6,width=12,justify="center")
VisScoreEntryField.place(x=390, y=165)
VisScoreLab = Label(tab6, text = "Score Threshold")
VisScoreLab.place(x=390, y=185)




VisRetScoreEntryField = Entry(tab6,width=12,justify="center")
VisRetScoreEntryField.place(x=750, y=55)
VisRetScoreLab = Label(tab6, text = "Scored Value")
VisRetScoreLab.place(x=750, y=75)

VisRetAngleEntryField = Entry(tab6,width=12,justify="center")
VisRetAngleEntryField.place(x=750, y=105)
VisRetAngleLab = Label(tab6, text = "Found Angle")
VisRetAngleLab.place(x=750, y=125)

VisRetXpixEntryField = Entry(tab6,width=12,justify="center")
VisRetXpixEntryField.place(x=750, y=155)
VisRetXpixLab = Label(tab6, text = "Pixel X Position")
VisRetXpixLab.place(x=750, y=175)

VisRetYpixEntryField = Entry(tab6,width=12,justify="center")
VisRetYpixEntryField.place(x=750, y=205)
VisRetYpixLab = Label(tab6, text = "Pixel Y Position")
VisRetYpixLab.place(x=750, y=225)

VisRetXrobEntryField = Entry(tab6,width=12,justify="center")
VisRetXrobEntryField.place(x=750, y=255)
VisRetXrobLab = Label(tab6, text = "Robot X Position")
VisRetXrobLab.place(x=750, y=275)

VisRetYrobEntryField = Entry(tab6,width=12,justify="center")
VisRetYrobEntryField.place(x=750, y=305)
VisRetYrobLab = Label(tab6, text = "Robot Y Position")
VisRetYrobLab.place(x=750, y=325)







VisX1PixEntryField = Entry(tab6,width=12,justify="center")
VisX1PixEntryField.place(x=900, y=55)
VisX1PixLab = Label(tab6, text = "X1 Pixel Pos")
VisX1PixLab.place(x=900, y=75)

VisY1PixEntryField = Entry(tab6,width=12,justify="center")
VisY1PixEntryField.place(x=900, y=105)
VisY1PixLab = Label(tab6, text = "Y1 Pixel Pos")
VisY1PixLab.place(x=900, y=125)

VisX2PixEntryField = Entry(tab6,width=12,justify="center")
VisX2PixEntryField.place(x=900, y=155)
VisX2PixLab = Label(tab6, text = "X2 Pixel Pos")
VisX2PixLab.place(x=900, y=175)

VisY2PixEntryField = Entry(tab6,width=12,justify="center")
VisY2PixEntryField.place(x=900, y=205)
VisY2PixLab = Label(tab6, text = "Y2 Pixel Pos")
VisY2PixLab.place(x=900, y=225)


VisX1RobEntryField = Entry(tab6,width=12,justify="center")
VisX1RobEntryField.place(x=1010, y=55)
VisX1RobLab = Label(tab6, text = "X1 Robot Pos")
VisX1RobLab.place(x=1010, y=75)

VisY1RobEntryField = Entry(tab6,width=12,justify="center")
VisY1RobEntryField.place(x=1010, y=105)
VisY1RobLab = Label(tab6, text = "Y1 Robot Pos")
VisY1RobLab.place(x=1010, y=125)

VisX2RobEntryField = Entry(tab6,width=12,justify="center")
VisX2RobEntryField.place(x=1010, y=155)
VisX2RobLab = Label(tab6, text = "X2 Robot Pos")
VisX2RobLab.place(x=1010, y=175)

VisY2RobEntryField = Entry(tab6,width=12,justify="center")
VisY2RobEntryField.place(x=1010, y=205)
VisY2RobLab = Label(tab6, text = "Y2 Robot Pos")
VisY2RobLab.place(x=1010, y=225)





####################################################################################################################################################
####################################################################################################################################################
####################################################################################################################################################
####TAB 7

GcodeProgEntryField = Entry(tab7,width=60,justify="center")
GcodeProgEntryField.place(x=20, y=55)

GcodCurRowEntryField = Entry(tab7,width=10,justify="center")
GcodCurRowEntryField.place(x=1175, y=20)

GC_ST_E1_EntryField = Entry(tab7,width=5,justify="center")
GC_ST_E1_EntryField.place(x=20, y=140)

GC_ST_E2_EntryField = Entry(tab7,width=5,justify="center")
GC_ST_E2_EntryField.place(x=75, y=140)

GC_ST_E3_EntryField = Entry(tab7,width=5,justify="center")
GC_ST_E3_EntryField.place(x=130, y=140)

GC_ST_E4_EntryField = Entry(tab7,width=5,justify="center")
GC_ST_E4_EntryField.place(x=185, y=140)

GC_ST_E5_EntryField = Entry(tab7,width=5,justify="center")
GC_ST_E5_EntryField.place(x=240, y=140)

GC_ST_E6_EntryField = Entry(tab7,width=5,justify="center")
GC_ST_E6_EntryField.place(x=295, y=140)

GC_ST_WC_EntryField = Entry(tab7,width=3,justify="center")
GC_ST_WC_EntryField.place(x=350, y=140)


GC_SToff_E1_EntryField = Entry(tab7,width=5,justify="center")
GC_SToff_E1_EntryField.place(x=20, y=205)

GC_SToff_E2_EntryField = Entry(tab7,width=5,justify="center")
GC_SToff_E2_EntryField.place(x=75, y=205)

GC_SToff_E3_EntryField = Entry(tab7,width=5,justify="center")
GC_SToff_E3_EntryField.place(x=130, y=205)

GC_SToff_E4_EntryField = Entry(tab7,width=5,justify="center")
GC_SToff_E4_EntryField.place(x=185, y=205)

GC_SToff_E5_EntryField = Entry(tab7,width=5,justify="center")
GC_SToff_E5_EntryField.place(x=240, y=205)

GC_SToff_E6_EntryField = Entry(tab7,width=5,justify="center")
GC_SToff_E6_EntryField.place(x=295, y=205)

GcodeFilenameField = Entry(tab7,width=40,justify="center")
GcodeFilenameField.place(x=20, y=340)


GCalmStatusLab = Label(tab7, text = "GCODE IDLE", style="OK.TLabel")
GCalmStatusLab.place(x=400, y=20)


gcodeframe=Frame(tab7)
gcodeframe.place(x=400,y=53)
gcodescrollbar = Scrollbar(gcodeframe) 
gcodescrollbar.pack(side=RIGHT, fill=Y)
tab7.gcodeView = Listbox(gcodeframe,exportselection=0,width=105,height=43, yscrollcommand=gcodescrollbar.set)
tab7.gcodeView.bind('<<ListboxSelect>>', gcodeViewselect)
tab7.gcodeView.pack()
gcodescrollbar.config(command=tab7.gcodeView.yview)

def GCcallback(event):
    selection = event.widget.curselection()
    try:
      if selection:
          index = selection[0]
          data = event.widget.get(index)
          data = data.replace('.txt','')
          GcodeFilenameField.delete(0, 'end')
          GcodeFilenameField.insert(0,data)
          PlayGCEntryField.delete(0, 'end')
          PlayGCEntryField.insert(0,data)    
      else:
          GcodeFilenameField.insert(0,"")  
    except:
      logger.error("not an SD file")
      
tab7.gcodeView.bind("<<ListboxSelect>>", GCcallback)


LoadGcodeBut = Button(tab7,  text="Load Program", width=25, command = loadGcodeProg)
LoadGcodeBut.place(x=20, y=20)

GcodeStartPosBut = Button(tab7,  text="Set Start Position", width=25, command = SetGcodeStartPos)
GcodeStartPosBut.place(x=20, y=100)

GcodeMoveStartPosBut = Button(tab7,  text="Move to Start Offset", width=25, command = MoveGcodeStartPos)
GcodeMoveStartPosBut.place(x=20, y=240)

runGcodeBut = Button(tab7, text="Convert & Upload to SD", width=25,   command = GCconvertProg)
#playGPhoto=PhotoImage(file="play-icon.gif")
#runGcodeBut.config(image=playGPhoto)
runGcodeBut.place(x=20, y=375)

stopGcodeBut = Button(tab7, text="Stop Conversion & Upload", width=25,  command = GCstopProg)
#stopGPhoto=PhotoImage(file="stop-icon.gif")
#stopGcodeBut.config(image=stopGPhoto)
stopGcodeBut.place(x=190, y=375)

delGcodeBut = Button(tab7, text="Delete File from SD", width=25,   command = GCdelete)
delGcodeBut.place(x=20, y=415)

readGcodeBut = Button(tab7, text="Read Files from SD", width=25,   command = partial(GCread, "yes"))
readGcodeBut.place(x=20, y=455)

playGPhoto=PhotoImage(file="play-icon.png")
readGcodeBut = Button(tab7, text="Play Gcode File", width=20,   command = GCplay, image = playGPhoto, compound=LEFT)
readGcodeBut.place(x=20, y=495)

#revGcodeBut = Button(tab7,  text="REV ",  command = stepRev)
#revGcodeBut.place(x=180, y=290)

#fwdGcodeBut = Button(tab7,  text="FWD", command = GCstepFwd)
#fwdGcodeBut.place(x=230, y=290)

saveGCBut = Button(tab7,  text="SAVE DATA",  width=26, command = SaveAndApplyCalibration)
saveGCBut.place(x=20, y=600)











gcodeCurRowLab = Label(tab7, text = "Current Row: ")
gcodeCurRowLab.place(x=1100, y=21)

gcodeStartPosOFfLab = Label(tab7, text = "Start Position Offset")
gcodeStartPosOFfLab.place(x=20, y=180)

gcodeFilenameLab = Label(tab7, text = "Filename:")
gcodeFilenameLab.place(x=20, y=320)







####################################################################################################################################################
####################################################################################################################################################
####################################################################################################################################################
####TAB 8

Elogframe=Frame(tab8)
Elogframe.place(x=40,y=15)
scrollbar = Scrollbar(Elogframe) 
scrollbar.pack(side=RIGHT, fill=Y)
tab8.ElogView = Listbox(Elogframe,width=230,height=40, yscrollcommand=scrollbar.set)
try:
  Elog = pickle.load(open("ErrorLog","rb"))
except:
  Elog = ['##BEGINNING OF LOG##']
  pickle.dump(Elog,open("ErrorLog","wb"))
time.sleep(.1)
for item in Elog:
  tab8.ElogView.insert(END,item) 
tab8.ElogView.pack()
scrollbar.config(command=tab8.ElogView.yview)

def clearLog():
 tab8.ElogView.delete(1,END)
 value=tab8.ElogView.get(0,END)
 pickle.dump(value,open("ErrorLog","wb"))

clearLogBut = Button(tab8,  text="Clear Log",  width=26, command = clearLog)
clearLogBut.place(x=40, y=690)




####################################################################################################################################################
####################################################################################################################################################
####################################################################################################################################################
####TAB 9

link = Label(tab9, font='12', text="https://www.anninrobotics.com/tutorials",  cursor="hand2")
link.bind("<Button-1>", lambda event: webbrowser.open(link.cget("text")))
link.place(x=10, y=9)

def callback():
    webbrowser.open_new(r"https://www.paypal.me/ChrisAnnin")

donateBut = Button(tab9,  command = callback)
donatePhoto=PhotoImage(file="pp.gif")
donateBut.config(image=donatePhoto)
donateBut.place(x=1250, y=2)


scroll = Scrollbar(tab9)
scroll.pack(side=RIGHT, fill=Y)
configfile = Text(tab9, wrap=WORD, width=166, height=40, yscrollcommand=scroll.set)
filename='information.txt'
with open(filename, 'r', encoding='utf-8-sig') as file:
  configfile.insert(INSERT, file.read())
configfile.pack(side="left")
scroll.config(command=configfile.yview)
configfile.place(x=10, y=40)






##############################################################################################################################################################
### OPEN CAL FILE AND LOAD LIST ##############################################################################################################################
##############################################################################################################################################################


loaded_calibration = _load_startup_calibration()
apply_calibration(loaded_calibration, CAL)

logger.debug(f"Comport 1 restored value is: {CAL['comPort']}")
logger.debug(f"Comport 1 restored value is: {CAL['com2Port']}")

if CAL['comPort'] in port_choices:
  com1SelectedValue.set(CAL['comPort'])

if CAL['com2Port'] in port_choices:
  com2SelectedValue.set(CAL['com2Port'])

try:
  restored_auxiliary_board = normalize_auxiliary_board_profile(
    CAL.get('auxiliaryBoard', AUXILIARY_BOARD_NONE),
    allow_none=True,
  )
except MotionInputError as exc:
  logger.error("Invalid saved auxiliary-board profile: %s", exc)
  restored_auxiliary_board = None
CAL['auxiliaryBoard'] = restored_auxiliary_board or AUXILIARY_BOARD_NONE
auxiliaryBoardSelectedValue.set(CAL['auxiliaryBoard'])


incrementEntryField.insert(0,"10")
speedEntryField.insert(0,"25")
ACCspeedField.insert(0,"15")
DECspeedField.insert(0,"15")
ACCrampField.insert(0,"80")
roundEntryField.insert(0,"0")
#ProgEntryField.insert(0,(Prog))
SavePosEntryField.insert(0,"1")
R1EntryField.insert(0,"0")
R2EntryField.insert(0,"0")
R3EntryField.insert(0,"0")
R4EntryField.insert(0,"0")
R5EntryField.insert(0,"0")
R6EntryField.insert(0,"0")
R7EntryField.insert(0,"0")
R8EntryField.insert(0,"0")
R9EntryField.insert(0,"0")
R10EntryField.insert(0,"0")
R11EntryField.insert(0,"0")
R12EntryField.insert(0,"0")
R13EntryField.insert(0,"0")
R14EntryField.insert(0,"0")
R15EntryField.insert(0,"0")
R16EntryField.insert(0,"0")
SP_1_E1_EntryField.insert(0,"0")
SP_2_E1_EntryField.insert(0,"0")
SP_3_E1_EntryField.insert(0,"0")
SP_4_E1_EntryField.insert(0,"0")
SP_5_E1_EntryField.insert(0,"0")
SP_6_E1_EntryField.insert(0,"0")
SP_7_E1_EntryField.insert(0,"0")
SP_8_E1_EntryField.insert(0,"0")
SP_9_E1_EntryField.insert(0,"0")
SP_10_E1_EntryField.insert(0,"0")
SP_11_E1_EntryField.insert(0,"0")
SP_12_E1_EntryField.insert(0,"0")
SP_13_E1_EntryField.insert(0,"0")
SP_14_E1_EntryField.insert(0,"0")
SP_15_E1_EntryField.insert(0,"0")
SP_16_E1_EntryField.insert(0,"0")
SP_1_E2_EntryField.insert(0,"0")
SP_2_E2_EntryField.insert(0,"0")
SP_3_E2_EntryField.insert(0,"0")
SP_4_E2_EntryField.insert(0,"0")
SP_5_E2_EntryField.insert(0,"0")
SP_6_E2_EntryField.insert(0,"0")
SP_7_E2_EntryField.insert(0,"0")
SP_8_E2_EntryField.insert(0,"0")
SP_9_E2_EntryField.insert(0,"0")
SP_10_E2_EntryField.insert(0,"0")
SP_11_E2_EntryField.insert(0,"0")
SP_12_E2_EntryField.insert(0,"0")
SP_13_E2_EntryField.insert(0,"0")
SP_14_E2_EntryField.insert(0,"0")
SP_15_E2_EntryField.insert(0,"0")
SP_16_E2_EntryField.insert(0,"0")
SP_1_E3_EntryField.insert(0,"0")
SP_2_E3_EntryField.insert(0,"0")
SP_3_E3_EntryField.insert(0,"0")
SP_4_E3_EntryField.insert(0,"0")
SP_5_E3_EntryField.insert(0,"0")
SP_6_E3_EntryField.insert(0,"0")
SP_7_E3_EntryField.insert(0,"0")
SP_8_E3_EntryField.insert(0,"0")
SP_9_E3_EntryField.insert(0,"0")
SP_10_E3_EntryField.insert(0,"0")
SP_11_E3_EntryField.insert(0,"0")
SP_12_E3_EntryField.insert(0,"0")
SP_13_E3_EntryField.insert(0,"0")
SP_14_E3_EntryField.insert(0,"0")
SP_15_E3_EntryField.insert(0,"0")
SP_16_E3_EntryField.insert(0,"0")
SP_1_E4_EntryField.insert(0,"0")
SP_2_E4_EntryField.insert(0,"0")
SP_3_E4_EntryField.insert(0,"0")
SP_4_E4_EntryField.insert(0,"0")
SP_5_E4_EntryField.insert(0,"0")
SP_6_E4_EntryField.insert(0,"0")
SP_7_E4_EntryField.insert(0,"0")
SP_8_E4_EntryField.insert(0,"0")
SP_9_E4_EntryField.insert(0,"0")
SP_10_E4_EntryField.insert(0,"0")
SP_11_E4_EntryField.insert(0,"0")
SP_12_E4_EntryField.insert(0,"0")
SP_13_E4_EntryField.insert(0,"0")
SP_14_E4_EntryField.insert(0,"0")
SP_15_E4_EntryField.insert(0,"0")
SP_16_E4_EntryField.insert(0,"0")
SP_1_E5_EntryField.insert(0,"0")
SP_2_E5_EntryField.insert(0,"0")
SP_3_E5_EntryField.insert(0,"0")
SP_4_E5_EntryField.insert(0,"0")
SP_5_E5_EntryField.insert(0,"0")
SP_6_E5_EntryField.insert(0,"0")
SP_7_E5_EntryField.insert(0,"0")
SP_8_E5_EntryField.insert(0,"0")
SP_9_E5_EntryField.insert(0,"0")
SP_10_E5_EntryField.insert(0,"0")
SP_11_E5_EntryField.insert(0,"0")
SP_12_E5_EntryField.insert(0,"0")
SP_13_E5_EntryField.insert(0,"0")
SP_14_E5_EntryField.insert(0,"0")
SP_15_E5_EntryField.insert(0,"0")
SP_16_E5_EntryField.insert(0,"0")
SP_1_E6_EntryField.insert(0,"0")
SP_2_E6_EntryField.insert(0,"0")
SP_3_E6_EntryField.insert(0,"0")
SP_4_E6_EntryField.insert(0,"0")
SP_5_E6_EntryField.insert(0,"0")
SP_6_E6_EntryField.insert(0,"0")
SP_7_E6_EntryField.insert(0,"0")
SP_8_E6_EntryField.insert(0,"0")
SP_9_E6_EntryField.insert(0,"0")
SP_10_E6_EntryField.insert(0,"0")
SP_11_E6_EntryField.insert(0,"0")
SP_12_E6_EntryField.insert(0,"0")
SP_13_E6_EntryField.insert(0,"0")
SP_14_E6_EntryField.insert(0,"0")
SP_15_E6_EntryField.insert(0,"0")
SP_16_E6_EntryField.insert(0,"0")
servo0onEntryField.insert(0,str(CAL['Servo0on']))
servo0offEntryField.insert(0,str(CAL['Servo0off']))
servo1onEntryField.insert(0,str(CAL['Servo1on']))
servo1offEntryField.insert(0,str(CAL['Servo1off']))
servo2onEntryField.insert(0,str(CAL.get('Servo2on', '')))
servo2offEntryField.insert(0,str(CAL.get('Servo2off', '')))
servo3onEntryField.insert(0,str(CAL.get('Servo3on', '')))
servo3offEntryField.insert(0,str(CAL.get('Servo3off', '')))
DO1onEntryField.insert(0,str(CAL['DO1on']))
DO1offEntryField.insert(0,str(CAL['DO1off']))
DO2onEntryField.insert(0,str(CAL['DO2on']))
DO2offEntryField.insert(0,str(CAL['DO2off']))
DO3onEntryField.insert(0,str(CAL.get('DO3on', '')))
DO3offEntryField.insert(0,str(CAL.get('DO3off', '')))
DO4onEntryField.insert(0,str(CAL.get('DO4on', '')))
DO4offEntryField.insert(0,str(CAL.get('DO4off', '')))
DO5onEntryField.insert(0,str(CAL.get('DO5on', '')))
DO5offEntryField.insert(0,str(CAL.get('DO5off', '')))
DO6onEntryField.insert(0,str(CAL.get('DO6on', '')))
DO6offEntryField.insert(0,str(CAL.get('DO6off', '')))
TFxEntryField.insert(0,str(CAL['TFx']))
TFyEntryField.insert(0,str(CAL['TFy']))
TFzEntryField.insert(0,str(CAL['TFz']))
TFrxEntryField.insert(0,str(CAL['TFrx']))
TFryEntryField.insert(0,str(CAL['TFry']))
TFrzEntryField.insert(0,str(CAL['TFrz']))
J7curAngEntryField.insert(0,str(CAL['J7PosCur']))
J8curAngEntryField.insert(0,str(CAL['J8PosCur']))
J9curAngEntryField.insert(0,str(CAL['J9PosCur']))
J1calOffEntryField.insert(0,str(CAL['J1calOff']))
J2calOffEntryField.insert(0,str(CAL['J2calOff']))
J3calOffEntryField.insert(0,str(CAL['J3calOff']))
J4calOffEntryField.insert(0,str(CAL['J4calOff']))
J5calOffEntryField.insert(0,str(CAL['J5calOff']))
J6calOffEntryField.insert(0,str(CAL['J6calOff']))
J7calOffEntryField.insert(0,str(CAL['J7calOff']))
J8calOffEntryField.insert(0,str(CAL['J8calOff']))
J9calOffEntryField.insert(0,str(CAL['J9calOff']))

if (CAL['curTheme'] == 1): 
  lightTheme()
else:
  darkTheme()
'''
if (CAL['J1CalStatVal'] == 1):
  RUN['J1CalStat1'].set(True)
if (CAL['J2CalStatVal'] == 1):
  RUN['J2CalStat1'].set(True)
if (CAL['J3CalStatVal'] == 1):
  RUN['J3CalStat1'].set(True)
if (CAL['J4CalStatVal'] == 1):
  RUN['J4CalStat1'].set(True)
if (CAL['J5CalStatVal'] == 1):
  RUN['J5CalStat1'].set(True)
if (CAL['J6CalStatVal'] == 1):
  RUN['J6CalStat1'].set(True)
if (CAL['J7CalStatVal'] == 1):
  RUN['J7CalStat1'].set(True) 
if (CAL['J8CalStatVal'] == 1):
  RUN['J8CalStat1'].set(True) 
if (CAL['J9CalStatVal'] == 1):
  RUN['J9CalStat1'].set(True)         
if (CAL['J1CalStatVal2'] == 1):
  RUN['J1CalStat2'].set(True)
if (CAL['J2CalStatVal2'] == 1):
  RUN['J2CalStat2'].set(True)
if (CAL['J3CalStatVal2'] == 1):
  RUN['J3CalStat2'].set(True)
if (CAL['J4CalStatVal2'] == 1):
  RUN['J4CalStat2'].set(True)
if (CAL['J5CalStatVal2'] == 1):
  RUN['J5CalStat2'].set(True)
if (CAL['J6CalStatVal2'] == 1):
  RUN['J6CalStat2'].set(True)
if (CAL['J7CalStatVal2'] == 1):
  RUN['J7CalStat2'].set(True) 
if (CAL['J8CalStatVal2'] == 1):
  RUN['J8CalStat2'].set(True) 
if (CAL['J9CalStatVal2'] == 1):
  RUN['J9CalStat2'].set(True)
'''          
axis7lengthEntryField.insert(0,str(CAL['J7PosLim']))
axis7rotEntryField.insert(0,str(CAL['J7rotation']))
axis7stepsEntryField.insert(0,str(CAL['J7steps']))
VisBrightSlide.set(CAL['VisBrightVal'])
VisContrastSlide.set(CAL['VisContVal'])
VisBacColorEntryField.insert(0,str(CAL['VisBacColor']))
VisScoreEntryField.insert(0,str(CAL['VisScore']))
VisX1PixEntryField.insert(0,str(CAL['VisX1Val']))
VisY1PixEntryField.insert(0,str(CAL['VisY1Val']))
VisX2PixEntryField.insert(0,str(CAL['VisX2Val']))
VisY2PixEntryField.insert(0,str(CAL['VisY2Val']))
VisX1RobEntryField.insert(0,str(CAL['VisRobX1Val']))
VisY1RobEntryField.insert(0,str(CAL['VisRobY1Val']))
VisX2RobEntryField.insert(0,str(CAL['VisRobX2Val']))
VisY2RobEntryField.insert(0,str(CAL['VisRobY2Val']))
VisZoomSlide.set(CAL['zoom'])
if (CAL['pickClosestVal'] == 1):
  RUN['pickClosest'].set(True)
if (CAL['pick180Val'] == 1):
  RUN['pick180'].set(True)  
visoptions.set(CAL['curCam'])
if (CAL['fullRotVal'] == 1):
  RUN['fullRot'].set(True)
if (CAL['autoBGVal'] == 1):
  RUN['autoBG'].set(True)  
RUN['mX1'] = CAL['mX1val']
RUN['mY1'] = CAL['mY1val']
RUN['mX2'] = CAL['mX2val']
RUN['mY2'] = CAL['mY2val']
axis8lengthEntryField.insert(0,str(CAL['J8length']))
axis8rotEntryField.insert(0,str(CAL['J8rotation']))
axis8stepsEntryField.insert(0,str(CAL['J8steps']))
axis9lengthEntryField.insert(0,str(CAL['J9length']))
axis9rotEntryField.insert(0,str(CAL['J9rotation']))
axis9stepsEntryField.insert(0,str(CAL['J9steps']))
GC_ST_E1_EntryField.insert(0,str(CAL['GC_ST_E1']))
GC_ST_E2_EntryField.insert(0,str(CAL['GC_ST_E2']))
GC_ST_E3_EntryField.insert(0,str(CAL['GC_ST_E3']))
GC_ST_E4_EntryField.insert(0,str(CAL['GC_ST_E4']))
GC_ST_E5_EntryField.insert(0,str(CAL['GC_ST_E5']))
GC_ST_E6_EntryField.insert(0,str(CAL['GC_ST_E6']))
GC_ST_WC_EntryField.insert(0,str(CAL['GC_ST_WC']))
GC_SToff_E1_EntryField.insert(0,str(CAL['GC_SToff_E1']))
GC_SToff_E2_EntryField.insert(0,str(CAL['GC_SToff_E2']))
GC_SToff_E3_EntryField.insert(0,str(CAL['GC_SToff_E3']))
GC_SToff_E4_EntryField.insert(0,str(CAL['GC_SToff_E4']))
GC_SToff_E5_EntryField.insert(0,str(CAL['GC_SToff_E5']))
GC_SToff_E6_EntryField.insert(0,str(CAL['GC_SToff_E6']))
J1MotDirEntryField.insert(0,str(CAL['J1MotDir']))
J2MotDirEntryField.insert(0,str(CAL['J2MotDir']))
J3MotDirEntryField.insert(0,str(CAL['J3MotDir']))
J4MotDirEntryField.insert(0,str(CAL['J4MotDir']))
J5MotDirEntryField.insert(0,str(CAL['J5MotDir']))
J6MotDirEntryField.insert(0,str(CAL['J6MotDir']))
J7MotDirEntryField.insert(0,str(CAL['J7MotDir']))
J8MotDirEntryField.insert(0,str(CAL['J8MotDir']))
J9MotDirEntryField.insert(0,str(CAL['J9MotDir']))
J1CalDirEntryField.insert(0,str(CAL['J1CalDir']))
J2CalDirEntryField.insert(0,str(CAL['J2CalDir']))
J3CalDirEntryField.insert(0,str(CAL['J3CalDir']))
J4CalDirEntryField.insert(0,str(CAL['J4CalDir']))
J5CalDirEntryField.insert(0,str(CAL['J5CalDir']))
J6CalDirEntryField.insert(0,str(CAL['J6CalDir']))
J7CalDirEntryField.insert(0,str(CAL['J7CalDir']))
J8CalDirEntryField.insert(0,str(CAL['J8CalDir']))
J9CalDirEntryField.insert(0,str(CAL['J9CalDir']))
J1PosLimEntryField.insert(0,str(CAL['J1PosLim']))
J1NegLimEntryField.insert(0,str(CAL['J1NegLim']))
J2PosLimEntryField.insert(0,str(CAL['J2PosLim']))
J2NegLimEntryField.insert(0,str(CAL['J2NegLim']))
J3PosLimEntryField.insert(0,str(CAL['J3PosLim']))
J3NegLimEntryField.insert(0,str(CAL['J3NegLim']))
J4PosLimEntryField.insert(0,str(CAL['J4PosLim']))
J4NegLimEntryField.insert(0,str(CAL['J4NegLim']))
J5PosLimEntryField.insert(0,str(CAL['J5PosLim']))
J5NegLimEntryField.insert(0,str(CAL['J5NegLim']))
J6PosLimEntryField.insert(0,str(CAL['J6PosLim']))
J6NegLimEntryField.insert(0,str(CAL['J6NegLim']))  
J1StepDegEntryField.insert(0,str(CAL['J1StepDeg']))
J2StepDegEntryField.insert(0,str(CAL['J2StepDeg'])) 
J3StepDegEntryField.insert(0,str(CAL['J3StepDeg'])) 
J4StepDegEntryField.insert(0,str(CAL['J4StepDeg'])) 
J5StepDegEntryField.insert(0,str(CAL['J5StepDeg'])) 
J6StepDegEntryField.insert(0,str(CAL['J6StepDeg']))
J1DriveMSEntryField.insert(0,str(CAL['J1DriveMS']))
J2DriveMSEntryField.insert(0,str(CAL['J2DriveMS']))  
J3DriveMSEntryField.insert(0,str(CAL['J3DriveMS']))  
J4DriveMSEntryField.insert(0,str(CAL['J4DriveMS']))  
J5DriveMSEntryField.insert(0,str(CAL['J5DriveMS']))  
J6DriveMSEntryField.insert(0,str(CAL['J6DriveMS']))
J1EncCPREntryField.insert(0,str(CAL['J1EncCPR']))
J2EncCPREntryField.insert(0,str(CAL['J2EncCPR']))
J3EncCPREntryField.insert(0,str(CAL['J3EncCPR']))
J4EncCPREntryField.insert(0,str(CAL['J4EncCPR']))
J5EncCPREntryField.insert(0,str(CAL['J5EncCPR']))
J6EncCPREntryField.insert(0,str(CAL['J6EncCPR']))
J1ΘEntryField.insert(0,str(CAL['J1ΘDHpar']))
J2ΘEntryField.insert(0,str(CAL['J2ΘDHpar']))
J3ΘEntryField.insert(0,str(CAL['J3ΘDHpar']))
J4ΘEntryField.insert(0,str(CAL['J4ΘDHpar']))
J5ΘEntryField.insert(0,str(CAL['J5ΘDHpar']))
J6ΘEntryField.insert(0,str(CAL['J6ΘDHpar']))
J1αEntryField.insert(0,str(CAL['J1αDHpar']))
J2αEntryField.insert(0,str(CAL['J2αDHpar']))
J3αEntryField.insert(0,str(CAL['J3αDHpar']))
J4αEntryField.insert(0,str(CAL['J4αDHpar']))
J5αEntryField.insert(0,str(CAL['J5αDHpar']))
J6αEntryField.insert(0,str(CAL['J6αDHpar']))
J1dEntryField.insert(0,str(CAL['J1dDHpar']))
J2dEntryField.insert(0,str(CAL['J2dDHpar']))
J3dEntryField.insert(0,str(CAL['J3dDHpar']))
J4dEntryField.insert(0,str(CAL['J4dDHpar']))
J5dEntryField.insert(0,str(CAL['J5dDHpar']))
J6dEntryField.insert(0,str(CAL['J6dDHpar']))
J1aEntryField.insert(0,str(CAL['J1aDHpar']))
J2aEntryField.insert(0,str(CAL['J2aDHpar']))
J3aEntryField.insert(0,str(CAL['J3aDHpar']))
J4aEntryField.insert(0,str(CAL['J4aDHpar']))
J5aEntryField.insert(0,str(CAL['J5aDHpar']))
J6aEntryField.insert(0,str(CAL['J6aDHpar']))

if not update_CPP_kin_from_entries():
  message = "MOTION DISABLED: NATIVE KINEMATICS CONFIGURATION FAILED"
  logger.error(message)
  almStatusLab.config(text=message, style="Alarm.TLabel")
  almStatusLab2.config(text=message, style="Alarm.TLabel")
RUN['VR_angles'] = [float(CAL['J1AngCur']), float(CAL['J2AngCur']), float(CAL['J3AngCur']), float(CAL['J4AngCur']), float(CAL['J5AngCur']), float(CAL['J6AngCur'])]
RUN['JangleOut'] = np.array([float(CAL['J1AngCur']), float(CAL['J2AngCur']), float(CAL['J3AngCur']), float(CAL['J4AngCur']), float(CAL['J5AngCur']), float(CAL['J6AngCur'])])
RUN['negLim'] = [float(CAL['J1NegLim']), float(CAL['J2NegLim']), float(CAL['J3NegLim']), float(CAL['J4NegLim']), float(CAL['J5NegLim']), float(CAL['J6NegLim'])]

#axis limits in each direction
RUN['J1axisLimNeg'] = float(CAL['J1NegLim'])
RUN['J2axisLimNeg'] = float(CAL['J2NegLim'])
RUN['J3axisLimNeg'] = float(CAL['J3NegLim'])
RUN['J4axisLimNeg'] = float(CAL['J4NegLim'])
RUN['J5axisLimNeg'] = float(CAL['J5NegLim'])
RUN['J6axisLimNeg'] = float(CAL['J6NegLim'])
J1axisLimPos = float(CAL['J1PosLim'])
J2axisLimPos = float(CAL['J2PosLim'])
J3axisLimPos = float(CAL['J3PosLim'])
J4axisLimPos = float(CAL['J4PosLim'])
J5axisLimPos = float(CAL['J5PosLim'])
J6axisLimPos = float(CAL['J6PosLim'])


#degrees full movement of each axis
J1axisLim = J1axisLimPos + RUN['J1axisLimNeg'];
J2axisLim = J2axisLimPos + RUN['J2axisLimNeg'];
J3axisLim = J3axisLimPos + RUN['J3axisLimNeg'];
J4axisLim = J4axisLimPos + RUN['J4axisLimNeg'];
J5axisLim = J5axisLimPos + RUN['J5axisLimNeg'];
J6axisLim = J6axisLimPos + RUN['J6axisLimNeg'];
#steps full movement of each axis
J1StepLim = J1axisLim * float(CAL['J1StepDeg'])
J2StepLim = J2axisLim * float(CAL['J2StepDeg'])
J3StepLim = J3axisLim * float(CAL['J3StepDeg'])
J4StepLim = J4axisLim * float(CAL['J4StepDeg'])
J5StepLim = J5axisLim * float(CAL['J5StepDeg'])
J6StepLim = J6axisLim * float(CAL['J6StepDeg'])
RUN['stepDeg'] = [float(CAL['J1StepDeg']), float(CAL['J2StepDeg']), float(CAL['J3StepDeg']), float(CAL['J4StepDeg']), float(CAL['J5StepDeg']), float(CAL['J6StepDeg'])]
setStepMonitorsVR()
main_color_var.set(CAL['setColor'])




msg = "ANNIN ROBOTICS SOFTWARE AND DESIGNS ARE FREE:\n\
\n\
*for personal use.\n\
*for educational use.\n\
*for building your own robot(s).\n\
*for automating your own business.\n\
\n\
IT IS NOT OK TO RESELL THIS SOFTWARE OR ROBOTS\n\
FOR A PROFIT - IT MUST REMAIN FREE.\n\
\n\
IT IS NOT OK TO SELL ANNIN ROBOTICS ROBOTS,\n\
ROBOT PARTS, OR ANY OTHER VERSION \n\
OF ROBOT OR SOFTWARE BASED ON\n\
ANNIN ROBOTICS DESIGNS FOR PROFIT.\n\
\n\
ANY AR ROBOTS OR PARTS FOR SALE ON ALIEXPRESS\n\
OR ANY OTHER PLATFORM NOT PURCHASED FROM ANNIN ROBOTICS\n\
ARE COUNTERFEIT & ILLEGAL\n\
\n\
AR3 and AR4 are registered trademarks of Annin Robotics\n\
Copyright © 2022 by Annin Robotics. All Rights Reserved"


#tkinter.messagebox.showwarning("AR4 License / Copyright notice", msg)
RUN['xboxUse'] = 0

tab1.lastProg = ""
tab1.after(25, _poll_joint_motion_events)
tab1.after(25, _poll_serial_events)
tab1.after(25, _poll_calibration_events)
tab1.after(25, _poll_auxiliary_serial_events)
tab1.after(25, _poll_manual_auxiliary_events)
tab1.after(25, _poll_xbox_auxiliary_events)
tab1.after(25, _poll_virtual_motion_events)
tab1.after(100, setCom)

#tab1.mainloop()
root.mainloop()



#manEntryField.delete(0, 'end')
#manEntryField.insert(0,value)
