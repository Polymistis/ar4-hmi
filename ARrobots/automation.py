"""Import-safe JSON sessions owning bounded terminal settlement and close."""
import math
import threading

from . import protocol as _p


class AutomationStateError(RuntimeError):
    """A disconnected, closed, or busy facade rejected an operation."""


class AutomationCommandError(RuntimeError):
    """Retain exact controller, command, status, failure, and rejected terminal."""
    def __init__(self, controller, terminal):
        response = terminal.response
        self.controller, self.command = controller, response.cmd
        self.status, self.failure = response.status, terminal.failure
        self.terminal = terminal
        super().__init__(f"{controller} {response.cmd} returned {response.status}")


class AutomationCleanupError(RuntimeError):
    """Retain operation, cleanup, client, and serial ownership after close failure."""
    def __init__(self, operation_error, cleanup_error, client, serial_port):
        if operation_error is not None and not isinstance(operation_error, BaseException):
            raise TypeError("automation operation error is invalid")
        if not isinstance(cleanup_error, BaseException):
            raise TypeError("automation cleanup error is invalid")
        self.operation_error, self.cleanup_error = operation_error, cleanup_error
        self.client, self.serial_port = client, serial_port
        super().__init__(
            "automation cleanup failed" if operation_error is None
            else "automation operation and cleanup both failed")


def _timeout(value, name):
    if type(value) not in (int, float):
        raise TypeError(f"{name} must be numeric")
    try:
        value = float(value)
    except OverflowError as error:
        raise ValueError(f"{name} must be positive and finite") from error
    if not math.isfinite(value) or value <= 0:
        raise ValueError(f"{name} must be positive and finite")
    return value


def _close_unowned(serial_port, operation_error):
    cleanup_error = _p.close_unowned_serial_port(serial_port)
    if cleanup_error is not None:
        error = AutomationCleanupError(operation_error, cleanup_error, None, serial_port)
        raise error from cleanup_error
    raise operation_error


def _auxiliary_terminal(client, ticket):
    while True:
        delivery = client.pop_delivery()
        if delivery is None:
            client.poll()
            continue
        if type(delivery) is not _p.JsonResponseDelivery or delivery.ticket is not ticket:
            raise _p.JsonSessionProtocolError("unexpected auxiliary controller delivery")
        terminal = client.take_terminal(ticket)
        client.acknowledge_terminal(ticket)
        return terminal


class _AutomationSession:
    def __init__(self):
        self._client = self._serial_port = self._close_timeout = None
        self._cleanup_failed = False
        self._call_lock = threading.Lock()
    def _activate(self, client, serial_port, close_timeout):
        self._client, self._serial_port = client, serial_port
        self._close_timeout = close_timeout
    def _active(self):
        client = self._client
        if client is None or client.closed or client.closing or client.quarantined:
            raise AutomationStateError("automation session is not active")
        return client
    def _finish(self, operation_error=None):
        client, serial_port = self._client, self._serial_port
        try:
            client.close(timeout=self._close_timeout)
        except BaseException as cleanup_error:
            self._cleanup_failed = True
            error = AutomationCleanupError(operation_error, cleanup_error, client, serial_port)
            raise error from cleanup_error
        self._client = self._serial_port = None
        if operation_error is not None:
            raise operation_error
    def _call(self, operation, *args, preflight=None):
        if not self._call_lock.acquire(blocking=False):
            raise AutomationStateError("automation call is already active")
        try:
            client = self._active()
            if preflight is not None:
                preflight(client, *args)
            try:
                return operation(client, *args)
            except AutomationCommandError:
                raise
            except BaseException as error:
                self._finish(error)
        finally:
            self._call_lock.release()
    def close(self):
        if not self._call_lock.acquire(blocking=False):
            raise AutomationStateError("automation call is already active")
        try:
            if self._client is None:
                raise AutomationStateError("automation session is not active")
            self._finish()
        finally:
            self._call_lock.release()
    def __enter__(self):
        self._active()
        return self
    def __exit__(self, exc_type, exc_value, traceback):
        if self._client is None or self._cleanup_failed:
            return False
        try:
            self.close()
        except AutomationCleanupError as error:
            if exc_value is not None:
                raise AutomationCleanupError(
                    exc_value, error.cleanup_error, error.client, error.serial_port
                ) from error.cleanup_error
            raise
        return False


class ARRobot(_AutomationSession):
    """Own main JSON identity, terminal commands, stop publication, and close."""
    def __init__(self):
        super().__init__()
        self._startup = self._physical_stop_callback = None
    @classmethod
    def connect(
        cls, serial_port, *, configuration, physical_stop_callback,
        request_timeout, cancellation_boundary=None,
        close_timeout=_p.JSON_COORDINATOR_DEFAULT_CLOSE_TIMEOUT_SECONDS,
    ):
        try:
            if type(configuration) is not _p.JsonMainControllerStartupConfiguration:
                raise TypeError("main startup configuration is invalid")
            if not callable(physical_stop_callback):
                raise TypeError("physical-stop callback must be callable")
            request_timeout = _timeout(request_timeout, "request timeout")
            close_timeout = _timeout(close_timeout, "close timeout")
            session, boundary = cls(), cancellation_boundary
            if boundary is None:
                boundary = _p.SerialWriteCancellationBoundary("automation main startup")
            if type(boundary) is not _p.SerialWriteCancellationBoundary:
                raise TypeError("cancellation boundary is invalid")
        except BaseException as error:
            _close_unowned(serial_port, error)
        try:
            startup = _p.run_main_controller_json_startup(
                serial_port, configuration, boundary,
                physical_stop_callback=physical_stop_callback,
                request_timeout=request_timeout, close_timeout=close_timeout,
            )
        except _p.JsonMainControllerStartupCleanupError as error:
            raise AutomationCleanupError(
                error.operation_error, error.cleanup_error, error.client, serial_port
            ) from error.cleanup_error
        _AutomationSession._activate(session, startup.client, serial_port, close_timeout)
        session._startup = startup.startup
        session._physical_stop_callback = physical_stop_callback
        return session
    @property
    def identity(self):
        self._active()
        return self._startup.hello
    @property
    def startup_position(self):
        self._active()
        return self._startup.position
    @property
    def startup_home_reference(self):
        self._active()
        return self._startup.home_reference
    def _main_terminal_owned(
        self, client, command, request, params, timeout,
        accepted_response_allowed,
    ):
        drain_timeout = min(timeout, _p.JSON_SERIAL_DEFAULT_FRAME_TIMEOUT_SECONDS)
        _p.drain_main_controller_input(
            client, command, self._physical_stop_callback, drain_timeout
        )
        ticket = getattr(client, request)(**params, timeout=timeout)
        terminal = _p.await_main_controller_terminal(
            client, ticket, f"main-controller JSON {command}",
            physical_stop_callback=self._physical_stop_callback,
            drain_timeout=drain_timeout,
            accepted_response_allowed=accepted_response_allowed,
        )
        _p.drain_main_controller_input(
            client, command, self._physical_stop_callback, drain_timeout
        )
        if client.release_reader() is not True:
            raise _p.JsonSessionProtocolError("main reader release was not confirmed")
        if terminal.response.status != "completed":
            raise AutomationCommandError(_p.MAIN_CONTROLLER, terminal)
        result = terminal.parsed_result
        if type(result) is _p.JsonMainPositionResult and result.motion_fault:
            raise _p.JsonMainControllerStartupError(
                "main-controller JSON position reported motion fault: " + result.motion_fault,
                terminal=terminal,
            )
        return result
    def _read(self, command, request, timeout):
        self._active()
        timeout = _timeout(timeout, "request timeout")
        return self._call(
            self._main_terminal_owned,
            command, request, {}, timeout, False,
        )
    def refresh_position(self, *, timeout):
        return self._read(
            "get_position_disposition", "request_position_disposition", timeout
        )
    def refresh_home_reference(self, *, timeout):
        return self._read("get_home_reference", "request_home_reference", timeout)
    def read_limit_switches(self, *, timeout):
        return self._read("test_limit_switches", "request_test_limit_switches", timeout)
    def read_encoders(self, *, timeout):
        return self._read("read_encoders", "request_read_encoders", timeout)
    def _validate_modbus_read_owned(
        self, _client, _command, _request, params, _timeout_value,
        _accepted_response_allowed,
    ):
        _p.validate_main_modbus_read_request({
            "slave_id": params["slave_id"],
            "address": params["address"],
            "count": 1,
        })
    def _modbus_read(self, command, *, slave_id, address, timeout):
        self._active()
        timeout = _timeout(timeout, "request timeout")
        params = {
            "command": command,
            "slave_id": slave_id,
            "address": address,
        }
        return self._call(
            self._main_terminal_owned,
            command, "request_modbus_read", params, timeout, False,
            preflight=self._validate_modbus_read_owned,
        )
    def modbus_read_holding_register(self, *, slave_id, address, timeout):
        return self._modbus_read(
            "modbus_read_holding_register",
            slave_id=slave_id, address=address, timeout=timeout,
        )
    def modbus_read_coil(self, *, slave_id, address, timeout):
        return self._modbus_read(
            "modbus_read_coil",
            slave_id=slave_id, address=address, timeout=timeout,
        )
    def modbus_read_discrete_input(self, *, slave_id, address, timeout):
        return self._modbus_read(
            "modbus_read_discrete_input",
            slave_id=slave_id, address=address, timeout=timeout,
        )
    def modbus_read_input_register(self, *, slave_id, address, timeout):
        return self._modbus_read(
            "modbus_read_input_register",
            slave_id=slave_id, address=address, timeout=timeout,
        )
    def _validate_modbus_write_owned(
        self, _client, command, _request, params, _timeout_value,
        _accepted_response_allowed,
    ):
        _p.validate_main_modbus_write_request({
            "slave_id": params["slave_id"],
            "address": params["address"],
            "value": params["value"],
        }, command=command)
    def _modbus_write(self, command, *, slave_id, address, value, timeout):
        self._active()
        timeout = _timeout(timeout, "request timeout")
        params = {
            "command": command,
            "slave_id": slave_id,
            "address": address,
            "value": value,
        }
        return self._call(
            self._main_terminal_owned,
            command, "request_modbus_write", params, timeout, False,
            preflight=self._validate_modbus_write_owned,
        )
    def modbus_write_coil(self, *, slave_id, address, value, timeout):
        return self._modbus_write(
            "modbus_write_coil", slave_id=slave_id, address=address,
            value=value, timeout=timeout,
        )
    def modbus_write_register(self, *, slave_id, address, value, timeout):
        return self._modbus_write(
            "modbus_write_register", slave_id=slave_id, address=address,
            value=value, timeout=timeout,
        )
    def _validate_modbus_wait_owned(self, _client, command, params):
        _p.validate_main_modbus_wait_request(params, command=command)
    def _modbus_wait_owned(self, client, command, params):
        host_timeout = (
            params["timeout_seconds"]
            + 2 * _p.JSON_SERIAL_DEFAULT_FRAME_TIMEOUT_SECONDS
        )
        result = self._main_terminal_owned(
            client, command, "request_command",
            {"command": command, "params": params}, host_timeout, False,
        )
        if result.value != params["expected"]:
            raise _p.JsonSessionProtocolError(
                "completed Modbus wait returned an unexpected value"
            )
        return result
    def _modbus_wait(
        self, command, *, slave_id, address, expected, timeout_seconds,
    ):
        self._active()
        params = {
            "slave_id": slave_id,
            "address": address,
            "expected": expected,
            "timeout_seconds": timeout_seconds,
        }
        return self._call(
            self._modbus_wait_owned, command, params,
            preflight=self._validate_modbus_wait_owned,
        )
    def wait_modbus_coil(
        self, *, slave_id, address, expected, timeout_seconds,
    ):
        return self._modbus_wait(
            "wait_modbus_coil", slave_id=slave_id, address=address,
            expected=expected, timeout_seconds=timeout_seconds,
        )
    def wait_modbus_discrete_input(
        self, *, slave_id, address, expected, timeout_seconds,
    ):
        return self._modbus_wait(
            "wait_modbus_discrete_input", slave_id=slave_id, address=address,
            expected=expected, timeout_seconds=timeout_seconds,
        )
    def wait_modbus_holding_register(
        self, *, slave_id, address, expected, timeout_seconds,
    ):
        return self._modbus_wait(
            "wait_modbus_holding_register", slave_id=slave_id, address=address,
            expected=expected, timeout_seconds=timeout_seconds,
        )
    def _validate_move_joints_owned(
        self, _client, _command, _request, params, _timeout_value,
        _accepted_response_allowed,
    ):
        _p.validate_main_move_joints_request(params)
    def _validate_calibrate_owned(
        self, _client, _command, _request, params, _timeout_value,
        _accepted_response_allowed,
    ):
        _p.validate_main_calibrate_request(params)
    def calibrate(
        self, *, axes, offsets, timeout,
    ):
        """Run one selected-axis calibration through retained motion ownership.

        Axes and offsets use J1-J9 order. Offsets use degrees for J1-J6 and
        configured travel units for J7-J9.
        Timeout bounds host settlement without claiming controller cancellation.
        """
        self._active()
        timeout = _timeout(timeout, "request timeout")
        params = {
            "axes": axes,
            "offsets": offsets,
        }
        return self._call(
            self._main_terminal_owned,
            "calibrate", "request_calibrate", params, timeout, True,
            preflight=self._validate_calibrate_owned,
        )
    def move_joints(
        self, *, robot_joints_degrees, external_axes_units, speed_mode,
        speed_value, acceleration_percent, deceleration_percent, ramp_percent,
        wrist_configuration, loop_modes, timeout,
    ):
        self._active()
        timeout = _timeout(timeout, "request timeout")
        params = {
            "acceleration_percent": acceleration_percent,
            "deceleration_percent": deceleration_percent,
            "external_axes_units": external_axes_units,
            "loop_modes": loop_modes,
            "ramp_percent": ramp_percent,
            "robot_joints_degrees": robot_joints_degrees,
            "speed_mode": speed_mode,
            "speed_value": speed_value,
            "telemetry_enabled": False,
            "wrist_configuration": wrist_configuration,
        }
        return self._call(
            self._main_terminal_owned,
            "move_joints", "request_move_joints", params, timeout, True,
            preflight=self._validate_move_joints_owned,
        )
    def _validate_finite_motion_owned(
        self, _client, contract, params, _timeout_value,
    ):
        contract.request_validator(params)
    def _finite_motion_owned(self, client, contract, params, timeout):
        command = contract.name
        if command == "jog_tool":
            request, request_params = "request_jog_tool", params
        else:
            request = "request_command"
            request_params = {"command": command, "params": params}
        return self._main_terminal_owned(
            client, command, request, request_params, timeout,
            command in ("move_cartesian", "move_linear", "move_vision", "jog_tool"),
        )
    def _finite_motion(self, contract, params, timeout):
        self._active()
        timeout = _timeout(timeout, "request timeout")
        return self._call(
            self._finite_motion_owned, contract, params, timeout,
            preflight=self._validate_finite_motion_owned,
        )
    @staticmethod
    def _cartesian_motion(
        translation_millimeters, orientation_degrees, external_axes_units,
        speed_mode, speed_value, acceleration_percent, deceleration_percent,
        ramp_percent, wrist_configuration, loop_modes,
    ):
        return {
            "acceleration_percent": acceleration_percent,
            "deceleration_percent": deceleration_percent,
            "external_axes_units": external_axes_units,
            "loop_modes": loop_modes,
            "orientation_degrees": orientation_degrees,
            "ramp_percent": ramp_percent,
            "speed_mode": speed_mode,
            "speed_value": speed_value,
            "telemetry_enabled": False,
            "translation_millimeters": translation_millimeters,
            "wrist_configuration": wrist_configuration,
        }
    def _cartesian_finite_motion(
        self, contract, *, translation_millimeters, orientation_degrees,
        external_axes_units, speed_mode, speed_value, acceleration_percent,
        deceleration_percent, ramp_percent, wrist_configuration, loop_modes,
        timeout, **geometry,
    ):
        motion = self._cartesian_motion(
            translation_millimeters, orientation_degrees, external_axes_units,
            speed_mode, speed_value, acceleration_percent,
            deceleration_percent, ramp_percent, wrist_configuration, loop_modes,
        )
        params = motion if not geometry else {"motion": motion, **geometry}
        return self._finite_motion(contract, params, timeout)
    def move_cartesian(
        self, *, translation_millimeters, orientation_degrees,
        external_axes_units, speed_mode, speed_value, acceleration_percent,
        deceleration_percent, ramp_percent, wrist_configuration, loop_modes,
        timeout,
    ):
        return self._cartesian_finite_motion(
            _p.MAIN_MOVE_CARTESIAN_COMMAND_CONTRACT,
            translation_millimeters=translation_millimeters,
            orientation_degrees=orientation_degrees,
            external_axes_units=external_axes_units, speed_mode=speed_mode,
            speed_value=speed_value, acceleration_percent=acceleration_percent,
            deceleration_percent=deceleration_percent, ramp_percent=ramp_percent,
            wrist_configuration=wrist_configuration, loop_modes=loop_modes,
            timeout=timeout,
        )
    def move_linear(
        self, *, translation_millimeters, orientation_degrees,
        external_axes_units, speed_mode, speed_value, acceleration_percent,
        deceleration_percent, ramp_percent, wrist_configuration, loop_modes,
        timeout,
    ):
        return self._cartesian_finite_motion(
            _p.MAIN_MOVE_LINEAR_COMMAND_CONTRACT,
            translation_millimeters=translation_millimeters,
            orientation_degrees=orientation_degrees,
            external_axes_units=external_axes_units, speed_mode=speed_mode,
            speed_value=speed_value, acceleration_percent=acceleration_percent,
            deceleration_percent=deceleration_percent, ramp_percent=ramp_percent,
            wrist_configuration=wrist_configuration, loop_modes=loop_modes,
            rounding_millimeters=0.0, disable_wrist_rotation=False,
            timeout=timeout,
        )
    def move_vision(
        self, *, translation_millimeters, orientation_degrees,
        external_axes_units, speed_mode, speed_value, acceleration_percent,
        deceleration_percent, ramp_percent, wrist_configuration, loop_modes,
        vision_rotation_degrees, timeout,
    ):
        return self._cartesian_finite_motion(
            _p.MAIN_MOVE_VISION_COMMAND_CONTRACT,
            translation_millimeters=translation_millimeters,
            orientation_degrees=orientation_degrees,
            external_axes_units=external_axes_units, speed_mode=speed_mode,
            speed_value=speed_value, acceleration_percent=acceleration_percent,
            deceleration_percent=deceleration_percent, ramp_percent=ramp_percent,
            wrist_configuration=wrist_configuration, loop_modes=loop_modes,
            vision_rotation_degrees=vision_rotation_degrees, timeout=timeout,
        )
    def jog_tool(
        self, *, axis, direction, distance, speed_mode, speed_value,
        acceleration_percent, deceleration_percent, ramp_percent,
        wrist_configuration, loop_modes, timeout,
    ):
        return self._finite_motion(
            _p.MAIN_JOG_TOOL_COMMAND_CONTRACT,
            {
                "acceleration_percent": acceleration_percent,
                "axis": axis,
                "deceleration_percent": deceleration_percent,
                "direction": direction,
                "distance": distance,
                "loop_modes": loop_modes,
                "ramp_percent": ramp_percent,
                "speed_mode": speed_mode,
                "speed_value": speed_value,
                "wrist_configuration": wrist_configuration,
            },
            timeout,
        )
    def move_arc(
        self, *, translation_millimeters, orientation_degrees,
        external_axes_units, speed_mode, speed_value, acceleration_percent,
        deceleration_percent, ramp_percent, wrist_configuration, loop_modes,
        midpoint_translation_millimeters, timeout,
    ):
        return self._cartesian_finite_motion(
            _p.MAIN_MOVE_ARC_COMMAND_CONTRACT,
            translation_millimeters=translation_millimeters,
            orientation_degrees=orientation_degrees,
            external_axes_units=external_axes_units, speed_mode=speed_mode,
            speed_value=speed_value, acceleration_percent=acceleration_percent,
            deceleration_percent=deceleration_percent, ramp_percent=ramp_percent,
            wrist_configuration=wrist_configuration, loop_modes=loop_modes,
            midpoint_translation_millimeters=midpoint_translation_millimeters,
            timeout=timeout,
        )
    def move_circle(
        self, *, translation_millimeters, orientation_degrees,
        external_axes_units, speed_mode, speed_value, acceleration_percent,
        deceleration_percent, ramp_percent, wrist_configuration, loop_modes,
        center_translation_millimeters, plane_translation_millimeters, timeout,
    ):
        return self._cartesian_finite_motion(
            _p.MAIN_MOVE_CIRCLE_COMMAND_CONTRACT,
            translation_millimeters=translation_millimeters,
            orientation_degrees=orientation_degrees,
            external_axes_units=external_axes_units, speed_mode=speed_mode,
            speed_value=speed_value, acceleration_percent=acceleration_percent,
            deceleration_percent=deceleration_percent, ramp_percent=ramp_percent,
            wrist_configuration=wrist_configuration, loop_modes=loop_modes,
            center_translation_millimeters=center_translation_millimeters,
            plane_translation_millimeters=plane_translation_millimeters,
            timeout=timeout,
        )
    def move_spline(self, *, segments, timeout):
        return self._finite_motion(
            _p.MAIN_MOVE_SPLINE_COMMAND_CONTRACT,
            {"segments": segments},
            timeout,
        )


class ARAuxiliary(_AutomationSession):
    """Own one role-validated Nano or Mega JSON identity, reader, and close."""
    def __init__(self):
        super().__init__()
        self._identity = None
    @classmethod
    def connect(
        cls, serial_port, *, expected_board, request_timeout,
        close_timeout=_p.JSON_COORDINATOR_DEFAULT_CLOSE_TIMEOUT_SECONDS,
    ):
        try:
            if type(expected_board) is not str or expected_board not in ("nano", "mega"):
                raise ValueError("expected auxiliary board is invalid")
            session, request_timeout = cls(), _timeout(request_timeout, "request timeout")
            close_timeout = _timeout(close_timeout, "close timeout")
            client = _p.JsonAuxiliaryControllerClient(serial_port)
        except BaseException as error:
            _close_unowned(serial_port, error)
        _AutomationSession._activate(session, client, serial_port, close_timeout)
        try:
            ticket = client.request_hello(timeout=request_timeout)
            terminal = _auxiliary_terminal(client, ticket)
            if client.release_reader() is not True:
                raise _p.JsonSessionProtocolError("auxiliary reader release failed")
            if terminal.response.status != "completed":
                raise AutomationCommandError(_p.AUXILIARY_CONTROLLER, terminal)
            if terminal.parsed_result.board != expected_board:
                raise AutomationStateError("auxiliary board identity mismatch")
        except BaseException as error:
            session._finish(error)
        session._identity = terminal.parsed_result
        return session
    @property
    def identity(self):
        self._active()
        return self._identity
    @property
    def board_profile(self):
        self._active()
        return self._identity.board
    def _command_owned(self, client, command, params, timeout):
        ticket = client.request_command(command, params, timeout=timeout)
        terminal = _auxiliary_terminal(client, ticket)
        if client.release_reader() is not True:
            raise _p.JsonSessionProtocolError("auxiliary reader release failed")
        if terminal.response.status != "completed":
            raise AutomationCommandError(_p.AUXILIARY_CONTROLLER, terminal)
        return terminal.parsed_result
    def _wait_input_owned(self, client, params):
        timeout_seconds = params["timeout_seconds"]
        request_timeout = (
            timeout_seconds
            + 2 * _p.JSON_SERIAL_DEFAULT_FRAME_TIMEOUT_SECONDS
        )
        return self._command_owned(
            client,
            "wait_input",
            params,
            request_timeout,
        )
    def _validate_wait_input_owned(self, client, params):
        client.validate_command("wait_input", params)
    def _validate_command_owned(self, client, command, params, _timeout_value):
        client.validate_command(command, params)
    def _command(self, command, params, timeout):
        self._active()
        timeout = _timeout(timeout, "request timeout")
        return self._call(
            self._command_owned, command, params, timeout,
            preflight=self._validate_command_owned,
        )
    def read_input(self, pin, *, timeout):
        return self._command("input_read", {"pin": pin}, timeout)
    def set_servo(self, channel, position, *, timeout):
        return self._command(
            "servo", {"channel": channel, "position": position}, timeout
        )
    def set_output(self, pin, state, *, timeout):
        return self._command("set_output", {"pin": pin, "state": state}, timeout)
    def wait_input(self, pin, state, *, timeout_seconds):
        self._active()
        params = {
            "pin": pin,
            "state": state,
            "timeout_seconds": timeout_seconds,
        }
        return self._call(
            self._wait_input_owned,
            params,
            preflight=self._validate_wait_input_owned,
        )
    def test_gripper_amps(self, *, timeout):
        return self._command("test_gripper_amps", {}, timeout)
    def detach_gripper(self, *, timeout):
        return self._command("gripper_detach", {}, timeout)


__all__ = ("ARAuxiliary", "ARRobot", "AutomationCleanupError",
           "AutomationCommandError", "AutomationStateError")
