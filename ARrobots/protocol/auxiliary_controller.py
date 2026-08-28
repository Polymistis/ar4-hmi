"""Device-bound client for a JSON-only auxiliary controller."""

from dataclasses import dataclass, field
import threading
import time

from .catalog import AUXILIARY_CONTROLLER
from .coordinator import (
    JSON_COORDINATOR_DEFAULT_CLOSE_TIMEOUT_SECONDS,
    JSON_COORDINATOR_MAXIMUM_DELIVERIES,
    JsonSerialSessionCoordinator,
)
from .messages import JSON_PROTOCOL_MAXIMUM_PAYLOAD_BYTES, Response
from .schemas import (
    AUXILIARY_GRIPPER_DETACH_COMMAND_CONTRACT,
    AUXILIARY_HELLO_COMMAND_CONTRACT,
    AUXILIARY_INPUT_READ_COMMAND_CONTRACT,
    AUXILIARY_SERVO_COMMAND_CONTRACT,
    AUXILIARY_SET_OUTPUT_COMMAND_CONTRACT,
    AUXILIARY_STOP_COMMAND_CONTRACT,
    AUXILIARY_TEST_GRIPPER_AMPS_COMMAND_CONTRACT,
    AUXILIARY_WAIT_INPUT_COMMAND_CONTRACT,
    JsonCommandSchemaError,
    parse_auxiliary_gripper_amps_result,
    parse_auxiliary_hello_result,
    parse_auxiliary_input_read_result,
)
from .session import (
    JSON_SESSION_MAXIMUM_PENDING_REQUESTS,
    JSON_SESSION_MAXIMUM_TELEMETRY_STREAMS,
)
from .transport import (
    JSON_SERIAL_DEFAULT_DRAIN_POLL_INTERVAL_SECONDS,
    JSON_SERIAL_DEFAULT_FRAME_TIMEOUT_SECONDS,
    JSON_SERIAL_DEFAULT_POLL_INTERVAL_SECONDS,
    JSON_SERIAL_DEFAULT_READ_CHUNK_BYTES,
    JSON_SERIAL_DEFAULT_WRITE_TIMEOUT_SECONDS,
)


def _parse_empty_result(result):
    return None


_AUXILIARY_COMMANDS = (
    (AUXILIARY_HELLO_COMMAND_CONTRACT, parse_auxiliary_hello_result),
    (AUXILIARY_SERVO_COMMAND_CONTRACT, _parse_empty_result),
    (AUXILIARY_INPUT_READ_COMMAND_CONTRACT, parse_auxiliary_input_read_result),
    (AUXILIARY_SET_OUTPUT_COMMAND_CONTRACT, _parse_empty_result),
    (AUXILIARY_WAIT_INPUT_COMMAND_CONTRACT, _parse_empty_result),
    (
        AUXILIARY_TEST_GRIPPER_AMPS_COMMAND_CONTRACT,
        parse_auxiliary_gripper_amps_result,
    ),
    (AUXILIARY_STOP_COMMAND_CONTRACT, _parse_empty_result),
    (AUXILIARY_GRIPPER_DETACH_COMMAND_CONTRACT, _parse_empty_result),
)

AUXILIARY_JSON_COMMAND_CONTRACTS = tuple(
    contract for contract, _parser in _AUXILIARY_COMMANDS
)
_AUXILIARY_COMMANDS_BY_NAME = {
    contract.name: (contract, parser)
    for contract, parser in _AUXILIARY_COMMANDS
}
_AUXILIARY_SEMANTIC_COMMANDS = frozenset(
    name for name in _AUXILIARY_COMMANDS_BY_NAME if name != "hello"
)


@dataclass(frozen=True)
class JsonAuxiliaryControllerTerminal:
    """One validated auxiliary terminal and optional typed success value."""

    response: Response
    parsed_result: object = field(init=False)

    def __init__(self, response):
        if (
            type(response) is not Response
            or not response.terminal
            or response.cmd not in _AUXILIARY_COMMANDS_BY_NAME
        ):
            raise JsonCommandSchemaError(
                "auxiliary-controller terminal response is invalid"
            )
        contract, parser = _AUXILIARY_COMMANDS_BY_NAME[response.cmd]
        contract.response_validator(response)
        parsed_result = (
            parser(response.result)
            if response.status == "completed"
            else None
        )
        object.__setattr__(self, "response", response)
        object.__setattr__(self, "parsed_result", parsed_result)

    @property
    def failure(self):
        return self.response.error


class JsonAuxiliaryControllerClientStateError(RuntimeError):
    """Auxiliary session state rejects a semantic operation."""


class JsonAuxiliaryControllerClient:
    """Own an open Nano or Mega serial handle through JSON v1 contracts."""

    def __init__(
        self,
        serial_port,
        *,
        clock=time.monotonic,
        clock_resolution=None,
        deadline_scheduler=None,
        sleeper=time.sleep,
        poll_interval=JSON_SERIAL_DEFAULT_POLL_INTERVAL_SECONDS,
        frame_timeout=JSON_SERIAL_DEFAULT_FRAME_TIMEOUT_SECONDS,
        write_timeout=JSON_SERIAL_DEFAULT_WRITE_TIMEOUT_SECONDS,
        read_chunk_bytes=JSON_SERIAL_DEFAULT_READ_CHUNK_BYTES,
        drain_poll_interval=(
            JSON_SERIAL_DEFAULT_DRAIN_POLL_INTERVAL_SECONDS
        ),
        maximum_pending_requests=JSON_SESSION_MAXIMUM_PENDING_REQUESTS,
        maximum_telemetry_streams=JSON_SESSION_MAXIMUM_TELEMETRY_STREAMS,
        delivery_capacity=JSON_COORDINATOR_MAXIMUM_DELIVERIES,
    ):
        self._coordinator = JsonSerialSessionCoordinator(
            serial_port,
            AUXILIARY_CONTROLLER,
            AUXILIARY_JSON_COMMAND_CONTRACTS,
            clock=clock,
            clock_resolution=clock_resolution,
            deadline_scheduler=deadline_scheduler,
            sleeper=sleeper,
            poll_interval=poll_interval,
            frame_timeout=frame_timeout,
            write_timeout=write_timeout,
            read_chunk_bytes=read_chunk_bytes,
            drain_poll_interval=drain_poll_interval,
            maximum_pending_requests=maximum_pending_requests,
            maximum_telemetry_streams=maximum_telemetry_streams,
            delivery_capacity=delivery_capacity,
        )
        self._state_lock = threading.RLock()
        self._hello_ticket = None
        self._binding = None
        self._submission_active = False
        self._serial_port = serial_port

    @property
    def serial_port(self):
        return self._serial_port

    @property
    def session_binding(self):
        with self._state_lock:
            return self._binding

    @property
    def session_ready(self):
        with self._state_lock:
            return self._binding is not None and not self._submission_active

    @property
    def quarantined(self):
        return self._coordinator.quarantined

    @property
    def quarantine_reason(self):
        return self._coordinator.quarantine_reason

    @property
    def closed(self):
        return self._coordinator.closed

    @property
    def closing(self):
        return self._coordinator.closing

    @property
    def delivery_count(self):
        return self._coordinator.delivery_count

    @property
    def pending_tickets(self):
        return self._coordinator.pending_tickets

    @property
    def deadline_cleanup_count(self):
        return self._coordinator.deadline_cleanup_count

    @property
    def reader_owner(self):
        return self._coordinator.reader_owner

    def _begin_submission(self, command):
        with self._state_lock:
            if command not in _AUXILIARY_COMMANDS_BY_NAME:
                raise JsonAuxiliaryControllerClientStateError(
                    "auxiliary-controller command is inactive"
                )
            if self._submission_active:
                raise JsonAuxiliaryControllerClientStateError(
                    "auxiliary-controller request submission is active"
                )
            if command == "hello":
                if self._binding is not None or self._hello_ticket is not None:
                    raise JsonAuxiliaryControllerClientStateError(
                        "auxiliary-controller session is already established"
                    )
            elif self._binding is None:
                raise JsonAuxiliaryControllerClientStateError(
                    "auxiliary-controller session is not established"
                )
            waits = tuple(
                ticket
                for ticket in self._coordinator.pending_tickets
                if ticket.command == "wait_input"
            )
            if command == "stop" and len(waits) > 1:
                raise JsonAuxiliaryControllerClientStateError(
                    "auxiliary stop request ownership is invalid"
                )
            if command != "stop" and waits:
                raise JsonAuxiliaryControllerClientStateError(
                    "auxiliary input wait is active"
                )
            self._submission_active = True

    def _end_submission(self):
        with self._state_lock:
            self._submission_active = False

    def _validate_board_params(self, command, params):
        binding = self._binding
        if binding is None or command in (
            "gripper_detach",
            "hello",
            "stop",
            "test_gripper_amps",
        ):
            return
        board = binding.board
        if command == "servo":
            maximum = 5 if board == "nano" else 6
            if not 0 <= params["channel"] <= maximum:
                raise JsonCommandSchemaError(
                    "servo channel is invalid for the bound auxiliary board"
                )
        elif command in ("input_read", "wait_input"):
            maximum = 7 if board == "nano" else 27
            if not 2 <= params["pin"] <= maximum:
                raise JsonCommandSchemaError(
                    "input pin is invalid for the bound auxiliary board"
                )
        elif command == "set_output":
            minimum, maximum = (8, 13) if board == "nano" else (28, 53)
            if not minimum <= params["pin"] <= maximum:
                raise JsonCommandSchemaError(
                    "output pin is invalid for the bound auxiliary board"
                )

    def request_hello(self, *, timeout, write_admission=None):
        self._begin_submission("hello")
        try:
            ticket = self._coordinator.submit(
                "hello",
                {},
                timeout=timeout,
                write_admission=write_admission,
                maximum_payload_bytes=JSON_PROTOCOL_MAXIMUM_PAYLOAD_BYTES,
            )
            with self._state_lock:
                self._hello_ticket = ticket
            return ticket
        finally:
            self._end_submission()

    def request_command(
        self,
        command,
        params,
        *,
        timeout,
        write_admission=None,
    ):
        """Submit one active auxiliary semantic command."""
        if command not in _AUXILIARY_SEMANTIC_COMMANDS:
            raise JsonCommandSchemaError(
                "auxiliary-controller semantic command is invalid"
            )
        self._begin_submission(command)
        try:
            contract, _parser = _AUXILIARY_COMMANDS_BY_NAME[command]
            contract.request_validator(params)
            self._validate_board_params(command, params)
            return self._coordinator.submit(
                command,
                params,
                timeout=timeout,
                write_admission=write_admission,
                maximum_payload_bytes=(
                    self._binding.protocol.maximum_payload_bytes
                ),
            )
        finally:
            self._end_submission()

    def poll(self):
        return self._coordinator.poll()

    def release_reader(self):
        with self._state_lock:
            if self._submission_active:
                raise JsonAuxiliaryControllerClientStateError(
                    "auxiliary reader release rejected during submission"
                )
        return self._coordinator.release_reader()

    def pop_delivery(self):
        return self._coordinator.pop_delivery()

    def snapshot(self, ticket):
        return self._coordinator.snapshot(ticket)

    def take_terminal(self, ticket):
        return JsonAuxiliaryControllerTerminal(
            self._coordinator.take_terminal(ticket)
        )

    def acknowledge_terminal(self, ticket):
        terminal = None
        with self._state_lock:
            if ticket is self._hello_ticket:
                terminal = self.take_terminal(ticket)
        self._coordinator.acknowledge_terminal(ticket)
        if terminal is not None:
            with self._state_lock:
                self._hello_ticket = None
                self._binding = (
                    terminal.parsed_result
                    if terminal.response.status == "completed"
                    else None
                )

    def expire(self):
        self._coordinator.expire()

    def retry_deadline_cleanup(self):
        return self._coordinator.retry_deadline_cleanup()

    def close(
        self,
        *,
        timeout=JSON_COORDINATOR_DEFAULT_CLOSE_TIMEOUT_SECONDS,
    ):
        return self._coordinator.close(timeout=timeout)


__all__ = (
    "AUXILIARY_JSON_COMMAND_CONTRACTS",
    "JsonAuxiliaryControllerClient",
    "JsonAuxiliaryControllerClientStateError",
    "JsonAuxiliaryControllerTerminal",
)
