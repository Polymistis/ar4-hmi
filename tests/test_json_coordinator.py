import threading
import unittest

from ARrobots.protocol import (
    MAIN_CONTROLLER,
    Event,
    JsonCommandContract,
    JsonEventDelivery,
    JsonResponseDelivery,
    JsonSerialSessionCoordinator,
    JsonSerialSessionCoordinatorAdmissionError,
    JsonSerialSessionCoordinatorCloseError,
    JsonSerialSessionCoordinatorStateError,
    JsonSerialTransportQuarantinedError,
    JsonSessionAdmissionError,
    JsonSessionClosedError,
    JsonSessionDeadlineError,
    JsonSessionProtocolError,
    JsonSessionQuarantinedError,
    JsonSessionTimeoutError,
    Response,
    encode_message,
)


class FakeClock:
    def __init__(self, value=10.0):
        self.value = value

    def __call__(self):
        return self.value


class ManualDeadlineScheduler:
    def __init__(self):
        self.registrations = []
        self.cancel_error = None

    def __call__(self, deadline, callback, publish_owner):
        registration = {
            "active": True,
            "callback": callback,
            "deadline": deadline,
        }

        def cancel():
            if self.cancel_error is not None:
                raise self.cancel_error
            registration["active"] = False

        self.registrations.append(registration)
        publish_owner(registration, cancel)


class FakeSerial:
    def __init__(self, reads=()):
        self._is_open = True
        self.is_open_error = None
        self.timeout = 3.0
        self.write_timeout = 4.0
        self.out_waiting = 0
        self.reads = list(reads)
        self.writes = []
        self.close_calls = 0
        self.close_error = None
        self.leave_open = False
        self.read_started = None
        self.read_release = None
        self.write_started = None
        self.write_release = None

    @property
    def is_open(self):
        if self.is_open_error is not None:
            raise self.is_open_error
        return self._is_open

    @is_open.setter
    def is_open(self, value):
        self._is_open = value

    @property
    def in_waiting(self):
        if not self.reads:
            return 0
        return len(self.reads[0])

    def read(self, size):
        if self.read_started is not None:
            self.read_started.set()
        if self.read_release is not None:
            self.read_release.wait()
        if not self.reads:
            return b""
        chunk = self.reads.pop(0)
        if len(chunk) <= size:
            return chunk
        self.reads.insert(0, chunk[size:])
        return chunk[:size]

    def write(self, frame):
        if self.write_started is not None:
            self.write_started.set()
        if self.write_release is not None:
            self.write_release.wait()
        self.writes.append(frame)
        return len(frame)

    def close(self):
        self.close_calls += 1
        if not self.leave_open:
            self.is_open = False
        if self.close_error is not None:
            raise self.close_error


class JsonSerialSessionCoordinatorTests(unittest.TestCase):
    def make_coordinator(
        self,
        serial_port=None,
        *,
        delivery_capacity=128,
        additional_commands=(),
    ):
        if serial_port is None:
            serial_port = FakeSerial()
        clock = FakeClock()
        scheduler = ManualDeadlineScheduler()
        contract = JsonCommandContract(
            "hello",
            lambda _params: None,
            lambda _response: None,
        )
        contracts = (contract,) + tuple(
            JsonCommandContract(
                command,
                lambda _params: None,
                lambda _response: None,
            )
            for command in additional_commands
        )
        coordinator = JsonSerialSessionCoordinator(
            serial_port,
            MAIN_CONTROLLER,
            contracts,
            clock=clock,
            clock_resolution=0.0,
            deadline_scheduler=scheduler,
            delivery_capacity=delivery_capacity,
        )
        return coordinator, serial_port, clock, scheduler

    def test_construction_requires_serial_close_ownership(self):
        class NoCloseSerial(FakeSerial):
            close = None

        with self.assertRaises(
            JsonSerialSessionCoordinatorAdmissionError
        ):
            self.make_coordinator(NoCloseSerial())
        for capacity in (0, 129, True):
            with self.subTest(capacity=capacity):
                with self.assertRaises(
                    JsonSerialSessionCoordinatorAdmissionError
                ):
                    self.make_coordinator(
                        delivery_capacity=capacity
                    )
    def test_poll_acknowledges_session_before_transport_frame(self):
        coordinator, serial_port, _clock, _scheduler = self.make_coordinator()
        serial_port.reads.append(
            encode_message(Event(0, "emergency_stop", {"asserted": True}))
        )
        acknowledgement_order = []
        acknowledge_session = coordinator._session.acknowledge_public_return
        acknowledge_transport = coordinator._transport.acknowledge_frame

        def record_session_acknowledgement(delivery):
            self.assertEqual(coordinator.delivery_count, 1)
            acknowledgement_order.append("session")
            return acknowledge_session(delivery)

        def record_transport_acknowledgement(frame):
            self.assertEqual(coordinator.delivery_count, 1)
            acknowledgement_order.append("transport")
            return acknowledge_transport(frame)

        coordinator._session.acknowledge_public_return = (
            record_session_acknowledgement
        )
        coordinator._transport.acknowledge_frame = (
            record_transport_acknowledgement
        )

        self.assertTrue(coordinator.poll())

        self.assertEqual(acknowledgement_order, ["session", "transport"])
        self.assertFalse(coordinator.quarantined)

    def test_poll_retries_failed_operation_cleanup(self):
        coordinator, serial_port, _clock, _scheduler = self.make_coordinator()
        serial_port.reads.append(
            encode_message(Event(0, "emergency_stop", {"asserted": True}))
        )
        force_end_operation = coordinator._force_end_operation
        cleanup_calls = 0

        def fail_first_cleanup(token):
            nonlocal cleanup_calls
            cleanup_calls += 1
            if cleanup_calls == 1:
                raise KeyboardInterrupt()
            return force_end_operation(token)

        coordinator._force_end_operation = fail_first_cleanup
        try:
            with self.assertRaises(KeyboardInterrupt):
                coordinator.poll()
        finally:
            coordinator._force_end_operation = force_end_operation

        self.assertGreater(cleanup_calls, 1)
        self.assertEqual(coordinator.delivery_count, 1)
        self.assertFalse(coordinator.quarantined)
        self.assertIsNotNone(coordinator.pop_delivery())
        self.assertTrue(coordinator.release_reader())
        ticket = coordinator.submit("hello", {}, timeout=2.0)
        self.assertEqual(ticket.command, "hello")

    def test_pending_inbound_frame_blocks_submission(self):
        coordinator, serial_port, _clock, _scheduler = self.make_coordinator()
        serial_port.reads.append(
            encode_message(Event(0, "emergency_stop", {"asserted": True}))
        )
        receive_started = threading.Event()
        resume_receive = threading.Event()
        receive = coordinator._session.receive
        poll_results = []
        poll_failures = []

        def hold_receive(frame):
            receive_started.set()
            if not resume_receive.wait(timeout=2):
                raise TimeoutError("session receive was not resumed")
            return receive(frame)

        def poll_frame():
            try:
                poll_results.append(coordinator.poll())
                poll_results.append(coordinator.release_reader())
            except BaseException as exc:
                poll_failures.append(exc)

        coordinator._session.receive = hold_receive
        worker = threading.Thread(target=poll_frame)
        worker.start()
        self.assertTrue(receive_started.wait(timeout=1))
        try:
            with self.assertRaisesRegex(
                JsonSerialSessionCoordinatorStateError,
                "rejected during inbound frame handoff",
            ):
                coordinator.submit("hello", {}, timeout=2.0)
            self.assertEqual(serial_port.writes, [])
        finally:
            resume_receive.set()
        worker.join(timeout=2)

        self.assertFalse(worker.is_alive())
        self.assertEqual(poll_failures, [])
        self.assertEqual(poll_results, [True, True])
        self.assertFalse(coordinator.quarantined)
        self.assertIsNone(coordinator.reader_owner)
        self.assertIsNotNone(coordinator.pop_delivery())

    def test_failed_frame_quarantine_mirror_remains_retryable(self):
        coordinator, serial_port, _clock, _scheduler = self.make_coordinator()
        serial_port.reads.append(
            encode_message(Event(0, "emergency_stop", {"asserted": True}))
        )
        quarantine_session = coordinator._session.quarantine
        quarantine_transport = coordinator._transport.quarantine
        transport_quarantine_deferred = True

        def reject_receive(_frame):
            raise JsonSessionAdmissionError("receive admission failed")

        def reject_session_quarantine(_reason):
            raise JsonSessionAdmissionError("session dependency is active")

        def defer_first_transport_quarantine(reason):
            nonlocal transport_quarantine_deferred
            if transport_quarantine_deferred:
                transport_quarantine_deferred = False
                raise KeyboardInterrupt()
            return quarantine_transport(reason)

        coordinator._session.receive = reject_receive
        coordinator._session.quarantine = reject_session_quarantine
        coordinator._transport.quarantine = defer_first_transport_quarantine
        with self.assertRaises(KeyboardInterrupt):
            coordinator.poll()

        self.assertTrue(coordinator.quarantined)
        self.assertFalse(coordinator._session.quarantined)
        self.assertTrue(coordinator._transport.quarantined)
        self.assertIn(
            "session quarantine mirror remains pending after "
            "JsonSessionAdmissionError",
            coordinator.quarantine_reason,
        )
        with self.assertRaisesRegex(
            JsonSerialSessionCoordinatorStateError,
            "coordinator is quarantined",
        ):
            coordinator.submit("hello", {}, timeout=2.0)
        self.assertEqual(serial_port.writes, [])

        coordinator._session.quarantine = quarantine_session
        coordinator._coordinate_quarantine()

        self.assertTrue(coordinator._session.quarantined)
        self.assertNotIn(
            "mirror remains pending",
            coordinator.quarantine_reason,
        )

    def test_reader_identity_transfers_with_event_sequence_state(self):
        coordinator, serial_port, _clock, _scheduler = self.make_coordinator()
        serial_port.reads.append(
            encode_message(Event(7, "emergency_stop", {"asserted": True}))
        )

        self.assertTrue(coordinator.poll())
        self.assertIs(coordinator.reader_owner, threading.current_thread())
        serial_port.reads.append(
            encode_message(Event(8, "emergency_stop", {"asserted": True}))
        )
        self.assertTrue(coordinator.release_reader())
        self.assertIsNone(coordinator.reader_owner)
        results = []
        errors = []

        def poll_from_next_owner():
            try:
                results.append(coordinator.poll())
                results.append(coordinator.release_reader())
            except BaseException as exc:
                errors.append(exc)

        worker = threading.Thread(target=poll_from_next_owner)
        worker.start()
        worker.join(timeout=2)

        self.assertFalse(worker.is_alive())
        self.assertEqual(errors, [])
        self.assertEqual(results, [True, True])
        first = coordinator.pop_delivery()
        second = coordinator.pop_delivery()
        self.assertIsNone(first.sequence_contiguous)
        self.assertTrue(second.sequence_contiguous)
        self.assertIsNone(coordinator.reader_owner)
        self.assertFalse(coordinator.quarantined)

    def test_transport_only_reader_binding_releases_after_empty_poll(self):
        coordinator, _serial_port, _clock, _scheduler = self.make_coordinator()

        self.assertFalse(coordinator.poll())
        self.assertIs(coordinator.reader_owner, threading.current_thread())
        self.assertTrue(coordinator.release_reader())
        self.assertIsNone(coordinator.reader_owner)
        self.assertFalse(coordinator.release_reader())
        self.assertFalse(coordinator.quarantined)

    def test_reader_release_rejects_foreign_owner_without_quarantine(self):
        coordinator, serial_port, _clock, _scheduler = self.make_coordinator()
        serial_port.reads.append(
            encode_message(Event(0, "emergency_stop", {"asserted": True}))
        )
        self.assertTrue(coordinator.poll())
        errors = []

        def release_from_foreign_thread():
            try:
                coordinator.release_reader()
            except BaseException as exc:
                errors.append(exc)

        worker = threading.Thread(target=release_from_foreign_thread)
        worker.start()
        worker.join(timeout=2)

        self.assertFalse(worker.is_alive())
        self.assertEqual(len(errors), 1)
        self.assertIsInstance(
            errors[0],
            JsonSerialSessionCoordinatorAdmissionError,
        )
        self.assertFalse(coordinator.quarantined)
        self.assertTrue(coordinator.release_reader())

    def test_reader_release_reservation_rejects_poll_and_submission(self):
        coordinator, serial_port, _clock, _scheduler = self.make_coordinator()
        serial_port.reads.append(
            encode_message(Event(0, "emergency_stop", {"asserted": True}))
        )
        release_started = threading.Event()
        finish_release = threading.Event()
        results = []
        errors = []
        release_transport = coordinator._transport.release_reader

        def wait_after_transport_release():
            released = release_transport()
            release_started.set()
            if not finish_release.wait(timeout=2):
                raise TimeoutError("reader release was not resumed")
            return released

        def transfer_reader():
            try:
                results.append(coordinator.poll())
                coordinator._transport.release_reader = (
                    wait_after_transport_release
                )
                results.append(coordinator.release_reader())
            except BaseException as exc:
                errors.append(exc)

        worker = threading.Thread(target=transfer_reader)
        worker.start()
        self.assertTrue(release_started.wait(timeout=1))
        try:
            with self.assertRaisesRegex(
                JsonSerialSessionCoordinatorStateError,
                "rejected during reader release",
            ):
                coordinator.poll()
            with self.assertRaisesRegex(
                JsonSerialSessionCoordinatorStateError,
                "rejected during reader release",
            ):
                coordinator.submit("hello", {}, timeout=2.0)
            with self.assertRaisesRegex(
                JsonSerialSessionCoordinatorStateError,
                "rejected during reader release",
            ):
                _owner = coordinator.reader_owner
        finally:
            finish_release.set()
        worker.join(timeout=2)

        self.assertFalse(worker.is_alive())
        self.assertEqual(errors, [])
        self.assertEqual(results, [True, True])
        self.assertEqual(serial_port.writes, [])
        self.assertEqual(serial_port.reads, [])
        self.assertEqual(coordinator._active_operations, set())
        self.assertFalse(coordinator.quarantined)

    def test_interrupted_reader_release_quarantines_split_ownership(self):
        coordinator, serial_port, _clock, _scheduler = self.make_coordinator()
        serial_port.reads.append(
            encode_message(Event(0, "emergency_stop", {"asserted": True}))
        )
        self.assertTrue(coordinator.poll())
        release_session = coordinator._session.release_reader

        def interrupt_after_session_release():
            release_session()
            raise KeyboardInterrupt()

        coordinator._session.release_reader = (
            interrupt_after_session_release
        )
        with self.assertRaises(KeyboardInterrupt):
            coordinator.release_reader()

        self.assertTrue(coordinator.quarantined)
        self.assertTrue(coordinator._session.quarantined)
        self.assertTrue(coordinator._transport.quarantined)
        self.assertIn("reader release", coordinator.quarantine_reason)

    def test_successful_release_postcleanup_interruption_preserves_successors(self):
        coordinator, serial_port, _clock, _scheduler = self.make_coordinator()
        serial_port.reads.append(
            encode_message(Event(0, "emergency_stop", {"asserted": True}))
        )
        force_end_operation = coordinator._force_end_operation
        cleanup_started = threading.Event()
        resume_cleanup = threading.Event()
        cleanup_calls = 0
        results = []
        errors = []

        def interrupt_first_cleanup(token):
            nonlocal cleanup_calls
            force_end_operation(token)
            cleanup_calls += 1
            if cleanup_calls == 1:
                cleanup_started.set()
                if not resume_cleanup.wait(timeout=2):
                    raise TimeoutError("reader cleanup was not resumed")
                raise KeyboardInterrupt()

        def transfer_reader():
            try:
                results.append(coordinator.poll())
                coordinator._force_end_operation = interrupt_first_cleanup
                results.append(coordinator.release_reader())
            except BaseException as exc:
                errors.append(exc)

        worker = threading.Thread(target=transfer_reader)
        worker.start()
        self.assertTrue(cleanup_started.wait(timeout=1))

        serial_port.reads.append(
            encode_message(Event(1, "emergency_stop", {"asserted": True}))
        )
        self.assertTrue(coordinator.poll())
        ticket = coordinator.submit("hello", {}, timeout=2.0)
        self.assertEqual(ticket.command, "hello")
        self.assertEqual(len(serial_port.writes), 1)
        self.assertIs(coordinator.reader_owner, threading.current_thread())
        self.assertFalse(coordinator.quarantined)

        resume_cleanup.set()
        worker.join(timeout=2)

        coordinator._force_end_operation = force_end_operation
        self.assertFalse(worker.is_alive())
        self.assertGreaterEqual(cleanup_calls, 2)
        self.assertEqual(results, [True])
        self.assertEqual(len(errors), 1)
        self.assertIsInstance(errors[0], KeyboardInterrupt)
        self.assertIsNone(coordinator._reader_release_operation)
        self.assertEqual(coordinator._active_operations, set())
        self.assertFalse(coordinator.quarantined)
        first = coordinator.pop_delivery()
        second = coordinator.pop_delivery()
        self.assertIsNone(first.sequence_contiguous)
        self.assertTrue(second.sequence_contiguous)
        self.assertTrue(coordinator.release_reader())

    def test_failed_release_postcleanup_blocks_successor_admission(self):
        coordinator, serial_port, _clock, _scheduler = self.make_coordinator()
        serial_port.reads.append(
            encode_message(Event(0, "emergency_stop", {"asserted": True}))
        )
        force_end_operation = coordinator._force_end_operation
        cleanup_started = threading.Event()
        resume_cleanup = threading.Event()
        cleanup_calls = 0
        results = []
        errors = []

        def interrupt_before_transport_release():
            raise KeyboardInterrupt()

        def interrupt_first_cleanup(token):
            nonlocal cleanup_calls
            force_end_operation(token)
            cleanup_calls += 1
            if cleanup_calls == 1:
                cleanup_started.set()
                if not resume_cleanup.wait(timeout=2):
                    raise TimeoutError("reader cleanup was not resumed")
                raise KeyboardInterrupt()

        def transfer_reader():
            try:
                results.append(coordinator.poll())
                coordinator._force_end_operation = interrupt_first_cleanup
                coordinator.release_reader()
            except BaseException as exc:
                errors.append(exc)

        coordinator._transport.release_reader = (
            interrupt_before_transport_release
        )
        worker = threading.Thread(target=transfer_reader)
        worker.start()
        self.assertTrue(cleanup_started.wait(timeout=1))

        with self.assertRaisesRegex(
            JsonSerialSessionCoordinatorStateError,
            "coordinator is quarantined",
        ):
            coordinator.poll()
        with self.assertRaisesRegex(
            JsonSerialSessionCoordinatorStateError,
            "coordinator is quarantined",
        ):
            coordinator.submit("hello", {}, timeout=2.0)
        self.assertEqual(serial_port.writes, [])

        resume_cleanup.set()
        worker.join(timeout=2)
        coordinator._force_end_operation = force_end_operation

        self.assertFalse(worker.is_alive())
        self.assertGreaterEqual(cleanup_calls, 2)
        self.assertEqual(results, [True])
        self.assertEqual(len(errors), 1)
        self.assertIsInstance(errors[0], KeyboardInterrupt)
        self.assertIsNone(coordinator._reader_release_operation)
        self.assertEqual(coordinator._active_operations, set())
        self.assertTrue(coordinator.quarantined)
        self.assertTrue(coordinator._session.quarantined)
        self.assertTrue(coordinator._transport.quarantined)

    def test_reader_release_retains_failed_session_quarantine_mirror(self):
        coordinator, serial_port, _clock, _scheduler = self.make_coordinator()
        serial_port.reads.append(
            encode_message(Event(0, "emergency_stop", {"asserted": True}))
        )
        self.assertTrue(coordinator.poll())
        quarantine_session = coordinator._session.quarantine
        quarantine_attempts = 0

        def interrupt_before_transport_release():
            raise KeyboardInterrupt()

        def reject_session_quarantine(_reason):
            nonlocal quarantine_attempts
            quarantine_attempts += 1
            raise JsonSessionAdmissionError("session dependency is active")

        coordinator._transport.release_reader = (
            interrupt_before_transport_release
        )
        coordinator._session.quarantine = reject_session_quarantine
        with self.assertRaises(KeyboardInterrupt):
            coordinator.release_reader()

        self.assertEqual(quarantine_attempts, 1)
        self.assertFalse(coordinator._session.quarantined)
        self.assertTrue(coordinator._transport.quarantined)
        self.assertIn(
            "session quarantine mirror remains pending after "
            "JsonSessionAdmissionError",
            coordinator.quarantine_reason,
        )

        coordinator._session.quarantine = quarantine_session
        coordinator._coordinate_quarantine()
        self.assertTrue(coordinator._session.quarantined)
        self.assertNotIn("mirror remains pending", coordinator.quarantine_reason)

    def test_submit_uses_one_clock_domain_and_retains_ticket(self):
        coordinator, serial_port, _clock, scheduler = (
            self.make_coordinator()
        )

        ticket = coordinator.submit("hello", {}, timeout=2.0)

        self.assertEqual(ticket.deadline, 12.0)
        self.assertEqual(
            scheduler.registrations[0]["deadline"],
            ticket.deadline,
        )
        self.assertEqual(coordinator.pending_tickets, (ticket,))
        self.assertEqual(len(serial_port.writes), 1)

    def test_interrupted_ticket_acknowledgement_remains_recoverable(self):
        coordinator, _serial_port, _clock, _scheduler = (
            self.make_coordinator()
        )
        acknowledge = coordinator._session.acknowledge_public_return
        calls = 0

        def interrupt_once(result):
            nonlocal calls
            calls += 1
            if calls == 1:
                raise KeyboardInterrupt()
            return acknowledge(result)

        coordinator._session.acknowledge_public_return = interrupt_once
        with self.assertRaises(KeyboardInterrupt):
            coordinator.submit("hello", {}, timeout=2.0)

        self.assertEqual(calls, 2)
        self.assertEqual(len(coordinator.pending_tickets), 1)
        self.assertFalse(coordinator.quarantined)
        self.assertEqual(
            coordinator.snapshot(coordinator.pending_tickets[0]).accepted,
            None,
        )

    def test_interrupted_session_ticket_return_is_recovered(self):
        coordinator, _serial_port, _clock, _scheduler = (
            self.make_coordinator()
        )
        submit = coordinator._session.submit

        def lose_return(*args, **kwargs):
            submit(*args, **kwargs)
            raise KeyboardInterrupt()

        coordinator._session.submit = lose_return
        with self.assertRaises(KeyboardInterrupt):
            coordinator.submit("hello", {}, timeout=2.0)

        self.assertEqual(len(coordinator.pending_tickets), 1)
        self.assertFalse(coordinator.quarantined)
        self.assertIsNone(
            coordinator.snapshot(coordinator.pending_tickets[0]).accepted
        )

    def test_write_admission_rejects_recursive_coordinator_submission(self):
        coordinator, serial_port, _clock, _scheduler = self.make_coordinator()
        failures = []

        def submit_recursively():
            return coordinator.submit("hello", {}, timeout=2.0)

        def submit_outer():
            try:
                coordinator.submit(
                    "hello",
                    {},
                    timeout=2.0,
                    write_admission=submit_recursively,
                )
            except BaseException as exc:
                failures.append(exc)

        worker = threading.Thread(target=submit_outer, daemon=True)
        worker.start()
        worker.join(timeout=0.5)

        self.assertFalse(worker.is_alive())
        self.assertEqual(len(failures), 1)
        self.assertIsInstance(
            failures[0],
            JsonSerialSessionCoordinatorStateError,
        )
        self.assertIn("rejected during write admission", str(failures[0]))
        self.assertEqual(serial_port.writes, [])
        self.assertEqual(coordinator.pending_tickets, ())
        self.assertFalse(coordinator.quarantined)

    def test_write_admission_rejects_an_already_active_serial_poll(self):
        serial_port = FakeSerial()
        serial_port.read_started = threading.Event()
        serial_port.read_release = threading.Event()
        coordinator, _serial_port, _clock, scheduler = self.make_coordinator(
            serial_port
        )
        poll_results = []
        poll_failures = []

        def poll_serial():
            try:
                poll_results.append(coordinator.poll())
            except BaseException as exc:
                poll_failures.append(exc)

        worker = threading.Thread(target=poll_serial, daemon=True)
        worker.start()
        self.assertTrue(serial_port.read_started.wait(timeout=0.5))

        admission_calls = []

        def admit_write():
            admission_calls.append(True)
            return True

        try:
            with self.assertRaisesRegex(
                JsonSerialSessionCoordinatorStateError,
                "write-admitted submission requires an idle coordinator",
            ):
                coordinator.submit(
                    "hello",
                    {},
                    timeout=2.0,
                    write_admission=admit_write,
                )
        finally:
            serial_port.read_release.set()
        worker.join(timeout=0.5)

        self.assertFalse(worker.is_alive())
        self.assertEqual(poll_failures, [])
        self.assertEqual(poll_results, [False])
        self.assertEqual(admission_calls, [])
        self.assertEqual(serial_port.writes, [])
        self.assertEqual(scheduler.registrations, [])
        self.assertFalse(coordinator.quarantined)

        ticket = coordinator.submit(
            "hello",
            {},
            timeout=2.0,
            write_admission=admit_write,
        )
        self.assertEqual(admission_calls, [True])
        self.assertEqual(coordinator.pending_tickets, (ticket,))
        self.assertEqual(len(serial_port.writes), 1)

    def test_active_write_admission_rejects_a_concurrent_serial_poll(self):
        coordinator, serial_port, _clock, _scheduler = self.make_coordinator()
        admission_started = threading.Event()
        release_admission = threading.Event()
        submission_results = []
        submission_failures = []

        def admit_write():
            admission_started.set()
            if not release_admission.wait(timeout=0.5):
                raise TimeoutError("test did not release write admission")
            return True

        def submit_request():
            try:
                submission_results.append(
                    coordinator.submit(
                        "hello",
                        {},
                        timeout=2.0,
                        write_admission=admit_write,
                    )
                )
            except BaseException as exc:
                submission_failures.append(exc)

        worker = threading.Thread(target=submit_request, daemon=True)
        worker.start()
        self.assertTrue(admission_started.wait(timeout=0.5))

        try:
            with self.assertRaisesRegex(
                JsonSerialSessionCoordinatorStateError,
                "rejected during write admission",
            ):
                coordinator.poll()
        finally:
            release_admission.set()
        worker.join(timeout=0.5)

        self.assertFalse(worker.is_alive())
        self.assertEqual(submission_failures, [])
        self.assertEqual(len(submission_results), 1)
        self.assertEqual(
            coordinator.pending_tickets,
            (submission_results[0],),
        )
        self.assertEqual(len(serial_port.writes), 1)
        self.assertEqual(serial_port.reads, [])
        self.assertFalse(coordinator.quarantined)

    def test_interrupted_public_return_recovery_preserves_original_failure(self):
        coordinator, _serial_port, _clock, _scheduler = (
            self.make_coordinator()
        )
        submit = coordinator._session.submit
        recover = coordinator._session.recover_public_returns
        recovery_calls = 0
        operation_failure = RuntimeError("session return failed")
        recovery_failure = KeyboardInterrupt("recovery interrupted")
        root_failure = OSError("serial write failed")
        operation_failure.__cause__ = root_failure

        def lose_return(*args, **kwargs):
            submit(*args, **kwargs)
            raise operation_failure

        def interrupt_recovery_once():
            nonlocal recovery_calls
            recovery_calls += 1
            if recovery_calls == 1:
                raise recovery_failure
            return recover()

        coordinator._session.submit = lose_return
        coordinator._session.recover_public_returns = (
            interrupt_recovery_once
        )
        with self.assertRaises(RuntimeError) as raised:
            coordinator.submit("hello", {}, timeout=2.0)

        self.assertIs(raised.exception, operation_failure)
        self.assertIs(raised.exception.__cause__, root_failure)
        self.assertTrue(
            any(
                "KeyboardInterrupt" in note
                for note in raised.exception.__notes__
            )
        )
        self.assertEqual(recovery_calls, 2)
        self.assertEqual(len(coordinator.pending_tickets), 1)
        self.assertEqual(coordinator._active_operations, set())
        self.assertFalse(coordinator.quarantined)

    def test_failed_public_return_recovery_preserves_original_failure(self):
        coordinator, _serial_port, _clock, _scheduler = (
            self.make_coordinator()
        )
        operation_failure = RuntimeError("session return failed")
        root_failure = OSError("serial write failed")
        operation_failure.__cause__ = root_failure
        recovery_failure = JsonSerialSessionCoordinatorStateError(
            "recovery failed"
        )

        def fail_submit(*_args, **_kwargs):
            raise operation_failure

        def fail_recovery():
            raise recovery_failure

        coordinator._session.submit = fail_submit
        coordinator._recover_public_handoffs = fail_recovery

        with self.assertRaises(RuntimeError) as raised:
            coordinator.submit("hello", {}, timeout=2.0)

        self.assertIs(raised.exception, operation_failure)
        self.assertIs(raised.exception.__cause__, root_failure)
        self.assertIsNone(raised.exception.__context__)
        self.assertTrue(
            any(
                "JsonSerialSessionCoordinatorStateError" in note
                for note in raised.exception.__notes__
            )
        )
        self.assertEqual(coordinator._active_operations, set())

    def test_recovered_delivery_cannot_enter_the_queue_twice(self):
        coordinator, serial_port, _clock, _scheduler = (
            self.make_coordinator()
        )
        ticket = coordinator.submit("hello", {}, timeout=2.0)
        serial_port.reads.extend(
            (
                encode_message(
                    Response(
                        ticket.request_id,
                        "hello",
                        "completed",
                        result={},
                    )
                ),
                encode_message(Event(0, "state", {})),
            )
        )
        acknowledge = coordinator._acknowledge_public_handoff
        acknowledgement_calls = 0

        def interrupt_before_acknowledgement(result, reason):
            nonlocal acknowledgement_calls
            acknowledgement_calls += 1
            if acknowledgement_calls == 1:
                raise KeyboardInterrupt()
            return acknowledge(result, reason)

        coordinator._acknowledge_public_handoff = (
            interrupt_before_acknowledgement
        )
        with self.assertRaises(KeyboardInterrupt):
            coordinator.poll()
        self.assertEqual(coordinator.delivery_count, 1)

        with self.assertRaises(JsonSessionAdmissionError) as raised:
            coordinator.poll()

        self.assertIsNone(raised.exception.__cause__)
        self.assertTrue(
            any(
                "JsonSerialSessionCoordinatorStateError" in note
                for note in raised.exception.__notes__
            )
        )
        self.assertEqual(coordinator.delivery_count, 1)
        self.assertIsNotNone(coordinator.pop_delivery())
        self.assertIsNone(coordinator.pop_delivery())
        self.assertTrue(coordinator._session.quarantined)
        self.assertTrue(coordinator._transport.quarantined)

    def test_unrecoverable_ticket_handoff_quarantines_both_layers(self):
        coordinator, _serial_port, _clock, _scheduler = (
            self.make_coordinator()
        )

        def fail_acknowledgement(_result):
            raise RuntimeError("handoff failed")

        coordinator._session.acknowledge_public_return = (
            fail_acknowledgement
        )
        with self.assertRaisesRegex(
            JsonSerialSessionCoordinatorStateError,
            "acknowledgement failed",
        ) as raised:
            coordinator.submit("hello", {}, timeout=2.0)

        self.assertIsInstance(raised.exception.__cause__, RuntimeError)
        self.assertTrue(coordinator._session.quarantined)
        self.assertTrue(coordinator._transport.quarantined)
        self.assertTrue(coordinator.quarantined)

    def test_abandonment_failure_retains_diagnostic_and_quarantines(self):
        coordinator, _serial_port, _clock, _scheduler = (
            self.make_coordinator()
        )

        def fail_abandonment(_reason):
            raise RuntimeError("abandonment failed")

        coordinator._session.abandon_public_returns = fail_abandonment
        with self.assertRaises(
            JsonSerialSessionCoordinatorStateError
        ) as raised:
            coordinator._abandon_public_handoff(
                "forced public-return abandonment failure"
            )

        self.assertIsInstance(raised.exception.__cause__, RuntimeError)
        self.assertTrue(coordinator._transport.quarantined)
        self.assertIn("forced public-return", coordinator.quarantine_reason)

    def test_store_boundary_rejects_collisions_capacity_and_invalid_type(self):
        coordinator, _serial_port, _clock, _scheduler = (
            self.make_coordinator()
        )
        ticket = coordinator.submit("hello", {}, timeout=2.0)
        with self.assertRaises(JsonSerialSessionCoordinatorStateError):
            coordinator._store_public_handoff(ticket)

        bounded, _serial, _clock, _scheduler = self.make_coordinator(
            delivery_capacity=1
        )
        bounded._store_public_handoff(
            JsonEventDelivery(Event(0, "state", {}), None)
        )
        with self.assertRaises(JsonSerialSessionCoordinatorStateError):
            bounded._store_public_handoff(
                JsonEventDelivery(Event(1, "state", {}), True)
            )
        with self.assertRaises(JsonSerialSessionCoordinatorStateError):
            bounded._store_public_handoff(object())

    def test_response_handoff_terminal_acknowledgement_and_clean_close(self):
        coordinator, serial_port, _clock, scheduler = (
            self.make_coordinator()
        )
        ticket = coordinator.submit("hello", {}, timeout=2.0)
        serial_port.reads.append(
            encode_message(
                Response(
                    ticket.request_id,
                    "hello",
                    "completed",
                    result={"ready": True},
                )
            )
        )

        self.assertTrue(coordinator.poll())
        delivery = coordinator.pop_delivery()
        self.assertIsInstance(delivery, JsonResponseDelivery)
        self.assertIs(delivery.ticket, ticket)
        self.assertEqual(
            coordinator.take_terminal(ticket),
            delivery.response,
        )
        coordinator.acknowledge_terminal(ticket)
        self.assertEqual(coordinator.pending_tickets, ())
        self.assertFalse(scheduler.registrations[0]["active"])

        coordinator.close()
        self.assertTrue(coordinator.closed)
        self.assertFalse(coordinator.quarantined)
        self.assertFalse(serial_port.is_open)
        self.assertEqual(serial_port.close_calls, 1)
        self.assertFalse(coordinator.closing)
        self.assertIsNone(coordinator.pop_delivery())
        with self.assertRaises(JsonSerialSessionCoordinatorStateError):
            coordinator.submit("hello", {}, timeout=2.0)

    def test_terminal_acknowledgement_interruption_does_not_retain_ticket(self):
        coordinator, serial_port, _clock, _scheduler = (
            self.make_coordinator()
        )
        ticket = coordinator.submit("hello", {}, timeout=2.0)
        serial_port.reads.append(
            encode_message(
                Response(
                    ticket.request_id,
                    "hello",
                    "completed",
                    result={},
                )
            )
        )
        self.assertTrue(coordinator.poll())
        acknowledge = coordinator._session.acknowledge_terminal

        def interrupt_after_acknowledgement(value):
            acknowledge(value)
            raise KeyboardInterrupt()

        coordinator._session.acknowledge_terminal = (
            interrupt_after_acknowledgement
        )
        with self.assertRaises(KeyboardInterrupt):
            coordinator.acknowledge_terminal(ticket)

        self.assertEqual(coordinator.pending_tickets, ())
        self.assertTrue(
            coordinator._session.terminal_acknowledgement_complete(
                ticket
            )
        )

    def test_interrupted_session_delivery_return_is_recovered(self):
        coordinator, serial_port, _clock, _scheduler = (
            self.make_coordinator()
        )
        ticket = coordinator.submit("hello", {}, timeout=2.0)
        serial_port.reads.append(
            encode_message(
                Response(
                    ticket.request_id,
                    "hello",
                    "completed",
                    result={},
                )
            )
        )
        receive = coordinator._session.receive

        def lose_return(frame):
            receive(frame)
            raise KeyboardInterrupt()

        coordinator._session.receive = lose_return
        with self.assertRaises(KeyboardInterrupt):
            coordinator.poll()

        delivery = coordinator.pop_delivery()
        self.assertIsInstance(delivery, JsonResponseDelivery)
        self.assertIs(delivery.ticket, ticket)
        self.assertFalse(coordinator.quarantined)

    def test_delivery_capacity_stops_reads_until_the_queue_is_drained(self):
        serial_port = FakeSerial(
            (
                encode_message(Event(0, "state", {})),
                encode_message(Event(1, "state", {})),
            )
        )
        coordinator, _serial, _clock, _scheduler = self.make_coordinator(
            serial_port,
            delivery_capacity=1,
        )

        self.assertTrue(coordinator.poll())
        with self.assertRaises(JsonSerialSessionCoordinatorStateError):
            coordinator.poll()
        self.assertEqual(len(serial_port.reads), 1)
        self.assertIsNotNone(coordinator.pop_delivery())
        self.assertTrue(coordinator.poll())
        self.assertEqual(coordinator.delivery_count, 1)

    def test_transport_quarantine_is_mirrored_into_the_session(self):
        coordinator, serial_port, _clock, _scheduler = (
            self.make_coordinator()
        )
        serial_port.reads.append(b"\x01\n")

        with self.assertRaises(JsonSerialTransportQuarantinedError):
            coordinator.poll()

        self.assertTrue(coordinator._transport.quarantined)
        self.assertTrue(coordinator._session.quarantined)
        self.assertIn(
            "byte boundary",
            coordinator.quarantine_reason,
        )
        with self.assertRaises(JsonSerialSessionCoordinatorStateError):
            coordinator.poll()

    def test_session_quarantine_is_mirrored_into_the_transport(self):
        coordinator, serial_port, _clock, _scheduler = (
            self.make_coordinator()
        )
        serial_port.reads.append(
            encode_message(
                Response(
                    99,
                    "hello",
                    "completed",
                    result={},
                )
            )
        )

        with self.assertRaises(JsonSessionProtocolError):
            coordinator.poll()

        self.assertTrue(coordinator._session.quarantined)
        self.assertTrue(coordinator._transport.quarantined)
        self.assertIn(
            "no retained request",
            coordinator.quarantine_reason,
        )

    def test_transport_quarantine_close_race_does_not_leak_closed_error(self):
        coordinator, _serial_port, _clock, _scheduler = (
            self.make_coordinator()
        )
        coordinator._transport.quarantine("forced transport quarantine")

        def closed_race(_reason):
            raise JsonSessionClosedError("session closed during bridge")

        coordinator._session.quarantine = closed_race
        with self.assertRaises(JsonSerialSessionCoordinatorStateError):
            coordinator.poll()

    def test_concurrent_transport_quarantine_preserves_poll_failure(self):
        serial_port = FakeSerial()
        serial_port.write_started = threading.Event()
        serial_port.write_release = threading.Event()
        coordinator, _serial, _clock, _scheduler = self.make_coordinator(
            serial_port
        )
        submit_failures = []

        def submit():
            try:
                coordinator.submit("hello", {}, timeout=2.0)
            except BaseException as exc:
                submit_failures.append(exc)

        worker = threading.Thread(target=submit)
        worker.start()
        self.assertTrue(serial_port.write_started.wait(timeout=1.0))

        def fail_poll():
            self.assertEqual(len(coordinator._active_operations), 2)
            coordinator._transport.quarantine(
                "forced concurrent transport failure"
            )
            raise JsonSerialTransportQuarantinedError(
                "forced concurrent transport failure"
            )

        coordinator._transport.poll_frame = fail_poll
        try:
            with self.assertRaises(JsonSerialTransportQuarantinedError):
                coordinator.poll()
            self.assertIn(
                "session quarantine mirror remains pending",
                coordinator.quarantine_reason,
            )
        finally:
            serial_port.write_release.set()
            worker.join(timeout=1.0)

        self.assertFalse(worker.is_alive())
        self.assertFalse(
            any(
                isinstance(failure, JsonSessionAdmissionError)
                for failure in submit_failures
            )
        )
        self.assertTrue(coordinator._session.quarantined)
        self.assertTrue(coordinator._transport.quarantined)
        self.assertNotIn(
            "session quarantine mirror remains pending",
            coordinator.quarantine_reason,
        )

    def test_unverified_serial_close_remains_retryable(self):
        coordinator, serial_port, _clock, _scheduler = (
            self.make_coordinator()
        )
        serial_port.leave_open = True

        with self.assertRaises(JsonSerialSessionCoordinatorCloseError):
            coordinator.close(timeout=0.1)

        self.assertFalse(coordinator.closed)
        self.assertTrue(coordinator.closing)
        self.assertTrue(serial_port.is_open)
        with self.assertRaises(JsonSerialSessionCoordinatorStateError):
            coordinator.retry_deadline_cleanup()
        serial_port.leave_open = False
        coordinator.close(timeout=0.1)
        self.assertTrue(coordinator.closed)
        self.assertFalse(coordinator.closing)
        self.assertFalse(coordinator.quarantined)
        self.assertEqual(serial_port.close_calls, 2)

    def test_close_reports_serial_error_after_verified_closure(self):
        coordinator, serial_port, _clock, _scheduler = (
            self.make_coordinator()
        )
        serial_port.close_error = OSError("close cleanup failed")

        with self.assertRaises(JsonSerialSessionCoordinatorCloseError):
            coordinator.close(timeout=0.1)

        self.assertTrue(coordinator.closed)
        self.assertFalse(coordinator.closing)
        self.assertFalse(serial_port.is_open)
        self.assertIn("serial close", coordinator.quarantine_reason)
        with self.assertRaises(JsonSerialSessionCoordinatorCloseError):
            coordinator.close(timeout=0.1)

    def test_close_retries_after_serial_state_read_failure(self):
        coordinator, serial_port, _clock, _scheduler = (
            self.make_coordinator()
        )
        state_failure = OSError("serial state unavailable")
        serial_port.is_open_error = state_failure

        with self.assertRaises(
            JsonSerialSessionCoordinatorCloseError
        ) as raised:
            coordinator.close(timeout=0.1)

        self.assertIs(raised.exception.__cause__, state_failure)
        self.assertFalse(coordinator.closed)
        self.assertTrue(coordinator.closing)
        self.assertEqual(serial_port.close_calls, 1)

        serial_port.is_open_error = None
        coordinator.close(timeout=0.1)
        self.assertTrue(coordinator.closed)
        self.assertFalse(coordinator.closing)
        self.assertEqual(serial_port.close_calls, 2)

    def test_close_retries_session_after_verified_serial_closure(self):
        coordinator, serial_port, _clock, _scheduler = (
            self.make_coordinator()
        )
        close_session = coordinator._session.close
        close_calls = 0

        def fail_once():
            nonlocal close_calls
            close_calls += 1
            if close_calls == 1:
                raise JsonSessionAdmissionError(
                    "session shutdown remains active"
                )
            return close_session()

        coordinator._session.close = fail_once
        with self.assertRaises(JsonSerialSessionCoordinatorCloseError):
            coordinator.close(timeout=0.1)

        self.assertFalse(coordinator.closed)
        self.assertTrue(coordinator.closing)
        self.assertFalse(serial_port.is_open)
        self.assertEqual(serial_port.close_calls, 1)
        self.assertFalse(coordinator._session.closed)

        coordinator.close(timeout=0.1)
        self.assertTrue(coordinator.closed)
        self.assertFalse(coordinator.closing)
        self.assertTrue(coordinator._session.closed)
        self.assertEqual(serial_port.close_calls, 1)

    def test_close_abandons_unresolved_public_return_ownership(self):
        coordinator, serial_port, _clock, _scheduler = (
            self.make_coordinator()
        )
        ticket = coordinator.submit("hello", {}, timeout=2.0)
        serial_port.reads.append(
            encode_message(
                Response(
                    ticket.request_id,
                    "hello",
                    "completed",
                    result={},
                )
            )
        )

        def interrupt_before_acknowledgement(_result, _reason):
            raise KeyboardInterrupt()

        coordinator._acknowledge_public_handoff = (
            interrupt_before_acknowledgement
        )
        with self.assertRaises(KeyboardInterrupt):
            coordinator.poll()

        with self.assertRaises(JsonSerialSessionCoordinatorCloseError):
            coordinator.close(timeout=0.1)

        self.assertTrue(coordinator.closed)
        self.assertFalse(coordinator.closing)
        self.assertTrue(coordinator._session.quarantined)
        self.assertTrue(coordinator._transport.quarantined)
        self.assertFalse(serial_port.is_open)

    def test_close_reports_session_and_serial_failures_together(self):
        coordinator, serial_port, _clock, _scheduler = (
            self.make_coordinator()
        )
        coordinator.submit("hello", {}, timeout=2.0)
        serial_port.close_error = OSError("serial close failed")

        with self.assertRaises(
            JsonSerialSessionCoordinatorCloseError
        ) as raised:
            coordinator.close(timeout=0.1)

        self.assertTrue(coordinator.closed)
        self.assertFalse(coordinator.closing)
        self.assertIsInstance(raised.exception.__cause__, ExceptionGroup)
        causes = raised.exception.__cause__.exceptions
        self.assertTrue(
            any(
                isinstance(cause, JsonSessionQuarantinedError)
                for cause in causes
            )
        )
        self.assertTrue(any(isinstance(cause, OSError) for cause in causes))
        self.assertFalse(serial_port.is_open)
        self.assertIn(
            "session and serial close",
            coordinator.quarantine_reason,
        )
        with self.assertRaises(JsonSerialSessionCoordinatorCloseError):
            coordinator.close(timeout=0.1)

    def test_close_with_retained_request_quarantines_and_closes(self):
        coordinator, serial_port, _clock, _scheduler = (
            self.make_coordinator()
        )
        ticket = coordinator.submit("hello", {}, timeout=2.0)

        with self.assertRaises(JsonSerialSessionCoordinatorCloseError):
            coordinator.close(timeout=0.1)

        self.assertTrue(coordinator.closed)
        self.assertFalse(coordinator.closing)
        self.assertTrue(coordinator.quarantined)
        self.assertFalse(serial_port.is_open)
        self.assertEqual(coordinator.pending_tickets, (ticket,))

    def test_retained_delivery_remains_drainable_after_close(self):
        serial_port = FakeSerial(
            (encode_message(Event(0, "state", {})),)
        )
        coordinator, _serial, _clock, _scheduler = self.make_coordinator(
            serial_port
        )
        self.assertTrue(coordinator.poll())

        coordinator.close(timeout=0.1)

        self.assertTrue(coordinator.closed)
        self.assertIsInstance(
            coordinator.pop_delivery(),
            JsonEventDelivery,
        )
        self.assertIsNone(coordinator.pop_delivery())

    def test_retained_deadline_cleanup_is_retryable_after_close(self):
        coordinator, serial_port, _clock, scheduler = (
            self.make_coordinator()
        )
        scheduler.cancel_error = RuntimeError("cancellation failed")
        ticket = coordinator.submit("hello", {}, timeout=2.0)
        serial_port.reads.append(
            encode_message(
                Response(
                    ticket.request_id,
                    "hello",
                    "completed",
                    result={},
                )
            )
        )

        with self.assertRaises(JsonSessionDeadlineError):
            coordinator.poll()
        self.assertEqual(coordinator.deadline_cleanup_count, 1)
        with self.assertRaises(JsonSerialSessionCoordinatorCloseError):
            coordinator.close(timeout=0.1)
        self.assertTrue(coordinator.closed)
        self.assertFalse(coordinator.closing)

        scheduler.cancel_error = None
        self.assertEqual(coordinator.retry_deadline_cleanup(), 0)
        self.assertEqual(coordinator.deadline_cleanup_count, 0)

    def test_expire_quarantines_a_missed_request_deadline(self):
        coordinator, _serial_port, clock, _scheduler = (
            self.make_coordinator()
        )
        ticket = coordinator.submit("hello", {}, timeout=2.0)
        clock.value = ticket.deadline

        with self.assertRaises(JsonSessionTimeoutError):
            coordinator.expire()

        self.assertTrue(coordinator._session.quarantined)
        self.assertTrue(coordinator._transport.quarantined)

    def test_close_timeout_never_closes_during_active_serial_io(self):
        serial_port = FakeSerial()
        serial_port.read_started = threading.Event()
        serial_port.read_release = threading.Event()
        coordinator, _serial, _clock, _scheduler = self.make_coordinator(
            serial_port
        )
        failures = []

        def poll():
            try:
                coordinator.poll()
            except BaseException as exc:
                failures.append(exc)

        worker = threading.Thread(target=poll)
        worker.start()
        self.assertTrue(serial_port.read_started.wait(timeout=1.0))

        with self.assertRaises(JsonSerialSessionCoordinatorCloseError):
            coordinator.close(timeout=0.01)
        self.assertEqual(serial_port.close_calls, 0)
        self.assertTrue(serial_port.is_open)

        serial_port.read_release.set()
        worker.join(timeout=1.0)
        self.assertFalse(worker.is_alive())
        self.assertTrue(failures)

        with self.assertRaises(JsonSerialSessionCoordinatorCloseError):
            coordinator.close(timeout=0.1)
        self.assertTrue(coordinator.closed)
        self.assertFalse(serial_port.is_open)

    def test_operation_admission_interruption_never_strands_close(self):
        operations = (
            ("submit", lambda value: value.submit("hello", {}, timeout=2.0)),
            ("poll", lambda value: value.poll()),
            ("snapshot", lambda value: value.snapshot(object())),
            ("take_terminal", lambda value: value.take_terminal(object())),
            (
                "acknowledge_terminal",
                lambda value: value.acknowledge_terminal(object()),
            ),
            ("expire", lambda value: value.expire()),
        )
        for name, operation in operations:
            with self.subTest(operation=name):
                coordinator, serial_port, _clock, _scheduler = (
                    self.make_coordinator()
                )
                begin_name = (
                    "_begin_submit_operation"
                    if name == "submit"
                    else "_begin_operation"
                )
                begin = getattr(coordinator, begin_name)

                def interrupt_after_admission(*args):
                    begin(*args)
                    raise KeyboardInterrupt()

                setattr(
                    coordinator,
                    begin_name,
                    interrupt_after_admission,
                )
                with self.assertRaises(KeyboardInterrupt):
                    operation(coordinator)
                self.assertEqual(coordinator._active_operations, set())
                self.assertEqual(coordinator._submit_operations, set())

                setattr(coordinator, begin_name, begin)
                coordinator.close(timeout=0.1)
                self.assertFalse(serial_port.is_open)

    def test_cleanup_admission_interruption_never_strands_close(self):
        coordinator, serial_port, _clock, _scheduler = (
            self.make_coordinator()
        )
        begin = coordinator._begin_cleanup_operation

        def interrupt_after_admission(token):
            begin(token)
            raise KeyboardInterrupt()

        coordinator._begin_cleanup_operation = interrupt_after_admission
        with self.assertRaises(KeyboardInterrupt):
            coordinator.retry_deadline_cleanup()
        self.assertEqual(coordinator._active_operations, set())

        coordinator._begin_cleanup_operation = begin
        coordinator.close(timeout=0.1)
        self.assertFalse(serial_port.is_open)

    def test_close_timeout_and_ticket_validation_reject_invalid_input(self):
        for timeout in (0, -1, float("inf"), True, "slow"):
            with self.subTest(timeout=timeout):
                coordinator, serial_port, _clock, _scheduler = (
                    self.make_coordinator()
                )
                with self.assertRaises(
                    JsonSerialSessionCoordinatorAdmissionError
                ):
                    coordinator.close(timeout=timeout)
                self.assertEqual(serial_port.close_calls, 0)
                self.assertFalse(coordinator.closing)

        coordinator, _serial_port, _clock, _scheduler = (
            self.make_coordinator()
        )
        with self.assertRaises(
            JsonSerialSessionCoordinatorAdmissionError
        ):
            coordinator.snapshot(object())
        self.assertEqual(coordinator._active_operations, set())

    def test_foreign_ticket_rejects_without_mutating_either_session(self):
        first, _serial_a, _clock_a, _scheduler_a = self.make_coordinator()
        second, _serial_b, _clock_b, _scheduler_b = self.make_coordinator()
        ticket = first.submit("hello", {}, timeout=2.0)

        with self.assertRaises(
            JsonSerialSessionCoordinatorAdmissionError
        ):
            second.snapshot(ticket)

        self.assertFalse(first.quarantined)
        self.assertFalse(second.quarantined)


if __name__ == "__main__":
    unittest.main()
