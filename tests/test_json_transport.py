import threading
import time
import unittest

from ARrobots.protocol import (
    JSON_PROTOCOL_MAXIMUM_PAYLOAD_BYTES,
    JSON_SERIAL_DEFAULT_WRITE_TIMEOUT_SECONDS,
    MAIN_CONTROLLER,
    CorrelatedJsonSession,
    JsonCommandContract,
    JsonResponseDelivery,
    JsonSerialTransport,
    JsonSerialTransportAdmissionError,
    JsonSerialTransportDeadlineError,
    JsonSerialTransportFrameTimeoutError,
    JsonSerialTransportQuarantinedError,
    JsonSerialTransportReadDeferredError,
    Request,
    Response,
    decode_message,
    encode_message,
)


class FakeClock:
    def __init__(self, value=0.0):
        self.value = value

    def __call__(self):
        return self.value


class FakeSerial:
    def __init__(self, reads=()):
        self.is_open = True
        self._timeout = 3.0
        self._write_timeout = 4.0
        self._out_waiting = 0
        self.fail_read_timeout_restore = False
        self.fail_write_timeout_restore = False
        self._in_waiting_override = None
        self.in_waiting_error = None
        self.in_waiting_reads = 0
        self.out_waiting_error = None
        self.out_waiting_reads = 0
        self.reads = list(reads)
        self.read_sizes = []
        self.read_timeouts = []
        self.writes = []
        self.write_timeouts = []
        self.write_result = None
        self.write_error = None
        self.write_hook = None
        self.read_hook = None
        self.over_return = False

    @property
    def timeout(self):
        return self._timeout

    @timeout.setter
    def timeout(self, value):
        if self.fail_read_timeout_restore and value == 3.0:
            raise RuntimeError("read timeout restoration failed")
        self._timeout = value

    @property
    def write_timeout(self):
        return self._write_timeout

    @write_timeout.setter
    def write_timeout(self, value):
        if self.fail_write_timeout_restore and value == 4.0:
            raise RuntimeError("write timeout restoration failed")
        self._write_timeout = value

    @property
    def in_waiting(self):
        self.in_waiting_reads += 1
        if self.in_waiting_error is not None:
            raise self.in_waiting_error
        if self._in_waiting_override is not None:
            return self._in_waiting_override
        if not self.reads:
            return 0
        return len(self.reads[0])

    @property
    def out_waiting(self):
        self.out_waiting_reads += 1
        if self.out_waiting_error is not None:
            raise self.out_waiting_error
        return self._out_waiting

    def read(self, size):
        self.read_sizes.append(size)
        self.read_timeouts.append(self.timeout)
        if self.read_hook is not None:
            self.read_hook()
        if not self.reads:
            return b""
        chunk = self.reads.pop(0)
        if self.over_return or len(chunk) <= size:
            return chunk
        self.reads.insert(0, chunk[size:])
        return chunk[:size]

    def write(self, frame):
        self.writes.append(frame)
        self.write_timeouts.append(self.write_timeout)
        if self.write_hook is not None:
            self.write_hook()
        if self.write_error is not None:
            raise self.write_error
        if self.write_result is not None:
            return self.write_result
        return len(frame)


class JsonSerialTransportConstructionTests(unittest.TestCase):
    def test_constructor_requires_an_open_serial_contract(self):
        cases = (None, object())
        for serial_port in cases:
            with self.subTest(serial_port=serial_port):
                with self.assertRaises(JsonSerialTransportAdmissionError):
                    JsonSerialTransport(serial_port)

        closed = FakeSerial()
        closed.is_open = False
        with self.assertRaises(JsonSerialTransportAdmissionError):
            JsonSerialTransport(closed)

    def test_constructor_rejects_invalid_limits_and_clock_results(self):
        for keyword, value in (
            ("poll_interval", 0),
            ("frame_timeout", float("inf")),
            ("write_timeout", 0),
            ("drain_poll_interval", -1),
            ("read_chunk_bytes", True),
            ("read_chunk_bytes", 4097),
        ):
            with self.subTest(keyword=keyword, value=value):
                with self.assertRaises(JsonSerialTransportAdmissionError):
                    JsonSerialTransport(FakeSerial(), **{keyword: value})

        with self.assertRaises(JsonSerialTransportAdmissionError):
            JsonSerialTransport(FakeSerial(), clock=lambda: "now")
        with self.assertRaises(JsonSerialTransportAdmissionError):
            JsonSerialTransport(FakeSerial(), sleeper=None)

        invalid_queue = FakeSerial()
        invalid_queue._out_waiting = True
        with self.assertRaises(JsonSerialTransportAdmissionError):
            JsonSerialTransport(invalid_queue)

        invalid_input_queue = FakeSerial()
        invalid_input_queue._in_waiting_override = True
        with self.assertRaises(JsonSerialTransportAdmissionError):
            JsonSerialTransport(invalid_input_queue)

    def test_manual_quarantine_preserves_the_first_reason(self):
        transport = JsonSerialTransport(FakeSerial())
        transport.quarantine("session decoder rejected a frame")
        transport.quarantine("later failure")

        self.assertTrue(transport.quarantined)
        self.assertEqual(
            transport.quarantine_reason,
            "session decoder rejected a frame",
        )
        with self.assertRaises(JsonSerialTransportQuarantinedError):
            transport.poll_frame()
        with self.assertRaises(JsonSerialTransportQuarantinedError):
            transport.write_frame(b"invalid", 1.0)

        for reason in ("", " padded ", "bad\nreason", "x" * 513):
            with self.subTest(reason=reason):
                fresh = JsonSerialTransport(FakeSerial())
                with self.assertRaises(JsonSerialTransportAdmissionError):
                    fresh.quarantine(reason)
                self.assertFalse(fresh.quarantined)


class JsonSerialTransportWriteTests(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock(10.0)
        self.serial = FakeSerial()
        self.transport = JsonSerialTransport(
            self.serial,
            clock=self.clock,
        )
        self.frame = encode_message(Request(1, "hello", {}))

    def test_complete_write_uses_remaining_deadline_and_restores_timeout(self):
        written = self.transport.write_frame(self.frame, 12.5)

        self.assertEqual(written, len(self.frame))
        self.assertEqual(self.serial.writes, [self.frame])
        self.assertEqual(self.serial.write_timeouts, [2.5])
        self.assertEqual(self.serial.write_timeout, 4.0)
        self.assertEqual(self.serial.out_waiting_reads, 2)
        self.assertFalse(self.transport.quarantined)

    def test_write_wait_uses_the_independent_transport_cap(self):
        self.assertEqual(
            self.transport.write_frame(self.frame, 100.0),
            len(self.frame),
        )
        self.assertEqual(
            self.serial.write_timeouts,
            [JSON_SERIAL_DEFAULT_WRITE_TIMEOUT_SECONDS],
        )
        self.assertFalse(self.transport.quarantined)

    def test_invalid_or_expired_writes_do_not_touch_the_serial_port(self):
        for frame in (
            bytearray(self.frame),
            b"",
            b"{}",
            b"{}\n{}\n",
            b"{\x00}\n",
            encode_message(Response(1, "hello", "completed", {})),
        ):
            with self.subTest(frame=frame):
                with self.assertRaises(JsonSerialTransportAdmissionError):
                    self.transport.write_frame(frame, 12.0)
        with self.assertRaises(JsonSerialTransportDeadlineError):
            self.transport.write_frame(self.frame, 10.0)
        with self.assertRaises(JsonSerialTransportAdmissionError):
            self.transport.write_frame(self.frame, float("inf"))
        self.assertEqual(self.serial.writes, [])
        self.assertFalse(self.transport.quarantined)

        unrepresentable = JsonSerialTransport(
            FakeSerial(),
            clock=FakeClock(1e308),
        )
        with self.assertRaises(JsonSerialTransportAdmissionError):
            unrepresentable.write_frame(self.frame, 1.1e308)
        self.assertFalse(unrepresentable.quarantined)

    def test_partial_or_failed_write_quarantines_without_retry(self):
        self.serial.write_result = len(self.frame) - 1
        with self.assertRaises(JsonSerialTransportQuarantinedError):
            self.transport.write_frame(self.frame, 12.0)
        self.assertEqual(self.serial.writes, [self.frame])
        self.assertEqual(self.serial.out_waiting_reads, 1)

        with self.assertRaises(JsonSerialTransportQuarantinedError):
            self.transport.write_frame(self.frame, 12.0)
        self.assertEqual(self.serial.writes, [self.frame])

        failed_serial = FakeSerial()
        failed_serial.write_error = OSError("disconnected")
        failed = JsonSerialTransport(failed_serial, clock=self.clock)
        with self.assertRaises(
            JsonSerialTransportQuarantinedError
        ) as context:
            failed.write_frame(self.frame, 12.0)
        self.assertIsInstance(context.exception.__cause__, OSError)
        self.assertTrue(failed.quarantined)

        queue_failed_serial = FakeSerial()
        queue_failed = JsonSerialTransport(
            queue_failed_serial,
            clock=self.clock,
        )
        queue_failed_serial.out_waiting_error = OSError("queue failed")
        with self.assertRaises(
            JsonSerialTransportQuarantinedError
        ) as queue_context:
            queue_failed.write_frame(self.frame, 12.0)
        self.assertIsInstance(queue_context.exception.__cause__, OSError)
        self.assertTrue(queue_failed.quarantined)

        interrupted_serial = FakeSerial()
        interrupted_serial.write_error = KeyboardInterrupt()
        interrupted = JsonSerialTransport(
            interrupted_serial,
            clock=self.clock,
        )
        with self.assertRaises(KeyboardInterrupt):
            interrupted.write_frame(self.frame, 12.0)
        self.assertEqual(interrupted_serial.write_timeout, 4.0)
        self.assertTrue(interrupted.quarantined)

    def test_write_completion_at_or_after_the_deadline_is_ambiguous(self):
        self.serial.write_hook = lambda: setattr(self.clock, "value", 12.0)
        with self.assertRaises(JsonSerialTransportQuarantinedError):
            self.transport.write_frame(self.frame, 12.0)
        self.assertTrue(self.transport.quarantined)

    def test_write_ownership_expiry_before_serial_io_stays_clean(self):
        transport = JsonSerialTransport(FakeSerial())
        frame = encode_message(Request(1, "hello", {}))
        transport._write_lock.acquire()
        try:
            with self.assertRaises(JsonSerialTransportDeadlineError):
                transport.write_frame(frame, time.monotonic() + 0.01)
        finally:
            transport._write_lock.release()
        self.assertFalse(transport.quarantined)

    def test_post_ownership_deadline_expiry_stays_clean(self):
        class DeadlineAdvancingLock:
            def acquire(inner_self, **_kwargs):
                self.clock.value = 12.0
                return True

            def release(inner_self):
                return None

        self.transport._write_lock = DeadlineAdvancingLock()

        with self.assertRaises(JsonSerialTransportDeadlineError):
            self.transport.write_frame(self.frame, 12.0)
        self.assertEqual(self.serial.writes, [])
        self.assertFalse(self.transport.quarantined)

    def test_post_io_ownership_deadline_expiry_stays_clean(self):
        class DeadlineAdvancingLock:
            def acquire(inner_self, **_kwargs):
                self.clock.value = 12.0
                return True

            def release(inner_self):
                return None

        self.transport._serial_io_lock = DeadlineAdvancingLock()

        with self.assertRaises(JsonSerialTransportDeadlineError):
            self.transport.write_frame(self.frame, 12.0)
        self.assertEqual(self.serial.writes, [])
        self.assertFalse(self.transport.quarantined)

    def test_write_timeout_restoration_failure_quarantines(self):
        self.serial.fail_write_timeout_restore = True
        with self.assertRaises(
            JsonSerialTransportQuarantinedError
        ) as context:
            self.transport.write_frame(self.frame, 12.0)

        self.assertIsInstance(context.exception.__cause__, RuntimeError)
        self.assertEqual(len(self.serial.writes), 1)
        self.assertTrue(self.transport.quarantined)

    def test_output_drain_is_polled_under_the_request_deadline(self):
        serial_port = FakeSerial()
        serial_port._out_waiting = 4
        delays = []

        def drain_wait(delay):
            delays.append(delay)
            self.clock.value += 0.25
            serial_port._out_waiting = 0

        transport = JsonSerialTransport(
            serial_port,
            clock=self.clock,
            sleeper=drain_wait,
            drain_poll_interval=0.25,
        )
        self.assertEqual(
            transport.write_frame(self.frame, 12.0),
            len(self.frame),
        )
        self.assertEqual(delays, [0.25])
        self.assertEqual(serial_port.out_waiting_reads, 3)
        self.assertFalse(transport.quarantined)

    def test_stalled_output_drain_quarantines_at_the_deadline(self):
        serial_port = FakeSerial()
        serial_port._out_waiting = 4

        def expire_deadline(_delay):
            self.clock.value = 12.0

        transport = JsonSerialTransport(
            serial_port,
            clock=self.clock,
            sleeper=expire_deadline,
        )
        with self.assertRaises(JsonSerialTransportQuarantinedError):
            transport.write_frame(self.frame, 12.0)
        self.assertTrue(transport.quarantined)

    def test_output_drain_tolerates_repeated_coarse_clock_samples(self):
        serial_port = FakeSerial()
        serial_port._out_waiting = 4
        waits = []

        def coarse_wait(delay):
            waits.append(delay)
            if len(waits) == 4:
                self.clock.value += 0.015625
                serial_port._out_waiting = 0

        transport = JsonSerialTransport(
            serial_port,
            clock=self.clock,
            sleeper=coarse_wait,
        )

        self.assertEqual(
            transport.write_frame(self.frame, 12.0),
            len(self.frame),
        )
        self.assertEqual(len(waits), 4)
        self.assertFalse(transport.quarantined)

    def test_output_drain_rejects_a_nonadvancing_clock(self):
        serial_port = FakeSerial()
        serial_port._out_waiting = 4
        transport = JsonSerialTransport(
            serial_port,
            clock=self.clock,
            sleeper=lambda _delay: None,
            drain_poll_interval=0.25,
        )

        with self.assertRaises(JsonSerialTransportQuarantinedError):
            transport.write_frame(self.frame, 10.5)
        self.assertTrue(transport.quarantined)

    def test_reader_io_ownership_blocks_writer_without_quarantine(self):
        serial_port = FakeSerial()
        read_entered = threading.Event()
        release_read = threading.Event()
        reader_errors = []

        def block_read():
            read_entered.set()
            if not release_read.wait(5):
                raise RuntimeError("reader test release timed out")

        def poll_once():
            try:
                transport.poll_frame()
            except BaseException as exc:
                reader_errors.append(exc)

        serial_port.read_hook = block_read
        transport = JsonSerialTransport(serial_port)
        reader = threading.Thread(target=poll_once)
        reader.start()
        try:
            self.assertTrue(read_entered.wait(5))
            with self.assertRaises(JsonSerialTransportDeadlineError):
                transport.write_frame(
                    self.frame,
                    time.monotonic() + 0.01,
                )
            self.assertEqual(serial_port.writes, [])
            self.assertFalse(transport.quarantined)
        finally:
            release_read.set()
            reader.join(5)
        self.assertFalse(reader.is_alive())
        self.assertEqual(reader_errors, [])

    def test_writer_io_ownership_defers_reader_without_quarantine(self):
        serial_port = FakeSerial()
        write_entered = threading.Event()
        release_write = threading.Event()
        writer_errors = []

        def block_write():
            write_entered.set()
            if not release_write.wait(5):
                raise RuntimeError("writer test release timed out")

        def write_once():
            try:
                transport.write_frame(
                    self.frame,
                    time.monotonic() + 5.0,
                )
            except BaseException as exc:
                writer_errors.append(exc)

        serial_port.write_hook = block_write
        transport = JsonSerialTransport(serial_port, poll_interval=0.01)
        writer = threading.Thread(target=write_once)
        writer.start()
        try:
            self.assertTrue(write_entered.wait(5))
            with self.assertRaises(
                JsonSerialTransportReadDeferredError
            ) as context:
                transport.poll_frame()
            self.assertEqual(context.exception.deferred_for, 0.01)
            self.assertEqual(serial_port.read_sizes, [])
            self.assertFalse(transport.quarantined)
        finally:
            release_write.set()
            writer.join(5)
        self.assertFalse(writer.is_alive())
        self.assertEqual(writer_errors, [])


class JsonSerialTransportReadTests(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()

    def transport(self, reads=(), **kwargs):
        serial_port = FakeSerial(reads)
        transport = JsonSerialTransport(
            serial_port,
            clock=self.clock,
            **kwargs,
        )
        return transport, serial_port

    def poll_and_acknowledge(self, transport):
        frame = transport.poll_frame()
        if frame is not None:
            self.assertTrue(transport.acknowledge_frame(frame))
        return frame

    def test_split_and_back_to_back_frames_are_preserved(self):
        first = b'{"type":"event","v":1}\n'
        second = b'{"type":"telemetry","v":1}\r\n'
        transport, serial_port = self.transport(
            (first[:8], first[8:] + second)
        )

        self.assertIsNone(transport.poll_frame())
        self.assertEqual(self.poll_and_acknowledge(transport), first)
        read_count = len(serial_port.read_sizes)
        self.assertEqual(self.poll_and_acknowledge(transport), second)
        self.assertEqual(len(serial_port.read_sizes), read_count)
        self.assertEqual(serial_port.timeout, 3.0)

    def test_idle_poll_is_not_a_transport_failure(self):
        transport, serial_port = self.transport((b"",))

        self.assertIsNone(transport.poll_frame())
        self.assertFalse(transport.quarantined)
        self.assertEqual(serial_port.read_timeouts, [0.05])
        self.assertEqual(serial_port.read_sizes, [1])

    def test_queued_frame_sizes_the_read_to_available_input(self):
        frame = b'{"type":"event","v":1}\n'
        transport, serial_port = self.transport((frame,))

        self.assertEqual(self.poll_and_acknowledge(transport), frame)
        self.assertEqual(serial_port.read_sizes, [len(frame)])

    def test_frame_handoff_requires_exact_acknowledgement(self):
        frame = b'{"type":"event","v":1}\n'
        transport, _serial_port = self.transport((frame,))

        returned = transport.poll_frame()

        self.assertEqual(returned, frame)
        self.assertTrue(transport.frame_handoff_pending)
        self.assertEqual(transport.buffered_input_bytes, len(frame))
        with self.assertRaisesRegex(
            JsonSerialTransportAdmissionError,
            "acknowledged frame handoff",
        ):
            transport.release_reader()
        equal_copy = memoryview(returned).tobytes()
        self.assertIsNot(equal_copy, returned)
        with self.assertRaisesRegex(
            JsonSerialTransportAdmissionError,
            "does not own handoff",
        ):
            transport.acknowledge_frame(equal_copy)
        self.assertTrue(transport.acknowledge_frame(returned))
        self.assertFalse(transport.frame_handoff_pending)
        self.assertTrue(transport.frame_acknowledgement_complete(returned))
        self.assertEqual(transport.buffered_input_bytes, 0)
        with self.assertRaisesRegex(
            JsonSerialTransportAdmissionError,
            "handoff is not pending",
        ):
            transport.acknowledge_frame(returned)
        self.assertTrue(transport.release_reader())

    def test_unacknowledged_frame_blocks_later_polling(self):
        first = b'{"v":1}\n'
        second = b'{"v":2}\n'
        transport, serial_port = self.transport((first, second))

        self.assertEqual(transport.poll_frame(), first)
        read_count = len(serial_port.read_sizes)
        with self.assertRaises(JsonSerialTransportQuarantinedError):
            transport.poll_frame()

        self.assertEqual(len(serial_port.read_sizes), read_count)
        self.assertTrue(transport.quarantined)
        self.assertFalse(transport.frame_handoff_pending)

    def test_frame_handoff_precedes_buffer_consumption(self):
        frame = b'{"v":1}\n'
        transport, _serial_port = self.transport((frame,))
        handoff_states = []

        class FailingBuffer(bytearray):
            def __delitem__(self, key):
                handoff_states.append(transport.frame_handoff_pending)
                raise RuntimeError("buffer deletion failed")

        transport._buffer = FailingBuffer()
        with self.assertRaisesRegex(RuntimeError, "buffer deletion failed"):
            transport.poll_frame()

        self.assertEqual(handoff_states, [True])
        self.assertTrue(transport.quarantined)
        self.assertFalse(transport.frame_handoff_pending)

    def test_read_timeout_restoration_failure_quarantines(self):
        transport, serial_port = self.transport((b"",))
        serial_port.fail_read_timeout_restore = True

        with self.assertRaises(
            JsonSerialTransportQuarantinedError
        ) as context:
            transport.poll_frame()
        self.assertIsInstance(context.exception.__cause__, RuntimeError)
        self.assertTrue(transport.quarantined)

    def test_maximum_crlf_frame_is_accepted(self):
        payload = b"x" * JSON_PROTOCOL_MAXIMUM_PAYLOAD_BYTES
        frame = payload + b"\r\n"
        transport, _ = self.transport(
            (frame,),
            read_chunk_bytes=len(frame),
        )

        self.assertEqual(self.poll_and_acknowledge(transport), frame)

    def test_invalid_byte_boundaries_quarantine(self):
        cases = (
            b"\n",
            b"{}\x00\n",
            b"x" * (JSON_PROTOCOL_MAXIMUM_PAYLOAD_BYTES + 1) + b"\n",
        )
        for frame in cases:
            with self.subTest(frame_length=len(frame)):
                transport, _ = self.transport(
                    (frame,),
                    read_chunk_bytes=len(frame),
                )
                with self.assertRaises(JsonSerialTransportQuarantinedError):
                    transport.poll_frame()
                self.assertTrue(transport.quarantined)

    def test_unterminated_maximum_frame_quarantines(self):
        frame = b"x" * 4096
        transport, _ = self.transport(
            (frame,),
            read_chunk_bytes=len(frame),
        )

        with self.assertRaises(JsonSerialTransportQuarantinedError):
            transport.poll_frame()

    def test_non_bytes_or_overlong_read_result_quarantines(self):
        transport, _ = self.transport(("not bytes",))
        with self.assertRaises(JsonSerialTransportQuarantinedError):
            transport.poll_frame()

        transport, serial_port = self.transport(
            (b"{}\nextra",),
            read_chunk_bytes=3,
        )
        serial_port.over_return = True
        with self.assertRaises(JsonSerialTransportQuarantinedError):
            transport.poll_frame()

    def test_invalid_input_queue_state_quarantines(self):
        transport, serial_port = self.transport()
        serial_port._in_waiting_override = True
        with self.assertRaises(JsonSerialTransportQuarantinedError):
            transport.poll_frame()

        failed, failed_serial = self.transport()
        failed_serial.in_waiting_error = OSError("queue failed")
        with self.assertRaises(
            JsonSerialTransportQuarantinedError
        ) as context:
            failed.poll_frame()
        self.assertIsInstance(context.exception.__cause__, OSError)

    def test_partial_frame_timeout_is_fail_closed(self):
        transport, _ = self.transport((b'{"v":',), frame_timeout=1.0)
        self.assertIsNone(transport.poll_frame())
        self.clock.value = 1.0

        with self.assertRaises(JsonSerialTransportFrameTimeoutError):
            transport.poll_frame()
        self.assertTrue(transport.quarantined)

    def test_io_contention_defers_a_partial_frame_deadline(self):
        transport, serial_port = self.transport(
            (b'{"v":',),
            frame_timeout=1.0,
            poll_interval=0.01,
        )
        self.assertIsNone(transport.poll_frame())
        transport._serial_io_lock.acquire()
        try:
            with self.assertRaises(
                JsonSerialTransportReadDeferredError
            ) as context:
                transport.poll_frame()
        finally:
            transport._serial_io_lock.release()

        self.assertEqual(context.exception.deferred_for, 0.01)
        self.assertFalse(transport.quarantined)
        self.clock.value = 1.0
        serial_port.reads.append(b"1}\n")
        self.assertEqual(
            self.poll_and_acknowledge(transport),
            b'{"v":1}\n',
        )

    def test_reader_thread_identity_is_stable(self):
        transport, _ = self.transport((b"", b""))
        worker_errors = []

        def establish_reader():
            try:
                transport.poll_frame()
            except Exception as exc:
                worker_errors.append(exc)

        worker = threading.Thread(target=establish_reader)
        worker.start()
        worker.join(5)
        self.assertFalse(worker.is_alive())
        self.assertEqual(worker_errors, [])

        with self.assertRaises(JsonSerialTransportQuarantinedError):
            transport.poll_frame()

    def test_reader_identity_transfers_only_at_complete_frame_boundary(self):
        transport, serial_port = self.transport((b"",))

        self.assertIsNone(transport.poll_frame())
        self.assertIs(transport.reader_owner, threading.current_thread())
        self.assertTrue(transport.validate_reader_release())
        self.assertTrue(transport.release_reader())
        self.assertIsNone(transport.reader_owner)

        serial_port.reads.append(b"")
        results = []
        errors = []

        def poll_from_next_owner():
            try:
                results.append(transport.poll_frame())
                results.append(transport.release_reader())
            except BaseException as exc:
                errors.append(exc)

        worker = threading.Thread(target=poll_from_next_owner)
        worker.start()
        worker.join(timeout=2)

        self.assertFalse(worker.is_alive())
        self.assertEqual(errors, [])
        self.assertEqual(results, [None, True])
        self.assertIsNone(transport.reader_owner)
        self.assertFalse(transport.quarantined)

        serial_port.reads.append(b'{"v":')
        self.assertIsNone(transport.poll_frame())
        with self.assertRaisesRegex(
            JsonSerialTransportAdmissionError,
            "complete frame boundary",
        ):
            transport.release_reader()
        serial_port.reads.append(b"1}\n")
        self.assertEqual(
            self.poll_and_acknowledge(transport),
            b'{"v":1}\n',
        )
        self.assertTrue(transport.release_reader())
        self.assertFalse(transport.quarantined)

    def test_lifecycle_contention_uses_reservation_diagnostic(self):
        transport, _serial_port = self.transport((b"",))
        self.assertIsNone(transport.poll_frame())
        ensure_open = transport._ensure_open
        nested_release_checked = False

        def ensure_open_with_nested_release():
            nonlocal nested_release_checked
            ensure_open()
            if not nested_release_checked:
                nested_release_checked = True
                with self.assertRaisesRegex(
                    JsonSerialTransportAdmissionError,
                    "lifecycle operation overlaps an active reservation",
                ):
                    transport.release_reader()

        transport._ensure_open = ensure_open_with_nested_release
        try:
            self.assertTrue(transport.validate_reader_release())
        finally:
            transport._ensure_open = ensure_open

        self.assertTrue(nested_release_checked)
        self.assertFalse(transport.quarantined)
        self.assertTrue(transport.release_reader())

    def test_reader_release_rejects_foreign_thread_without_quarantine(self):
        transport, _serial_port = self.transport((b"",))
        self.assertIsNone(transport.poll_frame())
        errors = []

        def release_from_foreign_thread():
            try:
                transport.release_reader()
            except BaseException as exc:
                errors.append(exc)

        worker = threading.Thread(target=release_from_foreign_thread)
        worker.start()
        worker.join(timeout=2)

        self.assertFalse(worker.is_alive())
        self.assertEqual(len(errors), 1)
        self.assertIsInstance(
            errors[0],
            JsonSerialTransportAdmissionError,
        )
        self.assertIs(transport.reader_owner, threading.current_thread())
        self.assertFalse(transport.quarantined)
        self.assertTrue(transport.release_reader())

    def test_reader_release_postcleanup_interruption_preserves_cleanup(self):
        transport, _serial_port = self.transport((b"",))
        self.assertIsNone(transport.poll_frame())
        force_end = transport._force_end_read_reservation
        cleanup_calls = 0

        def interrupt_first_cleanup(token):
            nonlocal cleanup_calls
            force_end(token)
            cleanup_calls += 1
            if cleanup_calls == 1:
                raise KeyboardInterrupt()

        transport._force_end_read_reservation = interrupt_first_cleanup
        with self.assertRaises(KeyboardInterrupt):
            transport.release_reader()

        self.assertGreaterEqual(cleanup_calls, 2)
        self.assertIsNone(transport._read_reservation)
        self.assertIsNone(transport.reader_owner)
        self.assertFalse(transport.quarantined)

    def test_poll_postcleanup_interruption_retains_consumed_frame(self):
        transport, _serial_port = self.transport((b'{"v":1}\n',))
        force_end = transport._force_end_read_reservation
        cleanup_calls = 0

        def interrupt_first_cleanup(token):
            nonlocal cleanup_calls
            force_end(token)
            cleanup_calls += 1
            if cleanup_calls == 1:
                raise KeyboardInterrupt()

        transport._force_end_read_reservation = interrupt_first_cleanup
        with self.assertRaises(KeyboardInterrupt):
            transport.poll_frame()

        self.assertGreaterEqual(cleanup_calls, 2)
        self.assertIsNone(transport._read_reservation)
        self.assertTrue(transport.frame_handoff_pending)
        self.assertFalse(transport.quarantined)
        with self.assertRaises(JsonSerialTransportQuarantinedError):
            transport.poll_frame()
        self.assertTrue(transport.quarantined)

    def test_successor_reader_survives_predecessor_release_finalization(self):
        transport, _serial_port = self.transport((b"", b""))
        release_started = threading.Event()
        finish_release = threading.Event()
        results = []
        errors = []

        def predecessor():
            try:
                results.append(transport.poll_frame())
                force_end = transport._force_end_read_reservation

                def pause_after_cleanup(token):
                    force_end(token)
                    if not release_started.is_set():
                        release_started.set()
                        if not finish_release.wait(timeout=2):
                            raise TimeoutError(
                                "reader release was not resumed"
                            )

                transport._force_end_read_reservation = pause_after_cleanup
                results.append(transport.release_reader())
            except BaseException as exc:
                errors.append(exc)

        worker = threading.Thread(target=predecessor)
        worker.start()
        self.assertTrue(release_started.wait(timeout=1))
        try:
            self.assertIsNone(transport.poll_frame())
            self.assertIs(
                transport.reader_owner,
                threading.current_thread(),
            )
        finally:
            finish_release.set()
        worker.join(timeout=2)

        self.assertFalse(worker.is_alive())
        self.assertEqual(errors, [])
        self.assertEqual(results, [None, True])
        self.assertIsNone(transport._read_reservation)
        self.assertIs(transport.reader_owner, threading.current_thread())
        self.assertFalse(transport.quarantined)
        self.assertTrue(transport.release_reader())

    def test_interrupted_predecessor_release_preserves_admitted_successor(self):
        transport, _serial_port = self.transport((b"", b""))
        release_cleanup_started = threading.Event()
        resume_release = threading.Event()
        successor_admitted = threading.Event()
        resume_successor = threading.Event()
        successor_polled = threading.Event()
        release_successor = threading.Event()
        predecessor_results = []
        predecessor_errors = []
        successor_results = []
        successor_errors = []
        force_end = transport._force_end_read_reservation
        poll_reserved = transport._poll_frame_reserved

        def predecessor():
            try:
                predecessor_results.append(transport.poll_frame())
                first_cleanup = True

                def interrupt_after_cleanup(token):
                    nonlocal first_cleanup
                    force_end(token)
                    if first_cleanup:
                        first_cleanup = False
                        release_cleanup_started.set()
                        if not resume_release.wait(timeout=2):
                            raise TimeoutError(
                                "reader release was not resumed"
                            )
                        raise KeyboardInterrupt()

                transport._force_end_read_reservation = (
                    interrupt_after_cleanup
                )
                transport.release_reader()
            except BaseException as exc:
                predecessor_errors.append(exc)

        def hold_successor_poll():
            successor_admitted.set()
            if not resume_successor.wait(timeout=2):
                raise TimeoutError("successor poll was not resumed")
            return poll_reserved()

        def successor():
            try:
                successor_results.append(transport.poll_frame())
                successor_polled.set()
                if not release_successor.wait(timeout=2):
                    raise TimeoutError("successor release was not resumed")
                successor_results.append(transport.release_reader())
            except BaseException as exc:
                successor_errors.append(exc)

        predecessor_thread = threading.Thread(target=predecessor)
        predecessor_thread.start()
        self.assertTrue(release_cleanup_started.wait(timeout=1))
        transport._poll_frame_reserved = hold_successor_poll
        successor_thread = threading.Thread(target=successor)
        successor_thread.start()
        self.assertTrue(successor_admitted.wait(timeout=1))

        resume_release.set()
        predecessor_thread.join(timeout=2)
        self.assertFalse(predecessor_thread.is_alive())
        self.assertEqual(predecessor_results, [None])
        self.assertEqual(len(predecessor_errors), 1)
        self.assertIsInstance(predecessor_errors[0], KeyboardInterrupt)
        self.assertIs(transport.reader_owner, successor_thread)
        self.assertFalse(transport.quarantined)

        resume_successor.set()
        self.assertTrue(successor_polled.wait(timeout=1))
        self.assertEqual(successor_results, [None])
        self.assertIs(transport.reader_owner, successor_thread)
        release_successor.set()
        successor_thread.join(timeout=2)

        transport._force_end_read_reservation = force_end
        transport._poll_frame_reserved = poll_reserved
        self.assertFalse(successor_thread.is_alive())
        self.assertEqual(successor_errors, [])
        self.assertEqual(successor_results, [None, True])
        self.assertIsNone(transport._read_reservation)
        self.assertIsNone(transport.reader_owner)
        self.assertFalse(transport.quarantined)

    def test_concurrent_read_ownership_quarantines(self):
        transport, _serial_port = self.transport((b"",))
        read_started = threading.Event()
        finish_read = threading.Event()
        errors = []
        read_chunk = transport._read_chunk

        def hold_read(*args):
            read_started.set()
            if not finish_read.wait(timeout=2):
                raise TimeoutError("serial read was not resumed")
            return read_chunk(*args)

        def poll_from_first_reader():
            try:
                transport.poll_frame()
            except BaseException as exc:
                errors.append(exc)

        transport._read_chunk = hold_read
        worker = threading.Thread(target=poll_from_first_reader)
        worker.start()
        self.assertTrue(read_started.wait(timeout=1))
        try:
            with self.assertRaises(JsonSerialTransportQuarantinedError):
                transport.poll_frame()
        finally:
            finish_read.set()
        worker.join(timeout=2)

        self.assertFalse(worker.is_alive())
        self.assertEqual(len(errors), 1)
        self.assertIsInstance(errors[0], JsonSerialTransportQuarantinedError)
        self.assertIsNone(transport._read_reservation)
        self.assertTrue(transport.quarantined)

    def test_closed_connection_and_clock_regression_quarantine(self):
        transport, serial_port = self.transport((b"",))
        serial_port.is_open = False
        with self.assertRaises(JsonSerialTransportQuarantinedError):
            transport.poll_frame()

        self.clock.value = 2.0
        regressed, _ = self.transport((b"",))
        self.clock.value = 1.0
        with self.assertRaises(JsonSerialTransportQuarantinedError):
            regressed.poll_frame()

    def test_interruption_after_serial_read_quarantines_consumed_bytes(self):
        calls = 0

        def interrupt_after_read():
            nonlocal calls
            calls += 1
            if calls == 3:
                raise KeyboardInterrupt()
            return 0.0

        serial_port = FakeSerial((b'{"v":',))
        transport = JsonSerialTransport(
            serial_port,
            clock=interrupt_after_read,
        )
        with self.assertRaises(KeyboardInterrupt):
            transport.poll_frame()
        self.assertEqual(serial_port.reads, [])
        self.assertTrue(transport.quarantined)


class JsonSerialTransportSessionIntegrationTests(unittest.TestCase):
    def test_transport_frames_round_trip_through_the_session_contract(self):
        clock = FakeClock(10.0)
        serial_port = FakeSerial()
        transport = JsonSerialTransport(serial_port, clock=clock)
        registrations = []

        def schedule_deadline(deadline, _callback, publish_owner):
            registration = {"active": True, "deadline": deadline}

            def cancel():
                registration["active"] = False

            registrations.append(registration)
            publish_owner(registration, cancel)

        contract = JsonCommandContract(
            "hello",
            lambda _params: None,
            lambda _response: None,
        )
        session = CorrelatedJsonSession(
            MAIN_CONTROLLER,
            (contract,),
            transport.write_frame,
            clock=clock,
            deadline_scheduler=schedule_deadline,
        )

        ticket = session.submit("hello", {}, timeout=2.0)
        session.acknowledge_public_return(ticket)
        self.assertEqual(
            decode_message(serial_port.writes[0]),
            Request(ticket.request_id, "hello", {}),
        )

        response = encode_message(
            Response(
                ticket.request_id,
                "hello",
                "completed",
                result={"ready": True},
            )
        )
        serial_port.reads.append(response[:-1] + b"\r\n")
        frame = transport.poll_frame()
        delivery = session.receive(frame)
        self.assertIsInstance(delivery, JsonResponseDelivery)
        session.acknowledge_public_return(delivery)
        self.assertTrue(transport.acknowledge_frame(frame))
        self.assertEqual(session.take_terminal(ticket), delivery.response)
        session.acknowledge_terminal(ticket)
        self.assertEqual(session.pending_count, 0)
        self.assertFalse(registrations[0]["active"])


if __name__ == "__main__":
    unittest.main()
