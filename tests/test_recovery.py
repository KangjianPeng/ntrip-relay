import os
import select
import socket
import threading
import time
import unittest
from datetime import datetime, timedelta
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import pynmea2
import serial

import ntrip_relay as relay
from tests.test_gga import NO_FIX, POSITION, FakeSerial, FakeSocket


def position_at(timestamp):
    message = pynmea2.parse(POSITION.decode().strip(), check=True)
    data = list(message.data)
    data[0] = timestamp
    return (str(pynmea2.GGA("GN", "GGA", data)) + "\r\n").encode()


def arguments():
    return SimpleNamespace(
        host="localhost",
        caster_port=2101,
        mount="TEST",
        connect_timeout=2,
        serial="/dev/fake",
        baud=115200,
        gga_interval=1,
        gga_timeout=15,
        rtcm_timeout=4,
    )


class Clock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


class FreshReader:
    updated_at = 0.0

    def wait_for_fix(self):
        return POSITION

    def get_latest(self):
        return POSITION


class ResponseSocket:
    def __init__(self, chunks):
        self.chunks = iter(chunks)
        self.timeout = 1.0
        self.reads = 0

    def gettimeout(self):
        return self.timeout

    def settimeout(self, timeout):
        self.timeout = timeout

    def recv(self, count):
        self.reads += 1
        chunk = next(self.chunks, b"")
        if isinstance(chunk, BaseException):
            raise chunk
        return chunk


class NtripResponseTests(unittest.TestCase):
    def test_short_errors_are_rejected_before_waiting_for_more_bytes(self):
        cases = [
            (b"HTTP/1.0 401 Unauthorized", "401"),
            (b"HTTP/1.1 403 Forbidden\r\n", "403"),
            (b"HTTP/1.1 404 Not Found", "404"),
            (b"ERROR - Bad MountPoint\r\n", "Bad MountPoint"),
            (b"SOURCETABLE 200 OK\r\n", "源表"),
        ]
        for raw, detail in cases:
            with self.subTest(raw=raw):
                sock = ResponseSocket([raw, AssertionError("must not wait again")])
                with self.assertRaisesRegex(relay.NtripConfigurationError, detail):
                    relay._read_ntrip_response(sock)
                self.assertEqual(sock.reads, 1)
                self.assertEqual(sock.timeout, 1.0)

    def test_fragmented_auth_error_is_recognized(self):
        sock = ResponseSocket([b"HTTP/1.0 4", b"0", b"1 Unauthorized"])
        with self.assertRaisesRegex(relay.NtripConfigurationError, "401"):
            relay._read_ntrip_response(sock)

    def test_icy_and_http_headers_preserve_pending_binary_bytes(self):
        for status in (b"ICY 200 OK", b"HTTP/1.1 200 OK"):
            with self.subTest(status=status):
                payload = b"\xd3\x00\x01\x80\xff\x00"
                sock = ResponseSocket(
                    [status[:3], status[3:] + b"\r\n", b"\r\n" + payload]
                )
                header, pending = relay._read_ntrip_response(sock)
                self.assertEqual(header, status)
                self.assertEqual(pending, payload)

    def test_server_failure_remains_retryable(self):
        sock = ResponseSocket([b"HTTP/1.1 503 Service Unavailable"])
        with self.assertRaises(relay.RelayError) as error:
            relay._read_ntrip_response(sock)
        self.assertNotIsInstance(error.exception, relay.NtripConfigurationError)

    def test_header_timeout_and_size_limit(self):
        with self.assertRaisesRegex(relay.RelayError, "超时"):
            relay._read_ntrip_response(ResponseSocket([socket.timeout()]))
        with self.assertRaisesRegex(relay.RelayError, "超过限制"):
            relay._read_ntrip_response(ResponseSocket([b"X" * 32]), limit=32)


class StreamTimeoutTests(unittest.TestCase):
    def exercise(self, first_packet=False):
        clock = Clock()
        port = FakeSerial()
        calls = []

        def receive(count):
            clock.now += 1
            if first_packet and clock.now == 1:
                return b"\xd3\x00\x00"
            raise socket.timeout()

        sock = FakeSocket(receive)
        with patch.object(relay.time, "monotonic", side_effect=clock):
            with patch.object(relay, "connect_mount", return_value=(sock, b"")):
                with self.assertRaises(relay.RelayError) as error:
                    relay.relay_session(
                        arguments(),
                        port,
                        "test",
                        "test",
                        FreshReader(),
                        lambda: calls.append("recovered"),
                    )
        self.assertTrue(sock.closed)
        self.assertGreater(
            len(sock.sent), 1, "periodic GGA must not reset the RTCM deadline"
        )
        return error.exception, port, calls, clock.now

    def test_no_first_packet_expires_despite_continued_gga_upload(self):
        error, port, calls, elapsed = self.exercise()
        self.assertIn("首包", str(error))
        self.assertEqual(port.writes, b"")
        self.assertEqual(calls, [])
        self.assertEqual(elapsed, 4)

    def test_stalled_stream_expires_and_marks_previous_recovery(self):
        error, port, calls, elapsed = self.exercise(first_packet=True)
        self.assertIn("已停止", str(error))
        self.assertEqual(port.writes, b"\xd3\x00\x00")
        self.assertEqual(calls, ["recovered"])
        self.assertEqual(elapsed, 5)

    def test_pending_data_also_resets_backoff(self):
        sock = FakeSocket(lambda count: b"")
        recovered = MagicMock()
        with patch.object(relay, "connect_mount", return_value=(sock, b"corrections")):
            with self.assertRaises(relay.RelayError):
                relay.relay_session(
                    arguments(), FakeSerial(), "test", "test", FreshReader(), recovered
                )
        recovered.assert_called_once()


class EpochTests(unittest.TestCase):
    def setUp(self):
        self.port = FakeSerial()
        self.reader = relay.GgaReader(self.port, stale_after=15)

    def feed(self, data, clock):
        self.port.buffer.extend(data)
        with patch.object(relay.time, "monotonic", return_value=clock):
            self.reader.poll()
            return self.reader.get_latest()

    def test_repeated_epoch_does_not_rejuvenate_an_old_position(self):
        self.assertIsNotNone(self.feed(POSITION, 100))
        self.assertIsNotNone(self.feed(POSITION, 110))
        self.assertEqual(self.reader.updated_at, 100)
        self.assertIsNone(self.feed(POSITION, 116))
        self.assertIsNotNone(self.feed(position_at("123520.00"), 117))

    def test_utc_crossing_midnight_remains_valid(self):
        self.assertIsNotNone(self.feed(position_at("235959.00"), 100))
        self.assertIsNotNone(self.feed(position_at("000000.00"), 101))
        self.assertEqual(self.reader.updated_at, 101)

    def test_utc_backwards_is_invalid_until_a_new_epoch_arrives(self):
        self.assertIsNotNone(self.feed(position_at("123519.00"), 100))
        self.assertIsNone(self.feed(position_at("123518.00"), 101))
        self.assertIsNone(self.feed(position_at("123518.00"), 102))
        self.assertIsNotNone(self.feed(position_at("123519.00"), 103))


class ThreadSerial:
    def __init__(self):
        self.buffer = bytearray()
        self.condition = threading.Condition()
        self.failed = False
        self.resets = 0

    @property
    def in_waiting(self):
        with self.condition:
            return len(self.buffer)

    def read(self, count):
        with self.condition:
            self.condition.wait_for(lambda: self.buffer or self.failed, timeout=0.05)
            if self.failed:
                raise serial.SerialException("device disconnected")
            chunk = bytes(self.buffer[:count])
            del self.buffer[:count]
            return chunk

    def reset_input_buffer(self):
        with self.condition:
            self.buffer.clear()
            self.resets += 1

    def feed(self, data):
        with self.condition:
            self.buffer.extend(data)
            self.condition.notify_all()


def wait_until(predicate):
    deadline = time.monotonic() + 2
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.005)
    raise AssertionError("serial worker did not reach the expected state")


class SerialWorkerTests(unittest.TestCase):
    def test_start_discards_queued_data_and_worker_keeps_latest_during_network_wait(
        self,
    ):
        port = ThreadSerial()
        port.feed(POSITION)
        with relay.GgaReader(port, stale_after=15) as reader:
            self.assertIsNone(reader.get_latest())
            self.assertEqual(port.resets, 1)
            port.feed(position_at("123520.00"))
            wait_until(lambda: reader.get_latest() is not None)
            port.feed(position_at("123521.00"))
            wait_until(lambda: reader.get_latest() == position_at("123521.00"))
            port.feed(NO_FIX)
            wait_until(lambda: reader.get_latest() is None)
        self.assertFalse(reader._thread.is_alive())

    def test_read_failure_is_propagated_and_old_position_cleared(self):
        port = ThreadSerial()
        with relay.GgaReader(port, stale_after=15) as reader:
            port.feed(POSITION)
            wait_until(lambda: reader.get_latest() is not None)
            with port.condition:
                port.failed = True
                port.condition.notify_all()
            wait_until(lambda: reader._error is not None)
            with self.assertRaises(relay.SerialLinkError):
                reader.get_latest()
            self.assertIsNone(reader.latest)

    def test_long_reader_pause_flushes_backlog(self):
        port = ThreadSerial()
        port.feed(POSITION)
        reader = relay.GgaReader(port, stale_after=15)
        clock = Clock()

        def wait(delay):
            if clock.now == 0:
                port.feed(POSITION)
                clock.now = 20
            else:
                reader._stop.set()

        with patch.object(relay.time, "monotonic", side_effect=clock):
            with patch.object(reader._stop, "wait", side_effect=wait):
                reader._run()
            self.assertIsNone(reader.get_latest())
        self.assertEqual(port.resets, 1)

    def test_serial_write_errors_and_zero_writes_are_classified(self):
        port = MagicMock()
        for result in (serial.SerialException("USB removed"), OSError("I/O error"), 0):
            with self.subTest(result=result):
                port.write.side_effect = (
                    result if isinstance(result, BaseException) else None
                )
                port.write.return_value = result
                with self.assertRaises(relay.SerialLinkError):
                    relay.write_all(port, b"corrections")


class RecoveryLoopTests(unittest.TestCase):
    def test_usb_fault_reopens_device_and_creates_a_new_reader(self):
        first_port, second_port = MagicMock(), MagicMock()
        first_port.__enter__.return_value = first_port
        second_port.__enter__.return_value = second_port
        first_reader, second_reader = MagicMock(), MagicMock()
        first_reader.__enter__.return_value = first_reader
        second_reader.__enter__.return_value = second_reader
        with patch.object(
            relay.serial, "Serial", side_effect=[first_port, second_port]
        ) as opened:
            with patch.object(
                relay, "GgaReader", side_effect=[first_reader, second_reader]
            ) as readers:
                with patch.object(
                    relay,
                    "relay_session",
                    side_effect=[relay.SerialLinkError("unplugged"), KeyboardInterrupt],
                ) as sessions:
                    with patch.object(relay.time, "sleep") as sleep:
                        with self.assertRaises(KeyboardInterrupt):
                            relay.run_relay(arguments(), "test", "test")
        self.assertEqual(opened.call_count, 2)
        self.assertEqual(readers.call_count, 2)
        self.assertIs(sessions.call_args_list[0].args[1], first_port)
        self.assertIs(sessions.call_args_list[1].args[1], second_port)
        self.assertIs(sessions.call_args_list[1].args[4], second_reader)
        first_reader.__exit__.assert_called_once()
        first_port.__exit__.assert_called_once()
        sleep.assert_called_once_with(1.0)

    def test_network_failure_reuses_serial_and_recovery_resets_backoff(self):
        port = MagicMock()
        port.__enter__.return_value = port
        attempts = []

        def session(*args):
            attempts.append(args)
            if len(attempts) == 4:
                raise KeyboardInterrupt
            if len(attempts) == 3:
                args[5]()
            raise relay.RelayError("network lost")

        with patch.object(relay.serial, "Serial", return_value=port) as opened:
            with patch.object(relay, "GgaReader") as readers:
                with patch.object(relay, "relay_session", side_effect=session):
                    with patch.object(relay.time, "sleep") as sleep:
                        with self.assertRaises(KeyboardInterrupt):
                            relay.run_relay(arguments(), "test", "test")
        self.assertEqual(opened.call_count, 1)
        self.assertEqual(readers.call_count, 1)
        self.assertEqual(
            [call.args[0] for call in sleep.call_args_list], [1.0, 2.0, 1.0]
        )

    def test_missing_serial_port_is_retried(self):
        with patch.object(
            relay.serial,
            "Serial",
            side_effect=[serial.SerialException("not present"), MagicMock()],
        ) as opened:
            with (
                patch.object(relay, "GgaReader"),
                patch.object(relay, "relay_session", side_effect=KeyboardInterrupt),
            ):
                with patch.object(relay.time, "sleep") as sleep:
                    with self.assertRaises(KeyboardInterrupt):
                        relay.run_relay(arguments(), "test", "test")
        self.assertEqual(opened.call_count, 2)
        sleep.assert_called_once_with(1.0)

    def test_bad_credentials_exit_without_retry_and_close_resources(self):
        with patch.object(relay.serial, "Serial") as opened:
            with patch.object(relay, "GgaReader") as readers:
                with patch.object(
                    relay,
                    "relay_session",
                    side_effect=relay.NtripConfigurationError("401"),
                ) as session:
                    with patch.object(relay.time, "sleep") as sleep:
                        self.assertEqual(
                            relay.run_relay(arguments(), "test", "test"), 2
                        )
        self.assertEqual(session.call_count, 1)
        sleep.assert_not_called()
        readers.return_value.__exit__.assert_called_once()
        opened.return_value.__exit__.assert_called_once()


@unittest.skipUnless(os.name == "posix", "PTY integration requires POSIX")
class LocalIntegrationTests(unittest.TestCase):
    def test_tcp_ntrip_to_pyserial_pty_with_continuous_gga_and_stall_recovery(self):
        import pty

        master, slave = pty.openpty()
        server = socket.socket()
        server.bind(("127.0.0.1", 0))
        server.listen(1)
        server.settimeout(2)
        payload = b"\xd3\x00\x01\x80\xff\x00"
        seen = {}
        errors = []
        stop = threading.Event()

        def serve():
            try:
                conn, _ = server.accept()
                with conn:
                    conn.settimeout(2)
                    request = bytearray()
                    while b"\r\n\r\n" not in request:
                        data = conn.recv(4096)
                        if not data:
                            raise AssertionError(
                                "client closed before sending a request"
                            )
                        request.extend(data)
                    seen["request"] = bytes(request)
                    conn.sendall(b"ICY 200 OK\r\n\r\n")
                    gga = bytearray()
                    while b"\r\n" not in gga:
                        data = conn.recv(512)
                        if not data:
                            raise AssertionError("client closed before uploading GGA")
                        gga.extend(data)
                    seen["gga"] = bytes(gga).split(b"\r\n")[0] + b"\r\n"
                    conn.sendall(payload)
                    while conn.recv(512):
                        pass
                    seen["closed"] = True
            except Exception as exc:
                errors.append(exc)

        def feed():
            start = datetime(2026, 1, 1, 12, 35, 19)
            epoch = 0
            try:
                while not stop.is_set():
                    stamp = (start + timedelta(seconds=epoch)).strftime("%H%M%S.00")
                    os.write(master, position_at(stamp))
                    epoch += 1
                    stop.wait(0.02)
            except Exception as exc:
                errors.append(exc)

        caster = threading.Thread(target=serve, daemon=True)
        feeder = threading.Thread(target=feed, daemon=True)
        args = arguments()
        args.host, args.caster_port = server.getsockname()
        args.rtcm_timeout = 0.4
        try:
            caster.start()
            with serial.Serial(
                os.ttyname(slave), timeout=0.05, write_timeout=1
            ) as port:
                with relay.GgaReader(port, stale_after=15) as reader:
                    feeder.start()
                    with self.assertRaisesRegex(relay.RelayError, "已停止"):
                        relay.relay_session(args, port, "test", "test", reader)
                    self.assertIsNotNone(reader.get_latest())
            ready, _, _ = select.select([master], [], [], 1)
            self.assertTrue(ready)
            self.assertEqual(os.read(master, 4096), payload)
        finally:
            stop.set()
            if feeder.ident is not None:
                feeder.join(timeout=2)
            caster.join(timeout=3)
            server.close()
            os.close(master)
            os.close(slave)
        self.assertFalse(caster.is_alive())
        self.assertFalse(feeder.is_alive())
        self.assertEqual(errors, [])
        self.assertIn(b"GET /TEST HTTP/1.0", seen["request"])
        self.assertTrue(relay.parse_gga(seen["gga"]).has_position)
        self.assertTrue(seen["closed"])


if __name__ == "__main__":
    unittest.main()
