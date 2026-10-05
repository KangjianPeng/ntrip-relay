import unittest
from types import SimpleNamespace
from unittest.mock import patch

import pynmea2

from ntrip_relay import GgaReader, RelayError, parse_gga, relay_session

NO_FIX = b"$GNGGA,,,,,,0,,,,,,,,*78\r\n"
# Synthetic fixtures exercise the parser; production data comes only from UART.
POSITION = b"$GPGGA,123519.00,3112.3456,N,12128.1234,E,1,12,0.8,10.0,M,0.0,M,,*62\r\n"


def variant(*, talker="GP", **fields):
    original = pynmea2.parse(POSITION.decode("ascii").strip(), check=True)
    data = list(original.data)
    indices = {
        "latitude": 1,
        "north_south": 2,
        "longitude": 3,
        "east_west": 4,
        "quality": 5,
    }
    for name, value in fields.items():
        data[indices[name]] = str(value)
    return (str(pynmea2.GGA(talker, "GGA", data)) + "\r\n").encode("ascii")


class FakeSerial:
    def __init__(self):
        self.buffer = bytearray()
        self.writes = bytearray()

    @property
    def in_waiting(self):
        return len(self.buffer)

    def read(self, count):
        chunk = bytes(self.buffer[:count])
        del self.buffer[:count]
        return chunk

    def write(self, data):
        self.writes.extend(data)
        return len(data)


class GgaParserTests(unittest.TestCase):
    def test_real_no_fix_sample_has_no_invented_values(self):
        gga = parse_gga(NO_FIX)
        self.assertIsNotNone(gga)
        self.assertEqual(gga.quality, 0)
        self.assertFalse(gga.has_position)
        self.assertIsNone(gga.latitude)
        self.assertIsNone(gga.longitude)
        self.assertIsNone(gga.satellites)
        self.assertIsNone(gga.hdop)
        self.assertIsNone(gga.altitude_m)
        self.assertIsNone(gga.time_utc)
        self.assertEqual(gga.sentence, NO_FIX)

    def test_single_point_fix_is_enough_and_original_bytes_are_preserved(self):
        gga = parse_gga(POSITION)
        self.assertTrue(gga.has_position)
        self.assertAlmostEqual(gga.latitude, 31 + 12.3456 / 60)
        self.assertAlmostEqual(gga.longitude, 121 + 28.1234 / 60)
        self.assertEqual(gga.satellites, 12)
        self.assertEqual(gga.hdop, 0.8)
        self.assertEqual(gga.altitude_m, 10.0)
        self.assertEqual(gga.time_utc, "12:35:19+00:00")
        self.assertEqual(gga.sentence, POSITION)

    def test_gn_talker_and_south_west_hemispheres(self):
        gga = parse_gga(variant(talker="GN", north_south="S", east_west="W"))
        self.assertTrue(gga.has_position)
        self.assertLess(gga.latitude, 0)
        self.assertLess(gga.longitude, 0)

    def test_checksum_must_be_present_and_correct(self):
        self.assertIsNone(parse_gga(NO_FIX.replace(b"*78", b"*00")))
        self.assertIsNone(parse_gga(NO_FIX.split(b"*")[0]))
        self.assertIsNone(parse_gga(NO_FIX.replace(b"GNGGA", b"GN\xffGGA")))

    def test_missing_and_out_of_range_coordinates_are_not_usable(self):
        cases = [
            {"latitude": ""},
            {"longitude": ""},
            {"latitude": "3160.0"},
            {"latitude": "9100.0"},
            {"longitude": "18100.0"},
            {"north_south": "X"},
        ]
        for fields in cases:
            with self.subTest(fields=fields):
                self.assertFalse(parse_gga(variant(**fields)).has_position)

    def test_artificial_and_dead_reckoning_positions_are_not_usable(self):
        for quality in (6, 7, 8):
            with self.subTest(quality=quality):
                self.assertFalse(parse_gga(variant(quality=quality)).has_position)

    def test_float_and_fixed_are_recognized(self):
        for quality in (4, 5):
            with self.subTest(quality=quality):
                self.assertTrue(parse_gga(variant(quality=quality)).has_position)


class GgaReaderTests(unittest.TestCase):
    def setUp(self):
        self.port = FakeSerial()
        self.reader = GgaReader(self.port, stale_after=15)

    def feed(self, value):
        self.port.buffer.extend(value)
        self.reader.poll()

    def test_fragmented_uart_data(self):
        self.feed(POSITION[:30])
        self.assertIsNone(self.reader.latest)
        self.feed(POSITION[30:])
        self.assertEqual(self.reader.latest, POSITION)
        self.assertTrue(self.reader.is_fresh())

    def test_no_fix_immediately_clears_previous_position_and_can_recover(self):
        self.feed(POSITION)
        self.assertTrue(self.reader.is_fresh())
        self.feed(NO_FIX)
        self.assertIsNone(self.reader.latest)
        self.assertFalse(self.reader.is_fresh())
        self.assertEqual(self.reader.last_observation.quality, 0)
        self.feed(POSITION)
        self.assertTrue(self.reader.is_fresh())

    def test_many_no_fix_sentences_do_not_supply_a_location(self):
        self.feed(NO_FIX * 3)
        self.assertIsNone(self.reader.latest)
        self.assertFalse(self.reader.is_fresh())

    def test_corrupted_data_cannot_replace_a_verified_position(self):
        self.feed(POSITION)
        self.feed(NO_FIX.replace(b"*78", b"*00"))
        self.assertEqual(self.reader.latest, POSITION)

    def test_expired_position_is_not_fresh(self):
        with patch("ntrip_relay.time.monotonic", return_value=100):
            self.feed(POSITION)
        with patch("ntrip_relay.time.monotonic", return_value=116):
            self.assertFalse(self.reader.is_fresh())


class FakeSocket:
    def __init__(self, receive):
        self.receive = receive
        self.sent = []
        self.closed = False

    def sendall(self, data):
        self.sent.append(data)

    def recv(self, count):
        return self.receive(count)

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.closed = True


class RelayPositionTests(unittest.TestCase):
    def setUp(self):
        self.port = FakeSerial()
        self.reader = GgaReader(self.port, stale_after=15)
        self.args = SimpleNamespace(
            host="localhost",
            caster_port=2101,
            mount="TEST",
            connect_timeout=2,
            gga_interval=5,
            rtcm_timeout=15,
        )

    def test_no_connection_while_only_no_fix_data_is_received(self):
        self.port.buffer.extend(NO_FIX)
        self.reader.poll()
        with patch("ntrip_relay.connect_mount") as connect:
            with patch.object(self.reader._stop, "wait", side_effect=KeyboardInterrupt):
                with self.assertRaises(KeyboardInterrupt):
                    relay_session(self.args, self.port, "user", "password", self.reader)
            connect.assert_not_called()
        self.assertEqual(self.port.writes, b"")

    def test_fix_lost_during_connection_is_not_uploaded(self):
        self.port.buffer.extend(POSITION)
        self.reader.poll()
        sock = FakeSocket(lambda count: b"")

        def connect(*args):
            self.port.buffer.extend(NO_FIX)
            self.reader.poll()
            return sock, b"corrections already received"

        with patch("ntrip_relay.connect_mount", side_effect=connect):
            with self.assertRaises(RelayError):
                relay_session(self.args, self.port, "user", "password", self.reader)
        self.assertEqual(sock.sent, [])
        self.assertEqual(self.port.writes, b"")
        self.assertTrue(sock.closed)

    def test_fix_lost_while_receiving_corrections_stops_forwarding(self):
        self.port.buffer.extend(POSITION)
        self.reader.poll()

        def receive(count):
            self.port.buffer.extend(NO_FIX)
            self.reader.poll()
            return b"corrections"

        sock = FakeSocket(receive)
        with patch("ntrip_relay.connect_mount", return_value=(sock, b"")):
            with self.assertRaises(RelayError):
                relay_session(self.args, self.port, "user", "password", self.reader)
        self.assertEqual(sock.sent, [POSITION])
        self.assertEqual(self.port.writes, b"")
        self.assertTrue(sock.closed)

    def test_real_uart_sentence_and_binary_corrections_are_preserved(self):
        self.port.buffer.extend(POSITION)
        self.reader.poll()
        payload = b"\xd3\x00\x01\x00\xff\x80\x00"
        chunks = iter((payload, b""))
        sock = FakeSocket(lambda count: next(chunks))
        with patch("ntrip_relay.connect_mount", return_value=(sock, b"")):
            with self.assertRaises(RelayError):
                relay_session(self.args, self.port, "user", "password", self.reader)
        self.assertEqual(sock.sent, [POSITION])
        self.assertEqual(self.port.writes, payload)
        self.assertTrue(sock.closed)


if __name__ == "__main__":
    unittest.main()
