#!/usr/bin/env python3
"""Forward NTRIP RTCM corrections to a GNSS receiver over a serial port."""

from __future__ import annotations

import argparse
import base64
import getpass
import logging
import math
import os
import re
import socket
import sys
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
from datetime import time as utc_time
from pathlib import Path
from typing import Union

import pynmea2
import serial
from dotenv import load_dotenv

LOG = logging.getLogger("ntrip_gnss_relay")
DEFAULT_ENV_FILE = Path(__file__).resolve().with_name(".env")


class RelayError(RuntimeError):
    pass


class SerialLinkError(RelayError):
    pass


class NtripConfigurationError(RelayError):
    pass


class RetryBackoff:
    def __init__(self):
        self.delay = 1.0

    def reset(self) -> None:
        self.delay = 1.0

    def next_delay(self) -> float:
        delay = self.delay
        self.delay = min(delay * 2, 30.0)
        return delay


FIX_NAMES = {
    0: "No fix",
    1: "Autonomous",
    2: "Differential",
    3: "PPS",
    4: "RTK fixed",
    5: "RTK float",
    6: "Estimated",
    7: "Manual",
    8: "Simulation",
}

PVT_TOPIC = "/gnss/pvt"
GPS_EPOCH = datetime(1980, 1, 6, tzinfo=timezone.utc)
GPS_UTC_LEAP_SECONDS = 18
SECONDS_PER_GPS_WEEK = 604800
RMC_MAX_AGE = 5.0


def configure_logging(verbose: bool) -> None:
    handler = logging.StreamHandler(sys.stderr)
    handler.setFormatter(
        logging.Formatter(
            "%(asctime)s %(levelname)s %(message)s",
            datefmt="%H:%M:%S",
        )
    )
    LOG.handlers.clear()
    LOG.addHandler(handler)
    LOG.setLevel(logging.DEBUG if verbose else logging.INFO)
    LOG.propagate = False


@dataclass(frozen=True)
class GgaData:
    sentence: bytes
    time_utc: str | None
    quality: int
    latitude: float | None
    longitude: float | None
    satellites: int | None
    hdop: float | None
    altitude_m: float | None
    geoid_separation_m: float | None

    @property
    def has_position(self) -> bool:
        return (
            self.quality in (1, 2, 3, 4, 5)
            and self.latitude is not None
            and self.longitude is not None
            and self.time_utc is not None
        )


@dataclass(frozen=True)
class RmcData:
    timestamp: datetime
    speed_mps: float
    course_deg: float
    valid: bool


@dataclass(frozen=True)
class GsaData:
    mode: int
    pdop: float


@dataclass(frozen=True)
class GstData:
    h_acc: float
    v_acc: float


AuxiliaryNmea = Union[RmcData, GsaData, GstData]


def _valid_coordinate(raw: str, direction: str, limit: int) -> bool:
    if not raw:
        return False
    directions = ("N", "S") if limit == 90 else ("E", "W")
    if direction not in directions or not raw.replace(".", "", 1).isdigit():
        return False
    value = float(raw)
    if not math.isfinite(value):
        return False
    degrees = int(value // 100)
    minutes = value - degrees * 100
    return 0 <= minutes < 60 and degrees <= limit and (degrees != limit or minutes == 0)


def parse_gga(value: str | bytes) -> GgaData | None:
    """Parse receiver data, including no-fix GGA; never synthesize a position."""
    try:
        text = value.decode("ascii") if isinstance(value, bytes) else value
        text = text.strip()
        if not text.startswith("$"):
            return None
        message = pynmea2.parse(text, check=True)
        if not isinstance(message, pynmea2.GGA) or len(message.data) != 14:
            return None
        quality = int(message.gps_qual)
        if quality not in FIX_NAMES:
            return None
        satellites = int(message.num_sats) if message.num_sats else None
        hdop = float(message.horizontal_dil) if message.horizontal_dil else None
        altitude = (
            float(message.altitude) if message.altitude not in (None, "") else None
        )
        geoid_separation = (
            float(message.geo_sep) if message.geo_sep not in (None, "") else None
        )
        if satellites is not None and satellites < 0:
            return None
        if hdop is not None and (not math.isfinite(hdop) or hdop < 0):
            return None
        if altitude is not None and not math.isfinite(altitude):
            return None
        if geoid_separation is not None and not math.isfinite(geoid_separation):
            return None
        return GgaData(
            sentence=(text + "\r\n").encode("ascii"),
            time_utc=message.timestamp.isoformat() if message.data[0] else None,
            quality=quality,
            latitude=message.latitude
            if _valid_coordinate(message.lat, message.lat_dir, 90)
            else None,
            longitude=message.longitude
            if _valid_coordinate(message.lon, message.lon_dir, 180)
            else None,
            satellites=satellites,
            hdop=hdop,
            altitude_m=altitude if message.altitude_units == "M" else None,
            geoid_separation_m=(
                geoid_separation if message.geo_sep_units == "M" else None
            ),
        )
    except (pynmea2.ParseError, ValueError, TypeError, OverflowError):
        return None


def parse_auxiliary_nmea(value: str | bytes) -> AuxiliaryNmea | None:
    """Parse the RMC, GSA, and GST fields used to assemble a PVT solution."""
    try:
        text = value.decode("ascii") if isinstance(value, bytes) else value
        text = text.strip()
        if not text.startswith("$"):
            return None
        message = pynmea2.parse(text, check=True)
        sentence_type = message.sentence_type
        if sentence_type == "RMC":
            timestamp = datetime.combine(
                message.datestamp,
                message.timestamp,
            ).replace(tzinfo=timezone.utc)
            speed = float(message.spd_over_grnd or 0.0) * 0.5144444444444445
            course = float(message.true_course or 0.0)
            if (
                not math.isfinite(speed)
                or speed < 0
                or not math.isfinite(course)
                or not 0 <= course <= 360
            ):
                return None
            return RmcData(
                timestamp=timestamp,
                speed_mps=speed,
                course_deg=course,
                valid=message.status == "A",
            )
        if sentence_type == "GSA":
            mode = int(message.mode_fix_type)
            pdop = float(message.pdop) if message.pdop else 0.0
            if mode not in (1, 2, 3) or not math.isfinite(pdop) or pdop < 0:
                return None
            return GsaData(mode=mode, pdop=pdop)
        if sentence_type == "GST":
            lat_error = float(message.std_dev_latitude or 0.0)
            lon_error = float(message.std_dev_longitude or 0.0)
            alt_error = float(message.std_dev_altitude or 0.0)
            if any(
                not math.isfinite(error) or error < 0
                for error in (lat_error, lon_error, alt_error)
            ):
                return None
            return GstData(
                h_acc=math.hypot(lat_error, lon_error),
                v_acc=alt_error,
            )
        return None
    except (pynmea2.ParseError, AttributeError, ValueError, TypeError, OverflowError):
        return None


class GgaReader:
    """Read checked NMEA sentences and retain the latest usable GGA."""

    def __init__(
        self,
        serial_port,
        stale_after: float,
        on_gga: Callable[[GgaData], None] | None = None,
        on_auxiliary_nmea: Callable[[AuxiliaryNmea], None] | None = None,
        shutdown_event: threading.Event | None = None,
    ):
        self.serial_port = serial_port
        self.stale_after = stale_after
        self.buffer = bytearray()
        self.latest: bytes | None = None
        self.updated_at: float | None = None
        self.last_observation: GgaData | None = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._error: SerialLinkError | None = None
        self._last_utc_seconds: float | None = None
        self._epoch_changed_at: float | None = None
        self.on_gga = on_gga
        self.on_auxiliary_nmea = on_auxiliary_nmea
        self.shutdown_event = shutdown_event
        self._last_callback_error: float | None = None
        self._last_auxiliary_callback_error: float | None = None

    def __enter__(self):
        self._discard_backlog()
        self._thread = threading.Thread(target=self._run, name="gnss-nmea", daemon=True)
        self._thread.start()
        return self

    def __exit__(self, *args):
        self._stop.set()
        if self._thread is not None:
            self._thread.join()

    def _discard_backlog(self) -> None:
        try:
            self.serial_port.reset_input_buffer()
        except (OSError, serial.SerialException) as exc:
            raise SerialLinkError(
                f"Failed to clear serial input buffer: {exc}"
            ) from exc
        self.buffer.clear()
        with self._lock:
            self.latest = None
            self.updated_at = None
            self.last_observation = None
            self._last_utc_seconds = None
            self._epoch_changed_at = None

    def _run(self) -> None:
        last_poll = time.monotonic()
        try:
            while not self.should_stop():
                now = time.monotonic()
                if now - last_poll > self.stale_after:
                    LOG.warning("Serial reader stalled; input buffer cleared")
                    self._discard_backlog()
                self.poll()
                last_poll = now
                self._stop.wait(0.01)
        except (OSError, serial.SerialException, SerialLinkError) as exc:
            with self._lock:
                self.latest = None
                self._error = (
                    exc
                    if isinstance(exc, SerialLinkError)
                    else SerialLinkError(f"Serial read failed: {exc}")
                )

    def _record(self, gga: GgaData, now: float) -> None:
        # A repeated UTC epoch must not renew freshness, even if UART keeps sending it.
        backwards = False
        if gga.time_utc is not None:
            stamp = utc_time.fromisoformat(gga.time_utc)
            seconds = (
                stamp.hour * 3600
                + stamp.minute * 60
                + stamp.second
                + stamp.microsecond / 1e6
            )
            if self._last_utc_seconds is None:
                self._epoch_changed_at = now
            else:
                delta = (seconds - self._last_utc_seconds + 43200) % 86400 - 43200
                if delta > 0:
                    self._epoch_changed_at = now
                elif delta < 0:
                    self._epoch_changed_at = None
                    backwards = True
            self._last_utc_seconds = seconds

        with self._lock:
            previous = self.last_observation
            self.last_observation = gga
            usable = (
                gga.has_position
                and self._epoch_changed_at is not None
                and now - self._epoch_changed_at <= self.stale_after
            )
            was_usable = self.latest is not None
            self.latest = gga.sentence if usable else None
            self.updated_at = self._epoch_changed_at if usable else None
        changed = (
            previous is None or previous.quality != gga.quality or was_usable != usable
        )
        if backwards:
            LOG.warning("GGA UTC moved backwards; position cleared")
        if changed:
            self._log_position(gga)
            if gga.has_position and not usable:
                LOG.warning("GGA epoch is stale or unchanged; fix ignored")
        if self.on_gga is not None:
            try:
                self.on_gga(gga)
                self._last_callback_error = None
            except Exception:
                if (
                    self._last_callback_error is None
                    or now - self._last_callback_error >= 5
                ):
                    LOG.exception("Failed to publish GNSS PVT")
                    self._last_callback_error = now

    def poll(self) -> None:
        available = getattr(self.serial_port, "in_waiting", 0)
        chunk = self.serial_port.read(min(max(available, 1), 4096))
        if not chunk:
            return

        self.buffer.extend(chunk)
        while b"\n" in self.buffer:
            line, _, remainder = self.buffer.partition(b"\n")
            self.buffer[:] = remainder
            gga = parse_gga(bytes(line))
            now = time.monotonic()
            if gga is not None:
                self._record(gga, now)
                continue
            sentence = parse_auxiliary_nmea(bytes(line))
            if sentence is not None:
                if self.on_auxiliary_nmea is not None:
                    try:
                        self.on_auxiliary_nmea(sentence)
                        self._last_auxiliary_callback_error = None
                    except Exception:
                        if (
                            self._last_auxiliary_callback_error is None
                            or now - self._last_auxiliary_callback_error >= 5
                        ):
                            LOG.exception("Failed to process auxiliary NMEA sentence")
                            self._last_auxiliary_callback_error = now
                continue
            LOG.debug("Ignored invalid or unsupported NMEA sentence: %r", line[:160])

        # Recover from garbage or incomplete oversized sentences on the UART.
        if len(self.buffer) > 8192:
            start = self.buffer.rfind(b"$")
            self.buffer[:] = self.buffer[start:] if start >= 0 else b""
            if len(self.buffer) > 8192:
                self.buffer.clear()

    def _log_position(self, gga: GgaData) -> None:
        LOG.debug("Raw GGA: %s", gga.sentence.decode("ascii").strip())
        if not gga.has_position:
            LOG.warning("No valid GNSS fix (quality=%d)", gga.quality)
            return
        LOG.info("GNSS fix: %s (quality=%d)", FIX_NAMES[gga.quality], gga.quality)

    def get_latest(self) -> bytes | None:
        with self._lock:
            if self._error is not None:
                raise self._error
            if (
                self.updated_at is None
                or time.monotonic() - self.updated_at > self.stale_after
            ):
                return None
            return self.latest

    def is_fresh(self) -> bool:
        return self.get_latest() is not None

    def wait_for_fix(self) -> bool:
        last_notice = 0.0
        while not self.should_stop():
            if self.get_latest() is not None:
                return True
            now = time.monotonic()
            if now - last_notice >= 30:
                LOG.info("Waiting for a valid GNSS fix")
                last_notice = now
            self._stop.wait(0.1)
        return False

    def should_stop(self) -> bool:
        return self._stop.is_set() or (
            self.shutdown_event is not None and self.shutdown_event.is_set()
        )


class Ros1PvtPublisher:
    """Publish GNSS PVT solutions assembled from checked NMEA sentences."""

    topic = PVT_TOPIC

    def __init__(self, ros_args: list[str]):
        try:
            import rospy
            from gnss_comm.msg import GnssPVTSolnMsg
        except ImportError as exc:
            raise RuntimeError("ROS 1, rospy, and gnss_comm are required") from exc

        self.GnssPVTSolnMsg = GnssPVTSolnMsg
        message = GnssPVTSolnMsg()
        message_module = sys.modules.get(GnssPVTSolnMsg.__module__)
        message_path = getattr(message_module, "__file__", "<unknown>")
        required_fields = (
            "time",
            "fix_type",
            "valid_fix",
            "diff_soln",
            "carr_soln",
            "num_sv",
            "latitude",
            "longitude",
            "altitude",
            "height_msl",
            "h_acc",
            "v_acc",
            "p_dop",
            "vel_n",
            "vel_e",
            "vel_d",
            "vel_acc",
        )
        missing_fields = [
            field for field in required_fields if not hasattr(message, field)
        ]
        time_message = getattr(message, "time", None)
        missing_time_fields = not all(
            hasattr(time_message, field) for field in ("week", "tow")
        )
        if missing_fields or missing_time_fields:
            actual_fields = ", ".join(getattr(message, "__slots__", ()))
            missing = list(missing_fields)
            if missing_time_fields:
                missing.append("time.week/time.tow")
            raise RuntimeError(
                "Incompatible gnss_comm/GnssPVTSolnMsg: "
                f"missing fields {', '.join(missing)} in {message_path} "
                f"(available: {actual_fields})"
            )

        self.stop_event = threading.Event()
        rospy.init_node(
            "ntrip_gnss_relay",
            argv=[sys.argv[0], *ros_args],
        )
        self.pvt_publisher = rospy.Publisher(self.topic, GnssPVTSolnMsg, queue_size=10)
        LOG.info(
            "PVT publisher ready: %s [%s]",
            self.topic,
            getattr(GnssPVTSolnMsg, "_type", "gnss_comm/GnssPVTSolnMsg"),
        )
        LOG.debug("PVT message definition: %s", message_path)
        self._latest_rmc: tuple[RmcData, float] | None = None
        self._latest_gsa: tuple[GsaData, float] | None = None
        self._latest_gst: tuple[GstData, float] | None = None
        self._pending_gga: tuple[GgaData, float] | None = None
        self._last_pvt_status = ""
        rospy.on_shutdown(self.stop_event.set)

    def publish(self, gga: GgaData) -> None:
        now = time.monotonic()
        if not self._publish_pvt(gga, now):
            self._pending_gga = (gga, now)

    def update_auxiliary(self, sentence: AuxiliaryNmea) -> None:
        now = time.monotonic()
        if isinstance(sentence, RmcData):
            self._latest_rmc = (sentence, now)
            pending = self._pending_gga
            if pending is not None and now - pending[1] <= RMC_MAX_AGE:
                self._publish_pvt(pending[0], now)
                self._pending_gga = None
        elif isinstance(sentence, GsaData):
            self._latest_gsa = (sentence, now)
        elif isinstance(sentence, GstData):
            self._latest_gst = (sentence, now)

    def _publish_pvt(self, gga: GgaData, now: float) -> bool:
        if not gga.has_position:
            return False
        if gga.altitude_m is None or gga.geoid_separation_m is None:
            self._log_pvt_status(
                "PVT skipped: missing altitude or geoid separation in meters"
            )
            return False
        latest_rmc = self._latest_rmc
        if latest_rmc is None or now - latest_rmc[1] > RMC_MAX_AGE:
            self._log_pvt_status("PVT skipped: no recent valid RMC timestamp")
            return False

        latest_gsa = self._latest_gsa
        has_recent_gsa = latest_gsa is not None and now - latest_gsa[1] <= 5.0
        if has_recent_gsa and latest_gsa[0].mode == 1:
            self._log_pvt_status("PVT skipped: recent GSA reports no fix")
            return False

        rmc = latest_rmc[0]
        try:
            gga_time = utc_time.fromisoformat(gga.time_utc)
        except (TypeError, ValueError):
            return False
        utc_timestamp = datetime.combine(rmc.timestamp.date(), gga_time).replace(
            tzinfo=timezone.utc
        )
        date_delta = (utc_timestamp - rmc.timestamp).total_seconds()
        if date_delta > 43200:
            utc_timestamp -= timedelta(days=1)
        elif date_delta < -43200:
            utc_timestamp += timedelta(days=1)
        gps_timestamp = utc_timestamp + timedelta(seconds=GPS_UTC_LEAP_SECONDS)
        gps_elapsed = (gps_timestamp - GPS_EPOCH).total_seconds()
        week, tow = divmod(gps_elapsed, SECONDS_PER_GPS_WEEK)

        msg = self.GnssPVTSolnMsg()
        msg.time.week = int(week)
        msg.time.tow = tow
        msg.fix_type = 2 if has_recent_gsa and latest_gsa[0].mode == 2 else 3
        msg.valid_fix = True
        msg.diff_soln = gga.quality in (2, 4, 5)
        msg.carr_soln = {4: 2, 5: 1}.get(gga.quality, 0)
        msg.num_sv = gga.satellites or 0
        msg.latitude = gga.latitude
        msg.longitude = gga.longitude
        msg.altitude = gga.altitude_m + gga.geoid_separation_m
        msg.height_msl = gga.altitude_m
        msg.p_dop = latest_gsa[0].pdop if has_recent_gsa else 0.0
        latest_gst = self._latest_gst
        if latest_gst is not None and now - latest_gst[1] <= 2.0:
            msg.h_acc = latest_gst[0].h_acc
            msg.v_acc = latest_gst[0].v_acc
        else:
            msg.h_acc = msg.v_acc = 0.0
        speed_mps = rmc.speed_mps if rmc.valid else 0.0
        course_rad = math.radians(rmc.course_deg)
        msg.vel_n = speed_mps * math.cos(course_rad)
        msg.vel_e = speed_mps * math.sin(course_rad)
        msg.vel_d = 0.0
        msg.vel_acc = 0.0
        self.pvt_publisher.publish(msg)
        self._log_pvt_status(
            f"PVT published: {FIX_NAMES[gga.quality]} (quality={gga.quality})",
        )
        return True

    def _log_pvt_status(self, status: str) -> None:
        if status != self._last_pvt_status:
            LOG.info("%s", status)
            self._last_pvt_status = status

    def close(self) -> None:
        self.stop_event.set()
        self.pvt_publisher.unregister()


def _check_ntrip_status(status: bytes) -> int | None:
    if status.startswith(b"SOURCETABLE"):
        raise NtripConfigurationError(
            "Caster returned a source table; check the mountpoint"
        )
    if status.startswith(b"ERROR - Bad MountPoint"):
        raise NtripConfigurationError("Mountpoint unavailable (Bad MountPoint)")
    match = re.match(rb"(?:HTTP/\d\.\d|ICY)\s+(\d{3})(?:\s|$)", status)
    if match is None:
        return None
    code = int(match.group(1))
    if code in (400, 401, 403, 404, 410):
        details = {
            400: "invalid request",
            401: "authentication failed or credentials expired",
            403: "access denied",
            404: "mountpoint not found",
            410: "mountpoint disabled",
        }
        raise NtripConfigurationError(f"NTRIP {code}: {details[code]}")
    if code != 200:
        raise RelayError(f"NTRIP server returned {code}")
    return code


def _read_ntrip_response(
    sock: socket.socket, limit: int = 16384
) -> tuple[bytes, bytes]:
    response = bytearray()
    timeout = sock.gettimeout()
    deadline = time.monotonic() + (timeout if timeout is not None else 10.0)
    try:
        while True:
            # Some casters return short errors without a complete HTTP header.
            status = bytes(response).split(b"\r\n", 1)[0]
            code = _check_ntrip_status(status)
            if b"\r\n\r\n" in response:
                if code != 200:
                    raise RelayError("Invalid NTRIP response")
                return tuple(bytes(response).split(b"\r\n\r\n", 1))
            if len(response) >= limit:
                raise RelayError("NTRIP response headers exceed size limit")
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise RelayError("NTRIP response timeout")
            sock.settimeout(remaining)
            try:
                chunk = sock.recv(min(4096, limit - len(response)))
            except socket.timeout as exc:
                raise RelayError("NTRIP response timeout") from exc
            if not chunk:
                raise RelayError("NTRIP server closed connection before response headers")
            response.extend(chunk)
    finally:
        sock.settimeout(timeout)


def connect_mount(
    host: str, port: int, mount: str, username: str, password: str, timeout: float
) -> tuple[socket.socket, bytes]:
    sock = socket.create_connection((host, port), timeout=timeout)
    sock.settimeout(timeout)
    token = base64.b64encode(f"{username}:{password}".encode("utf-8")).decode("ascii")
    request = (
        f"GET /{mount} HTTP/1.0\r\n"
        f"Host: {host}:{port}\r\n"
        "User-Agent: NTRIP GNSSRelay/1.0\r\n"
        "Ntrip-Version: Ntrip/2.0\r\n"
        "Accept: */*\r\n"
        f"Authorization: Basic {token}\r\n"
        "Connection: keep-alive\r\n\r\n"
    ).encode("ascii")
    try:
        sock.sendall(request)
        response, remainder = _read_ntrip_response(sock)
        sock.settimeout(0.2)
        return sock, remainder
    except BaseException:
        sock.close()
        raise


def write_all(serial_port, data: bytes) -> None:
    offset = 0
    while offset < len(data):
        try:
            count = serial_port.write(data[offset:])
        except (OSError, serial.SerialException) as exc:
            raise SerialLinkError(f"Serial write failed: {exc}") from exc
        if not count:
            raise SerialLinkError("Serial write timeout")
        offset += count


def relay_session(
    args,
    serial_port,
    username: str,
    password: str,
    gga_reader: GgaReader,
    on_data: Callable[[], None] | None = None,
) -> None:
    if not gga_reader.wait_for_fix():
        return
    sock, pending = connect_mount(
        args.host,
        args.caster_port,
        args.mount,
        username,
        password,
        args.connect_timeout,
    )
    with sock:
        gga = gga_reader.get_latest()
        if gga is None:
            raise RelayError("No fresh GNSS fix")
        sock.sendall(gga)
        last_gga_sent = time.monotonic()
        rtcm_wait_started_at = last_gga_sent
        last_report = last_gga_sent
        last_rtcm_at: float | None = None
        total_bytes = 0
        LOG.info(
            "NTRIP connected: %s:%d/%s",
            args.host,
            args.caster_port,
            args.mount,
        )

        if pending:
            write_all(serial_port, pending)
            total_bytes += len(pending)
            last_rtcm_at = time.monotonic()
            LOG.info("RTCM received: %d bytes", len(pending))
            if on_data:
                on_data()

        while not gga_reader.should_stop():
            now = time.monotonic()
            gga = gga_reader.get_latest()
            if gga is None:
                raise RelayError("GNSS fix expired")
            since = last_rtcm_at if last_rtcm_at is not None else rtcm_wait_started_at
            if now - since >= args.rtcm_timeout:
                detail = (
                    "No RTCM data received"
                    if last_rtcm_at is None
                    else "RTCM stream stalled"
                )
                raise RelayError(f"{detail} for {args.rtcm_timeout:g}s")
            if now - last_gga_sent >= args.gga_interval:
                sock.sendall(gga)
                last_gga_sent = now

            timed_out = False
            try:
                data = sock.recv(4096)
            except socket.timeout:
                data = b""
                timed_out = True
            except OSError as exc:
                raise RelayError(f"NTRIP receive failed: {exc}") from exc

            if data:
                if gga_reader.get_latest() is None:
                    raise RelayError("GNSS fix expired")
                first_packet = last_rtcm_at is None
                write_all(serial_port, data)
                total_bytes += len(data)
                last_rtcm_at = time.monotonic()
                if first_packet:
                    LOG.info("RTCM received: %d bytes", len(data))
                    if on_data:
                        on_data()
            elif not timed_out:
                raise RelayError("NTRIP server closed the RTCM stream")

            if now - last_report >= 10:
                updated_at = gga_reader.updated_at
                age = now - updated_at if updated_at is not None else 0.0
                LOG.debug("RTCM forwarded: %d bytes; GGA age %.1fs", total_bytes, age)
                last_report = now


def run_relay(
    args,
    username: str,
    password: str,
    on_gga: Callable[[GgaData], None] | None = None,
    on_auxiliary_nmea: Callable[[AuxiliaryNmea], None] | None = None,
    shutdown_event: threading.Event | None = None,
) -> int:
    """Keep reading through network retries; reopen the UART after serial faults."""

    network_backoff = RetryBackoff()
    serial_backoff = RetryBackoff()

    def recovered() -> None:
        network_backoff.reset()
        serial_backoff.reset()

    def wait_backoff(delay: float) -> None:
        if shutdown_event is None:
            time.sleep(delay)
        else:
            shutdown_event.wait(delay)

    while shutdown_event is None or not shutdown_event.is_set():
        try:
            with serial.Serial(
                args.serial,
                baudrate=args.baud,
                timeout=0.1,
                write_timeout=2,
                rtscts=False,
                dsrdtr=False,
            ) as serial_port:
                LOG.info("Serial open: %s @ %d baud", args.serial, args.baud)
                with GgaReader(
                    serial_port,
                    args.gga_timeout,
                    on_gga=on_gga,
                    on_auxiliary_nmea=on_auxiliary_nmea,
                    shutdown_event=shutdown_event,
                ) as gga_reader:
                    while not gga_reader.should_stop():
                        try:
                            relay_session(
                                args,
                                serial_port,
                                username,
                                password,
                                gga_reader,
                                recovered,
                            )
                        except (SerialLinkError, NtripConfigurationError):
                            raise
                        except (OSError, RelayError) as exc:
                            delay = network_backoff.next_delay()
                            LOG.warning("NTRIP failed: %s; retrying in %.0fs", exc, delay)
                            wait_backoff(delay)
        except NtripConfigurationError as exc:
            LOG.error("%s", exc)
            return 2
        except (SerialLinkError, serial.SerialException, OSError) as exc:
            delay = serial_backoff.next_delay()
            LOG.warning(
                "Serial error: %s; reopening %s in %.0fs", exc, args.serial, delay
            )
            wait_backoff(delay)
    return 0


def parse_args(argv=None):
    env_parser = argparse.ArgumentParser(add_help=False)
    env_parser.add_argument(
        "--env-file",
        type=Path,
        default=DEFAULT_ENV_FILE,
        help="配置文件路径，默认读取脚本所在目录的 .env",
    )
    env_args, _ = env_parser.parse_known_args(argv)
    load_dotenv(env_args.env_file, override=False)

    parser = argparse.ArgumentParser(
        description="通过串口将 NTRIP RTCM 差分数据转发到 GNSS 接收机。",
        parents=[env_parser],
    )
    serial_device = os.getenv("SERIAL_DEVICE") or None
    parser.add_argument(
        "--serial",
        default=serial_device,
        required=serial_device is None,
        help="接收机串口，也可用 SERIAL_DEVICE 提供",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=os.getenv("SERIAL_BAUD", "115200"),
        help="接收机串口波特率，也可用 SERIAL_BAUD；默认 115200",
    )
    host = os.getenv("NTRIP_HOST") or None
    parser.add_argument(
        "--host",
        default=host,
        required=host is None,
        help="NTRIP 服务域名或 IP，也可用 NTRIP_HOST 提供",
    )
    parser.add_argument(
        "--caster-port",
        type=int,
        default=os.getenv("NTRIP_PORT", "2101"),
        help="NTRIP TCP 端口，也可用 NTRIP_PORT；默认 2101",
    )
    mount = os.getenv("NTRIP_MOUNT") or None
    parser.add_argument(
        "--mount",
        default=mount,
        required=mount is None,
        help="CORS 服务商提供的 RTCM 挂载点，也可用 NTRIP_MOUNT 提供",
    )
    parser.add_argument(
        "--username",
        default=os.getenv("NTRIP_USER"),
        help="NTRIP 账号，也可用 NTRIP_USER",
    )
    parser.add_argument(
        "--gga-interval", type=float, default=5.0, help="向服务端发送 GGA 的间隔秒数"
    )
    parser.add_argument(
        "--gga-timeout",
        type=float,
        default=15.0,
        help="动态 GGA 超时时间；超时会断开并停止使用旧位置",
    )
    parser.add_argument(
        "--connect-timeout", type=float, default=10.0, help="NTRIP 连接超时秒数"
    )
    parser.add_argument(
        "--rtcm-timeout",
        type=float,
        default=15.0,
        help="等待首包或后续 RTCM 的最长秒数；超时自动重连",
    )
    parser.add_argument(
        "--verbose", action="store_true", help="显示调试日志和串口原始 GGA"
    )
    args, ros_args = parser.parse_known_args(argv)
    if not 1 <= args.caster_port <= 65535:
        parser.error("NTRIP_PORT / --caster-port 必须在 1 到 65535 之间")
    args.mount = args.mount.lstrip("/")
    for name, value in (
        ("NTRIP_HOST / --host", args.host),
        ("NTRIP_MOUNT / --mount", args.mount),
    ):
        if re.fullmatch(r"[!-~]+", value) is None:
            parser.error(f"{name} 必须为非空可打印 ASCII 字符串，且不能包含空白")
    if args.baud <= 0:
        parser.error("SERIAL_BAUD / --baud 必须大于 0")
    args.ros_args = ros_args
    return args


def main(argv=None) -> int:
    args = parse_args(argv)
    configure_logging(args.verbose)
    LOG.debug("Relay script: %s", Path(__file__).resolve())
    if any(
        not math.isfinite(value) or value <= 0
        for value in (
            args.gga_interval,
            args.gga_timeout,
            args.connect_timeout,
            args.rtcm_timeout,
        )
    ):
        LOG.error("Timeouts and intervals must be finite and greater than zero")
        return 2

    username = args.username or input("NTRIP 账号: ").strip()
    password = os.getenv("NTRIP_PASSWORD") or getpass.getpass("NTRIP 密码: ")
    if not username or not password:
        LOG.error("NTRIP username and password are required")
        return 2

    try:
        pvt_publisher = Ros1PvtPublisher(args.ros_args)
    except (ImportError, RuntimeError) as exc:
        LOG.error("Failed to start ROS 1 GNSS publisher: %s", exc)
        return 2

    try:
        return run_relay(
            args,
            username,
            password,
            on_gga=pvt_publisher.publish,
            on_auxiliary_nmea=pvt_publisher.update_auxiliary,
            shutdown_event=pvt_publisher.stop_event,
        )
    except KeyboardInterrupt:
        LOG.info("Stopped")
        return 0
    finally:
        pvt_publisher.close()


if __name__ == "__main__":
    raise SystemExit(main())
