#!/usr/bin/env python3
"""Forward NTRIP RTCM corrections to a UM960 UART through a USB-serial adapter."""

from __future__ import annotations

import argparse
import base64
import getpass
import logging
import math
import os
import re
import socket
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass
from datetime import time as utc_time

import pynmea2
import serial

LOG = logging.getLogger("ntrip_um960_relay")


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
    0: "未定位",
    1: "单点定位",
    2: "差分定位",
    3: "PPS 定位",
    4: "RTK 固定解",
    5: "RTK 浮点解",
    6: "推算定位",
    7: "人工位置",
    8: "模拟位置",
}


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

    @property
    def has_position(self) -> bool:
        return (
            self.quality in (1, 2, 3, 4, 5)
            and self.latitude is not None
            and self.longitude is not None
            and self.time_utc is not None
        )


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
        if satellites is not None and satellites < 0:
            return None
        if hdop is not None and (not math.isfinite(hdop) or hdop < 0):
            return None
        if altitude is not None and not math.isfinite(altitude):
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
        )
    except (pynmea2.ParseError, ValueError, TypeError, OverflowError):
        return None


class GgaReader:
    """Continuously retain the latest checked GGA and propagate UART faults."""

    def __init__(self, serial_port, stale_after: float):
        self.serial_port = serial_port
        self.stale_after = stale_after
        self.buffer = bytearray()
        self.latest: bytes | None = None
        self.updated_at: float | None = None
        self.last_observation: GgaData | None = None
        self.last_status_report = 0.0
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._error: SerialLinkError | None = None
        self._last_utc_seconds: float | None = None
        self._epoch_changed_at: float | None = None

    def __enter__(self):
        self._discard_backlog()
        self._thread = threading.Thread(target=self._run, name="um960-gga", daemon=True)
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
            raise SerialLinkError(f"清理串口积压数据失败：{exc}") from exc
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
            while not self._stop.is_set():
                now = time.monotonic()
                if now - last_poll > self.stale_after:
                    LOG.warning("串口读取曾暂停过久，丢弃积压数据并等待新 GGA")
                    self._discard_backlog()
                self.poll()
                last_poll = now
                self._stop.wait(0.01)
        except (OSError, serial.SerialException, SerialLinkError) as exc:
            with self._lock:
                self.latest = None
                self._error = SerialLinkError(f"UM960 串口读取失败：{exc}")

    def _record(self, gga: GgaData, now: float) -> bool:
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
            LOG.warning("GGA UTC 回退，清除旧位置并等待下一定位历元")
        if changed or now - self.last_status_report >= 5:
            self._log_position(gga)
            if gga.has_position and not usable:
                LOG.warning("GGA 定位历元未推进或已过期，暂停使用该位置")
            self.last_status_report = now
        return usable

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
            if gga is None:
                LOG.debug("忽略非 GGA 或校验/格式错误的串口报文：%r", line[:160])
                continue
            self._record(gga, time.monotonic())

        # Recover from garbage or incomplete oversized sentences on the UART.
        if len(self.buffer) > 8192:
            start = self.buffer.rfind(b"$")
            self.buffer[:] = self.buffer[start:] if start >= 0 else b""
            if len(self.buffer) > 8192:
                self.buffer.clear()

    def _log_position(self, gga: GgaData) -> None:
        LOG.debug("UM960 原始 GGA：%s", gga.sentence.decode("ascii").strip())
        if not gga.has_position:
            LOG.warning(
                "UM960 GGA：%s（quality=%d），卫星数=%s；无可用实测坐标，等待定位",
                FIX_NAMES[gga.quality],
                gga.quality,
                gga.satellites if gga.satellites is not None else "未知",
            )
            return
        LOG.info(
            "UM960 GGA：%s；纬度=%.8f，经度=%.8f，卫星数=%s，HDOP=%s，海拔=%s m，UTC=%s",
            FIX_NAMES[gga.quality],
            gga.latitude,
            gga.longitude,
            gga.satellites,
            gga.hdop,
            gga.altitude_m,
            gga.time_utc,
        )

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

    def wait_for_fix(self) -> bytes:
        last_notice = 0.0
        while True:
            gga = self.get_latest()
            if gga is not None:
                return gga
            now = time.monotonic()
            if now - last_notice >= 5:
                LOG.info("等待 UM960 的真实有效 GGA；单点定位即可请求 RTCM")
                last_notice = now
            self._stop.wait(0.1)


def _check_ntrip_status(status: bytes) -> int | None:
    if status.startswith(b"SOURCETABLE"):
        raise NtripConfigurationError("服务端返回源表，请检查挂载点名称")
    if status.startswith(b"ERROR - Bad MountPoint"):
        raise NtripConfigurationError("挂载点不存在或不可用（Bad MountPoint）")
    match = re.match(rb"(?:HTTP/\d\.\d|ICY)\s+(\d{3})(?:\s|$)", status)
    if match is None:
        return None
    code = int(match.group(1))
    if code in (400, 401, 403, 404, 410):
        details = {
            400: "请求配置不正确",
            401: "账号或密码错误，也可能已过期",
            403: "账号没有该服务的访问权限",
            404: "挂载点不存在",
            410: "挂载点已停用",
        }
        raise NtripConfigurationError(f"NTRIP {code}：{details[code]}")
    if code != 200:
        raise RelayError(f"NTRIP 服务响应 {code}")
    return code


def _read_ntrip_response(
    sock: socket.socket, limit: int = 16384
) -> tuple[bytes, bytes]:
    response = bytearray()
    timeout = sock.gettimeout()
    deadline = time.monotonic() + (timeout if timeout is not None else 10.0)
    try:
        while True:
            # This caster also returns short errors without a complete HTTP header.
            status = bytes(response).split(b"\r\n", 1)[0]
            code = _check_ntrip_status(status)
            if b"\r\n\r\n" in response:
                if code != 200:
                    raise RelayError("无法识别 NTRIP 响应状态")
                return tuple(bytes(response).split(b"\r\n\r\n", 1))
            if len(response) >= limit:
                raise RelayError("NTRIP 响应头超过限制")
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise RelayError("等待 NTRIP 响应头超时")
            sock.settimeout(remaining)
            try:
                chunk = sock.recv(min(4096, limit - len(response)))
            except socket.timeout as exc:
                raise RelayError("等待 NTRIP 响应头超时") from exc
            if not chunk:
                raise RelayError("NTRIP 服务在返回响应头前关闭了连接")
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
        "User-Agent: NTRIP UM960Relay/1.0\r\n"
        "Ntrip-Version: Ntrip/2.0\r\n"
        "Accept: */*\r\n"
        f"Authorization: Basic {token}\r\n"
        "Connection: keep-alive\r\n\r\n"
    ).encode("ascii")
    try:
        sock.sendall(request)
        _, remainder = _read_ntrip_response(sock)
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
            raise SerialLinkError(f"UM960 串口写入失败：{exc}") from exc
        if not count:
            raise SerialLinkError("UM960 串口写入超时")
        offset += count


def relay_session(
    args,
    serial_port,
    username: str,
    password: str,
    gga_reader: GgaReader,
    on_data: Callable[[], None] | None = None,
) -> None:
    gga_reader.wait_for_fix()
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
            raise RelayError("UM960 无有效实时位置，暂停转发并等待新定位")
        sock.sendall(gga)
        last_gga_sent = time.monotonic()
        rtcm_wait_started_at = last_gga_sent
        last_report = last_gga_sent
        last_rtcm_at: float | None = None
        total_bytes = 0
        recovered = False
        LOG.info(
            "NTRIP 已连接：%s:%d/%s；等待 RTCM 数据",
            args.host,
            args.caster_port,
            args.mount,
        )

        if pending:
            write_all(serial_port, pending)
            total_bytes += len(pending)
            last_rtcm_at = time.monotonic()
            if on_data:
                on_data()
            recovered = True

        while True:
            now = time.monotonic()
            gga = gga_reader.get_latest()
            if gga is None:
                raise RelayError("UM960 未定位或 GGA 已超时，暂停转发并等待新定位")
            since = last_rtcm_at if last_rtcm_at is not None else rtcm_wait_started_at
            if now - since >= args.rtcm_timeout:
                detail = (
                    "一直未收到首包 RTCM"
                    if last_rtcm_at is None
                    else "RTCM 数据流已停止"
                )
                raise RelayError(f"{detail}，超过 {args.rtcm_timeout:g} 秒")
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
                raise RelayError(f"NTRIP 接收失败：{exc}") from exc

            if data:
                if gga_reader.get_latest() is None:
                    raise RelayError("UM960 未定位或 GGA 已超时，暂停转发并等待新定位")
                write_all(serial_port, data)
                total_bytes += len(data)
                last_rtcm_at = time.monotonic()
                if not recovered:
                    if on_data:
                        on_data()
                    recovered = True
            elif not timed_out:
                raise RelayError("NTRIP 服务端关闭了 RTCM 数据流")

            if now - last_report >= 10:
                age = now - (gga_reader.updated_at or now)
                LOG.info("已转发 %d 字节 RTCM；GGA 年龄 %.1f 秒", total_bytes, age)
                last_report = now


def run_relay(args, username: str, password: str) -> int:
    """Keep reading through network retries; reopen the UART after serial faults."""

    network_backoff = RetryBackoff()
    serial_backoff = RetryBackoff()

    def recovered() -> None:
        network_backoff.reset()
        serial_backoff.reset()

    while True:
        try:
            with serial.Serial(
                args.serial,
                baudrate=args.baud,
                timeout=0.1,
                write_timeout=2,
                rtscts=False,
                dsrdtr=False,
            ) as serial_port:
                LOG.info("串口已打开：%s @ %d baud", args.serial, args.baud)
                LOG.info("GGA 来源：UM960 串口实时 NMEA 输出")
                with GgaReader(serial_port, args.gga_timeout) as gga_reader:
                    while True:
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
                            LOG.warning("NTRIP 中断：%s；%.0f 秒后重试", exc, delay)
                            time.sleep(delay)
        except NtripConfigurationError as exc:
            LOG.error("%s；请修正配置后重新运行", exc)
            return 2
        except (SerialLinkError, serial.SerialException, OSError) as exc:
            delay = serial_backoff.next_delay()
            LOG.warning("串口故障：%s；%.0f 秒后重新打开 %s", exc, delay, args.serial)
            time.sleep(delay)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="通过 CH340 USB-TTL 将 NTRIP RTCM 数据转发到 UM960。"
    )
    parser.add_argument(
        "--serial", required=True, help="CH340 串口，如 /dev/ttyUSB0 或 COM3"
    )
    parser.add_argument(
        "--baud", type=int, default=115200, help="UM960 串口波特率，默认 115200"
    )
    parser.add_argument("--host", default="rtkcq.cn", help="NTRIP 服务域名")
    parser.add_argument(
        "--caster-port",
        type=int,
        default=8002,
        choices=(8001, 8002, 8003),
        help="8001 ITRF2008；8002 WGS84；8003 CGCS2000",
    )
    parser.add_argument(
        "--mount",
        default="RTCM33_GRCEJ",
        choices=("RTCM33_GRC", "RTCM33_GRCEJ"),
        help="RTCM 挂载点",
    )
    parser.add_argument(
        "--username",
        default=os.getenv("NTRIP_USER"),
        help="NTRIP 卡号，也可用 NTRIP_USER",
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
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
        datefmt="%H:%M:%S",
    )
    if any(
        not math.isfinite(value) or value <= 0
        for value in (
            args.gga_interval,
            args.gga_timeout,
            args.connect_timeout,
            args.rtcm_timeout,
        )
    ):
        logging.error("时间参数必须为有限且大于 0 的数值")
        return 2

    username = args.username or input("NTRIP 卡号: ").strip()
    password = os.getenv("NTRIP_PASSWORD") or getpass.getpass("NTRIP 密码: ")
    if not username or not password:
        logging.error("NTRIP 卡号和密码不能为空")
        return 2

    try:
        return run_relay(args, username, password)
    except KeyboardInterrupt:
        LOG.info("已停止")
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
