from __future__ import annotations

import contextlib
import io
import os
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import ntrip_relay as relay


class EnvConfigTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.config = Path(self.directory.name) / ".env"
        self.environment = patch.dict(os.environ, {}, clear=True)
        self.environment.start()
        self.addCleanup(self.environment.stop)

    def parse(self, text, *arguments):
        self.config.write_text(text)
        return relay.parse_args(["--env-file", str(self.config), *arguments])

    def test_file_supplies_serial_service_and_quoted_credentials(self):
        args = self.parse(
            "SERIAL_DEVICE=/dev/test-usb\nSERIAL_BAUD=57600\n"
            "NTRIP_HOST=example.test\nNTRIP_PORT=9000\n"
            "NTRIP_MOUNT=NETWORK_RTCM\nNTRIP_USER=test-user\n"
            "NTRIP_PASSWORD='password with # spaces'\n"
        )
        self.assertEqual(args.serial, "/dev/test-usb")
        self.assertEqual(args.baud, 57600)
        self.assertEqual(args.host, "example.test")
        self.assertEqual(args.caster_port, 9000)
        self.assertEqual(args.mount, "NETWORK_RTCM")
        self.assertEqual(args.username, "test-user")
        self.assertEqual(os.environ["NTRIP_PASSWORD"], "password with # spaces")

    def test_command_line_overrides_file_and_environment_overrides_file(self):
        os.environ["NTRIP_USER"] = "environment-user"
        os.environ["NTRIP_PASSWORD"] = "environment-password"
        args = self.parse(
            "SERIAL_DEVICE=/dev/file\nSERIAL_BAUD=57600\n"
            "NTRIP_HOST=file.test\nNTRIP_PORT=9000\n"
            "NTRIP_MOUNT=FILE_STREAM\nNTRIP_USER=file-user\n"
            "NTRIP_PASSWORD=file-password\n",
            "--serial",
            "/dev/cli",
            "--baud",
            "115200",
            "--host",
            "cli.test",
            "--caster-port",
            "2201",
            "--mount",
            "CLI_STREAM",
        )
        self.assertEqual(args.serial, "/dev/cli")
        self.assertEqual(args.baud, 115200)
        self.assertEqual(args.host, "cli.test")
        self.assertEqual(args.caster_port, 2201)
        self.assertEqual(args.mount, "CLI_STREAM")
        self.assertEqual(args.username, "environment-user")
        self.assertEqual(os.environ["NTRIP_PASSWORD"], "environment-password")

    def test_default_file_is_loaded_without_command_line_options(self):
        self.config.write_text(
            "SERIAL_DEVICE=/dev/default\nNTRIP_HOST=example.test\nNTRIP_MOUNT=TEST\n"
        )
        with patch.object(relay, "DEFAULT_ENV_FILE", self.config):
            self.assertEqual(relay.parse_args([]).serial, "/dev/default")

    def test_missing_file_preserves_explicit_cli_usage(self):
        args = relay.parse_args(
            [
                "--env-file",
                str(self.config),
                "--serial",
                "/dev/cli",
                "--host",
                "example.test",
                "--mount",
                "TEST",
            ]
        )
        self.assertEqual(args.serial, "/dev/cli")
        self.assertEqual(args.baud, 115200)
        self.assertEqual(args.caster_port, 2101)

    def test_all_valid_tcp_ports_are_supported(self):
        for port in (1, 2101, 9000, 65535):
            with self.subTest(port=port), patch.dict(os.environ, {}, clear=True):
                args = self.parse(
                    "SERIAL_DEVICE=/dev/test\nNTRIP_HOST=example.test\n"
                    "NTRIP_MOUNT=ANY_STREAM\nNTRIP_PORT=" + str(port) + "\n"
                )
                self.assertEqual(args.caster_port, port)

    def test_leading_slash_is_removed_from_mount(self):
        args = self.parse(
            "SERIAL_DEVICE=/dev/test\nNTRIP_HOST=example.test\n"
            "NTRIP_MOUNT=/NETWORK_RTCM\n"
        )
        self.assertEqual(args.mount, "NETWORK_RTCM")

    def test_invalid_file_values_fail_before_opening_serial(self):
        for value in (
            "SERIAL_BAUD=invalid",
            "SERIAL_BAUD=0",
            "NTRIP_PORT=invalid",
            "NTRIP_PORT=0",
            "NTRIP_PORT=65536",
            "NTRIP_MOUNT='bad mount'",
            "NTRIP_MOUNT=/",
            "NTRIP_HOST='bad host'",
        ):
            with self.subTest(value=value), patch.dict(os.environ, {}, clear=True):
                with contextlib.redirect_stderr(io.StringIO()):
                    with self.assertRaises(SystemExit) as raised:
                        self.parse(
                            "SERIAL_DEVICE=/dev/test\nNTRIP_HOST=example.test\n"
                            "NTRIP_MOUNT=TEST\n" + value + "\n"
                        )
                self.assertEqual(raised.exception.code, 2)

    def test_serial_host_and_mount_must_be_configured(self):
        values = {
            "SERIAL_DEVICE": "/dev/test",
            "NTRIP_HOST": "example.test",
            "NTRIP_MOUNT": "TEST",
        }
        for missing in values:
            with self.subTest(missing=missing), patch.dict(os.environ, {}, clear=True):
                text = "\n".join(
                    key + "=" + value for key, value in values.items() if key != missing
                )
                error = io.StringIO()
                with contextlib.redirect_stderr(error):
                    with self.assertRaises(SystemExit) as raised:
                        self.parse(text)
                self.assertEqual(raised.exception.code, 2)
                self.assertIn(
                    {
                        "SERIAL_DEVICE": "--serial",
                        "NTRIP_HOST": "--host",
                        "NTRIP_MOUNT": "--mount",
                    }[missing],
                    error.getvalue(),
                )

    def test_main_uses_file_credentials_without_interactive_prompts(self):
        self.config.write_text(
            "SERIAL_DEVICE=/dev/test\nNTRIP_HOST=example.test\nNTRIP_MOUNT=TEST\n"
            "NTRIP_USER=test-user\nNTRIP_PASSWORD=test-password\n"
        )
        with (
            patch.object(relay, "run_relay", return_value=0) as run,
            patch.object(relay.logging, "basicConfig"),
            patch("builtins.input") as prompt,
            patch.object(relay.getpass, "getpass") as password_prompt,
        ):
            self.assertEqual(relay.main(["--env-file", str(self.config)]), 0)
        self.assertEqual(run.call_args.args[1:], ("test-user", "test-password"))
        prompt.assert_not_called()
        password_prompt.assert_not_called()


if __name__ == "__main__":
    unittest.main()
