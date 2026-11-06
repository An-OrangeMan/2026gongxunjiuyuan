"""通信调试器的离线回归；使用假时钟、假串口，不访问硬件。"""
import sys
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import comm_debug as debug
from rescue_comm import encode, STOP, TURN, RESULT


class Clock:
    def __init__(self, until_ms=2200):
        self.ms, self.until_ms = 0, until_ms

    def monotonic(self):
        return self.ms / 1000

    def sleep(self, seconds):
        self.ms += round(seconds * 1000)
        if self.ms >= self.until_ms:
            raise KeyboardInterrupt


class Serial:
    def __init__(self, clock, events=(), stop_error=False):
        self.clock, self.events = clock, list(events)
        self.stop_error = stop_error
        self.writes, self.reads, self.closed = [], [], False

    def write(self, raw):
        self.writes.append((self.clock.monotonic(), raw))
        if self.stop_error and raw == encode(STOP, (debug.SEND_SEQ + 1) & 255, b"\x01"):
            raise OSError("stop write failed")
        return len(raw)

    def read(self, **kwargs):
        self.reads.append(kwargs)
        data = bytearray()
        while self.events and self.events[0][0] <= self.clock.ms:
            data.extend(self.events.pop(0)[1])
        return bytes(data)

    def close(self):
        self.closed = True


class CommDebugTests(unittest.TestCase):
    def setUp(self):
        settings = patch.multiple(debug, SEND_CMD=STOP, SEND_DATA=[1], SEND_SEQ=1, SEND_INTERVAL_S=.1)
        settings.start()
        self.addCleanup(settings.stop)
        self.cfg = debug.load_cfg()

    def run_debug(self, events=(), until_ms=2200, stop_error=False):
        clock = Clock(until_ms)
        serial = Serial(clock, events, stop_error)
        raw = debug.send_frame()
        with patch.object(debug, "time", clock), patch("builtins.print") as output:
            debug.run(serial, self.cfg, raw)
        self.assertTrue(all(call.kwargs.get("flush") is True for call in output.call_args_list))
        return serial, [str(call.args[0]) for call in output.call_args_list]

    def test_no_ack_keeps_sending_same_frame_at_ten_hz_and_logs_empty_rx(self):
        serial, lines = self.run_debug()
        raw = encode(STOP, 1, b"\x01")
        times = [when for when, data in serial.writes if data == raw]
        self.assertEqual(times[0], 0)
        self.assertGreaterEqual(len(times), 21)
        self.assertLessEqual(len(times), 23)
        for previous, current in zip(times, times[1:]):
            self.assertGreaterEqual(current - previous, .1 - 1e-9)
            self.assertLessEqual(current - previous, .105 + 1e-9)
        summaries = [line for line in lines if line.startswith("[+")]
        self.assertEqual(len(summaries), 2)
        self.assertTrue(all("RX=0B FRAMES=0 ERRORS=0" in line for line in summaries))
        self.assertEqual(sum("本秒未收到数据" in line for line in lines), 2)
        self.assertEqual(serial.writes[-1][1], encode(STOP, 2, b"\x01"))
        self.assertTrue(serial.closed)
        self.assertTrue(all(call == {"len": 256, "timeout": 0} for call in serial.reads))

    def test_partial_frame_survives_across_log_bucket_reset(self):
        ack = encode(RESULT, 1, bytes((STOP, 1)))
        serial, lines = self.run_debug(events=[(995, ack[:4]), (1020, ack[4:])], until_ms=2100)
        summaries = [line for line in lines if line.startswith("[+")]
        self.assertIn("RX=4B FRAMES=0 ERRORS=0", summaries[0])
        self.assertIn("RX=5B FRAMES=1 ERRORS=0", summaries[1])
        self.assertIn("RX HEX: " + ack[:4].hex(" ").upper(), lines)
        self.assertIn("RX HEX: " + ack[4:].hex(" ").upper(), lines)
        self.assertIn("RX 0x81 0x01 1 SEQ=1 DATA=01 01", lines)
        self.assertTrue(serial.closed)

    def test_noise_bad_crc_and_two_valid_frames_are_reported_without_accumulating(self):
        ack = encode(RESULT, 7, bytes((STOP, 1)))
        bad = ack[:-1] + bytes((ack[-1] ^ 1,))
        busy = encode(RESULT, 8, bytes((TURN, 0)))
        received = b"noise" + bad + ack + busy
        serial, lines = self.run_debug(events=[(20, received)], until_ms=2100)
        summaries = [line for line in lines if line.startswith("[+")]
        self.assertIn("RX=%dB FRAMES=2 ERRORS=1" % len(received), summaries[0])
        self.assertIn("RX=0B FRAMES=0 ERRORS=0", summaries[1])
        self.assertIn("RX HEX: " + received.hex(" ").upper(), lines)
        self.assertIn("RX 0x81 0x01 1 SEQ=7 DATA=01 01", lines)
        self.assertIn("RX 0x81 0x03 0 SEQ=8 DATA=03 00", lines)
        self.assertTrue(serial.closed)

    def test_expired_partial_frame_reports_error_then_accepts_next_frame(self):
        ack = encode(RESULT, 1, bytes((STOP, 1)))
        _, lines = self.run_debug(events=[(10, ack[:4]), (200, ack[4:]), (300, ack)], until_ms=1100)
        summary = next(line for line in lines if line.startswith("[+"))
        self.assertIn("FRAMES=1 ERRORS=1", summary)
        self.assertEqual(sum(line == "RX 0x81 0x01 1 SEQ=1 DATA=01 01" for line in lines), 1)

    def test_invalid_settings_are_rejected_before_uart_or_pinmap_is_used(self):
        constructor, set_pin = Mock(), Mock(return_value=0)
        maix = ModuleType("maix")
        maix.uart = SimpleNamespace(UART=constructor)
        maix.pinmap = SimpleNamespace(set_pin_function=set_pin)
        invalid = ({"SEND_CMD": 0}, {"SEND_SEQ": 256}, {"SEND_DATA": []},
                   {"SEND_DATA": [256]}, {"SEND_DATA": 1}, {"SEND_INTERVAL_S": 0},
                   {"SEND_INTERVAL_S": -1}, {"SEND_INTERVAL_S": float("nan")},
                   {"SEND_INTERVAL_S": float("inf")})
        for settings in invalid:
            with self.subTest(settings=settings), patch.multiple(debug, **settings), \
                    patch.dict(sys.modules, {"maix": maix}), patch.object(debug, "load_cfg") as load_cfg:
                with self.assertRaises(ValueError):
                    debug.main()
                constructor.assert_not_called()
                set_pin.assert_not_called()
                load_cfg.assert_not_called()

    def test_main_opens_configured_uart_and_passes_valid_encoded_frame_to_run(self):
        serial = object()
        constructor, set_pin = Mock(return_value=serial), Mock(return_value=0)
        maix = ModuleType("maix")
        maix.uart = SimpleNamespace(UART=constructor)
        maix.pinmap = SimpleNamespace(set_pin_function=set_pin)
        with patch.dict(sys.modules, {"maix": maix}), patch.object(debug, "run") as run, \
                patch("builtins.print"):
            debug.main()
        constructor.assert_called_once_with(self.cfg["uart"]["device"], self.cfg["uart"]["baud"])
        self.assertEqual(set_pin.call_count, len(self.cfg["uart"]["pins"]))
        run.assert_called_once_with(serial, self.cfg, encode(STOP, 1, b"\x01"))

    def test_keyboard_interrupt_closes_uart_even_if_stop_write_fails(self):
        with patch.object(debug, "SEND_SEQ", 255):
            serial, lines = self.run_debug(until_ms=20, stop_error=True)
        self.assertEqual(serial.writes[0][1], encode(STOP, 255, b"\x01"))
        self.assertEqual(serial.writes[-1][1], encode(STOP, 0, b"\x01"))
        self.assertTrue(any("停车请求发送失败" in line and "stop write failed" in line for line in lines))
        self.assertTrue(serial.closed)


if __name__ == "__main__":
    unittest.main()
