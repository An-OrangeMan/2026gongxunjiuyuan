"""AA55 CMD SEQ LEN DATA CRC16；串口与离线测试共用，不导入maix。"""
import math
import struct
import time
from collections import deque, namedtuple

STOP, MOVE, TURN, GRIP, SPIN = 0x01, 0x02, 0x03, 0x04, 0x05
RESULT, TEAM, START = 0x81, 0x82, 0x83
LENGTHS = {STOP: 1, MOVE: 1, TURN: 2, GRIP: 1, SPIN: 1, RESULT: 2, TEAM: 1, START: 1}
Packet = namedtuple("Packet", "cmd seq data")


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def crc16(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def encode(cmd, seq, data):
    if type(cmd) is not int or cmd not in LENGTHS or type(seq) is not int or not 0 <= seq <= 255:
        raise ValueError("CMD/SEQ")
    if not isinstance(data, bytes) or len(data) != LENGTHS[cmd]:
        raise ValueError("DATA_LENGTH")
    raw = bytes((cmd, seq, len(data))) + data
    return b"\xaa\x55" + raw + struct.pack("<H", crc16(raw))


def command_data(cmd, value):
    if type(value) is not int:
        raise ValueError("INTEGER_COMMAND_REQUIRED")
    if cmd == TURN:
        if not -1800 <= value <= 1800 or not value:
            raise ValueError("TURN_RANGE")
        return struct.pack("<h", value)
    if cmd not in (STOP, MOVE, GRIP, SPIN) or value not in (0, 1) or (cmd == STOP and value != 1):
        raise ValueError("COMMAND_VALUE")
    return bytes((value,))


def describe(packet):
    if packet is None:
        return "--"
    if packet.cmd == RESULT:
        return "0x%02X %d" % tuple(packet.data)
    value = struct.unpack("<h", packet.data)[0] if packet.cmd == TURN else packet.data[0]
    return "0x%02X %d" % (packet.cmd, value)


class Framer:
    def __init__(self, cfg):
        self.cfg, self.buffer, self.errors = cfg, bytearray(), 0
        self.partial_at = None

    def feed(self, data, now):
        if self.partial_at is not None and now - self.partial_at > self.cfg["partial_timeout_s"]:
            self.buffer.clear()
            self.partial_at = None
            self.errors += 1
        if len(data) + len(self.buffer) > self.cfg["rx_max_buffer"]:
            self.buffer.clear()
            self.errors += 1
            data = data[-self.cfg["rx_max_buffer"]:]
        self.buffer.extend(data)
        packets = []
        while self.buffer:
            start = self.buffer.find(b"\xaa\x55")
            if start < 0:
                self.buffer[:] = self.buffer[-1:] if self.buffer[-1] == 0xAA else b""
                break
            if start:
                del self.buffer[:start]
            if len(self.buffer) < 5:
                break
            cmd, seq, length = self.buffer[2:5]
            if cmd not in LENGTHS or length != LENGTHS[cmd] or length > self.cfg["rx_max_data"]:
                del self.buffer[0]
                self.errors += 1
                continue
            total = length + 7
            if len(self.buffer) < total:
                break
            raw = bytes(self.buffer[:total])
            if crc16(raw[2:-2]) != struct.unpack("<H", raw[-2:])[0]:
                del self.buffer[0]
                self.errors += 1
                continue
            packets.append(Packet(cmd, seq, raw[5:-2]))
            del self.buffer[:total]
        if self.buffer and self.partial_at is None:
            self.partial_at = now
        elif not self.buffer:
            self.partial_at = None
        return packets


class Link:
    def __init__(self, cfg, send, log=lambda event: None):
        self.cfg, self.send_bytes, self.log = cfg, send, log
        self.framer, self.seq = Framer(cfg), 0
        self.pending, self.streaming, self.current_key = None, None, None
        self.events, self.replies = deque(), {}
        self.last_tx = self.last_rx = None
        self.last_feedback, self.last_stream = -1e9, -1e9
        self.error = ""
        self.cycle_encode_ms = self.cycle_write_ms = 0.0
        self.last_turn_tenths = None

    def next_seq(self):
        self.seq = (self.seq + 1) & 255
        return self.seq

    def write(self, packet, now):
        started = time.monotonic()
        raw = encode(*packet)
        self.cycle_encode_ms += (time.monotonic() - started) * 1000
        started = time.monotonic()
        written = self.send_bytes(raw)
        self.cycle_write_ms += (time.monotonic() - started) * 1000
        if written != len(raw):
            self.error = "UART_SHORT_WRITE"
            raise OSError(self.error)
        self.last_tx = packet
        if packet.cmd == TURN:
            self.last_turn_tenths = struct.unpack("<h", packet.data)[0]
        self.log({"event": "tx", "t": now, "cmd": packet.cmd, "seq": packet.seq, "data": list(packet.data)})

    def request(self, cmd, value, now):
        if cmd not in (STOP, TURN, GRIP):
            raise ValueError("DISCRETE_COMMAND")
        if self.pending:
            if cmd == STOP and self.pending["packet"].cmd == STOP:
                return self.pending
            if cmd != STOP:
                raise RuntimeError("ACTION_PENDING")
        packet = Packet(cmd, self.next_seq(), command_data(cmd, value))
        timeout = self.cfg[{STOP: "stop_timeout_s", TURN: "turn_timeout_s", GRIP: "gripper_timeout_s"}[cmd]]
        self.streaming = None
        self.pending = {"packet": packet, "at": now, "timeout": timeout, "done": False}
        self.current_key = (cmd, packet.seq)
        self.last_feedback = now
        self.write(packet, now)
        return self.pending

    def consume(self):
        if self.pending and self.pending["done"]:
            self.pending = None
            return True
        return False

    def stream(self, cmd, value, now):
        if self.pending:
            raise RuntimeError("STREAM_DURING_ACTION")
        if cmd not in (MOVE, SPIN):
            raise ValueError("STREAM_COMMAND")
        data = command_data(cmd, value)
        if self.streaming is None or (self.streaming.cmd, self.streaming.data) != (cmd, data):
            self.streaming = Packet(cmd, self.next_seq(), data)
            self.current_key = (cmd, self.streaming.seq)
            self.last_stream = -1e9
            self.last_feedback = now
        if now - self.last_stream >= 1 / self.cfg["command_hz"]:
            self.write(self.streaming, now)
            self.last_stream = now

    def receive(self, packet, now):
        self.last_rx = packet
        self.log({"event": "rx", "t": now, "cmd": packet.cmd, "seq": packet.seq, "data": list(packet.data)})
        if packet.cmd in (TEAM, START):
            key = (packet.cmd, packet.seq, packet.data)
            if key in self.replies:
                self.write(Packet(RESULT, packet.seq, bytes((packet.cmd, self.replies[key]))), now)
            elif len(self.events) < 16:
                self.events.append(packet)
            return
        if packet.cmd != RESULT or packet.data[0] not in (STOP, MOVE, TURN, GRIP, SPIN) or packet.data[1] not in (0, 1):
            self.framer.errors += 1
            return
        if (packet.data[0], packet.seq) != self.current_key:
            return
        self.last_feedback = now
        if self.pending and packet.data[1] == 1:
            self.pending["done"] = True

    def reply(self, packet, accepted, now):
        key = (packet.cmd, packet.seq, packet.data)
        if len(self.replies) >= 32:
            del self.replies[next(iter(self.replies))]
        self.replies[key] = int(bool(accepted))
        self.write(Packet(RESULT, packet.seq, bytes((packet.cmd, int(bool(accepted))))), now)

    def poll(self, data, now):
        for packet in self.framer.feed(data, now):
            self.receive(packet, now)
        if self.pending and not self.pending["done"] and now - self.pending["at"] > self.pending["timeout"]:
            self.error = "ACTION_TIMEOUT_0x%02X" % self.pending["packet"].cmd
        active = self.streaming is not None or (self.pending is not None and not self.pending["done"])
        if active and now - self.last_feedback > self.cfg["feedback_timeout_s"]:
            self.error = self.error or "FEEDBACK_LOST"
