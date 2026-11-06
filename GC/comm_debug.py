"""MaixPy通信单测：改下面四项，SSH运行 python3 -u comm_debug.py。"""
import importlib.util
import time
from pathlib import Path

from rescue_comm import Framer, STOP, RESULT, encode, describe, finite

# 修改后Ctrl+C停掉旧程序，再重新运行；DATA是字节列表，自动添加AA55/LEN/CRC。
SEND_CMD = 0x01
SEND_DATA = [0x01]
SEND_SEQ = 1                  # 重复发同一帧；新动作可改SEQ，不会自动反复执行新动作。
SEND_INTERVAL_S = 0.1         # 每100ms发一次。


def load_cfg():
    path = Path(__file__).with_name("GC-01-config.py")
    spec = importlib.util.spec_from_file_location("gc01_config", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.CFG


def send_frame():
    if not finite(SEND_INTERVAL_S) or SEND_INTERVAL_S <= 0:
        raise ValueError("SEND_INTERVAL_S must be finite and > 0")
    if isinstance(SEND_DATA, int):
        raise ValueError("SEND_DATA must be a byte list, for example [0x01]")
    return encode(SEND_CMD, SEND_SEQ, bytes(SEND_DATA))


def run(serial, cfg, raw):
    framer = Framer(cfg)
    started = time.monotonic()
    next_send, next_log = started, started + 1.0
    received, packets, tx_count = bytearray(), [], 0
    last_errors = 0
    try:
        print("TX CMD=0x%02X DATA=%s every %.3fs SEQ=%d HEX=%s" %
              (raw[2], raw[5:-2].hex(" ").upper(), SEND_INTERVAL_S, raw[3], raw.hex(" ").upper()), flush=True)
        print("每秒刷新接收日志；没有反馈也继续发送；Ctrl+C退出。", flush=True)
        while True:
            now = time.monotonic()
            if now >= next_send:
                if serial.write(raw) != len(raw):
                    raise OSError("UART_SHORT_WRITE")
                tx_count += 1
                next_send = now + SEND_INTERVAL_S
            data = serial.read(len=256, timeout=0) or b""
            received.extend(data)
            packets.extend(framer.feed(data, now))
            if now >= next_log:
                print("[+%.1fs] TX=%d RX=%dB FRAMES=%d ERRORS=%d" %
                      (now-started, tx_count, len(received), len(packets), framer.errors-last_errors), flush=True)
                print("RX HEX: " + (received.hex(" ").upper() if received else "-- 本秒未收到数据"), flush=True)
                for packet in packets:
                    message = ("0x81 " if packet.cmd == RESULT else "") + describe(packet)
                    print("RX %s SEQ=%d DATA=%s" %
                          (message, packet.seq, packet.data.hex(" ").upper()), flush=True)
                received.clear()
                packets.clear()
                tx_count, last_errors, next_log = 0, framer.errors, now + 1.0
            time.sleep(0.005)
    except KeyboardInterrupt:
        print("停止通信调试。", flush=True)
    finally:
        try:
            stop = encode(STOP, (SEND_SEQ+1) & 255, b"\x01")
            if serial.write(stop) != len(stop):
                raise OSError("UART_SHORT_WRITE")
            print("退出：已发送0x01 1停车请求。", flush=True)
        except Exception as exc:
            print("退出停车请求发送失败：" + str(exc), flush=True)
        finally:
            serial.close()


def main():
    raw, cfg = send_frame(), load_cfg()
    from maix import pinmap, uart
    for pin, function in cfg["uart"]["pins"]:
        if pinmap.set_pin_function(pin, function) != 0:
            raise RuntimeError("PINMAP:" + pin)
    serial = uart.UART(cfg["uart"]["device"], cfg["uart"]["baud"])
    print("UART %s %d baud" % (cfg["uart"]["device"], cfg["uart"]["baud"]), flush=True)
    run(serial, cfg, raw)


if __name__ == "__main__":
    main()
