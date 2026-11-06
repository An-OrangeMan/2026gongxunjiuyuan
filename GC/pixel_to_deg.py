"""MaixCAM Pro pixel_to_deg calibration; default is preview-only."""
import argparse
import importlib.util
import struct
import time
from pathlib import Path

from rescue_comm import Link, STOP, TURN, RESULT, encode

SAMPLE_COUNT = 7
STABLE_SPREAD_PX = 4
MIN_SHIFT_PX = 3
PHASE_TIMEOUT_S = 20.0


def load_cfg():
    path = Path(__file__).with_name("GC-01-config.py")
    spec = importlib.util.spec_from_file_location("gc01_config", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.CFG


def target_center(objects, labels, label):
    found = [o for o in objects if labels[o.class_id] == label]
    if len(found) != 1:
        return None, len(found)
    o = found[0]
    return (o.x + o.w / 2, o.y + o.h / 2), 1


def stable_center(samples, point):
    if point is None:
        samples.clear()
        return None
    samples.append(point)
    del samples[:-SAMPLE_COUNT]
    if len(samples) < SAMPLE_COUNT:
        return None
    xs, ys = [p[0] for p in samples], [p[1] for p in samples]
    if max(xs) - min(xs) > STABLE_SPREAD_PX or max(ys) - min(ys) > STABLE_SPREAD_PX:
        return None
    return median(xs), median(ys)


def median(values):
    ordered = sorted(values)
    middle = len(ordered) // 2
    return ordered[middle] if len(ordered) % 2 else (ordered[middle - 1] + ordered[middle]) / 2


def measure_pixel_to_deg(angle10, before, after):
    if type(angle10) is not int or not -1800 <= angle10 <= 1800 or angle10 == 0:
        raise ValueError("angle10 must be a non-zero int16 in 0.1-degree units")
    dx = after[0] - before[0]
    if abs(dx) < MIN_SHIFT_PX:
        raise ValueError("target moved less than %d pixels; use a larger test angle or clearer target" % MIN_SHIFT_PX)
    k = abs(angle10) / 10.0 / abs(dx)
    suggested_turn_sign = -1 if angle10 * dx > 0 else 1
    return k, dx, suggested_turn_sign


def print_comm(event):
    data = event["data"]
    if event["event"] == "tx":
        cmd = event["cmd"]
        if cmd == TURN:
            value = struct.unpack("<h", bytes(data))[0]
        elif cmd == RESULT:
            print("TX 0x81 0x%02X %d" % (data[0], data[1]), flush=True)
            return
        else:
            value = data[0]
        print("TX 0x%02X %d" % (cmd, value), flush=True)
    elif event["event"] == "rx":
        if event["cmd"] == RESULT:
            print("RX 0x81 0x%02X %d" % (data[0], data[1]), flush=True)
        elif event["cmd"] == TURN:
            print("RX 0x03 %d" % struct.unpack("<h", bytes(data))[0], flush=True)
        else:
            print("RX 0x%02X %d" % (event["cmd"], data[0]), flush=True)


def self_test():
    k, dx, turn_sign = measure_pixel_to_deg(50, (100, 80), (120, 80))
    assert k == 0.25 and dx == 20 and turn_sign == -1
    assert measure_pixel_to_deg(-50, (100, 80), (80, 80)) == (0.25, -20, -1)
    raw = encode(TURN, 1, struct.pack("<h", 50))
    assert raw[:5] == bytes((0xAA, 0x55, 0x03, 0x01, 0x02)) and raw[5:7] == b"\x32\x00"
    print("SELF_TEST PASS")


def run(args):
    cfg = load_cfg()
    if args.label not in cfg["labels"]:
        raise ValueError("unknown label %s; choose one of %s" % (args.label, ", ".join(cfg["labels"])))
    if not -1800 <= args.angle10 <= 1800 or args.angle10 == 0:
        raise ValueError("--angle10 must be non-zero and within -1800..1800")

    from maix import app, camera, display, image, nn, pinmap, uart

    detector = nn.YOLOv8(model=cfg["model"], dual_buff=False)
    if tuple(detector.labels) != tuple(cfg["labels"]):
        raise RuntimeError("YOLO_LABELS_MISMATCH")
    cam = disp = serial = None
    link = None
    if args.send:
        for pin, function in cfg["uart"]["pins"]:
            if pinmap.set_pin_function(pin, function) != 0:
                raise RuntimeError("PINMAP:" + pin)
        serial = uart.UART(cfg["uart"]["device"], cfg["uart"]["baud"])
        link = Link(cfg, serial.write, print_comm)

    cam = camera.Camera(*cfg["camera_size"], detector.input_format(),
                        fps=cfg["camera_fps"], buff_num=3)
    disp = display.Display()
    phase = "STOP_SEND" if args.send else "PREVIEW"
    phase_at = time.monotonic()
    samples, before, after = [], None, None
    result_text, error = "", ""

    try:
        print("标定目标=%s；" % args.label +
              ("待发送0x03 %d (%.1f度)" % (args.angle10, args.angle10 / 10.0) if args.send else
               "预览模式，不发送串口命令；加 --send 才执行转角"), flush=True)
        while not app.need_exit():
            now = time.monotonic()
            if serial:
                link.poll(serial.read(len=256, timeout=0) or b"", now)
                while link.events:
                    link.reply(link.events.popleft(), False, now)

            if link and link.error and phase not in ("DONE", "ERROR"):
                error, phase = link.error, "ERROR"
                pending = link.pending
                if pending and pending["packet"].cmd != STOP:
                    try:
                        link.request(STOP, 1, now)
                    except Exception:
                        pass

            img = cam.read()
            if img is None:
                continue
            objects = detector.detect(img, conf_th=cfg["confidence"], iou_th=0.45)
            point, count = target_center(objects, detector.labels, args.label)

            if phase == "STOP_SEND":
                link.request(STOP, 1, now)
                phase, phase_at = "STOP_WAIT", now
            elif phase == "STOP_WAIT" and link.pending and link.pending["done"]:
                link.consume()
                phase, phase_at = "BEFORE", now
                samples.clear()
            elif phase == "BEFORE":
                stable = stable_center(samples, point)
                if stable:
                    before = stable
                    if args.send:
                        link.request(TURN, args.angle10, now)
                        phase, phase_at = "TURN_WAIT", now
                        samples.clear()
                    else:
                        phase = "PREVIEW"
                elif now - phase_at > PHASE_TIMEOUT_S:
                    error, phase = "TARGET_NOT_STABLE_BEFORE_TURN", "ERROR"
            elif phase == "TURN_WAIT" and link.pending and link.pending["done"]:
                link.consume()
                phase, phase_at = "AFTER", now
                samples.clear()
            elif phase == "AFTER":
                stable = stable_center(samples, point)
                if stable:
                    after = stable
                    try:
                        k, dx, turn_sign = measure_pixel_to_deg(args.angle10, before, after)
                        result_text = "pixel_to_deg=%.6f deg/pixel; dx=%+.1f px; turn_sign=%d" % (k, dx, turn_sign)
                        print("RESULT " + result_text, flush=True)
                        phase = "DONE"
                    except ValueError as exc:
                        error, phase = str(exc), "ERROR"
                elif now - phase_at > PHASE_TIMEOUT_S:
                    error, phase = "TARGET_NOT_STABLE_AFTER_TURN", "ERROR"

            colors = {"ordinary": image.Color.from_rgb(0, 255, 0),
                      "core": image.Color.from_rgb(0, 0, 0),
                      "injured": image.Color.from_rgb(255, 165, 0),
                      "danger": image.Color.from_rgb(0, 255, 255)}
            for o in objects:
                label = detector.labels[o.class_id]
                selected = point is not None and label == args.label
                img.draw_rect(o.x, o.y, o.w, o.h,
                              color=image.COLOR_RED if selected else colors.get(label, image.COLOR_WHITE),
                              thickness=3 if selected else 2)
            lines = ["PIXEL TO DEG: " + phase,
                     "LABEL %s count=%d" % (args.label, count),
                     "CENTER --" if point is None else "CENTER %.1f,%.1f" % point,
                     "CMD 0x03 %d (%.1f deg)" % (args.angle10, args.angle10 / 10.0) if args.send else "CMD 0x03 -- preview",
                     "BEFORE --" if before is None else "BEFORE %.1f,%.1f" % before,
                     "AFTER --" if after is None else "AFTER %.1f,%.1f" % after,
                     "RESULT " + result_text if result_text else "RESULT --",
                     "ERROR " + error if error else ""]
            for i, line in enumerate(lines):
                if line:
                    img.draw_string(3, 3 + i * 16, line, color=image.COLOR_WHITE,
                                    scale=cfg["debug_text_scale"], wrap=False)
            disp.show(img)
    finally:
        if link and link.pending and link.pending["packet"].cmd != STOP:
            try:
                link.request(STOP, 1, time.monotonic())
            except Exception:
                pass
        if serial:
            serial.close()
        if cam:
            cam.close()
        if disp:
            disp.close()


def main():
    parser = argparse.ArgumentParser(description="用一次已知0x03转角测量pixel_to_deg")
    parser.add_argument("--send", action="store_true", help="实测模式：先发停车，再发一次0x03；默认仅预览")
    parser.add_argument("--angle10", type=int, default=50, help="测试角度，单位0.1度；50表示左转5度")
    parser.add_argument("--label", default="ordinary", help="画面中唯一的标定物料类别")
    parser.add_argument("--self-test", action="store_true", help="PC端计算与通信帧自检")
    args = parser.parse_args()
    self_test() if args.self_test else run(args)


if __name__ == "__main__":
    main()
