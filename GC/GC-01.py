"""GC-01：MaixCAM Pro救援视觉；只用YOLO分类/坐标和MaixPy原生图像接口。"""
import argparse
import json
import time
from pathlib import Path

from rescue_comm import Link, Packet, STOP, MOVE, TURN, GRIP, SPIN, TEAM, START, describe


def load_cfg():
    import importlib.util
    path = Path(__file__).with_name("GC-01-config.py")
    spec = importlib.util.spec_from_file_location("gc01_config", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.CFG


def center(box):
    return box[0] + box[2] / 2, box[1] + box[3] / 2


def zone_target(zone):
    x, y = center(zone["box"])
    return int(x), int(y)


def inside(point, rect):
    x, y, w, h = rect
    return x <= point[0] <= x + w and y <= point[1] <= y + h


def angle10_for_dx(dx, cfg):
    k = cfg["pixel_to_deg"]
    if k is None or k <= 0:
        return None
    if abs(dx) <= cfg["fine_center_px"]:
        return 0
    angle = max(-cfg["turn_limit_deg"], min(cfg["turn_limit_deg"], cfg["turn_sign"] * k * dx))
    return round(angle * 10)


def combo_ok(mode, detections, roi):
    found = [d for d in detections if inside(center(d["box"]), roi)]
    labels = [d["label"] for d in found]
    if mode == "A":
        return labels.count("ordinary") == 1 and len(labels) == 1
    if mode == "C":
        return labels.count("injured") == 1 and len(labels) == 1
    return 1 <= len(labels) <= 3 and all(x in ("ordinary", "core") for x in labels)


class Bot:
    def __init__(self, cfg, link, test_mode=False):
        self.cfg, self.link = cfg, link
        self.state, self.phase = "00", "WAIT"
        self.test_mode = test_mode
        self.team, self.started = None, False
        self.first_done, self.mode = test_mode, "B" if test_mode else "A"
        self.target, self.carrying = None, False
        self.bad_load_recovery, self.boot_stop_sent = False, False
        self.zone, self.avoidances = None, 0
        self.target_turn_tries = 0
        self.phase_at = 0.0
        self.reason = "WAIT_TEAM_START"
        self.last_log_key = None

    def events(self, now):
        while self.link.events:
            p = self.link.events.popleft()
            ok = False
            if p.cmd == TEAM and p.data[0] in (0, 1):
                team = "blue" if p.data[0] == 0 else "red"
                ok = not self.started or self.team == team
                if ok:
                    self.team = team
            elif p.cmd == START and p.data == b"\x01" and self.team:
                ok = True
                if not self.started:
                    self.started = True
                    self.phase, self.reason = "START_STOP", ""
            self.link.reply(p, ok, now)

    def select_mode(self, detections):
        if not self.first_done:
            self.mode = "A"
            return next((d for d in detections if d["label"] == "ordinary"), None)
        injured = next((d for d in detections if d["label"] == "injured"), None)
        if injured:
            self.mode = "C"
            return injured
        self.mode = "B"
        return next((d for d in detections if d["label"] in ("ordinary", "core")), None)

    def locked_detection(self, detections):
        if not self.target:
            return None
        same = [d for d in detections if d["label"] == self.target["label"]]
        if not same:
            return None
        tx, ty = center(self.target["box"])
        return min(same, key=lambda d: (center(d["box"])[0] - tx) ** 2 + (center(d["box"])[1] - ty) ** 2)

    def action_done(self, cmd, value, now):
        pending = self.link.pending
        if pending is None:
            self.link.request(cmd, value, now)
            return False
        if pending["packet"].cmd != cmd:
            return False
        return self.link.consume()

    def align_target(self, detections, now):
        d = self.locked_detection(detections)
        if not d:
            self.phase, self.target = "SEARCH", None
            self.target_turn_tries = 0
            return
        self.target = d
        dx = center(d["box"])[0] - self.cfg["camera_size"][0] / 2
        if abs(dx) <= self.cfg["fine_center_px"]:
            self.state, self.phase = "01", "OPEN"
            self.reason = ""
            self.target_turn_tries = 0
            return
        angle10 = angle10_for_dx(dx, self.cfg)
        if angle10 is None:
            self.phase, self.reason = "HOLD", "PIXEL_CALIBRATION_REQUIRED"
            return
        if angle10 == 0:
            self.state, self.phase = "01", "OPEN"
            self.reason = ""
            self.target_turn_tries = 0
            return
        if self.target_turn_tries >= self.cfg["target_turn_max_tries"]:
            self.phase, self.reason = "HOLD", "TARGET_ALIGN_LIMIT"
            return
        self.target_turn_tries += 1
        self.link.request(TURN, angle10, now)
        self.phase = "TARGET_TURN"

    def recovery(self, now, bad_load=False):
        if self.phase == "REC_STOP" and self.action_done(STOP, 1, now):
            self.phase, self.phase_at = "REC_BACK", now
        elif self.phase == "REC_BACK":
            self.link.stream(MOVE, 0, now)
            if now - self.phase_at >= (self.cfg["bad_load_back_ms"] if bad_load else self.cfg["avoid_back_ms"]) / 1000:
                self.link.request(STOP, 1, now)
                self.phase = "REC_BACK_STOP"
        elif self.phase == "REC_BACK_STOP" and self.action_done(STOP, 1, now):
            if bad_load:
                self.phase, self.state, self.target = "SEARCH", "00", None
                self.bad_load_recovery = False
            else:
                angle = self.cfg["avoid_turn_deg"] * 10 * (1 if self.cfg["avoid_turn_direction"] == "left" else -1)
                self.link.request(TURN, int(angle), now)
                self.phase = "REC_TURN"
        elif self.phase == "REC_TURN" and self.action_done(TURN, 0, now):
            self.link.request(STOP, 1, now)
            self.phase = "REC_FINAL_STOP"
        elif self.phase == "REC_FINAL_STOP" and self.action_done(STOP, 1, now):
            self.phase, self.state, self.target = "SEARCH", "00", None

    def tick(self, detections, zones, obstacle, roi_combo, now):
        if not self.boot_stop_sent and self.link.pending is None:
            self.link.request(STOP, 1, now)
            self.boot_stop_sent = True
        self.events(now)
        if not self.started:
            return
        if self.link.error:
            self.reason = self.link.error
            if not self.link.pending or self.link.pending["packet"].cmd != STOP:
                try:
                    self.link.request(STOP, 1, now)
                except RuntimeError:
                    pass
            return
        if obstacle and self.phase not in ("REC_STOP", "REC_BACK", "REC_BACK_STOP", "REC_TURN", "REC_FINAL_STOP"):
            if self.avoidances >= self.cfg["max_avoidances"]:
                self.link.request(STOP, 1, now)
                self.phase, self.reason = "HOLD", "AVOID_LIMIT_10"
                return
            self.avoidances += 1
            self.link.request(STOP, 1, now)
            self.phase, self.reason = "REC_STOP", "SHADOW_OBSTACLE"
        if self.phase.startswith("REC_"):
            self.recovery(now, self.bad_load_recovery)
            return
        if self.phase == "HOLD":
            return

        if self.state == "00":
            if self.phase == "START_STOP":
                if self.action_done(STOP, 1, now):
                    self.phase = "SEARCH"
                return
            if self.carrying:
                own = next((z for z in zones if z["team"] == self.team), None)
                if self.phase == "SEARCH":
                    self.link.stream(SPIN, self.cfg["search_spin"], now)
                    if own and abs(zone_target(own)[0] - self.cfg["camera_size"][0] / 2) <= self.cfg["coarse_center_px"]:
                        self.zone = own
                        self.link.request(STOP, 1, now)
                        self.phase = "ZONE_STOP"
                elif self.phase == "ZONE_STOP" and self.action_done(STOP, 1, now):
                    self.state, self.phase = "03", "DRIVE_ZONE"
                return
            if self.phase == "SEARCH":
                self.link.stream(SPIN, self.cfg["search_spin"], now)
                chosen = self.locked_detection(detections) if self.target else None
                if chosen is None and self.target is None:
                    chosen = self.select_mode(detections)
                if chosen:
                    self.target = chosen
                    dx = center(chosen["box"])[0] - self.cfg["camera_size"][0] / 2
                    if abs(dx) <= self.cfg["coarse_center_px"]:
                        self.target_turn_tries = 0
                        self.link.request(STOP, 1, now)
                        self.phase = "TARGET_STOP"
            elif self.phase == "TARGET_STOP" and self.action_done(STOP, 1, now):
                self.align_target(detections, now)
            elif self.phase == "TARGET_TURN" and self.action_done(TURN, 0, now):
                self.phase = "TARGET_RECHECK"
            elif self.phase == "TARGET_RECHECK":
                self.align_target(detections, now)
            return

        if self.state == "01":
            if self.phase == "OPEN" and self.action_done(GRIP, 0, now):
                self.phase = "APPROACH"
            elif self.phase == "APPROACH":
                d = self.locked_detection(detections)
                if d:
                    self.target = d
                    if inside(center(d["box"]), self.cfg["target_roi"]):
                        self.link.request(STOP, 1, now)
                        self.phase = "APPROACH_STOP"
                    else:
                        self.link.stream(MOVE, 1, now)
                else:
                    self.link.stream(MOVE, 1, now)
            elif self.phase == "APPROACH_STOP" and self.action_done(STOP, 1, now):
                self.state, self.phase = "02", "CLOSE"
            return

        if self.state == "02":
            if self.phase == "CLOSE" and self.action_done(GRIP, 1, now):
                if roi_combo:
                    self.carrying = True
                    self.state, self.phase = "00", "SEARCH"
                else:
                    self.link.request(GRIP, 0, now)
                    self.phase = "BAD_OPEN"
            elif self.phase == "BAD_OPEN" and self.action_done(GRIP, 0, now):
                self.bad_load_recovery = True
                self.phase, self.phase_at = "REC_BACK", now
            return

        if self.state == "03":
            d = self.locked_detection(detections)
            if self.phase == "DRIVE_ZONE":
                self.zone = next((z for z in zones if z["team"] == self.team), None)
                self.link.stream(MOVE, 1, now)
                if self.zone and d and inside(center(d["box"]), self.zone["box"]):
                    self.target = d
                    self.link.request(STOP, 1, now)
                    self.phase = "DROP_STOP"
            elif self.phase == "DROP_STOP" and self.action_done(STOP, 1, now):
                self.link.request(GRIP, 0, now)
                self.phase = "DROP_OPEN"
            elif self.phase == "DROP_OPEN" and self.action_done(GRIP, 0, now):
                self.first_done = self.first_done or self.mode == "A"
                self.carrying = False
                self.link.stream(MOVE, 0, now)
                self.phase, self.phase_at = "DELIVERY_BACK", now
            elif self.phase == "DELIVERY_BACK":
                self.link.stream(MOVE, 0, now)
                if now - self.phase_at >= self.cfg["delivery_back_ms"] / 1000:
                    self.link.request(STOP, 1, now)
                    self.phase = "DELIVERY_STOP"
            elif self.phase == "DELIVERY_STOP" and self.action_done(STOP, 1, now):
                self.target, self.zone = None, None
                self.state, self.phase = "00", "SEARCH"


def measure_zones(img, cfg, detections=()):
    out = []
    for team in ("red", "blue"):
        blobs = img.find_blobs(cfg["zone_" + team + "_lab"], pixels_threshold=1, area_threshold=1,
                               x_stride=1, y_stride=1, merge=True, margin=2)
        boxes = [tuple(b.rect()) for b in blobs]
        boxes = [b for b in boxes if b[2] >= 2 * b[3]
                 and not any(inside(center(b), d["box"]) for d in detections)]
        if not boxes:
            continue
        # shortcut: 按前视地板的横向形状筛选，换俯视/侧视相机后需重新标定。
        sx, sy, sw, sh = max(boxes, key=lambda b: b[2])
        left, top, right, bottom = sx, sy, sx + sw, sy + sh
        for x, y, w, h in boxes:
            overlap = min(sy + sh, y + h) - max(sy, y)
            gap = max(sx - x - w, x - sx - sw, 0)
            if overlap * 2 >= min(sh, h) and gap <= max(sh, h):
                left, top = min(left, x), min(top, y)
                right, bottom = max(right, x + w), max(bottom, y + h)
        out.append({"team": team, "box": (left, top, right - left, bottom - top)})
    return out


def obstacle_seen(img, cfg):
    blobs = img.find_blobs(cfg["shadow_lab"], roi=cfg["obstacle_roi"],
                           pixels_threshold=cfg["shadow_min_pixels"], area_threshold=cfg["shadow_min_pixels"],
                           x_stride=2, y_stride=2, merge=True)
    return any(blob.pixels() >= cfg["shadow_min_pixels"] for blob in blobs)


def draw(img, cfg, bot, detections, zones):
    from maix import image
    colors = {"ordinary": image.Color.from_rgb(0, 255, 0), "core": image.Color.from_rgb(0, 0, 0),
              "injured": image.Color.from_rgb(255, 165, 0), "danger": image.Color.from_rgb(0, 255, 255)}
    current = bot.locked_detection(detections) if bot.target else None
    for d in detections:
        box = d["box"]
        selected = current is d
        img.draw_rect(*map(int, box), color=image.COLOR_RED if selected else colors[d["label"]],
                      thickness=3 if selected else 2)
    for z in zones:
        color = image.COLOR_RED if z["team"] == "red" else image.COLOR_BLUE
        img.draw_rect(*z["box"], color=color, thickness=2)
    img.draw_rect(*cfg["target_roi"], color=image.Color.from_rgb(128, 128, 128), thickness=1)
    indicator = (bot.state if bot.test_mode else
                 "D" if bot.link.last_rx is None else
                 "W" if cfg["match_id"] == "UNCONFIGURED" else bot.state)
    img.draw_string(3, 3, bot.mode + " " + indicator, color=image.COLOR_WHITE,
                    scale=cfg["debug_text_scale"], wrap=False)
    if bot.test_mode:
        angle10 = angle10_for_dx(center(current["box"])[0] - cfg["camera_size"][0] / 2, cfg) if current else None
    else:
        angle10 = bot.link.last_turn_tenths
    angle_text = "CMD 0x03 --" if angle10 is None or angle10 == 0 else "CMD 0x03 %d" % angle10
    img.draw_string(3, 18, angle_text, color=image.COLOR_WHITE,
                    scale=cfg["debug_text_scale"], wrap=False)
    target_text = "T --" if current is None else "T %d,%d" % tuple(int(round(v)) for v in center(current["box"]))
    img.draw_string(3, 33, target_text, color=image.COLOR_WHITE,
                    scale=cfg["debug_text_scale"], wrap=False)
    img.draw_string(214, 3, "TX " + (describe(bot.link.last_tx) if bot.link.last_tx else "-----"), color=image.COLOR_WHITE,
                    scale=cfg["debug_text_scale"], wrap=False)
    img.draw_string(214, 18, "RX " + (describe(bot.link.last_rx) if bot.link.last_rx else "-----"), color=image.COLOR_WHITE,
                    scale=cfg["debug_text_scale"], wrap=False)
    for z in zones:
        if bot.team is not None and z["team"] != bot.team:
            continue
        x, y = zone_target(z)
        img.draw_cross(x, y, color=image.COLOR_RED, size=7, thickness=2)
        text = "%s %s(%d,%d)" % (z["team"][0].upper(), "P" if bot.team is None else "G", x, y)
        size = image.string_size(text, scale=cfg["debug_text_scale"], thickness=-1)
        tx = max(0, min(x + 10, cfg["camera_size"][0] - size.width()))
        ty = max(0, min(y + 8, cfg["camera_size"][1] - size.height()))
        img.draw_rect(tx, ty, size.width(), size.height(), color=image.COLOR_BLACK, thickness=-1)
        img.draw_string(tx, ty, text, color=image.COLOR_WHITE,
                        scale=cfg["debug_text_scale"], thickness=-1, wrap=False)
    return img


def print_changed_status(bot, cfg):
    state = (bot.state if bot.test_mode else
             "D" if bot.link.last_rx is None else
             "W" if cfg["match_id"] == "UNCONFIGURED" else bot.state)
    tx = describe(bot.link.last_tx) if bot.link.last_tx else "-----"
    rx = describe(bot.link.last_rx) if bot.link.last_rx else "-----"
    key = (bot.mode, state, tx, rx)
    if key != bot.last_log_key:
        print("%s %s TX：%s  RX：%s" % key, flush=True)
        bot.last_log_key = key


def profile_report(samples, camera_fps, setup_ms):
    report = {}
    for key, vals in samples.items():
        if not vals:
            continue
        report[key] = {"avg_ms": round(sum(vals) / len(vals), 3),
                       "p95_ms": round(sorted(vals)[int((len(vals) - 1) * .95)], 3),
                       "max_ms": round(max(vals), 3)}
    report["fps_end_to_end"] = round(1000 / report["cycle"]["avg_ms"], 2) if report.get("cycle", {}).get("avg_ms") else 0
    report["camera_fps_requested"] = load_cfg()["camera_fps"]
    report["camera_fps_reported"] = camera_fps
    report["setup_ms"] = setup_ms
    print("PROFILE " + json.dumps(report, ensure_ascii=False), flush=True)


def run(preview=False, profile_frames=0):
    from maix import app, camera, display, image, pinmap, uart, nn
    cfg = load_cfg()
    setup = {}
    s = time.monotonic()
    detector = nn.YOLOv8(model=cfg["model"], dual_buff=False)
    setup["model_init_ms"] = round((time.monotonic() - s) * 1000, 3)
    if tuple(detector.labels) != tuple(cfg["labels"]):
        raise RuntimeError("YOLO_LABELS_MISMATCH")
    if (detector.input_width(), detector.input_height()) != cfg["camera_size"]:
        raise RuntimeError("YOLO_INPUT_SIZE_MISMATCH")
    serial = None
    if not preview:
        s = time.monotonic()
        for pin, fn in cfg["uart"]["pins"]:
            if pinmap.set_pin_function(pin, fn) != 0:
                raise RuntimeError("PINMAP:" + pin)
        serial = uart.UART(cfg["uart"]["device"], cfg["uart"]["baud"])
        setup["uart_init_ms"] = round((time.monotonic() - s) * 1000, 3)
    link = Link(cfg, serial.write if serial else lambda data: len(data), lambda e: None)
    bot = Bot(cfg, link, test_mode=preview)
    fmt = detector.input_format()
    s = time.monotonic()
    cam = camera.Camera(*cfg["camera_size"], fmt, fps=cfg["camera_fps"], buff_num=3)
    setup["camera_init_ms"] = round((time.monotonic() - s) * 1000, 3)
    s = time.monotonic()
    disp = display.Display()
    setup["display_init_ms"] = round((time.monotonic() - s) * 1000, 3)
    names = ("camera_read", "uart_rx", "uart_encode", "uart_write", "yolo_inference", "yolo_box_extract", "zone_blobs", "shadow_blobs", "roi_combo",
             "state_machine", "draw", "display", "cycle")
    samples = {n: [] for n in names}
    frames, warming = 0, 10
    try:
        while not app.need_exit():
            t0 = time.monotonic()
            t = time.monotonic()
            link.cycle_encode_ms = link.cycle_write_ms = 0.0
            if serial:
                s = time.monotonic()
                link.poll(serial.read(len=256, timeout=0), t)
                samples["uart_rx"].append((time.monotonic() - s) * 1000)
            s = time.monotonic()
            img = cam.read()
            samples["camera_read"].append((time.monotonic() - s) * 1000)
            if img is None:
                continue
            if warming:
                warming -= 1
                continue
            s = time.monotonic()
            objects = detector.detect(img, conf_th=cfg["confidence"], iou_th=0.45)
            samples["yolo_inference"].append((time.monotonic() - s) * 1000)
            s = time.monotonic()
            detections = [{"label": detector.labels[obj.class_id], "box": (obj.x, obj.y, obj.w, obj.h), "score": obj.score}
                          for obj in objects]
            samples["yolo_box_extract"].append((time.monotonic() - s) * 1000)
            s = time.monotonic()
            zones = measure_zones(img, cfg, detections)
            samples["zone_blobs"].append((time.monotonic() - s) * 1000)
            s = time.monotonic()
            obstacle = obstacle_seen(img, cfg)
            samples["shadow_blobs"].append((time.monotonic() - s) * 1000)
            s = time.monotonic()
            valid_combo = combo_ok(bot.mode, detections, cfg["target_roi"]) if bot.state == "02" else False
            if bot.state == "02":
                samples["roi_combo"].append((time.monotonic() - s) * 1000)
            s = time.monotonic()
            if not preview:
                bot.tick(detections, zones, obstacle, valid_combo, time.monotonic())
            else:
                bot.mode = "B"
                bot.state = "00"
                bot.target = next((d for d in detections if d["label"] in ("ordinary", "core")), None)
            print_changed_status(bot, cfg)
            samples["state_machine"].append((time.monotonic() - s) * 1000)
            samples["uart_encode"].append(link.cycle_encode_ms)
            samples["uart_write"].append(link.cycle_write_ms)
            if frames % cfg["display_every"] == 0:
                s = time.monotonic()
                draw(img, cfg, bot, detections, zones)
                samples["draw"].append((time.monotonic() - s) * 1000)
                s = time.monotonic()
                disp.show(img)
                samples["display"].append((time.monotonic() - s) * 1000)
            samples["cycle"].append((time.monotonic() - t0) * 1000)
            frames += 1
            if profile_frames and frames >= profile_frames:
                break
    finally:
        actual_fps = cam.fps()
        if serial:
            try:
                link.request(STOP, 1, time.monotonic())
            except Exception:
                pass
            serial.close()
        cam.close()
        disp.close()
        if profile_frames:
            profile_report(samples, actual_fps, setup)


def self_test():
    from collections import deque
    roi = (100, 140, 120, 80)
    det = lambda label, x, y: {"label": label, "box": (x - 5, y - 5, 10, 10)}
    assert combo_ok("A", [det("ordinary", 110, 150)], roi)
    assert not combo_ok("A", [det("ordinary", 110, 150), det("danger", 120, 150)], roi)
    assert combo_ok("B", [det("ordinary", 110, 150), det("core", 130, 150)], roi)
    assert combo_ok("B", [det("ordinary", 110, 150), det("core", 130, 150), det("ordinary", 150, 150)], roi)
    assert not combo_ok("B", [det("ordinary", 110, 150), det("ordinary", 130, 150), det("core", 150, 150), det("core", 170, 150)], roi)
    assert combo_ok("C", [det("injured", 110, 150)], roi)
    assert not combo_ok("C", [det("injured", 110, 150), det("ordinary", 120, 150)], roi)
    assert not combo_ok("A", [det("ordinary", 20, 20)], roi)
    angle_cfg = {"pixel_to_deg": 0.25, "fine_center_px": 5, "turn_limit_deg": 5.0, "turn_sign": -1}
    assert angle10_for_dx(12, angle_cfg) == -30
    assert angle10_for_dx(20, angle_cfg) == -50
    assert angle10_for_dx(-20, angle_cfg) == 50
    assert angle10_for_dx(100, angle_cfg) == -50
    assert angle10_for_dx(2, angle_cfg) == 0
    angle_cfg["pixel_to_deg"] = 0
    assert angle10_for_dx(20, angle_cfg) is None
    angle_cfg["pixel_to_deg"] = None
    assert angle10_for_dx(20, angle_cfg) is None

    class TurnRecorder:
        pending = None
        def __init__(self):
            self.requests = []
        def request(self, cmd, value, now):
            self.requests.append((cmd, value))
            self.pending = {"packet": Packet(cmd, 1, b"")}

    align_link = TurnRecorder()
    align_cfg = {"camera_size": (320, 224), "fine_center_px": 5, "pixel_to_deg": 0.25,
                 "turn_sign": -1, "turn_limit_deg": 5.0, "target_turn_max_tries": 3}
    align_bot = Bot(align_cfg, align_link)
    align_bot.target = det("ordinary", 200, 100)
    align_bot.align_target([align_bot.target], 0.0)
    assert align_link.requests[-1] == (TURN, -50)
    align_link.pending = None
    moved_target = det("ordinary", 170, 100)
    align_bot.align_target([moved_target], 1.0)
    assert align_link.requests[-1] == (TURN, -25)
    aligned_target = det("ordinary", 162, 100)
    align_bot.align_target([aligned_target], 2.0)
    assert (align_bot.state, align_bot.phase) == ("01", "OPEN")

    class FakeLink:
        def __init__(self):
            self.events, self.acks = deque(), []
        def reply(self, packet, accepted, now):
            self.acks.append((packet.cmd, accepted))
    link = FakeLink()
    bot = Bot({"camera_size": (320, 224)}, link)
    link.events.extend((Packet(TEAM, 1, bytes([1])), Packet(START, 2, bytes([1]))))
    bot.events(0.0)
    bot.state, bot.phase = "02", "CLOSE"
    link.events.extend((Packet(START, 3, bytes([1])), Packet(TEAM, 4, bytes([0]))))
    bot.events(1.0)
    assert bot.started and bot.team == "red" and (bot.state, bot.phase) == ("02", "CLOSE")
    assert link.acks == [(TEAM, True), (START, True), (START, True), (TEAM, False)]
    from rescue_comm import TURN as TURN_CMD
    writer = Link({}, lambda data: len(data))
    writer.write(Packet(TURN_CMD, 9, (-25).to_bytes(2, "little", signed=True)), 0.0)
    assert writer.last_turn_tenths == -25
    print("SELF_TEST PASS", flush=True)


def main():
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--preview", action="store_true", help="相机+YOLO显示，不开控制UART、不运动")
    group.add_argument("--run", action="store_true", help="比赛模式")
    group.add_argument("--self-test", action="store_true", help="PC端组合/ROI逻辑自检")
    parser.add_argument("--profile", type=int, default=0, help="统计指定帧数后退出并打印分段耗时")
    args = parser.parse_args()
    if args.self_test:
        self_test()
    else:
        run(args.preview, args.profile)


if __name__ == "__main__":
    main()
