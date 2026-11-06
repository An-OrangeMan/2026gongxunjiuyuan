"""现场可调参数；全部坐标按320x224相机原图像素。"""
from pathlib import Path

ROOT = Path(__file__).resolve().parent
CFG = {
    "model": str(ROOT / "models" / "model_10735.mud"),
    "labels": ("ordinary", "core", "injured", "danger"),
    "camera_size": (320, 224),
    "camera_fps": 60,                 # 请求值；官方资料对GC4653上限存在差异，以cam.fps()为准
    "display_every": 2,
    "confidence": 0.55,
    "target_roi": (100, 140, 120, 80),
    "coarse_center_px": 30,
    "fine_center_px": 8,
    "pixel_to_deg": 0.05,             # 实测后填度/像素；None时用粗对准范围
    "turn_sign": -1,
    "turn_limit_deg": 5.0,
    "target_turn_max_tries": 3,
    "search_spin": 0,                 # 0右转、1左转
    # 板端对两张蓝区照片和真实红区照片取样：橙块b更高，蓝物料a更高。
    "zone_red_lab": [[18, 50, 20, 60, 0, 31]],
    "zone_blue_lab": [[30, 72, -24, -5, -34, -8]],
    "shadow_lab": [[0, 35, -15, 15, -15, 15]],
    "obstacle_roi": (20, 105, 280, 114),
    "shadow_min_pixels": 300,
    "avoid_back_ms": 600,
    "avoid_turn_deg": 30,
    "avoid_turn_direction": "right",
    "max_avoidances": 10,
    "bad_load_back_ms": 600,
    "delivery_back_ms": 1500,
    "command_hz": 10,
    "feedback_timeout_s": 0.5,
    "stop_timeout_s": 1.5,
    "turn_timeout_s": 5.0,
    "gripper_timeout_s": 3.0,
    "partial_timeout_s": 0.1,
    "rx_max_data": 16,
    "rx_max_buffer": 4096,
    "uart": {"device": "/dev/ttyS1", "baud": 115200,
             "pins": (("A18", "UART1_RX"), ("A19", "UART1_TX"))},
    "match_id": "UNCONFIGURED",       # 设置真实场次ID后屏幕左上角显示00—03；未设置显示W
    "debug_text_scale": 0.7,
}
