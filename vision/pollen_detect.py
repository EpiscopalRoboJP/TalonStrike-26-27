"""Yellow pollen detection shared with the robot processor.

Pipeline, in order. PollenDetector.java does these same steps:
1. Convert to OpenCV HSV. This file starts from BGR. The robot frame is RGB,
   so the Java copy uses RGB to HSV. Hue numbers match either way.
2. Swap any reversed low/high channel.
3. Gaussian blur when blur_size is at least 3, with an odd kernel.
4. inRange yellow mask.
5. Close (dilate, then erode) when close_size is at least 2.
6. Every external contour on the full mask is a ball when its area is at
   least min_area_fraction of the frame and its circularity clears
   min_circularity. Circularity is 4*pi*area/perimeter^2, clamped to 1.
7. A ball whose center is inside the front rectangle counts as front.
   Otherwise a ball whose center is inside the storage rectangle counts
   as storage. Front wins when the two rectangles overlap.
"""

import json
import math
from pathlib import Path

import cv2
import numpy as np

ASSET_CONFIG = (
    Path(__file__).resolve().parents[1]
    / "TeamCode"
    / "src"
    / "main"
    / "assets"
    / "pollen_vision.json"
)


def default_config():
    return {
        "width": 640,
        "height": 480,
        "hsv_low": [15, 15, 180],
        "hsv_high": [40, 255, 255],
        "blur_size": 0,
        "close_size": 21,
        "min_area_fraction": 0.003,
        "min_circularity": 0.38,
        "front": {"name": "front", "x": 0.18, "y": 0.00, "w": 0.45, "h": 0.08},
        "storage": {"name": "storage", "x": 0.00, "y": 0.08, "w": 1.00, "h": 0.92},
    }


def load_config(path=None):
    config_path = Path(path) if path is not None else ASSET_CONFIG
    with config_path.open(encoding="utf-8") as handle:
        return normalize(json.load(handle))


def save_config(path, config):
    config_path = Path(path)
    config_path.parent.mkdir(parents=True, exist_ok=True)
    text = json.dumps(normalize(config), indent=2) + "\n"
    config_path.write_text(text, encoding="utf-8")


def normalize(config):
    base = default_config()
    source = config or {}
    front_source = source.get("front")
    if front_source is None:
        front = dict(base["front"])
    else:
        front = sanitize_region(front_source, "front")
    storage_source = source.get("storage")
    if storage_source is None:
        storage = dict(base["storage"])
    else:
        storage = sanitize_region(storage_source, "storage")
    return {
        "width": clamp(read_int(source, "width", base["width"]), 160, 1920),
        "height": clamp(read_int(source, "height", base["height"]), 120, 1920),
        "hsv_low": sanitize_hsv(source.get("hsv_low"), base["hsv_low"]),
        "hsv_high": sanitize_hsv(source.get("hsv_high"), base["hsv_high"]),
        "blur_size": clamp(read_int(source, "blur_size", base["blur_size"]), 0, 31),
        "close_size": clamp(read_int(source, "close_size", base["close_size"]), 0, 31),
        "min_area_fraction": read_min_area(source, front, base["min_area_fraction"]),
        "min_circularity": clamp_unit(
            source.get("min_circularity", source.get("front_min_circularity")),
            base["min_circularity"],
        ),
        "front": front,
        "storage": storage,
    }


def detect(frame_bgr, config):
    if frame_bgr is None or frame_bgr.size == 0:
        return empty_result()
    height, width = frame_bgr.shape[:2]
    if width < 2 or height < 2:
        return empty_result()

    low, high = ordered_hsv(config["hsv_low"], config["hsv_high"])
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    kernel = odd_kernel(config["blur_size"])
    if kernel is not None:
        hsv = cv2.GaussianBlur(hsv, (kernel, kernel), 0)
    mask = cv2.inRange(hsv, np.array(low, dtype=np.uint8), np.array(high, dtype=np.uint8))
    close = int(config["close_size"])
    if close >= 2:
        element = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (close, close))
        mask = cv2.dilate(mask, element)
        mask = cv2.erode(mask, element)

    front_rect = region_rect(config["front"], width, height)
    storage_rect = region_rect(config["storage"], width, height)
    front_balls = []
    storage_balls = []
    for ball in find_balls(mask, config, width, height):
        center = ball["center"]
        if point_inside(center, front_rect):
            front_balls.append(ball)
        elif point_inside(center, storage_rect):
            storage_balls.append(ball)

    return {
        "front_count": len(front_balls),
        "storage_count": len(storage_balls),
        "front": {"name": config["front"]["name"], "rect": front_rect, "balls": front_balls},
        "storage": {"name": config["storage"]["name"], "rect": storage_rect, "balls": storage_balls},
        "mask": mask,
    }


def find_balls(mask, config, width, height):
    frame_area = float(width * height)
    min_area = float(config["min_area_fraction"]) * frame_area
    min_circularity = float(config["min_circularity"])
    contours, _hierarchy = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    balls = []
    for contour in contours:
        area = float(cv2.contourArea(contour))
        if area < 1.0 or area < min_area or frame_area <= 0.0:
            continue
        perimeter = float(cv2.arcLength(contour, True))
        circularity = 0.0
        if perimeter > 1e-6:
            circularity = min(1.0, (4.0 * math.pi * area) / (perimeter * perimeter))
        if circularity < min_circularity:
            continue
        (cx, cy), radius = cv2.minEnclosingCircle(contour)
        balls.append({
            "circularity": circularity,
            "area_fraction": area / frame_area,
            "center": (float(cx), float(cy)),
            "radius": float(radius),
        })
    return balls


def region_rect(region, width, height):
    """Pixel rectangle. Rounding matches Java Math.round so both sides agree."""
    x = clamp(round_half_up(region["x"] * width), 0, width - 1)
    y = clamp(round_half_up(region["y"] * height), 0, height - 1)
    right = clamp(round_half_up((region["x"] + region["w"]) * width), x + 1, width)
    bottom = clamp(round_half_up((region["y"] + region["h"]) * height), y + 1, height)
    return x, y, right - x, bottom - y


def point_inside(center, rect):
    x, y, rect_w, rect_h = rect
    return x <= center[0] < x + rect_w and y <= center[1] < y + rect_h


def empty_result():
    return {
        "front_count": 0,
        "storage_count": 0,
        "front": {"name": "front", "rect": (0, 0, 0, 0), "balls": []},
        "storage": {"name": "storage", "rect": (0, 0, 0, 0), "balls": []},
        "mask": None,
    }


def ordered_hsv(low, high):
    lower = []
    upper = []
    for index in range(3):
        lower.append(min(int(low[index]), int(high[index])))
        upper.append(max(int(low[index]), int(high[index])))
    return lower, upper


def odd_kernel(size):
    kernel = int(size)
    if kernel < 3:
        return None
    if kernel % 2 == 0:
        kernel += 1
    return kernel


def read_min_area(source, front, fallback):
    """Older files stored the cutoff as a fraction of the front box."""
    if isinstance(source, dict) and "min_area_fraction" in source:
        return clamp_unit(source.get("min_area_fraction"), fallback)
    old = None if not isinstance(source, dict) else source.get("front_min_area_fraction")
    if old is None:
        return clamp_unit(fallback, fallback)
    converted = read_number(old, fallback) * float(front["w"]) * float(front["h"])
    return clamp(converted, 0.0, 1.0)


def sanitize_region(region, fallback_name):
    source = region or {}
    name = source.get("name")
    if not name:
        name = fallback_name
    x = clamp_unit(source.get("x"), 0.0)
    y = clamp_unit(source.get("y"), 0.0)
    w = clamp_unit(source.get("w"), 0.2)
    h = clamp_unit(source.get("h"), 0.2)
    if w < 0.03:
        w = 0.03
    if h < 0.03:
        h = 0.03
    if x + w > 1.0:
        x = max(0.0, 1.0 - w)
    if y + h > 1.0:
        y = max(0.0, 1.0 - h)
    return {"name": str(name), "x": x, "y": y, "w": w, "h": h}


def sanitize_hsv(values, fallback):
    limits = (179, 255, 255)
    cleaned = []
    for index, limit in enumerate(limits):
        raw = fallback[index]
        if isinstance(values, (list, tuple)) and index < len(values):
            raw = values[index]
        cleaned.append(clamp(read_number(raw, fallback[index]), 0, limit))
    return cleaned


def clamp_unit(value, fallback):
    return clamp(read_number(value, fallback), 0.0, 1.0)


def read_int(source, key, fallback):
    if not isinstance(source, dict) or key not in source:
        return int(fallback)
    return int(round(read_number(source[key], fallback)))


def read_number(value, fallback):
    try:
        number = float(value)
    except (TypeError, ValueError):
        return float(fallback)
    if math.isnan(number) or math.isinf(number):
        return float(fallback)
    return number


def clamp(value, low, high):
    return max(low, min(high, value))


def round_half_up(value):
    return int(math.floor(float(value) + 0.5))
