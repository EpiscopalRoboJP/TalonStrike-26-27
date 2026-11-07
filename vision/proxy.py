"""Tune pollen detection on this computer with the ELP plugged in here.

    .venv\\Scripts\\python vision\\proxy.py
    .venv\\Scripts\\python vision\\proxy.py --camera 1
    .venv\\Scripts\\python vision\\proxy.py --image snapshot.png

Press s to write TeamCode/src/main/assets/pollen_vision.json. The robot
op mode reads that file the next time the app is installed. Rectangles are
normalized. The preview keeps the camera's real shape instead of stretching
it to the robot's requested size.
"""

import argparse
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from frame_source import build_source, is_virtual_camera
from pollen_detect import ASSET_CONFIG, detect, load_config, region_rect, save_config

WINDOW = "Pollen proxy"
MAX_PREVIEW_SIDE = 960


class Proxy:
    def __init__(self, config, config_path, source, cameras):
        self.config = config
        self.config_path = config_path
        self.source = source
        self.cameras = cameras
        self.camera_pos = 0
        for index, camera in enumerate(cameras):
            if camera["index"] == getattr(source, "index", None):
                self.camera_pos = index
                break
        self.show_mask = False
        self.paused = False
        self.last_frame = None
        self.frame_w = 0
        self.frame_h = 0
        self.selected = ("front", 0)
        self.drag = None
        self.press = (0, 0)
        self.snapshot = None
        self.status = getattr(source, "notice", None) or self.camera_status()

    def run(self):
        cv2.namedWindow(WINDOW, cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(WINDOW, self.on_mouse)
        self.create_trackbars()
        try:
            while True:
                frame = self.read_frame()
                if cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 0:
                    break
                if frame is None:
                    self.status = "no frame from the camera"
                    if cv2.waitKey(50) & 0xFF in (ord("q"), 27):
                        break
                    continue
                self.frame_h, self.frame_w = frame.shape[:2]
                self.sync_config_from_trackbars()
                result = detect(frame, self.config)
                view = self.draw(frame, result)
                cv2.imshow(WINDOW, self.compose(view, result))
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    break
                if cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 0:
                    break
                self.handle_key(key)
        finally:
            self.source.close()
            cv2.destroyAllWindows()

    def read_frame(self):
        if self.paused and self.last_frame is not None:
            return self.last_frame
        frame = self.source.read()
        if frame is None:
            return self.last_frame
        frame = fit_frame(frame, MAX_PREVIEW_SIDE)
        self.last_frame = frame
        return frame

    def handle_key(self, key):
        if key == ord("m"):
            self.show_mask = not self.show_mask
            self.status = "mask" if self.show_mask else "color"
        elif key == ord(" "):
            self.paused = not self.paused
            self.status = "paused" if self.paused else "live"
        elif key == ord("s"):
            self.save()
        elif key == ord("r"):
            self.reload()
        elif key == ord("c"):
            self.next_camera()

    def camera_status(self):
        name = getattr(self.source, "name", "camera")
        if len(self.cameras) > 1:
            return name + "    press c for the other camera"
        return name

    def next_camera(self):
        usable = [
            pos for pos, camera in enumerate(self.cameras)
            if not is_virtual_camera(camera["name"])
        ]
        if len(usable) < 2 or not hasattr(self.source, "reopen"):
            self.status = "only one camera is connected"
            return
        start = self.camera_pos if self.camera_pos in usable else usable[0]
        start_slot = usable.index(start)
        last_error = None
        for step in range(1, len(usable) + 1):
            pos = usable[(start_slot + step) % len(usable)]
            camera = self.cameras[pos]
            try:
                self.source.reopen(camera["index"], camera["name"], camera["backend"])
            except RuntimeError as error:
                last_error = error
                continue
            self.camera_pos = pos
            self.last_frame = None
            self.paused = False
            if last_error is not None and pos == start:
                self.status = str(last_error)
            else:
                self.status = self.camera_status()
            return
        if last_error is not None:
            self.status = str(last_error)

    def save(self):
        self.sync_config_from_trackbars()
        save_config(self.config_path, self.config)
        self.config = load_config(self.config_path)
        self.status = "saved " + str(self.config_path)

    def reload(self):
        self.config = load_config(self.config_path)
        self.selected = ("front", 0)
        self.push_trackbars()
        self.status = "reloaded"

    def on_mouse(self, event, x, y, _flags, _param):
        if self.frame_w <= 0 or y >= self.frame_h:
            return
        if event in (cv2.EVENT_LBUTTONDOWN, cv2.EVENT_RBUTTONDOWN):
            hit = self.hit(x, y)
            if hit is None:
                return
            self.selected = hit
            region = self.selected_region()
            if region is None:
                return
            self.drag = "resize" if event == cv2.EVENT_RBUTTONDOWN else "move"
            self.press = (x, y)
            self.snapshot = dict(region)
            return
        if event in (cv2.EVENT_LBUTTONUP, cv2.EVENT_RBUTTONUP):
            self.drag = None
            return
        if event != cv2.EVENT_MOUSEMOVE or self.drag is None or self.snapshot is None:
            return
        region = self.selected_region()
        if region is None:
            return
        dx = (x - self.press[0]) / float(self.frame_w)
        dy = (y - self.press[1]) / float(self.frame_h)
        if self.drag == "move":
            region["x"] = clamp(self.snapshot["x"] + dx, 0.0, 1.0 - self.snapshot["w"])
            region["y"] = clamp(self.snapshot["y"] + dy, 0.0, 1.0 - self.snapshot["h"])
        else:
            region["w"] = clamp(self.snapshot["w"] + dx, 0.03, 1.0 - self.snapshot["x"])
            region["h"] = clamp(self.snapshot["h"] + dy, 0.03, 1.0 - self.snapshot["y"])

    def hit(self, x, y):
        if point_in_rect(x, y, self.config["front"], self.frame_w, self.frame_h):
            return ("front", 0)
        if point_in_rect(x, y, self.config["storage"], self.frame_w, self.frame_h):
            return ("storage", 0)
        return None

    def selected_region(self):
        if self.selected is None:
            return None
        kind, _index = self.selected
        if kind == "storage":
            return self.config["storage"]
        return self.config["front"]

    def draw(self, frame, result):
        if self.show_mask and result["mask"] is not None:
            view = cv2.cvtColor(result["mask"], cv2.COLOR_GRAY2BGR)
        else:
            view = frame.copy()
        self.draw_zone(view, result["storage"], (0, 180, 0), "storage")
        self.draw_zone(view, result["front"], (0, 220, 255), "front")
        return view

    def draw_zone(self, view, zone, color, kind):
        rect = zone.get("rect")
        if rect is None or rect[2] < 1 or rect[3] < 1:
            return
        x, y, w, h = rect
        balls = zone["balls"]
        box = color if balls else (140, 140, 140)
        cv2.rectangle(view, (x, y), (x + w, y + h), box, 3 if self.is_selected(kind, 0) else 2)
        for ball in balls:
            center = ball["center"]
            cv2.circle(
                view,
                (int(round(center[0])), int(round(center[1]))),
                max(1, int(round(ball["radius"]))),
                color,
                2,
            )
        label = "{0} {1}".format(zone["name"], len(balls))
        cv2.putText(
            view,
            label,
            (x + 4, min(y + h - 6, y + 16)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 0, 0),
            3,
            cv2.LINE_AA,
        )
        cv2.putText(
            view,
            label,
            (x + 4, min(y + h - 6, y + 16)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )

    def is_selected(self, kind, index):
        return self.selected == (kind, index)

    def compose(self, view, result):
        selected = "none"
        region = self.selected_region()
        if region is not None:
            selected = region["name"]
        lines = [
            "front {0}   storage {1}   selected {2}".format(
                result["front_count"], result["storage_count"], selected
            ),
            self.status,
            "drag move   right-drag resize",
            "c camera   m mask   s save   r reload   space pause   q quit",
        ]
        panel = np.zeros((76, view.shape[1], 3), dtype=np.uint8)
        for index, line in enumerate(lines):
            cv2.putText(
                panel,
                line,
                (8, 16 + (index * 18)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.42,
                (230, 230, 230),
                1,
                cv2.LINE_AA,
            )
        return np.vstack([view, panel])

    def create_trackbars(self):
        cfg = self.config
        trackbars = (
            ("H low", cfg["hsv_low"][0], 179),
            ("H high", cfg["hsv_high"][0], 179),
            ("S low", cfg["hsv_low"][1], 255),
            ("S high", cfg["hsv_high"][1], 255),
            ("V low", cfg["hsv_low"][2], 255),
            ("V high", cfg["hsv_high"][2], 255),
            ("Min area", area_step(cfg["min_area_fraction"]), 100),
            ("Circularity pct", percent(cfg["min_circularity"]), 100),
            ("Blur", cfg["blur_size"], 31),
            ("Close", cfg["close_size"], 31),
        )
        for name, value, limit in trackbars:
            cv2.createTrackbar(name, WINDOW, int(value), limit, lambda _value: None)

    def sync_config_from_trackbars(self):
        cfg = self.config
        cfg["hsv_low"] = [
            cv2.getTrackbarPos("H low", WINDOW),
            cv2.getTrackbarPos("S low", WINDOW),
            cv2.getTrackbarPos("V low", WINDOW),
        ]
        cfg["hsv_high"] = [
            cv2.getTrackbarPos("H high", WINDOW),
            cv2.getTrackbarPos("S high", WINDOW),
            cv2.getTrackbarPos("V high", WINDOW),
        ]
        cfg["min_area_fraction"] = cv2.getTrackbarPos("Min area", WINDOW) / 1000.0
        cfg["min_circularity"] = cv2.getTrackbarPos("Circularity pct", WINDOW) / 100.0
        cfg["blur_size"] = cv2.getTrackbarPos("Blur", WINDOW)
        cfg["close_size"] = cv2.getTrackbarPos("Close", WINDOW)

    def push_trackbars(self):
        cfg = self.config
        cv2.setTrackbarPos("H low", WINDOW, cfg["hsv_low"][0])
        cv2.setTrackbarPos("H high", WINDOW, cfg["hsv_high"][0])
        cv2.setTrackbarPos("S low", WINDOW, cfg["hsv_low"][1])
        cv2.setTrackbarPos("S high", WINDOW, cfg["hsv_high"][1])
        cv2.setTrackbarPos("V low", WINDOW, cfg["hsv_low"][2])
        cv2.setTrackbarPos("V high", WINDOW, cfg["hsv_high"][2])
        cv2.setTrackbarPos("Min area", WINDOW, area_step(cfg["min_area_fraction"]))
        cv2.setTrackbarPos("Circularity pct", WINDOW, percent(cfg["min_circularity"]))
        cv2.setTrackbarPos("Blur", WINDOW, cfg["blur_size"])
        cv2.setTrackbarPos("Close", WINDOW, cfg["close_size"])


def point_in_rect(x, y, region, width, height):
    rx, ry, rw, rh = region_rect(region, width, height)
    return rx <= x < rx + rw and ry <= y < ry + rh


def clamp(value, low, high):
    return max(low, min(high, value))


def fit_frame(frame, max_side):
    """Scale down evenly. A 9:16 frame stays 9:16."""
    height, width = frame.shape[:2]
    long_side = max(height, width)
    if long_side <= max_side or long_side < 2:
        return frame
    scale = max_side / float(long_side)
    new_width = max(1, int(round(width * scale)))
    new_height = max(1, int(round(height * scale)))
    return cv2.resize(frame, (new_width, new_height), interpolation=cv2.INTER_AREA)


def percent(fraction):
    return int(round(float(fraction) * 100.0))


def area_step(fraction):
    """Trackbar units are 0.1 percent of the frame. 4 means 0.4 percent."""
    return int(round(float(fraction) * 1000.0))


def main(argv):
    parser = argparse.ArgumentParser(description="Tune the pollen detector on this computer.")
    parser.add_argument(
        "--camera",
        type=int,
        default=None,
        help="Webcam index. Omit this to use the ELP instead of the laptop webcam.",
    )
    parser.add_argument("--image", help="Tune against one saved frame instead of a camera.")
    parser.add_argument("--video", help="Tune against a video file, looping at the end.")
    parser.add_argument("--config", default=str(ASSET_CONFIG), help="JSON file shared with the robot.")
    args = parser.parse_args(argv)
    config_path = args.config
    config = load_config(config_path)
    print("Config: " + config_path)
    source, cameras = build_source(args)
    for camera in cameras:
        marker = "  <-- using" if camera["index"] == getattr(source, "index", None) else ""
        print("  {0}: {1}{2}".format(camera["index"], camera["name"], marker))
    print("Press c to switch cameras. Press s to save for the robot.")
    Proxy(config, config_path, source, cameras).run()


if __name__ == "__main__":
    main(sys.argv[1:])
