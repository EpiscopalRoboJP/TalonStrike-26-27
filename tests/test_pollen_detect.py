"""Synthetic frames for the pollen detector. No camera required."""

import math
import sys
import unittest
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from vision.frame_source import choose_camera
from vision.pollen_detect import ASSET_CONFIG, default_config, detect, load_config, normalize, region_rect
from vision.proxy import fit_frame

WIDTH = 640
HEIGHT = 480
YELLOW = (0, 255, 255)
BLUE = (255, 0, 0)


class PollenDetectTest(unittest.TestCase):
    def test_region_rect_matches_robot_rounding(self):
        config = default_config()
        self.assertEqual(region_rect(config["front"], WIDTH, HEIGHT), (115, 0, 288, 38))
        self.assertEqual(region_rect(config["storage"], WIDTH, HEIGHT), (0, 38, 640, 442))

    def test_front_and_storage_balls(self):
        config = default_config()
        image = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
        self.paint_circle(image, config["front"], YELLOW, 0.50)
        self.paint_circle(image, {"x": 0.20, "y": 0.30, "w": 0.25, "h": 0.25}, YELLOW, 0.35)
        self.paint_circle(image, {"x": 0.60, "y": 0.55, "w": 0.25, "h": 0.25}, YELLOW, 0.35)
        self.paint_circle(image, {"x": 0.40, "y": 0.70, "w": 0.20, "h": 0.20}, BLUE, 0.35)

        result = detect(image, config)

        self.assertEqual(result["front_count"], 1)
        self.assertEqual(result["storage_count"], 2)
        self.assertGreater(result["front"]["balls"][0]["circularity"], 0.6)

    def test_wide_rectangle_is_not_a_ball(self):
        config = default_config()
        image = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
        self.paint_rect(image, config["front"], YELLOW)
        result = detect(image, config)
        self.assertEqual(result["front_count"], 0)
        self.assertEqual(result["storage_count"], 0)

    def test_small_speck_is_ignored(self):
        config = default_config()
        image = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
        cv2.circle(image, (200, 240), 4, YELLOW, -1, lineType=cv2.LINE_8)
        result = detect(image, config)
        self.assertEqual(result["front_count"], 0)
        self.assertEqual(result["storage_count"], 0)

    def test_overlap_counts_as_front(self):
        config = default_config()
        config["storage"] = {"name": "storage", "x": 0.0, "y": 0.0, "w": 1.0, "h": 1.0}
        image = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
        self.paint_circle(image, config["front"], YELLOW, 0.50)
        result = detect(image, config)
        self.assertEqual(result["front_count"], 1)
        self.assertEqual(result["storage_count"], 0)

    def test_ball_outside_both_areas_is_ignored(self):
        config = default_config()
        config["storage"] = {"name": "storage", "x": 0.0, "y": 0.50, "w": 0.40, "h": 0.40}
        image = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
        self.paint_circle(image, {"x": 0.70, "y": 0.20, "w": 0.20, "h": 0.20}, YELLOW, 0.35)
        result = detect(image, config)
        self.assertEqual(result["front_count"], 0)
        self.assertEqual(result["storage_count"], 0)

    def test_reversed_hue_bounds_still_match(self):
        config = default_config()
        config["hsv_low"] = [45, 255, 255]
        config["hsv_high"] = [15, 70, 70]
        image = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
        self.paint_circle(image, {"x": 0.30, "y": 0.40, "w": 0.25, "h": 0.25}, YELLOW, 0.35)
        result = detect(image, config)
        self.assertEqual(result["storage_count"], 1)

    def test_old_front_area_setting_keeps_the_same_ball_size(self):
        config = normalize({
            "front_min_area_fraction": 0.04,
            "front_min_circularity": 0.45,
            "front": {"name": "front", "x": 0.20, "y": 0.00, "w": 0.60, "h": 0.16},
        })
        self.assertAlmostEqual(config["min_area_fraction"], 0.04 * 0.60 * 0.16)
        self.assertAlmostEqual(config["min_circularity"], 0.45)

    def test_empty_frame_counts_nothing(self):
        result = detect(np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8), default_config())
        self.assertEqual(result["front_count"], 0)
        self.assertEqual(result["storage_count"], 0)

    def test_preview_keeps_aspect_ratio(self):
        tall = np.zeros((1280, 720, 3), dtype=np.uint8)
        fitted = fit_frame(tall, 960)
        self.assertEqual(fitted.shape[0], 960)
        self.assertEqual(fitted.shape[1], 540)

    def test_choose_camera_skips_the_laptop_webcam(self):
        cameras = [
            {"index": 0, "name": "Integrated Camera", "backend": 0},
            {"index": 1, "name": "USB2.0 HD UVC WebCam", "backend": 700},
        ]
        chosen = choose_camera(cameras, None)
        self.assertEqual(chosen["index"], 1)
        elp = choose_camera(
            [
                {"index": 0, "name": "Integrated Camera", "backend": 0},
                {"index": 2, "name": "ELP USB Camera", "backend": 700},
            ],
            None,
        )
        self.assertEqual(elp["index"], 2)
        forced = choose_camera(cameras, 0)
        self.assertEqual(forced["name"], "Integrated Camera")
        laptop_only = choose_camera(
            [
                {"index": 0, "name": "Integrated Camera", "backend": 700},
                {"index": 2, "name": "Lenovo Virtual Camera", "backend": 700},
            ],
            None,
        )
        self.assertEqual(laptop_only["name"], "Integrated Camera")

    def test_asset_file_loads(self):
        config = load_config(ASSET_CONFIG)
        self.assertEqual(config["width"], 640)
        self.assertEqual(config["height"], 480)
        self.assertAlmostEqual(config["min_area_fraction"], 0.003)
        self.assertAlmostEqual(config["min_circularity"], 0.38)
        for key in ("name", "x", "y", "w", "h"):
            self.assertIn(key, config["front"])
            self.assertIn(key, config["storage"])
        self.assertEqual(len(config["hsv_low"]), 3)
        self.assertEqual(len(config["hsv_high"]), 3)

    def paint_rect(self, image, region, color):
        x, y, w, h = region_rect(region, WIDTH, HEIGHT)
        image[y:y + h, x:x + w] = color

    def paint_circle(self, image, region, color, scale):
        x, y, w, h = region_rect(region, WIDTH, HEIGHT)
        radius = int(min(w, h) * scale)
        self.assertGreater(radius, 8)
        area = math.pi * radius * radius
        self.assertGreater(area / float(WIDTH * HEIGHT), default_config()["min_area_fraction"])
        cv2.circle(image, (x + (w // 2), y + (h // 2)), radius, color, -1, lineType=cv2.LINE_8)


if __name__ == "__main__":
    unittest.main()
