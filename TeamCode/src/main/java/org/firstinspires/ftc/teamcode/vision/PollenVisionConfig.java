package org.firstinspires.ftc.teamcode.vision;

import android.content.Context;

import org.json.JSONException;
import org.json.JSONObject;
import org.opencv.core.Rect;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;

/**
 * Rectangles and yellow range shared with vision/proxy.py.
 * The asset file is the copy the laptop proxy saves.
 */
public final class PollenVisionConfig {
    public static final String ASSET_NAME = "pollen_vision.json";

    public final int width;
    public final int height;
    public final double[] hsvLow;
    public final double[] hsvHigh;
    public final int blurSize;
    public final int closeSize;
    public final double minAreaFraction;
    public final double minCircularity;
    public final Region front;
    public final Region storage;

    public PollenVisionConfig(
            int width,
            int height,
            double[] hsvLow,
            double[] hsvHigh,
            int blurSize,
            int closeSize,
            double minAreaFraction,
            double minCircularity,
            Region front,
            Region storage) {
        this.width = width;
        this.height = height;
        this.hsvLow = hsvLow;
        this.hsvHigh = hsvHigh;
        this.blurSize = blurSize;
        this.closeSize = closeSize;
        this.minAreaFraction = minAreaFraction;
        this.minCircularity = minCircularity;
        this.front = front;
        this.storage = storage;
    }

    public static PollenVisionConfig load(Context context) throws IOException, JSONException {
        InputStream input = context.getAssets().open(ASSET_NAME);
        try {
            return parse(readAll(input));
        } finally {
            input.close();
        }
    }

    public static PollenVisionConfig parse(String json) throws JSONException {
        return fromObject(new JSONObject(json));
    }

    /**
     * Same starting numbers as pollen_vision.json and vision/pollen_detect.py.
     * Used only when the asset cannot be read.
     */
    public static PollenVisionConfig defaults() {
        return new PollenVisionConfig(
                640,
                480,
                new double[]{15, 15, 180},
                new double[]{40, 255, 255},
                0,
                21,
                0.003,
                0.38,
                new Region("front", 0.18, 0.00, 0.45, 0.08),
                new Region("storage", 0.00, 0.08, 1.00, 0.92));
    }

    private static PollenVisionConfig fromObject(JSONObject root) throws JSONException {
        PollenVisionConfig seed = defaults();
        Region front = root.has("front")
                ? readRegion(root.getJSONObject("front"), "front")
                : seed.front;
        Region storage = root.has("storage")
                ? readRegion(root.getJSONObject("storage"), "storage")
                : seed.storage;
        return new PollenVisionConfig(
                clamp(readInt(root, "width", seed.width), 160, 1920),
                clamp(readInt(root, "height", seed.height), 120, 1920),
                readHsv(root.optJSONArray("hsv_low"), seed.hsvLow),
                readHsv(root.optJSONArray("hsv_high"), seed.hsvHigh),
                clamp(readInt(root, "blur_size", seed.blurSize), 0, 31),
                clamp(readInt(root, "close_size", seed.closeSize), 0, 31),
                readMinArea(root, front, seed.minAreaFraction),
                clamp(readDouble(
                        root,
                        root.has("min_circularity") ? "min_circularity" : "front_min_circularity",
                        seed.minCircularity), 0, 1),
                front,
                storage);
    }

    /**
     * Older files stored the cutoff as a fraction of the front box.
     */
    private static double readMinArea(JSONObject root, Region front, double fallback) {
        if (root.has("min_area_fraction")) {
            return clamp(readDouble(root, "min_area_fraction", fallback), 0, 1);
        }
        if (!root.has("front_min_area_fraction")) {
            return fallback;
        }
        return clamp(readDouble(root, "front_min_area_fraction", fallback) * front.w * front.h, 0, 1);
    }

    private static Region readRegion(JSONObject obj, String fallbackName) throws JSONException {
        String name = obj.optString("name", fallbackName);
        if (name == null || name.length() == 0) {
            name = fallbackName;
        }
        double x = clamp(obj.optDouble("x", 0), 0, 1);
        double y = clamp(obj.optDouble("y", 0), 0, 1);
        double w = clamp(obj.optDouble("w", 0.2), 0.03, 1);
        double h = clamp(obj.optDouble("h", 0.2), 0.03, 1);
        if (x + w > 1) {
            x = Math.max(0, 1 - w);
        }
        if (y + h > 1) {
            y = Math.max(0, 1 - h);
        }
        return new Region(name, x, y, w, h);
    }

    private static double[] readHsv(org.json.JSONArray array, double[] fallback) throws JSONException {
        double[] limits = new double[]{179, 255, 255};
        double[] values = new double[3];
        for (int i = 0; i < 3; i++) {
            double raw = fallback[i];
            if (array != null && array.length() > i) {
                raw = array.getDouble(i);
            }
            values[i] = clamp(raw, 0, limits[i]);
        }
        return values;
    }

    private static int readInt(JSONObject obj, String key, int fallback) {
        if (!obj.has(key)) {
            return fallback;
        }
        return (int) Math.round(obj.optDouble(key, fallback));
    }

    private static double readDouble(JSONObject obj, String key, double fallback) {
        if (!obj.has(key)) {
            return fallback;
        }
        return obj.optDouble(key, fallback);
    }

    private static String readAll(InputStream input) throws IOException {
        ByteArrayOutputStream buffer = new ByteArrayOutputStream();
        byte[] chunk = new byte[4096];
        int count;
        while ((count = input.read(chunk)) != -1) {
            buffer.write(chunk, 0, count);
        }
        return new String(buffer.toByteArray(), StandardCharsets.UTF_8);
    }

    private static int clamp(int value, int low, int high) {
        return Math.max(low, Math.min(high, value));
    }

    private static double clamp(double value, double low, double high) {
        return Math.max(low, Math.min(high, value));
    }

    public static final class Region {
        public final String name;
        public final double x;
        public final double y;
        public final double w;
        public final double h;

        public Region(String name, double x, double y, double w, double h) {
            this.name = name;
            this.x = x;
            this.y = y;
            this.w = w;
            this.h = h;
        }

        /**
         * Half-up rounding, matching vision/pollen_detect.py region_rect.
         */
        public Rect toRect(int imageWidth, int imageHeight) {
            int left = clamp(roundHalfUp(x * imageWidth), 0, Math.max(0, imageWidth - 1));
            int top = clamp(roundHalfUp(y * imageHeight), 0, Math.max(0, imageHeight - 1));
            int right = clamp(roundHalfUp((x + w) * imageWidth), left + 1, Math.max(left + 1, imageWidth));
            int bottom = clamp(roundHalfUp((y + h) * imageHeight), top + 1, Math.max(top + 1, imageHeight));
            return new Rect(left, top, right - left, bottom - top);
        }

        private static int roundHalfUp(double value) {
            return (int) Math.round(value);
        }
    }
}
