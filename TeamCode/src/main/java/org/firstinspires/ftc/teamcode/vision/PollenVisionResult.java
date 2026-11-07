package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Rect;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public final class PollenVisionResult {
    public final int frontCount;
    public final int storageCount;
    public final Zone front;
    public final Zone storage;
    public final int width;
    public final int height;

    public PollenVisionResult(Zone front, Zone storage, int width, int height) {
        this.front = front;
        this.storage = storage;
        this.width = width;
        this.height = height;
        this.frontCount = front.balls.size();
        this.storageCount = storage.balls.size();
    }

    public static PollenVisionResult empty() {
        return new PollenVisionResult(Zone.empty("front"), Zone.empty("storage"), 0, 0);
    }

    public static final class Zone {
        public final String name;
        public final Rect rect;
        public final List<Ball> balls;

        public Zone(String name, Rect rect, List<Ball> balls) {
            this.name = name;
            this.rect = rect;
            this.balls = Collections.unmodifiableList(new ArrayList<Ball>(balls));
        }

        public static Zone empty(String name) {
            return new Zone(name, new Rect(), Collections.<Ball>emptyList());
        }
    }

    public static final class Ball {
        public final double circularity;
        public final double areaFraction;
        public final double centerX;
        public final double centerY;
        public final double radius;

        public Ball(
                double circularity,
                double areaFraction,
                double centerX,
                double centerY,
                double radius) {
            this.circularity = circularity;
            this.areaFraction = areaFraction;
            this.centerX = centerX;
            this.centerY = centerY;
            this.radius = radius;
        }
    }
}
