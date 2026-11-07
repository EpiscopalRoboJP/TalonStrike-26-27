package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

/**
 * Yellow mask, then every round blob in the frame.
 * A blob in the front rectangle counts as front. Any other blob in the
 * storage rectangle counts as storage. Keep the steps in step with
 * vision/pollen_detect.py. The robot frame is RGB.
 */
public final class PollenDetector {
    private final PollenVisionConfig config;
    private final Scalar hsvLower;
    private final Scalar hsvUpper;
    private final Mat hsv = new Mat();
    private final Mat rgb = new Mat();
    private final Mat mask = new Mat();
    private final Mat hierarchy = new Mat();
    private final Mat display = new Mat();
    private Mat closeKernel;
    private int frameChannels = 3;

    public PollenDetector(PollenVisionConfig config) {
        this.config = config;
        this.hsvLower = new Scalar(
                Math.min(config.hsvLow[0], config.hsvHigh[0]),
                Math.min(config.hsvLow[1], config.hsvHigh[1]),
                Math.min(config.hsvLow[2], config.hsvHigh[2]));
        this.hsvUpper = new Scalar(
                Math.max(config.hsvLow[0], config.hsvHigh[0]),
                Math.max(config.hsvLow[1], config.hsvHigh[1]),
                Math.max(config.hsvLow[2], config.hsvHigh[2]));
        if (config.closeSize >= 2) {
            closeKernel = Imgproc.getStructuringElement(
                    Imgproc.MORPH_ELLIPSE,
                    new Size(config.closeSize, config.closeSize));
        }
    }

    public PollenVisionResult process(Mat frame, boolean showMask) {
        if (frame.empty() || frame.cols() < 2 || frame.rows() < 2) {
            return PollenVisionResult.empty();
        }
        frameChannels = frame.channels();
        Mat source = frame;
        if (frameChannels == 4) {
            Imgproc.cvtColor(frame, rgb, Imgproc.COLOR_RGBA2RGB);
            source = rgb;
        }
        Imgproc.cvtColor(source, hsv, Imgproc.COLOR_RGB2HSV);
        int blur = config.blurSize;
        if (blur >= 3) {
            if (blur % 2 == 0) {
                blur += 1;
            }
            Imgproc.GaussianBlur(hsv, hsv, new Size(blur, blur), 0);
        }
        Core.inRange(hsv, hsvLower, hsvUpper, mask);
        if (closeKernel != null) {
            Imgproc.dilate(mask, mask, closeKernel);
            Imgproc.erode(mask, mask, closeKernel);
        }

        int width = frame.cols();
        int height = frame.rows();
        Rect frontRect = config.front.toRect(width, height);
        Rect storageRect = config.storage.toRect(width, height);
        List<PollenVisionResult.Ball> frontBalls = new ArrayList<PollenVisionResult.Ball>();
        List<PollenVisionResult.Ball> storageBalls = new ArrayList<PollenVisionResult.Ball>();
        for (PollenVisionResult.Ball ball : findBalls(width, height)) {
            if (contains(frontRect, ball.centerX, ball.centerY)) {
                frontBalls.add(ball);
            } else if (contains(storageRect, ball.centerX, ball.centerY)) {
                storageBalls.add(ball);
            }
        }
        PollenVisionResult result = new PollenVisionResult(
                new PollenVisionResult.Zone(config.front.name, frontRect, frontBalls),
                new PollenVisionResult.Zone(config.storage.name, storageRect, storageBalls),
                width,
                height);
        if (showMask) {
            paintMask(frame);
        }
        draw(frame, result);
        return result;
    }

    public void close() {
        hsv.release();
        rgb.release();
        mask.release();
        hierarchy.release();
        display.release();
        if (closeKernel != null) {
            closeKernel.release();
            closeKernel = null;
        }
    }

    private List<PollenVisionResult.Ball> findBalls(int width, int height) {
        Mat contourSource = mask.clone();
        List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
        List<PollenVisionResult.Ball> balls = new ArrayList<PollenVisionResult.Ball>();
        try {
            Imgproc.findContours(
                    contourSource, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
            double frameArea = (double) width * (double) height;
            double minArea = config.minAreaFraction * frameArea;
            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area < 1 || area < minArea || frameArea <= 0) {
                    continue;
                }
                MatOfPoint2f curve = new MatOfPoint2f(contour.toArray());
                double perimeter = Imgproc.arcLength(curve, true);
                Point center = new Point();
                float[] radius = new float[1];
                Imgproc.minEnclosingCircle(curve, center, radius);
                curve.release();
                double circularity = 0;
                if (perimeter > 1e-6) {
                    circularity = (4.0 * Math.PI * area) / (perimeter * perimeter);
                }
                if (circularity > 1) {
                    circularity = 1;
                }
                if (circularity < config.minCircularity) {
                    continue;
                }
                balls.add(new PollenVisionResult.Ball(
                        circularity,
                        area / frameArea,
                        center.x,
                        center.y,
                        radius[0]));
            }
            return balls;
        } finally {
            contourSource.release();
            for (MatOfPoint contour : contours) {
                contour.release();
            }
        }
    }

    private boolean contains(Rect rect, double x, double y) {
        return x >= rect.x && x < rect.x + rect.width && y >= rect.y && y < rect.y + rect.height;
    }

    private void paintMask(Mat frame) {
        Imgproc.cvtColor(mask, display, Imgproc.COLOR_GRAY2RGB);
        if (frameChannels == 4) {
            Imgproc.cvtColor(display, frame, Imgproc.COLOR_RGB2RGBA);
        } else {
            display.copyTo(frame);
        }
    }

    private void draw(Mat frame, PollenVisionResult result) {
        drawZone(frame, result.storage, rgb(0, 180, 0));
        drawZone(frame, result.front, rgb(255, 220, 0));
        String summary = "front " + result.frontCount + "  storage " + result.storageCount;
        outlineText(frame, summary, 8, Math.max(16, frame.rows() - 10));
    }

    private void drawZone(Mat frame, PollenVisionResult.Zone zone, Scalar color) {
        Rect rect = zone.rect;
        if (rect.width < 1 || rect.height < 1) {
            return;
        }
        Scalar box = zone.balls.isEmpty() ? rgb(140, 140, 140) : color;
        Imgproc.rectangle(
                frame,
                new Point(rect.x, rect.y),
                new Point(rect.x + rect.width, rect.y + rect.height),
                box,
                2);
        for (PollenVisionResult.Ball ball : zone.balls) {
            if (ball.radius > 1) {
                Imgproc.circle(
                        frame,
                        new Point(ball.centerX, ball.centerY),
                        (int) Math.round(ball.radius),
                        color,
                        2);
            }
        }
        outlineText(frame, zone.name + " " + zone.balls.size(), rect.x + 4, textBaseline(rect));
    }

    private int textBaseline(Rect rect) {
        int baseline = rect.y + 16;
        int limit = rect.y + rect.height - 4;
        if (baseline > limit) {
            return Math.max(rect.y + 12, limit);
        }
        return baseline;
    }

    private void outlineText(Mat frame, String text, int x, int y) {
        Imgproc.putText(frame, text, new Point(x, y), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, rgb(0, 0, 0), 3);
        Imgproc.putText(frame, text, new Point(x, y), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, rgb(255, 255, 255), 1);
    }

    private Scalar rgb(int r, int g, int b) {
        if (frameChannels == 4) {
            return new Scalar(r, g, b, 255);
        }
        return new Scalar(r, g, b);
    }
}
