package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;

/**
 * Runs PollenDetector on the ELP preview. Boxes are drawn on the frame
 * so the Driver Station camera stream shows them.
 */
public class PollenVisionProcessor implements VisionProcessor {
    private final PollenDetector detector;
    private volatile PollenVisionResult result = PollenVisionResult.empty();
    private volatile String error;
    private volatile boolean showMask;
    private volatile boolean sawFrame;

    public PollenVisionProcessor(PollenVisionConfig config) {
        detector = new PollenDetector(config);
    }

    public void setShowMask(boolean showMask) {
        this.showMask = showMask;
    }

    public boolean getShowMask() {
        return showMask;
    }

    public PollenVisionResult getResult() {
        return result;
    }

    public String getError() {
        return error;
    }

    /**
     * True after at least one frame has been counted and the latest frame did not throw.
     * An empty count is a real reading. A missing camera is not.
     */
    public boolean hasReading() {
        return sawFrame && error == null;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        try {
            result = detector.process(frame, showMask);
            error = null;
            sawFrame = true;
        } catch (RuntimeException exception) {
            error = exception.getClass().getSimpleName() + ": " + exception.getMessage();
        }
        return null;
    }

    @Override
    public void onDrawFrame(
            Canvas canvas,
            int onscreenWidth,
            int onscreenHeight,
            float scaleBmpPxToCanvasPx,
            float scaleCanvasDensity,
            Object userContext) {
    }

    public void close() {
        detector.close();
    }
}
