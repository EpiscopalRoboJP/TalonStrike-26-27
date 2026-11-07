package org.firstinspires.ftc.teamcode.vision;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

/**
 * Downward pollen camera used by teleop. Missing from the robot configuration
 * is normal: openIfConfigured returns null and the caller leaves ball-count
 * lights off.
 */
public final class PollenCamera {
    public static final String WEBCAM_NAME = "Webcam 1";

    private final VisionPortal portal;
    private final PollenVisionProcessor processor;

    private PollenCamera(VisionPortal portal, PollenVisionProcessor processor) {
        this.portal = portal;
        this.processor = processor;
    }

    public static PollenCamera openIfConfigured(HardwareMap hardwareMap) {
        WebcamName webcam;
        try {
            webcam = hardwareMap.get(WebcamName.class, WEBCAM_NAME);
        } catch (RuntimeException exception) {
            return null;
        }

        PollenVisionConfig config = loadConfig(hardwareMap);
        PollenVisionProcessor processor = new PollenVisionProcessor(config);
        try {
            VisionPortal portal = new VisionPortal.Builder()
                    .addProcessor(processor)
                    .setCamera(webcam)
                    .setCameraResolution(new Size(config.width, config.height))
                    .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                    .build();
            return new PollenCamera(portal, processor);
        } catch (RuntimeException exception) {
            processor.close();
            return null;
        }
    }

    public boolean hasReading() {
        return processor.hasReading();
    }

    public PollenVisionResult result() {
        return processor.getResult();
    }

    public void close() {
        portal.close();
        processor.close();
    }

    private static PollenVisionConfig loadConfig(HardwareMap hardwareMap) {
        try {
            return PollenVisionConfig.load(hardwareMap.appContext);
        } catch (Exception exception) {
            return PollenVisionConfig.defaults();
        }
    }
}
