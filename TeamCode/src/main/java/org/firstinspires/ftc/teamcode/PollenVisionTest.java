package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;

import org.firstinspires.ftc.teamcode.vision.PollenVisionConfig;
import org.firstinspires.ftc.teamcode.vision.PollenVisionProcessor;
import org.firstinspires.ftc.teamcode.vision.PollenVisionResult;

import java.util.concurrent.TimeUnit;

/**
 * Downward ELP view. Round yellow balls in the front box are counted apart
 * from round yellow balls in the storage box.
 * Configure the camera as a webcam named "Webcam 1".
 * Open the Driver Station camera stream to see the boxes.
 *
 * Tune rectangles and the yellow range on a laptop with vision/proxy.py.
 * That saves TeamCode/src/main/assets/pollen_vision.json, which this op mode reads
 * the next time the app is installed. Stream format is MJPEG. If the preview
 * never opens, switch STREAM_FORMAT to YUY2.
 */
@TeleOp(name = "Pollen Vision Test", group = "TeamCode")
public class PollenVisionTest extends LinearOpMode {
    private static final String WEBCAM_NAME = "Webcam 1";
    private static final VisionPortal.StreamFormat STREAM_FORMAT = VisionPortal.StreamFormat.MJPEG;

    private VisionPortal portal;
    private PollenVisionProcessor processor;
    private String configWarning;
    private String controlWarning;

    private boolean manualExposure;
    private boolean cameraControlsReady;
    private boolean cameraControlsSupported;
    private int minExposure = 1;
    private int maxExposure = 1;
    private int myExposure = 1;
    private int minGain;
    private int maxGain;
    private int myGain;

    private boolean lastManual;
    private boolean lastMask;
    private boolean lastExposureUp;
    private boolean lastExposureDown;
    private boolean lastGainUp;
    private boolean lastGainDown;

    @Override
    public void runOpMode() {
        PollenVisionConfig config = loadConfig();
        try {
            if (!buildPortal(config)) {
                return;
            }
            while (opModeIsActive() || opModeInInit()) {
                if (!cameraControlsReady) {
                    cameraControlsReady = readCameraLimits();
                }
                updateExposure();
                updateTelemetry();
                sleep(20);
            }
        } finally {
            if (portal != null) {
                portal.close();
            }
            if (processor != null) {
                processor.close();
            }
        }
    }

    private PollenVisionConfig loadConfig() {
        try {
            return PollenVisionConfig.load(hardwareMap.appContext);
        } catch (Exception exception) {
            configWarning = exception.getMessage();
            return PollenVisionConfig.defaults();
        }
    }

    private boolean buildPortal(PollenVisionConfig config) {
        WebcamName webcam;
        try {
            webcam = hardwareMap.get(WebcamName.class, WEBCAM_NAME);
        } catch (Exception exception) {
            while (!isStopRequested() && opModeInInit()) {
                telemetry.addLine("Name the ELP webcam \"Webcam 1\" in the robot configuration.");
                telemetry.update();
                sleep(50);
            }
            return false;
        }

        processor = new PollenVisionProcessor(config);
        portal = new VisionPortal.Builder()
                .addProcessor(processor)
                .setCamera(webcam)
                .setCameraResolution(new Size(config.width, config.height))
                .setStreamFormat(STREAM_FORMAT)
                .build();
        return true;
    }

    private boolean readCameraLimits() {
        if (portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            return false;
        }
        ExposureControl exposure = portal.getCameraControl(ExposureControl.class);
        GainControl gain = portal.getCameraControl(GainControl.class);
        if (exposure == null || gain == null) {
            cameraControlsSupported = false;
            return true;
        }
        int reportedMin = (int) exposure.getMinExposure(TimeUnit.MILLISECONDS);
        minExposure = Math.max(reportedMin, 1);
        maxExposure = Math.max((int) exposure.getMaxExposure(TimeUnit.MILLISECONDS), minExposure);
        minGain = gain.getMinGain();
        maxGain = Math.max(gain.getMaxGain(), minGain);
        myExposure = minExposure;
        myGain = minGain;
        cameraControlsSupported = true;
        return true;
    }

    private void updateExposure() {
        if (!cameraControlsReady || !cameraControlsSupported) {
            return;
        }
        try {
            updateExposureControls();
        } catch (RuntimeException exception) {
            manualExposure = false;
            controlWarning = exception.getMessage();
        }
    }

    private void updateExposureControls() {
        boolean manualPressed = gamepad1.a;
        if (manualPressed && !lastManual) {
            manualExposure = !manualExposure;
            if (manualExposure) {
                captureCurrentExposure();
                applyManualExposure();
            } else {
                setAutoExposure();
            }
        }
        lastManual = manualPressed;
        if (!manualExposure) {
            return;
        }

        boolean exposureUp = gamepad1.dpad_up;
        boolean exposureDown = gamepad1.dpad_down;
        boolean gainUp = gamepad1.dpad_right;
        boolean gainDown = gamepad1.dpad_left;
        boolean changed = false;
        if (exposureUp && !lastExposureUp) {
            myExposure = Math.min(maxExposure, myExposure + 1);
            changed = true;
        } else if (exposureDown && !lastExposureDown) {
            myExposure = Math.max(minExposure, myExposure - 1);
            changed = true;
        }
        if (gainUp && !lastGainUp) {
            myGain = Math.min(maxGain, myGain + 1);
            changed = true;
        } else if (gainDown && !lastGainDown) {
            myGain = Math.max(minGain, myGain - 1);
            changed = true;
        }
        lastExposureUp = exposureUp;
        lastExposureDown = exposureDown;
        lastGainUp = gainUp;
        lastGainDown = gainDown;
        if (changed) {
            applyManualExposure();
        }
    }

    private void captureCurrentExposure() {
        ExposureControl exposure = portal.getCameraControl(ExposureControl.class);
        GainControl gain = portal.getCameraControl(GainControl.class);
        if (exposure == null || gain == null) {
            return;
        }
        int currentExposure = (int) exposure.getExposure(TimeUnit.MILLISECONDS);
        int currentGain = gain.getGain();
        if (currentExposure >= minExposure && currentExposure <= maxExposure) {
            myExposure = currentExposure;
        }
        if (currentGain >= minGain && currentGain <= maxGain) {
            myGain = currentGain;
        }
    }

    private void applyManualExposure() {
        ExposureControl exposure = portal.getCameraControl(ExposureControl.class);
        GainControl gain = portal.getCameraControl(GainControl.class);
        if (exposure == null || gain == null) {
            return;
        }
        if (exposure.getMode() != ExposureControl.Mode.Manual) {
            exposure.setMode(ExposureControl.Mode.Manual);
            sleep(50);
        }
        exposure.setExposure(myExposure, TimeUnit.MILLISECONDS);
        gain.setGain(myGain);
    }

    private void setAutoExposure() {
        ExposureControl exposure = portal.getCameraControl(ExposureControl.class);
        if (exposure != null) {
            exposure.setMode(ExposureControl.Mode.Auto);
        }
    }

    private void updateTelemetry() {
        boolean maskPressed = gamepad1.b;
        if (maskPressed && !lastMask && processor != null) {
            processor.setShowMask(!processor.getShowMask());
        }
        lastMask = maskPressed;

        PollenVisionResult result = processor.getResult();
        telemetry.addData("Camera", portal.getCameraState());
        if (configWarning != null) {
            telemetry.addData("Config", configWarning);
        }
        if (controlWarning != null) {
            telemetry.addData("Camera control", controlWarning);
        }
        String processorError = processor.getError();
        if (processorError != null) {
            telemetry.addData("Vision", processorError);
        }
        telemetry.addData("Front", result.frontCount);
        telemetry.addData("Storage", result.storageCount);
        telemetry.addData("View", processor.getShowMask() ? "mask" : "color");
        if (!cameraControlsSupported && cameraControlsReady) {
            telemetry.addData("Exposure", "not supported");
        } else if (!manualExposure) {
            telemetry.addData("Exposure", "auto");
        } else {
            telemetry.addData("Exposure", "%d  (%d - %d)", myExposure, minExposure, maxExposure);
            telemetry.addData("Gain", "%d  (%d - %d)", myGain, minGain, maxGain);
        }
        telemetry.addLine("3 dots, Camera Stream, to see boxes");
        telemetry.addLine("A: manual exposure    B: mask");
        telemetry.addLine("Dpad up/down exposure, left/right gain");
        telemetry.update();
    }
}
