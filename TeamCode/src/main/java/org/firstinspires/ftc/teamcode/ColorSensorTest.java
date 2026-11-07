package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Reads color and short-range distance from one REV Color Sensor V3.
 * Configure the I2C device as "REV Color Sensor V3" named sensor_color.
 * Hold gamepad 1 A to raise gain and B to lower it. Gain stays at or above 1.
 * Distance is only meaningful from about 0.25 to 6 inches.
 */
@TeleOp(name = "Color Sensor Test", group = "TeamCode")
public class ColorSensorTest extends LinearOpMode {
    private NormalizedColorSensor colorSensor;
    private DistanceSensor distanceSensor;

    private float gain;
    private final float[] hsvValues = new float[3];

    private static final float GAIN_START = 2;
    private static final float GAIN_MIN = 1;
    private static final float GAIN_STEP = 0.005f;
    private static final double DISTANCE_MAX_INCHES = 6.0;

    @Override
    public void runOpMode() {
        initHardware();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Gain", "%.3f", gain);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            updateGain();
            updateTelemetry();
        }
    }

    private void initHardware() {
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        gain = GAIN_START;
        colorSensor.setGain(gain);

        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor).enableLight(true);
        }
        if (colorSensor instanceof DistanceSensor) {
            distanceSensor = (DistanceSensor) colorSensor;
        }
    }

    private void updateGain() {
        if (gamepad1.a) {
            gain += GAIN_STEP;
        } else if (gamepad1.b && gain > GAIN_MIN) {
            gain -= GAIN_STEP;
            if (gain < GAIN_MIN) {
                gain = GAIN_MIN;
            }
        }
        colorSensor.setGain(gain);
    }

    private void updateTelemetry() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);

        telemetry.addData("Gain", "%.3f", gain);
        telemetry.addLine()
                .addData("Red", "%.3f", colors.red)
                .addData("Green", "%.3f", colors.green)
                .addData("Blue", "%.3f", colors.blue);
        telemetry.addLine()
                .addData("Hue", "%.3f", hsvValues[0])
                .addData("Saturation", "%.3f", hsvValues[1])
                .addData("Value", "%.3f", hsvValues[2]);
        telemetry.addData("Alpha", "%.3f", colors.alpha);

        if (distanceSensor == null) {
            telemetry.addData("Distance", "not available");
        } else {
            double inches = distanceSensor.getDistance(DistanceUnit.INCH);
            double cm = DistanceUnit.INCH.toCm(inches);
            telemetry.addData("Distance (cm)", "%.3f", cm);
            telemetry.addData("Distance (in)", "%.3f", inches);
            if (isOutOfRange(inches)) {
                telemetry.addData("Distance", "out of range");
            } else {
                telemetry.addData("Distance", "in range");
            }
        }

        telemetry.addLine("Controls (gamepad 1)");
        telemetry.addLine("A: increase gain");
        telemetry.addLine("B: decrease gain");
        telemetry.update();
    }

    private boolean isOutOfRange(double inches) {
        return Double.isNaN(inches)
                || inches >= DISTANCE_MAX_INCHES
                || inches == DistanceSensor.distanceOutOfRange;
    }
}
