package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Tune ramp positions and javelin run time.
 * Ramp degrees assume a 300 degree servo range. Copy the 0-1 position into setPosition.
 */
@TeleOp(name = "Servo Test", group = "TeamCode")
public class ServoTest extends LinearOpMode {
    private Servo ramp;
    private CRServo javelin;

    private double rampPosition = 0.5;
    private double rampStep = 0.01;
    private double rampDownPosition = 0.5;
    private double rampUpPosition = 0.5;
    private boolean rampDownSaved;
    private boolean rampUpSaved;

    private double javelinPower;
    private double javelinRunSeconds;
    private double javelinDirection = 1.0;
    private boolean javelinRunning;
    private boolean javelinReplaying;

    private boolean prevDpadUp;
    private boolean prevDpadDown;
    private boolean prevDpadLeft;
    private boolean prevDpadRight;
    private boolean prevSaveDown;
    private boolean prevSaveUp;
    private boolean prevReplay;
    private boolean prevReset;

    private ElapsedTime javelinTimer = new ElapsedTime();
    private ElapsedTime rampStepTimer = new ElapsedTime();

    private static final double RAMP_RANGE_DEGREES = 300.0;
    private static final double RAMP_STEP_SECONDS = 0.2;
    private static final double JAVELIN_POWER = 1.0;

    @Override
    public void runOpMode() {
        initHardware();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            updateRamp();
            updateJavelin();
            updateTelemetry();
        }

        stopActuators();
    }

    private void initHardware() {
        ramp = hardwareMap.get(Servo.class, "Ramp");
        javelin = hardwareMap.get(CRServo.class, "Javelin");

        ramp.setPosition(rampPosition);
        javelin.setPower(0);
    }

    private void updateRamp() {
        boolean stepUp = gamepad1.dpad_up;
        boolean stepDown = gamepad1.dpad_down;
        if (stepUp && stepDown) {
            stepUp = false;
            stepDown = false;
        }

        boolean stepPressed = (stepUp && !prevDpadUp) || (stepDown && !prevDpadDown);
        boolean stepHeld = (stepUp && prevDpadUp) || (stepDown && prevDpadDown);
        if (stepPressed || (stepHeld && rampStepTimer.seconds() >= RAMP_STEP_SECONDS)) {
            rampPosition += stepUp ? rampStep : -rampStep;
            rampPosition = Math.max(0.0, Math.min(1.0, rampPosition));
            rampPosition = Math.round(rampPosition * 100.0) / 100.0;
            rampStepTimer.reset();
        }
        prevDpadUp = gamepad1.dpad_up;
        prevDpadDown = gamepad1.dpad_down;

        if (gamepad1.dpad_right && !prevDpadRight) {
            rampStep = Math.min(0.10, Math.round((rampStep + 0.01) * 100.0) / 100.0);
        }
        if (gamepad1.dpad_left && !prevDpadLeft) {
            rampStep = Math.max(0.01, Math.round((rampStep - 0.01) * 100.0) / 100.0);
        }
        prevDpadLeft = gamepad1.dpad_left;
        prevDpadRight = gamepad1.dpad_right;

        if (gamepad1.a && !prevSaveDown) {
            rampDownPosition = rampPosition;
            rampDownSaved = true;
        }
        if (gamepad1.y && !prevSaveUp) {
            rampUpPosition = rampPosition;
            rampUpSaved = true;
        }
        prevSaveDown = gamepad1.a;
        prevSaveUp = gamepad1.y;

        if (gamepad1.left_stick_button && rampDownSaved) {
            rampPosition = rampDownPosition;
        }
        if (gamepad1.right_stick_button && rampUpSaved) {
            rampPosition = rampUpPosition;
        }

        ramp.setPosition(rampPosition);
    }

    private void updateJavelin() {
        boolean forward = gamepad1.right_bumper;
        boolean reverse = gamepad1.left_bumper;

        if (gamepad1.x && !prevReset) {
            javelinRunning = false;
            javelinReplaying = false;
            javelinPower = 0.0;
            if (!forward && !reverse) {
                javelinRunSeconds = 0.0;
            }
        } else if (forward != reverse) {
            if (!javelinRunning || javelinReplaying) {
                javelinTimer.reset();
                javelinReplaying = false;
            }
            javelinRunning = true;
            javelinDirection = forward ? 1.0 : -1.0;
            javelinPower = javelinDirection * JAVELIN_POWER;
            javelinRunSeconds = javelinTimer.seconds();
        } else if (javelinReplaying) {
            if (javelinTimer.seconds() >= javelinRunSeconds) {
                javelinReplaying = false;
                javelinPower = 0.0;
            } else {
                javelinPower = javelinDirection * JAVELIN_POWER;
            }
        } else if (gamepad1.b && !prevReplay && javelinRunSeconds > 0.0) {
            javelinReplaying = true;
            javelinRunning = false;
            javelinTimer.reset();
            javelinPower = javelinDirection * JAVELIN_POWER;
        } else {
            javelinRunning = false;
            javelinPower = 0.0;
        }

        prevReplay = gamepad1.b;
        prevReset = gamepad1.x;
        javelin.setPower(javelinPower);
    }

    private void stopActuators() {
        javelinPower = 0.0;
        javelinRunning = false;
        javelinReplaying = false;
        javelin.setPower(0);
    }

    private void updateTelemetry() {
        telemetry.addData("Ramp position", "%.2f", rampPosition);
        telemetry.addData("Ramp degrees", "%.0f", rampPosition * RAMP_RANGE_DEGREES);
        telemetry.addData("Ramp step", "%.2f", rampStep);
        addSavedPosition("Ramp down", rampDownSaved, rampDownPosition);
        addSavedPosition("Ramp up", rampUpSaved, rampUpPosition);
        telemetry.addData("Javelin power", "%.2f", javelinPower);
        telemetry.addData("Javelin time", "%.3f s %s", javelinRunSeconds, javelinDirection > 0.0 ? "forward" : "reverse");
        if (javelinReplaying) {
            telemetry.addData("Javelin", "replaying");
        } else if (javelinPower == 0.0) {
            telemetry.addData("Javelin", "stopped");
        } else {
            telemetry.addData("Javelin", "running");
        }
        telemetry.addLine("Controls (gamepad 1)");
        telemetry.addLine("Dpad up/down: move ramp");
        telemetry.addLine("Dpad left/right: smaller/bigger step");
        telemetry.addLine("A: save ramp down");
        telemetry.addLine("Y: save ramp up");
        telemetry.addLine("Left stick click: go to saved down");
        telemetry.addLine("Right stick click: go to saved up");
        telemetry.addLine("Right bumper: javelin forward, release to stop");
        telemetry.addLine("Left bumper: javelin reverse, release to stop");
        telemetry.addLine("B: replay that javelin time");
        telemetry.addLine("X: clear javelin time");
        telemetry.update();
    }

    private void addSavedPosition(String label, boolean saved, double position) {
        if (saved) {
            telemetry.addData(label, "%.2f  %.0f deg", position, position * RAMP_RANGE_DEGREES);
        } else {
            telemetry.addData(label, "not saved");
        }
    }
}
