package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Tune javelin run time and launcher speed.
 * Hold gamepad 2 dpad up to run the gate forward and down to run it back.
 */
@TeleOp(name = "Servo Test", group = "TeamCode")
public class ServoTest extends LinearOpMode {
    private CRServo gate;
    private CRServo javelin;
    private DcMotor laucher;

    private double gatePower;
    private double gateSpeed = 0.5;
    private boolean prevGateSlower;
    private boolean prevGateFaster;
    private double javelinPower;
    private double javelinRunSeconds;
    private double javelinDirection = 1.0;
    private boolean javelinRunning;
    private boolean javelinReplaying;

    private boolean prevReplay;
    private boolean prevReset;

    private double laucherPower;
    private double laucherStep = 0.05;
    private boolean prevLauncherUp;
    private boolean prevLauncherDown;
    private boolean prevLauncherDpadLeft;
    private boolean prevLauncherDpadRight;

    private ElapsedTime javelinTimer = new ElapsedTime();
    private ElapsedTime laucherStepTimer = new ElapsedTime();
    private ElapsedTime gateSpeedTimer = new ElapsedTime();

    private static final double STICK_STEP_THRESHOLD = 0.5;
    private static final double JAVELIN_POWER = 1.0;
    private static final double GATE_SPEED_STEP = 0.05;
    private static final double GATE_SPEED_STEP_SECONDS = 0.2;
    private static final double LAUNCHER_STEP_SECONDS = 0.2;

    @Override
    public void runOpMode() {
        initHardware();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            updateGate();
            updateJavelin();
            updateLauncher();
            updateTelemetry();
        }

        stopActuators();
    }

    private void initHardware() {
        gate = hardwareMap.get(CRServo.class, "Gate");
        javelin = hardwareMap.get(CRServo.class, "Javelin");
        laucher = hardwareMap.get(DcMotor.class, "Laucher");

        laucher.setDirection(DcMotor.Direction.FORWARD);
        laucher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        laucher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        gate.setPower(0);
        javelin.setPower(0);
        laucher.setPower(0);
    }

    private void updateGate() {
        boolean slower = gamepad2.left_bumper;
        boolean faster = gamepad2.right_bumper;
        boolean speedPressed = (faster && !prevGateFaster) || (slower && !prevGateSlower);
        boolean speedHeld = (faster && prevGateFaster) || (slower && prevGateSlower);
        if (slower != faster && (speedPressed || (speedHeld && gateSpeedTimer.seconds() >= GATE_SPEED_STEP_SECONDS))) {
            gateSpeed += faster ? GATE_SPEED_STEP : -GATE_SPEED_STEP;
            gateSpeed = Math.max(GATE_SPEED_STEP, Math.min(1.0, gateSpeed));
            gateSpeed = Math.round(gateSpeed * 100.0) / 100.0;
            gateSpeedTimer.reset();
        }
        prevGateSlower = slower;
        prevGateFaster = faster;

        boolean forward = gamepad2.dpad_up;
        boolean back = gamepad2.dpad_down;
        if (forward && !back) {
            gatePower = gateSpeed;
        } else if (back && !forward) {
            gatePower = -gateSpeed;
        } else {
            gatePower = 0.0;
        }
        gate.setPower(gatePower);
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

    private void updateLauncher() {
        boolean stepUp = gamepad2.right_stick_y < -STICK_STEP_THRESHOLD;
        boolean stepDown = gamepad2.right_stick_y > STICK_STEP_THRESHOLD;
        if (stepUp && stepDown) {
            stepUp = false;
            stepDown = false;
        }

        boolean stepPressed = (stepUp && !prevLauncherUp) || (stepDown && !prevLauncherDown);
        boolean stepHeld = (stepUp && prevLauncherUp) || (stepDown && prevLauncherDown);
        if (stepPressed || (stepHeld && laucherStepTimer.seconds() >= LAUNCHER_STEP_SECONDS)) {
            laucherPower += stepUp ? laucherStep : -laucherStep;
            laucherPower = Math.max(-1.0, Math.min(1.0, laucherPower));
            laucherPower = Math.round(laucherPower * 100.0) / 100.0;
            laucherStepTimer.reset();
        }
        prevLauncherUp = stepUp;
        prevLauncherDown = stepDown;

        if (gamepad2.dpad_right && !prevLauncherDpadRight) {
            laucherStep = Math.min(0.10, Math.round((laucherStep + 0.01) * 100.0) / 100.0);
        }
        if (gamepad2.dpad_left && !prevLauncherDpadLeft) {
            laucherStep = Math.max(0.01, Math.round((laucherStep - 0.01) * 100.0) / 100.0);
        }
        prevLauncherDpadLeft = gamepad2.dpad_left;
        prevLauncherDpadRight = gamepad2.dpad_right;

        laucher.setPower(laucherPower);
    }

    private void stopActuators() {
        javelinPower = 0.0;
        javelinRunning = false;
        javelinReplaying = false;
        javelin.setPower(0);
        laucherPower = 0.0;
        laucher.setPower(0);
        gatePower = 0.0;
        gate.setPower(0);
    }

    private void updateTelemetry() {
        telemetry.addData("Gate speed", "%.2f", gateSpeed);
        if (gatePower > 0.0) {
            telemetry.addData("Gate", "forward");
        } else if (gatePower < 0.0) {
            telemetry.addData("Gate", "back");
        } else {
            telemetry.addData("Gate", "stopped");
        }
        telemetry.addData("Javelin power", "%.2f", javelinPower);
        telemetry.addData("Javelin time", "%.3f s %s", javelinRunSeconds, javelinDirection > 0.0 ? "forward" : "reverse");
        if (javelinReplaying) {
            telemetry.addData("Javelin", "replaying");
        } else if (javelinPower == 0.0) {
            telemetry.addData("Javelin", "stopped");
        } else {
            telemetry.addData("Javelin", "running");
        }
        telemetry.addData("Launcher power", "%.2f", laucherPower);
        telemetry.addData("Launcher step", "%.2f", laucherStep);
        telemetry.addLine("Controls (gamepad 1)");
        telemetry.addLine("Right bumper: javelin forward, release to stop");
        telemetry.addLine("Left bumper: javelin reverse, release to stop");
        telemetry.addLine("B: replay that javelin time");
        telemetry.addLine("X: clear javelin time");
        telemetry.addLine("Controls (gamepad 2)");
        telemetry.addLine("Dpad up: gate forward, release to stop");
        telemetry.addLine("Dpad down: gate back, release to stop");
        telemetry.addLine("Bumpers: slower/faster gate");
        telemetry.addLine("Right stick up/down: launcher speed");
        telemetry.addLine("Dpad left/right: smaller/bigger launcher step");
        telemetry.update();
    }
}
