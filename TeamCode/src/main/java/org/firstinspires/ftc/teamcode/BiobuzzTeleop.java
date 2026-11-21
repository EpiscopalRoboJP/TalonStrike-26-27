package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.vision.PollenCamera;
import org.firstinspires.ftc.teamcode.vision.PollenVisionResult;

/**
 * Teleop for GoBilda mecanum drive with a continuous-rotation intake.
 * Gamepad 1 X toggles shooting defense: slower drive, finer aim, and an
 * odometry hold that drives back to the last commanded pose when the sticks are released.
 * RGB shows drive, intake, and launcher. RGB2 is red while adjusting in defense,
 * orange while holding, a slow rainbow while being pushed, and green when the
 * launcher encoder is holding shoot speed. Gamepad 2 both triggers plus dpad
 * right toggles a rainbow on both lights. The pollen webcam is optional.
 */
@TeleOp(name = "BiobuzzTeleop", group = "TeamCode")
public class BiobuzzTeleop extends LinearOpMode {
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;
    private CRServo intake;
    private DcMotorEx laucher;
    private CRServo gate;
    private CRServo javelin;

    private double frontLeftPower;
    private double frontRightPower;
    private double backLeftPower;
    private double backRightPower;
    private double intakePower;
    private double laucherPower;
    private double javelinPower;
    private double gatePower;
    private double drive;
    private double strafe;
    private double turn;
    private double batteryVoltage;

    private ThreeDeadWheelLocalizer localizer;
    private Pose2d currentPose = new Pose2d(0, 0, 0);
    private PoseVelocity2d currentVel = new PoseVelocity2d(new Vector2d(0, 0), 0);
    private Pose2d holdPose = new Pose2d(0, 0, 0);
    private boolean defenseMode;
    private boolean prevXButton;
    private boolean holdingPosition;
    private boolean fighting;
    private double holdErrorX;
    private double holdErrorY;
    private double holdErrorHeading;

    private boolean prevDpadRight;
    private boolean partyMode;
    private double launcherTicksPerSec;
    private final BiobuzzMechanisms.LauncherSpeedTracker launcherSpeed =
            new BiobuzzMechanisms.LauncherSpeedTracker();
    private RgbIndicator indicator;
    private PollenCamera pollenCamera;

    private static final double JOYSTICK_DEADZONE = 0.05;
    private static final double TRIGGER_PRESSED = 0.5;
    private static final double AIM_TURN_POWER = 0.08;
    private static final double DEFENSE_TRANSLATION_SCALE = 0.4;
    private static final double DEFENSE_TURN_SCALE = 0.35;
    private static final double DEFENSE_AIM_TURN_POWER = 0.1;
    private static final double DEFENSE_NUDGE_DRIVE_POWER = 0.12;
    private static final double HOLD_POS_DEADBAND_IN = 0.35;
    private static final double HOLD_HEADING_DEADBAND_RAD = Math.toRadians(1.5);
    private static final double HOLD_KP_TRANSLATION = 0.22;
    private static final double HOLD_KP_STRAFE = 0.25;
    private static final double HOLD_KP_HEADING = 2.5;
    private static final double HOLD_KD_TRANSLATION = 0.02;
    private static final double HOLD_KD_STRAFE = 0.02;
    private static final double HOLD_KD_HEADING = 0.12;
    private static final double HOLD_MAX_POWER = 0.85;
    private static final double HOLD_MIN_POWER = 0.08;
    private static final double HOLD_PUSH_SPEED_IN_PER_S = 6.0;
    private static final double HOLD_PUSH_TURN_RAD_PER_S = 0.5;
    private static final double HOLD_FIGHT_KP_SCALE = 2.2;
    private static final double HOLD_FIGHT_KD_SCALE = 4.0;
    private static final double HOLD_FIGHT_MAX_POWER = 1.0;
    private static final double HOLD_FIGHT_MIN_POWER = 0.28;

    @Override
    public void runOpMode() {
        initHardware();

        try {
            telemetry.addData("Status", "Initialized");
            telemetry.update();

            waitForStart();

            while (opModeIsActive()) {
                updateOdometry();
                getGamepadInputs();
                applyPositionHold();
                calculateMecanumDrive();
                setMotorPowers();
                updateServoPositions();
                updateIndicator();
                updateTelemetry();
            }
        } finally {
            stopActuators();
            if (pollenCamera != null) {
                pollenCamera.close();
                pollenCamera = null;
            }
        }
    }

    private void initHardware() {
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        frontLeftMotor = hardwareMap.get(DcMotor.class, "FL");
        frontRightMotor = hardwareMap.get(DcMotor.class, "FR");
        backLeftMotor = hardwareMap.get(DcMotor.class, "BL");
        backRightMotor = hardwareMap.get(DcMotor.class, "BR");
        intake = hardwareMap.get(CRServo.class, "Intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        laucher = hardwareMap.get(DcMotorEx.class, "Laucher");
        gate = hardwareMap.get(CRServo.class, "Gate");
        javelin = hardwareMap.get(CRServo.class, "Javelin");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        laucher.setDirection(DcMotor.Direction.FORWARD);
        laucher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Power stays open-loop. The encoder is read for speed only.
        laucher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        gate.setPower(0);
        stopActuators();

        localizer = new ThreeDeadWheelLocalizer(hardwareMap, MecanumDrive.PARAMS.inPerTick, new Pose2d(0, 0, 0));
        updateOdometry();
        holdPose = currentPose;

        indicator = RgbIndicator.attach(hardwareMap);
        pollenCamera = PollenCamera.openIfConfigured(hardwareMap);
    }

    private void getGamepadInputs() {
        double leftStickY = joystickExponentialCurve(gamepad1.left_stick_y);
        double leftStickX = joystickExponentialCurve(gamepad1.left_stick_x);
        double rightStickX = joystickExponentialCurve(gamepad1.right_stick_x);
        drive = leftStickY;
        strafe = leftStickX;
        turn = -rightStickX;

        if (gamepad1.x && !prevXButton) {
            defenseMode = !defenseMode;
            if (defenseMode) {
                holdPose = currentPose;
            }
        }
        prevXButton = gamepad1.x;

        if (defenseMode) {
            drive *= DEFENSE_TRANSLATION_SCALE;
            strafe *= DEFENSE_TRANSLATION_SCALE;
            turn *= DEFENSE_TURN_SCALE;
        }

        double aimPower = defenseMode ? DEFENSE_AIM_TURN_POWER : AIM_TURN_POWER;
        if (gamepad1.dpad_left && !gamepad1.dpad_right) {
            turn += aimPower;
        } else if (gamepad1.dpad_right && !gamepad1.dpad_left) {
            turn -= aimPower;
        }
        if (defenseMode) {
            if (gamepad1.dpad_up && !gamepad1.dpad_down) {
                drive -= DEFENSE_NUDGE_DRIVE_POWER;
            } else if (gamepad1.dpad_down && !gamepad1.dpad_up) {
                drive += DEFENSE_NUDGE_DRIVE_POWER;
            }
        }

        boolean triggersHeld = gamepad2.left_trigger >= TRIGGER_PRESSED
                && gamepad2.right_trigger >= TRIGGER_PRESSED;
        if (triggersHeld && gamepad2.dpad_right && !prevDpadRight) {
            partyMode = !partyMode;
        }
        prevDpadRight = gamepad2.dpad_right;

        double launcherCap = BiobuzzMechanisms.flywheelPowerForVoltage(readBatteryVoltage());
        laucherPower = launcherCap * joystickExponentialCurve(deadzone(gamepad2.right_stick_y));
        intakePower = joystickExponentialCurve(deadzone(gamepad2.left_stick_y));

        boolean gateForward = gamepad2.dpad_up;
        boolean gateBack = gamepad2.dpad_down;
        if (gateForward && !gateBack) {
            gatePower = BiobuzzMechanisms.GATE_POWER;
        } else if (gateBack && !gateForward) {
            gatePower = -BiobuzzMechanisms.GATE_POWER;
        } else {
            gatePower = 0.0;
        }

        boolean retractHeld = gamepad2.left_bumper;
        boolean deployHeld = gamepad2.right_bumper;
        if (retractHeld && !deployHeld) {
            javelinPower = BiobuzzMechanisms.JAVELIN_POWER;
        } else if (deployHeld && !retractHeld) {
            javelinPower = -BiobuzzMechanisms.JAVELIN_POWER;
        } else {
            javelinPower = 0.0;
        }
    }

    private double readBatteryVoltage() {
        batteryVoltage = BiobuzzMechanisms.readBatteryVoltage(hardwareMap);
        return batteryVoltage;
    }

    private double deadzone(double gamepadInput) {
        if (Math.abs(gamepadInput) < JOYSTICK_DEADZONE) {
            return 0.0;
        }
        return gamepadInput;
    }

    private double joystickExponentialCurve(double gamepadInput) {
        gamepadInput = Math.max(-1.0, Math.min(1.0, gamepadInput));
        double sign = Math.signum(gamepadInput);
        double absInput = Math.abs(gamepadInput);
        return sign * Math.pow(absInput, 4);
    }

    private void updateOdometry() {
        currentVel = localizer.update();
        currentPose = localizer.getPose();
    }

    /**
     * While defense mode is on and the driver has released the sticks, drive back
     * to holdPose. Robot +x is forward and +y is left; teleop forward is negative drive
     * and teleop right is positive strafe.
     */
    private void applyPositionHold() {
        if (!defenseMode) {
            clearHold();
            return;
        }

        boolean driverCommanding = Math.abs(drive) > 1e-4
                || Math.abs(strafe) > 1e-4
                || Math.abs(turn) > 1e-4;
        if (driverCommanding) {
            holdPose = currentPose;
            clearHold();
            return;
        }

        holdingPosition = true;
        Pose2d error = holdPose.minusExp(currentPose);
        holdErrorX = error.position.x;
        holdErrorY = error.position.y;
        holdErrorHeading = error.heading.toDouble();

        boolean fightX = axisPushed(
                holdErrorX, currentVel.linearVel.x, HOLD_POS_DEADBAND_IN, HOLD_PUSH_SPEED_IN_PER_S);
        boolean fightY = axisPushed(
                holdErrorY, currentVel.linearVel.y, HOLD_POS_DEADBAND_IN, HOLD_PUSH_SPEED_IN_PER_S);
        boolean fightHeading = axisPushed(
                holdErrorHeading, currentVel.angVel, HOLD_HEADING_DEADBAND_RAD, HOLD_PUSH_TURN_RAD_PER_S);
        fighting = fightX || fightY || fightHeading;
        double maxPower = fighting ? HOLD_FIGHT_MAX_POWER : HOLD_MAX_POWER;

        double forwardCmd = holdAxis(
                holdErrorX, currentVel.linearVel.x, HOLD_POS_DEADBAND_IN, fightX,
                HOLD_KP_TRANSLATION, HOLD_KD_TRANSLATION);
        double leftCmd = holdAxis(
                holdErrorY, currentVel.linearVel.y, HOLD_POS_DEADBAND_IN, fightY,
                HOLD_KP_STRAFE, HOLD_KD_STRAFE);
        double turnCmd = holdAxis(
                holdErrorHeading, currentVel.angVel, HOLD_HEADING_DEADBAND_RAD, fightHeading,
                HOLD_KP_HEADING, HOLD_KD_HEADING);

        drive = clamp(-forwardCmd, -maxPower, maxPower);
        strafe = clamp(-leftCmd, -maxPower, maxPower);
        turn = clamp(turnCmd, -maxPower, maxPower);
    }

    private void clearHold() {
        holdingPosition = false;
        fighting = false;
        holdErrorX = 0.0;
        holdErrorY = 0.0;
        holdErrorHeading = 0.0;
    }

    /**
     * True when this axis is moving while the hold should be still, or moving
     * so the pose error is growing. That is another robot pushing.
     */
    private boolean axisPushed(double error, double velocity, double deadband, double speed) {
        if (Math.abs(velocity) < speed) {
            return false;
        }
        if (Math.abs(error) < deadband) {
            return true;
        }
        return error * velocity < 0.0;
    }

    private double holdAxis(
            double error,
            double velocity,
            double deadband,
            boolean fight,
            double kp,
            double kd) {
        if (Math.abs(error) < deadband && !fight) {
            return 0.0;
        }
        double commandKp = fight ? kp * HOLD_FIGHT_KP_SCALE : kp;
        double commandKd = fight ? kd * HOLD_FIGHT_KD_SCALE : kd;
        double command = commandKp * error - commandKd * velocity;
        double minPower = fight ? HOLD_FIGHT_MIN_POWER : HOLD_MIN_POWER;
        double signSource = Math.abs(error) >= deadband ? error : -velocity;
        return withHoldMinPower(command, signSource, minPower);
    }

    private double withHoldMinPower(double command, double signSource, double minPower) {
        if (command * signSource < 0.0) {
            return command;
        }
        if (Math.abs(command) < minPower && signSource != 0.0) {
            return Math.copySign(minPower, signSource);
        }
        return command;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private void calculateMecanumDrive() {
        frontLeftPower = drive - strafe + turn;
        frontRightPower = drive - strafe - turn;
        backLeftPower = drive + strafe + turn;
        backRightPower = drive + strafe - turn;

        double maxPower = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));

        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }
    }

    private void setMotorPowers() {
        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);
        intake.setPower(intakePower);
        laucher.setPower(laucherPower);
    }

    private void updateServoPositions() {
        gate.setPower(gatePower);
        javelin.setPower(javelinPower);
    }

    private void updateIndicator() {
        boolean robotMoving = Math.abs(drive) > BiobuzzMechanisms.MOVE_COMMAND
                || Math.abs(strafe) > BiobuzzMechanisms.MOVE_COMMAND
                || Math.abs(turn) > BiobuzzMechanisms.MOVE_COMMAND;
        double ticksPerSec = readLauncherTicksPerSec();
        indicator.update(
                robotMoving,
                Math.abs(intakePower) > BiobuzzMechanisms.MOVE_COMMAND,
                ticksPerSec >= BiobuzzMechanisms.LAUNCHER_MOVE_TICKS_PER_SEC,
                defenseMode,
                holdingPosition,
                fighting,
                launcherAtShootSpeed(ticksPerSec),
                partyMode);
    }

    private double readLauncherTicksPerSec() {
        if (laucher == null) {
            launcherTicksPerSec = 0.0;
            return 0.0;
        }
        launcherTicksPerSec = Math.abs(laucher.getVelocity());
        return launcherTicksPerSec;
    }

    /**
     * Full stick asks for the shot. Green once the encoder stops climbing and
     * holds that speed, which is the wheel at the speed this power produces.
     */
    private boolean launcherAtShootSpeed(double ticksPerSec) {
        double cap = BiobuzzMechanisms.flywheelPowerForVoltage(batteryVoltage);
        return launcherSpeed.atShootSpeed(laucherPower, cap, ticksPerSec);
    }

    private void stopActuators() {
        if (frontLeftMotor != null) {
            frontLeftMotor.setPower(0);
        }
        if (frontRightMotor != null) {
            frontRightMotor.setPower(0);
        }
        if (backLeftMotor != null) {
            backLeftMotor.setPower(0);
        }
        if (backRightMotor != null) {
            backRightMotor.setPower(0);
        }
        if (intake != null) {
            intake.setPower(0);
        }
        if (laucher != null) {
            laucher.setPower(0);
        }
        javelinPower = 0.0;
        if (javelin != null) {
            javelin.setPower(0);
        }
        gatePower = 0.0;
        if (gate != null) {
            gate.setPower(0);
        }
        if (indicator != null) {
            indicator.off();
        }
    }

    private void updateTelemetry() {
        telemetry.addData("Drive", "%.2f", drive);
        telemetry.addData("Strafe", "%.2f", strafe);
        telemetry.addData("Turn", "%.2f", turn);
        if (!defenseMode) {
            telemetry.addData("Defense mode", "off");
        } else if (fighting) {
            telemetry.addData("Defense mode", "ACTIVE (FIGHTING)");
        } else if (holdingPosition) {
            telemetry.addData("Defense mode", "ACTIVE (HOLDING)");
        } else {
            telemetry.addData("Defense mode", "ACTIVE (ADJUSTING)");
        }
        if (defenseMode) {
            telemetry.addData("Hold error", "x %.2f in  y %.2f in  h %.1f deg",
                    holdErrorX, holdErrorY, Math.toDegrees(holdErrorHeading));
        }
        telemetry.addData("Intake", "%.2f", intakePower);
        telemetry.addData("Battery", "%.2f V", batteryVoltage);
        telemetry.addData("RGB", "%s", indicator.rgbLabel());
        telemetry.addData("RGB2", "%s", indicator.rgb2Label());
        telemetry.addData("Lights", indicator.lightCount());
        if (pollenCamera == null) {
            telemetry.addData("Balls", "camera not configured");
        } else if (!pollenCamera.hasReading()) {
            telemetry.addData("Balls", "waiting for camera");
        } else {
            PollenVisionResult balls = pollenCamera.result();
            telemetry.addData("Balls", "front %d  storage %d", balls.frontCount, balls.storageCount);
        }
        telemetry.addData("Laucher", "%.2f power  %.0f ticks/s", laucherPower, launcherTicksPerSec);
        if (gatePower > 0.0) {
            telemetry.addData("Gate", "forward %.2f", BiobuzzMechanisms.GATE_POWER);
        } else if (gatePower < 0.0) {
            telemetry.addData("Gate", "back %.2f", BiobuzzMechanisms.GATE_POWER);
        } else {
            telemetry.addData("Gate", "stopped");
        }
        if (javelinPower > 0.0) {
            telemetry.addData("Javelin", "retracting");
        } else if (javelinPower < 0.0) {
            telemetry.addData("Javelin", "deploying");
        } else {
            telemetry.addData("Javelin", "stopped");
        }

        telemetry.addLine("Controls");
        telemetry.addLine("GP1 left stick: drive / strafe");
        telemetry.addLine("GP1 right stick: turn");
        telemetry.addLine("GP1 dpad left / right: minor aim");
        telemetry.addLine("GP1 dpad up / down: fine drive in defense");
        telemetry.addLine("GP1 X: toggle shooting defense mode");
        telemetry.addLine("GP2 left stick: intake");
        telemetry.addLine("GP2 right stick: launcher");
        telemetry.addLine("GP2 dpad up: gate forward");
        telemetry.addLine("GP2 dpad down: gate back");
        telemetry.addLine("GP2 both triggers + dpad right: toggle rainbow lights");
        telemetry.addLine("GP2 left bumper: hold to retract javelin");
        telemetry.addLine("GP2 right bumper: hold to deploy javelin");
        telemetry.addLine("RGB: yellow drive, green intake, purple launcher");
        telemetry.addLine("RGB2 defense: red adjusting, orange hold, rainbow fight");
        telemetry.addLine("RGB2 green when launcher encoder holds shoot speed");
        telemetry.update();
    }
}
