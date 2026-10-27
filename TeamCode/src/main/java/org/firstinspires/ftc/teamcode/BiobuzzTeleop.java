package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

/**
 * Teleop for GoBilda mecanum drive with a continuous-rotation intake.
 * Gamepad 1 X toggles shooting defense: slower drive, finer aim, and an
 * odometry hold that drives back to the last commanded pose when the sticks are released.
 */
@TeleOp(name = "BiobuzzTeleop", group = "TeamCode")
public class BiobuzzTeleop extends LinearOpMode {
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;
    private CRServo intake;
    private DcMotor laucher;
    private Servo ramp;
    private CRServo javelin;

    private double frontLeftPower;
    private double frontRightPower;
    private double backLeftPower;
    private double backRightPower;
    private double intakePower;
    private double laucherPower;
    private double javelinPower;
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
    private double holdErrorX;
    private double holdErrorY;
    private double holdErrorHeading;

    private boolean rampPosition;
    private boolean rampLatchedUp;
    private boolean prevBButton;
    private boolean nectarMode;
    private boolean prevDpadDown;

    private static final double JOYSTICK_DEADZONE = 0.05;
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
    private static final double RAMP_RANGE_DEGREES = 300.0;
    private static final double RAMP_UP_POSITION = 180.0 / RAMP_RANGE_DEGREES;
    private static final double RAMP_DOWN_POSITION = 90.0 / RAMP_RANGE_DEGREES;
    private static final double JAVELIN_POWER = 1.0;
    private static final double NECTAR_MAX_POWER = 1.0;
    private static final double FLYWHEEL_VOLTAGE_HIGH = 13.0;
    private static final double FLYWHEEL_VOLTAGE_MID = 12.6;
    private static final double FLYWHEEL_VOLTAGE_LOW = 12.3;
    private static final double FLYWHEEL_POWER_HIGH = 0.55;
    private static final double FLYWHEEL_POWER_MID = 0.6;
    private static final double FLYWHEEL_POWER_LOW = 0.65;

    @Override
    public void runOpMode() {
        initHardware();

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
            updateTelemetry();
        }

        stopActuators();
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
        laucher = hardwareMap.get(DcMotor.class, "Laucher");
        ramp = hardwareMap.get(Servo.class, "Ramp");
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
        laucher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ramp.setPosition(RAMP_DOWN_POSITION);
        stopActuators();

        localizer = new ThreeDeadWheelLocalizer(hardwareMap, MecanumDrive.PARAMS.inPerTick, new Pose2d(0, 0, 0));
        updateOdometry();
        holdPose = currentPose;
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

        if (gamepad2.dpad_down && !prevDpadDown) {
            nectarMode = !nectarMode;
        }
        prevDpadDown = gamepad2.dpad_down;

        double launcherCap = nectarMode ? NECTAR_MAX_POWER : flywheelPowerForVoltage(readBatteryVoltage());
        laucherPower = launcherCap * joystickExponentialCurve(deadzone(gamepad2.right_stick_y));
        intakePower = joystickExponentialCurve(deadzone(gamepad2.left_stick_y));

        if (gamepad2.b && !prevBButton) {
            rampLatchedUp = !rampLatchedUp;
        }
        prevBButton = gamepad2.b;

        boolean rampHeldUp = gamepad2.a && intakePower == 0.0;
        rampPosition = rampHeldUp || rampLatchedUp;

        boolean retractHeld = gamepad2.left_bumper;
        boolean deployHeld = gamepad2.right_bumper;
        if (retractHeld && !deployHeld) {
            javelinPower = JAVELIN_POWER;
        } else if (deployHeld && !retractHeld) {
            javelinPower = -JAVELIN_POWER;
        } else {
            javelinPower = 0.0;
        }
    }

    private double readBatteryVoltage() {
        double voltage = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double reading = sensor.getVoltage();
            if (reading > 0) {
                voltage = Math.min(voltage, reading);
            }
        }
        batteryVoltage = voltage == Double.POSITIVE_INFINITY ? FLYWHEEL_VOLTAGE_HIGH : voltage;
        return batteryVoltage;
    }

    private double flywheelPowerForVoltage(double voltage) {
        if (voltage >= FLYWHEEL_VOLTAGE_HIGH) {
            return FLYWHEEL_POWER_HIGH;
        }
        if (voltage >= FLYWHEEL_VOLTAGE_MID) {
            return FLYWHEEL_POWER_MID;
        }
        if (voltage <= FLYWHEEL_VOLTAGE_LOW) {
            return FLYWHEEL_POWER_LOW;
        }
        double span = FLYWHEEL_VOLTAGE_MID - FLYWHEEL_VOLTAGE_LOW;
        double drop = (FLYWHEEL_VOLTAGE_MID - voltage) / span;
        return FLYWHEEL_POWER_MID + drop * (FLYWHEEL_POWER_LOW - FLYWHEEL_POWER_MID);
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
            holdingPosition = false;
            holdErrorX = 0.0;
            holdErrorY = 0.0;
            holdErrorHeading = 0.0;
            return;
        }

        boolean driverCommanding = Math.abs(drive) > 1e-4
                || Math.abs(strafe) > 1e-4
                || Math.abs(turn) > 1e-4;
        if (driverCommanding) {
            holdPose = currentPose;
            holdingPosition = false;
            holdErrorX = 0.0;
            holdErrorY = 0.0;
            holdErrorHeading = 0.0;
            return;
        }

        holdingPosition = true;
        Pose2d error = holdPose.minusExp(currentPose);
        holdErrorX = error.position.x;
        holdErrorY = error.position.y;
        holdErrorHeading = error.heading.toDouble();

        double forwardCmd = 0.0;
        double leftCmd = 0.0;
        double turnCmd = 0.0;
        if (Math.abs(holdErrorX) >= HOLD_POS_DEADBAND_IN) {
            forwardCmd = HOLD_KP_TRANSLATION * holdErrorX - HOLD_KD_TRANSLATION * currentVel.linearVel.x;
            forwardCmd = withHoldMinPower(forwardCmd, holdErrorX);
        }
        if (Math.abs(holdErrorY) >= HOLD_POS_DEADBAND_IN) {
            leftCmd = HOLD_KP_STRAFE * holdErrorY - HOLD_KD_STRAFE * currentVel.linearVel.y;
            leftCmd = withHoldMinPower(leftCmd, holdErrorY);
        }
        if (Math.abs(holdErrorHeading) >= HOLD_HEADING_DEADBAND_RAD) {
            turnCmd = HOLD_KP_HEADING * holdErrorHeading - HOLD_KD_HEADING * currentVel.angVel;
            turnCmd = withHoldMinPower(turnCmd, holdErrorHeading);
        }

        drive = clamp(-forwardCmd, -HOLD_MAX_POWER, HOLD_MAX_POWER);
        strafe = clamp(-leftCmd, -HOLD_MAX_POWER, HOLD_MAX_POWER);
        turn = clamp(turnCmd, -HOLD_MAX_POWER, HOLD_MAX_POWER);
    }

    private double withHoldMinPower(double command, double error) {
        if (command * error < 0.0) {
            return command;
        }
        if (Math.abs(command) < HOLD_MIN_POWER) {
            return Math.copySign(HOLD_MIN_POWER, error);
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
        ramp.setPosition(rampPosition ? RAMP_UP_POSITION : RAMP_DOWN_POSITION);
        javelin.setPower(javelinPower);
    }

    private void stopActuators() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        intake.setPower(0);
        laucher.setPower(0);
        javelinPower = 0.0;
        javelin.setPower(0);
    }

    private void updateTelemetry() {
        telemetry.addData("Drive", "%.2f", drive);
        telemetry.addData("Strafe", "%.2f", strafe);
        telemetry.addData("Turn", "%.2f", turn);
        if (!defenseMode) {
            telemetry.addData("Defense mode", "off");
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
        telemetry.addData("Launch mode", nectarMode ? "nectar" : "pollen");
        telemetry.addData("Battery", "%.2f V", batteryVoltage);
        telemetry.addData("Laucher", "%.2f", laucherPower);
        telemetry.addData("Ramp", rampPosition ? "up 180 deg" : "down 90 deg");
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
        telemetry.addLine("GP2 dpad down: pollen / nectar");
        telemetry.addLine("GP2 A: hold ramp up while intake is stopped");
        telemetry.addLine("GP2 B: toggle ramp up or down");
        telemetry.addLine("GP2 left bumper: hold to retract javelin");
        telemetry.addLine("GP2 right bumper: hold to deploy javelin");
        telemetry.update();
    }
}
