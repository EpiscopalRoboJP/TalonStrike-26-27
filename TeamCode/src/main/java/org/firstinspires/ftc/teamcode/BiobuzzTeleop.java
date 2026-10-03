package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Teleop for GoBilda mecanum drive with a continuous-rotation intake.
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

    private boolean rampPosition;
    private boolean prevLeftBumper;
    private boolean prevRightBumper;
    private boolean javelinDeployed;
    private boolean javelinRunning;
    private final ElapsedTime javelinTimer = new ElapsedTime();

    private static final double JOYSTICK_DEADZONE = 0.05;
    private static final double RAMP_RANGE_DEGREES = 300.0;
    private static final double RAMP_UP_POSITION = 180.0 / RAMP_RANGE_DEGREES;
    private static final double RAMP_DOWN_POSITION = 90.0 / RAMP_RANGE_DEGREES;
    private static final double JAVELIN_RUN_SECONDS = 8.0;
    private static final double JAVELIN_POWER = 1.0;

    @Override
    public void runOpMode() {
        initHardware();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            getGamepadInputs();
            calculateMecanumDrive();
            setMotorPowers();
            updateServoPositions();
            updateTelemetry();
        }

        stopActuators();
    }

    private void initHardware() {
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
    }

    private void getGamepadInputs() {
        drive = -applyJoystickCurve(gamepad1.left_stick_y);
        strafe = applyJoystickCurve(gamepad1.left_stick_x);
        turn = applyJoystickCurve(gamepad1.right_stick_x);

        laucherPower = applyJoystickCurve(gamepad2.right_stick_y);
        intakePower = applyJoystickCurve(gamepad2.left_stick_y);

        rampPosition = gamepad2.a && intakePower == 0.0;

        boolean retractPressed = gamepad2.left_bumper && !prevLeftBumper;
        boolean deployPressed = gamepad2.right_bumper && !prevRightBumper;
        prevLeftBumper = gamepad2.left_bumper;
        prevRightBumper = gamepad2.right_bumper;

        if (retractPressed != deployPressed) {
            double commandedPower = retractPressed ? JAVELIN_POWER : -JAVELIN_POWER;
            if (!javelinRunning || javelinPower != commandedPower) {
                javelinRunning = true;
                javelinTimer.reset();
                javelinPower = commandedPower;
            }
        }

        if (javelinRunning && javelinTimer.seconds() >= JAVELIN_RUN_SECONDS) {
            javelinDeployed = javelinPower < 0.0;
            javelinRunning = false;
            javelinPower = 0.0;
        }
    }

    private double applyJoystickCurve(double gamepadInput) {
        if (Math.abs(gamepadInput) < JOYSTICK_DEADZONE) {
            return 0.0;
        }
        gamepadInput = Math.max(-1.0, Math.min(1.0, gamepadInput));
        return Math.signum(gamepadInput) * (gamepadInput * gamepadInput);
    }

    private void calculateMecanumDrive() {
        frontLeftPower = drive - strafe + turn;
        frontRightPower = drive + strafe - turn;
        backLeftPower = drive + strafe + turn;
        backRightPower = drive - strafe - turn;

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
        javelinRunning = false;
        javelinPower = 0.0;
        javelin.setPower(0);
    }

    private void updateTelemetry() {
        telemetry.addData("Drive", "%.2f", drive);
        telemetry.addData("Strafe", "%.2f", strafe);
        telemetry.addData("Turn", "%.2f", turn);
        telemetry.addData("Intake", "%.2f", intakePower);
        telemetry.addData("Laucher", "%.2f", laucherPower);
        telemetry.addData("Ramp", rampPosition ? "up 180 deg" : "down 90 deg");
        if (javelinRunning) {
            telemetry.addData("Javelin", "%s  %.1f s left",
                    javelinPower < 0.0 ? "deploying" : "retracting",
                    JAVELIN_RUN_SECONDS - javelinTimer.seconds());
        } else {
            telemetry.addData("Javelin", javelinDeployed ? "deployed" : "start");
        }

        telemetry.addLine("Controls");
        telemetry.addLine("GP1 left stick: drive / strafe");
        telemetry.addLine("GP1 right stick: turn");
        telemetry.addLine("GP2 left stick: intake");
        telemetry.addLine("GP2 right stick: launcher");
        telemetry.addLine("GP2 A: ramp up while held, down while intake runs");
        telemetry.addLine("GP2 left bumper: retract javelin");
        telemetry.addLine("GP2 right bumper: deploy javelin");
        telemetry.update();
    }
}
