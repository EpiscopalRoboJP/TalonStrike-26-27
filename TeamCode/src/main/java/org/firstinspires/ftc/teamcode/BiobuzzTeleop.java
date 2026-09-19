package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

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

    private double frontLeftPower;
    private double frontRightPower;
    private double backLeftPower;
    private double backRightPower;
    private double intakePower;
    private double drive;
    private double strafe;
    private double turn;

    private static final double JOYSTICK_DEADZONE = 0.05;

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

        stopActuators();
    }

    private void getGamepadInputs() {
        drive = -applyJoystickCurve(gamepad1.left_stick_y);
        strafe = applyJoystickCurve(gamepad1.left_stick_x);
        turn = applyJoystickCurve(gamepad1.right_stick_x);

        if (gamepad1.dpad_up) {
            intakePower = 1.0;
        } else if (gamepad1.dpad_down) {
            intakePower = -1.0;
        } else {
            intakePower = 0;
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
        frontLeftPower = drive + strafe + turn;
        frontRightPower = drive - strafe - turn;
        backLeftPower = drive - strafe + turn;
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
    }

    private void stopActuators() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        intake.setPower(0);
    }

    private void updateTelemetry() {
        telemetry.addData("Drive", "%.2f", drive);
        telemetry.addData("Strafe", "%.2f", strafe);
        telemetry.addData("Turn", "%.2f", turn);
        telemetry.addData("Intake", "%.2f", intakePower);
        telemetry.update();
    }
}
