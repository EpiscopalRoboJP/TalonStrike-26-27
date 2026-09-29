package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

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
    private boolean prevJavelinButton;
    // 0 forward, 1 stop, 2 reverse, 3 stop. Starts stopped so the first B press runs forward.
    private int javelinState = 3;

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

        stopActuators();
    }

    private void getGamepadInputs() {
        drive = -applyJoystickCurve(gamepad1.left_stick_y);
        strafe = applyJoystickCurve(gamepad1.left_stick_x);
        turn = applyJoystickCurve(gamepad1.right_stick_x);

        laucherPower = applyJoystickCurve(gamepad2.right_stick_y);
        intakePower = applyJoystickCurve(gamepad2.left_stick_y);

        rampPosition = gamepad2.a;

        if (gamepad2.b && !prevJavelinButton) {
            javelinState = (javelinState + 1) % 4;
        }
        prevJavelinButton = gamepad2.b;

        if (javelinState == 0) {
            javelinPower = 1.0;
        } else if (javelinState == 2) {
            javelinPower = -1.0;
        } else {
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
        laucher.setPower(laucherPower);
    }

    private void updateServoPositions() {
        ramp.setPosition(rampPosition ? 1.0 : 0.0);
        javelin.setPower(javelinPower);
    }

    private void stopActuators() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        intake.setPower(0);
        laucher.setPower(0);
        javelinState = 3;
        javelinPower = 0.0;
        javelin.setPower(0);
    }

    private void updateTelemetry() {
        telemetry.addData("Drive", "%.2f", drive);
        telemetry.addData("Strafe", "%.2f", strafe);
        telemetry.addData("Turn", "%.2f", turn);
        telemetry.addData("Intake", "%.2f", intakePower);
        telemetry.addData("Laucher", "%.2f", laucherPower);
        telemetry.addData("Ramp", "%.2f", rampPosition);
        telemetry.addData("Javelin", "Power: %.2f  State: %d", javelinPower, javelinState);
   
        telemetry.addLine("Controls");
        telemetry.addLine("GP1 left stick: drive / strafe");
        telemetry.addLine("GP1 right stick: turn");
        telemetry.addLine("GP2 left stick: intake");
        telemetry.addLine("GP2 right stick: launcher");
        telemetry.addLine("GP2 A: ramp up while held");
        telemetry.addLine("GP2 B: javelin forward, stop, reverse, stop");
        telemetry.update();
    }
}
