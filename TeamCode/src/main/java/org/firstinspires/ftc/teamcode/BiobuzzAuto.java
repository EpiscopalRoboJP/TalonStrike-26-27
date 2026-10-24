package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import static com.acmerobotics.roadrunner.Actions.now;

/**
 * Autonomous: shoot the 4 preloaded Pollen from the start spot, then park in the Loading Zone.
 * Driving uses Road Runner. Position comes from the three dead-wheel pods on FL, BR, and BL.
 *
 * No driver input after the opmode is selected. Heading stays pointed away from the audience.
 *
 * Place the robot on the red side, touching the audience wall, facing away from
 * the audience. The route and drive limits live in AutoProfile, which MeepMeep reads too.
 * The preload is 4 pollen, so the launcher stays on BiobuzzTeleop's pollen curve
 * for the whole auto and stops only when the routine ends. The javelin stroke matches teleop.
 * Feedforward and dead-wheel geometry in MecanumDrive.PARAMS are still placeholders
 * until the tuning opmodes have been run.
 */
@Autonomous(name = "BiobuzzAuto", group = "TeamCode", preselectTeleOp = "BiobuzzTeleop")
public class BiobuzzAuto extends LinearOpMode {
    private DcMotor laucher;
    private CRServo intake;
    private Servo ramp;
    private CRServo javelin;
    private MecanumDrive drive;

    private String stage = "init";
    private final ElapsedTime autoTimer = new ElapsedTime();

    private static final double RAMP_RANGE_DEGREES = 300.0;
    private static final double RAMP_UP_POSITION = 180.0 / RAMP_RANGE_DEGREES;
    private static final double RAMP_DOWN_POSITION = 90.0 / RAMP_RANGE_DEGREES;
    // Full deploy stroke from BiobuzzTeleop. Longer than this stalls the servo at the stop.
    private static final double JAVELIN_RUN_SECONDS = 13.0;
    private static final double JAVELIN_POWER = 1.0;

    private static final double INTAKE_FEED_POWER = -1.0;

    // Pollen flywheel curve from BiobuzzTeleop. The preload is pollen, so nectar power is never used.
    private static final double FLYWHEEL_VOLTAGE_HIGH = 13.0;
    private static final double FLYWHEEL_VOLTAGE_MID = 12.6;
    private static final double FLYWHEEL_VOLTAGE_LOW = 12.3;
    private static final double FLYWHEEL_POWER_MOD = 0.0;
    private static final double FLYWHEEL_POWER_HIGH = 0.55 + FLYWHEEL_POWER_MOD;
    private static final double FLYWHEEL_POWER_MID = 0.6 + FLYWHEEL_POWER_MOD;
    private static final double FLYWHEEL_POWER_LOW = 0.65 + FLYWHEEL_POWER_MOD;

    private static final double SHOOT_CUTOFF_SECONDS = 24.0;

    @Override
    public void runOpMode() {
        initHardware();

        while (opModeInInit()) {
            drive.updatePoseEstimate();
            updateTelemetry();
        }

        if (!opModeIsActive()) {
            stopActuators();
            return;
        }

        autoTimer.reset();
        drive.localizer.setPose(AutoProfile.START);
        laucher.setPower(pollenPower());

        Actions.runBlocking(new ParallelAction(
                // deployJavelin(),
                new SequentialAction(
                        driveTo(AutoProfile.shootPose(), "approach", false),
                        shoot(),
                        driveTo(AutoProfile.afterLeftPose(), "park", false),
                        driveTo(AutoProfile.parkPose(), "park", false),
                        driveTo(AutoProfile.fullParkPose(), "park", true)
                )
        ));

        stopActuators();
    }

    private void initHardware() {
        intake = hardwareMap.get(CRServo.class, "Intake");
        laucher = hardwareMap.get(DcMotor.class, "Laucher");
        ramp = hardwareMap.get(Servo.class, "Ramp");
        javelin = hardwareMap.get(CRServo.class, "Javelin");
        drive = new MecanumDrive(hardwareMap, AutoProfile.START);

        laucher.setDirection(DcMotor.Direction.FORWARD);
        laucher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        laucher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ramp.setPosition(RAMP_DOWN_POSITION);
        stopActuators();
    }

    private Action driveTo(Pose2d target, String stageName, boolean endsPark) {
        return new Action() {
            private Action inner;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                stage = stageName;
                laucher.setPower(pollenPower());
                if (inner == null) {
                    inner = follow(target.position, target.heading.toDouble());
                }
                boolean running = inner.run(packet);
                if (!running && endsPark) {
                    stage = "parked";
                }
                updateTelemetry();
                return running;
            }
        };
    }

    private Action follow(Vector2d target, double targetHeadingRadians) {
        Pose2d pose = drive.localizer.getPose();
        return AutoProfile.append(drive.actionBuilder(pose), pose, target, targetHeadingRadians).build();
    }

    private Action deployJavelin() {
        return new Action() {
            private double startedAt = -1;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (startedAt < 0) {
                    startedAt = now();
                    javelin.setPower(-JAVELIN_POWER);
                }
                if (now() - startedAt >= JAVELIN_RUN_SECONDS) {
                    javelin.setPower(0);
                    return false;
                }
                return true;
            }
        };
    }

    private Action shoot() {
        final double totalSeconds = AutoProfile.shootSeconds();
        return new Action() {
            private double startedAt = -1;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                stage = "shoot";
                laucher.setPower(pollenPower());
                if (autoTimer.seconds() >= SHOOT_CUTOFF_SECONDS) {
                    intake.setPower(0);
                    ramp.setPosition(RAMP_DOWN_POSITION);
                    return false;
                }
                if (startedAt < 0) {
                    startedAt = now();
                }
                double elapsed = now() - startedAt;
                if (elapsed >= totalSeconds) {
                    intake.setPower(0);
                    ramp.setPosition(RAMP_DOWN_POSITION);
                    return false;
                }

                boolean rampUp = false;
                boolean intakeOn = false;
                double cursor = AutoProfile.SPINUP_SECONDS;
                for (int shot = 0; shot < AutoProfile.SHOT_COUNT; shot++) {
                    double feedEnd = cursor + AutoProfile.FEED_PULSE_SECONDS;
                    if (elapsed >= cursor && elapsed < feedEnd) {
                        rampUp = true;
                    }
                    cursor = feedEnd;
                    if (shot < AutoProfile.SHOT_COUNT - 1) {
                        double intakeEnd = cursor + AutoProfile.INTAKE_INDEX_SECONDS;
                        if (elapsed >= cursor && elapsed < intakeEnd) {
                            intakeOn = true;
                        }
                        cursor = intakeEnd + AutoProfile.BETWEEN_SHOTS_SECONDS;
                    }
                }
                ramp.setPosition(rampUp ? RAMP_UP_POSITION : RAMP_DOWN_POSITION);
                intake.setPower(intakeOn ? INTAKE_FEED_POWER : 0.0);
                updateTelemetry();
                return true;
            }
        };
    }

    private void stopActuators() {
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
        intake.setPower(0);
        laucher.setPower(0);
        javelin.setPower(0);
    }

    private double pollenPower() {
        double voltage = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double reading = sensor.getVoltage();
            if (reading > 0) {
                voltage = Math.min(voltage, reading);
            }
        }
        if (voltage == Double.POSITIVE_INFINITY) {
            voltage = FLYWHEEL_VOLTAGE_HIGH;
        }
        return flywheelPowerForVoltage(voltage);
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

    private void updateTelemetry() {
        Pose2d pose = drive.localizer.getPose();
        telemetry.addData("Stage", stage);
        telemetry.addData("Auto time", "%.1f s", opModeIsActive() ? autoTimer.seconds() : 0.0);
        telemetry.addData("Launcher", "%.2f", laucher.getPower());
        telemetry.addData("Pose", "x %.1f in  y %.1f in  h %.1f deg",
                pose.position.x, pose.position.y, Math.toDegrees(pose.heading.toDouble()));
        telemetry.update();
    }
}
