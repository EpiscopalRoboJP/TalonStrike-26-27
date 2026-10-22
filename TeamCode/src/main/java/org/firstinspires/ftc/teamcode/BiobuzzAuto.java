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
import com.qualcomm.robotcore.util.ElapsedTime;

import static com.acmerobotics.roadrunner.Actions.now;

/**
 * Autonomous: shoot the 4 preloaded Pollen from the start spot, then park in the Loading Zone.
 * Driving uses Road Runner. Position comes from the three dead-wheel pods on FL, BR, and BL.
 *
 * No driver input after the opmode is selected. The robot aims at the Cell that starts facing up.
 * X is forward and Y is left, both measured from the start pose. Positive heading is counter-clockwise.
 *
 * Before this is accurate, run the quickstart tuning opmodes and copy the results into
 * MecanumDrive.PARAMS and ThreeDeadWheelLocalizer.PARAMS. The values there now are placeholders.
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
    private static final double JAVELIN_RUN_SECONDS = 20.0;
    private static final double JAVELIN_POWER = 1.0;

    private static final double LAUNCH_POWER = 1.0;
    private static final double INTAKE_FEED_POWER = 1.0;
    private static final double SPINUP_SECONDS = 1.5;
    private static final double FEED_PULSE_SECONDS = 0.4;
    private static final double INTAKE_INDEX_SECONDS = 0.6;
    private static final double BETWEEN_SHOTS_SECONDS = 0.5;
    private static final int SHOT_COUNT = 4;

    private static final double CELL_HEADING_DEG = 0.0;

    private static final double PARK_X_IN = 24.0;
    private static final double PARK_Y_IN = 0.0;
    private static final double PARK_HEADING_DEG = 0.0;

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
        drive.localizer.setPose(new Pose2d(0, 0, 0));
        laucher.setPower(LAUNCH_POWER);

        Actions.runBlocking(new ParallelAction(
                deployJavelin(),
                new SequentialAction(
                        aim(Math.toRadians(CELL_HEADING_DEG)),
                        shoot(),
                        park()
                )
        ));

        stopActuators();
    }

    private void initHardware() {
        intake = hardwareMap.get(CRServo.class, "Intake");
        laucher = hardwareMap.get(DcMotor.class, "Laucher");
        ramp = hardwareMap.get(Servo.class, "Ramp");
        javelin = hardwareMap.get(CRServo.class, "Javelin");
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        laucher.setDirection(DcMotor.Direction.FORWARD);
        laucher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        laucher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ramp.setPosition(RAMP_DOWN_POSITION);
        stopActuators();
    }

    private Action aim(double aimRadians) {
        return new Action() {
            private Action inner;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                stage = "aim";
                if (inner == null) {
                    inner = drive.actionBuilder(drive.localizer.getPose())
                            .turn(turnDelta(aimRadians))
                            .build();
                }
                boolean running = inner.run(packet);
                updateTelemetry();
                return running;
            }
        };
    }

    private Action park() {
        return new Action() {
            private Action inner;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                stage = "park";
                if (inner == null) {
                    inner = drive.actionBuilder(drive.localizer.getPose())
                            .strafeTo(new Vector2d(PARK_X_IN, PARK_Y_IN))
                            .turn(turnDelta(Math.toRadians(PARK_HEADING_DEG)))
                            .build();
                }
                boolean running = inner.run(packet);
                if (!running) {
                    stage = "parked";
                }
                updateTelemetry();
                return running;
            }
        };
    }

    private double turnDelta(double targetRadians) {
        double delta = targetRadians - drive.localizer.getPose().heading.toDouble();
        while (delta > Math.PI) {
            delta -= 2.0 * Math.PI;
        }
        while (delta < -Math.PI) {
            delta += 2.0 * Math.PI;
        }
        return delta;
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
        final double totalSeconds = SPINUP_SECONDS
                + SHOT_COUNT * FEED_PULSE_SECONDS
                + (SHOT_COUNT - 1) * (INTAKE_INDEX_SECONDS + BETWEEN_SHOTS_SECONDS);
        return new Action() {
            private double startedAt = -1;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                stage = "shoot";
                if (autoTimer.seconds() >= SHOOT_CUTOFF_SECONDS) {
                    intake.setPower(0);
                    ramp.setPosition(RAMP_DOWN_POSITION);
                    laucher.setPower(0);
                    return false;
                }
                if (startedAt < 0) {
                    startedAt = now();
                }
                double elapsed = now() - startedAt;
                if (elapsed >= totalSeconds) {
                    intake.setPower(0);
                    ramp.setPosition(RAMP_DOWN_POSITION);
                    laucher.setPower(0);
                    return false;
                }

                boolean rampUp = false;
                boolean intakeOn = false;
                double cursor = SPINUP_SECONDS;
                for (int shot = 0; shot < SHOT_COUNT; shot++) {
                    double feedEnd = cursor + FEED_PULSE_SECONDS;
                    if (elapsed >= cursor && elapsed < feedEnd) {
                        rampUp = true;
                    }
                    cursor = feedEnd;
                    if (shot < SHOT_COUNT - 1) {
                        double intakeEnd = cursor + INTAKE_INDEX_SECONDS;
                        if (elapsed >= cursor && elapsed < intakeEnd) {
                            intakeOn = true;
                        }
                        cursor = intakeEnd + BETWEEN_SHOTS_SECONDS;
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

    private void updateTelemetry() {
        Pose2d pose = drive.localizer.getPose();
        telemetry.addData("Stage", stage);
        telemetry.addData("Auto time", "%.1f s", opModeIsActive() ? autoTimer.seconds() : 0.0);
        telemetry.addData("Pose", "x %.1f in  y %.1f in  h %.1f deg",
                pose.position.x, pose.position.y, Math.toDegrees(pose.heading.toDouble()));
        telemetry.update();
    }
}
