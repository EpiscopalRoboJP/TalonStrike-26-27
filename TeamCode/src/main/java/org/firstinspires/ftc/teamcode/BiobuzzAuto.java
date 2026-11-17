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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
 * for the whole auto and stops only when the routine ends. RGB and RGB2 follow
 * teleop: purple while the launcher spins, green while the intake runs, yellow
 * while a wheel is commanded, and green on RGB2 once the encoder holds shoot speed.
 * The javelin stroke matches teleop.
 * Feedforward and dead-wheel geometry in MecanumDrive.PARAMS are still placeholders
 * until the tuning opmodes have been run.
 */
@Autonomous(name = "BiobuzzAuto", group = "TeamCode", preselectTeleOp = "BiobuzzTeleop")
public class BiobuzzAuto extends LinearOpMode {
    private DcMotorEx laucher;
    private CRServo intake;
    private CRServo javelin;
    private MecanumDrive drive;
    private RgbIndicator indicator;

    private String stage = "init";
    private final ElapsedTime autoTimer = new ElapsedTime();
    private double launcherTicksPerSec;
    private double launcherPeakTicksPerSec;
    private long launcherPeakMs;

    // Full deploy stroke from BiobuzzTeleop. Longer than this stalls the servo at the stop.
    private static final double JAVELIN_RUN_SECONDS = 13.0;
    private static final double JAVELIN_POWER = 1.0;

    private static final double INTAKE_FEED_POWER = -1.0;

    // Pollen flywheel curve from BiobuzzTeleop.
    private static final double FLYWHEEL_VOLTAGE_HIGH = 13.0;
    private static final double FLYWHEEL_VOLTAGE_MID = 12.6;
    private static final double FLYWHEEL_VOLTAGE_LOW = 12.3;
    private static final double FLYWHEEL_POWER_MOD = 0.0;
    private static final double FLYWHEEL_POWER_HIGH = 0.55 + FLYWHEEL_POWER_MOD;
    private static final double FLYWHEEL_POWER_MID = 0.6 + FLYWHEEL_POWER_MOD;
    private static final double FLYWHEEL_POWER_LOW = 0.65 + FLYWHEEL_POWER_MOD;

    private static final double SHOOT_CUTOFF_SECONDS = 24.0;

    private static final double MOVE_COMMAND = 0.02;
    private static final double LAUNCHER_READY_FRACTION = 0.97;
    private static final double LAUNCHER_MOVE_TICKS_PER_SEC = 40.0;
    private static final double LAUNCHER_MIN_TICKS_PER_SEC = 100.0;
    private static final double LAUNCHER_SPEED_HOLD = 0.92;
    private static final long LAUNCHER_SETTLE_MS = 150;

    @Override
    public void runOpMode() {
        initHardware();

        while (opModeInInit()) {
            drive.updatePoseEstimate();
            updateIndicator();
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
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        laucher = hardwareMap.get(DcMotorEx.class, "Laucher");
        javelin = hardwareMap.get(CRServo.class, "Javelin");
        drive = new MecanumDrive(hardwareMap, AutoProfile.START);

        laucher.setDirection(DcMotor.Direction.FORWARD);
        laucher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Power stays open-loop. The encoder is read for speed only.
        laucher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        indicator = RgbIndicator.attach(hardwareMap);
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
                updateIndicator();
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
                    updateIndicator();
                    updateTelemetry();
                    return false;
                }
                if (startedAt < 0) {
                    startedAt = now();
                }
                double elapsed = now() - startedAt;
                if (elapsed >= totalSeconds) {
                    intake.setPower(0);
                    updateIndicator();
                    updateTelemetry();
                    return false;
                }

                boolean intakeOn = false;
                double cursor = AutoProfile.SPINUP_SECONDS;
                for (int shot = 0; shot < AutoProfile.SHOT_COUNT; shot++) {
                    cursor += AutoProfile.FEED_PULSE_SECONDS;
                    if (shot < AutoProfile.SHOT_COUNT - 1) {
                        double intakeEnd = cursor + AutoProfile.INTAKE_INDEX_SECONDS;
                        if (elapsed >= cursor && elapsed < intakeEnd) {
                            intakeOn = true;
                        }
                        cursor = intakeEnd + AutoProfile.BETWEEN_SHOTS_SECONDS;
                    }
                }
                intake.setPower(intakeOn ? INTAKE_FEED_POWER : 0.0);
                updateIndicator();
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
        if (indicator != null) {
            indicator.off();
        }
    }

    private void updateIndicator() {
        if (indicator == null) {
            return;
        }
        double ticksPerSec = readLauncherTicksPerSec();
        indicator.update(
                robotMoving(),
                Math.abs(intake.getPower()) > MOVE_COMMAND,
                ticksPerSec >= LAUNCHER_MOVE_TICKS_PER_SEC,
                false,
                false,
                false,
                launcherAtShootSpeed(ticksPerSec),
                false);
    }

    private boolean robotMoving() {
        return Math.abs(drive.leftFront.getPower()) > MOVE_COMMAND
                || Math.abs(drive.leftBack.getPower()) > MOVE_COMMAND
                || Math.abs(drive.rightFront.getPower()) > MOVE_COMMAND
                || Math.abs(drive.rightBack.getPower()) > MOVE_COMMAND;
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
     * Green once the encoder stops climbing and holds the speed this pollen power produces.
     */
    private boolean launcherAtShootSpeed(double ticksPerSec) {
        double cap = pollenPower();
        boolean atCap = cap > 0.0 && Math.abs(laucher.getPower()) >= cap * LAUNCHER_READY_FRACTION;
        if (!atCap) {
            launcherPeakTicksPerSec = 0.0;
            launcherPeakMs = 0L;
            return false;
        }
        long nowMs = System.currentTimeMillis();
        if (ticksPerSec > launcherPeakTicksPerSec * 1.02) {
            launcherPeakTicksPerSec = ticksPerSec;
            launcherPeakMs = nowMs;
        }
        return launcherPeakTicksPerSec >= LAUNCHER_MIN_TICKS_PER_SEC
                && ticksPerSec >= launcherPeakTicksPerSec * LAUNCHER_SPEED_HOLD
                && nowMs - launcherPeakMs >= LAUNCHER_SETTLE_MS;
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
        telemetry.addData("Launcher", "%.2f power  %.0f ticks/s", laucher.getPower(), launcherTicksPerSec);
        telemetry.addData("RGB", "%s", indicator.rgbLabel());
        telemetry.addData("RGB2", "%s", indicator.rgb2Label());
        telemetry.addData("Lights", indicator.lightCount());
        telemetry.addData("Pose", "x %.1f in  y %.1f in  h %.1f deg",
                pose.position.x, pose.position.y, Math.toDegrees(pose.heading.toDouble()));
        telemetry.update();
    }
}
