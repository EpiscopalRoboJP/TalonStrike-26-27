package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import static com.acmerobotics.roadrunner.Actions.now;

/**
 * Shared autonomous runner. Subclasses only choose the route.
 * Shooting feeds with the gate, indexes with the intake, and jostles between shots.
 * The A1 sweep backs the intake into the audience wall, then pulls clear.
 */
public abstract class BiobuzzAutoBase extends LinearOpMode {
    private static final int PHASE_SPINUP = 0;
    private static final int PHASE_FEED = 1;
    private static final int PHASE_INDEX = 2;
    private static final int PHASE_JOSTLE = 3;

    // Full deploy stroke from BiobuzzTeleop. Longer than this stalls the servo at the stop.
    private static final double JAVELIN_RUN_SECONDS = 13.0;

    protected DcMotorEx laucher;
    protected CRServo intake;
    protected CRServo gate;
    protected CRServo javelin;
    protected MecanumDrive drive;
    protected RgbIndicator indicator;

    protected String stage = "init";
    protected final ElapsedTime autoTimer = new ElapsedTime();
    private double launcherTicksPerSec;
    private boolean launcherAtSpeed;
    private final BiobuzzMechanisms.LauncherSpeedTracker launcherSpeed =
            new BiobuzzMechanisms.LauncherSpeedTracker();

    @Override
    public void runOpMode() {
        AutoTuningStore.loadOnce();
        initHardware();

        while (opModeInInit()) {
            handleTuningPersistenceToggles();
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
        runAuto();
        stopActuators();
    }

    protected abstract void runAuto();

    private void initHardware() {
        intake = hardwareMap.get(CRServo.class, "Intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        laucher = hardwareMap.get(DcMotorEx.class, "Laucher");
        gate = hardwareMap.get(CRServo.class, "Gate");
        javelin = hardwareMap.get(CRServo.class, "Javelin");
        drive = new MecanumDrive(hardwareMap, AutoProfile.START);

        laucher.setDirection(DcMotor.Direction.FORWARD);
        laucher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Power stays open-loop. The encoder is read for speed only.
        laucher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        indicator = RgbIndicator.attach(hardwareMap);
        stopActuators();
    }

    /**
     * @param launcherOn pollen flywheel while this move runs
     * @param intakeOn intake while this move runs; the gate stays stopped
     */
    protected Action driveTo(
            Pose2d target,
            String stageName,
            boolean endsPark,
            boolean launcherOn,
            boolean intakeOn) {
        return new Action() {
            private Action inner;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                stage = stageName;
                laucher.setPower(launcherOn ? pollenPower() : 0.0);
                intake.setPower(intakeOn ? BiobuzzMechanisms.INTAKE_FEED_POWER : 0.0);
                gate.setPower(0.0);
                if (inner == null) {
                    Pose2d pose = drive.localizer.getPose();
                    inner = AutoProfile.append(
                            drive.actionBuilder(pose),
                            pose,
                            target.position,
                            target.heading.toDouble()).build();
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

    /**
     * One A1 station: line up off the wall, back the intake into it, press, pull clear.
     * Skipped once the sweep cutoff has passed so the return shot and park still fit.
     */
    protected Action sweepIfTime(final double stationX) {
        return new Action() {
            private Action inner;
            private boolean decided;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!decided) {
                    decided = true;
                    if (autoTimer.seconds() >= BiobuzzMechanisms.SWEEP_CUTOFF_SECONDS) {
                        return false;
                    }
                    inner = wallBumpIntake(stationX);
                }
                return inner.run(packet);
            }
        };
    }

    private Action wallBumpIntake(double stationX) {
        Pose2d standoff = AutoProfile.sweepStandoffPose(stationX);
        Pose2d wall = AutoProfile.sweepWallPose(stationX);
        return new SequentialAction(
                driveTo(standoff, "intake", false, false, true),
                driveTo(wall, "intake", false, false, true),
                bumpPress(),
                driveTo(standoff, "intake", false, false, true));
    }

    /**
     * Timed backup into the wall. Road Runner's profile would finish against the
     * wall and stall the wheels, so this press is open-loop and the dead wheels
     * keep integrating.
     */
    private Action bumpPress() {
        return new Action() {
            private double startedAt = -1;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                stage = "intake";
                laucher.setPower(0.0);
                gate.setPower(0.0);
                intake.setPower(BiobuzzMechanisms.INTAKE_FEED_POWER);
                drive.updatePoseEstimate();
                if (startedAt < 0) {
                    startedAt = now();
                }
                if (now() - startedAt >= AutoProfile.BUMP_HOLD_SECONDS) {
                    drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                    updateIndicator();
                    updateTelemetry();
                    return false;
                }
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(-BiobuzzMechanisms.BUMP_BACK_POWER, 0), 0));
                updateIndicator();
                updateTelemetry();
                return true;
            }
        };
    }

    /**
     * Spin up (or until the encoder holds speed), then for each pollen: gate forward,
     * intake index, and a symmetric forward/back jostle. The jostle uses the
     * between-shots window. Cut off early so the park still runs.
     */
    protected Action shoot(final String stageName) {
        return new Action() {
            private double phaseStart = -1;
            private int phase = PHASE_SPINUP;
            private int shot;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                stage = stageName;
                double cap = pollenPower();
                laucher.setPower(cap);
                drive.updatePoseEstimate();
                double ticks = readLauncherTicksPerSec();
                launcherAtSpeed = launcherSpeed.atShootSpeed(laucher.getPower(), cap, ticks);
                if (autoTimer.seconds() >= BiobuzzMechanisms.SHOOT_CUTOFF_SECONDS) {
                    idleShotMechanisms();
                    updateIndicator();
                    updateTelemetry();
                    return false;
                }
                if (phaseStart < 0) {
                    phaseStart = now();
                }
                if (!advanceShotPhases()) {
                    idleShotMechanisms();
                    updateIndicator();
                    updateTelemetry();
                    return false;
                }

                boolean intakeOn = false;
                double gatePower = 0.0;
                double jostle = 0.0;
                double phaseElapsed = now() - phaseStart;
                if (phase == PHASE_FEED) {
                    gatePower = BiobuzzMechanisms.GATE_POWER;
                } else if (phase == PHASE_INDEX) {
                    intakeOn = true;
                } else if (phase == PHASE_JOSTLE && AutoProfile.BETWEEN_SHOTS_SECONDS > 0.0) {
                    double half = AutoProfile.BETWEEN_SHOTS_SECONDS / 2.0;
                    jostle = phaseElapsed < half
                            ? BiobuzzMechanisms.JOSTLE_POWER
                            : -BiobuzzMechanisms.JOSTLE_POWER;
                }
                intake.setPower(intakeOn ? BiobuzzMechanisms.INTAKE_FEED_POWER : 0.0);
                gate.setPower(gatePower);
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(jostle, 0), 0));
                updateIndicator();
                updateTelemetry();
                return true;
            }

            private boolean advanceShotPhases() {
                int steps = 0;
                int stepLimit = Math.max(1, AutoProfile.SHOT_COUNT) + 3;
                boolean advanced = true;
                while (advanced && steps < stepLimit) {
                    advanced = false;
                    steps++;
                    double phaseElapsed = now() - phaseStart;
                    if (phase == PHASE_SPINUP) {
                        boolean spun = launcherAtSpeed || phaseElapsed >= AutoProfile.SPINUP_SECONDS;
                        if (!spun) {
                            continue;
                        }
                        if (AutoProfile.SHOT_COUNT <= 0) {
                            return false;
                        }
                        phase = PHASE_FEED;
                        phaseStart = now();
                        advanced = true;
                    } else if (phase == PHASE_FEED) {
                        if (phaseElapsed < AutoProfile.FEED_PULSE_SECONDS) {
                            continue;
                        }
                        if (shot >= AutoProfile.SHOT_COUNT - 1) {
                            return false;
                        }
                        phase = PHASE_INDEX;
                        phaseStart = now();
                        advanced = true;
                    } else if (phase == PHASE_INDEX) {
                        if (phaseElapsed < AutoProfile.INTAKE_INDEX_SECONDS) {
                            continue;
                        }
                        phase = PHASE_JOSTLE;
                        phaseStart = now();
                        advanced = true;
                    } else if (phaseElapsed >= AutoProfile.BETWEEN_SHOTS_SECONDS) {
                        shot++;
                        phase = PHASE_FEED;
                        phaseStart = now();
                        advanced = true;
                    }
                }
                return true;
            }
        };
    }

    protected Action deployJavelin() {
        return new Action() {
            private double startedAt = -1;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (startedAt < 0) {
                    startedAt = now();
                    javelin.setPower(-BiobuzzMechanisms.JAVELIN_POWER);
                }
                if (now() - startedAt >= JAVELIN_RUN_SECONDS) {
                    javelin.setPower(0);
                    return false;
                }
                return true;
            }
        };
    }

    private void idleShotMechanisms() {
        intake.setPower(0);
        gate.setPower(0);
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
    }

    private void stopActuators() {
        if (drive != null) {
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
        }
        if (intake != null) {
            intake.setPower(0);
        }
        if (laucher != null) {
            laucher.setPower(0);
        }
        if (gate != null) {
            gate.setPower(0);
        }
        if (javelin != null) {
            javelin.setPower(0);
        }
        if (indicator != null) {
            indicator.off();
        }
    }

    private void handleTuningPersistenceToggles() {
        if (BiobuzzMechanisms.SAVE_TUNING_TO_HUB) {
            BiobuzzMechanisms.SAVE_TUNING_TO_HUB = false;
            AutoTuningStore.saveCurrent();
        }
        if (BiobuzzMechanisms.CLEAR_SAVED_TUNING) {
            BiobuzzMechanisms.CLEAR_SAVED_TUNING = false;
            AutoTuningStore.clearSaved();
        }
    }

    private double pollenPower() {
        return BiobuzzMechanisms.pollenPower(hardwareMap);
    }

    private void updateIndicator() {
        if (indicator == null || laucher == null) {
            return;
        }
        double ticksPerSec = readLauncherTicksPerSec();
        launcherAtSpeed = launcherSpeed.atShootSpeed(laucher.getPower(), pollenPower(), ticksPerSec);
        indicator.update(
                robotMoving(),
                Math.abs(intake.getPower()) > BiobuzzMechanisms.MOVE_COMMAND,
                ticksPerSec >= BiobuzzMechanisms.LAUNCHER_MOVE_TICKS_PER_SEC,
                false,
                false,
                false,
                launcherAtSpeed,
                false);
    }

    private boolean robotMoving() {
        return Math.abs(drive.leftFront.getPower()) > BiobuzzMechanisms.MOVE_COMMAND
                || Math.abs(drive.leftBack.getPower()) > BiobuzzMechanisms.MOVE_COMMAND
                || Math.abs(drive.rightFront.getPower()) > BiobuzzMechanisms.MOVE_COMMAND
                || Math.abs(drive.rightBack.getPower()) > BiobuzzMechanisms.MOVE_COMMAND;
    }

    private double readLauncherTicksPerSec() {
        if (laucher == null) {
            launcherTicksPerSec = 0.0;
            return 0.0;
        }
        launcherTicksPerSec = Math.abs(laucher.getVelocity());
        return launcherTicksPerSec;
    }

    private void updateTelemetry() {
        Pose2d pose = drive.localizer.getPose();
        telemetry.addData("Stage", stage);
        telemetry.addData("Tuning", AutoTuningStore.statusLabel());
        telemetry.addData("Battery", "%.2f V  Shoot plan: %.1f s",
                BiobuzzMechanisms.readBatteryVoltage(hardwareMap), AutoProfile.shootSeconds());
        telemetry.addData("Auto time", "%.1f s", opModeIsActive() ? autoTimer.seconds() : 0.0);
        telemetry.addData("Launcher", "%.2f power  %.0f ticks/s", laucher.getPower(), launcherTicksPerSec);
        telemetry.addData("Gate", "%.2f", gate.getPower());
        telemetry.addData("Intake", "%.2f", intake.getPower());
        telemetry.addData("RGB", "%s", indicator.rgbLabel());
        telemetry.addData("RGB2", "%s", indicator.rgb2Label());
        telemetry.addData("Lights", indicator.lightCount());
        telemetry.addData("Pose", "x %.1f in  y %.1f in  h %.1f deg",
                pose.position.x, pose.position.y, Math.toDegrees(pose.heading.toDouble()));
        telemetry.update();
    }
}
