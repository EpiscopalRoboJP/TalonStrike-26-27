package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

/**
 * Intake, gate, and pollen flywheel shared by BiobuzzTeleop and the autonomous modes.
 * FTC Dashboard edits here are what both modes run. AutoTuningStore persists them.
 */
@Config
public final class BiobuzzMechanisms {
    private BiobuzzMechanisms() {
    }

    public static final double GATE_POWER = 0.5;
    public static final double JAVELIN_POWER = 1.0;
    public static final double MOVE_COMMAND = 0.02;

    public static final double LAUNCHER_READY_FRACTION = 0.97;
    public static final double LAUNCHER_MOVE_TICKS_PER_SEC = 40.0;
    public static final double LAUNCHER_MIN_TICKS_PER_SEC = 100.0;
    public static final double LAUNCHER_SPEED_HOLD = 0.92;
    public static final long LAUNCHER_SETTLE_MS = 150;

    public static final double FLYWHEEL_VOLTAGE_HIGH = 13.0;
    public static final double FLYWHEEL_VOLTAGE_MID = 12.6;
    public static final double FLYWHEEL_VOLTAGE_LOW = 12.3;

    public static double FLYWHEEL_POWER_MOD = 0.0;
    public static double FLYWHEEL_POWER_HIGH = 0.55;
    public static double FLYWHEEL_POWER_MID = 0.60;
    public static double FLYWHEEL_POWER_LOW = 0.65;

    public static double INTAKE_FEED_POWER = -1.0;

    public static double SHOOT_CUTOFF_SECONDS = 24.0;
    public static double SWEEP_CUTOFF_SECONDS = 16.0;
    public static double BUMP_BACK_POWER = 0.28;
    public static double JOSTLE_POWER = 0.22;

    public static boolean SAVE_TUNING_TO_HUB = false;
    public static boolean CLEAR_SAVED_TUNING = false;

    public static double readBatteryVoltage(HardwareMap hardwareMap) {
        double voltage = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double reading = sensor.getVoltage();
            if (reading > 0) {
                voltage = Math.min(voltage, reading);
            }
        }
        if (voltage == Double.POSITIVE_INFINITY) {
            return FLYWHEEL_VOLTAGE_HIGH;
        }
        return voltage;
    }

    public static double pollenPower(HardwareMap hardwareMap) {
        return flywheelPowerForVoltage(readBatteryVoltage(hardwareMap));
    }

    /**
     * Pollen flywheel curve. Above 13.0 V uses the high power, the 12.3-12.6 V
     * band blends mid to low, and anything under 12.3 V uses the low power.
     */
    public static double flywheelPowerForVoltage(double voltage) {
        double base;
        if (voltage >= FLYWHEEL_VOLTAGE_HIGH) {
            base = FLYWHEEL_POWER_HIGH;
        } else if (voltage >= FLYWHEEL_VOLTAGE_MID) {
            base = FLYWHEEL_POWER_MID;
        } else if (voltage <= FLYWHEEL_VOLTAGE_LOW) {
            base = FLYWHEEL_POWER_LOW;
        } else {
            double span = FLYWHEEL_VOLTAGE_MID - FLYWHEEL_VOLTAGE_LOW;
            double drop = (FLYWHEEL_VOLTAGE_MID - voltage) / span;
            base = FLYWHEEL_POWER_MID + drop * (FLYWHEEL_POWER_LOW - FLYWHEEL_POWER_MID);
        }
        return Math.max(0.0, Math.min(1.0, base + FLYWHEEL_POWER_MOD));
    }

    /**
     * Green once the encoder stops climbing and holds the speed this pollen power produces.
     * {@code cap} is the voltage curve for this moment. {@code commandedPower} is what was just requested.
     */
    public static final class LauncherSpeedTracker {
        private double peakTicksPerSec;
        private long peakMs;

        public boolean atShootSpeed(double commandedPower, double cap, double ticksPerSec) {
            boolean atCap = cap > 0.0
                    && Math.abs(commandedPower) >= cap * LAUNCHER_READY_FRACTION;
            if (!atCap) {
                peakTicksPerSec = 0.0;
                peakMs = 0L;
                return false;
            }
            long nowMs = System.currentTimeMillis();
            if (ticksPerSec > peakTicksPerSec * 1.02) {
                peakTicksPerSec = ticksPerSec;
                peakMs = nowMs;
            }
            return peakTicksPerSec >= LAUNCHER_MIN_TICKS_PER_SEC
                    && ticksPerSec >= peakTicksPerSec * LAUNCHER_SPEED_HOLD
                    && nowMs - peakMs >= LAUNCHER_SETTLE_MS;
        }
    }
}
