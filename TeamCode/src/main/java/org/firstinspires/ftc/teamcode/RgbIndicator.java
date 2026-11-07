package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * goBILDA RGB Indicator Light (3118-0808-0002) on a servo port.
 * Configure it as a Servo named "RGB". The hub's normal 500-2500 us range
 * maps positions below 0.30 to off, 0.30-0.70 through red to magenta, and
 * above 0.70 to white. These positions stay inside the solid colors.
 *
 * One color at a time, highest match first:
 * fight rainbow, nectar, defense hold, then pollen counts when the camera
 * has a reading. Purple and orange do not need the camera. With no reading
 * the ball colors stay off. A low battery blinks red only when that would
 * otherwise be off or empty.
 */
public final class RgbIndicator {
    public static final String DEVICE_NAME = "RGB";

    static final double OFF = 0.0;
    static final double RED = 0.33;
    static final double ORANGE = 0.39;
    static final double YELLOW = 0.45;
    static final double GREEN = 0.50;
    static final double PURPLE = 0.66;
    static final double RAINBOW_START = 0.32;
    static final double RAINBOW_END = 0.69;

    private static final long BLINK_HALF_PERIOD_MS = 400;
    private static final long RAINBOW_PERIOD_MS = 80;

    private final Servo light;
    private String label = "off";

    private RgbIndicator(Servo light) {
        this.light = light;
        if (light != null) {
            light.setPosition(OFF);
        }
    }

    public static RgbIndicator attach(HardwareMap hardwareMap) {
        try {
            return new RgbIndicator(hardwareMap.get(Servo.class, DEVICE_NAME));
        } catch (RuntimeException exception) {
            return new RgbIndicator(null);
        }
    }

    public void update(
            boolean fighting,
            boolean nectarMode,
            boolean defenseHolding,
            boolean lowBattery,
            boolean ballReading,
            int frontCount,
            int storageCount) {
        if (light == null) {
            label = "not configured";
            return;
        }

        Choice choice = choose(
                fighting,
                nectarMode,
                defenseHolding,
                lowBattery,
                ballReading,
                frontCount,
                storageCount);
        label = choice.label;
        if (choice.rainbow) {
            light.setPosition(rainbowPosition());
            return;
        }
        boolean lit = !choice.blink || blinkLit();
        light.setPosition(lit ? choice.position : OFF);
    }

    public void off() {
        label = light == null ? "not configured" : "off";
        if (light != null) {
            light.setPosition(OFF);
        }
    }

    public String label() {
        return label;
    }

    static Choice choose(
            boolean fighting,
            boolean nectarMode,
            boolean defenseHolding,
            boolean lowBattery,
            boolean ballReading,
            int frontCount,
            int storageCount) {
        if (fighting) {
            return new Choice(RAINBOW_START, false, true, "fighting");
        }
        if (nectarMode) {
            if (defenseHolding) {
                return new Choice(PURPLE, true, "nectar, defense hold");
            }
            return new Choice(PURPLE, false, "nectar");
        }
        if (defenseHolding) {
            return new Choice(ORANGE, true, "defense hold");
        }
        if (ballReading && frontCount > 0) {
            return new Choice(GREEN, false, "front loaded");
        }
        if (ballReading && storageCount > 0) {
            return new Choice(YELLOW, false, "storage only");
        }
        if (ballReading && lowBattery) {
            return new Choice(RED, true, "empty, low battery");
        }
        if (ballReading) {
            return new Choice(RED, false, "empty");
        }
        if (lowBattery) {
            return new Choice(RED, true, "low battery");
        }
        return new Choice(OFF, false, "off");
    }

    private static boolean blinkLit() {
        return (System.currentTimeMillis() / BLINK_HALF_PERIOD_MS) % 2 == 0;
    }

    private static double rainbowPosition() {
        double phase = (System.currentTimeMillis() % RAINBOW_PERIOD_MS) / (double) RAINBOW_PERIOD_MS;
        return RAINBOW_START + phase * (RAINBOW_END - RAINBOW_START);
    }

    static final class Choice {
        final double position;
        final boolean blink;
        final boolean rainbow;
        final String label;

        Choice(double position, boolean blink, String label) {
            this(position, blink, false, label);
        }

        Choice(double position, boolean blink, boolean rainbow, String label) {
            this.position = position;
            this.blink = blink;
            this.rainbow = rainbow;
            this.label = label;
        }
    }
}
