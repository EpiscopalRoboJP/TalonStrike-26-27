package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Random;

/**
 * Two goBILDA RGB Indicator Lights (3118-0808-0002).
 * Configure them as Servos named "RGB" and "RGB2".
 * The hub's normal 500-2500 us range maps positions below 0.30 to off,
 * 0.30-0.70 through red to magenta, and above 0.70 to white.
 *
 * RGB shows what is moving, launcher first, then intake, then drive.
 * RGB2 is red in defense mode, otherwise green once the launcher is at shoot speed.
 * Rainbow mode picks a new color for each light on its own.
 */
public final class RgbIndicator {
    public static final String DEVICE_NAME = "RGB";
    public static final String DEVICE_NAME_2 = "RGB2";

    static final double OFF = 0.0;
    static final double RED = 0.33;
    static final double YELLOW = 0.45;
    static final double GREEN = 0.50;
    static final double PURPLE = 0.66;
    static final double COLOR_MIN = 0.32;
    static final double COLOR_MAX = 0.69;

    private static final long PARTY_STEP_MS = 180;
    private static final double PARTY_SEPARATION = 0.06;

    private final Servo rgb;
    private final Servo rgb2;
    private final Random random = new Random();
    private double partyRgb = YELLOW;
    private double partyRgb2 = PURPLE;
    private long nextPartyMs;
    private String rgbLabel = "off";
    private String rgb2Label = "off";

    private RgbIndicator(Servo rgb, Servo rgb2) {
        this.rgb = rgb;
        this.rgb2 = rgb2;
        set(rgb, OFF);
        set(rgb2, OFF);
    }

    public static RgbIndicator attach(HardwareMap hardwareMap) {
        return new RgbIndicator(find(hardwareMap, DEVICE_NAME), find(hardwareMap, DEVICE_NAME_2));
    }

    public int lightCount() {
        int count = 0;
        if (rgb != null) {
            count++;
        }
        if (rgb2 != null) {
            count++;
        }
        return count;
    }

    public void update(
            boolean robotMoving,
            boolean intakeMoving,
            boolean launcherMoving,
            boolean defenseMode,
            boolean launcherAtSpeed,
            boolean party) {
        if (party) {
            rollPartyColors();
            rgbLabel = "rainbow";
            rgb2Label = "rainbow";
            set(rgb, partyRgb);
            set(rgb2, partyRgb2);
            return;
        }

        double rgbPosition = rgbPosition(launcherMoving, intakeMoving, robotMoving);
        double rgb2Position = rgb2Position(defenseMode, launcherAtSpeed);
        rgbLabel = rgbLabel(launcherMoving, intakeMoving, robotMoving);
        rgb2Label = rgb2Label(defenseMode, launcherAtSpeed);
        set(rgb, rgbPosition);
        set(rgb2, rgb2Position);
    }

    public void off() {
        rgbLabel = rgb == null ? "not configured" : "off";
        rgb2Label = rgb2 == null ? "not configured" : "off";
        set(rgb, OFF);
        set(rgb2, OFF);
    }

    public String rgbLabel() {
        return rgbLabel;
    }

    public String rgb2Label() {
        return rgb2Label;
    }

    static double rgbPosition(boolean launcherMoving, boolean intakeMoving, boolean robotMoving) {
        if (launcherMoving) {
            return PURPLE;
        }
        if (intakeMoving) {
            return GREEN;
        }
        if (robotMoving) {
            return YELLOW;
        }
        return OFF;
    }

    static double rgb2Position(boolean defenseMode, boolean launcherAtSpeed) {
        if (defenseMode) {
            return RED;
        }
        if (launcherAtSpeed) {
            return GREEN;
        }
        return OFF;
    }

    private static String rgbLabel(boolean launcherMoving, boolean intakeMoving, boolean robotMoving) {
        if (launcherMoving) {
            return "purple launcher";
        }
        if (intakeMoving) {
            return "green intake";
        }
        if (robotMoving) {
            return "yellow drive";
        }
        return "off";
    }

    private static String rgb2Label(boolean defenseMode, boolean launcherAtSpeed) {
        if (defenseMode) {
            return "red defense";
        }
        if (launcherAtSpeed) {
            return "green ready";
        }
        return "off";
    }

    private void rollPartyColors() {
        long now = System.currentTimeMillis();
        if (now < nextPartyMs) {
            return;
        }
        nextPartyMs = now + PARTY_STEP_MS;
        partyRgb = randomColor();
        partyRgb2 = randomColor();
        int guard = 0;
        while (Math.abs(partyRgb - partyRgb2) < PARTY_SEPARATION && guard < 8) {
            partyRgb2 = randomColor();
            guard++;
        }
    }

    private double randomColor() {
        return COLOR_MIN + random.nextDouble() * (COLOR_MAX - COLOR_MIN);
    }

    private static Servo find(HardwareMap hardwareMap, String name) {
        try {
            return hardwareMap.get(Servo.class, name);
        } catch (RuntimeException exception) {
            return null;
        }
    }

    private static void set(Servo light, double position) {
        if (light != null) {
            light.setPosition(position);
        }
    }
}
