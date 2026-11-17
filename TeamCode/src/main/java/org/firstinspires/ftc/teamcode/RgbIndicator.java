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
 * RGB2 is red while defense is adjusting, orange blink while holding,
 * a slow rainbow while being pushed, and green once the launcher is at shoot speed.
 * Party mode picks a new color for each light on its own.
 * E108 forbids a bright light that flashes faster than about 5 times a second.
 * Each light changes at most once every 250 ms, including blinks and rainbow steps.
 */
public final class RgbIndicator {
    public static final String DEVICE_NAME = "RGB";
    public static final String DEVICE_NAME_2 = "RGB2";

    static final double OFF = 0.0;
    static final double RED = 0.33;
    static final double ORANGE = 0.39;
    static final double YELLOW = 0.45;
    static final double GREEN = 0.50;
    static final double PURPLE = 0.66;
    static final double COLOR_MIN = 0.32;
    static final double COLOR_MAX = 0.69;

    static final long MIN_CHANGE_MS = 250;
    private static final double PARTY_SEPARATION = 0.06;

    private final Servo rgb;
    private final Servo rgb2;
    private final Random random = new Random();
    private double partyRgb = YELLOW;
    private double partyRgb2 = PURPLE;
    private long nextPartyMs;
    private long rgbChangedMs;
    private long rgb2ChangedMs;
    private double rgbShown = OFF;
    private double rgb2Shown = OFF;
    private boolean rgbHasShown;
    private boolean rgb2HasShown;
    private String rgbLabel = "off";
    private String rgb2Label = "off";

    private RgbIndicator(Servo rgb, Servo rgb2) {
        this.rgb = rgb;
        this.rgb2 = rgb2;
        long now = System.currentTimeMillis();
        show(rgb, OFF, now, true, true);
        show(rgb2, OFF, now, false, true);
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
            boolean defenseHolding,
            boolean fighting,
            boolean launcherAtSpeed,
            boolean party) {
        long now = System.currentTimeMillis();
        if (party) {
            rollPartyColors(now, true);
            rgbLabel = "rainbow";
            rgb2Label = "rainbow";
            show(rgb, partyRgb, now, true, false);
            show(rgb2, partyRgb2, now, false, false);
            return;
        }

        rgbLabel = rgbLabel(launcherMoving, intakeMoving, robotMoving);
        show(rgb, rgbPosition(launcherMoving, intakeMoving, robotMoving), now, true, false);
        applyRgb2(defenseMode, defenseHolding, fighting, launcherAtSpeed, now);
    }

    public void off() {
        long now = System.currentTimeMillis();
        rgbLabel = rgb == null ? "not configured" : "off";
        rgb2Label = rgb2 == null ? "not configured" : "off";
        show(rgb, OFF, now, true, true);
        show(rgb2, OFF, now, false, true);
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

    private void applyRgb2(
            boolean defenseMode,
            boolean defenseHolding,
            boolean fighting,
            boolean launcherAtSpeed,
            long now) {
        if (fighting) {
            rollPartyColors(now, false);
            rgb2Label = "rainbow fight";
            show(rgb2, partyRgb2, now, false, false);
            return;
        }
        if (defenseHolding) {
            rgb2Label = "orange hold";
            show(rgb2, blinkLit(now) ? ORANGE : OFF, now, false, false);
            return;
        }
        if (defenseMode) {
            rgb2Label = "red defense";
            show(rgb2, RED, now, false, false);
            return;
        }
        if (launcherAtSpeed) {
            rgb2Label = "green ready";
            show(rgb2, GREEN, now, false, false);
            return;
        }
        rgb2Label = "off";
        show(rgb2, OFF, now, false, false);
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

    private static boolean blinkLit(long now) {
        return (now / MIN_CHANGE_MS) % 2 == 0;
    }

    private void rollPartyColors(long now, boolean bothLights) {
        if (now < nextPartyMs) {
            return;
        }
        nextPartyMs = now + MIN_CHANGE_MS;
        if (bothLights) {
            partyRgb = randomColor();
        }
        partyRgb2 = randomColor();
        int guard = 0;
        while (bothLights && Math.abs(partyRgb - partyRgb2) < PARTY_SEPARATION && guard < 8) {
            partyRgb2 = randomColor();
            guard++;
        }
    }

    private double randomColor() {
        return COLOR_MIN + random.nextDouble() * (COLOR_MAX - COLOR_MIN);
    }

    /**
     * Writes a position only when it differs from what that light is already showing
     * and at least MIN_CHANGE_MS has passed since its last change. Stop forces off.
     */
    private void show(Servo light, double position, long now, boolean firstLight, boolean force) {
        if (light == null) {
            return;
        }
        boolean hasShown = firstLight ? rgbHasShown : rgb2HasShown;
        double shown = firstLight ? rgbShown : rgb2Shown;
        long changedMs = firstLight ? rgbChangedMs : rgb2ChangedMs;
        if (!force && hasShown && position != shown && now - changedMs < MIN_CHANGE_MS) {
            return;
        }
        if (!hasShown || position != shown) {
            light.setPosition(position);
            if (firstLight) {
                rgbShown = position;
                rgbChangedMs = now;
                rgbHasShown = true;
            } else {
                rgb2Shown = position;
                rgb2ChangedMs = now;
                rgb2HasShown = true;
            }
        }
    }

    private static Servo find(HardwareMap hardwareMap, String name) {
        try {
            return hardwareMap.get(Servo.class, name);
        } catch (RuntimeException exception) {
            return null;
        }
    }
}
