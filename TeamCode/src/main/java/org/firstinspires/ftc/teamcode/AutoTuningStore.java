package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.json.JSONObject;

import java.io.File;

/**
 * Persists live FTC Dashboard tuning values to the Control Hub so practice-zone
 * adjustments survive a Robot Controller restart or battery swap.
 */
public final class AutoTuningStore {
    public static final String FILE_NAME = "auto_tuning.json";

    private static boolean loadedOnBoot = false;
    private static String status = "CODE DEFAULTS";

    private AutoTuningStore() {
    }

    public static String statusLabel() {
        return status;
    }

    /**
     * Loads saved tuning values once per Robot Controller process startup.
     * Later OpMode inits keep whatever values are currently active in FTC Dashboard.
     */
    public static synchronized void loadOnce() {
        if (loadedOnBoot) {
            return;
        }
        loadedOnBoot = true;
        try {
            File file = AppUtil.getInstance().getSettingsFile(FILE_NAME);
            if (!file.exists()) {
                status = "CODE DEFAULTS";
                return;
            }
            JSONObject root = new JSONObject(ReadWriteFile.readFile(file));

            JSONObject profile = root.optJSONObject("autoProfile");
            if (profile != null) {
                AutoProfile.SHOOT_FORWARD_TILES = profile.optDouble("SHOOT_FORWARD_TILES", AutoProfile.SHOOT_FORWARD_TILES);
                AutoProfile.SHOOT_LEFT_TILES = profile.optDouble("SHOOT_LEFT_TILES", AutoProfile.SHOOT_LEFT_TILES);
                AutoProfile.SHOOT_HEADING_OFFSET_DEG = profile.optDouble("SHOOT_HEADING_OFFSET_DEG", AutoProfile.SHOOT_HEADING_OFFSET_DEG);
                AutoProfile.TUCK_FORWARD_IN = profile.optDouble("TUCK_FORWARD_IN", AutoProfile.TUCK_FORWARD_IN);
                AutoProfile.TUCK_LEFT_TILES = profile.optDouble("TUCK_LEFT_TILES", AutoProfile.TUCK_LEFT_TILES);
                AutoProfile.PARK_FORWARD_TILES = profile.optDouble("PARK_FORWARD_TILES", AutoProfile.PARK_FORWARD_TILES);
                AutoProfile.FULL_PARK_LEFT_TILES = profile.optDouble("FULL_PARK_LEFT_TILES", AutoProfile.FULL_PARK_LEFT_TILES);
                AutoProfile.SPINUP_SECONDS = profile.optDouble("SPINUP_SECONDS", AutoProfile.SPINUP_SECONDS);
                AutoProfile.FEED_PULSE_SECONDS = profile.optDouble("FEED_PULSE_SECONDS", AutoProfile.FEED_PULSE_SECONDS);
                AutoProfile.INTAKE_INDEX_SECONDS = profile.optDouble("INTAKE_INDEX_SECONDS", AutoProfile.INTAKE_INDEX_SECONDS);
                AutoProfile.BETWEEN_SHOTS_SECONDS = profile.optDouble("BETWEEN_SHOTS_SECONDS", AutoProfile.BETWEEN_SHOTS_SECONDS);
                AutoProfile.SHOT_COUNT = profile.optInt("SHOT_COUNT", AutoProfile.SHOT_COUNT);
                AutoProfile.BUMP_HOLD_SECONDS = profile.optDouble("BUMP_HOLD_SECONDS", AutoProfile.BUMP_HOLD_SECONDS);
            }

            JSONObject auto = root.optJSONObject("biobuzzAuto");
            if (auto != null) {
                BiobuzzMechanisms.FLYWHEEL_POWER_MOD = auto.optDouble("FLYWHEEL_POWER_MOD", BiobuzzMechanisms.FLYWHEEL_POWER_MOD);
                BiobuzzMechanisms.FLYWHEEL_POWER_HIGH = auto.optDouble("FLYWHEEL_POWER_HIGH", BiobuzzMechanisms.FLYWHEEL_POWER_HIGH);
                BiobuzzMechanisms.FLYWHEEL_POWER_MID = auto.optDouble("FLYWHEEL_POWER_MID", BiobuzzMechanisms.FLYWHEEL_POWER_MID);
                BiobuzzMechanisms.FLYWHEEL_POWER_LOW = auto.optDouble("FLYWHEEL_POWER_LOW", BiobuzzMechanisms.FLYWHEEL_POWER_LOW);
                BiobuzzMechanisms.INTAKE_FEED_POWER = auto.optDouble("INTAKE_FEED_POWER", BiobuzzMechanisms.INTAKE_FEED_POWER);
                BiobuzzMechanisms.SHOOT_CUTOFF_SECONDS = auto.optDouble("SHOOT_CUTOFF_SECONDS", BiobuzzMechanisms.SHOOT_CUTOFF_SECONDS);
                BiobuzzMechanisms.SWEEP_CUTOFF_SECONDS = auto.optDouble("SWEEP_CUTOFF_SECONDS", BiobuzzMechanisms.SWEEP_CUTOFF_SECONDS);
                BiobuzzMechanisms.BUMP_BACK_POWER = auto.optDouble("BUMP_BACK_POWER", BiobuzzMechanisms.BUMP_BACK_POWER);
                BiobuzzMechanisms.JOSTLE_POWER = auto.optDouble("JOSTLE_POWER", BiobuzzMechanisms.JOSTLE_POWER);
            }

            JSONObject drive = root.optJSONObject("mecanumDrive");
            if (drive != null) {
                MecanumDrive.Params p = MecanumDrive.PARAMS;
                p.inPerTick = drive.optDouble("inPerTick", p.inPerTick);
                p.lateralInPerTick = drive.optDouble("lateralInPerTick", p.lateralInPerTick);
                p.trackWidthTicks = drive.optDouble("trackWidthTicks", p.trackWidthTicks);
                p.kS = drive.optDouble("kS", p.kS);
                p.kV = drive.optDouble("kV", p.kV);
                p.kA = drive.optDouble("kA", p.kA);
                p.maxWheelVel = drive.optDouble("maxWheelVel", p.maxWheelVel);
                p.minProfileAccel = drive.optDouble("minProfileAccel", p.minProfileAccel);
                p.maxProfileAccel = drive.optDouble("maxProfileAccel", p.maxProfileAccel);
                p.maxAngVel = drive.optDouble("maxAngVel", p.maxAngVel);
                p.maxAngAccel = drive.optDouble("maxAngAccel", p.maxAngAccel);
                p.axialGain = drive.optDouble("axialGain", p.axialGain);
                p.lateralGain = drive.optDouble("lateralGain", p.lateralGain);
                p.headingGain = drive.optDouble("headingGain", p.headingGain);
                p.axialVelGain = drive.optDouble("axialVelGain", p.axialVelGain);
                p.lateralVelGain = drive.optDouble("lateralVelGain", p.lateralVelGain);
                p.headingVelGain = drive.optDouble("headingVelGain", p.headingVelGain);
            }

            JSONObject odom = root.optJSONObject("threeDeadWheel");
            if (odom != null) {
                ThreeDeadWheelLocalizer.Params o = ThreeDeadWheelLocalizer.PARAMS;
                o.par0YTicks = odom.optDouble("par0YTicks", o.par0YTicks);
                o.par1YTicks = odom.optDouble("par1YTicks", o.par1YTicks);
                o.perpXTicks = odom.optDouble("perpXTicks", o.perpXTicks);
                o.par0Reversed = odom.optBoolean("par0Reversed", o.par0Reversed);
                o.par1Reversed = odom.optBoolean("par1Reversed", o.par1Reversed);
                o.perpReversed = odom.optBoolean("perpReversed", o.perpReversed);
            }

            status = "SAVED FILE (" + FILE_NAME + ")";
        } catch (Exception e) {
            status = "LOAD ERR: " + e.getMessage();
        }
    }

    public static synchronized void saveCurrent() {
        try {
            JSONObject profile = new JSONObject();
            profile.put("SHOOT_FORWARD_TILES", AutoProfile.SHOOT_FORWARD_TILES);
            profile.put("SHOOT_LEFT_TILES", AutoProfile.SHOOT_LEFT_TILES);
            profile.put("SHOOT_HEADING_OFFSET_DEG", AutoProfile.SHOOT_HEADING_OFFSET_DEG);
            profile.put("TUCK_FORWARD_IN", AutoProfile.TUCK_FORWARD_IN);
            profile.put("TUCK_LEFT_TILES", AutoProfile.TUCK_LEFT_TILES);
            profile.put("PARK_FORWARD_TILES", AutoProfile.PARK_FORWARD_TILES);
            profile.put("FULL_PARK_LEFT_TILES", AutoProfile.FULL_PARK_LEFT_TILES);
            profile.put("SPINUP_SECONDS", AutoProfile.SPINUP_SECONDS);
            profile.put("FEED_PULSE_SECONDS", AutoProfile.FEED_PULSE_SECONDS);
            profile.put("INTAKE_INDEX_SECONDS", AutoProfile.INTAKE_INDEX_SECONDS);
            profile.put("BETWEEN_SHOTS_SECONDS", AutoProfile.BETWEEN_SHOTS_SECONDS);
            profile.put("SHOT_COUNT", AutoProfile.SHOT_COUNT);
            profile.put("BUMP_HOLD_SECONDS", AutoProfile.BUMP_HOLD_SECONDS);

            JSONObject auto = new JSONObject();
            auto.put("FLYWHEEL_POWER_MOD", BiobuzzMechanisms.FLYWHEEL_POWER_MOD);
            auto.put("FLYWHEEL_POWER_HIGH", BiobuzzMechanisms.FLYWHEEL_POWER_HIGH);
            auto.put("FLYWHEEL_POWER_MID", BiobuzzMechanisms.FLYWHEEL_POWER_MID);
            auto.put("FLYWHEEL_POWER_LOW", BiobuzzMechanisms.FLYWHEEL_POWER_LOW);
            auto.put("INTAKE_FEED_POWER", BiobuzzMechanisms.INTAKE_FEED_POWER);
            auto.put("SHOOT_CUTOFF_SECONDS", BiobuzzMechanisms.SHOOT_CUTOFF_SECONDS);
            auto.put("SWEEP_CUTOFF_SECONDS", BiobuzzMechanisms.SWEEP_CUTOFF_SECONDS);
            auto.put("BUMP_BACK_POWER", BiobuzzMechanisms.BUMP_BACK_POWER);
            auto.put("JOSTLE_POWER", BiobuzzMechanisms.JOSTLE_POWER);

            MecanumDrive.Params p = MecanumDrive.PARAMS;
            JSONObject drive = new JSONObject();
            drive.put("inPerTick", p.inPerTick);
            drive.put("lateralInPerTick", p.lateralInPerTick);
            drive.put("trackWidthTicks", p.trackWidthTicks);
            drive.put("kS", p.kS);
            drive.put("kV", p.kV);
            drive.put("kA", p.kA);
            drive.put("maxWheelVel", p.maxWheelVel);
            drive.put("minProfileAccel", p.minProfileAccel);
            drive.put("maxProfileAccel", p.maxProfileAccel);
            drive.put("maxAngVel", p.maxAngVel);
            drive.put("maxAngAccel", p.maxAngAccel);
            drive.put("axialGain", p.axialGain);
            drive.put("lateralGain", p.lateralGain);
            drive.put("headingGain", p.headingGain);
            drive.put("axialVelGain", p.axialVelGain);
            drive.put("lateralVelGain", p.lateralVelGain);
            drive.put("headingVelGain", p.headingVelGain);

            ThreeDeadWheelLocalizer.Params o = ThreeDeadWheelLocalizer.PARAMS;
            JSONObject odom = new JSONObject();
            odom.put("par0YTicks", o.par0YTicks);
            odom.put("par1YTicks", o.par1YTicks);
            odom.put("perpXTicks", o.perpXTicks);
            odom.put("par0Reversed", o.par0Reversed);
            odom.put("par1Reversed", o.par1Reversed);
            odom.put("perpReversed", o.perpReversed);

            JSONObject root = new JSONObject();
            root.put("autoProfile", profile);
            root.put("biobuzzAuto", auto);
            root.put("mecanumDrive", drive);
            root.put("threeDeadWheel", odom);

            File file = AppUtil.getInstance().getSettingsFile(FILE_NAME);
            ReadWriteFile.writeFile(file, root.toString(2));
            status = "SAVED TO HUB OK";
        } catch (Exception e) {
            status = "SAVE ERR: " + e.getMessage();
        }
    }

    public static synchronized void clearSaved() {
        try {
            File file = AppUtil.getInstance().getSettingsFile(FILE_NAME);
            if (!file.exists() || file.delete()) {
                status = "CLEARED (restart RC for defaults)";
            } else {
                status = "CLEAR FAILED";
            }
        } catch (Exception e) {
            status = "CLEAR ERR: " + e.getMessage();
        }
    }
}
