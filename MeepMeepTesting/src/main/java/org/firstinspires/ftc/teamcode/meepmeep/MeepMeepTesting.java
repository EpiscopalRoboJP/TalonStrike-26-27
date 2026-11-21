package org.firstinspires.ftc.teamcode.meepmeep;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import org.firstinspires.ftc.teamcode.AutoProfile;

import java.awt.Image;
import java.io.IOException;

import javax.imageio.ImageIO;

/**
 * Desktop preview of the autonomous routes. Run with {@code gradlew :MeepMeepTesting:run}.
 * That draws the pollen cycle. Pass {@code park} to draw BiobuzzAuto instead.
 *
 * Constraints and the route come from AutoProfile, which MecanumDrive also uses.
 * MeepMeep brakes at {@code -maxAccel}. The robot's min accel is
 * {@link AutoProfile#MIN_PROFILE_ACCEL}, so a stop on the field is a little longer
 * than the one drawn here.
 */
public final class MeepMeepTesting {
    // Pass "park" to draw BiobuzzAuto. Anything else draws the pollen cycle.
    private static final boolean PREVIEW_POLLEN_CYCLE = true;

    private MeepMeepTesting() {
    }

    public static void main(String[] args) {
        System.setProperty("sun.java2d.opengl", "true");

        MeepMeep meepMeep = new MeepMeep(800);
        Pose2d start = AutoProfile.START;

        RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
                .setDimensions(18, 18)
                .setStartPose(start)
                .setConstraints(
                        AutoProfile.MAX_WHEEL_VEL,
                        AutoProfile.MAX_PROFILE_ACCEL,
                        AutoProfile.MAX_ANG_VEL,
                        AutoProfile.MAX_ANG_ACCEL,
                        AutoProfile.TRACK_WIDTH_IN)
                .build();

        boolean pollen = PREVIEW_POLLEN_CYCLE;
        for (String arg : args) {
            if ("park".equals(arg)) {
                pollen = false;
            } else if ("pollen".equals(arg)) {
                pollen = true;
            }
        }
        Action route = pollen
                ? AutoProfile.buildPollenCycleAuto(bot.getDrive().actionBuilder(start), start)
                : AutoProfile.buildAuto(bot.getDrive().actionBuilder(start), start);
        bot.runAction(route);

        meepMeep.setBackground(loadBiobuzzField())
                .setBackgroundAlpha(1.0f)
                .addEntity(bot)
                .start();
    }

    private static Image loadBiobuzzField() {
        try {
            return ImageIO.read(MeepMeepTesting.class.getResource("/field-biobuzz.png"));
        } catch (IOException | IllegalArgumentException e) {
            throw new IllegalStateException("Missing BIOBUZZ field image field-biobuzz.png", e);
        }
    }
}
