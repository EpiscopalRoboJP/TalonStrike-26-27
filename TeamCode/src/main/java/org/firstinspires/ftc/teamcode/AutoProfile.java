package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;

/**
 * Motion limits and the autonomous route. MecanumDrive and MeepMeepTesting both
 * read this class, so a change here is what the robot and the simulator run.
 *
 * Field frame: origin at center, +X toward blue, +Y away from the audience,
 * heading 0 faces +X and positive heading is counter-clockwise. On the robot,
 * forward is along the heading and left is 90 degrees counter-clockwise.
 *
 * Red starts against the audience wall, facing away from it. The route is
 * 0.5 tile forward, shoot, 1.25 tiles left, 3 tiles forward, then 0.5 tile
 * left into the red loading zone.
 */
public final class AutoProfile {
    private AutoProfile() {
    }

    public static final double MAX_WHEEL_VEL = 50.0;
    public static final double MIN_PROFILE_ACCEL = -30.0;
    public static final double MAX_PROFILE_ACCEL = 50.0;
    public static final double MAX_ANG_VEL = Math.PI;
    public static final double MAX_ANG_ACCEL = Math.PI;
    public static final double TRACK_WIDTH_IN = 12.0;

    public static final double TILE_IN = 24.0;
    public static final double ROBOT_IN = 18.0;
    public static final double HALF_FIELD_IN = 72.0;

    // 1.5 tiles left of this point puts an 18 in robot against the red wall.
    public static final Pose2d START = new Pose2d(
            -HALF_FIELD_IN + ROBOT_IN / 2.0 + 1.5 * TILE_IN,
            -HALF_FIELD_IN + ROBOT_IN / 2.0,
            Math.PI / 2.0);

    public static final double SPINUP_SECONDS = 1.5;
    public static final double FEED_PULSE_SECONDS = 0.4;
    public static final double INTAKE_INDEX_SECONDS = 0.6;
    public static final double BETWEEN_SHOTS_SECONDS = 0.2;
    public static final int SHOT_COUNT = 10;

    // Road Runner refuses a profile whose length is 0.
    public static final double MIN_TURN_RAD = Math.toRadians(0.5);
    public static final double MIN_MOVE_IN = 0.25;

    public static double shootSeconds() {
        return SPINUP_SECONDS
                + SHOT_COUNT * FEED_PULSE_SECONDS
                + (SHOT_COUNT - 1) * (INTAKE_INDEX_SECONDS + BETWEEN_SHOTS_SECONDS);
    }

    public static Pose2d shootPose() {
        return shootPose(START);
    }

    public static Pose2d afterLeftPose() {
        return afterLeftPose(START);
    }

    public static Pose2d parkPose() {
        return parkPose(START);
    }

    public static Pose2d fullParkPose() {
        return fullParkPose(START);
    }

    public static Pose2d shootPose(Pose2d start) {
        return offset(start, 0.5 * TILE_IN, 0.0);
    }

    public static Pose2d afterLeftPose(Pose2d start) {
        return offset(shootPose(start), 0.0, 1.25 * TILE_IN);
    }

    public static Pose2d parkPose(Pose2d start) {
        return offset(afterLeftPose(start), 3.0 * TILE_IN, 0.0);
    }

    public static Pose2d fullParkPose(Pose2d start) {
        return offset(parkPose(start), 0.0, 0.1 * TILE_IN);
    }

    /**
     * Drive forward half a tile, wait out the four pollen shots, strafe left,
     * drive forward, then strafe left half a tile to finish in the loading zone.
     * The builder must already be started at {@code start}.
     */
    public static Action buildAuto(TrajectoryActionBuilder builder, Pose2d start) {
        Pose2d shoot = shootPose(start);
        Pose2d tucked = afterLeftPose(start);
        Pose2d park = parkPose(start);
        Pose2d fullPark = fullParkPose(start);
        builder = append(builder, start, shoot.position, shoot.heading.toDouble());
        builder = builder.waitSeconds(shootSeconds());
        builder = append(builder, shoot, tucked.position, tucked.heading.toDouble());
        builder = append(builder, tucked, park.position, park.heading.toDouble());
        builder = append(builder, park, fullPark.position, fullPark.heading.toDouble());
        return builder.build();
    }

    private static Pose2d offset(Pose2d pose, double forwardIn, double leftIn) {
        double heading = pose.heading.toDouble();
        double forwardX = Math.cos(heading);
        double forwardY = Math.sin(heading);
        double leftX = -Math.sin(heading);
        double leftY = Math.cos(heading);
        return new Pose2d(
                pose.position.x + forwardX * forwardIn + leftX * leftIn,
                pose.position.y + forwardY * forwardIn + leftY * leftIn,
                heading);
    }

    /**
     * Strafes to {@code target} when it is far enough to matter, then turns to
     * {@code targetHeadingRadians}. Either piece is left out when its length is ~0.
     * Pass a null target to turn only.
     */
    public static TrajectoryActionBuilder append(
            TrajectoryActionBuilder builder,
            Pose2d pose,
            Vector2d target,
            double targetHeadingRadians) {
        if (target != null) {
            double dx = target.x - pose.position.x;
            double dy = target.y - pose.position.y;
            if (Math.hypot(dx, dy) >= MIN_MOVE_IN) {
                builder = builder.strafeTo(target);
            }
        }
        double headingDelta = wrap(targetHeadingRadians - pose.heading.toDouble());
        if (Math.abs(headingDelta) >= MIN_TURN_RAD) {
            builder = builder.turn(headingDelta);
        }
        return builder;
    }

    private static double wrap(double delta) {
        while (delta > Math.PI) {
            delta -= 2.0 * Math.PI;
        }
        while (delta < -Math.PI) {
            delta += 2.0 * Math.PI;
        }
        return delta;
    }
}
