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
 * Red starts against the audience wall, facing it. The route backs up
 * 0.5 tile, shoots toward the audience, strafes 1.25 tiles toward the red
 * wall, backs up 3 tiles, then strafes 0.1 tile into the red loading zone.
 *
 * The pollen cycle shoots that preload, then sweeps tile A1 along the
 * audience wall. The intake is on the back, half a robot wide, so the
 * robot turns to +90 deg and bumps the wall at stations spaced within
 * that width. It then returns to the shoot pose and parks as above.
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

    // 1.5 tiles toward the red wall from here puts an 18 in robot against that wall.
    public static final Pose2d START = new Pose2d(
            -HALF_FIELD_IN + ROBOT_IN / 2.0 + 1.5 * TILE_IN,
            -HALF_FIELD_IN + ROBOT_IN / 2.0,
            -Math.PI / 2.0);

    // Route offsets exposed in FTC Dashboard via TuningOpModes.
    // Robot frame: +forward is along heading (toward audience at start),
    // +left is 90 deg CCW (toward blue wall at start; negative is toward red wall).
    public static double SHOOT_FORWARD_TILES = -0.5;
    public static double SHOOT_LEFT_TILES = 0.0;
    public static double SHOOT_HEADING_OFFSET_DEG = 0.0;
    public static double TUCK_FORWARD_IN = -0.4;
    public static double TUCK_LEFT_TILES = -1.25;
    public static double PARK_FORWARD_TILES = -3.0;
    public static double FULL_PARK_LEFT_TILES = -0.1;

    public static double SPINUP_SECONDS = 1.5;
    public static double FEED_PULSE_SECONDS = 0.4;
    public static double INTAKE_INDEX_SECONDS = 0.6;
    public static double BETWEEN_SHOTS_SECONDS = 0.2;
    public static int SHOT_COUNT = 4;

    // Back-facing intake is half the robot. A1's audience wall is y = -72.
    public static final double INTAKE_WIDTH_IN = ROBOT_IN / 2.0;
    public static final double SWEEP_HEADING = Math.PI / 2.0;
    public static final double WALL_CONTACT_Y = -HALF_FIELD_IN + ROBOT_IN / 2.0;
    public static final double SWEEP_CLEARANCE_IN = 9.0;
    public static final double SWEEP_STANDOFF_Y = WALL_CONTACT_Y + SWEEP_CLEARANCE_IN;
    public static final double A1_MIN_CENTER_X = -HALF_FIELD_IN + ROBOT_IN / 2.0;
    public static final double A1_MAX_CENTER_X = -HALF_FIELD_IN + TILE_IN - INTAKE_WIDTH_IN / 2.0;
    public static double BUMP_HOLD_SECONDS = 0.35;

    // Road Runner refuses a profile whose length is 0.
    public static final double MIN_TURN_RAD = Math.toRadians(0.5);
    public static final double MIN_MOVE_IN = 0.25;

    public static double shootSeconds() {
        return SPINUP_SECONDS
                + SHOT_COUNT * FEED_PULSE_SECONDS
                + Math.max(0, SHOT_COUNT - 1) * (INTAKE_INDEX_SECONDS + BETWEEN_SHOTS_SECONDS);
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
        Pose2d shifted = offset(start, SHOOT_FORWARD_TILES * TILE_IN, SHOOT_LEFT_TILES * TILE_IN);
        return new Pose2d(
                shifted.position.x,
                shifted.position.y,
                wrap(shifted.heading.toDouble() + Math.toRadians(SHOOT_HEADING_OFFSET_DEG)));
    }

    public static Pose2d afterLeftPose(Pose2d start) {
        Pose2d shootPosWithStartHeading = offset(
                start, SHOOT_FORWARD_TILES * TILE_IN, SHOOT_LEFT_TILES * TILE_IN);
        return offset(shootPosWithStartHeading, TUCK_FORWARD_IN, TUCK_LEFT_TILES * TILE_IN);
    }

    public static Pose2d parkPose(Pose2d start) {
        return offset(afterLeftPose(start), PARK_FORWARD_TILES * TILE_IN, 0.0);
    }

    public static Pose2d fullParkPose(Pose2d start) {
        return offset(parkPose(start), 0.0, FULL_PARK_LEFT_TILES * TILE_IN);
    }

    /**
     * Back up half a tile, wait out the pollen shots while facing the audience,
     * strafe toward the red wall, back up, then strafe into the loading zone.
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

    /**
     * Centers along A1's audience wall, from the blue side of the tile toward
     * the red wall. Step is at most {@link #INTAKE_WIDTH_IN}, so a half-width
     * intake covers the reachable line with overlap.
     */
    public static double[] sweepStationXs() {
        double span = A1_MAX_CENTER_X - A1_MIN_CENTER_X;
        if (span <= MIN_MOVE_IN) {
            return new double[] { A1_MAX_CENTER_X };
        }
        int gaps = (int) Math.ceil(span / INTAKE_WIDTH_IN - 1e-9);
        if (gaps < 1) {
            gaps = 1;
        }
        double step = span / gaps;
        double[] xs = new double[gaps + 1];
        for (int i = 0; i <= gaps; i++) {
            xs[i] = A1_MAX_CENTER_X - step * i;
        }
        return xs;
    }

    public static Pose2d sweepStandoffPose(double stationX) {
        return new Pose2d(stationX, SWEEP_STANDOFF_Y, SWEEP_HEADING);
    }

    public static Pose2d sweepWallPose(double stationX) {
        return new Pose2d(stationX, WALL_CONTACT_Y, SWEEP_HEADING);
    }

    /**
     * Shoot the preload, bump along A1's audience wall, shoot again, then park.
     * The builder must already be started at {@code start}. The open-loop press
     * at the wall is the wait here; the robot applies the backup power itself.
     */
    public static Action buildPollenCycleAuto(TrajectoryActionBuilder builder, Pose2d start) {
        Pose2d shoot = shootPose(start);
        builder = append(builder, start, shoot.position, shoot.heading.toDouble());
        builder = builder.waitSeconds(shootSeconds());
        Pose2d cursor = shoot;
        double[] stations = sweepStationXs();
        for (int i = 0; i < stations.length; i++) {
            Pose2d standoff = sweepStandoffPose(stations[i]);
            Pose2d wall = sweepWallPose(stations[i]);
            builder = append(builder, cursor, standoff.position, standoff.heading.toDouble());
            builder = append(builder, standoff, wall.position, wall.heading.toDouble());
            builder = builder.waitSeconds(Math.max(0.0, BUMP_HOLD_SECONDS));
            builder = append(builder, wall, standoff.position, standoff.heading.toDouble());
            cursor = standoff;
        }
        builder = append(builder, cursor, shoot.position, shoot.heading.toDouble());
        builder = builder.waitSeconds(shootSeconds());
        Pose2d tucked = afterLeftPose(start);
        Pose2d park = parkPose(start);
        Pose2d fullPark = fullParkPose(start);
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
