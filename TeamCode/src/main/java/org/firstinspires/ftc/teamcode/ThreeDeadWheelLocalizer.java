package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.messages.ThreeDeadWheelInputsMessage;

@Config
public final class ThreeDeadWheelLocalizer implements Localizer {
    public static class Params {
        // Placeholders for a 12 in left-to-right span and a back pod 6 in behind center.
        // Left is +Y and forward is +X. Replace these with the AngularRampLogger results.
        public double par0YTicks = 6.0 / (48.0 / 23825.0);
        public double par1YTicks = -par0YTicks;
        public double perpXTicks = -6.0 / (48.0 / 23825.0);

        // Direction debugger: par0 and perp were already reversed, so leave them forward.
        public boolean par0Reversed = false;
        public boolean par1Reversed = false;
        public boolean perpReversed = false;
    }

    public static Params PARAMS = new Params();

    public final Encoder par0, par1, perp;

    public final double inPerTick;

    private int lastPar0Pos, lastPar1Pos, lastPerpPos;
    private boolean initialized;
    private Pose2d pose;

    public ThreeDeadWheelLocalizer(HardwareMap hardwareMap, double inPerTick, Pose2d initialPose) {
        // FL left pod, BR right pod, BL perpendicular pod. setDirection here flips the count only.
        par0 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "FL")));
        par1 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "BR")));
        perp = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "BL")));
        if (PARAMS.par0Reversed) {
            par0.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        if (PARAMS.par1Reversed) {
            par1.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        if (PARAMS.perpReversed) {
            perp.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        this.inPerTick = inPerTick;

        FlightRecorder.write("THREE_DEAD_WHEEL_PARAMS", PARAMS);

        pose = initialPose;
    }

    @Override
    public void setPose(Pose2d pose) {
        this.pose = pose;
    }

    @Override
    public Pose2d getPose() {
        return pose;
    }

    @Override
    public PoseVelocity2d update() {
        PositionVelocityPair par0PosVel = par0.getPositionAndVelocity();
        PositionVelocityPair par1PosVel = par1.getPositionAndVelocity();
        PositionVelocityPair perpPosVel = perp.getPositionAndVelocity();

        FlightRecorder.write("THREE_DEAD_WHEEL_INPUTS", new ThreeDeadWheelInputsMessage(par0PosVel, par1PosVel, perpPosVel));

        if (!initialized) {
            initialized = true;

            lastPar0Pos = par0PosVel.position;
            lastPar1Pos = par1PosVel.position;
            lastPerpPos = perpPosVel.position;

            return new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0);
        }

        int par0PosDelta = par0PosVel.position - lastPar0Pos;
        int par1PosDelta = par1PosVel.position - lastPar1Pos;
        int perpPosDelta = perpPosVel.position - lastPerpPos;

        Twist2dDual<Time> twist = new Twist2dDual<>(
                new Vector2dDual<>(
                        new DualNum<Time>(new double[] {
                                (PARAMS.par0YTicks * par1PosDelta - PARAMS.par1YTicks * par0PosDelta) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                                (PARAMS.par0YTicks * par1PosVel.velocity - PARAMS.par1YTicks * par0PosVel.velocity) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                        }).times(inPerTick),
                        new DualNum<Time>(new double[] {
                                (PARAMS.perpXTicks / (PARAMS.par0YTicks - PARAMS.par1YTicks) * (par1PosDelta - par0PosDelta) + perpPosDelta),
                                (PARAMS.perpXTicks / (PARAMS.par0YTicks - PARAMS.par1YTicks) * (par1PosVel.velocity - par0PosVel.velocity) + perpPosVel.velocity),
                        }).times(inPerTick)
                ),
                new DualNum<>(new double[] {
                        (par0PosDelta - par1PosDelta) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                        (par0PosVel.velocity - par1PosVel.velocity) / (PARAMS.par0YTicks - PARAMS.par1YTicks),
                })
        );

        lastPar0Pos = par0PosVel.position;
        lastPar1Pos = par1PosVel.position;
        lastPerpPos = perpPosVel.position;

        pose = pose.plus(twist.value());
        return twist.velocity().value();
    }
}
