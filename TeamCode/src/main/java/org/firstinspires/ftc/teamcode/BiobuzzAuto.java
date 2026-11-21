package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * Autonomous: shoot the preloaded pollen from the start spot, then park in the Loading Zone.
 * Driving uses Road Runner. Position comes from the three dead-wheel pods on FL, BR, and BL.
 *
 * No driver input after the opmode is selected. Heading stays pointed at the audience.
 *
 * Place the robot on the red side, touching the audience wall, facing the
 * audience. The route and drive limits live in AutoProfile, which MeepMeep reads too.
 * Flywheel, intake, and gate timing live in BiobuzzMechanisms, shared with teleop.
 * The gate feeds each shot and the chassis jostles between shots.
 * RGB and RGB2 follow teleop: purple while the launcher spins, green while the
 * intake runs, yellow while a wheel is commanded, and green on RGB2 once the
 * encoder holds shoot speed.
 * Feedforward and dead-wheel geometry in MecanumDrive.PARAMS are still placeholders
 * until the tuning opmodes have been run.
 */
@Autonomous(name = "BiobuzzAuto", group = "TeamCode", preselectTeleOp = "BiobuzzTeleop")
public class BiobuzzAuto extends BiobuzzAutoBase {
    @Override
    protected void runAuto() {
        Actions.runBlocking(new ParallelAction(
                // deployJavelin(),
                new SequentialAction(
                        driveTo(AutoProfile.shootPose(), "approach", false, true, false),
                        shoot("shoot"),
                        driveTo(AutoProfile.afterLeftPose(), "park", false, true, false),
                        driveTo(AutoProfile.parkPose(), "park", false, true, false),
                        driveTo(AutoProfile.fullParkPose(), "park", true, true, false)
                )
        ));
    }
}
