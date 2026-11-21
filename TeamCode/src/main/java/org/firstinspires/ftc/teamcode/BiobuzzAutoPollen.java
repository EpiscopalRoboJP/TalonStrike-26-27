package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.ArrayList;
import java.util.List;

/**
 * Autonomous: shoot the preload, intake along A1's audience wall, shoot again, park.
 *
 * Same start as BiobuzzAuto: red side, audience wall, facing the audience.
 * After the first volley the robot turns so the back intake faces that wall and
 * bumps along it at half-intake spacing. It then returns to the shoot pose.
 * Stations that would start after the sweep cutoff are skipped. The second
 * volley still stops at the shoot cutoff so the loading-zone park can run.
 */
@Autonomous(name = "BiobuzzAutoPollen", group = "TeamCode", preselectTeleOp = "BiobuzzTeleop")
public class BiobuzzAutoPollen extends BiobuzzAutoBase {
    @Override
    protected void runAuto() {
        List<Action> steps = new ArrayList<Action>();
        steps.add(driveTo(AutoProfile.shootPose(), "approach", false, true, false));
        steps.add(shoot("shoot"));
        double[] stations = AutoProfile.sweepStationXs();
        for (int i = 0; i < stations.length; i++) {
            steps.add(sweepIfTime(stations[i]));
        }
        steps.add(driveTo(AutoProfile.shootPose(), "return", false, true, false));
        steps.add(shoot("shoot"));
        steps.add(driveTo(AutoProfile.afterLeftPose(), "park", false, true, false));
        steps.add(driveTo(AutoProfile.parkPose(), "park", false, true, false));
        steps.add(driveTo(AutoProfile.fullParkPose(), "park", true, true, false));
        Actions.runBlocking(new SequentialAction(steps));
    }
}
