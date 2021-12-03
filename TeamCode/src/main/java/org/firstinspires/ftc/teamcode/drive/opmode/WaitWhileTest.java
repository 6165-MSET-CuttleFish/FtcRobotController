package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.trajectorysequenceimproved.TrajectorySequence;

@Autonomous
public class WaitWhileTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this);

        waitForStart();

        if (isStopRequested()) return;

        TrajectorySequence traj = robot.trajectorySequenceBuilder(new Pose2d())
                .splineTo(new Vector2d(30, 30), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0.0, robot.deposit::dump)
                .waitWhile(robot.deposit::isDoingWork)
                .setReversed(true)
                .splineTo(new Vector2d(0, 0), Math.PI)
                .build();

        robot.followTrajectorySequence(traj);
    }
}
