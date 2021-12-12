package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.Robot;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this);

        waitForStart();

        if (isStopRequested()) return;
        while (opModeIsActive()) {
            Trajectory traj = robot.trajectoryBuilder(new Pose2d())
                    .splineTo(new Vector2d(30, 30), Math.toRadians(0))
                    .build();

            robot.followTrajectory(traj);

            sleep(500);

            robot.followTrajectory(
                    robot.trajectoryBuilder(traj.end(), true)
                            .splineTo(new Vector2d(0, 0), Math.toRadians(180))
                            .build()
            );
        }
    }
}
