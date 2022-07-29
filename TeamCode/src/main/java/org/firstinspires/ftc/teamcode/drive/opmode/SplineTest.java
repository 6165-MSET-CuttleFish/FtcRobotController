package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.drive.StaticConfig;
import org.firstinspires.ftc.teamcode.roadrunnerext.drive.ImprovedDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
@Disabled
public class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ImprovedDrive drive = StaticConfig.getDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;
        while (opModeIsActive()) {
            Trajectory traj = drive.trajectoryBuilder(new Pose2d())
                    .splineTo(new Vector2d(40, 15), Math.toRadians(0))
                    .build();

            drive.followTrajectory(traj);

            sleep(500);

            drive.followTrajectory(
                    drive.trajectoryBuilder(traj.end(), true)
                            .splineTo(new Vector2d(0, 0), Math.toRadians(180))
                            .build()
            );
        }
    }
}
