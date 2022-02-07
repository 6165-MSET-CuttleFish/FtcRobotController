package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.trajectorysequenceimproved.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.field.Context;
import org.firstinspires.ftc.teamcode.util.field.OpModeType;

@Autonomous
@Disabled
public class WaitWhileTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this, OpModeType.AUTO);

        TrajectorySequence traj = robot.trajectorySequenceBuilder(Context.robotPose)
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> robot.intake.setPower(1))
                .splineTo(new Vector2d(30, 30), 0)
                .setReversed(true)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> robot.intake.setPower(0))
                .splineTo(new Vector2d(0, 0), Math.PI)
                .UNSTABLE_addTemporalMarkerOffset(0.0, robot.deposit::dump)
                .waitWhile(robot.deposit::isDoingInternalWork)
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> robot.intake.setPower(1))
                .setReversed(false)
                .splineTo(new Vector2d(30, 30), 0)
                .setReversed(true)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> robot.intake.setPower(0))
                .splineTo(new Vector2d(0, 0), Math.PI)
                .UNSTABLE_addTemporalMarkerOffset(0.0, robot.deposit::dump)
                .waitWhile(robot.deposit::isDoingInternalWork)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        robot.followTrajectorySequence(traj);
    }
}
