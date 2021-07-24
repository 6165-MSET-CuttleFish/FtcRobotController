package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.Async;
import org.firstinspires.ftc.teamcode.Components.OpModeType;
import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "RedAggressiveAuto", group = "red")
public class RedTest extends LinearOpMode {
    Robot robot;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this, new Pose2d(-62, -22.7, 0), OpModeType.AUTO);
        TrajectorySequence mainSequence = robot.trajectorySequenceBuilder(new Pose2d(-62, -22.7, 0))
                .splineTo(new Vector2d(45.5275, -22.7), Math.toRadians(0))
                .splineTo(new Vector2d(50.5275, 10), Math.toRadians(90))
                .splineTo(new Vector2d(50.5, 20), Math.toRadians(90))
                .setReversed(true)
                .splineTo(new Vector2d(50.5275, -50), Math.toRadians(-90))
                .waitSeconds(0.5) // Drop Wobble
                .setReversed(false)
                .splineTo(new Vector2d(-5, -22.7), Math.toRadians(180))
                .waitSeconds(0.5) // Shoot bonked rings
                .lineTo(new Vector2d(-62, -22.7)) // Intake starter rings
                .lineTo(new Vector2d(-5, -22.7))
                .waitSeconds(0.6) // Shoot powershots
                .lineToLinearHeading(new Pose2d(65.5275, -10.7, Math.toRadians(-90)))
                .lineToSplineHeading(new Pose2d(60.5275, -57, Math.toRadians(-90)))
                .setReversed(true)
                .splineTo(new Vector2d(-5.8, -20), Math.toRadians(180))
                .waitSeconds(0.5) // Shoot bouncebacks
                .lineTo(new Vector2d(12, -20))
                .build();
        waitForStart();
        robot.followTrajectorySequence(mainSequence);
    }
    private void generatePath(){
        Async.start(()->{});
    }
}
