package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.Async;
import org.firstinspires.ftc.teamcode.Components.OpModeType;
import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.Components.Shooter;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "RedAggressiveAuto", group = "red")
public class RedTest extends LinearOpMode {
    Robot robot;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this, new Pose2d(-62, -22.7, 0), OpModeType.AUTO);
        TrajectorySequence mainSequence = robot.trajectorySequenceBuilder(new Pose2d(-62, -22.7, 0))
                .splineTo(new Vector2d(40.5275, -22.7), Math.toRadians(90))
                //.splineTo(new Vector2d(65.5275, -22.7), Math.toRadians(90))

                .splineTo(new Vector2d(40, 10), Math.toRadians(90))
                .splineTo(new Vector2d(40, 20), Math.toRadians(90))
                .splineTo(new Vector2d(20, 20), Math.toRadians(180))
                .splineTo(new Vector2d(-5, -16), Math.toRadians(180))
//                .addDisplacementMarker(() -> {
//                    robot.shooter.tripleShot();
//                    robot.waitForActionsCompleted();
//                }) // Shoot Boinked Rings
                .lineTo(new Vector2d(-55, -22.7)) // Intake starter rings
//                .addDisplacementMarker(() -> {
//                    robot.shooter.setState(Shooter.State.POWERSHOTS);
//                    robot.shooter.turret.setTarget(Robot.powerShots[0]);
//                })
                .lineToLinearHeading(new Pose2d(-5, -16))
//                .addDisplacementMarker(() -> {
//                    robot.shooter.powerShots();
//                    robot.waitForActionsCompleted();
//                })// Shoot powershots
                .lineToLinearHeading(new Pose2d(65.5275, -10.7, Math.toRadians(-90)))
                .addTemporalMarker(2, this::generatePath)
                .lineToSplineHeading(new Pose2d(60.5275, -57, Math.toRadians(-90)))
                .lineToSplineHeading(new Pose2d(60.5275, -50, Math.toRadians(90)))
                .addDisplacementMarker(() -> {
                    robot.wobbleArm.dropMacro();
                    robot.waitForActionsCompleted();
                }) // Drop Wobble
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
