package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.Async;
import org.firstinspires.ftc.teamcode.Components.Claw;
import org.firstinspires.ftc.teamcode.Components.Gunner;
import org.firstinspires.ftc.teamcode.Components.Intake;
import org.firstinspires.ftc.teamcode.Components.Magazine;
import org.firstinspires.ftc.teamcode.Components.OpModeType;
import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.Components.Shooter;
import org.firstinspires.ftc.teamcode.Components.Side;
import org.firstinspires.ftc.teamcode.Components.Turret;
import org.firstinspires.ftc.teamcode.Components.WobbleArm;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import static org.firstinspires.ftc.teamcode.Components.Details.robotPose;

@Autonomous(name = "RED_Aggressive", group = "red")
public class RedAggressive extends LinearOpMode {
    Robot robot;
    Shooter shooter;
    Gunner gunner;
    Turret turret;
    Magazine magazine;
    Intake intake;
    WobbleArm wobbleArm;
    Claw claw;

    TrajectorySequence mainSequence;
    Trajectory wobbleDrop;
    TrajectorySequence shootBonked;
    TrajectorySequence powerShots;
    TrajectorySequence bouncebacks;
    TrajectorySequence park;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this, new Pose2d(-61.5975, -16.8475, 0), OpModeType.AUTO, Side.RED);
        shooter = robot.shooter;
        intake = robot.intake;
        wobbleArm = robot.wobbleArm;
        turret = shooter.turret;
        gunner = shooter.gunner;
        magazine = shooter.magazine;
        claw = wobbleArm.claw;
        shooter.setState(Shooter.State.EMPTY_MAG);
        mainSequence = robot.trajectorySequenceBuilder(robotPose)
                .splineTo(new Vector2d(45.5275, -22.7), Math.toRadians(0))
                .addDisplacementMarker(() -> {
                    shooter.setState(Shooter.State.IDLE);
                    intake.setPower(1);
                })
                .splineTo(new Vector2d(50.5275, 10), Math.toRadians(90))
                .splineTo(new Vector2d(50.5, 20), Math.toRadians(90))
                .build();
        wobbleDrop = robot.trajectoryBuilder(mainSequence.end(), true)
                .splineTo(new Vector2d(50.5275, -50), Math.toRadians(-90))
                .addDisplacementMarker(() -> {
                    wobbleArm.dropMacro();
                    intake.setPower(0);
                })
                .build();
        shootBonked = robot.trajectorySequenceBuilder(wobbleDrop.end())
                .splineTo(new Vector2d(-5, -22.7), Math.toRadians(180))
                .addTemporalMarker(0.4, () -> {
                    magazine.magMacro();
                    shooter.setState(Shooter.State.CONTINUOUS);
                })
                .addDisplacementMarker(() -> gunner.shoot(3))
                .build();
        powerShots = robot.trajectorySequenceBuilder(shootBonked.end())
                .addDisplacementMarker(() -> intake.setPower(1))
                .lineTo(new Vector2d(-58, -22.7)) // Intake starter rings
                .addTemporalMarker(0.3, () -> {
                    intake.setPower(0);
                    magazine.magMacro();
                    shooter.setState(Shooter.State.POWERSHOTS);
                })
                .setReversed(true)
                .splineTo(Robot.powerShotLocals[0], 0)
                .build();
        bouncebacks = robot.trajectorySequenceBuilder(powerShots.end())
                .lineToLinearHeading(new Pose2d(50.5275, -10.7, Math.toRadians(-90)))
                .lineToSplineHeading(new Pose2d(50.5275, -57, Math.toRadians(-90)))
                .setReversed(true)
                .addDisplacementMarker(() -> shooter.setState(Shooter.State.CONTINUOUS))
                .splineTo(new Vector2d(-5.8, -20), Math.toRadians(180))
                .addDisplacementMarker(() -> gunner.shoot(3))
                .build();
        park = robot.trajectorySequenceBuilder(bouncebacks.end())
                .lineTo(new Vector2d(12, -20))
                .build();
        waitForStart();
        // intake.setShieldState(Intake.ShieldState.DOWN);
        robot.followTrajectorySequence(mainSequence);
        robot.followTrajectory(wobbleDrop);
        robot.waitForActionsCompleted();
        robot.followTrajectorySequence(shootBonked);
        robot.waitForActionsCompleted();
        robot.followTrajectorySequence(powerShots);
        shooter.powerShots();
        telemetry.addData("Power Shots", shooter.powerShotsController.getRunning());
        telemetry.update();
        robot.waitForActionsCompleted();
        shooter.setState(Shooter.State.IDLE);
        robot.followTrajectorySequence(bouncebacks);
        robot.waitForActionsCompleted();
        shooter.setState(Shooter.State.IDLE);
        robot.followTrajectorySequence(park);
        while (opModeIsActive()){
            robot.update();
        }
    }
    private void generatePaths(){
        Async.start(()->{});
    }
}
