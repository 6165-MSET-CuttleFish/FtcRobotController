package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
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
import static org.firstinspires.ftc.teamcode.Components.Robot.stackHeight;

@Autonomous(name = "RED_AGGRESSIVE", group = "red")
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
    Trajectory wobbleDrop4, wobbleDrop1, wobbleDrop0;
    TrajectorySequence shootBonked4, shootBonked1, shootBonked0;
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
        generatePaths();
        sleep(1000);
        stackHeight = UGContourRingPipeline.Height.ONE;
        robot.setPoseEstimate(robotPose);
        sleep(1000);
        telemetry.addData("Ready", true);
        telemetry.update();
        waitForStart();

        intake.dropIntake();
        robot.followTrajectorySequence(mainSequence);
        robot.followTrajectory(getWobbleDrop());
        wobbleArm.dropMacro();
        intake.setPower(0);
        robot.waitForActionsCompleted();
        robot.followTrajectorySequence(getShootBonked());
        gunner.shoot(3);
        robot.waitForActionsCompleted();
        robot.followTrajectorySequence(powerShots);
        shooter.powerShots();
        robot.waitForActionsCompleted();
        shooter.setState(Shooter.State.IDLE);
        robot.followTrajectorySequence(bouncebacks);
        gunner.shoot(3);
        robot.waitForActionsCompleted();
        shooter.setState(Shooter.State.IDLE);
        robot.followTrajectorySequence(park);
        while (opModeIsActive()){
            robot.update();
        }
    }

    private void generatePaths(){
        mainSequence = robot.trajectorySequenceBuilder(robotPose)
                .splineTo(new Vector2d(25.5275, -18.7), Math.toRadians(0))
                .splineTo(new Vector2d(45.5275, -22.7), Math.toRadians(0))
                .addDisplacementMarker(() -> {
                    shooter.setState(Shooter.State.IDLE);
                    intake.setPower(1);
                })
                .splineTo(new Vector2d(50.5275, 10), Math.toRadians(90))
                .splineTo(new Vector2d(50.5, 17), Math.toRadians(90))
                .build();
        // Wobble Drop
        wobbleDrop0 = robot.trajectoryBuilder(mainSequence.end(), true)
                .splineTo(new Vector2d(50.5275, -5), Math.toRadians(-90))
                .splineTo(Robot.dropZonesPS()[0].vec(), Robot.dropZonesPS()[0].getHeading())
                .build();
        wobbleDrop1 = robot.trajectoryBuilder(mainSequence.end(), true)
                .splineTo(new Vector2d(50.5275, -5), Math.toRadians(-90))
                .splineTo(Robot.dropZonesPS()[1].vec(), Robot.dropZonesPS()[1].getHeading())
                .build();
        wobbleDrop4 = robot.trajectoryBuilder(mainSequence.end(), true)
                .splineTo(new Vector2d(50.5275, -5), Math.toRadians(-90))
                .splineTo(Robot.dropZonesPS()[2].vec(), Robot.dropZonesPS()[2].getHeading())
                .build();
        // Shoot Bonked
        shootBonked0 = robot.trajectorySequenceBuilder(wobbleDrop0.end())
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    magazine.magMacro();
                    shooter.setState(Shooter.State.CONTINUOUS);
                })
                .splineTo(new Vector2d(-5, -16.7), Math.toRadians(180))
                .build();
        shootBonked1 = robot.trajectorySequenceBuilder(wobbleDrop1.end())
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    magazine.magMacro();
                    shooter.setState(Shooter.State.CONTINUOUS);
                })
                .splineTo(new Vector2d(-5, -16.7), Math.toRadians(180))
                .build();
        shootBonked4 = robot.trajectorySequenceBuilder(wobbleDrop4.end())
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    magazine.magMacro();
                    shooter.setState(Shooter.State.CONTINUOUS);
                })
                .splineTo(new Vector2d(-5, -16.7), Math.toRadians(180))
                .build();
        // PowerShots
        powerShots = robot.trajectorySequenceBuilder(shootBonked4.end())
                .addDisplacementMarker(() -> intake.setPower(1))
                .lineTo(new Vector2d(-55, -16.7)) // Intake starter rings
                .setReversed(true)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    intake.setPower(0);
                    magazine.magMacro();
                    shooter.setState(Shooter.State.POWERSHOTS);
                })
                .splineTo(Robot.pwrShotLocal(), 0)
                .build();
        bouncebacks = robot.trajectorySequenceBuilder(powerShots.end())
                .addDisplacementMarker(() -> intake.setPower(1))
                .lineToLinearHeading(new Pose2d(50.5275, -10.7, Math.toRadians(-90)))
                .lineToSplineHeading(new Pose2d(50.5275, -42, Math.toRadians(-90)))
                .setReversed(true)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    intake.setPower(0);
                    magazine.magMacro();
                    shooter.setState(Shooter.State.CONTINUOUS);
                })
                .splineTo(new Vector2d(-5.8, -14), Math.toRadians(180))
                .build();
        park = robot.trajectorySequenceBuilder(bouncebacks.end())
                .lineTo(new Vector2d(12, -14))
                .build();
    }

    private Trajectory getWobbleDrop() {
        switch (stackHeight){
            case ZERO: return wobbleDrop0;
            case ONE: return wobbleDrop1;
            default: return wobbleDrop4;
        }
    }

    private TrajectorySequence getShootBonked() {
        switch (stackHeight){
            case ZERO: return shootBonked0;
            case ONE: return shootBonked1;
            default: return shootBonked4;
        }
    }
}
