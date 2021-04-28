package org.firstinspires.ftc.teamcode.Auto;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.Async;
import org.firstinspires.ftc.teamcode.Components.OpModeType;
import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.PurePursuit.Coordinate;
import org.firstinspires.ftc.teamcode.PurePursuit.MathFunctions;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.Arrays;

import static org.firstinspires.ftc.teamcode.PurePursuit.MathFunctions.AngleWrap;
import static org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive.*;

@Autonomous(name = "AutoCase1", group = "LinearOpMode")
public class Case1Auto extends LinearOpMode {
    Robot robot;
    Vector2d dropZone;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, 8.5, 47.8125, 0, OpModeType.auto, this);
        //robot.autoInit();
        Trajectory powerShotsTraj1 = robot.driveTrain.trajectoryBuilder(Robot.robotPose)
                .addTemporalMarker(0.5, () -> {
                    robot.launcher.flapUp();
                    robot.launcher.safeLeftOut();
                })
                .lineToLinearHeading(Coordinate.toPose(Robot.pwrShotLocals[0],0),
                        getVelocityConstraint(40, Math.toRadians(140), DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> Async.start(() -> robot.launcher.singleRound()))
                .build();
        Trajectory powerShotsTraj2 = robot.driveTrain.trajectoryBuilder(powerShotsTraj1.end())
                .addTemporalMarker(0.3, ()->robot.launcher.wingsOut())
                .lineToLinearHeading(Coordinate.toPose(Robot.pwrShotLocals[1], 0),
                        getVelocityConstraint(7, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> Async.start(() -> robot.launcher.singleRound()))
                .build();
        Trajectory powerShotsTraj3 = robot.driveTrain.trajectoryBuilder(powerShotsTraj2.end())
                .lineToLinearHeading(Coordinate.toPose(Robot.pwrShotLocals[2], 0),
                        getVelocityConstraint(7, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> Async.start(() -> robot.launcher.singleRound()))
                .build();
        Trajectory wobbleDrop = robot.driveTrain.trajectoryBuilder(powerShotsTraj3.end())
                .addTemporalMarker(0.7, () -> {
                    robot.launcher.wingsVert();
                    Async.set(() -> Robot.B.distTo(robot.driveTrain.getPoseEstimate().vec()) <= 12, () -> robot.wobbleArmDown());
                })
                //.splineToConstantHeading(powerShotsTraj2.end().vec().plus(new Vector2d(10, 5)), 0)
                .splineTo(new Vector2d(45, 10), 0)
                .splineTo(new Vector2d(55, 10), 0)
                .splineTo(new Vector2d(58, 6), Math.toRadians(-75))
                .splineToConstantHeading(new Vector2d(58.6, -10.4725), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(58.6, -43.4725), Math.toRadians(-90))
                .splineToSplineHeading(Coordinate.toPose(Robot.B, Math.toRadians(-185)), Math.toRadians(-185))
                .addDisplacementMarker(() -> {
                    Async.start(() -> {
                        robot.release();
                        sleep(400);
                        robot.wobbleArmUp();
                    });
                    robot.launcher.setVelocity(robot.getPoseVelo(Robot.shootingPose) - 30);
                    telemetry.addData("distance", Coordinate.distanceToLine(Robot.shootingPose, Robot.goal.getX()));
                    telemetry.addData("velo", robot.launcher.getTargetVelo());
                    telemetry.update();
                    robot.launcher.flapDown();
                })
                .build();
        Trajectory firstShot = robot.driveTrain.trajectoryBuilder(wobbleDrop.end())
                .lineToLinearHeading(Coordinate.toPose(Robot.shootingPose.vec(), Math.toRadians(180)))
                .addDisplacementMarker(()-> {
                    robot.intake(0);
                    robot.launcher.magUp();
                    robot.launcher.safeLeftOut();
                    robot.driveTrain.turn(MathFunctions.AngleWrap(Robot.shootingPose.getHeading() - robot.driveTrain.getPoseEstimate().getHeading()));
                })
                .build();
        Trajectory wobblePickup = robot.driveTrain.trajectoryBuilder(Robot.shootingPose)
                .lineToSplineHeading(Coordinate.toPose(Robot.rightWobble, Math.toRadians(-8)),
                        getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(()->{
                    robot.intake(1);
                    robot.launcher.setVelocity(robot.getPoseVelo(new Vector2d(-29, Robot.goal.getY())) - 20);
                })
                .splineToConstantHeading(new Vector2d(-40, -50), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-37, Robot.goal.getY() + 1), Math.toRadians(0),
                        getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToSplineHeading(new Pose2d(-29, Robot.goal.getY(), Math.toRadians(-1)), Math.toRadians(0),
                        getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory wobbleDrop2 = robot.driveTrain.trajectoryBuilder(wobblePickup.end())
                .addDisplacementMarker(() -> Async.set(() -> robot.driveTrain.getPoseEstimate().vec().distTo(Robot.B) <= 15, () -> robot.wobbleArmDown()))
                .lineToLinearHeading(Coordinate.toPose(Robot.B.plus(new Vector2d(0, -5)), Math.toRadians(180)))
                .addDisplacementMarker(() -> Async.start(() -> {
                    robot.launcher.leftOut();
                    robot.release();
                    sleep(60);
                    robot.wobbleArmUp();
                }))
                .build();
        Trajectory park = robot.driveTrain.trajectoryBuilder(wobbleDrop2.end())
                .forward(7)
                .build();
        telemetry.addData("Initialization", "Complete");
        telemetry.update();
        robot.launcher.magUp();
        robot.grab();
//        while (!opModeIsActive()) {
//            robot.scan();
//            telemetry.addData("Stack Height", robot.height);
//            telemetry.addData("Discs", robot.dice.toString());
//            telemetry.update();
//        }
        waitForStart();
        Async.start(()-> {
            while (opModeIsActive()) {
                robot.launcher.updatePID();
                Robot.robotPose = robot.driveTrain.getPoseEstimate();
            }
        });
        //robot.turnOffVision();
        robot.wobbleArmUp();
        robot.launcher.setLauncherVelocity(908);
        robot.launcher.unlockIntake();
        //Async.start(this::generatePaths);
        sleep(700);
        robot.driveTrain.followTrajectory(powerShotsTraj1);
        robot.driveTrain.followTrajectory(powerShotsTraj2);
        robot.driveTrain.followTrajectory(powerShotsTraj3);
        sleep(100);
        robot.launcher.setLauncherVelocity(0);
        robot.intake(1);
        robot.driveTrain.followTrajectory(wobbleDrop);
        robot.driveTrain.followTrajectory(firstShot);
        robot.wobbleArmDown();
        robot.optimalShoot(robot.launcher.getRings());
        sleep(40);
        robot.launcher.setLauncherVelocity(0);
        Async.set(() -> robot.driveTrain.getPoseEstimate().vec().distTo(Robot.rightWobble) <= 10, () -> {
            robot.grab();
            sleep(1000);
            robot.wobbleArmUp();
        });
        robot.driveTrain.followTrajectory(wobblePickup);
        sleep(120);
        robot.intake(-1);
        robot.launcher.magUp();
        sleep(200);
        robot.launcher.singleRound();
        sleep(40);
        robot.intake(0);
        robot.launcher.setLauncherVelocity(0);
        robot.driveTrain.followTrajectory(wobbleDrop2);
        robot.driveTrain.followTrajectory(park);
    }
    private void generatePaths(){

    }
}
