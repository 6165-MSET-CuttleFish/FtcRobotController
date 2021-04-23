package org.firstinspires.ftc.teamcode.Auto;

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
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.Arrays;

import static org.firstinspires.ftc.teamcode.PurePursuit.MathFunctions.AngleWrap;
import static org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive.*;

@Autonomous(name = "newAuto", group = "LinearOpMode")
public class BounceBackAuto extends LinearOpMode {
    Robot robot;
    Vector2d dropZone;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, 8.5, 47.8125, 0, OpModeType.auto, this);
        //robot.autoInit();
        Trajectory powerShotsTraj1 = robot.driveTrain.trajectoryBuilder(Robot.robotPose)
                .addTemporalMarker(0.5, () -> robot.launcher.safeLeftOut())
                //.lineToSplineHeading(new Pose2d(-40.4725, -21))
                .splineToConstantHeading(Robot.pwrShotLocals[0],0,
                        getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> Async.start(() -> robot.launcher.singleRound()))
                .build();
        Trajectory powerShotsTraj2 = robot.driveTrain.trajectoryBuilder(powerShotsTraj1.end())
                .lineToLinearHeading(Coordinate.toPose(Robot.pwrShotLocals[1], 0),
                        getVelocityConstraint(6.92, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> Async.start(() -> robot.launcher.singleRound()))
                .lineToLinearHeading(Coordinate.toPose(Robot.pwrShotLocals[2], 0),
                        getVelocityConstraint(6.92, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> Async.start(() -> robot.launcher.singleRound()))
                .build();
        Trajectory wobbleDrop = robot.driveTrain.trajectoryBuilder(powerShotsTraj2.end())
                .addTemporalMarker(0.3, () -> {
                    robot.launcher.wingsVert();
                    Async.set(() -> Robot.C.distTo(robot.driveTrain.getPoseEstimate().vec()) <= 11, () -> robot.wobbleArmDown());
                })
                .splineToConstantHeading(powerShotsTraj2.end().vec().plus(new Vector2d(5, 5)), 0)
                .splineTo(new Vector2d(45, 12), 0)
                .splineTo(new Vector2d(59, 4), Math.toRadians(-85))
                .splineToConstantHeading(new Vector2d(59.3, -10.4725), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(59.3, -40.4725), Math.toRadians(-90))
                .splineToSplineHeading(Coordinate.toPose(Robot.C, Math.toRadians(-190)), Math.toRadians(-190))
                .addDisplacementMarker(() -> {
                    Async.start(() -> {
                        robot.release();
                        sleep(400);
                        robot.wobbleArmUp();
                    });
                    robot.launcher.setVelocity(robot.getPoseVelo(Robot.shootingPose));
                    robot.launcher.flapDown();
                    robot.launcher.safeLeftOut();
                })
                .build();
        Trajectory firstShot = robot.driveTrain.trajectoryBuilder(wobbleDrop.end())
                .addTemporalMarker(2, ()-> {
                    robot.intake(0);
                    robot.launcher.magUp();
                    robot.launcher.safeLeftOut();
                })
                .lineToLinearHeading(Robot.shootingPose)
                .build();
        Trajectory wobblePickup = robot.driveTrain.trajectoryBuilder(firstShot.end())
                .lineToSplineHeading(Coordinate.toPose(Robot.rightWobble, Math.toRadians(-3)), new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(30, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    //robot.launcher.setVelocity(robot.getPoseVelo(new Vector2d(-23, Robot.get)));
                    robot.launcher.setVelocity(robot.getPoseVelo(new Vector2d(-24, Robot.goal.getY())));
                    robot.intake(1);
                    robot.launcher.wingsIn();
                })
                .splineToConstantHeading(new Vector2d(-40, Robot.goal.getY()), Math.toRadians(-3),
                        getVelocityConstraint(28, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(-38.7, Robot.goal.getY()), Math.toRadians(-3),
                        getVelocityConstraint(14, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToLinearHeading(new Pose2d(-24, Robot.goal.getY(), Math.toRadians(-3)),0,
                        getVelocityConstraint(7, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory finalShot = robot.driveTrain.trajectoryBuilder(wobblePickup.end())
                .addDisplacementMarker(() -> robot.intake(1))
                .lineToLinearHeading(Robot.shootingPoseTele)
                .build();
        Trajectory wobbleDrop2 = robot.driveTrain.trajectoryBuilder(finalShot.end())
                .addDisplacementMarker(() -> Async.set(() -> robot.driveTrain.getPoseEstimate().vec().distTo(Robot.C) <= 15, () -> robot.wobbleArmDown()))
                .lineToLinearHeading(Coordinate.toPose(Robot.C.plus(new Vector2d(-4.5, 8)), Math.toRadians(130)))
                .addDisplacementMarker(() -> Async.start(() -> {
                    robot.launcher.leftOut();
                    robot.release();
                    sleep(60);
                    robot.wobbleArmUp();
                }))
                .build();
        Trajectory park = robot.driveTrain.trajectoryBuilder(wobbleDrop2.end())
                .lineToLinearHeading(new Pose2d(33, -54, Math.toRadians(120)))
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
        robot.launcher.flapUp();
        robot.wobbleArmUp();
        robot.launcher.setLauncherVelocity(906);
        robot.launcher.unlockIntake();
        //Async.start(this::generatePaths);
        sleep(500);
        robot.driveTrain.followTrajectory(powerShotsTraj1);
        robot.driveTrain.followTrajectory(powerShotsTraj2);
//        robot.driveTrain.followTrajectory(powerShotsTraj3);
        sleep(40);
        robot.launcher.setLauncherVelocity(0);
        robot.intake(1);
        robot.driveTrain.followTrajectory(wobbleDrop);
        robot.driveTrain.followTrajectory(firstShot);
        robot.wobbleArmDown();
        robot.optimalShoot(robot.launcher.getRings());
        sleep(40);
        robot.launcher.setLauncherVelocity(0);
        robot.wobbleArmDown();
        Async.set(() -> robot.driveTrain.getPoseEstimate().vec().distTo(Robot.rightWobble) <= 16.5, () -> {
            robot.grab();
            sleep(700);
            robot.wobbleArmUp();
        });
        robot.driveTrain.followTrajectory(wobblePickup);
        robot.intake(-1);
        robot.launcher.magUp();
        sleep(170);
        robot.launcher.singleRound();
        robot.launcher.setVelocity(robot.getPoseVelo(Robot.shootingPoseTele));
        sleep(40);
        robot.launcher.magDown();
        robot.driveTrain.followTrajectory(finalShot);
        robot.intake(-1);
        robot.launcher.magUp();
        sleep(170);
        robot.optimalShoot(3);
        sleep(60);
        robot.intake(0);
        robot.launcher.setLauncherVelocity(0);
        robot.driveTrain.followTrajectory(wobbleDrop2);
        robot.driveTrain.followTrajectory(park);
    }
    private void generatePaths(){

    }
}
