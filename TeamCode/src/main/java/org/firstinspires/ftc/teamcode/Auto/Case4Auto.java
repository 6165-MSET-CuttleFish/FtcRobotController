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
import org.firstinspires.ftc.teamcode.PurePursuit.MathFunctions;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.Arrays;

import static org.firstinspires.ftc.teamcode.PurePursuit.MathFunctions.AngleWrap;
import static org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive.*;

@Autonomous(name = "AutoCase4", group = "LinearOpMode")
public class Case4Auto extends LinearOpMode {
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
                        getVelocityConstraint(40, Math.toRadians(80), DriveConstants.TRACK_WIDTH),
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
                .addTemporalMarker(0.7, () -> robot.launcher.wingsVert())
                .splineTo(new Vector2d(45, 10), 0)
                .splineTo(new Vector2d(53, 10), 0)
                .splineTo(new Vector2d(58, 6), Math.toRadians(-75))
                .splineToConstantHeading(new Vector2d(58.6, -10.4725), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(58.6, -43.4725), Math.toRadians(-90))
                .addDisplacementMarker(()->robot.wobbleArmDown())
                .splineToSplineHeading(Coordinate.toPose(Robot.C, Math.toRadians(-185)), Math.toRadians(-185))
                .addDisplacementMarker(() -> {
                    Async.start(() -> {
                        robot.release();
                        sleep(400);
                        robot.wobbleArmUp();
                    });
                })
                .build();
        Trajectory firstShot = robot.driveTrain.trajectoryBuilder(wobbleDrop.end())
                .addTemporalMarker(0.5, ()->{
                    robot.launcher.setVelocity(robot.getPoseVelo(Robot.shootingPose) - 40);
                    telemetry.addData("distance", Coordinate.distanceToLine(Robot.shootingPose, Robot.goal.getX()));
                    telemetry.addData("velo", robot.launcher.getTargetVelo());
                    telemetry.update();
                    robot.launcher.flapDown();
                })
                .lineToLinearHeading(Coordinate.toPose(Robot.shootingPose.vec(), Math.toRadians(180)))
                .addDisplacementMarker(()-> {
                    robot.intake(0);
                    robot.launcher.magUp();
                    robot.launcher.safeLeftOut();
                    Vector2d goalPost = Robot.goal.plus(new Vector2d(0, -11));
                    Pose2d position = robot.driveTrain.getPoseEstimate();
                    double absAngleToTarget = Math.atan2(goalPost.getY() - position.getY(), goalPost.getX() - position.getX());
                    double relAngleToPoint = AngleWrap(absAngleToTarget - robot.driveTrain.getPoseEstimate().getHeading());
                    robot.driveTrain.turn(relAngleToPoint);
                })
                .build();
        Trajectory wobblePickup = robot.driveTrain.trajectoryBuilder(Robot.shootingPose)
                .lineToSplineHeading(Coordinate.toPose(Robot.rightWobble, Math.toRadians(-30)),
                        getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(()->Async.start(()->{
                    robot.grab();
                    sleep(800);
                    robot.wobbleArmUp();
                }))
                .addDisplacementMarker(()->{
                    robot.intake(1);
                    robot.launcher.setVelocity(robot.getPoseVelo(new Vector2d(-29, Robot.goal.getY())) - 15);
                })
                .splineToConstantHeading(new Vector2d(-40, -40), Math.toRadians(90),
                        getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(-37, Robot.goal.getY() + 1), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-29, Robot.goal.getY(), Math.toRadians(-1)), Math.toRadians(0),
                        getVelocityConstraint(24, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory finalShot = robot.driveTrain.trajectoryBuilder(wobblePickup.end())
                .addDisplacementMarker(() -> robot.intake(1))
                .lineToLinearHeading(Robot.shootingPoseTele)
                .build();
        Trajectory wobbleDrop2 = robot.driveTrain.trajectoryBuilder(finalShot.end())
                .addTemporalMarker(1.4, () -> robot.wobbleArmDown())
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
        robot.wobbleArmUp();
        robot.launcher.setLauncherVelocity(910);
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
        robot.driveTrain.followTrajectory(wobblePickup);
        sleep(120);
        robot.intake(-1);
        robot.launcher.magUp();
        sleep(200);
        robot.launcher.singleRound();
        robot.launcher.setVelocity(robot.getPoseVelo(Robot.shootingPoseTele) - 70);
        sleep(40);
        robot.launcher.magDown();
        robot.driveTrain.followTrajectory(finalShot);
        sleep(120);
        robot.intake(-1);
        robot.launcher.magUp();
        sleep(190);
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
