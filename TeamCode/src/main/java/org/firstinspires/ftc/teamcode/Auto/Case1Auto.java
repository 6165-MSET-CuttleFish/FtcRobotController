package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.Async;
import org.firstinspires.ftc.teamcode.Components.OpModeType;
import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.PurePursuit.Coordinate;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.opencv.core.Mat;

import java.util.concurrent.atomic.AtomicBoolean;

import static org.firstinspires.ftc.teamcode.PurePursuit.MathFunctions.AngleWrap;
import static org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive.*;

@Autonomous(name = "AutoCase1", group = "LinearOpMode")
@Disabled
public class Case1Auto extends LinearOpMode {
    Robot robot;
    Vector2d dropZone;
    AtomicBoolean actionComplete = new AtomicBoolean(false);
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, 8.5, 47.8125, 0, OpModeType.auto, this);
        Trajectory powerShotsTraj1 = robot.driveTrain.trajectoryBuilder(Robot.robotPose)
                .addTemporalMarker(0.5, () -> {
                    robot.launcher.flapUp();
                    robot.launcher.safeLeftOut();
                })
                .lineToLinearHeading(Coordinate.toPose(Robot.pwrShotLocals[0],0),
                        getVelocityConstraint(40, Math.toRadians(60), DriveConstants.TRACK_WIDTH),
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
                .splineTo(new Vector2d(10, 8), new Vector2d(10, 8).angleBetween(new Vector2d(42, 9)))
                .splineTo(new Vector2d(42, 9), 0)
                .splineTo(new Vector2d(55.3, 4), Math.toRadians(-72))
                .splineToConstantHeading(new Vector2d(55, -10.4725), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(55, -38.4725), Math.toRadians(-90))
                .splineTo(Robot.B, Math.toRadians(-193))
                .addDisplacementMarker(() -> Async.start(() -> {
                    robot.release();
                    sleep(400);
                    robot.wobbleArmUp();
                }))
                .build();
        Trajectory firstShot = robot.driveTrain.trajectoryBuilder(wobbleDrop.end())
                .addTemporalMarker(0.5, ()->{
                    robot.launcher.setVelocity(robot.getPoseVelo(Robot.shootingPose) - 20);
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
                    robot.driveTrain.turn(Math.toRadians(-10));
                    Vector2d goalPost = Robot.goal.plus(new Vector2d(0, -11.5));
                    Pose2d position = robot.driveTrain.getPoseEstimate();
                    double absAngleToTarget = Math.atan2(goalPost.getY() - position.getY(), goalPost.getX() - position.getX());
                    double relAngleToPoint = AngleWrap(absAngleToTarget - robot.driveTrain.getPoseEstimate().getHeading());
                    robot.driveTrain.turn(relAngleToPoint);
                })
                .build();
        Trajectory wobblePickup = robot.driveTrain.trajectoryBuilder(Robot.shootingPose)
                .lineToSplineHeading(Coordinate.toPose(Robot.rightWobble, Math.toRadians(-30)),
                        getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
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
                        getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(-37, Robot.goal.getY() + 1), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-29, Robot.goal.getY(), Math.toRadians(-1)), Math.toRadians(0),
                        getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        Trajectory wobbleDrop2 = robot.driveTrain.trajectoryBuilder(wobblePickup.end())
                .lineToLinearHeading(Coordinate.toPose(Robot.B.plus(new Vector2d(-6, -5)), Math.toRadians(180)))
                .addDisplacementMarker(() -> Async.start(() -> {
                    robot.launcher.leftOut();
                    robot.release();
                    sleep(60);
                    robot.wobbleArmUp();
                }))
                .build();
        Trajectory park = robot.driveTrain.trajectoryBuilder(wobbleDrop2.end())
                .forward(8)
                .build();
        telemetry.addData("Initialization", "Complete");
        telemetry.update();
        robot.launcher.magUp();
        robot.grab();
        waitForStart();
        Async.start(()-> {
            while (opModeIsActive()) {
                robot.launcher.updatePID();
                Robot.robotPose = robot.driveTrain.getPoseEstimate();
            }
        });
        //robot.turnOffVision();
        robot.wobbleArmUp();
        robot.launcher.setLauncherVelocity(DriveConstants.BounceBackVelo);
        robot.launcher.unlockIntake();
        //Async.start(this::generatePaths);
        sleep(700);
        robot.driveTrain.followTrajectory(powerShotsTraj1);
        robot.driveTrain.followTrajectory(powerShotsTraj2);
        robot.driveTrain.followTrajectory(powerShotsTraj3);
        sleep(100);
        robot.launcher.setLauncherVelocity(0);
        robot.intake(1);
        actionComplete.set(false);
        robot.driveTrain.followTrajectory(wobbleDrop, ()-> {
            if(robot.driveTrain.getPoseEstimate().vec().distTo(Robot.B) < 13 && !actionComplete.get()) {
                robot.wobbleArmDown();
                actionComplete.set(true);
            }
        });
        robot.driveTrain.followTrajectory(firstShot);
        robot.wobbleArmDown();
        robot.optimalShoot(3);
        sleep(40);
        robot.launcher.setLauncherVelocity(0);
        robot.driveTrain.followTrajectory(wobblePickup, ()->{
            if(robot.driveTrain.getPoseEstimate().vec().distTo(Robot.rightWobble) < 8){
                Async.start(()->{
                    robot.grab();
                    sleep(700);
                    robot.wobbleArmUp();
                    robot.launcher.wingsIn();
                });
            }
        });
        sleep(120);
        robot.intake(-1);
        robot.launcher.magUp();
        sleep(200);
        robot.launcher.singleRound();
        sleep(40);
        robot.intake(0);
        robot.launcher.setLauncherVelocity(0);
        actionComplete.set(false);
        robot.driveTrain.followTrajectory(wobbleDrop2, ()->{
            if(robot.driveTrain.getPoseEstimate().vec().distTo(Robot.B) < 13 && !actionComplete.get()) {
                robot.wobbleArmDown();
                actionComplete.set(true);
            }
        });
        robot.driveTrain.followTrajectory(park);
    }
    private void generatePaths(){

    }
}