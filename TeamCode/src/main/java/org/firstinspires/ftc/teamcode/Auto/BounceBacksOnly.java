package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.Async;
import org.firstinspires.ftc.teamcode.Components.OpModeType;
import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.PurePursuit.Coordinate;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;

import java.util.concurrent.atomic.AtomicBoolean;

import static org.firstinspires.ftc.teamcode.PurePursuit.MathFunctions.AngleWrap;
import static org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive.getAccelerationConstraint;
import static org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive.getVelocityConstraint;

@Autonomous(name = "BounceBacks", group = "Op")
public class BounceBacksOnly extends LinearOpMode {
    Robot robot;
    Vector2d dropZone;
    AtomicBoolean actionComplete = new AtomicBoolean(false);

    Trajectory powerShotsTraj1;
    Trajectory powerShotsTraj2;
    Trajectory powerShotsTraj3;
    Trajectory wobbleDrop;
    Trajectory firstShot;
    Trajectory wobblePickup;
    Trajectory finalShot;
    Trajectory wobbleDrop2;
    Trajectory park;
    private double getDistance(){
        if(Robot.height == UGContourRingPipeline.Height.FOUR) return 4;
        else return 10;
    }
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this, 8.5, 47.8125, 0, OpModeType.auto);
        robot.autoInit();
        powerShotsTraj1 = robot.trajectoryBuilder(Robot.robotPose)
                .addTemporalMarker(0.5, () -> {
                    robot.shooter.flapUp();
                    robot.shooter.safeLeftOut();
                })
                .lineToLinearHeading(Coordinate.toPose(Robot.pwrShotLocals[0],0),
                        getVelocityConstraint(40, Math.toRadians(190), DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> Async.start(() -> robot.shooter.singleRound()))
                .build();
        powerShotsTraj2 = robot.trajectoryBuilder(powerShotsTraj1.end())
                .addTemporalMarker(0.3, ()->robot.shooter.wingsOut())
                .lineToLinearHeading(Coordinate.toPose(Robot.pwrShotLocals[1], 0),
                        getVelocityConstraint(7, Math.toRadians(240), DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> Async.start(() -> robot.shooter.singleRound()))
                .build();
        powerShotsTraj3 = robot.trajectoryBuilder(powerShotsTraj2.end())
                .lineToLinearHeading(Coordinate.toPose(Robot.pwrShotLocals[2], 0),
                        getVelocityConstraint(8, Math.toRadians(220), DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> Async.start(() -> robot.shooter.singleRound()))
                .build();
        telemetry.addData("Initialization", "Complete");
        telemetry.update();
        robot.shooter.magUp();
        robot.grab();
        while (!opModeIsActive()) {
            robot.scan();
            telemetry.addData("Stack Height", Robot.height.toString());
            telemetry.update();
        }
        waitForStart();
        Async.start(()-> {
            while (opModeIsActive()) {
                robot.shooter.update();
                Robot.robotPose = robot.getPoseEstimate();
            }
        });
        robot.turnOffVision();
        robot.wobbleArmUp();
        robot.shooter.setLauncherVelocity(DriveConstants.BounceBackVelo);
        robot.shooter.unlockIntake();
        Async.start(this::generatePaths);
        sleep(200);
        robot.followTrajectory(powerShotsTraj1);
        robot.followTrajectory(powerShotsTraj2);
        robot.followTrajectory(powerShotsTraj3);
        sleep(80);
        robot.shooter.setLauncherVelocity(0);
        robot.intake(1);
        actionComplete.set(false);
        robot.followTrajectory(wobbleDrop, ()-> {
            if(robot.getPoseEstimate().vec().distTo(dropZone) < getDistance() && !actionComplete.get()) {
                robot.wobbleArmDown();
                actionComplete.set(true);
            }
        });
        robot.followTrajectory(firstShot);
        robot.wobbleArmDown();
        robot.optimalShoot(3);
        sleep(40);
        robot.shooter.setLauncherVelocity(0);
        if(Robot.height == UGContourRingPipeline.Height.FOUR){
            case4();
        } else if(Robot.height == UGContourRingPipeline.Height.ONE){
            case1();
        } else {
            case0();
        }
    }
    private void case0(){
        robot.followTrajectory(wobblePickup, ()->{
            if(robot.getPoseEstimate().vec().distTo(Robot.rightWobble) < 8){
                Async.start(()->{
                    robot.grab();
                    sleep(700);
                    robot.wobbleArmUp();
                    robot.shooter.wingsIn();
                });
            }
        });
        actionComplete.set(false);
        robot.followTrajectory(wobbleDrop2, ()->{
            if(robot.getPoseEstimate().vec().distTo(Robot.A) < 27 && !actionComplete.get()) {
                robot.wobbleArmDown();
                actionComplete.set(true);
            }
        });
        robot.followTrajectory(park);
    }
    private void case1(){
        robot.followTrajectory(wobblePickup, ()->{
            if(robot.getPoseEstimate().vec().distTo(Robot.rightWobble) < 8){
                Async.start(()->{
                    robot.grab();
                    sleep(700);
                    robot.wobbleArmUp();
                    robot.shooter.wingsIn();
                });
            }
        });
        sleep(110);
        robot.intake(-1);
        robot.shooter.magUp();
        sleep(200);
        robot.shooter.singleRound();
        sleep(40);
        robot.intake(0);
        robot.shooter.setLauncherVelocity(0);
        actionComplete.set(false);
        robot.followTrajectory(wobbleDrop2, ()->{
            if(robot.getPoseEstimate().vec().distTo(Robot.B) < 27 && !actionComplete.get()) {
                robot.wobbleArmDown();
                actionComplete.set(true);
            }
        });
        robot.followTrajectory(park);
    }
    private void case4(){
        robot.followTrajectory(wobblePickup, ()->{
            if(robot.getPoseEstimate().vec().distTo(Robot.rightWobble) < 8){
                Async.start(()->{
                    robot.grab();
                    sleep(700);
                    robot.wobbleArmUp();
                    robot.shooter.wingsIn();
                });
            }
        });
        sleep(120);
        robot.intake(-1);
        robot.shooter.magUp();
        sleep(200);
        robot.shooter.singleRound();
        robot.shooter.setVelocity(Robot.shootingPoseTele.vec());
        sleep(40);
        robot.shooter.magDown();
        robot.followTrajectory(finalShot);
        sleep(100);
        robot.intake(-1);
        robot.shooter.magUp();
        sleep(190);
        robot.optimalShoot(3);
        sleep(40);
        robot.intake(0);
        robot.shooter.setLauncherVelocity(0);
        actionComplete.set(false);
        robot.followTrajectory(wobbleDrop2, ()->{
            if(robot.getPoseEstimate().vec().distTo(Robot.C) < 20 && !actionComplete.get()) {
                robot.wobbleArmDown();
                actionComplete.set(true);
            }
        });
        robot.followTrajectory(park);
    }
    private void generatePaths(){
        dropZone = robot.getDropZone();
        TrajectoryBuilder tempBuilder = robot.trajectoryBuilder(powerShotsTraj3.end())
                .addTemporalMarker(0.7, () -> robot.shooter.wingsVert())
                .splineTo(new Vector2d(15, 10), new Vector2d(10, 8).angleBetween(new Vector2d(42, 10)))
                .splineTo(new Vector2d(42, 10), 0)
                .splineTo(new Vector2d(54, 4), Math.toRadians(-80))
                .splineToConstantHeading(new Vector2d(55, -10.4725), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(55, -40.4725), Math.toRadians(-90));
        if(Robot.height == UGContourRingPipeline.Height.FOUR)
            wobbleDrop = tempBuilder
                    .splineToSplineHeading(Coordinate.toPose(dropZone, Math.toRadians(-185)), Math.toRadians(-180))
                    .addDisplacementMarker(() -> Async.start(() -> {
                        robot.release();
                        sleep(400);
                        robot.wobbleArmUp();
                    }))
                    .build();
        else if(Robot.height == UGContourRingPipeline.Height.ONE)
            wobbleDrop = tempBuilder
                    .splineToSplineHeading(Coordinate.toPose(dropZone, Math.toRadians(-193)), Math.toRadians(-180),
                            getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .addDisplacementMarker(() -> Async.start(() -> {
                        robot.release();
                        sleep(400);
                        robot.wobbleArmUp();
                    }))
                    .build();
        else wobbleDrop = tempBuilder
                    .splineTo(Robot.C.plus(new Vector2d(0, 10)), Math.toRadians(180))
                    .splineToSplineHeading(Coordinate.toPose(dropZone, Math.toRadians(-220)), Math.toRadians(-180),
                            getVelocityConstraint(44, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .addDisplacementMarker(() -> Async.start(() -> {
                        robot.release();
                        sleep(400);
                        robot.wobbleArmUp();
                    }))
                    .build();
        firstShot = robot.trajectoryBuilder(wobbleDrop.end())
                .addTemporalMarker(0.5, ()->{
                    robot.shooter.setVelocity(Robot.shootingPose.vec());
                    telemetry.addData("distance", Coordinate.distanceToLine(Robot.shootingPose, Robot.goal.getX()));
                    telemetry.addData("velo", robot.shooter.getTargetVelo());
                    telemetry.update();
                    robot.shooter.flapDown();
                })
                .lineToLinearHeading(Coordinate.toPose(Robot.shootingPose.vec(), Math.toRadians(180)))
                .addDisplacementMarker(()-> {
                    robot.intake(0);
                    robot.shooter.magUp();
                    robot.shooter.safeLeftOut();
                    robot.turn(Math.toRadians(-18));
                    Vector2d goalPost = Robot.goal.plus(new Vector2d(0, -10));
                    Pose2d position = robot.getPoseEstimate();
                    double absAngleToTarget = Math.atan2(goalPost.getY() - position.getY(), goalPost.getX() - position.getX());
                    double relAngleToPoint = AngleWrap(absAngleToTarget - robot.getPoseEstimate().getHeading());
                    robot.turn(relAngleToPoint);
                })
                .build();
        if(Robot.height == UGContourRingPipeline.Height.ZERO) wobblePickup = robot.trajectoryBuilder(Robot.shootingPose)
                .lineToSplineHeading(Coordinate.toPose(Robot.rightWobble, Math.toRadians(-30)),
                        getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(()->Async.start(()->{
                    robot.grab();
                    sleep(800);
                    robot.wobbleArmUp();
                }))
                .splineToConstantHeading(new Vector2d(-40, -40), Math.toRadians(90),
                        getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        else wobblePickup = robot.trajectoryBuilder(Robot.shootingPose)
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
                    robot.shooter.setVelocity(new Vector2d(-29, Robot.goal.getY()));
                })
                .splineToConstantHeading(new Vector2d(-40, -40), Math.toRadians(90),
                        getVelocityConstraint(35, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(-37, Robot.goal.getY() + 1), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-29, Robot.goal.getY(), Math.toRadians(1
                        )), Math.toRadians(0),
                        getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        finalShot = robot.trajectoryBuilder(wobblePickup.end())
                .addDisplacementMarker(() -> robot.intake(1))
                .lineToLinearHeading(Robot.shootingPoseTele)
                .build();
        if(Robot.height == UGContourRingPipeline.Height.FOUR)
            wobbleDrop2 = robot.trajectoryBuilder(finalShot.end())
                    .lineToLinearHeading(Coordinate.toPose(dropZone.plus(new Vector2d(-4, 6)), Math.toRadians(130)))
                    .addDisplacementMarker(() -> Async.start(() -> {
                        robot.shooter.leftOut();
                        robot.release();
                        sleep(60);
                        robot.wobbleArmUp();
                    }))
                    .build();
        else if(Robot.height == UGContourRingPipeline.Height.ONE)
            wobbleDrop2 = robot.trajectoryBuilder(wobblePickup.end())
                    .lineToLinearHeading(Coordinate.toPose(dropZone.plus(new Vector2d(-8, -5)), Math.toRadians(180)))
                    .addDisplacementMarker(() -> Async.start(() -> {
                        robot.shooter.leftOut();
                        robot.release();
                        sleep(60);
                        robot.wobbleArmUp();
                    }))
                    .build();
        else wobbleDrop2 = robot.trajectoryBuilder(wobblePickup.end())
                    .lineToLinearHeading(Coordinate.toPose(dropZone.plus(new Vector2d(15, 14)), Math.toRadians(90)))
                    .addDisplacementMarker(() -> Async.start(() -> {
                        robot.shooter.leftOut();
                        robot.release();
                        sleep(60);
                        robot.wobbleArmUp();
                    }))
                    .build();
        if(Robot.height == UGContourRingPipeline.Height.FOUR)
            park = robot.trajectoryBuilder(wobbleDrop2.end())
                    .lineToLinearHeading(new Pose2d(33, -54, Math.toRadians(120)))
                    .build();
        else park = robot.trajectoryBuilder(wobbleDrop2.end())
                .forward(10)
                .build();
    }
}
