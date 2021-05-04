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

@Autonomous(name = "Tourney Auto", group = "Op")
public class ChosenAuto extends LinearOpMode {
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
        robot = new Robot(hardwareMap, 8.5, 47.8125, 0, OpModeType.auto, this);
        robot.autoInit();
        powerShotsTraj1 = robot.driveTrain.trajectoryBuilder(Robot.robotPose)
                .addTemporalMarker(0.5, () -> {
                    robot.launcher.flapUp();
                    robot.launcher.safeLeftOut();
                })
                .lineToLinearHeading(Coordinate.toPose(Robot.pwrShotLocals[0],0),
                        getVelocityConstraint(40, Math.toRadians(190), DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> Async.start(() -> robot.launcher.singleRound()))
                .build();
        powerShotsTraj2 = robot.driveTrain.trajectoryBuilder(powerShotsTraj1.end())
                .addTemporalMarker(0.3, ()->robot.launcher.wingsOut())
                .lineToLinearHeading(Coordinate.toPose(Robot.pwrShotLocals[1], 0),
                        getVelocityConstraint(7, Math.toRadians(240), DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> Async.start(() -> robot.launcher.singleRound()))
                .build();
        powerShotsTraj3 = robot.driveTrain.trajectoryBuilder(powerShotsTraj2.end())
                .lineToLinearHeading(Coordinate.toPose(Robot.pwrShotLocals[2], 0),
                        getVelocityConstraint(8, Math.toRadians(220), DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> Async.start(() -> robot.launcher.singleRound()))
                .build();
        telemetry.addData("Initialization", "Complete");
        telemetry.update();
        robot.launcher.magUp();
        robot.grab();
        while (!opModeIsActive()) {
            robot.scan();
            telemetry.addData("Stack Height", Robot.height.toString());
            telemetry.update();
        }
        waitForStart();
        Async.start(()-> {
            while (opModeIsActive()) {
                robot.launcher.updatePID();
                Robot.robotPose = robot.driveTrain.getPoseEstimate();
            }
        });
        robot.turnOffVision();
        robot.wobbleArmUp();
        robot.launcher.setLauncherVelocity(DriveConstants.BounceBackVelo);
        robot.launcher.unlockIntake();
        Async.start(this::generatePaths);
        sleep(200);
        robot.driveTrain.followTrajectory(powerShotsTraj1);
        robot.driveTrain.followTrajectory(powerShotsTraj2);
        robot.driveTrain.followTrajectory(powerShotsTraj3);
        sleep(80);
        robot.launcher.setLauncherVelocity(0);
        robot.intake(1);
        actionComplete.set(false);
        robot.driveTrain.followTrajectory(wobbleDrop, ()-> {
            if(robot.driveTrain.getPoseEstimate().vec().distTo(dropZone) < getDistance() && !actionComplete.get()) {
                robot.wobbleArmDown();
                actionComplete.set(true);
            }
        });
        robot.driveTrain.followTrajectory(firstShot);
        robot.wobbleArmDown();
        robot.optimalShoot(3);
        sleep(40);
        robot.launcher.setLauncherVelocity(0);
        if(Robot.height == UGContourRingPipeline.Height.FOUR){
            case4();
        } else if(Robot.height == UGContourRingPipeline.Height.ONE){
            case1();
        } else {
            case0();
        }
    }
    private void case0(){
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
        actionComplete.set(false);
        robot.driveTrain.followTrajectory(wobbleDrop2, ()->{
            if(robot.driveTrain.getPoseEstimate().vec().distTo(Robot.A) < 27 && !actionComplete.get()) {
                robot.wobbleArmDown();
                actionComplete.set(true);
            }
        });
        robot.driveTrain.followTrajectory(park);
    }
    private void case1(){
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
        sleep(110);
        robot.intake(-1);
        robot.launcher.magUp();
        sleep(200);
        robot.launcher.singleRound();
        sleep(40);
        robot.intake(0);
        robot.launcher.setLauncherVelocity(0);
        actionComplete.set(false);
        robot.driveTrain.followTrajectory(wobbleDrop2, ()->{
            if(robot.driveTrain.getPoseEstimate().vec().distTo(Robot.B) < 27 && !actionComplete.get()) {
                robot.wobbleArmDown();
                actionComplete.set(true);
            }
        });
        robot.driveTrain.followTrajectory(park);
    }
    private void case4(){
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
        robot.launcher.setVelocity(robot.getPoseVelo(Robot.shootingPoseTele) - 80);
        sleep(40);
        robot.launcher.magDown();
        robot.driveTrain.followTrajectory(finalShot);
        sleep(100);
        robot.intake(-1);
        robot.launcher.magUp();
        sleep(190);
        robot.optimalShoot(3);
        sleep(40);
        robot.intake(0);
        robot.launcher.setLauncherVelocity(0);
        actionComplete.set(false);
        robot.driveTrain.followTrajectory(wobbleDrop2, ()->{
            if(robot.driveTrain.getPoseEstimate().vec().distTo(Robot.C) < 20 && !actionComplete.get()) {
                robot.wobbleArmDown();
                actionComplete.set(true);
            }
        });
        robot.driveTrain.followTrajectory(park);
    }
    private void generatePaths(){
        dropZone = robot.getDropZone();
        TrajectoryBuilder tempBuilder = robot.driveTrain.trajectoryBuilder(powerShotsTraj3.end())
                .addTemporalMarker(0.7, () -> robot.launcher.wingsVert())
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
        firstShot = robot.driveTrain.trajectoryBuilder(wobbleDrop.end())
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
                    robot.driveTrain.turn(Math.toRadians(-18));
                    Vector2d goalPost = Robot.goal.plus(new Vector2d(0, -10));
                    Pose2d position = robot.driveTrain.getPoseEstimate();
                    double absAngleToTarget = Math.atan2(goalPost.getY() - position.getY(), goalPost.getX() - position.getX());
                    double relAngleToPoint = AngleWrap(absAngleToTarget - robot.driveTrain.getPoseEstimate().getHeading());
                    robot.driveTrain.turn(relAngleToPoint);
                })
                .build();
        if(Robot.height == UGContourRingPipeline.Height.ZERO) wobblePickup = robot.driveTrain.trajectoryBuilder(Robot.shootingPose)
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
        else wobblePickup = robot.driveTrain.trajectoryBuilder(Robot.shootingPose)
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
                .splineToSplineHeading(new Pose2d(-29, Robot.goal.getY(), Math.toRadians(1
                        )), Math.toRadians(0),
                        getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        finalShot = robot.driveTrain.trajectoryBuilder(wobblePickup.end())
                .addDisplacementMarker(() -> robot.intake(1))
                .lineToLinearHeading(Robot.shootingPoseTele)
                .build();
        if(Robot.height == UGContourRingPipeline.Height.FOUR)
            wobbleDrop2 = robot.driveTrain.trajectoryBuilder(finalShot.end())
                .lineToLinearHeading(Coordinate.toPose(dropZone.plus(new Vector2d(-4, 6)), Math.toRadians(130)))
                .addDisplacementMarker(() -> Async.start(() -> {
                    robot.launcher.leftOut();
                    robot.release();
                    sleep(60);
                    robot.wobbleArmUp();
                }))
                .build();
        else if(Robot.height == UGContourRingPipeline.Height.ONE)
            wobbleDrop2 = robot.driveTrain.trajectoryBuilder(wobblePickup.end())
                    .lineToLinearHeading(Coordinate.toPose(dropZone.plus(new Vector2d(-8, -5)), Math.toRadians(180)))
                    .addDisplacementMarker(() -> Async.start(() -> {
                        robot.launcher.leftOut();
                        robot.release();
                        sleep(60);
                        robot.wobbleArmUp();
                    }))
                    .build();
        else wobbleDrop2 = robot.driveTrain.trajectoryBuilder(wobblePickup.end())
                .lineToLinearHeading(Coordinate.toPose(dropZone.plus(new Vector2d(15, 14)), Math.toRadians(90)))
                .addDisplacementMarker(() -> Async.start(() -> {
                    robot.launcher.leftOut();
                    robot.release();
                    sleep(60);
                    robot.wobbleArmUp();
                }))
                .build();
        if(Robot.height == UGContourRingPipeline.Height.FOUR)
        park = robot.driveTrain.trajectoryBuilder(wobbleDrop2.end())
                .lineToLinearHeading(new Pose2d(33, -54, Math.toRadians(120)))
                .build();
        else park = robot.driveTrain.trajectoryBuilder(wobbleDrop2.end())
                .forward(10)
                .build();
    }
}
