package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.Async;
import org.firstinspires.ftc.teamcode.Components.OpModeType;
import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.PurePursuit.Coordinate;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import static org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive.*;

@Autonomous(name = "newAuto", group = "LinearOpMode")
public class BounceBackAuto extends LinearOpMode {
    Robot robot;
    Trajectory firstShot;
    Trajectory wobbleDrop;
    Trajectory powerShotsTraj3;
    Trajectory powerShotsTraj2;
    Trajectory powerShotsTraj1;
    Trajectory wobblePickup;
    Trajectory finalShot;
    Trajectory wobbleDrop2;
    Trajectory park;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, 8.5, 47.8125, 0, OpModeType.auto, this);
        robot.autoInit();
        powerShotsTraj1 = robot.driveTrain.trajectoryBuilder(Robot.robotPose)
                .addTemporalMarker(0.5, () -> robot.launcher.safeLeftOut())
                //.lineToSplineHeading(new Pose2d(-40.4725, -21))
                .lineToLinearHeading(Coordinate.toPose(Robot.pwrShotLocals[0],0),
                        getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> Async.start(() -> robot.launcher.singleRound()))
                .build();
        powerShotsTraj2 = robot.driveTrain.trajectoryBuilder(powerShotsTraj1.end())
                .lineToLinearHeading(Coordinate.toPose(Robot.pwrShotLocals[1], 0),
                        getVelocityConstraint(6.93, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> Async.start(() -> robot.launcher.singleRound()))
                .build();
        powerShotsTraj3 = robot.driveTrain.trajectoryBuilder(powerShotsTraj2.end())
                .lineToLinearHeading(Coordinate.toPose(Robot.pwrShotLocals[2], 0),
                        getVelocityConstraint(6.93, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> Async.start(() -> robot.launcher.singleRound()))
                .build();
        wobbleDrop = robot.driveTrain.trajectoryBuilder(powerShotsTraj3.end())
                .addTemporalMarker(0.3, () -> {
                    robot.launcher.wingsVert();
                    Async.set(() -> Robot.C.distTo(robot.driveTrain.getPoseEstimate().vec()) <= 11, () -> robot.wobbleArmDown());
                })
                .splineToConstantHeading(powerShotsTraj2.end().vec().plus(new Vector2d(5, 5)), 0)
                .splineTo(new Vector2d(45, 10), 0)
                .splineTo(new Vector2d(49, 10), 0)
                .splineTo(new Vector2d(57.8, 4), Math.toRadians(-75))
                .splineToConstantHeading(new Vector2d(58.6, -10.4725), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(58.6, -43.4725), Math.toRadians(-90))
                .splineToSplineHeading(Coordinate.toPose(Robot.C, Math.toRadians(-185)), Math.toRadians(-185))
                .addDisplacementMarker(() -> {
                    Async.start(() -> {
                        robot.release();
                        sleep(400);
                        robot.wobbleArmUp();
                    });
                    robot.launcher.setVelocity(1290);
                    telemetry.addData("distance", Coordinate.distanceToLine(Robot.shootingPose, Robot.goal.getX()));
                    telemetry.addData("velo", robot.launcher.getTargetVelo());
                    telemetry.update();
                    robot.launcher.flapDown();
                    robot.launcher.safeLeftOut();
                })
                .splineToConstantHeading(Robot.shootingPose.vec(), Math.toRadians(180))
                .addDisplacementMarker(()-> {
                    robot.intake(0);
                    robot.launcher.magUp();
                    robot.launcher.safeLeftOut();
                    robot.driveTrain.turn(Math.toRadians(-169));
                })
                .build();
        firstShot = robot.driveTrain.trajectoryBuilder(wobbleDrop.end())
                .splineToConstantHeading(Robot.shootingPose.vec(), Math.toRadians(180))
                .addDisplacementMarker(()-> {
                    robot.intake(0);
                    robot.launcher.magUp();
                    robot.launcher.safeLeftOut();
                    robot.driveTrain.turn(Math.toRadians(-169));
                })
                .build();
        wobblePickup = robot.driveTrain.trajectoryBuilder(Robot.shootingPose)
                .lineToSplineHeading(Coordinate.toPose(Robot.rightWobble, Math.toRadians(-3)),
                        getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(() -> {
                    //robot.launcher.setVelocity(robot.getPoseVelo(new Vector2d(-23, Robot.get)));
                    robot.launcher.setVelocity(robot.getPoseVelo(new Vector2d(-24, Robot.goal.getY())) - 50);
                    robot.intake(1);
                    robot.launcher.wingsIn();
                })
                .splineToConstantHeading(new Vector2d(-40, Robot.goal.getY()), Math.toRadians(-3),
                        getVelocityConstraint(28, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(-38.7, Robot.goal.getY()), Math.toRadians(-3),
                        getVelocityConstraint(18, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineToLinearHeading(new Pose2d(-24, Robot.goal.getY(), Math.toRadians(-3)),0,
                        getVelocityConstraint(8, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        finalShot = robot.driveTrain.trajectoryBuilder(wobblePickup.end())
                .addDisplacementMarker(() -> robot.intake(1))
                .lineToLinearHeading(Robot.shootingPoseTele)
                .build();
        wobbleDrop2 = robot.driveTrain.trajectoryBuilder(finalShot.end())
                .addDisplacementMarker(() -> Async.set(() -> robot.driveTrain.getPoseEstimate().vec().distTo(Robot.C) <= 15, () -> robot.wobbleArmDown()))
                .lineToLinearHeading(Coordinate.toPose(Robot.C.plus(new Vector2d(-4.5, 8)), Math.toRadians(130)))
                .addDisplacementMarker(() -> Async.start(() -> {
                    robot.launcher.leftOut();
                    robot.release();
                    sleep(60);
                    robot.wobbleArmUp();
                }))
                .build();
        park = robot.driveTrain.trajectoryBuilder(wobbleDrop2.end())
                .lineToLinearHeading(new Pose2d(33, -54, Math.toRadians(120)))
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
        robot.scan();
        Async.start(()-> {
            while (opModeIsActive()) {
                robot.launcher.updatePID();
                Robot.robotPose = robot.driveTrain.getPoseEstimate();
            }
        });
        //robot.turnOffVision();
        robot.launcher.flapUp();
        robot.wobbleArmUp();
        robot.launcher.setLauncherVelocity(909);
        Async.start(this::generatePaths);
        robot.switchPipeline();
        robot.launcher.unlockIntake();
        //Async.start(this::generatePaths);
        sleep(700);
        robot.driveTrain.followTrajectory(powerShotsTraj1);
        robot.driveTrain.followTrajectory(powerShotsTraj2);
        robot.driveTrain.followTrajectory(powerShotsTraj3);
        sleep(40);
        robot.launcher.setLauncherVelocity(0);
        robot.intake(1);
        if(Robot.height == UGContourRingPipeline.Height.FOUR){
            path4();
        } else if(Robot.height == UGContourRingPipeline.Height.ONE){
            path1();
        } else {
            path0();
        }
    }
    private void path4(){
        robot.driveTrain.followTrajectory(wobbleDrop);
        robot.driveTrain.followTrajectory(firstShot);
        robot.wobbleArmDown();
        robot.optimalShoot(robot.launcher.getRings());
        sleep(40);
        robot.launcher.setLauncherVelocity(0);
        robot.wobbleArmDown();
        Async.set(() -> robot.driveTrain.getPoseEstimate().vec().distTo(Robot.rightWobble) <= 16.5, () -> {
            robot.grab();
            sleep(1000);
            robot.wobbleArmUp();
        });
        robot.driveTrain.followTrajectory(wobblePickup);
        sleep(200);
        robot.intake(-1);
        robot.launcher.magUp();
        sleep(190);
        robot.launcher.singleRound();
        robot.launcher.setVelocity(robot.getPoseVelo(Robot.shootingPoseTele));
        sleep(40);
        robot.launcher.magDown();
        robot.driveTrain.followTrajectory(finalShot);
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
    private void path1(){
        robot.driveTrain.followTrajectory(wobbleDrop);
        robot.driveTrain.followTrajectory(firstShot);
        robot.wobbleArmDown();
        robot.optimalShoot(robot.launcher.getRings());
        sleep(40);
        robot.launcher.setLauncherVelocity(0);
        robot.wobbleArmDown();
        Async.set(() -> robot.driveTrain.getPoseEstimate().vec().distTo(Robot.rightWobble) <= 16.5, () -> {
            robot.grab();
            sleep(1000);
            robot.wobbleArmUp();
        });
        robot.driveTrain.followTrajectory(wobblePickup);
        robot.launcher.magUp();
        sleep(190);
        robot.launcher.singleRound();
        sleep(40);
        robot.launcher.setLauncherVelocity(0);
        robot.driveTrain.followTrajectory(wobbleDrop2);
        robot.driveTrain.followTrajectory(park);
    }
    private void path0(){

    }
    private void generatePaths(){
        Vector2d dropZone;
        if(Robot.height == UGContourRingPipeline.Height.FOUR){
            dropZone = Robot.C;
        } else if(Robot.height == UGContourRingPipeline.Height.ONE){
            dropZone = Robot.B;
        } else {
            dropZone = Robot.A;
        }
        firstShot = robot.driveTrain.trajectoryBuilder(Coordinate.toPose(dropZone, Math.toRadians(-190)))
                .splineToConstantHeading(Robot.shootingPose.vec(), Math.toRadians(180))
                .addDisplacementMarker(()-> {
                    robot.intake(0);
                    robot.launcher.magUp();
                    robot.launcher.safeLeftOut();
                    robot.driveTrain.turn(Math.toRadians(-169));
                })
                .build();
        if(Robot.height == UGContourRingPipeline.Height.ZERO){
            wobblePickup = robot.driveTrain.trajectoryBuilder(Robot.shootingPose)
                    .lineToSplineHeading(Coordinate.toPose(Robot.rightWobble, Math.toRadians(-3)),
                            getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .splineToConstantHeading(new Vector2d(-40, Robot.goal.getY()), Math.toRadians(-3),
                            getVelocityConstraint(28, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .addDisplacementMarker(()->Async.set(() -> robot.driveTrain.getPoseEstimate().vec().distTo(dropZone) <= 15, () -> robot.wobbleArmDown()))
                    .splineToLinearHeading(Coordinate.toPose(dropZone.plus(new Vector2d(5, 10)), Math.toRadians(90)), 0)
                    .build();
        }
    }
}
