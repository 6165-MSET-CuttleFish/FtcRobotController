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

import java.util.Arrays;
import java.util.Vector;

@Autonomous(name = "newAuto", group = "LinearOpMode")
public class BounceBackAuto extends LinearOpMode {
    Robot robot;
    Coordinate targetPos;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, 9, 48, 0, OpModeType.auto);
        //robot.autoInit();
        Trajectory powerShotsTraj = robot.driveTrain.trajectoryBuilder(Robot.robotPose)
                .addTemporalMarker(0.6, ()->robot.launcher.rightOut())
                .splineTo(Robot.pwrShotLocals[2], 0, new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(46, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(()->Async.start(()->robot.launcher.singleRound()))
                .build();
        Trajectory powerShotsTrajCont = robot.driveTrain.trajectoryBuilder(powerShotsTraj.end())
                .splineToConstantHeading(Robot.pwrShotLocals[1], Math.toRadians(90), new MinVelocityConstraint(
                        Arrays.asList(
                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                new MecanumVelocityConstraint(15, DriveConstants.TRACK_WIDTH)
                        )
                ),
                new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(()->Async.start(()->robot.launcher.singleRound()))
                .build();
        Trajectory powerShotsContCont = robot.driveTrain.trajectoryBuilder(powerShotsTrajCont.end())
                .splineToConstantHeading(Robot.pwrShotLocals[0], Math.toRadians(90), new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(15, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(()->Async.start(()->robot.launcher.singleRound()))
                .build();
//        Trajectory wobbleDropA = robot.driveTrain.trajectoryBuilder(powerShotsTraj.end())
//                .splineTo(Robot.A, Math.toRadians(-180))
//                .addTemporalMarker(0.3, ()->robot.wobbleArmDown())
//                .splineToLinearHeading(Robot.shootingPose, 0)
//                .build();
//        Trajectory wobbleDropB = robot.driveTrain.trajectoryBuilder(powerShotsTraj.end())
//                .splineTo(Robot.B, Math.toRadians(-180))
//                .addTemporalMarker(0.3, ()->robot.wobbleArmDown())
//                .splineToLinearHeading(Robot.shootingPose, 0)
//                .build();
        Trajectory wobbleDropC = robot.driveTrain.trajectoryBuilder(powerShotsTraj.end())
                .addTemporalMarker(0.3, ()->robot.launcher.wingsVert())
                .splineTo(new Vector2d(108, 85), 0)
                .splineTo(new Vector2d(129, 80), Math.toRadians(-90))
                .splineTo(new Vector2d(129, 60), Math.toRadians(-90))
                .splineTo(Robot.C, Math.toRadians(-180))
                .addDisplacementMarker(()->{
                    robot.release();
                    robot.intake(0);
                    robot.launcher.setLauncherVelocity(robot.getPoseVelo(Robot.shootingPose));
                    robot.launcher.flapDown();
                    robot.launcher.wingsOut();
                })
                //.splineToLinearHeading(Robot.shootingPose, 0)
                .build();
        Trajectory firstShot = robot.driveTrain.trajectoryBuilder(wobbleDropC.end())
                .lineToLinearHeading(Robot.shootingPose)
                .addDisplacementMarker(()->Async.start(()->{
                    telemetry.addData("rings", robot.launcher.getRings());
                    telemetry.update();
                    //robot.launcher.magazineShoot();
                }))
                .build();
        Trajectory wobblePickup = robot.driveTrain.trajectoryBuilder(firstShot.end())
                .splineToConstantHeading(Robot.rightWobble, 0)
                .addDisplacementMarker(()->{
                    robot.intake(1);
                    robot.launcher.setVelocity(robot.getPoseVelo(new Vector2d(30, 40)));
                })
                .splineToConstantHeading(new Vector2d(30, 30), 0)
                .splineTo(new Vector2d(30, 40), Math.toRadians(2))
                .addDisplacementMarker(()->robot.intake(0))
                .build();
//        Trajectory stackIntake = robot.driveTrain.trajectoryBuilder(wobblePickup.end())
//                .splineTo(new Vector2d(30, 30), Math.toRadians(-2), new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(20, DriveConstants.TRACK_WIDTH)
//                                )
//                        ),
//                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .addTemporalMarker(0.3, ()->{
//                    robot.launcher.setLauncherVelocity();(1170);
//                    robot.intake(1);
//                })
//                .build();
        sleep(700);
        telemetry.addData("Initialization", "Complete");
        telemetry.update();
        robot.launcher.tiltUp();
        robot.grab();
        Thread shooterThread = new Thread(()->{
            while(opModeIsActive()){
                robot.launcher.updatePID();
                Robot.robotPose = robot.driveTrain.getPoseEstimate();
            }
        });
//        while (!opModeIsActive()) {
//            robot.scan();
//            telemetry.addData("Stack Height", robot.height);
//            telemetry.addData("Discs", robot.dice.toString());
//            telemetry.update();
//        }
        waitForStart();
        shooterThread.start();
        robot.turnOffVision();
        robot.launcher.flapUp();
        robot.wobbleArmUp();
        robot.launcher.setLauncherVelocity(907);
        robot.unlockIntake();
        robot.driveTrain.followTrajectory(powerShotsTraj);
        robot.driveTrain.followTrajectory(powerShotsTrajCont);
        robot.driveTrain.followTrajectory(powerShotsContCont);
        sleep(150);
        robot.launcher.setLauncherVelocity(0);
        Async.set(()->Robot.C.distTo(robot.driveTrain.getPoseEstimate().vec()) < 8, ()-> robot.wobbleArmDown());
        robot.intake(1);
        robot.driveTrain.followTrajectory(wobbleDropC);
        /*
        PICK UP BOUNCEBACKS
         */
        robot.driveTrain.followTrajectory(firstShot);
        robot.launcher.magazineShoot();
        sleep(150);
        robot.launcher.setLauncherVelocity(0);
        Async.set(()->Robot.rightWobble.distTo(robot.driveTrain.getPoseEstimate().vec()) < 7, ()->{
            robot.grab();
            sleep(300);
            robot.wobbleArmUp();
        });
        robot.driveTrain.followTrajectory(wobblePickup);
        robot.launcher.tiltUp();
        sleep(150);
        robot.launcher.singleRound();
//        Async.start(()->Robot.rightWobble.distTo(robot.driveTrain.getPoseEstimate().vec()) < 20, ()->{
//            robot.grab();
//            sleep(300);
//            robot.wobbleArmUp();
//        });
//        switch(Robot.dice){
//            case one: robot.driveTrain.followTrajectory(wobbleDropA);
//            case two: robot.driveTrain.followTrajectory(wobbleDropB);
//            case three: robot.driveTrain.followTrajectory(wobbleDropC);
//        }
//        robot.driveTrain.followTrajectory(wobblePickup);
//        robot.driveTrain.followTrajectory(stackIntake);
    }
}
