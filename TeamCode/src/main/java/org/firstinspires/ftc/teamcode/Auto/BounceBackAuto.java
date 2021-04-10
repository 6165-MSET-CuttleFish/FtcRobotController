package org.firstinspires.ftc.teamcode.Auto;

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

@Autonomous(name = "newAuto", group = "LinearOpMode")
public class BounceBackAuto extends LinearOpMode {
    Robot robot;
    Vector2d dropZone;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, 9, 48, 0, OpModeType.auto);
        //robot.autoInit();
        Trajectory powerShotsTraj1 = robot.driveTrain.trajectoryBuilder(Robot.robotPose)
                .addTemporalMarker(0.6, ()->robot.launcher.rightOut())
                .splineTo(Robot.pwrShotLocals[2], 0, new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(47, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(()->Async.start(()->robot.launcher.singleRound()))
                .build();
        Trajectory powerShotsTraj2 = robot.driveTrain.trajectoryBuilder(powerShotsTraj1.end())
                .splineToConstantHeading(Robot.pwrShotLocals[1], Math.toRadians(90), new MinVelocityConstraint(
                        Arrays.asList(
                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                new MecanumVelocityConstraint(15, DriveConstants.TRACK_WIDTH)
                        )
                ),
                new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(()->Async.start(()->robot.launcher.singleRound()))
                .build();
        Trajectory powerShotsTraj3 = robot.driveTrain.trajectoryBuilder(powerShotsTraj2.end())
                .splineToConstantHeading(Robot.pwrShotLocals[0], Math.toRadians(90), new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(15, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(()->Async.start(()->robot.launcher.singleRound()))
                .build();
        Trajectory wobbleDrop = robot.driveTrain.trajectoryBuilder(powerShotsTraj1.end())
                .addTemporalMarker(0.3, ()->{
                    robot.launcher.wingsVert();
                    Async.set(()->Robot.C.distTo(robot.driveTrain.getPoseEstimate().vec()) < 8, ()-> robot.wobbleArmDown());
                })
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
                .build();
        Trajectory firstShot = robot.driveTrain.trajectoryBuilder(wobbleDrop.end())
                .lineToLinearHeading(Robot.shootingPose)
                .addDisplacementMarker(()->Async.start(()->{
                    telemetry.addData("rings", robot.launcher.getRings());
                    telemetry.update();
                    //robot.launcher.magazineShoot();
                }))
                .build();
        Trajectory wobblePickup = robot.driveTrain.trajectoryBuilder(firstShot.end())
                .addSpatialMarker(Robot.rightWobble, ()-> {
                    robot.grab();
                    sleep(300);
                    robot.wobbleArmUp();
                })
                .splineToConstantHeading(new Vector2d(23, 25), 0)
                .addDisplacementMarker(()-> robot.launcher.setVelocity(robot.getPoseVelo(new Vector2d(30, 40))))
                .splineToConstantHeading(new Vector2d(35, Robot.goal.getY()), 0)
                .addDisplacementMarker(()-> robot.intake(1))
                .splineTo(new Vector2d(38, Robot.goal.getY()), Math.toRadians(2), new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(12, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(()->{
                    robot.intake(0);
                    robot.launcher.magUp();
                })
                .build();
        Trajectory stackIntake = robot.driveTrain.trajectoryBuilder(wobblePickup.end())
                .addDisplacementMarker(()->robot.intake(1))
                .lineToLinearHeading(Robot.shootingPose)
                .addDisplacementMarker(()->{
                    robot.intake(0);
                    robot.launcher.magUp();
                })
                .build();
        Trajectory wobbleDrop2 = robot.driveTrain.trajectoryBuilder(stackIntake.end())
                .addDisplacementMarker(()-> Async.set(()-> robot.driveTrain.getPoseEstimate().vec().distTo(Robot.C) < 8, ()-> robot.wobbleArmDown()))
                .lineToLinearHeading(Coordinate.toPose(Robot.C, 0))
                .addDisplacementMarker(()-> robot.release())
                .build();
        Trajectory park = robot.driveTrain.trajectoryBuilder(wobbleDrop2.end())
                .splineTo(new Vector2d(80, wobbleDrop2.end().getX()), Math.toRadians(180))
                .build();
        sleep(700);
        telemetry.addData("Initialization", "Complete");
        telemetry.update();
        robot.launcher.magUp();
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
        robot.driveTrain.followTrajectory(powerShotsTraj1);
        robot.driveTrain.followTrajectory(powerShotsTraj2);
        robot.driveTrain.followTrajectory(powerShotsTraj3);
        robot.launcher.setLauncherVelocity(0);
        robot.intake(1);
        robot.driveTrain.followTrajectory(wobbleDrop);
        /*
        PICK UP BOUNCEBACKS
         */
        robot.driveTrain.followTrajectory(firstShot);
        robot.launcher.safeShoot();
        robot.launcher.setLauncherVelocity(0);
        robot.driveTrain.followTrajectory(wobblePickup);
        sleep(100);
        robot.launcher.singleRound();
        robot.launcher.magDown();
        robot.launcher.setVelocity(robot.getPoseVelo(Robot.shootingPose));
        robot.driveTrain.followTrajectory(stackIntake);
        sleep(100);
        robot.launcher.magazineShoot();
        robot.driveTrain.followTrajectory(wobbleDrop2);
        robot.driveTrain.followTrajectory(park);
    }
}
