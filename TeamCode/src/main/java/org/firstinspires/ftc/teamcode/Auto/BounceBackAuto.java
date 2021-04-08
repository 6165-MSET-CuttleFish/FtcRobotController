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
    Coordinate targetPos;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, 9, 48, 0, OpModeType.auto);
        //robot.autoInit();
        Trajectory powerShotsTraj = robot.driveTrain.trajectoryBuilder(Robot.robotPose)
                .splineTo(Robot.pwrShotLocals[2], 0, new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(48, DriveConstants.TRACK_WIDTH)
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
                .splineTo(new Vector2d(130, 80), Math.toRadians(-90))
                .splineTo(new Vector2d(128, 30), Math.toRadians(-90))
                .splineTo(Robot.C, Math.toRadians(-180))
                .addDisplacementMarker(()->{robot.release(); robot.intake(0);})
                //.splineToLinearHeading(Robot.shootingPose, 0)
                .build();
//        Trajectory wobblePickup = robot.driveTrain.trajectoryBuilder(wobbleDropA.end())
//                .splineToConstantHeading(Robot.rightWobble, 0)
//                .splineTo(new Vector2d(20, 30), 0)
//                .build();
//        Trajectory stackIntake = robot.driveTrain.trajectoryBuilder(wobblePickup.end())
//                .splineTo(new Vector2d(30, 30), Math.toRadians(-2), new MinVelocityConstraint(
//                                Arrays.asList(
//                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                        new MecanumVelocityConstraint(20, DriveConstants.TRACK_WIDTH)
//                                )
//                        ),
//                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .addTemporalMarker(0.3, ()->{
//                    robot.launcher.setVelocity(1170);
//                    robot.intake(1);
//                })
//                .build();
        sleep(800);
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
        robot.launcher.setVelocity(907);
        robot.driveTrain.followTrajectory(powerShotsTraj);
        robot.driveTrain.followTrajectory(powerShotsTrajCont);
        robot.driveTrain.followTrajectory(powerShotsContCont);
        sleep(500);
        robot.launcher.setVelocity(0);
        Async.set(()->Robot.C.distTo(robot.driveTrain.getPoseEstimate().vec()) < 8, ()-> robot.wobbleArmDown());
        robot.intake(1);
        robot.launcher.tiltDown();
        robot.driveTrain.followTrajectory(wobbleDropC);
        /*
        PICK UP BOUNCEBACKS
         */
//        Async.start(()->Robot.rightWobble.distTo(robot.driveTrain.getPoseEstimate().vec()) < 20, ()->{
//            robot.release();
//            sleep(200);
//            robot.wobbleArmUp();
//        });
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
