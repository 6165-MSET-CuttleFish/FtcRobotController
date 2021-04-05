package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Components.Async;
import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.PurePursuit.Coordinate;
import org.firstinspires.ftc.teamcode.PurePursuit.MathFunctions;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;

import java.util.Arrays;

@Autonomous(name = "newAuto", group = "LinearOpMode")
public class BounceBackAuto extends LinearOpMode {
    Robot robot;
    Coordinate targetPos;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, 9, 48, 0);
        robot.autoInit();
        Trajectory trajectory = robot.driveTrain.trajectoryBuilder(robot.startPose)
                .splineTo(Robot.pwrShotLocals[2], 0)
                .addDisplacementMarker(()->Async.start(()->true, ()->robot.launcher.singleRound()))
                .splineToConstantHeading(Robot.pwrShotLocals[1], 0, new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(9, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(()->Async.start(()->true, ()->robot.launcher.singleRound()))
                .splineToConstantHeading(Robot.pwrShotLocals[0], Math.toRadians(-2), new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(9, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker(()->Async.start(()->true, ()->robot.launcher.singleRound()))
                .build();
        sleep(1000);
        telemetry.addData("Initialization", "Complete");
        telemetry.update();
        robot.launcher.tiltUp();
        robot.grab();
        Thread shooterThread = new Thread(()->{
            while(opModeIsActive()){
                robot.launcher.updatePID();
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
       // robot.scan();
        robot.turnOffVision();
        //robot.unlockIntake();
        robot.launcher.flapUp();
        robot.launcher.setVelocity(915);
        robot.driveTrain.followTrajectory(trajectory);
//        robot.driveTrain.followTrajectory(newTraj);
//        robot.launcher.singleRound();
//        robot.driveTrain.followTrajectory(newTraj1);
//        robot.launcher.singleRound();
        sleep(500);
        robot.launcher.setVelocity(0);
        /*
        PICK UP BOUNCEBACKS
         */
        //Robot.driveTrain.followTrajectory(robot.dropZone(), 10, ()->robot.wobbleArmDown());
        //robot.launcher.setFlyWheel(0.55);
        //Robot.driveTrain.followTrajectory(Robot.shootingPath);
    }
}
