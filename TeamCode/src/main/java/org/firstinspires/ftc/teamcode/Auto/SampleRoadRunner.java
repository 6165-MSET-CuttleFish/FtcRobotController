package org.firstinspires.ftc.teamcode.Auto;



import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.PurePursuit.Coordinate;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.concurrent.Callable;

@Autonomous
class SampleRoadRunner extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(DcMotor.RunMode.RUN_WITHOUT_ENCODER, hardwareMap,0, 0,telemetry, this::opModeIsActive);
        robot.autoInit();
        SampleMecanumDrive driveTrain = new SampleMecanumDrive(hardwareMap);

        waitForStart();

//        Trajectory goForward = driveTrain.trajectoryBuilder(new Pose2d(0, 0, 0))
//                .forward(50)
//                .build();
//        driveTrain.followTrajectory(goForward);
//
//        Trajectory lineToPosition = driveTrain.trajectoryBuilder(new Pose2d(10, 20, 0), 0)
//                .lineTo(new Vector2d(0, 0))
//                .build();
//        driveTrain.followTrajectory(lineToPosition);
//        Trajectory strafeLeft = driveTrain.trajectoryBuilder(new Pose2d(0, 0, 0), 0)
//                .strafeLeft(50)
//                .build();
//        Trajectory strafeToPosition = driveTrain.trajectoryBuilder(new Pose2d(0, 0, 0), 0)
//                .strafeTo(new Vector2d(0, 50))
//                .build();
        //driveTrain.followTrajectory(strafeToPosition);
        Trajectory splineToPosition = driveTrain.trajectoryBuilder(new Pose2d(0, 0, Math.toRadians(90)))
                .splineTo(new Vector2d(0, 0), 50)
                .build();
        robot.stateMachine.setState(() -> Coordinate.toPoint(driveTrain.getPoseEstimate()).distanceTo(Robot.B) <= 20, robot::wobbleArmDown);
        driveTrain.followTrajectory(splineToPosition);
    }
}
