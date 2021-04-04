package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Components.Async;
import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.PurePursuit.Coordinate;

import static org.firstinspires.ftc.teamcode.PurePursuit.MathFunctions.AngleWrap;

@Autonomous
public class autoTest extends LinearOpMode {
    Robot robot;
    @Override
    public void runOpMode() throws InterruptedException{
        robot = new Robot(hardwareMap, 0, 0, 0);
        robot.init();
        waitForStart();
        Trajectory trajectoryForward = robot.driveTrain.trajectoryBuilder(new Pose2d())
                .forward(50)
                .build();

        Trajectory trajectoryBackward = robot.driveTrain.trajectoryBuilder(trajectoryForward.end())
                .back(50)
                .build();
        //Async.start(()->opModeIsActive(), ()->robot.wobbleArmDown());
        while(opModeIsActive()){
            robot.driveTrain.followTrajectory(trajectoryForward);
            robot.driveTrain.followTrajectory(trajectoryBackward);
        }
    }
}
