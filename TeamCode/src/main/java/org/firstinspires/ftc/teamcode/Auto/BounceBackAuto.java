package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.PurePursuit.Coordinate;
import org.firstinspires.ftc.teamcode.PurePursuit.MathFunctions;

@Autonomous(name = "newAuto", group = "LinearOpMode")
public class BounceBackAuto extends LinearOpMode {
    Robot robot;
    Coordinate targetPos;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, 9, 35.5, 0, telemetry, this::opModeIsActive);
        robot.autoInit();
        sleep(1000);
        telemetry.addData("Initialization", "Complete");
        telemetry.update();
        robot.launcher.tiltUp();
        robot.grab();
        while (!opModeIsActive()) {
            robot.scan();
            telemetry.addData("Stack Height", robot.height);
            telemetry.addData("Discs", robot.dice.toString());
            telemetry.update();
        }
        waitForStart();
        robot.scan();
        robot.turnOffVision();
        robot.unlockIntake();
        robot.launcher.setFlyWheel(0.47);
        robot.knockPowerShots();
        /*
        PICK UP BOUNCEBACKS
         */
        //Robot.driveTrain.followTrajectory(robot.dropZone(), 10, ()->robot.wobbleArmDown());
        robot.launcher.setFlyWheel(0.55);
        //Robot.driveTrain.followTrajectory(Robot.shootingPath);
    }
}
