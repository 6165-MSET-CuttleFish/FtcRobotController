package org.firstinspires.ftc.teamcode.Tele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.Robot;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Components.PIDController;
import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.Odometry.OdometryGlobalCoordinatePosition;
import org.firstinspires.ftc.teamcode.PurePursuit.Coordinate;

import java.io.DataInput;
import java.util.ArrayList;
import java.util.Collections;

import static org.firstinspires.ftc.teamcode.PurePursuit.MathFunctions.AngleWrap;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="ShooterTest", group = "LinearOpMode")
public class ShooterTest extends LinearOpMode
{
    Robot robot;
    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Robot (hardwareMap, telemetry, this::opModeIsActive);

       // robot = new Robot (hardwareMap, telemetry, () -> opModeIsActive() && gamepadIdle());
        double targetVelo = 1000;
        robot.init();
        //wingDefault = ()->robot.launcher.wingsVert();
        // pidRotate = new PIDController(.07, 0.014, 0.0044);
        Thread shooterThread = new Thread(()->{
            while(opModeIsActive()){
                robot.launcher.updatePID();
            }
        });
        waitForStart();
        shooterThread.start();
        while(opModeIsActive()){
            if(gamepad1.dpad_up){
                targetVelo += 20;
                sleep(300);
            } else if(gamepad1.dpad_down){
                targetVelo -= 10;
                sleep(300);
            }
            if(gamepad1.left_trigger >= 0.1) {
                robot.launcher.setVelocity(targetVelo);
            } else {
                robot.launcher.setVelocity(0);
                if (gamepad1.left_bumper) robot.launcher.tiltUp();
                else robot.launcher.tiltDown();
            }
            telemetry.addData("targetVelo ", targetVelo);
            telemetry.addData("current velo ", robot.launcher.getVelocity());
        }
    }

}
