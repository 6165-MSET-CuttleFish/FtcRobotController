package org.firstinspires.ftc.teamcode.Tele;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Robot;

@TeleOp(name="colorTest", group = "LinearOpMode")
public class colorTest extends LinearOpMode {

    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this);
        waitForStart();
        while(opModeIsActive()){
            tiltMag();
            telemetry.addData("rings", robot.shooter.getRings());
            telemetry.addData("distance", robot.shooter.getDistance());
            telemetry.update();
        }
    }
    public void tiltMag() {
        if(gamepad2.left_bumper) robot.shooter.magDown();
        else if(gamepad2.right_bumper) robot.shooter.magUp();
    }

    private boolean gamepadIdle(){
        return gamepad1.left_stick_x == 0 && gamepad1.left_stick_y == 0 && gamepad1.right_stick_x == 0;
    }
}
