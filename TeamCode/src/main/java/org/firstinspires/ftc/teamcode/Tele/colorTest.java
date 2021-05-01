package org.firstinspires.ftc.teamcode.Tele;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Robot;

@TeleOp(name="colorTest", group = "LinearOpMode")
public class colorTest extends LinearOpMode {

    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        waitForStart();
        while(opModeIsActive()){
            tiltMag();
            telemetry.addData("rings", robot.launcher.getRings());
            telemetry.addData("distance", robot.launcher.getDistance());
            telemetry.update();
            if(gamepad1.a){
                robot.launcher.wingsOut();
            } if(gamepad1.b) robot.launcher.wingsVert();
        }
    }
    public void tiltMag() {
        if(gamepad2.left_bumper) robot.launcher.magDown();
        else if(gamepad2.right_bumper) robot.launcher.magUp();
    }

    private boolean gamepadIdle(){
        return gamepad1.left_stick_x == 0 && gamepad1.left_stick_y == 0 && gamepad1.right_stick_x == 0;
    }
}
