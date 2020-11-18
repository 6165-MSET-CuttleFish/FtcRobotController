package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Components.Robot;

@Autonomous
public class testAutoImplement extends LinearOpMode{
    Robot robot;
    public void runOpMode() throws InterruptedException{
        robot = new Robot(DcMotor.RunMode.RUN_USING_ENCODER, hardwareMap, 0, 0, 18, 18);

    }
}
