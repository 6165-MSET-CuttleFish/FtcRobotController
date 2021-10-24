package org.firstinspires.ftc.teamcode.modules.capstone;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.modules.deposit.Platform;

@TeleOp
public class TestOpModeClaw extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Slides slides = new Slides(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            slides.update();
            if(gamepad1.a){
               slides.setState(Slides.State.TRANSIT_OUT);
            }
            telemetry.addData("Seconds", slides.elapsedTime.seconds());
            telemetry.addData("State", slides.getState().toString());
            telemetry.addData("Target", slides.getState().time);
            telemetry.update();
        }
    }
}
