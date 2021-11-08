package org.firstinspires.ftc.teamcode.modules.deposit;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.modules.Module;
import org.firstinspires.ftc.teamcode.util.Details;

@TeleOp
public class TestOpModeDepo extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Platform platform = new Platform(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            platform.update();
            if(gamepad1.a){
               platform.dump();
            }
            telemetry.addData("Seconds", platform.elapsedTime.seconds());
            telemetry.addData("State", platform.getState().toString());
            telemetry.addData("Target", platform.getState().time);
            telemetry.update();
        }
    }
}
