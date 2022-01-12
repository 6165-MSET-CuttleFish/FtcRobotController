package org.firstinspires.ftc.teamcode.modules.capstone;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;

import org.firstinspires.ftc.teamcode.modules.ModuleTest;

@TeleOp
public class CapstoneTest extends ModuleTest {
    Servo claw;
    boolean open=true;

    @Override
    public void initialize() {
       claw = hardwareMap.servo.get("capstoneClaw");
    }

    @Override
    public void update() {
        telemetry.addData("Open: ",open);
        telemetry.update();
        if(gamepad1.b){
            claw.setPosition(1);
            open=true;
        }else if (gamepad1.a){
            claw.setPosition(0.5);
            open=false;
        }
    }
}
