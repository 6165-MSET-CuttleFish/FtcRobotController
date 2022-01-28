package org.firstinspires.ftc.teamcode.modules.capstone;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;

import org.firstinspires.ftc.teamcode.modules.ModuleTest;

@TeleOp
public class CapstoneTest extends ModuleTest {
    Servo wrist;
    boolean open=true;
    Capstone capstone;

    @Override
    public void initialize() {
       wrist = hardwareMap.servo.get("capstoneClaw");
    }

    @Override
    public void update() {
        telemetry.addData("Open: ",!open);
        telemetry.addData("Pos: ",wrist.getPosition());
        telemetry.update();
        if(gamepad1.b){
            wrist.setPosition(0);
            open=true;
        }else if (gamepad1.a){
            wrist.setPosition(0.2);
            open=false;
        }
/*
        if(gamepad1.a){
            capstone.ready();
        }else if (gamepad1.b){
            capstone.pickUp();
        }else if (gamepad1.x){
            capstone.preCap();
        }else if (gamepad1.y){
            capstone.cap();
        }
*/
    }
}
