package org.firstinspires.ftc.teamcode.modules.capstone;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;

import org.firstinspires.ftc.teamcode.modules.ModuleTest;

@TeleOp
public class CapstoneTest extends ModuleTest {
    Servo wrist, arm,claw;
    boolean open=true;
    Capstone capstone;

    @Override
    public void initialize() {
        capstone=new Capstone(hardwareMap);
    }

    @Override
    public void update() {
        if(gamepad1.a){
            capstone.ready();
        }else if (gamepad1.b){
            capstone.pickUp();
        }else if (gamepad1.x){
            capstone.preCap();
        }else if (gamepad1.y){
            capstone.cap();
        }
    }
}
