package org.firstinspires.ftc.teamcode.modules.capstone;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TestOpModeClaw extends OpMode {
    Capstone capstone;

    @Override
    public void init() {
        capstone = new Capstone(hardwareMap);
        capstone.ready();
    }

    @Override
    public void loop() {
        if(gamepad1.a){
           capstone.pickUp();
        }if(gamepad1.b){
            capstone.precap();
        }
        if(gamepad1.x){
            capstone.cap();
        }
    }
}
