package org.firstinspires.ftc.teamcode.modules.capstone;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.opmode.ModuleTest;

@TeleOp
public class TestOpModeClaw extends ModuleTest {
    Capstone capstone;

    @Override
    public void init() {
        capstone =new Capstone(hardwareMap);
        init(capstone);
        capstone.ready();
    }

    @Override
    public void loop() {
        update();
        if(gamepad1.a){
           capstone.pickUp();
        }if(gamepad1.b){
            capstone.cap();
        }
    }
}
