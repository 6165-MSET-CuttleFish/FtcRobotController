package org.firstinspires.ftc.teamcode.modules.capstone;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.modules.ModuleTest;

@TeleOp
public class CapstoneTest extends ModuleTest {
    Capstone capstone;

    @Override
    public void initialize() {
        capstone = new Capstone(hardwareMap);
        capstone.ready();
        setModules(capstone);
    }

    @Override
    public void update() {
        if(gamepad1.a){
           capstone.pickUp();
        }if(gamepad1.b){
            capstone.preCap();
        }
        if(gamepad1.x){
            capstone.cap();
        }
    }
}
