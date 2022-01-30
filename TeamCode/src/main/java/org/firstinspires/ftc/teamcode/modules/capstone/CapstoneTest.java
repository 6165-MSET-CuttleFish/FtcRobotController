package org.firstinspires.ftc.teamcode.modules.capstone;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;

import org.firstinspires.ftc.teamcode.modules.ModuleTest;
import org.firstinspires.ftc.teamcode.modules.wrappers.ControllableServos;

@TeleOp
public class CapstoneTest extends ModuleTest {
    Servo wrist, arm,claw;
    boolean open=true;
    Capstone capstone;
    ControllableServos servoSet;

    @Override
    public void initialize() {
        capstone=new Capstone(hardwareMap);
        //claw = hardwareMap.servo.get("capstoneClaw");
    }

    @Override
    public void update() {
        if(gamepad1.a){
            telemetry.addData("State","ready");
            capstone.ready();
        }else if (gamepad1.b){
            telemetry.addData("State","pickup");
            capstone.pickUp();
        }else if (gamepad1.x){
            telemetry.addData("State","precap");
            capstone.preCap();
        }else if (gamepad1.y){
            telemetry.addData("State","cap");
            capstone.cap();
        }
        /*
        if(gamepad1.a) {
            servoSet.lock();
        }
         */
        telemetry.update();
    }
}
