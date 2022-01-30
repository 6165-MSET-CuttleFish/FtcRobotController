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
    Claw capstoneClaw;
    ControllableServos servoSet;

    @Override
    public void initialize() {
        capstone=new Capstone(hardwareMap);
        capstoneClaw=new Claw(hardwareMap);
        //claw = hardwareMap.servo.get("capstoneClaw");
    }

    @Override
    public void update() {
        if(gamepad1.a){
            telemetry.addData("State","ready");
        }else if (gamepad1.b){
            telemetry.addData("State","pickup");
            capstoneClaw.pickUp();
        }else if (gamepad1.x){
            telemetry.addData("State","precap");
        }else if (gamepad1.y){
            telemetry.addData("State","cap");
            capstoneClaw.dropDown();
        }
        /*
        if(gamepad1.a) {
            servoSet.lock();
        }
         */
        telemetry.addData("",capstoneClaw.getState());
        telemetry.update();
    }
}
