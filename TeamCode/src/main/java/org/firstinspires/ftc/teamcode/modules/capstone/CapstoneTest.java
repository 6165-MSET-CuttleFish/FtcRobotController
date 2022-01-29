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

    @Override
    public void initialize() {
        //capstone=new Capstone(hardwareMap);
        wrist = hardwareMap.servo.get("capstoneWrist");
    }

    @Override
    public void update() {
        /*if(gamepad1.a){
            capstone.ready();
        }else if (gamepad1.b){
            capstone.pickUp();
        }else if (gamepad1.x){
            capstone.preCap();
        }else if (gamepad1.y){
            capstone.cap();
        }
         */
        ControllableServos servoSet= new ControllableServos(wrist);
        servoSet.init(0);
        servoSet.lock();
    }
}
