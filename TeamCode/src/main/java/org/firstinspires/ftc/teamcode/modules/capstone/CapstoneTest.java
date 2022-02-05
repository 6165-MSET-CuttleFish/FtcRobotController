package org.firstinspires.ftc.teamcode.modules.capstone;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;

import org.firstinspires.ftc.teamcode.modules.ModuleTest;
import org.firstinspires.ftc.teamcode.modules.wrappers.ControllableServos;

@TeleOp
public class CapstoneTest extends ModuleTest {
    Servo turntable, pointer;
    CRServo tape;

    @Override
    public void initialize() {
        pointer = hardwareMap.servo.get("capstoneWrist");
        tape = hardwareMap.crservo.get("capstoneExtension");
        turntable = hardwareMap.servo.get("capstoneBase");
    }

    @Override
    public void update() {
        if(gamepad1.left_bumper){
            turntable.setPosition((turntable.getPosition()-0.05)%1);
        }else if(gamepad1.right_bumper){
            turntable.setPosition((turntable.getPosition()+0.05)%1);
        }if (gamepad1.x&&pointer.getPosition()<0.5){
            pointer.setPosition((pointer.getPosition()+0.05));
        }else if (gamepad1.y&&pointer.getPosition()>0.0){
            pointer.setPosition((pointer.getPosition()-0.05));
        }if (gamepad1.a){
            tape.setPower(0);
        }else if (gamepad1.b){
            tape.setPower(1);
        }else {
            tape.setPower(0.5);
        }
    }
}
