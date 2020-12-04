package org.firstinspires.ftc.teamcode.Tele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOpFinal", group = "LinearOpMode")
public class Wobble extends LinearOpMode {

    public Servo arm1, arm2;
    public Boolean armUp;
    public void runOpMode() throws InterruptedException {
        arm1 = hardwareMap.get(Servo.class, "wobbleArm1");
        arm2 = hardwareMap.get(Servo.class, "wobbleArm2");
        arm1.setDirection(Servo.Direction.REVERSE);
        waitForStart();
        while(opModeIsActive()){
            if(gamepad2.b == true && armUp == false){
                arm1.setPosition(0.92);
                arm2.setPosition (0.92);
                armUp = true;
                wait(100);
                //lastTime = System.currentTimeMillis();
            }
            else if(gamepad2.b == true && armUp == true){

                arm1.setPosition(0.13);
                arm2.setPosition (0.13);
                armUp = false;
                wait(100);
                //lastTime = System.currentTimeMillis();
            }
        }
    }
}
