package org.firstinspires.ftc.teamcode.Tele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "MTIShooterTest", group = "LinearOpMode")
public class NewShooterTest extends LinearOpMode {
    public Servo mag;
    public DcMotor s1,s2;
    public void runOpMode(){
        mag = hardwareMap.get(Servo.class, "spanker");
        s1 = hardwareMap.get(DcMotor.class, "s1" );
        s2 = hardwareMap.get(DcMotor.class, "s2");
        s1.setDirection(DcMotor.Direction.REVERSE);
        s2.setDirection(DcMotor.Direction.REVERSE);
        mag.setPosition(1);
        waitForStart();
        while(opModeIsActive ()){
            if(gamepad1.b){
                s1.setPower(1);
                s2.setPower(1);
            }
            else{
                s1.setPower(0);
                s2.setPower(0);
            }
            if(gamepad1.a){
                    mag.setPosition(0.8);

            }
            if(gamepad1.x) mag.setPosition(1);
        }
    }
}
