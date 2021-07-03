package org.firstinspires.ftc.teamcode.Tele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Magazine;
@TeleOp
public class MagTest extends LinearOpMode {
    Magazine magazine;
    @Override
    public void runOpMode() throws InterruptedException {
        magazine = new Magazine(hardwareMap);
        waitForStart();
        while(opModeIsActive()){
            magazine.update();
            if(gamepad1.a){
                magazine.up();
            }
            if(gamepad1.b){
                magazine.down();
            }
            if(gamepad1.x){
                magazine.magMacro();
            }
        }
    }
}
