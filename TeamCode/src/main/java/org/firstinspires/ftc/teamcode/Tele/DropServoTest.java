package org.firstinspires.ftc.teamcode.Tele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Intake;

@TeleOp(name = "Intake Drop Test", group = "test")
public class DropServoTest extends LinearOpMode {
    public Intake intake;
    public void runOpMode() throws InterruptedException{
        intake = new Intake(hardwareMap);

        waitForStart ();
        boolean drop = false;
        while(opModeIsActive()){
            if(!drop) {
                intake.dropIntake();
                sleep(600);
                intake.raiseIntake ();
                drop = true;
            }
            intake.update();
            telemetry.update ();
        }
    }
}
