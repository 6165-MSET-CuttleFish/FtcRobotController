package org.firstinspires.ftc.teamcode.modules.intake;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.field.Details;

@TeleOp(group = "intake")
public class testIntake extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Intake intake = new Intake(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            intake.update();
            if(gamepad1.a){
                intake.setPower(1);
            }
            else if(gamepad1.b){
                intake.setPower(-1);
            }
            else intake.setPower(0);
            Details.packet = new TelemetryPacket();
        }
    }
}
