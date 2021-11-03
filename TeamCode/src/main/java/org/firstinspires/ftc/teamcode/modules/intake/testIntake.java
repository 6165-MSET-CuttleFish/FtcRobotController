package org.firstinspires.ftc.teamcode.modules.intake;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Details;

@TeleOp(group = "intake")
public class testIntake extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Intake intake = new Intake(hardwareMap);
        intake.init();
        waitForStart();
        while (opModeIsActive()) {
            if(gamepad1.a == true){
                intake.setState(Intake.State.INTAKING);
            }
            else if(gamepad1.b == true){
                intake.setState(Intake.State.EXTAKING);
            }
            else intake.setState(Intake.State.IDLE);
            Details.packet = new TelemetryPacket();
            intake.update();
        }
    }
}
