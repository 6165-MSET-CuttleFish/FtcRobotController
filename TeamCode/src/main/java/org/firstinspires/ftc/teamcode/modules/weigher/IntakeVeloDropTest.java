package org.firstinspires.ftc.teamcode.modules.weigher;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.util.field.Details;

public class IntakeVeloDropTest extends LinearOpMode {

    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Intake intake = new Intake(hardwareMap);
        waitForStart();

        if (isStopRequested()) return;

        while(opModeIsActive()){
            intake.update();
            Details.packet = new TelemetryPacket();
        }


        telemetry.update();

        while (!isStopRequested()) {
            idle();
        }
    }
}
