package org.firstinspires.ftc.teamcode.Tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Turret;

import static org.firstinspires.ftc.teamcode.Components.Details.packet;


@TeleOp(name = "TurretTest", group = "Test")
public class TurretTest extends LinearOpMode {
    Turret turret;
    FtcDashboard dashboard;
    public void runOpMode() throws InterruptedException {
        turret = new Turret(hardwareMap);
        turret.setState(Turret.State.TUNING);
        dashboard = FtcDashboard.getInstance();
        waitForStart();
        while (opModeIsActive()){
            packet = new TelemetryPacket();
            turret.update();
            dashboard.sendTelemetryPacket(packet);
        }
    }
}
