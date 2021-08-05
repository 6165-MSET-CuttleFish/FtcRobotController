package org.firstinspires.ftc.teamcode.Tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Details;
import org.firstinspires.ftc.teamcode.Components.Shooter;

import static org.firstinspires.ftc.teamcode.Components.Details.packet;

@TeleOp(name="ShooterTest", group = "Test")
public class ShooterTest extends LinearOpMode {
    Shooter shooter;
    FtcDashboard dashboard;
    @Override
    public void runOpMode() throws InterruptedException {
        shooter = new Shooter(hardwareMap);
        shooter.setState(Shooter.State.TUNING);
        dashboard = FtcDashboard.getInstance();
        waitForStart();
        while(opModeIsActive()){
            packet = new TelemetryPacket();
            shooter.update();
            dashboard.sendTelemetryPacket(packet);
        }
    }
}
