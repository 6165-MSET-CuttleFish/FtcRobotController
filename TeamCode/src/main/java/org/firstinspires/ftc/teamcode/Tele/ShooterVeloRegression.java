package org.firstinspires.ftc.teamcode.Tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.Components.Shooter;

import static org.firstinspires.ftc.teamcode.Components.Details.packet;

@TeleOp(name="VeloRegression", group = "Test")
public class ShooterVeloRegression extends LinearOpMode {
    Shooter shooter;
    FtcDashboard dashboard;
    @Override
    public void runOpMode() throws InterruptedException {
        shooter = new Shooter(hardwareMap);
        dashboard = FtcDashboard.getInstance();
        shooter.setState(Shooter.State.CUSTOMVELO);
        waitForStart();
        while(opModeIsActive()){
            packet = new TelemetryPacket();
            shooter.update();
            shooter.setVelocity(5000);
            if(gamepad1.a) {
                shooter.gunner.shoot(3);
            }
            dashboard.sendTelemetryPacket(packet);
        }
    }
}
