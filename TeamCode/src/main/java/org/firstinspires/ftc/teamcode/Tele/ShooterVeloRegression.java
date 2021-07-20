package org.firstinspires.ftc.teamcode.Tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Shooter;

import static org.firstinspires.ftc.teamcode.Components.Details.packet;

@TeleOp(name="VeloRegression", group = "Test")
@Config
public class ShooterVeloRegression extends LinearOpMode {
    Shooter shooter;
    FtcDashboard dashboard;
    public static double velocity = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        shooter = new Shooter(hardwareMap);
        dashboard = FtcDashboard.getInstance();
        shooter.setState(Shooter.State.CUSTOMVELO);
        ButtonReader inc = new ButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.DPAD_UP);
        ButtonReader dec = new ButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.DPAD_UP);
        waitForStart();
        while (opModeIsActive()){
            packet = new TelemetryPacket();
            shooter.setVelocity(velocity);
            shooter.update();
            if (gamepad1.a) {
                shooter.gunner.shoot(3);
            }
            if (dec.wasJustPressed()) {
                velocity -= 100;
            } else if (inc.wasJustPressed()) {
                velocity += 100;
            }
            dashboard.sendTelemetryPacket(packet);
            dec.readValue();
            inc.readValue();
        }
    }
}
