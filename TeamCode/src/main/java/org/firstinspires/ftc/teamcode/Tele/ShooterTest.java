package org.firstinspires.ftc.teamcode.Tele;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Shooter;

@TeleOp(name="VeloRegression", group = "Test")
@Config
@Disabled
public class ShooterTest extends LinearOpMode {
    Shooter shooter;
    @Override
    public void runOpMode() throws InterruptedException {
        shooter = new Shooter(hardwareMap);
        shooter.setState(Shooter.State.TUNING);
        waitForStart();
        while(opModeIsActive()){
            shooter.update();
        }
    }
}
