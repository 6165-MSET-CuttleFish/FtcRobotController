package org.firstinspires.ftc.teamcode.Tele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Shooter;

@TeleOp(name="getShooterVelocity", group = "Test")
public class getShooterVelocity extends LinearOpMode {
    Shooter shooter;
    public void runOpMode() throws InterruptedException{
        shooter = new Shooter(hardwareMap);
        double shooterVelo = 0;
        waitForStart();
        while(opModeIsActive()){
            shooterVelo = shooter.getVelocity();
            shooter.update();
            telemetry.addData("Velocity", shooterVelo);
            telemetry.update();
        }
    }
}
