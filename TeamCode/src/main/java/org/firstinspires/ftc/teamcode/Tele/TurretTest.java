package org.firstinspires.ftc.teamcode.Tele;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Turret;

@Config
@TeleOp(name = "TurretTest", group = "LinearOpMode")
public class TurretTest extends LinearOpMode {
    Turret turret;

    private double angle = 1;
    public void runOpMode() throws InterruptedException {
        turret = new Turret(hardwareMap);
        waitForStart();
        while (opModeIsActive()){
            turret.setTargetAngle(angle+60);
            turret.update();

        }
    }
}
