package org.firstinspires.ftc.teamcode.Tele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Turret;


@TeleOp(name = "TurretTest", group = "LinearOpMode")
public class TurretTest extends LinearOpMode {
    Turret turret;

    public void runOpMode() throws InterruptedException {
        turret = new Turret(hardwareMap);
        turret.setState(Turret.State.TUNING);
        waitForStart();
        while (opModeIsActive()){
            turret.update();
        }
    }
}
