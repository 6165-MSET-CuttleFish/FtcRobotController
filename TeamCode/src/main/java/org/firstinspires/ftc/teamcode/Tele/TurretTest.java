package org.firstinspires.ftc.teamcode.Tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Turret;

@Config
@TeleOp(name = "TurretTest", group = "LinearOpMode")
public class TurretTest extends LinearOpMode {
    Turret turret;
    FtcDashboard dashboard = FtcDashboard.getInstance();

    public void runOpMode() throws InterruptedException {
        turret = new Turret(hardwareMap);
        double angle = 45;
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        waitForStart();
        while (opModeIsActive()){
            turret.setTargetAngle(Math.toRadians(angle));
            turret.update();
            telemetry.addData("Current Angle", Math.toDegrees(turret.getAbsoluteAngle()));
            telemetry.addData("target angle", angle);
            telemetry.update();
        }
    }
}
