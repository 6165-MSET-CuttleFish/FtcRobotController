package org.firstinspires.ftc.teamcode.Tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.localizer.IntelLocalizer;

@TeleOp
public class t265Test extends LinearOpMode {
    Localizer localizer;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    @Override
    public void runOpMode() throws InterruptedException {
        localizer = new IntelLocalizer(hardwareMap, new Pose2d(0, 0));
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        waitForStart();
        while(opModeIsActive()){
            localizer.update();
            telemetry.addData("x", localizer.getPoseEstimate().getX());
            telemetry.addData("x", localizer.getPoseEstimate().getY());
            telemetry.update();
        }
    }
}
