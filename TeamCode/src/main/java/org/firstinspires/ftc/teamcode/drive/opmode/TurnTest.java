package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.drive.StaticConfig;
import org.firstinspires.ftc.teamcode.roadrunnerext.drive.ImprovedDrive;

/*
 * This is a simple routine to test turning capabilities.
 */
@Config
@Autonomous(group = "drive")
@Disabled
public class TurnTest extends LinearOpMode {
    public static double ANGLE = 90; // deg

    @Override
    public void runOpMode() throws InterruptedException {
        ImprovedDrive drive = StaticConfig.getDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;
        while (opModeIsActive()) {
            drive.turn(Math.toRadians(ANGLE));
        }
    }
}
