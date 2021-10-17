package org.firstinspires.ftc.teamcode.modules.weigher;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.MovingStatistics;

import org.firstinspires.ftc.robotcore.internal.system.Misc;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.Robot;

public class IntakeVeloDropTest extends LinearOpMode {

    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Robot robot = new Robot(this);
        waitForStart();

        if (isStopRequested()) return;
        int ticks = 0;
        int timeElapsed = (int) System.currentTimeMillis();
        while(opModeIsActive()){
            ticks = robot.intake.returnTicks();
            timeElapsed = (int) System.currentTimeMillis() - timeElapsed;
            int velo = ticks/timeElapsed;
            telemetry.addData("velocity", velo);
        }


        telemetry.update();

        while (!isStopRequested()) {
            idle();
        }
    }
}
