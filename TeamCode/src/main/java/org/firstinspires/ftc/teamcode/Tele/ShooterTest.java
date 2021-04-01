package org.firstinspires.ftc.teamcode.Tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Components.Robot;
@TeleOp(name="ShooterTest", group = "LinearOpMode")
public class ShooterTest extends LinearOpMode implements Runnable{
    Robot robot;
    double targetVelo = 1000;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Robot (hardwareMap);

       // robot = new Robot (hardwareMap, telemetry, () -> opModeIsActive() && gamepadIdle());

        robot.init();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        //wingDefault = ()->robot.launcher.wingsVert();
        // pidRotate = new PIDController(.07, 0.014, 0.0044);
        Thread shooterThread = new Thread(()->{
            while(opModeIsActive() && !isStopRequested()){
                robot.launcher.updatePID();
            }
        });
        Thread otherThread = new Thread(this);
        waitForStart();
        otherThread.start();
        shooterThread.start();
        while(opModeIsActive()){
            if(gamepad1.dpad_up){
                targetVelo += 20;
                sleep(300);
            } else if(gamepad1.dpad_down){
                targetVelo -= 10;
                sleep(300);
            }
            if(gamepad1.left_trigger >= 0.1) {
                robot.launcher.tiltUp();
                robot.launcher.setVelocity(targetVelo);
            } else {
                robot.launcher.setVelocity(0);
                if (gamepad1.left_bumper) robot.launcher.tiltUp();
                else robot.launcher.tiltDown();
            }
            if(gamepad1.right_trigger >= 0.1){
                robot.launcher.magazineShoot();
            }
        }
    }

    @Override
    public void run() {
        while(opModeIsActive() && !isStopRequested()){
            telemetry.addData("targetVelo ", targetVelo);
            telemetry.addData("current velo ", robot.launcher.getVelocity());
            telemetry.update();
        }
    }
}
