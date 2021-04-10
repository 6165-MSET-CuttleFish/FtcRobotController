package org.firstinspires.ftc.teamcode.Tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Components.Robot;
@TeleOp(name="ShooterTest", group = "LinearOpMode")
@Config
public class ShooterTest extends LinearOpMode implements Runnable{
    Robot robot;
    public static double targetVelo = 1000;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    boolean magCheck;
    boolean isMagUp;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot (hardwareMap);
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
                sleep(150);
            } else if(gamepad1.dpad_down){
                targetVelo -= 10;
                sleep(150);
            }
            if(gamepad1.left_trigger >= 0.1) {
                robot.launcher.magUp();
                robot.launcher.setVelocity(targetVelo);
            } else {
                robot.launcher.setVelocity(0);
                if (gamepad1.left_bumper && !magCheck){
                    magCheck = true;
                    if(!isMagUp) {
                        robot.launcher.magUp();
                        isMagUp = true;
                    }
                    else {
                        robot.launcher.magDown();
                        isMagUp = false;
                    }
                }
                if (!gamepad1.left_bumper) magCheck = false;
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
