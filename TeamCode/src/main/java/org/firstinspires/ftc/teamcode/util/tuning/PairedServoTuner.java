package org.firstinspires.ftc.teamcode.util.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class PairedServoTuner extends LinearOpMode {
    public static String servo1;
    public static String servo2;
    public static double sum = 1;
    public static double position;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        while (opModeIsActive()) {
            try {
                Servo
                        s1 = hardwareMap.servo.get(servo1),
                        s2 = hardwareMap.servo.get(servo2);
                s1.setPosition(position);
                s2.setPosition(sum - position);
            } catch (Exception e) {
                telemetry.addData(servo1, "not valid");
                telemetry.addData(servo2, "not valid");
            }
            telemetry.update();
        }
    }
}
