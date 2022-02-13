package org.firstinspires.ftc.teamcode.util.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class SeriesServoTuner extends LinearOpMode {
    public static String servoNames;
    public static double position;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        while (opModeIsActive()) {
            try {
                String[] names = servoNames.split(",");
                for (String name : names) {
                    Servo servo = hardwareMap.servo.get(name.trim());
                    servo.setPosition(position);
                }
            } catch(Exception exception) {
                telemetry.addData("Error", exception.getMessage());
            }
            telemetry.update();
        }
    }
}
