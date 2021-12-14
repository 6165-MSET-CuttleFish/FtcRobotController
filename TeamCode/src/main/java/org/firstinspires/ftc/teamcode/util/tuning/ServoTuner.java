package org.firstinspires.ftc.teamcode.util.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class ServoTuner extends LinearOpMode {
    public static double position = 0;
    public static String name = "";
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        while (opModeIsActive()) {
            try {
                Servo servo = hardwareMap.get(Servo.class, name);
                if (position < 0) {
                    // servo.disable();
                } else {
                    servo.setPosition(position);
                }
            } catch (Exception e) {
                telemetry.addData(name, "not valid");
                telemetry.update();
            }
        }
    }
}
