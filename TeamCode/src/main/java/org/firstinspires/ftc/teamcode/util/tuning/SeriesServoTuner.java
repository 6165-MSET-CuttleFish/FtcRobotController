package org.firstinspires.ftc.teamcode.util.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.modules.wrappers.actuators.ControllableServos;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.ArrayList;

@Config
@TeleOp
public class SeriesServoTuner extends LinearOpMode {
    public static String servoNames;
    public static String encoder;
    public static double position;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        while (opModeIsActive()) {
            try {

                String[] names = servoNames.split(",");
                Servo[] servos = new Servo[names.length];
                for (int i = 0; i < names.length; i++) {
                    servos[i] = hardwareMap.servo.get(names[i].trim());
                }
                ControllableServos controllableServos = new ControllableServos(servos);
                if (!encoder.isEmpty()) {
                    controllableServos.setEncoder(new Encoder(hardwareMap.get(DcMotorEx.class, encoder)));
                }
                controllableServos.setPosition(position);
            } catch(Exception exception) {
                telemetry.addData("Error", exception.getMessage());
            }
            telemetry.update();
        }
    }
}
