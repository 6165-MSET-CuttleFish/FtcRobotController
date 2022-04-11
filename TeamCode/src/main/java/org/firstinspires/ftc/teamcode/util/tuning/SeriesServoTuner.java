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
import org.firstinspires.ftc.teamcode.util.field.Context;

import java.util.ArrayList;

@Config
@TeleOp
public class SeriesServoTuner extends LinearOpMode {
    public static String servoNames;
    public static String encoder;
    public static double position;
    public static boolean calibrate;

    private String lastServoNames = servoNames;
    private String lastEncoder = encoder;

    private ControllableServos servo;
    FtcDashboard ftcDashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        while (opModeIsActive()) {
            if (!lastEncoder.equals(encoder) || !lastServoNames.equals(servoNames)) {
                lastEncoder = encoder;
                lastServoNames = servoNames;
                try {
                    String[] names = servoNames.split(",");
                    Servo[] servos = new Servo[names.length];
                    for (int i = 0; i < names.length; i++) {
                        servos[i] = hardwareMap.servo.get(names[i].trim());
                    }
                    servo = new ControllableServos(servos);
                    if (!encoder.isEmpty()) {
                        servo.setEncoder(new Encoder(hardwareMap.get(DcMotorEx.class, encoder)));
                    }
                } catch(Exception exception) {
                    telemetry.addData("Error", exception.getMessage());
                }
            }
            servo.setPosition(position);
            if (calibrate) {
                servo.init(servo.getPosition());
                calibrate = false;
            }
            Context.packet.put("Target Position", servo.getPosition());
            Context.packet.put("Estimated Position", servo.getEstimatedPosition());
            Context.packet.put("Real Position", servo.getRealPosition());
            telemetry.update();
        }
    }
}
