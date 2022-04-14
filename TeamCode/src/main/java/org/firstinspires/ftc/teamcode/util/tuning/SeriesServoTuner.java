package org.firstinspires.ftc.teamcode.util.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.modules.wrappers.actuators.ControllableServos;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.field.Context;

@Config
@TeleOp
public class SeriesServoTuner extends LinearOpMode {
    public static String servoNames = "";
    public static String encoder = "";
    public static double position;
    public static double totalRotation = 360;
    public static double gearing = 1.0;
    public static boolean calibrate;

    public static double minPwmRange = 0;
    public static double maxPwmRange = 0;

    private String lastServoNames = servoNames;
    private String lastEncoder = encoder;

    private ControllableServos servo;
    FtcDashboard ftcDashboard = FtcDashboard.getInstance();
    // lock port 1
    // intake flip port 3
    // linkage deposit port 4
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        try {
            String[] names = servoNames.split(",");
            Servo[] servos = new Servo[names.length];
            for (int i = 0; i < names.length; i++) {
                servos[i] = hardwareMap.servo.get(names[i].trim());
            }
            servo = new ControllableServos(servos);
            servo.setServoRotation(Math.toRadians(totalRotation));
            servo.setGearing(gearing);
            if (!encoder.isEmpty()) {
                servo.setEncoder(new Encoder(hardwareMap.get(DcMotorEx.class, encoder)));
            }
        } catch(Exception exception) {
            telemetry.addData("Error", exception.getMessage());
        }
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
                    if (minPwmRange != 0 && maxPwmRange != 0) {
                        servo.increaseRange(new PwmControl.PwmRange(minPwmRange, maxPwmRange));
                    }
                    if (!encoder.isEmpty()) {
                        servo.setEncoder(new Encoder(hardwareMap.get(DcMotorEx.class, encoder)));
                    }
                } catch(Exception exception) {
                    telemetry.addData("Error", exception.getMessage());
                }
            }
            try {
                servo.setGearing(gearing);
                servo.setPosition(position);
                if (calibrate) {
                    servo.init(servo.getPosition());
                    servo.calibrateOffset(servo.getPosition(), 0);
                    calibrate = false;
                }
                Context.packet.put("Target Position", servo.getPosition());
                Context.packet.put("Estimated Position", servo.getEstimatedPosition());
                Context.packet.put("Real Position", servo.getRealPosition());

                Context.packet.put("Target Angle", Math.toDegrees(servo.getAngle()));
                Context.packet.put("Estimated Angle", Math.toDegrees(servo.getEstimatedAngle()));
                if (servo.getRealAngle() != null) Context.packet.put("Real Angle", Math.toDegrees(servo.getRealAngle()));
                ftcDashboard.sendTelemetryPacket(Context.packet);
                Context.packet = new TelemetryPacket();
                telemetry.update();
            } catch (Exception ignored) {

            }

        }
    }
}
