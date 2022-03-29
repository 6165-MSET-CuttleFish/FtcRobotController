package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.util.field.Context;

import java.util.Objects;

@TeleOp
@Config
public class ForceCalculator extends LinearOpMode {
    Robot robot;
    public static double power = 0.5;
    public static double mass = 40;
    // F = ma
    ToggleButtonReader modeToggle;
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this);
        GamepadEx gamepadEx = new GamepadEx(gamepad1);
        modeToggle = new ToggleButtonReader(gamepadEx, GamepadKeys.Button.A);
        waitForStart();
        while (opModeIsActive()) {
            robot.update();
            modeToggle.readValue();
            if (modeToggle.getState()) {
                robot.setWeightedDrivePower(new Pose2d(-power));
            }
            else {
                robot.setWeightedDrivePower(new Pose2d(
                        -gamepad1.left_stick_y,
                        0.0,
                        -gamepad1.right_stick_x
                ));
            }
            double accel = Objects.requireNonNull(robot.getPoseAcceleration()).getX();
            Context.packet.put("Linear Acceleration", accel);
            Context.packet.put("Angular Acceleration", Math.toDegrees(Objects.requireNonNull(robot.getPoseAcceleration()).getHeading()));
            Context.packet.put("Force Estimate", accel * mass);
        }
    }
}
