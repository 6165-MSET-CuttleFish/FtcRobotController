package org.firstinspires.ftc.teamcode.Tele;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Gunner;
import org.firstinspires.ftc.teamcode.Components.Magazine;
import org.firstinspires.ftc.teamcode.Components.OpModeType;
import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.Components.Shooter;
import org.firstinspires.ftc.teamcode.Components.Turret;
import org.firstinspires.ftc.teamcode.Components.localizer.T265;

@TeleOp(name="VeloRegression", group = "Test")
@Config
public class ShooterVeloRegression extends OpMode {
    Robot robot;
    Shooter shooter;
    Turret turret;
    Gunner gunner;
    Magazine magazine;
    ToggleButtonReader inc;
    ToggleButtonReader dec;
    public static double velocity = 0;
    @Override
    public void init() {
        robot = new Robot(this, OpModeType.TELE);
        shooter = robot.shooter;
        turret = shooter.turret;
        gunner = shooter.gunner;
        magazine = shooter.magazine;
        shooter.setState(Shooter.State.CUSTOMVELO);
        turret.setTarget(Robot.goal);
        turret.setState(Turret.State.TARGET_LOCK);
        inc = new ToggleButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.DPAD_UP);
        dec = new ToggleButtonReader(new GamepadEx(gamepad1), GamepadKeys.Button.DPAD_UP);
    }

    @Override
    public void loop() {
        robot.update();
        dec.readValue();
        inc.readValue();
        shooter.setVelocity(velocity);
        robot.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x
                )
        );
        if (gamepad1.a) {
            gunner.shoot(3);
        } else if (dec.wasJustPressed()) {
            velocity -= 100;
        } else if (inc.wasJustPressed()) {
            velocity += 100;
        } else if (gamepad1.right_stick_button) {
            magazine.magMacro();
        }
        telemetry.addData("Shooter Distance", shooter.getShooterVec().distTo(Robot.goal));
    }

    @Override
    public void stop() {
        T265.stopCam();
        super.stop();
    }
}
