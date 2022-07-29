package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.KeyReader;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.field.OpModeType;

@TeleOp
@Disabled
public class SampleTeleOp extends LinearOpMode {
    Robot robot;
    KeyReader[] keyReaders;
    GamepadEx primary, secondary;
    ButtonReader someButton;
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this, OpModeType.TELE);
        primary = new GamepadEx(gamepad1);
        secondary = new GamepadEx(gamepad2);
        keyReaders = new KeyReader[] {
                someButton = new ButtonReader(primary, GamepadKeys.Button.A)
        };
        while (opModeIsActive()) {
            robot.update();
            for (KeyReader keyReader : keyReaders) {
                keyReader.readValue();
            }
        }
    }
}
