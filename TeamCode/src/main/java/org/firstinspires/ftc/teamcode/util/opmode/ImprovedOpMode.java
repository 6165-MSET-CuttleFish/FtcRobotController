package org.firstinspires.ftc.teamcode.util.opmode;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.KeyReader;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.modules.carousel.Carousel;
import org.firstinspires.ftc.teamcode.modules.deposit.Deposit;
import org.firstinspires.ftc.teamcode.modules.deposit.Platform;
import org.firstinspires.ftc.teamcode.modules.intake.Intake;

public abstract class ImprovedOpMode extends LinearOpMode {
    public Robot robot;
    public Deposit deposit;
    public Platform platform;
    public Carousel carousel;
    public Intake intake;

    public KeyReader[] keyReaders;
    public GamepadEx primary;
    public GamepadEx secondary;
    public TriggerReader intakeButton, ninjaMode;
    public ButtonReader levelIncrementor, levelDecrementor, dumpButton, outtakeButton;
    public ToggleButtonReader depositButton, carouselButton;

    public void initialize() {
        robot = new Robot(this);
        deposit = robot.deposit;
        platform = deposit.platform;
        carousel = robot.carousel;
        intake = robot.intake;
    }

    public void buttonMap() {
        primary = new GamepadEx(gamepad1);
        secondary = new GamepadEx(gamepad2);
        keyReaders = new KeyReader[]{
                intakeButton = new TriggerReader(secondary, GamepadKeys.Trigger.RIGHT_TRIGGER),
                outtakeButton = new ButtonReader(primary, GamepadKeys.Button.X),
                ninjaMode = new TriggerReader(primary, GamepadKeys.Trigger.LEFT_TRIGGER),
                levelIncrementor = new ButtonReader(primary, GamepadKeys.Button.DPAD_UP),
                levelDecrementor = new ButtonReader(primary, GamepadKeys.Button.DPAD_DOWN),
                depositButton = new ToggleButtonReader(primary, GamepadKeys.Button.Y),
                carouselButton = new ToggleButtonReader(primary, GamepadKeys.Button.LEFT_BUMPER),
                dumpButton = new ButtonReader(primary, GamepadKeys.Button.RIGHT_BUMPER),
        };
    }

    public void update() {
        robot.update();
        for (KeyReader keyReader : keyReaders) {
            keyReader.readValue();
        }
    }

}
