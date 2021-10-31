package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.KeyReader;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.Robot;
import org.firstinspires.ftc.teamcode.modules.carousel.Carousel;
import org.firstinspires.ftc.teamcode.modules.deposit.Deposit;
import org.firstinspires.ftc.teamcode.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.util.OpModeType;

@TeleOp
public class DriverPractice extends LinearOpMode {
    Robot robot;
    Intake intake;
    Deposit deposit;
    Carousel carousel;

    GamepadEx primary;
    GamepadEx secondary;
    KeyReader[] keyReaders;
    TriggerReader intakeButton, outtakeButton;
    ButtonReader levelIncrementor, levelDecrementor;
    ToggleButtonReader depositButton, carouselButton;

    Deposit.State defaultDepositState = Deposit.State.LEVEL3;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this, OpModeType.TELE);
        intake = robot.intake;
        deposit = robot.deposit;
        carousel = robot.carousel;
        primary = new GamepadEx(gamepad1);
        secondary = new GamepadEx(gamepad2);
        keyReaders = new KeyReader[]{
                intakeButton = new TriggerReader(primary, GamepadKeys.Trigger.RIGHT_TRIGGER),
                outtakeButton = new TriggerReader(primary, GamepadKeys.Trigger.LEFT_TRIGGER),
                levelIncrementor = new ButtonReader(primary, GamepadKeys.Button.DPAD_UP),
                levelDecrementor = new ButtonReader(primary, GamepadKeys.Button.DPAD_DOWN),
                depositButton = new ToggleButtonReader(primary, GamepadKeys.Button.Y),
                carouselButton = new ToggleButtonReader(primary, GamepadKeys.Button.LEFT_BUMPER),
        };
        waitForStart();
        while (opModeIsActive()) {
            robot.update();
            for (KeyReader reader : keyReaders) {
                reader.readValue();
            }
            robot.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            0,
                            -gamepad1.right_stick_x
                    )
            );
            setIntake();
            setDeposit();
            setCarousel();
        }
    }

    void setIntake() {
        if (intakeButton.isDown()) {
            intake.setState(Intake.State.INTAKING);
        } else if (outtakeButton.isDown()) {
            intake.setState(Intake.State.EXTAKING);
        } else {
            intake.setState(Intake.State.IDLE);
        }
    }

    void setDeposit() {
        if (levelIncrementor.wasJustPressed()) {
            switch (defaultDepositState) {
                case LEVEL2:
                    defaultDepositState = Deposit.State.LEVEL3;
                    break;
                case LEVEL1:
                    defaultDepositState = Deposit.State.LEVEL2;
                    break;
            }
        } else if (levelDecrementor.wasJustPressed()) {
            switch (defaultDepositState) {
                case LEVEL3:
                    defaultDepositState = Deposit.State.LEVEL2;
                    break;
                case LEVEL2:
                    defaultDepositState = Deposit.State.LEVEL1;
                    break;
            }
        }

        if (depositButton.getState()) {
            deposit.setState(defaultDepositState);
        } else {
            deposit.setState(Deposit.State.IDLE);
        }
    }

    void setCarousel() {
        if(carouselButton.getState()) {
            carousel.setState(Carousel.State.ON);
        } else {
            carousel.setState(Carousel.State.IDLE);
        }
    }
}