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
import org.firstinspires.ftc.teamcode.modules.capstone.Capstone;
import org.firstinspires.ftc.teamcode.modules.carousel.Carousel;
import org.firstinspires.ftc.teamcode.modules.deposit.Deposit;
import org.firstinspires.ftc.teamcode.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.util.field.Balance;
import org.firstinspires.ftc.teamcode.util.field.OpModeType;

import static org.firstinspires.ftc.teamcode.util.field.Context.balance;

@TeleOp
public class SingleDriverPractice extends LinearOpMode {
    Robot robot;
    Intake intake;
    Deposit deposit;
    Carousel carousel;
    Capstone capstone;
    GamepadEx primary;
    GamepadEx secondary;
    KeyReader[] keyReaders;
    TriggerReader intakeButton, ninjaMode, liftButton;
    ButtonReader levelIncrement, levelDecrement, dumpButton, tippedToward, tippedAway, capHorizontalInc, capVerticalInc, capHorizontalDec, capVerticalDec;
    ToggleButtonReader carouselButton;

    Deposit.State defaultDepositState = Deposit.State.LEVEL3;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this, OpModeType.TELE);
        intake = robot.intake;
        deposit = robot.deposit;
        carousel = robot.carousel;
        capstone = robot.capstone;
        primary = new GamepadEx(gamepad1);
        secondary = new GamepadEx(gamepad2);
        keyReaders = new KeyReader[] {
                intakeButton = new TriggerReader(primary, GamepadKeys.Trigger.RIGHT_TRIGGER),
                ninjaMode = new TriggerReader(primary, GamepadKeys.Trigger.LEFT_TRIGGER),
                levelIncrement = new ButtonReader(primary, GamepadKeys.Button.DPAD_UP),
                levelDecrement = new ButtonReader(primary, GamepadKeys.Button.DPAD_DOWN),
                liftButton = new TriggerReader(secondary, GamepadKeys.Trigger.LEFT_TRIGGER),
                tippedAway = new ButtonReader(secondary, GamepadKeys.Button.LEFT_BUMPER),
                tippedToward = new ButtonReader(secondary, GamepadKeys.Button.RIGHT_BUMPER),
                carouselButton = new ToggleButtonReader(primary, GamepadKeys.Button.LEFT_BUMPER),
                dumpButton = new ButtonReader(primary, GamepadKeys.Button.A),
        };
        Deposit.farDeposit = true;
        waitForStart();
        while (opModeIsActive()) {
            robot.update();
            for (KeyReader reader : keyReaders) {
                reader.readValue();
            }
            Pose2d drivePower = new Pose2d(
                    -gamepad1.left_stick_y,
                    0,
                    -gamepad1.right_stick_x
            );
            if (intake.getContainsBlock() && intake.getState() == Intake.State.OUT) {
                gamepad1.rumble(500);
                gamepad2.rumble(500);
            }
            if (ninjaMode.isDown()) drivePower = drivePower.times(0.60);
            robot.setWeightedDrivePower(drivePower);
            setIntake();
            setDeposit();
            if (tippedAway.isDown() && tippedToward.isDown()) {
                balance = Balance.BALANCED;
            } else if (tippedAway.isDown()) {
                balance = Balance.AWAY;
            } else if (tippedToward.isDown()) {
                balance = Balance.TOWARD;
            }
        }
    }
    void setIntake() {
        if (intakeButton.isDown()) {
            intake.setPower(1);
        } else {
            intake.setPower(0);
        }
    }

    void setDeposit() {
        if (levelIncrement.wasJustPressed()) {
            switch (defaultDepositState) {
                case LEVEL2:
                    defaultDepositState = Deposit.State.LEVEL3;
                    break;
                case IDLE:
                    defaultDepositState = Deposit.State.LEVEL2;
                    break;
            }
            deposit.setState(defaultDepositState);
        } else if (levelDecrement.wasJustPressed()) {
            switch (defaultDepositState) {
                case LEVEL3:
                    defaultDepositState = Deposit.State.LEVEL2;
                    break;
                case LEVEL2:
                    defaultDepositState = Deposit.State.LEVEL1;
                    break;
            }
            deposit.setState(defaultDepositState);
        }

        if (dumpButton.wasJustPressed()) {
            deposit.dump();
        }
    }
}
