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
    TriggerReader intakeButton, ninjaMode;
    ButtonReader levelIncrement, levelDecrement, dumpButton, liftButton, softDump;
    ToggleButtonReader carouselButton, closeDeposit, farDeposit, crossDeposit;

    Deposit.Level defaultDepositState = Deposit.Level.LEVEL3;
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
                liftButton = new ButtonReader(primary, GamepadKeys.Button.LEFT_BUMPER),
                carouselButton = new ToggleButtonReader(primary, GamepadKeys.Button.LEFT_BUMPER),
                dumpButton = new ButtonReader(primary, GamepadKeys.Button.RIGHT_BUMPER),
                closeDeposit = new ToggleButtonReader(primary, GamepadKeys.Button.B),
                farDeposit = new ToggleButtonReader(primary, GamepadKeys.Button.Y),
                crossDeposit = new ToggleButtonReader(primary, GamepadKeys.Button.X)
        };
        waitForStart();
        deposit.resetEncoder();
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
            if (deposit.getState() == Deposit.State.IN || deposit.getState() == Deposit.State.CREATE_CLEARANCE) setIntake();
            setDeposit();
            if (liftButton.wasJustPressed()) {
                deposit.toggleLift();
            }
            if (farDeposit.wasJustPressed()) {
                deposit.toggleFarDeposit();
                gamepad1.rumble(1.0, 1.0, 500);
            }
            if (closeDeposit.wasJustPressed()) {
                deposit.toggleCloseDeposit();
                gamepad1.rumble(1.0, 1.0, 500);
            }
            if (crossDeposit.wasJustPressed()) {
                deposit.toggleCrossDeposit();
                gamepad1.rumble(1.0, 1.0, 500);
            }
        }
    }
    void setIntake() {
        intake.setPower(gamepad1.right_trigger + gamepad1.left_trigger);
    }

    void setDeposit() {
        if (!deposit.platformIsOut()) {
            if (levelIncrement.wasJustPressed()) {
                switch (defaultDepositState) {
                    case LEVEL2:
                        defaultDepositState = Deposit.Level.LEVEL3;
                        break;
                    case SHARED:
                        defaultDepositState = Deposit.Level.LEVEL2;
                        break;
                }
                deposit.setLevel(defaultDepositState);
            } else if (levelDecrement.wasJustPressed()) {
                switch (defaultDepositState) {
                    case LEVEL3:
                        defaultDepositState = Deposit.Level.LEVEL2;
                        break;
                    case LEVEL2:
                        defaultDepositState = Deposit.Level.SHARED;
                        break;
                }
                deposit.setLevel(defaultDepositState);
            }
        } else {
            if (levelIncrement.wasJustPressed()) {
                deposit.incrementArmPosition(1.0);
            } else if (levelDecrement.wasJustPressed()) {
                deposit.incrementArmPosition(-1.0);
            }
        }
        if (dumpButton.wasJustPressed()) {
            deposit.dump();
        } else if (gamepad1.right_trigger > 0.5) {
            deposit.softDump();
        }
    }
}
