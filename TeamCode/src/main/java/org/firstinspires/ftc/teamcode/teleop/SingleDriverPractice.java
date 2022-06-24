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
import org.firstinspires.ftc.teamcode.util.field.OpModeType;

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
    ButtonReader levelIncrement, levelDecrement, dumpButton, liftButton, capRetract, capVerticalInc, capVerticalDec, capHorizontalInc, capHorizontalDec;
    ToggleButtonReader carouselButton, closeDeposit, farDeposit, crossDeposit, mediumDeposit;

    DriverPractice.Mode mode = DriverPractice.Mode.DRIVING;
    boolean toggleMode;
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
                crossDeposit = new ToggleButtonReader(primary, GamepadKeys.Button.X),
                mediumDeposit = new ToggleButtonReader(primary, GamepadKeys.Button.A),
                capRetract = new ButtonReader(primary, GamepadKeys.Button.X),
                capHorizontalDec = new ButtonReader(primary, GamepadKeys.Button.DPAD_LEFT),
                capHorizontalInc = new ButtonReader(primary, GamepadKeys.Button.DPAD_RIGHT),
                capVerticalDec = new ButtonReader(primary, GamepadKeys.Button.DPAD_DOWN),
                capVerticalInc = new ButtonReader(primary, GamepadKeys.Button.DPAD_UP),
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
            if (deposit.getState() == Deposit.State.OUT) drivePower = drivePower.times(0.5);
            if (ninjaMode.isDown()) drivePower = drivePower.times(0.60);
            if (capRetract.wasJustPressed()) {
                if (capstone.getState() == Capstone.State.ACTIVE) {
                    capstone.retract();
                } else {
                    capstone.idle();
                }
            }
            if (mode == DriverPractice.Mode.ENDGAME) {
                drivePower = new Pose2d(
                        (-gamepad2.left_stick_y),
                        0.0,
                        0.0
                ).div(8.0);
                setCapstone();
            } else {
                setDeposit();
                if (deposit.getState() == Deposit.State.IN || deposit.getState() == Deposit.State.CREATE_CLEARANCE || deposit.getState() == Deposit.State.HOLDING) setIntake();
            }
            robot.setWeightedDrivePower(drivePower);
            if (liftButton.wasJustPressed()) {
                deposit.toggleLift();
            }
            if (farDeposit.wasJustPressed()) {
                deposit.toggleFarDeposit();
                deposit.setLevel(Deposit.Level.LEVEL3);
                gamepad1.rumble(1.0, 1.0, 500);
            }
            if (closeDeposit.wasJustPressed()) {
                deposit.toggleCloseDeposit();
                deposit.setLevel(Deposit.Level.LEVEL3);
                gamepad1.rumble(1.0, 1.0, 500);
            }
            if (crossDeposit.wasJustPressed()) {
                deposit.toggleCrossDeposit();
                deposit.setLevel(Deposit.Level.SHARED);
                gamepad1.rumble(1.0, 1.0, 500);
            }
            if (mediumDeposit.wasJustPressed()) {
                deposit.toggleMediumDeposit();
                deposit.setLevel(Deposit.Level.LEVEL3);
                gamepad1.rumble(1.0, 1.0, 500);
            }
            if (gamepad1.touchpad) {
                if (!toggleMode) {
                    if (mode == DriverPractice.Mode.DRIVING) {
                        capstone.setTape(0.0);
                        capstone.active();
                        mode = DriverPractice.Mode.ENDGAME;
                    } else {
                        capstone.retract();
                        mode = DriverPractice.Mode.DRIVING;
                    }
                }
                toggleMode = true;
            } else {
                toggleMode = false;
            }
        }
    }

    boolean rightBumperCheck;
    boolean toggleCapSlow;

    private void setCapstone() {
        if (gamepad1.right_bumper && !rightBumperCheck) {
            toggleCapSlow = !toggleCapSlow;
            rightBumperCheck = true;
        }
        if (!gamepad1.right_bumper) rightBumperCheck = false;
        double mult = toggleCapSlow ? 0.5 : 1.0;
        capstone.setTape(gamepad1.right_trigger - gamepad1.left_trigger);
        capstone.setVerticalTurret(gamepad1.left_stick_y * mult);
        capstone.setHorizontalTurret(gamepad1.right_stick_x * mult);
        if (capHorizontalInc.wasJustPressed()) {
            capstone.incrementHorizontal(1.0);
        } else if (capHorizontalDec.wasJustPressed()) {
            capstone.incrementHorizontal(-1.0);
        }
        if (capVerticalInc.wasJustPressed()) {
            capstone.incrementVertical(1.0);
        } else if (capVerticalDec.wasJustPressed()) {
            capstone.incrementVertical(-1.0);
        }
    }

    void setIntake() {
        intake.setPower(gamepad1.right_trigger + gamepad1.left_trigger);
    }

    void setDeposit() {
        if (!deposit.platformIsOut()) {
            if (levelIncrement.wasJustPressed()) {
                switch (deposit.getDefaultLevel()) {
                    case LEVEL2:
                        deposit.setLevel(Deposit.Level.LEVEL3);
                        break;
                    case SHARED:
                        deposit.setLevel(Deposit.Level.LEVEL2);
                        break;
                }
            } else if (levelDecrement.wasJustPressed()) {
                switch (deposit.getDefaultLevel()) {
                    case LEVEL3:
                        deposit.setLevel(Deposit.Level.LEVEL2);
                        break;
                    case LEVEL2:
                        deposit.setLevel(Deposit.Level.SHARED);
                        break;
                }
            }
        } else {
            if (levelIncrement.wasJustPressed()) {
                deposit.incrementArmPosition(1.0);
            } else if (levelDecrement.wasJustPressed()) {
                deposit.incrementArmPosition(-1.0);
            }
            if (capHorizontalInc.wasJustPressed()) {
                deposit.incrementLinkagePosition(-1.0);
            } else if (capHorizontalDec.wasJustPressed()) {
                deposit.incrementLinkagePosition(1.0);
            }
        }
        if (dumpButton.wasJustPressed()) {
            deposit.softDump();
        } else if (gamepad1.right_trigger > 0.5) {
            deposit.dump();
        }
    }
}
