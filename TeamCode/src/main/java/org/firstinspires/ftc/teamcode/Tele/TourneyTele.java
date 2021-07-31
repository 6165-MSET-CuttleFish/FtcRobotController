package org.firstinspires.ftc.teamcode.Tele;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.KeyReader;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Gunner;
import org.firstinspires.ftc.teamcode.Components.Intake;
import org.firstinspires.ftc.teamcode.Components.Magazine;
import org.firstinspires.ftc.teamcode.Components.OpModeType;
import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.Components.Shooter;
import org.firstinspires.ftc.teamcode.Components.Side;
import org.firstinspires.ftc.teamcode.Components.Turret;
import org.firstinspires.ftc.teamcode.Components.WobbleArm;
import org.firstinspires.ftc.teamcode.Components.localizer.T265;

import static org.firstinspires.ftc.teamcode.Components.Details.robotPose;

@TeleOp(name = "MainTele", group = "Tele")
@Config
public class TourneyTele extends OpMode {
    enum DriveState {
        NORMAL,
        WOBBLE
    }
    Robot robot;
    DriveState driveState = DriveState.NORMAL;
    WobbleArm wobbleArm;
    Shooter shooter;
    Turret turret;
    Gunner gunner;
    Magazine magazine;
    Intake intake;
    GamepadEx g1, g2;
    ToggleButtonReader turretButton, shooterMode, reverseMode, defenseToggle;
    ButtonReader clawButton, wobbleButton, shieldButton, magButton, incrementOffset, decrementOffset;
    TriggerReader intakeButton, powerShots;
    KeyReader[] readers;

    private boolean defense = false;
    @Override
    public void init() {
        robot = new Robot(this, OpModeType.TELE);
        wobbleArm = robot.wobbleArm;
        shooter = robot.shooter;
        magazine = shooter.magazine;
        intake = robot.intake;
        turret = shooter.turret;
        gunner = shooter.gunner;
        initializeButtons();
        shooter.setState(Shooter.State.CONTINUOUS);
        telemetry.addData("Initialized", true);
        telemetry.update();
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        //robot.setPoseEstimate(new Pose2d(16.5275, -37.7225, Math.toRadians(180)));
        turret.setTarget(Robot.goal());
    }

    private void initializeButtons() {
        g1 = new GamepadEx(gamepad1);
        g2 = new GamepadEx(gamepad2);
        intakeButton = new TriggerReader(g2, GamepadKeys.Trigger.RIGHT_TRIGGER);
        reverseMode = new ToggleButtonReader(g1, GamepadKeys.Button.LEFT_BUMPER);
        turretButton = new ToggleButtonReader(g2, GamepadKeys.Button.DPAD_DOWN);
        shooterMode = new ToggleButtonReader(g2, GamepadKeys.Button.LEFT_BUMPER);
        powerShots = new TriggerReader(g2, GamepadKeys.Trigger.LEFT_TRIGGER);
        clawButton = new ToggleButtonReader(g2, GamepadKeys.Button.X);
        shieldButton = new ToggleButtonReader(g1, GamepadKeys.Button.DPAD_DOWN);
        wobbleButton = new ToggleButtonReader(g2, GamepadKeys.Button.B);
        magButton = new ToggleButtonReader(g2, GamepadKeys.Button.RIGHT_STICK_BUTTON);
        incrementOffset = new ButtonReader(g2, GamepadKeys.Button.DPAD_RIGHT);
        decrementOffset = new ButtonReader(g2, GamepadKeys.Button.DPAD_LEFT);
        defenseToggle = new ToggleButtonReader(g1, GamepadKeys.Button.RIGHT_BUMPER);
        readers = new KeyReader[]{turretButton, incrementOffset, decrementOffset, intakeButton, clawButton, shieldButton, wobbleButton, reverseMode, magButton, shooterMode, powerShots, /*defenseToggle*/};
    }

    @Override
    public void stop() {
        T265.stopCam();
        super.stop();
    }


    @Override
    public void loop() {
        intake.raiseIntake();
        if (incrementOffset.wasJustPressed()) {
            turret.offset -= 2;
        } else if (decrementOffset.wasJustPressed()) {
            turret.offset += 2;
        }
        if (shooterMode.getState()) {
            shooter.setState(Shooter.State.POWERSHOTS);
            turret.offset = 2;
            if (powerShots.isDown()) {
                shooter.powerShots();
            }
        } else {
            shooter.setState(Shooter.State.CONTINUOUS);
        }
        if (magButton.wasJustPressed() && gunner.getState() == Gunner.State.IDLE) {
            magazine.magMacro();
        }
        /*
        if(defenseToggle.wasJustPressed()){
            if(defense){
                defense = false;
                shooter.setState(Shooter.State.CONTINUOUS);
            }
            else{
                defense = true;
                shooter.setState(Shooter.State.MID);
            }
        }


         */
        robot.intake.setPower(gamepad2.right_stick_y);
        if (reverseMode.wasJustPressed()) {
            switch (driveState) {
                case NORMAL:
                    driveState = DriveState.WOBBLE;
                    break;
                case WOBBLE:
                    driveState = DriveState.NORMAL;
                    break;
            }
        }
            if(!reverseMode.getState()) {
                robot.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x,
                                -gamepad1.right_stick_x
                        )
                );
            } else {
                robot.setWeightedDrivePower(
                        new Pose2d(
                                gamepad1.left_stick_y,
                                gamepad1.left_stick_x,
                                -gamepad1.right_stick_x
                        )
                );
            }
        wobble();
        safety();
        if (turretButton.getState()) {
            turret.setState(Turret.State.IDLE);
        }
        robot.update();
        for (KeyReader reader : readers) {
            reader.readValue();
        }
    }

    public void safety() {
        if (robotPose.getX() > 2) {
            turret.setState(Turret.State.IDLE);
        } else {
            switch (WobbleArm.getState()) {
                case DOWN:
                case UP:
                    turret.setState(Turret.State.TARGET_LOCK);
                    break;
                case MACRO:
                case MID:
                    turret.setState(Turret.State.IDLE);
                    break;
            }
            if ((shooter.getState() == Shooter.State.CONTINUOUS && turret.getState() == Turret.State.TARGET_LOCK && turret.isIdle() && robotPose.getX() < 2 && Magazine.currentRings != 0 && magazine.getState() == Magazine.State.DOWN
                    && shooter.isWithinTolerance()) || intakeButton.wasJustPressed()) {
                gunner.shoot();
            }
            turret.setState(Turret.State.TARGET_LOCK);
        }
    }

    public void wobble() {
        if (wobbleButton.wasJustPressed()) {
            switch (WobbleArm.getState()) {
                case MID:
                case UP:
                    wobbleArm.setState(WobbleArm.State.MOVING_DOWN);
                    break;
                case DOWN:
                    wobbleArm.claw.grab();
                    wobbleArm.setState(WobbleArm.State.UP);
                    break;
            }
        }
        if (clawButton.wasJustPressed()) {
            switch (wobbleArm.claw.getState()) {
                case GRIP: {
                    if (WobbleArm.getState() != WobbleArm.State.UP) robot.wobbleArm.claw.release();
                    break;
                }
                case RELEASE: {
                    robot.wobbleArm.claw.grab();
                    break;
                }
            }
        }
        if (g2.gamepad.right_bumper) {
            robot.wobbleArm.pickUp();
        }
    }


}
