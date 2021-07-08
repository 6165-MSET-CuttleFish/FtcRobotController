package org.firstinspires.ftc.teamcode.Tele;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Intake;
import org.firstinspires.ftc.teamcode.Components.Magazine;
import org.firstinspires.ftc.teamcode.Components.OpModeType;
import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.Components.Shooter;
import org.firstinspires.ftc.teamcode.Components.WobbleArm;

@TeleOp(name = "RedTele", group = "Red")
public class FSMTele extends LinearOpMode {
    enum DriveState {
        normal,
        wobble
    }
    Robot robot;
    DriveState driveState = DriveState.normal;
    WobbleArm wobbleArm;
    Shooter shooter;
    Magazine magazine;
    Intake intake;
    GamepadEx allen;
    GamepadEx riya;
    double lxMult = 1;
    double lyMult = 1;
    double rxMult = 1;
    double targetVelocity;
    boolean wobbleCheck, powershots, wasPressed;
    ToggleButtonReader clawButton, wobbleButton, reverseMode, turret, shieldButton;
    TriggerReader magButton;
    TriggerReader powerShotsTrigger;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this, OpModeType.TELE);
        wobbleArm = robot.wobbleArm;
        shooter = robot.shooter;
        magazine = shooter.magazine;
        intake = robot.intake;
        initialize();
        waitForStart();

        // WHILE LOOP

        while (opModeIsActive()){
            robot.update();
//            if(riya.getButton(GamepadKeys.Button.LEFT_BUMPER)){
//                targetVelocity = 1100;
//                powershots = true;
//            } else {
//                targetVelocity = robot.shooter.getPoseVelo(robot.getPoseEstimate().vec());
//                powershots = false;
//            }
            if(shieldButton.wasJustPressed()){
                switch (intake.getState()) {
                    case UP:
                        intake.setState(Intake.State.DOWN);
                        break;
                    case DOWN:
                        intake.setState(Intake.State.UP);
                        break;
                }
            }
            if(allen.gamepad.x) {
                robot.wobbleArm.down();
            }
            if(allen.gamepad.y) {
                robot.wobbleArm.up();
            }
            if(allen.gamepad.left_bumper) {
                robot.wobbleArm.pickUp();
            }
            if(allen.gamepad.right_bumper) {
                if(wobbleArm.getState() != WobbleArm.State.UP) robot.wobbleArm.claw.release();
            }
            robot.intake.setPower(allen.gamepad.right_trigger - allen.gamepad.left_trigger);
            switch (driveState) {
                case normal: robot.setWeightedDrivePower(
                        new Pose2d(
                                allen.getLeftY(),
                                -allen.getLeftX(),
                                -allen.getRightX() * 0.92
                            )
                        );
                        break;
                case wobble: robot.setWeightedDrivePower(
                        new Pose2d(
                                allen.getLeftY(),
                                allen.getLeftX(),
                                -allen.getRightX() * 0.92
                        )
                );
                break;
            }
            telemetry.addData("leftY" , allen.getLeftY());
            telemetry.update();
            wobble();
            update();
        }

        //WHILE LOOP

    }
    public void wobble() {
        if (riya.getButton(GamepadKeys.Button.B) && !wobbleCheck) {
            wobbleCheck = true;
            switch (robot.wobbleArm.getState()){
                case UP:
                    robot.wobbleArm.down();
                    break;
                case MID: break;
                case DOWN: break;
            }
        }  else if(!riya.getButton(GamepadKeys.Button.B)){
            wobbleCheck = false;
        }
        if (clawButton.wasJustPressed()) {
            switch(robot.wobbleArm.claw.getState()){
                case GRIP: {
                    robot.wobbleArm.claw.release();
                    break;
                }
                case RELEASE:{
                    robot.wobbleArm.claw.grab();
                    break;
                }
            }
        }
    }
    public void shooter() {
        if (riya.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) >= 0.1) {
            robot.shooter.magMacro();
            robot.shooter.setVelocity(targetVelocity);
            if (Math.abs(robot.shooter.getError()) <= 30 && !robot.isBusy() && !wasPressed) {
                wasPressed = true;
                sleep(180);
                robot.optimalShoot();
            }
        } else if (riya.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
            robot.shooter.magMacro();
        } else {
            if (robot.shooter.getRings() > 0) robot.shooter.setVelocity(targetVelocity);
        }
        if (riya.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
            robot.shooter.singleRound();
        }
    }
    private void initialize(){
        allen = new GamepadEx(gamepad1);
        riya = new GamepadEx(gamepad2);
        clawButton = new ToggleButtonReader(riya, GamepadKeys.Button.A);
        reverseMode = new ToggleButtonReader(allen, GamepadKeys.Button.RIGHT_BUMPER);
        magButton = new TriggerReader(allen, GamepadKeys.Trigger.LEFT_TRIGGER);
        shieldButton = new ToggleButtonReader(allen, GamepadKeys.Button.DPAD_DOWN);
    }
    private void update(){
        clawButton.readValue();
        shieldButton.readValue();
    }
}
