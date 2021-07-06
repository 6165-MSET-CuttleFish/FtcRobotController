package org.firstinspires.ftc.teamcode.Tele;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.OpModeType;
import org.firstinspires.ftc.teamcode.Components.Robot;

@TeleOp
public class FSMTele extends LinearOpMode {
    Robot robot;
    GamepadEx allen;
    GamepadEx riya;
    double lxMult = 1;
    double lyMult = 1;
    double rxMult = 1;
    double targetVelocity;
    boolean wobbleCheck, powershots, wasPressed;
    ToggleButtonReader clawButton, wobbleButton, reverseMode;
    TriggerReader highGoalTrigger, powerShotsTrigger;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this, OpModeType.tele);
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
            if(allen.gamepad.a){
                robot.shieldDown();
            } else if (allen.gamepad.b){
                robot.shieldUp();
            }
            if(allen.gamepad.x) {
                robot.wobbleArm.down();
            }
            if(allen.gamepad.y) {
                robot.wobbleArm.up();
            }
            if(allen.gamepad.left_bumper) {
                robot.wobbleArm.claw.grab();
                robot.wobbleArm.mid();
            }
            if(allen.gamepad.right_bumper) {
                robot.wobbleArm.claw.release();
            }
            robot.intake.setPower(allen.gamepad.right_trigger - allen.gamepad.left_trigger);
            robot.setWeightedDrivePower(
                    new Pose2d(
                            -allen.getLeftY() * lyMult,
                            -allen.getLeftX() * lxMult,
                            allen.getRightX() * 0.92 * rxMult
                    )
            );
            setMultiplier();
            wobble();
        }

        //WHILE LOOP

    }
    private void setMultiplier() {
        if (allen.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) >= 0.3) {
            lxMult = 0.5;
            rxMult = 0.5;
            lyMult = 0.5;
        } else {
            lxMult = 1;
            rxMult = 1;
            lyMult = 1;
        }
        if (reverseMode.wasJustPressed()) {
            lxMult = -lxMult;
            lyMult = -lyMult;
        }
    }
    public void wobble() {
        if (riya.getButton(GamepadKeys.Button.B) && !wobbleCheck) {
            wobbleCheck = true;
            robot.wobbleArmUp();
            switch (robot.wobbleArm.getState()){
                case IN: break;
                case MID: break;
                case OUT: break;
            }
        }  else if(!riya.getButton(GamepadKeys.Button.B)){
            wobbleCheck = false;
        } if (riya.getButton(GamepadKeys.Button.A)) {
//            robot.wobbleArmVertical();
//            sleep(300);
//            robot.release();
//            sleep(200);
//            robot.wobbleArmUp();
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
        highGoalTrigger = new TriggerReader(riya, GamepadKeys.Trigger.LEFT_TRIGGER);
    }
    private void update(){
        clawButton.readValue();
        reverseMode.readValue();
        highGoalTrigger.readValue();
    }
}
