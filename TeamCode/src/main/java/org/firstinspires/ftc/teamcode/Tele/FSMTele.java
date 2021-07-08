package org.firstinspires.ftc.teamcode.Tele;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Intake;
import org.firstinspires.ftc.teamcode.Components.Magazine;
import org.firstinspires.ftc.teamcode.Components.OpModeType;
import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.Components.Shooter;
import org.firstinspires.ftc.teamcode.Components.UniversalGamepad;
import org.firstinspires.ftc.teamcode.Components.WobbleArm;

@TeleOp(name = "RedTele", group = "Red")
public class FSMTele extends LinearOpMode {
    enum DriveState {
        NORMAL,
        WOBBLE
    }
    Robot robot;
    DriveState driveState = DriveState.NORMAL;
    WobbleArm wobbleArm;
    Shooter shooter;
    Magazine magazine;
    Intake intake;
    UniversalGamepad universalGamepad;
    double targetVelocity;
    @Override
    public void runOpMode() throws InterruptedException {
        universalGamepad = new UniversalGamepad(this);
        robot = new Robot(this, OpModeType.TELE);
        wobbleArm = robot.wobbleArm;
        shooter = robot.shooter;
        magazine = shooter.magazine;
        intake = robot.intake;
        waitForStart();

        // WHILE LOOP

        while (opModeIsActive()){
            robot.update();
            universalGamepad.update();
//            if(universalGamepad.g2.getButton(GamepadKeys.Button.LEFT_BUMPER)){
//                targetVelocity = 1100;
//                powershots = true;
//            } else {
//                targetVelocity = robot.shooter.getPoseVelo(robot.getPoseEstimate().vec());
//                powershots = false;
//            }
            if(universalGamepad.shieldButton.wasJustPressed()){
                switch (intake.getState()) {
                    case UP:
                        intake.setState(Intake.State.DOWN);
                        break;
                    case DOWN:
                        intake.setState(Intake.State.UP);
                        break;
                }
            }
            if(universalGamepad.g1.gamepad.left_bumper) {
                robot.wobbleArm.pickUp();
            }
            if(universalGamepad.g1.gamepad.right_bumper) {
                if(wobbleArm.getState() != WobbleArm.State.UP) robot.wobbleArm.claw.release();
            }
            robot.intake.setPower(universalGamepad.g1.gamepad.right_trigger - universalGamepad.g1.gamepad.left_trigger);
            switch (driveState) {
                case NORMAL: robot.setWeightedDrivePower(
                        new Pose2d(
                                universalGamepad.g1.getLeftY(),
                                -universalGamepad.g1.getLeftX(),
                                -universalGamepad.g1.getRightX() * 0.92
                            )
                        );
                        break;
                case WOBBLE: robot.setWeightedDrivePower(
                        new Pose2d(
                                universalGamepad.g1.getLeftY(),
                                universalGamepad.g1.getLeftX(),
                                -universalGamepad.g1.getRightX() * 0.92
                        )
                );
                break;
            }
            telemetry.addData("leftY" , universalGamepad.g1.getLeftY());
            telemetry.update();
            wobble();
        }

        //WHILE LOOP

    }
    public void wobble() {
        if (universalGamepad.wobbleButton.wasJustPressed()) {
            switch (robot.wobbleArm.getState()){
                case UP:
                    robot.wobbleArm.setState(WobbleArm.State.DOWN);
                    break;
                case MID:
                case DOWN:
                    robot.wobbleArm.setState(WobbleArm.State.UP);
                    break;
            }
        }
        if (universalGamepad.clawButton.wasJustPressed()) {
            switch(robot.wobbleArm.claw.getState()){
                case GRIP: {
                    if(wobbleArm.getState() != WobbleArm.State.UP) robot.wobbleArm.claw.release();
                    break;
                }
                case RELEASE:{
                    robot.wobbleArm.claw.grab();
                    break;
                }
            }
        }
    }
}
