package org.firstinspires.ftc.teamcode.Tele;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Components.Gunner;
import org.firstinspires.ftc.teamcode.Components.Intake;
import org.firstinspires.ftc.teamcode.Components.Magazine;
import org.firstinspires.ftc.teamcode.Components.OpModeType;
import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.Components.Shooter;
import org.firstinspires.ftc.teamcode.Components.Turret;
import org.firstinspires.ftc.teamcode.Components.UniversalGamepad;
import org.firstinspires.ftc.teamcode.Components.WobbleArm;

import static org.firstinspires.ftc.teamcode.Components.Details.robotPose;

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
    Turret turret;
    Gunner gunner;
    Magazine magazine;
    Intake intake;
    UniversalGamepad universalGamepad;

    @Override
    public void runOpMode() throws InterruptedException {
        universalGamepad = new UniversalGamepad(this);
        robot = new Robot(this, OpModeType.TELE);
        wobbleArm = robot.wobbleArm;
        shooter = robot.shooter;
        magazine = shooter.magazine;
        intake = robot.intake;
        turret = shooter.turret;
        gunner = shooter.gunner;
        telemetry.addData("Initialized", true);
        telemetry.update();
        shooter.setState(Shooter.State.CUSTOMVELO);

        waitForStart();
        robot.setPoseEstimate(new Pose2d());

        // WHILE LOOP

        while (opModeIsActive()) {
            turret.setTarget(Robot.goal);
            universalGamepad.update();
            if (universalGamepad.shieldButton.wasJustPressed()) {
                switch (intake.getState()) {
                    case UP:
                        intake.setState(Intake.State.DOWN);
                        break;
                    case DOWN:
                        intake.setState(Intake.State.UP);
                        break;
                }
            }
            if (universalGamepad.g1.getButton(GamepadKeys.Button.A)) {
                robot.setPoseEstimate(new Pose2d());
            }
            if (universalGamepad.magButton.wasJustPressed()) {
                magazine.magMacro();
            }
            robot.intake.setPower(-universalGamepad.g2.getRightY());
            switch (driveState) {
                case NORMAL:
                    robot.setWeightedDrivePower(
                            new Pose2d(
                                    universalGamepad.g1.getLeftY(),
                                    -universalGamepad.g1.getLeftX(),
                                    -universalGamepad.g1.getRightX() * 0.92
                            )
                    );
                    break;
                case WOBBLE:
                    robot.setWeightedDrivePower(
                            new Pose2d(
                                    -universalGamepad.g1.getLeftY(),
                                    universalGamepad.g1.getLeftX(),
                                    -universalGamepad.g1.getRightX() * 0.92
                            )
                    );
                    break;
            }
            wobble();
            safety();
            robot.update();
            universalGamepad.update();
        }

        //WHILE LOOP

    }

    public void safety() {
        if (robotPose.getX() > 20) {
            turret.setState(Turret.State.IDLE);
        } else {
            switch (wobbleArm.getState()) {
                case DOWN:
                case UP:
                    turret.setState(Turret.State.TARGET_LOCK);
                    break;
                case MID:
                    turret.setState(Turret.State.IDLE);
                    break;
            } if (turret.getState() == Turret.State.TARGET_LOCK && turret.getError() < 3 && shooter.getPercentError() < 0.1 && robotPose.getX() < -10) {
                gunner.shoot();
            }
            turret.setState(Turret.State.TARGET_LOCK);
        }
    }

    public void wobble() {
        if (universalGamepad.wobbleButton.wasJustPressed()) {
            switch (robot.wobbleArm.getState()) {
                case UP:
                    wobbleArm.dropMacro();
                    break;
                case MID:
                case DOWN:
                    wobbleArm.claw.grab();
                    wobbleArm.setState(WobbleArm.State.UP);
                    break;
            }
        }
        if (universalGamepad.clawButton.wasJustPressed()) {
            switch (wobbleArm.claw.getState()) {
                case GRIP: {
                    if (wobbleArm.getState() != WobbleArm.State.UP) robot.wobbleArm.claw.release();
                    break;
                }
                case RELEASE: {
                    robot.wobbleArm.claw.grab();
                    break;
                }
            }
        }
        if (universalGamepad.g2.gamepad.right_bumper) {
            robot.wobbleArm.pickUp();
        }
    }
}
