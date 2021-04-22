package org.firstinspires.ftc.teamcode.Tele;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.Async;
import org.firstinspires.ftc.teamcode.Components.OpModeType;
import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.PurePursuit.Coordinate;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.ArrayList;
import java.util.Collections;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.arcrobotics.ftclib.gamepad.TriggerReader;

import static org.firstinspires.ftc.teamcode.PurePursuit.MathFunctions.AngleWrap;

@TeleOp(name = "Driver Practice", group = "LinearOpMode")
public class MainTele extends LinearOpMode implements Runnable {
    Robot robot;
    enum WingState {
        out,
        in,
        safe,
        vertical
    }
    WingState wingDefault;
    boolean armUp;
    boolean wingCheck, reverseCheck, raiseCheck;
    boolean shooterDisabled;
    ArrayList<Double> timer = new ArrayList<>();
    double currentMillis = 0;
    double lastMillis = 0;
    double coastDownTime = 1000;
    double setInterval = 5;
    int cycles = 0;
    Pose2d shootingPose = Robot.shootingPoseTele;
    public static double targetVelocity = 1330;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, 87, 32.75, Math.toRadians(180), OpModeType.tele, this);
        robot.init();
        wingDefault = WingState.out;
        waitForStart();
        Async.start(this);
        Async.start(() -> {
            while (opModeIsActive()) {
               robot.launcher.updatePID();
            }
        });

        while (opModeIsActive()) {
            currentMillis = System.currentTimeMillis();
            wobble();
            robot.intake(gamepad2.right_stick_y);
            if (!shooterDisabled) shooter();
            if (gamepad2.y && !wingCheck) {
                wingCheck = true;
                if (wingDefault == WingState.in) {
                    wingDefault = WingState.out;
                } else if (wingDefault == WingState.out) {
                    wingDefault = WingState.vertical;
                } else if (wingDefault == WingState.safe) {
                    wingDefault = WingState.vertical;
                } else {
                    wingDefault = WingState.out;
                }

            }
            if (!gamepad2.y) {
                wingCheck = false;
            }
            if (wingDefault == WingState.in) {
                robot.launcher.wingsIn();
            } else if (wingDefault == WingState.out) {
                if (robot.launcher.getRings() < 3) robot.launcher.wingsOut();
                else robot.launcher.wingsMid();
            } else if (wingDefault == WingState.safe) {
                robot.launcher.wingsMid();
            } else {
                robot.launcher.wingsVert();
            }
            if(raiseCheck) {
                raiseCheck = false;
                if (gamepad2.dpad_up) {
                    targetVelocity += 30;
                } else if (gamepad2.dpad_down) {
                    targetVelocity -= 30;
                }
            }
            if(!(gamepad2.dpad_up || gamepad2.dpad_down)) raiseCheck = true;
            telemetry.addData("Coast", wantsCoastDown);
            telemetry.addData("Target Velocity", targetVelocity);
            telemetry.addData("Distance To High Goal", Coordinate.distanceToLine(robot.driveTrain.getPoseEstimate(), Robot.goal.getX()));
            telemetry.update();
            idle();
        }
    }

    double lxMult = 1;
    double lyMult = 1;
    double rxMult = 1;

    @Override
    public void run() {
        while (opModeIsActive()) {
            shooterDisabled = false;
            targetVelocity = robot.getPoseVelo(robot.driveTrain.getPoseEstimate());
            if (!gamepadIdle()) robot.driveTrain.setMode(SampleMecanumDrive.Mode.IDLE);
            if (robot.driveTrain.getMode() == SampleMecanumDrive.Mode.IDLE) {
                robot.driveTrain.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y * lyMult,
                                -gamepad1.left_stick_x * lxMult,
                                -gamepad1.right_stick_x * 0.92 * rxMult
                        )
                );
                setMultiplier();
                robot.driveTrain.update();
            }
            if (gamepad1.x) {
                shooterDisabled = true;
                robot.driveTrain.followTrajectory(robot.driveTrain.trajectoryBuilder(robot.driveTrain.getPoseEstimate())
                        .addDisplacementMarker(() -> {
                            robot.launcher.setVelocity(1120);
                            wingDefault = WingState.out;
                        })
                        .addTemporalMarker(0.5, ()-> robot.launcher.magUp())
                        .lineToLinearHeading(Coordinate.toPose(Robot.pwrShotLocals[1].plus(new Vector2d(-8, 0)), 0))
                        .build(), () -> {
                    if (!gamepadIdle()) robot.driveTrain.setMode(SampleMecanumDrive.Mode.IDLE);
                });
                for (Vector2d pwrShot : Robot.pwrShots) {
                    if (!gamepadIdle()) {
                        break;
                    }
                    Coordinate position = Coordinate.toPoint(robot.driveTrain.getPoseEstimate());
                    position.polarAdd(robot.driveTrain.getPoseEstimate().getHeading() + Math.PI/2, 4);
                    double absAngleToTarget = Math.atan2(pwrShot.getY() - position.getY(), pwrShot.getX() - position.getX());
                    double relAngleToPoint = AngleWrap(absAngleToTarget - robot.driveTrain.getPoseEstimate().getHeading());
                    robot.driveTrain.turn(relAngleToPoint, () -> {
                        if (!gamepadIdle()) {
                            robot.driveTrain.setMode(SampleMecanumDrive.Mode.IDLE);
                        }
                    });
                    robot.launcher.singleRound();
                }
            } else if (gamepad1.left_bumper) {
                robot.driveTrain.followTrajectory(robot.driveTrain.trajectoryBuilder(robot.driveTrain.getPoseEstimate())
                        .addDisplacementMarker(() -> {
                            shooterDisabled = true;
                            robot.launcher.setVelocity(robot.getPoseVelo(Robot.shootingPoseTele));
                        })
                        .addTemporalMarker(0.7, () -> robot.launcher.magUp())
                        .splineToLinearHeading(Robot.shootingPoseTele, 0)
                        .build(), () -> {
                    if (!gamepadIdle()) robot.driveTrain.setMode(SampleMecanumDrive.Mode.IDLE);
                });
                while (gamepadIdle() || Math.abs(robot.launcher.getError()) <= 40 || gamepad2.right_trigger <= 0.2) {
                    robot.driveTrain.update();
                }
                robot.launcher.magazineShoot();
            } else if(gamepad1.right_trigger >= 0.2){
                shooterDisabled = true;
                robot.launcher.magUp();
                Pose2d robotPose = robot.driveTrain.getPoseEstimate();
                double absAngleToTarget = Math.atan2(Robot.goal.getY() - robotPose.getY(), Robot.goal.getX() - robotPose.getX());
                double relAngleToPoint = AngleWrap(absAngleToTarget - robot.driveTrain.getPoseEstimate().getHeading());
                robot.driveTrain.turn(relAngleToPoint, () -> {
                    if (!gamepadIdle()) {
                        robot.driveTrain.setMode(SampleMecanumDrive.Mode.IDLE);
                    }
                });
                robot.optimalShoot(3);
            }
        }
    }

    private double calcAvg() {
        double avg = 0;
        for (int i = 0; i < timer.size(); i++) {
            avg += timer.get(i) / 1000.0;
        }
        avg /= timer.size();
        return avg;
    }

    private double calcMedian() {
        Collections.sort(timer);
        return timer.size() != 0 ? timer.get(timer.size() / 2) : 0;
    }

    private void setMultiplier() {
        if (gamepad1.left_trigger >= 0.3) {
            lxMult = 0.5;
            rxMult = 0.5;
            lyMult = 0.5;
        } else {
            lxMult = 1;
            rxMult = 1;
            lyMult = 1;
        }
        if (gamepad1.right_bumper && !reverseCheck) {
            lxMult = -lxMult;
            lyMult = -lyMult;
        }
        if (!gamepad1.right_bumper) reverseCheck = false;
    }

    public void wobble() {
        if (gamepad2.b && !armUp) {
            robot.wobbleArmUp();
            armUp = true;
            sleep(300);
        } else if (gamepad2.b) {
            robot.wobbleArmDown();
            armUp = false;
            sleep(300);
        } else if (gamepad2.a) {
            robot.wobbleArmVertical();
            sleep(300);
            robot.release();
            sleep(200);
            robot.wobbleArmUp();
            armUp = true;
        }
        if (gamepad2.x && robot.grabber.getPosition() > 0.3) {
            robot.grab();
            sleep(300);
        } else if (gamepad2.x && robot.grabber.getPosition() < 0.3) {
            robot.release();
            sleep(300);
        }
    }
    boolean wasPressed;
    boolean dpadToggle;
    boolean wantsCoastDown = true;
    double coastTimer;
    double veloCadence;
    public void shooter() {
        if(gamepad2.dpad_right && !dpadToggle){
            dpadToggle = true;
            wantsCoastDown = !wantsCoastDown;
        } else if(dpadToggle){
            dpadToggle = false;
        }
        if (gamepad2.left_trigger >= 0.1) {
            wasPressed = true;
            robot.launcher.magUp();
            wingDefault = WingState.out;
            robot.launcher.setVelocity(targetVelocity);
            if (Math.abs(robot.launcher.getError()) <= 30 && gamepadIdle()) {
                sleep(180);
                robot.launcher.tripleShot();
            }
        } else if (gamepad2.left_bumper) {
            robot.launcher.magUp();
            robot.launcher.setVelocity(1130);
            wingDefault = WingState.out;
        } else {
            robot.launcher.magDown();
            if (robot.launcher.getRings() > 0) robot.launcher.setVelocity(targetVelocity);
            else {
                if(wantsCoastDown) {
                    if (wasPressed) {
                        wasPressed = false;
                        coastTimer = System.currentTimeMillis();
                        veloCadence = (robot.launcher.getVelocity() - 1000) * setInterval / coastDownTime;
                        robot.launcher.setVelocity(robot.launcher.getTargetVelo() - veloCadence);
                    }
                    if (System.currentTimeMillis() > coastTimer + setInterval && robot.launcher.getTargetVelo() > 1000) {
                        coastTimer = System.currentTimeMillis();
                        robot.launcher.setVelocity(robot.launcher.getTargetVelo() - veloCadence);
                    }
                }
                else robot.launcher.setVelocity(0);
            }
        }
        if (gamepad2.right_bumper) {
            robot.launcher.singleRound();
            storeCoordinate();
        }
        if (gamepad2.right_trigger >= 0.1) {
            robot.launcher.magazineShoot();
            if (cycles == 0) {
                cycles++;
            } else {
                timer.add(currentMillis - lastMillis);
            }
            lastMillis = currentMillis;
            storeCoordinate();
        }
    }

    private void storeCoordinate() {
        shootingPose = new Coordinate(robot.driveTrain.getPoseEstimate())
                .toPose2d(robot.driveTrain.getPoseEstimate().getHeading());
    }

    private boolean gamepadIdle() {
        return Math.abs(gamepad1.left_stick_x) <= 0.3 && Math.abs(gamepad1.left_stick_y) <= 0.3 && Math.abs(gamepad1.right_stick_x) <= 0.15;
    }
}
