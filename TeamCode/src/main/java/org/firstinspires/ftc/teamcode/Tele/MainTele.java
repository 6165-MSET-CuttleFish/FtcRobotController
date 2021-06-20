package org.firstinspires.ftc.teamcode.Tele;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.Async;
import org.firstinspires.ftc.teamcode.Components.OpModeType;
import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.PurePursuit.Coordinate;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.ArrayList;
import java.util.Collections;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
    double coastDownTime = 500;
    double setInterval = 10;
    int cycles = 0;
    Pose2d shootingPose = Robot.shootingPoseTele;
    public static double targetVelocity = 1330;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this, 87, 32.75, Math.toRadians(180), OpModeType.tele);
        wingDefault = WingState.out;
        sleep(500);
        robot.setPoseEstimate(Robot.robotPose);
        robot.shooter.wingsOut();
        telemetry.addData("Initialization", "Complete");
        telemetry.update();
        waitForStart();
        Async.start(this);
        Async.start(() -> {
            while (opModeIsActive()) {
                robot.shooter.update();
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
                robot.shooter.wingsIn();
            } else if (wingDefault == WingState.out) {
                if (robot.shooter.getRings() < 1){
                    robot.shooter.wingsOut();
                }
                else {
                    robot.shooter.wingsMid();
                    //robot.setSlappy(0);
                }
            } else if (wingDefault == WingState.safe) {
                robot.shooter.wingsMid();
                //robot.setSlappy(0);
            } else {
                robot.shooter.wingsVert();
            }
            if(!(gamepad2.dpad_up || gamepad2.dpad_down)) raiseCheck = true;
            telemetry.addData("Coast", wantsCoastDown);
            telemetry.addData("Cadence", veloCadence);
            telemetry.addData("Target Velocity", targetVelocity);
            telemetry.addData("Distance To High Goal", Coordinate.distanceToLine(robot.getPoseEstimate(), Robot.goal.getX()));
            telemetry.update();
            idle();
        }
    }

    double lxMult = 1;
    double lyMult = 1;
    double rxMult = 1;
    boolean g1Check;
    boolean powershots = false;
    @Override
    public void run() {
        while (opModeIsActive()) {
            shooterDisabled = false;
            if(!powershots) targetVelocity = robot.shooter.getPoseVelo(robot.getPoseEstimate().vec()) - 40;
            if (!gamepadIdle()) robot.setMode(Robot.Mode.IDLE);
            if (robot.getMode() == Robot.Mode.IDLE) {
                robot.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y * lyMult,
                                -gamepad1.left_stick_x * lxMult,
                                -gamepad1.right_stick_x * 0.92 * rxMult
                        )
                );
                setMultiplier();
                robot.update();
            }
            if(!g1Check) {
                if (gamepad1.x) {
                    g1Check = true;
                    robot.followTrajectory(robot.trajectoryBuilder(robot.getPoseEstimate())
                            .addDisplacementMarker(() -> {
                                shooterDisabled = true;
                                robot.shooter.setVelocity(1130);
                                wingDefault = WingState.out;
                            })
                            .addTemporalMarker(0.5, () -> robot.shooter.magUp())
                            .lineToLinearHeading(Coordinate.toPose(Robot.pwrShotLocals[1].plus(new Vector2d(-8, 0)), 0))
                            .build(), () -> {
                        if (!gamepadIdle()) robot.setMode(Robot.Mode.IDLE);
                    });
                    for (Vector2d pwrShot : Robot.pwrShots) {
                        if (!gamepadIdle()) {
                            break;
                        }
                        Coordinate position = Coordinate.toPoint(robot.getPoseEstimate());
                        position.polarAdd(robot.getPoseEstimate().getHeading() + Math.PI / 2, 4);
                        double absAngleToTarget = Math.atan2(pwrShot.getY() - position.getY(), pwrShot.getX() - position.getX());
                        double relAngleToPoint = AngleWrap(absAngleToTarget - robot.getPoseEstimate().getHeading());
                        robot.turn(relAngleToPoint, () -> {
                            if (!gamepadIdle()) {
                                robot.setMode(Robot.Mode.IDLE);
                            }
                        });
                        robot.shooter.singleRound();
                    }
                } else if (gamepad1.left_bumper) {
                    g1Check = true;
                    robot.followTrajectory(robot.trajectoryBuilder(robot.getPoseEstimate())
                            .addDisplacementMarker(() -> {
                                shooterDisabled = true;
                                robot.shooter.setVelocity(Robot.shootingPoseTele.vec());
                            })
                            .addTemporalMarker(0.7, () -> robot.shooter.magUp())
                            .splineToLinearHeading(Robot.shootingPoseTele, 0)
                            .build(), () -> {
                        if (!gamepadIdle()) robot.setMode(Robot.Mode.IDLE);
                    });
                    while (gamepadIdle() || Math.abs(robot.shooter.getError()) >= 40) {
                        robot.update();
                    }
                    robot.optimalShoot(3);
                } else if (gamepad1.right_trigger >= 0.2) {
                    wingDefault = WingState.out;
                    g1Check = true;
                    shooterDisabled = true;
                    robot.shooter.magUp();
                    Pose2d robotPose = robot.getPoseEstimate();
                    double absAngleToTarget = Math.atan2(Robot.goal.getY() - robotPose.getY(), Robot.goal.getX() - robotPose.getX());
                    double relAngleToPoint = AngleWrap(absAngleToTarget - robot.getPoseEstimate().getHeading());
                    robot.turn(relAngleToPoint, () -> {
                        if (!gamepadIdle()) {
                            robot.setMode(Robot.Mode.IDLE);
                        }
                    });
                    robot.optimalShoot(3);
                }
            } if(gamepad1.right_trigger < 0.2 && !gamepad1.left_bumper && !gamepad1.x){
                g1Check = false;
            }
            if (gamepad2.right_trigger >= 0.1) {
                robot.shooter.singleRound();
                robot.turn(Math.toRadians(9));
                robot.shooter.singleRound();
                robot.turn(Math.toRadians(5));
                robot.shooter.singleRound();
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
            robot.shooter.magUp();
            wingDefault = WingState.out;
            robot.shooter.setVelocity(targetVelocity);
            if (Math.abs(robot.shooter.getError()) <= 30 && gamepadIdle() && !robot.isBusy() && !wasPressed) {
                wasPressed = true;
                sleep(180);
                robot.optimalShoot(3);
            }
        } else if (gamepad2.left_bumper) {
            powershots = true;
            robot.shooter.magUp();
            robot.shooter.setVelocity(1130);
            wingDefault = WingState.out;
        } else {
            powershots = false;
            robot.shooter.magDown();
            if (robot.shooter.getRings() > 0) robot.shooter.setVelocity(targetVelocity);
            else {
                if(wantsCoastDown) {
                    if (wasPressed) {
                        wasPressed = false;
                        coastTimer = System.currentTimeMillis();
                        veloCadence = (robot.shooter.getVelocity() - 1000) * setInterval / coastDownTime;
                        robot.shooter.setVelocity(robot.shooter.getTargetVelo() - veloCadence);
                    }
                    if (System.currentTimeMillis() > coastTimer + setInterval && robot.shooter.getTargetVelo() > 1000) {
                        coastTimer = System.currentTimeMillis();
                        robot.shooter.setVelocity(robot.shooter.getTargetVelo() - veloCadence);
                    }
                }
                else robot.shooter.setVelocity(0);
            }
        }
        if (gamepad2.right_bumper) {
            robot.shooter.singleRound();
            storeCoordinate();
        }
    }

    private void storeCoordinate() {
        shootingPose = new Coordinate(robot.getPoseEstimate())
                .toPose2d(robot.getPoseEstimate().getHeading());
    }

    private boolean gamepadIdle() {
        return Math.abs(gamepad1.left_stick_x) <= 0.3 && Math.abs(gamepad1.left_stick_y) <= 0.3 && Math.abs(gamepad1.right_stick_x) <= 0.15;
    }
}
