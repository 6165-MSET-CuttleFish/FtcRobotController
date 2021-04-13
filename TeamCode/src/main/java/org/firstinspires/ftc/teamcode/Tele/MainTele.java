package org.firstinspires.ftc.teamcode.Tele;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.arcrobotics.ftclib.gamepad.*;

import org.firstinspires.ftc.teamcode.Components.Async;
import org.firstinspires.ftc.teamcode.Components.OpModeType;
import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.PurePursuit.Coordinate;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.ArrayList;
import java.util.Collections;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.PurePursuit.MathFunctions.AngleWrap;

@TeleOp(name = "TeleOp", group = "LinearOpMode")
@Config
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
    boolean wingCheck, reverseCheck;
    boolean shooterDisabled;
    ArrayList<Double> timer = new ArrayList<>();
    double currentMillis = 0;
    double lastMillis = 0;
    int cycles = 0;
    Pose2d shootingPose = Robot.shootingPoseTele;
    Trajectory shootingPath;
    Trajectory shootingPathAutoShoot;
    Trajectory powerShotPath;
    public static double targetVelocity = 1500;
    Vector2d targetVector;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, 87, 9, Math.toRadians(180), OpModeType.tele);
        robot.init();
        wingDefault = WingState.vertical;
        generatePaths();
        waitForStart();
        Async.start(this);
        Async.start(() -> {
            while (opModeIsActive()) {
                if (robot.driveTrain.getMode() == SampleMecanumDrive.Mode.IDLE) {
                    sleep(300);
                    generatePaths();
                }
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
            targetVector = null;
            robot.launcher.updatePID();
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
                robot.driveTrain.followTrajectory(robot.driveTrain.trajectoryBuilder(robot.driveTrain.getPoseEstimate())
                        .addDisplacementMarker(() -> {
                            shooterDisabled = true;
                            robot.launcher.setVelocity(1170);
                            wingDefault = WingState.out;
                        })
                        .lineToLinearHeading(Coordinate.toPose(Robot.pwrShotLocals[2], 0))
                        .build(), () -> {
                    robot.launcher.updatePID();
                    if (!gamepadIdle()) robot.driveTrain.setMode(SampleMecanumDrive.Mode.IDLE);
                });
                for (Vector2d pwrShot : Robot.pwrShots) {
                    Pose2d position = robot.driveTrain.getPoseEstimate();
                    double absAngleToTarget = Math.atan2(pwrShot.getY() - position.getY(), pwrShot.getX() - position.getX());
                    double relAngleToPoint = AngleWrap(absAngleToTarget - position.getHeading());
                    robot.driveTrain.turn(relAngleToPoint, () -> {
                        robot.launcher.updatePID();
                        if (!gamepadIdle()) robot.driveTrain.setMode(SampleMecanumDrive.Mode.IDLE);
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
                    robot.launcher.updatePID();
                    if (!gamepadIdle()) robot.driveTrain.setMode(SampleMecanumDrive.Mode.IDLE);
                });
                while (gamepadIdle() || Math.abs(robot.launcher.getError()) <= 40 || gamepad2.right_trigger <= 0.2) {
                    robot.driveTrain.update();
                    robot.launcher.updatePID();
                }
                robot.launcher.magazineShoot();
            } else if (gamepad1.right_trigger >= 0.2) {
                Pose2d position = robot.driveTrain.getPoseEstimate();
                double absAngleToTarget = Math.atan2(Robot.goal.getY() - position.getY(), Robot.goal.getX() - position.getX());
                double relAngleToPoint = AngleWrap(absAngleToTarget - position.getHeading());
                robot.driveTrain.turnAsync(relAngleToPoint);
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

    private void generatePaths() {
        shootingPath = robot.driveTrain.trajectoryBuilder(robot.driveTrain.getPoseEstimate())
                .lineToLinearHeading(Robot.shootingPoseTele)
                .build();
        shootingPathAutoShoot = robot.driveTrain.trajectoryBuilder(robot.driveTrain.getPoseEstimate())
                .addDisplacementMarker(() -> {
                    targetVector = Robot.shootingPoseTele.vec();
                    robot.launcher.setLauncherVelocity(getNeededVelocity());
                })
                .lineToLinearHeading(Robot.shootingPoseTele)
                .build();
        powerShotPath = robot.driveTrain.trajectoryBuilder(robot.driveTrain.getPoseEstimate())
                .addDisplacementMarker(() -> {
                    shooterDisabled = true;
                    robot.launcher.setVelocity(1170);
                    wingDefault = WingState.out;
                })
                .lineToLinearHeading(Coordinate.toPose(Robot.pwrShotLocals[2], 0))
                .build();
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

    public void shooter() {
        if (gamepad2.left_trigger >= 0.1) {
            robot.launcher.magUp();
            if (robot.launcher.getRings() < 3) {
                wingDefault = WingState.out;
            } else {
                wingDefault = WingState.safe;
            }
            robot.launcher.setVelocity(targetVelocity);
            if (Math.abs(robot.launcher.getTargetVelo() - robot.launcher.getVelocity()) <= 30) {
                sleep(400);
                robot.launcher.magazineShoot();
            }
        } else if (gamepad2.left_bumper) {
            robot.launcher.magUp();
            robot.launcher.setVelocity(1170);
            wingDefault = WingState.out;
        } else {
            robot.launcher.magDown();
            if (robot.launcher.getRings() > 0) robot.launcher.setVelocity(targetVelocity);
            else robot.launcher.setVelocity(0);
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

    private double getNeededVelocity() {
        if (targetVector != null) return robot.getPoseVelo(targetVector);
        else return robot.getPoseVelo(robot.driveTrain.getPoseEstimate());
    }

    private boolean gamepadIdle() {
        return Math.abs(gamepad1.left_stick_x) <= 0.3 && Math.abs(gamepad1.left_stick_y) <= 0.3 && Math.abs(gamepad1.right_stick_x) <= 0.3;
    }
}
