package org.firstinspires.ftc.teamcode.Tele;

import android.util.Log;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.Async;
import org.firstinspires.ftc.teamcode.Components.OpModeType;
import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.Components.TuningController;
import org.firstinspires.ftc.teamcode.PurePursuit.Coordinate;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.PurePursuit.Coordinate.toPose;

@TeleOp(name="TeleOp", group = "LinearOpMode")
@Config
public class MainTele extends LinearOpMode implements Runnable{
    Robot robot;
    Runnable wingDefault;
    boolean ninja, armUp;
    boolean isWingsOut = false;
    ArrayList<Double> timer = new ArrayList<Double> ();
    double currentMillis = 0;
    double lastMillis = 0;
    int cycles = 0;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    Pose2d shootingPose = new Pose2d();
    Trajectory powerShots;
    Trajectory shootingPath;
    public static double targetVelocity;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, OpModeType.tele);
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        robot.init();
        wingDefault = ()->robot.launcher.wingsVert();
        waitForStart();
        Async.start(this);
//        Async.set(this::opModeIsActive, () -> {
//            if(robot.intakeL.getPower() != 0) {
//                sleep(600);
//                powerShots = robot.driveTrain.trajectoryBuilder()
//                        .splineTo(Robot.pwrShotLocals[2], 0)
//                        .addDisplacementMarker(() -> Async.start(() -> robot.launcher.singleRound()))
//                        .splineToLinearHeading(toPose(Robot.pwrShotLocals[1], 0), 0, new MinVelocityConstraint(
//                                        Arrays.asList(
//                                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                                new MecanumVelocityConstraint(9, DriveConstants.TRACK_WIDTH)
//                                        )
//                                ),
//                                new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                        .addDisplacementMarker(() -> Async.start(() -> robot.launcher.singleRound()))
//                        .splineToLinearHeading(toPose(Robot.pwrShotLocals[0], 0), 0, new MinVelocityConstraint(
//                                        Arrays.asList(
//                                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
//                                                new MecanumVelocityConstraint(9, DriveConstants.TRACK_WIDTH)
//                                        )
//                                ),
//                                new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                        .addDisplacementMarker(() -> Async.start(() -> robot.launcher.singleRound()))
//                        .build();
//                shootingPath = robot.driveTrain.trajectoryBuilder()
//                        .splineToLinearHeading(shootingPose, 0)
//                        .build();
//            }
//        });

        while(opModeIsActive()){
            if(robot.driveTrain.getMode() == SampleMecanumDrive.Mode.IDLE) {
                currentMillis = System.currentTimeMillis();
                setMultiplier();
                wobble();
                robot.intake(gamepad2.right_stick_y);
                shooter();
                if (gamepad2.y || gamepad1.right_trigger >= 0.2) {
                    if (!isWingsOut) {
                        wingDefault = () -> robot.wingsOut();
                        robot.wingsOut();
                        isWingsOut = true;
                        sleep(400);
                    } else {
                        wingDefault = () -> robot.launcher.wingsVert();
                        robot.launcher.wingsVert();
                        isWingsOut = false;
                        sleep(400);
                    }
                }
                idle();
            }
        }
    }
    double lxMult = 1;
    double lyMult = 1;
    double rxMult = 1;
    @Override
    public void run() {
        while(opModeIsActive()) {
            robot.launcher.updatePID();
            if(robot.driveTrain.getMode() == SampleMecanumDrive.Mode.IDLE) {
                robot.driveTrain.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y * lyMult,
                                -gamepad1.left_stick_x * lxMult,
                                -gamepad1.right_stick_x * 0.9 * rxMult
                        )
                );
                robot.driveTrain.update();
            }

            if(gamepad1.y) {
                sleep(300);
                robot.driveTrain.followTrajectoryAsync(shootingPath);
            }
            if(gamepad1.b) {
                sleep(300);
                robot.driveTrain.followTrajectoryAsync(powerShots);
            }
            if(!gamepadIdle()) robot.driveTrain.setMode(SampleMecanumDrive.Mode.IDLE);

            telemetry.addData("velocity", robot.launcher.getVelocity());
            telemetry.addData("targetVelocity", robot.launcher.getTargetVelo());
            telemetry.addData("error", robot.launcher.getTargetVelo() - robot.launcher.getVelocity());
            telemetry.update();
        }
    }
    private double calcAvg(){
        double avg = 0;
        for(int i = 0; i < timer.size (); i++){
            avg += timer.get(i)/1000.0;
        }
        avg/= timer.size();
        return avg;
    }
    private double calcMedian(){
        Collections.sort(timer);
        return timer.size() != 0 ? timer.get(timer.size()/2) : 0;
    }
    private void setMultiplier(){
        if(!ninja && gamepad1.left_bumper){
            ninja = true;
            lxMult /= 3;
            rxMult /= 3;
            lyMult /= 3;
            sleep(300);
        }
        else if(ninja && gamepad1.left_bumper){
            ninja = false;
            lxMult *= 3;
            rxMult *= 3;
            lyMult *= 3;
            sleep(300);
        }
        if(gamepad1.right_bumper){
            lxMult = -lxMult;
            lyMult = -lyMult;
            sleep(300);
        }
    }
    public void wobble(){
        if(gamepad2.b && !armUp){
            robot.wobbleArmUp();
            armUp = true;
            sleep(300);
        }
        else if(gamepad2.b && armUp){
            robot.wobbleArmDown();
            armUp = false;
            sleep(300);
        }
        else if(gamepad2.a){
            robot.wobbleArmVertical();
            sleep(300);
            robot.release();
            sleep(200);
            robot.wobbleArmUp();
            armUp = true;
        }
        if(gamepad2.x && robot.grabber.getPosition()>0.3){
            robot.grab();
            sleep(300);
        }
        else if(gamepad2.x && robot.grabber.getPosition()<0.3){
            robot.release();
            sleep(300);
        }
    }
    private void strafePowerShot(){
        if(gamepad2.dpad_down){
            //robot.launcher.setFlyWheel(0.45);
            Trajectory traj = robot.driveTrain.trajectoryBuilder()
                    .strafeLeft(7.5)
                    .build();

            robot.launcher.tiltUp();
            robot.launcher.singleRound();
            for(int i = 0; i<2; i++){
                robot.driveTrain.followTrajectory(traj);
                robot.launcher.mag.setPosition(0.34);
                sleep(110);
                robot.launcher.mag.setPosition(0.48);
            }
           // robot.launcher.setFlyWheel(0);
            robot.launcher.tiltDown();
        }

    }
    public void shooter(){
        if(gamepad2.left_trigger >= 0.1){
            robot.launcher.flapDown();
            robot.launcher.tiltUp();
            if(robot.launcher.getRings() < 3){
                robot.wingsOut();
            } else {
                robot.wingsMid();
            }
            robot.launcher.setVelocity(targetVelocity);
            if(Math.abs(robot.launcher.getTargetVelo() - robot.launcher.getVelocity()) <= 60){
                sleep(400);
                robot.launcher.magazineShoot();
            }
        }
        else if(gamepad2.left_bumper){
            robot.launcher.tiltUp();
            robot.launcher.setVelocity(1170);
            robot.wingsOut();
            robot.launcher.flapDown();
        }
        else {
            robot.launcher.tiltDown();
            if(robot.launcher.getRings() < 3) {
                wingDefault.run();
            } else {
                robot.launcher.wingsMid();
            }
            if(robot.launcher.getRings() == 1){
                robot.launcher.setVelocity(targetVelocity/2); //change to make constant speed
            } else if(robot.launcher.getRings() == 2){
                robot.launcher.setVelocity(targetVelocity*4/5);
            } else if(robot.launcher.getRings() == 3){
                robot.launcher.setVelocity(targetVelocity);
            } else {
                robot.launcher.setVelocity(0);//change to make constant speed
            }
            robot.launcher.flapDown();
        }
        if(gamepad2.right_bumper){
            robot.launcher.singleRound();
            storeCoordinate();
        }
        if(gamepad2.right_trigger >= 0.1){
            robot.launcher.magazineShoot();
            if(cycles == 0){
                cycles++;
            }
            else{
                timer.add(currentMillis - lastMillis);
            }
            lastMillis = currentMillis;
            storeCoordinate();
        }
    }
    private void storeCoordinate(){
        shootingPose = new Coordinate(robot.driveTrain.getPoseEstimate())
                .toPose2d(robot.driveTrain.getPoseEstimate().getHeading());
    }
    private boolean gamepadIdle(){
        return Math.abs(gamepad1.left_stick_x) <= 0.3 && Math.abs(gamepad1.left_stick_y) <= 0.3 && Math.abs(gamepad1.right_stick_x) <= 0.3;
    }
}
