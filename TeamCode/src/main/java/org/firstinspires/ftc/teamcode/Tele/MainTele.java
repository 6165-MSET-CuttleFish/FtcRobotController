package org.firstinspires.ftc.teamcode.Tele;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.Async;
import org.firstinspires.ftc.teamcode.Components.OpModeType;
import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.PurePursuit.Coordinate;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import java.util.ArrayList;
import java.util.Collections;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="TeleOp", group = "LinearOpMode")
@Config
public class MainTele extends LinearOpMode implements Runnable{
    Robot robot;
    enum WingState{
        out,
        in,
        safe,
        vertical
    }
    WingState wingDefault;
    boolean ninja, armUp;
    boolean wingCheck;
    ArrayList<Double> timer = new ArrayList<>();
    double currentMillis = 0;
    double lastMillis = 0;
    int cycles = 0;
    Pose2d shootingPose = new Pose2d();
    Trajectory shootingPath;
    public static double targetVelocity= 1480;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, OpModeType.tele);
        robot.init();
        wingDefault = WingState.vertical;
        waitForStart();
        Async.start(this);
        Async.start(() -> {
            while(opModeIsActive()) {
                if (robot.intakeL.getPower() != 0) {
                    sleep(300);
                    shootingPath = robot.driveTrain.trajectoryBuilder()
                            .splineToLinearHeading(Robot.shootingPose, 0)
                            .build();
                }
            }
        });

        while(opModeIsActive()){
                currentMillis = System.currentTimeMillis();
                setMultiplier();
                wobble();
                robot.intake(gamepad2.right_stick_y);
                shooter();
                if (gamepad1.right_trigger >= 0.2 && !wingCheck) {
                    wingCheck = true;
                    switch (wingDefault){
                        case in: wingDefault = WingState.out;
                        case out: wingDefault = WingState.vertical;
                        case safe: wingDefault = WingState.vertical;
                        case vertical: wingDefault = WingState.out;
                    }
                }
                if(gamepad1.right_trigger < 0.2){
                    wingCheck = false;
                }
            switch (wingDefault){
                case in: robot.launcher.wingsIn();
                case out:
                    if(robot.launcher.getRings() < 3)robot.launcher.wingsOut();
                    else robot.launcher.wingsMid();
                case safe: robot.launcher.wingsMid();
                case vertical: robot.launcher.wingsVert();
            }
                idle();
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
                                -gamepad1.right_stick_x * 0.92 * rxMult
                        )
                );
                robot.driveTrain.update();
            }

            if(gamepad1.a) {
                if(shootingPath != null)
                robot.driveTrain.followTrajectory(shootingPath, ()->{
                    robot.launcher.updatePID();
                    if(!gamepadIdle()) robot.driveTrain.setMode(SampleMecanumDrive.Mode.IDLE);
                });
            }
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
    public void shooter(){
        if(gamepad2.left_trigger >= 0.1){
            robot.launcher.flapDown();
            robot.launcher.tiltUp();
            if(robot.launcher.getRings() < 3){
                wingDefault = WingState.out;
            } else {
                wingDefault = WingState.safe;
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
            wingDefault = WingState.out;
            robot.launcher.flapDown();
        }
        else {
            robot.launcher.tiltDown();
            switch(robot.launcher.getRings()){
                case 3: robot.launcher.setVelocity(targetVelocity);
                case 2: robot.launcher.setVelocity(targetVelocity);
                case 1: robot.launcher.setVelocity(targetVelocity/2);
                case 0: robot.launcher.setVelocity(0);
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
