package org.firstinspires.ftc.teamcode.Tele;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Components.PIDController;
import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.Odometry.OdometryGlobalCoordinatePosition;
import org.firstinspires.ftc.teamcode.PurePursuit.Coordinate;

import java.io.DataInput;
import java.util.ArrayList;
import java.util.Collections;

import static org.firstinspires.ftc.teamcode.PurePursuit.MathFunctions.AngleWrap;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp", group = "LinearOpMode")
public class TeleOp extends LinearOpMode implements Runnable{
    Robot robot;
    Runnable wingDefault;
    boolean ninja, armUp;
    Coordinate shootingPosition;
    double shootingAngle = 0;
    boolean isWingsOut = false;
    double velo = 0;
    int wingsLimit = 0;
    ArrayList<Double> timer = new ArrayList<Double> ();
    double currentMillis = 0;
    double lastMillis = 0;
    int cycles = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, telemetry, () -> opModeIsActive() && gamepadIdle());
        robot.init();
        wingDefault = ()->robot.launcher.wingsVert();
       // pidRotate = new PIDController(.07, 0.014, 0.0044);
        Thread driveTrain = new Thread(this);
        waitForStart();
        //shootingPosition = Robot.position.toPoint();
        driveTrain.start();

        while(opModeIsActive()){
            currentMillis = System.currentTimeMillis();
            setMultiplier();
            wobble();
            robot.intake(gamepad2.right_stick_y);
            shooter();
            robot.slap();
            idle();
            
            if(gamepad2.y || gamepad1.right_trigger >= 0.2){
                if(!isWingsOut) {
                    wingDefault = ()-> robot.wingsOut();
                    robot.wingsOut();
                    isWingsOut = true;
                    sleep(400);
                } else {
                    wingDefault = ()->robot.launcher.wingsVert();
                    robot.launcher.wingsVert();
                    isWingsOut = false;
                    sleep(400);
                }
            }


            telemetry.addData("Distance",robot.launcher.colorRangeSensor.getDistance(DistanceUnit.INCH));
//            telemetry.addData("left", Robot.position.verticalEncoderLeft.getCurrentPosition());
//            telemetry.addData("right", Robot.position.verticalEncoderRight.getCurrentPosition());
//            telemetry.addData("horizontal", Robot.position.horizontalEncoder.getCurrentPosition());
            telemetry.addData("Velocity", velo);
            telemetry.addData("current", robot.launcher.getVelocity());
            telemetry.addData("Average Cycle Time", calcAvg());
            telemetry.addData("Median Cycle Time", calcMedian());
            telemetry.update();
        }
        driveTrain.stop();
        Robot.position.stop();
    }
    double lxMult = 1;
    double lyMult = 1;
    double rxMult = 1;
    @Override
    public void run() {
        while(opModeIsActive()) {
            robot.driveTrain.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y * lyMult,
                            -gamepad1.left_stick_x * lxMult,
                            -gamepad1.right_stick_x *rxMult
                    )
            );
            //robot.setMovement(gamepad1.left_stick_x * lxMult, -gamepad1.left_stick_y * lyMult, gamepad1.right_stick_x * rxMult);
            if(gamepad1.y) {
                //robot.goTo(shootingPosition, 0.8, shootingAngle, 0.6);
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
    private void autoPowerShots(){
        for(Coordinate pwrShot : Robot.pwrShots){
            robot.turnTo(pwrShot, 0.24);
        }
    }
    private void strafePowerShot(){
        if(gamepad2.dpad_down){
            robot.launcher.setFlyWheel(0.45);
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
            robot.launcher.setFlyWheel(0);
            robot.launcher.tiltDown();
        }

    }
    public void shooter(){
        if(gamepad2.left_trigger >= 0.1){
            robot.launcher.flapDown();
            if(robot.launcher.getRings() < 3){
                robot.wingsOut();
            } else {
                robot.wingsMid();
            }
            robot.launcher.setFlyWheel(0.8);
            if(robot.launcher.getVelocity() >= 1340){
                robot.launcher.magazineShoot();
                if(cycles == 0){
                    cycles++;
                    lastMillis = currentMillis;
                }
                else{
                    timer.add(currentMillis - lastMillis);
                    lastMillis = currentMillis;
                }

            }
        }
        else if(gamepad2.left_bumper){
            robot.launcher.setFlyWheel(0.45);
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
            if(robot.launcher.getRings() >= 1){
                robot.launcher.setOnlyFlyWheel(0.55); //change to make constant speed
            } else {
                robot.launcher.setOnlyFlyWheel(Math.abs(gamepad2.left_stick_y));
            }//change to make constant speed
            robot.launcher.flapDown();
        }
        if(gamepad2.right_bumper){
            robot.launcher.singleRound();
            wingsLimit++;
            storeCoordinate();
        }
        if(gamepad2.right_trigger >= 0.1){
            wingsLimit++;
            velo = robot.launcher.getVelocity();
            robot.launcher.magazineShoot();
            storeCoordinate();
        }
    }
    private void storeCoordinate(){
        //shootingPosition = Robot.position.toPoint();
       // shootingAngle = Robot.position.radians();
    }
    private boolean gamepadIdle(){
        return gamepad1.left_stick_x == 0 && gamepad1.left_stick_y == 0 && gamepad1.right_stick_x == 0;
    }
    private void goToLaunchZone(){
        robot.goTo(shootingPosition, 0.5, shootingAngle, 0.5);
    }
}
