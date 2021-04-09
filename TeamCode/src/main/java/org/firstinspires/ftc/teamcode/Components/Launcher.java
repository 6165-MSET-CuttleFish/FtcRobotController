package org.firstinspires.ftc.teamcode.Components;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static org.firstinspires.ftc.teamcode.PurePursuit.MathFunctions.*;
@Config
public class Launcher {
    public ColorRangeSensor colorRangeSensor;
    static final double launcherHeight = 0.2032;
    static final double V = 9.9059;
    static final double g = -9.08711677875;
    public static PIDCoefficients MOTOR_VELO_PID = new PIDCoefficients(0.002, 0, 0.00009);

    public static double kV = 0.00052428571428572;//1 / TuningController.rpmToTicksPerSecond(TuningController.MOTOR_MAX_RPM);
    public static double kA = 0.0003;
    public static double kStatic = 0;
    public static long sleepTime = 550;

    double lastTargetVelo = 0.0;
    double lastKv = kV;
    double lastKa = kA;
    double lastKstatic = kStatic;
    VelocityPIDFController veloController = new VelocityPIDFController(MOTOR_VELO_PID, kV, kA, kStatic);

    private final ElapsedTime veloTimer = new ElapsedTime();
    public DcMotorEx flywheel, flywheel1;
    public Servo mag, flap, tilt;
    public Servo rightIntakeHolder, leftIntakeHolder;
    public double targetVelo;
    public volatile boolean isActive;

    public Launcher(HardwareMap map){
        colorRangeSensor = map.get(ColorRangeSensor.class, "range");
        flywheel = map.get(DcMotorEx.class, "fw");
        flywheel1 = map.get(DcMotorEx.class, "fw1");
        flywheel.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        mag = map.get(Servo.class, "mag");
        flap = map.get(Servo.class, "flap");
        tilt = map.get(Servo.class, "tilt");
        leftIntakeHolder = map.get(Servo.class,"wallL");
        rightIntakeHolder = map.get(Servo.class,"wallR");
        singleRound();
        tiltDown();
        if(Robot.opModeType == OpModeType.auto) wingsIn();
        for (LynxModule module : map.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        veloTimer.reset();
    }
    public void run(){
            while (isActive) {
                updatePID();
            }
    }
    public void updatePID(){
        veloController.setTargetVelocity(targetVelo);
        veloController.setTargetAcceleration((targetVelo - lastTargetVelo) / veloTimer.seconds());
        veloTimer.reset();
        lastTargetVelo = targetVelo;
        double motorPos = flywheel.getCurrentPosition();
        double motorVelo = flywheel.getVelocity();
        double power = veloController.update(motorPos, motorVelo);
        if (targetVelo == 0) {
            flywheel.setPower(0);
            flywheel1.setPower(0);
        } else {
            flywheel.setPower(power);
            flywheel1.setPower(power);
        }
        if(lastKv != kV || lastKa != kA || lastKstatic != kStatic) {
            lastKv = kV;
            lastKa = kA;
            lastKstatic = kStatic;
            veloController = new VelocityPIDFController(MOTOR_VELO_PID, kV, kA, kStatic);
        }
        if(getVelocity() > 100) Log.println(Log.INFO, "Velocity: ", getVelocity() + "");
    }
    public void stop(){
        isActive = false;
    }
    public int getRings(){
        double range = colorRangeSensor.getDistance(DistanceUnit.INCH);
            if (range < 2.6) {
                return 3;
            } else if (range < 4.1) {
                return 2;
            } else if (range < 7) {
                return 1;
            } else {
                return 0;
            }
    }
    public double getDistance(){
        return colorRangeSensor.getDistance(DistanceUnit.INCH);
    }
    public void setVelocity(double v){
        targetVelo = v;
    }
    public void setLauncherVelocity(double v){
        targetVelo = v;
        if(v > 100) tiltUp();
        else tiltDown();
    }
    public double getVelocity(){
        return flywheel.getVelocity();
    }
    public double getTargetVelo(){
        return targetVelo;
    }
    public void wingsOut() {
        leftIntakeHolder.setPosition(0.96);
        rightIntakeHolder.setPosition(0.18);
    }
    public void wingsIn() {
        leftIntakeHolder.setPosition(.3);
        rightIntakeHolder.setPosition(.84);
    }
    public void wingsMid() {
        leftIntakeHolder.setPosition(.85);
        rightIntakeHolder.setPosition(0.3);
    }
    public void wingsVert(){
        leftIntakeHolder.setPosition(0.6);
        rightIntakeHolder.setPosition(0.6);
    }
    public void leftOut() {
        leftIntakeHolder.setPosition(.96);
        rightIntakeHolder.setPosition(0.84);
    }
    public void rightOut() {
        leftIntakeHolder.setPosition(.3);
        rightIntakeHolder.setPosition(0.18);
    }

    public void unlockIntake(){
        rightIntakeHolder.setPosition(0.4);
    }
    public static void findAngle(double d, targets t, Servo flap){
        double goalHeight = 0;
        if(t == targets.highGoal) goalHeight = 35.5;
        else if(t == targets.lowGoal) goalHeight = 17;
        else if(t == targets.pwrShot) goalHeight = 23.5;
        goalHeight = inchesToMeters(goalHeight);
        double theta;
        double h = goalHeight - launcherHeight;
        theta = Math.toDegrees((Math.acos((g*d*d/(V*V) - h)/Math.sqrt(h*h + d*d)) - Math.acos(h/Math.sqrt(h*h + d*d)))/2);
        //System.out.println(theta);
        flap.setPosition(setAngle(theta));
    }
    public static double setAngle(double theta){
        theta -= 25;
        return -0.1*theta*theta + 3*theta;
    }
    public void tiltUp(){
        tilt.setPosition(0.75);
    }
    public void tiltDown(){
        tilt.setPosition(0.54);
    }
    public void magazineShoot(){
        int rounds = getRings();
        for(int i = 0; i < rounds; i++){
            singleRound();
            //setOnlyFlyWheel(flyWheelSpeed + 0.08);
            sleep(sleepTime);
            if(Robot.opModeType == OpModeType.tele) wingsOut();
            if(i == rounds - 2){
                sleep(80);
            }
        }
    }
    public void singleRound(){
        mag.setPosition(0.34);
        sleep(150);
        //this is sleep value to change
        mag.setPosition(0.48);
    }
    public void flapUp(){
        flap.setPosition(0.43);
    }
    public void flapDown(){
        flap.setPosition(0.35);
    }
    enum targets{
        highGoal,
        lowGoal,
        pwrShot
    }
    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}