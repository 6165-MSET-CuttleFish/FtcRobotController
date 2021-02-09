package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.PurePursuit.Coordinate;
import org.firstinspires.ftc.teamcode.PurePursuit.MathFunctions;

import java.util.concurrent.Callable;

import static org.firstinspires.ftc.teamcode.PurePursuit.MathFunctions.*;

public class Launcher {
    static final double launcherHeight = 0.2032;
    static final double V = 9.9059;
    static final double g = -9.08711677875;
    public DcMotor flywheel, flywheel1;
    public Servo mag, flap, tilt;
    public static Goal position;
    private boolean isRunning = true;
    private Coordinate robotPosition;
    private double lastPosition;
    private double lastTime = System.currentTimeMillis();
    private double targetVelocity = 0;
    PIDController controller;
    public boolean isShooting;
    public Launcher(HardwareMap map){
        controller = new PIDController(0.5, 0, 0.8);
        flywheel = map.get(DcMotor.class, "fw");
        flywheel1 = map.get(DcMotor.class, "fw1");
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mag = map.get(Servo.class, "mag");
        flap = map.get(Servo.class, "flap");
        tilt = map.get(Servo.class, "tilt");
        tiltDown();
    }
    public void stop(){
        isRunning = false;
    }
    private void updateCoordinate() {

    }
    public void setTargetVelocity(double targetVelocity){
        this.targetVelocity = targetVelocity;
    }
    public void aimAt(Goal g, Coordinate p){
        double angle = findAngle(g, p);
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
    public static double findAngle(Goal goal, Coordinate position){
        double d = goal.x - position.x;//position.distanceTo(goal);
        double goalHeight = inchesToMeters(goal.height);
        double theta;
        double h = goalHeight - launcherHeight;
        theta = Math.toDegrees((Math.acos((g*d*d/(V*V) - h)/Math.sqrt(h*h + d*d)) - Math.acos(h/Math.sqrt(h*h + d*d)))/2);
        return theta;
    }

    public static double setAngle(double theta){
        theta -= 25;
        return -0.1*theta*theta + 3*theta;
    }
    public void setFlyWheel(double pwr){
        if(pwr < 0.5) {
            tiltDown();
        }
        else if (pwr >= 0.5){
            tiltUp();
        }
        flywheel.setPower(pwr);
        flywheel1.setPower(pwr);
    }
    public void setOnlyFlyWheel(double pwr){
        flywheel.setPower(pwr);
        flywheel1.setPower(pwr);
    }
    public void tiltUp(){
        tilt.setPosition(0.75);
    }
    public void tiltDown(){
        tilt.setPosition(0.52);
    }

    public void magazineShoot(){
        tiltUp();
        sleep(600);
        for(int i = 0; i < 3; i++){
            mag.setPosition(0.32);
            sleep(150);
            mag.setPosition(.55);
            sleep(400);
        }
        tiltDown();
    }
    public void singleRound(){
        tiltUp();
        mag.setPosition(0.32);
        sleep(250);
        //this is sleep value to change
        mag.setPosition(0.55);
    }
    public void shoot(double rounds){
        for(int i = 0; i < rounds; i++){
            mag.setPosition(0.32);
            sleep(800);
            mag.setPosition(.55);
            if(i != rounds - 1) sleep(150);
        }
    }
    public void flapUp(){
        flap.setPosition(0.42);
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


