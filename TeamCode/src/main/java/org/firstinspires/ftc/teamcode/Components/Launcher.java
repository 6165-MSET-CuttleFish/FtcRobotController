package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.PurePursuit.Coordinate;
import org.firstinspires.ftc.teamcode.PurePursuit.MathFunctions;

import static org.firstinspires.ftc.teamcode.PurePursuit.MathFunctions.*;

public class Launcher {
    static final double launcherHeight = 0.2032;
    static final double V = 9.9059;
    static final double g = -9.08711677875;
    public DcMotor flywheel, flywheel1;
    public Servo mag, flap, tilt;
    public static Goal position;


    public Launcher(HardwareMap map){
        flywheel = map.get(DcMotor.class, "fw");
        flywheel1 = map.get(DcMotor.class, "fw1");
        flywheel.setDirection(DcMotor.Direction.REVERSE);
        flywheel1.setDirection(DcMotor.Direction.REVERSE);
        mag = map.get(Servo.class, "mag");
        flap = map.get(Servo.class, "flap");
        tilt = map.get(Servo.class, "tilt");
        mag.setPosition(0.47);
        tilt.setPosition(0.13);
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
        if(pwr == 0) {
            tiltDown();
        }
        else {
            tiltUp();
        }
//        flywheel.setPower(-pwr);
//        flywheel1.setPower(-pwr);
    }
    public void setOnlyFlyWheel(double pwr){
//        flywheel.setPower(-pwr);
//        flywheel1.setPower(-pwr);
    }
    public void tiltUp(){
        tilt.setPosition(0.72);
    }
    public void tiltDown(){
        tilt.setPosition(0.52);
    }

    public void magazineShoot(){
        tiltUp();
        sleep(500);
        for(int i = 0; i < 3; i++){
            mag.setPosition(0.35);
            sleep(150);
            mag.setPosition(.5);
            sleep(800);
        }
    }
    public void singleRound(){
        tiltUp();
        sleep(500);
        mag.setPosition(0.35);
        sleep(300);
        mag.setPosition(0.5);
    }
    public void shoot(double rounds){
        for(int i = 0; i < rounds; i++){
            mag.setPosition(0.35);
            sleep(800);
            mag.setPosition(.5);
            if(i != rounds - 1) sleep(150);
        }
    }
    public void flapUp(){
        flap.setPosition(0.07);
    }
    public void flapDown(){
        flap.setPosition(0);
    }
//    public static void main(String[] args){
//        findAngle(inchesToMeters(36), targets.highGoal);
//    }
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


