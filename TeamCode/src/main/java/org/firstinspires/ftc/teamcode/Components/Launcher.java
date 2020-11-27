package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.PurePursuit.Coordinate;
import org.firstinspires.ftc.teamcode.PurePursuit.MathFunctions;

import static org.firstinspires.ftc.teamcode.PurePursuit.MathFunctions.*;

public class Launcher implements Runnable{
    static final double V = 8.85; // 354 in/s
    static final double g = -9.81;
    public DcMotor flywheel, flywheel1;
    public Servo mag, flap, tilt;
    public static Goal position;
    static double launcherHeight = 0.2032;

    public Launcher(HardwareMap map){
        flywheel = map.get(DcMotor.class, "fw");
        flywheel1 = map.get(DcMotor.class, "fw1");
        flywheel.setDirection(DcMotor.Direction.REVERSE);
        flywheel1.setDirection(DcMotor.Direction.REVERSE);
        mag = map.get(Servo.class, "mag");
        flap = map.get(Servo.class, "flap");
        tilt = map.get(Servo.class, "tilt");
        mag.setPosition(0);
        tilt.setPosition(0.13);
        flap.setPosition(0);
    }

    public static void findAngle(double d, targets t/*, Servo flap*/){
        double goalHeight = 0;
        if(t == targets.highGoal) goalHeight = 35.5;
        else if(t == targets.lowGoal) goalHeight = 17;
        else if(t == targets.pwrShot) goalHeight = 23.5;
        goalHeight = inchesToMeters(goalHeight);
        double theta;
        double h = goalHeight - launcherHeight;
        theta = Math.toDegrees((Math.acos((g*d*d/(V*V) - h)/Math.sqrt(h*h + d*d)) - Math.acos(h/Math.sqrt(h*h + d*d)))/2);
        System.out.println(theta);
        //flap.setPosition(setAngle(theta));
    }
    public static void findAngle(Goal goal){
        double goalHeight = goal.height - position.height;
        double d = 1;
        goalHeight = inchesToMeters(goalHeight);
        double theta;
        double h = goalHeight - launcherHeight;
        theta = Math.toDegrees((Math.acos((g*d*d/(V*V) - h)/Math.sqrt(h*h + d*d)) - Math.acos(h/Math.sqrt(h*h + d*d)))/2);
        System.out.println(theta);
    }

    public static double setAngle(double theta){
        theta -= 25;
        return -0.1*theta*theta + 3*theta;
    }
    public void setFlyWheel(double pwr){
        tilt.setPosition(0.31);
        flywheel.setPower(-pwr);
        flywheel1.setPower(-pwr);
        sleep(400);
    }
    public void magazineShoot(){
        for(int i = 0; i < 3; i++){
            mag.setPosition(0.32);
            sleep(150);
            mag.setPosition(.5);
            sleep(150);
        }
    }
    public void setTilt(double angle){
        tilt.setPosition(12);
    }
    public static void main(String[] args){
        findAngle(2, targets.highGoal);
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
    @Override
    public void run(){

    }
}


