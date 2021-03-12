package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.PurePursuit.Coordinate;
import org.firstinspires.ftc.teamcode.PurePursuit.MathFunctions;

import java.util.concurrent.Callable;

import static org.firstinspires.ftc.teamcode.PurePursuit.MathFunctions.*;

public class Launcher {
    public ColorRangeSensor colorRangeSensor;
    static final double launcherHeight = 0.2032;
    static final double V = 9.9059;
    static final double g = -9.08711677875;
    InterpLUT controlPoints;
    public DcMotorEx flywheel, flywheel1;
    public Servo mag, flap, tilt;
    public Servo rightIntakeHolder, leftIntakeHolder;
    public static Goal position;
    PIDController controller;
    public double flyWheelSpeed;
    public Launcher(HardwareMap map, Pose2d pose2d){
        colorRangeSensor = map.get(ColorRangeSensor.class, "range");
        controller = new PIDController(0.5, 0, 0.8);
        flywheel = map.get(DcMotorEx.class, "fw");
        flywheel1 = map.get(DcMotorEx.class, "fw1");
        flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        mag = map.get(Servo.class, "mag");
        flap = map.get(Servo.class, "flap");
        tilt = map.get(Servo.class, "tilt");
        leftIntakeHolder = map.get(Servo.class,"wallL");
        rightIntakeHolder = map.get(Servo.class,"wallR");
        controlPoints = new InterpLUT();
        tiltDown();
        setControlPoints();
    }
    private void setControlPoints(){
        controlPoints.add(0, 3000);
    }
    public int getRings(){
        double range = colorRangeSensor.getDistance(DistanceUnit.INCH);
        if(range < 2){
            return 3;
        } else if(range < 4){
            return 2;
        } else if(range < 7){
            return 1;
        } else {
            return 0;
        }
    }
    public void setVelocity(double v){
        flywheel.setVelocity(v);
    }
    public double getVelocity(){
        return flywheel.getVelocity();
    }
    public void aimAt(Goal g, Coordinate p){
        double angle = findAngle(g, p);
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
        flyWheelSpeed = pwr;
        if(pwr < 0.3) {
            tiltDown();
        }
        else if (pwr >= 0.3){
            tiltUp();
        }
        flywheel.setPower(pwr);
        flywheel1.setPower(pwr);
    }
    public void setOnlyFlyWheel(double pwr){
        flyWheelSpeed = pwr;
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
        int rounds = getRings();
        for(int i = 0; i < rounds; i++){
            singleRound();
            setOnlyFlyWheel(flyWheelSpeed + 0.08);
            sleep(120);
            if(i == rounds - 2){
                sleep(50);
            }
        }
    }
    public void singleRound(){
        tiltUp();
        mag.setPosition(0.34);
        sleep(110);
            wingsOut();
        //this is sleep value to change
        mag.setPosition(0.48);
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


