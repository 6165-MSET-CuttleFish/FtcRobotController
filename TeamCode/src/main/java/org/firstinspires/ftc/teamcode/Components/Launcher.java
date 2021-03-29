package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.PurePursuit.Coordinate;

import static org.firstinspires.ftc.teamcode.PurePursuit.MathFunctions.*;

public class Launcher implements Runnable{
    public ColorRangeSensor colorRangeSensor;
    static final double launcherHeight = 0.2032;
    static final double V = 9.9059;
    static final double g = -9.08711677875;
    public static PIDCoefficients MOTOR_VELO_PID = new PIDCoefficients(0.003, 0, 0);

    public static double kV = 0.00067714285714285714;//1 / TuningController.rpmToTicksPerSecond(TuningController.MOTOR_MAX_RPM);
    public static double kA = 0.0003;
    public static double kStatic = 0;

    double lastTargetVelo = 0.0;
    VelocityPIDFController veloController = new VelocityPIDFController(MOTOR_VELO_PID, kV, kA, kStatic);

    private final ElapsedTime veloTimer = new ElapsedTime();
    InterpLUT controlPoints;
    public DcMotorEx flywheel, flywheel1;
    public Servo mag, flap, tilt;
    public Servo rightIntakeHolder, leftIntakeHolder;
    public static Goal position;
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
        controlPoints = new InterpLUT();
        tiltDown();
        for (LynxModule module : map.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        setControlPoints();
        veloTimer.reset();
    }
    @Override
    public void run(){
        isActive = true;
            veloController.setTargetVelocity(targetVelo);
            veloController.setTargetAcceleration((targetVelo - lastTargetVelo) / veloTimer.seconds());
            veloTimer.reset();
            lastTargetVelo = targetVelo;
            double motorPos = flywheel.getCurrentPosition();
            double motorVelo = flywheel.getVelocity();
            double power = veloController.update(motorPos, motorVelo);
            if(targetVelo == 0){
                flywheel.setPower(0);
                flywheel1.setPower(0);
            } else {
                flywheel.setPower(Range.clip(0, 1, power));
                flywheel1.setPower(Range.clip(0, 1, power));
            }
    }
    public void stop(){
        isActive = false;
    }
    private void setControlPoints(){
        controlPoints.add(0, 3000);
    }
    public int getRings(){
        double range = colorRangeSensor.getDistance(DistanceUnit.INCH);
        if(range < 2.5){
            return 3;
        } else if(range < 4.5){
            return 2;
        } else if(range < 7){
            return 1;
        } else {
            return 0;
        }
    }
    public void setVelocity(double v){
        targetVelo = v;
    }
    public double getVelocity(){
        return flywheel.getVelocity();
    }
    public double getTargetVelo(){
        return targetVelo;
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
            //setOnlyFlyWheel(flyWheelSpeed + 0.08);
            sleep(100);
            if(i == rounds - 2){
                sleep(80);
            }
        }
    }
    public void singleRound(){
        tiltUp();
        mag.setPosition(0.34);
        sleep(150);
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