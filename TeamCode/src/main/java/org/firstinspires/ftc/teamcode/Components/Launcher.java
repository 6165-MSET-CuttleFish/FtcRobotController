package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Launcher {
    final double V = 50; // random value rn
    final double g = -9.81;

    public Servo flap;
    public DcMotor launchWheel;
    public double launcherHeight = 0;
    public double goalHeight = 0;
    public double theta;

    HardwareMap map;
    DcMotor.RunMode newRun;

    public Launcher(DcMotor.RunMode runMode, HardwareMap imported, double launcherHeight){
        map = imported;
        newRun = runMode;
        this.launcherHeight = launcherHeight;

        launchWheel = map.dcMotor.get("launchWheel");
        flap = map.servo.get("flap");

        launchWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launchWheel.setMode(newRun);
        launchWheel.setDirection(DcMotorSimple.Direction.FORWARD);
    }


    public double findAngle(double x, double y, double goalX, double goalY){
        double h = goalHeight - launcherHeight;
        double d = Math.hypot(Math.abs(x-goalX), Math.abs(y-goalY));
        return Math.toDegrees((Math.acos((g*d*d/(V*V) - h)/Math.sqrt(h*h + d*d)) - Math.acos(h/Math.sqrt(h*h + d*d)))/2);
    }

    public void setAngle(double theta){
        double servoPos = -0.1*theta*theta + 3*theta;
        flap.setPosition(servoPos);
    }

}
